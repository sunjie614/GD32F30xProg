# IQMATH 辨识操作与仅弱电验证指南

核对日期：2026-10-08；源码基准：`测试2025-12-26` 分支，`dd50a5c`。

今天仅上弱电时，先完成第 1～4 节。第 5 节是实际连接电机、具备注入条件后的辨识操作。文档说明当前代码行为，目标板运行结果仍需填写记录。

## 1. 今天的操作顺序

1. 功率母线断电并释放残余电压，确认功率级不能产生电机电流；给控制板及所需采样、通信电路正常弱电供电。
2. VS Code 点击 Build，成功后点击 Flash。脚本烧录结束会自动复位并运行程序。
3. 上位机加载配套 A2L，保持 `Stop=1`、`Foc_Mode=IDLE`，读取启动、自检、周期、电流和母线变量。
4. 按第 4 节检查辨识界面和参数转换，保持停机，不执行 EST_RS 和 inj.State=1。
5. 保存烧录日志和实测值，填写第 9 节。

母线接近 0 V 时欠压保护会置位并保持停机，这是当前代码预期行为。仅弱电能检查通信、启动、采样和参数转换，不能完成真实 Rs/D/Q/DQ 辨识；不要为了推进状态而屏蔽保护或伪造母线值。

## 2. Build、A2L、Flash

| 按钮/任务 | 实际行为 |
|---|---|
| Build | 配置 IQMATH → 编译 → 生成 ELF/HEX → 生成 A2L |
| CMake Build | 编译并生成 ELF/HEX；单独执行后还需生成 A2L |
| A2L / Generate A2L | 从已有 ELF 读取符号、类型、地址并生成 A2L；不会先编译源码 |
| Flash | 烧录已有 IQMATH ELF，复位并运行；不会先执行 Build |

项目根目录下的配套文件：

```text
build-iqmath-softfp/GD32F30xProg1230.elf
build-iqmath-softfp/GD32F30xProg1230.hex
build-iqmath-softfp/GD32F30xProg1230.a2l
```

上位机加载这里的 A2L，不要与 build/、build-float-ref/ 的文件混用。源码修改后使用完整 Build；仅点击 A2L 不能把尚未编译的源码变成新固件。

```powershell
powershell -ExecutionPolicy Bypass -File Tools/GenerateA2L.ps1 -BuildDirectory build-iqmath-softfp
```

工具报告 Contents already match 表示生成内容与原文件一致，不是失败。文件时间不同不必然意味着地址不同，应从实际烧录 ELF 生成并核对。

### 2.1 两种 Flash 弹窗

J-Link Firmware Update 更新的是下载器自身固件。数据发送完成后，还要等待下载器重启、USB 重新连接，最终成功会显示成功信息或自动关闭；更新过程中不要拔 USB。

Secured GD32 device detected / Unsecure 是芯片读保护提示。解除会擦除芯片内部 Flash，并按弹窗说明恢复选项字节默认值；旧程序及 Flash 校准数据会丢失。确认已备份或可以丢弃、目标板确实要重烧时选 Yes，否则选 No。不要勾选 Remember selected action，避免以后自动擦除。电脑源码和 Git 历史不受影响。

依据：[SEGGER 固件说明](https://kb.segger.com/J-Link_-_Firmware)、[SEGGER GD32 解锁说明](https://kb.segger.com/GigaDevice_GD32)、[GD32F30x 用户手册](https://www.gd32mcu.com/data/documents/userManual/GD32F30x_User_Manual_Rev3.0.pdf)。

### 2.2 本次烧录日志怎样看

本次用户日志在 loadfile 后显示 Program & Verify、O.K.，随后执行 r、go、exit，确认 ELF 下载校验成功，并发出了复位和运行命令。运行中的初始化和通信还需检查。

解除保护后、下载前的 PC=FFFFFFFE、MSP=FFFFFFFC 与芯片擦空后的向量内容一致，不能据此判断新固件有故障。下载后仍异常时再读取启动状态和运行 PC。

## 3. 仅弱电观察清单

沿用原 CAN/CCP 上位机配置，当前 CAN 初始化为 1000 kbit/s。保持功率级隔离，只读以下变量。

| 项目 | 变量/位置 | 预期及判断 |
|---|---|---|
| 初始化 | FixedStartup_Status | 3=完成；0=尚未完成；1=算法初始化失败；2=ADC 校准失败 |
| 定点自检 | FixedControl_SelfTest.arithmetic_failures / trigonometric_failures / sqrt_failures | 应为 0；conversion 字段若无法搜索，用调试器读取完整结构体 |
| MTPA 启动表 | Mtpa_TableValid | 应为 1；启动时自动建表，无需手动设置 |
| 停机 | Stop / Foc_Mode / Foc_Reset | 保持 1 / IDLE / 1；Reset 由控制周期更新，不是独立使能开关 |
| 控制中断 | FixedControl_CycleCount | 连续增加 |
| 频率和负载 | MainInt_FrequencyHz / MainInt_MaxCycleTicks / MainInt_PeriodTicks / MainInt_LoadPermille | 约 5000 Hz；记录最大周期；负载不得达到或超过 1000‰；断点暂停的数据不用作周期测量 |
| 电流零偏 | adc_ch0_offset / adc_ch1_offset / adc_ch2_offset | 有效 12 位 ADC 范围；中心以原硬件为准，不强行要求 2048 |
| 电流和母线 | Foc_Id_Fdbk / Foc_Iq_Fdbk / Adc_VoltageBus / Foc_BusVoltage | 无电流时接近零并记录噪声；母线接近实测值 |
| 保护标志 | ProtectIq_CompatibilityState.flags | 母线低于 20 V 时 Low_Voltage=8 是预期；其他位按实际原因检查 |
| Break IRQ 记录 | FixedBreakIrqArmed | 母线从未超过 200 V 时为 0 可属预期；仅表示软件是否开启过 IRQ，不代表栅极安全状态 |
| 数值诊断 | McMath_Diagnostics.saturation_count / divide_by_zero_count / invalid_input_count | 记录初值和增量，持续异常增长需定位 |
| 主输出 | TIMER0 CCHP.POEN（常称 MOE） | 停机时应为 0；结合实际驱动引脚及原硬件极性检查 |

50% 比较值不等于开关管都关断，应结合主输出禁止状态和实际驱动输出判断。若 FixedStartup_Status=1/2，程序停在初始化失败分支，CAN 或控制中断可能未启动，先用 SWD 读取失败信息。

### 3.1 串口检查定浮点转换

JustFloat 数据通道下标从 0 开始，18 个数据 float 加帧尾，当前串口 4 Mbaud。

| 下标 | 数据 | 单位 |
|---:|---|---|
| 5 / 6 | 同一母线浮点样本 / 转 Q24 后回转物理值 | V |
| 7 | 母线 Q24 原始整数，以 float 数值装入帧 | raw count |
| 9 / 10 | 同一 A 相浮点样本 / 转 Q24 后回转物理值 | A |
| 11 | A 相 Q24 原始整数，以 float 数值装入帧 | raw count |
| 15 | 最终 d 轴电压指令 | V |
| 16 / 17 | d / q 轴反馈电流 | A |

比较 5≈6、9≈10；原始计数约为 `(V/800)*2^24`、`(A/30)*2^24`。仅零输入不能证明整个量程准确。原始整数装入 float 帧可能损失最低几位，精确比较用 A2L 整数变量。串口队列忙时会丢帧。

## 4. 仅弱电检查辨识参数

1. 保持 Stop=1，选择 Foc_Mode=IDENTIFY（5），等待模式入口初始化。
2. 确认 Experiment.Initialized=1、state=WAIT、inj.mode=INJECT_D、inj.State=0、Ud_amp/Uq_amp=0。
3. 默认 start_I=3、final_I=12、step_dir=1、repeat_times=3、max_steps=20、sample_capacity=512、wait_edges=3。
4. 停机下可以修改 start_I/final_I、Ud_amp/Uq_amp，核对读回和内部只读配置。例如 3 A 为 0.1 pu，100 V 为 0.125 pu。完成后将两轴幅值恢复为 0。
5. 返回 Foc_Mode=IDLE，保持 Stop=1。

Stop 保持时，每周期 Reset 将运行状态复位到 WAIT，因此此时写 state=EST_RS 会被覆盖，这是 Reset 行为。不得清欠压或强制使能来完成本节。

只读内部 `MainInt_ControlState.parameters.identification_config.current_start_pu`：3 A 原始 Q24 约 1677722；`injection_voltage_d_pu`：100 V 为 2097152。不要把 A/V 数字直接写入内部 *_pu 字段。

## 5. 实际模型辨识

以下需要真实电机电流响应。沿用原已验证电机、相序、位置对齐、母线供电及急停条件。用户确认原方案的 100 V 注入、3～12 A 扫描已经实验验证；定点版本需记录同工况对照结果。

### 5.1 观察窗口及模式准备

| 用途 | 变量 |
|---|---|
| 总开关 | Stop、Foc_Mode、Foc_Reset、MainInt_UseRealTheta、Sensorless_Method |
| 状态和错误 | Experiment.state、Identification_State、Identification_Error、Experiment.Complete |
| 轴和电流点 | Experiment.inj.mode、inj.Imax、inj.State、Experiment.step_index、repeat_count |
| 注入和采集 | Experiment.inj.Ud_amp/Uq_amp、inj.Vd/Vq、Foc_Id_Fdbk/Iq_Fdbk、Experiment.pos、edge_count |
| 模型结果 | Experiment.Rs_est、Experiment.LLS.ad0/add/aq0/aqq/adq、LLS.D/Q/DQ.J、LLS.D/Q/DQ.R2 |
| MTPA 重建 | Mtpa_TableValid、Mtpa_RebuildPending、Mtpa_RebuildFailed |

设置 MainInt_UseRealTheta=1、Sensorless_Method=0，沿用原位置对齐方式；HFI 开启后在 IDENTIFY 中也会额外叠加电压，不能把两套注入混在一起。不要临时猜测位置偏置来固定角度。

保持 Stop=1，先选择 IDENTIFY，初始化后再写下列参数。先在 IDLE 写再切模式会被默认值覆盖。

```text
Experiment.start_I         = 3
Experiment.final_I         = 12
Experiment.step_dir        = 1
Experiment.repeat_times    = 3
Experiment.max_steps       = 20
Experiment.sample_capacity = 512
Experiment.wait_edges      = 3
Experiment.inj.Ud_amp      = 0
Experiment.inj.Uq_amp      = 0
Experiment.inj.State       = 0
```

start_I/final_I 是整数 A；step_dir 仅表示方向，步长固定 1 A，写 2 不会成为 2 A 步长。12→3 A 用 -1。网关限制重复 1～3、最大点数 1～20、容量 2～512、等待边沿 0～100。

起止电流与方向应相符，幅值采用非负 V。接口未覆盖任意参数的完整工况安全校验，写入成功不等于参数合理。双系数单轴拟合至少保留两个有效电流点。

### 5.2 Rs 辨识

1. 确认供电正常、保护原因排除，PE4/PE15 处于原硬件允许状态。保护标志锁存时仅写 Stop=0 会立即再次停机，使用原已验证故障复位流程。
2. 保持 IDENTIFY/WAIT，按原流程释放 Stop=0，确认 Foc_Reset=0。选模式不自动释放 Stop，单写 Reset=0 也不能替代它。
3. 写 Experiment.state=EST_RS（旧枚举 1）；不要在 Stop 仍为 1 时发启动命令。
4. Rs 自动生成电压：d 轴从 1 V 起，正常电流响应时默认约每 3 s 增加 1 V，q 轴为 0；电流增量过小可能跳过增压，间隔会更长。不用写 100 V 方波幅值或 inj.State=1。
5. 成功后回到 WAIT，Rs_est 更新，两轴幅值/输出、Imax 清零，Error=0。只读 MainInt_ControlState.foc.identification.rs_complete=1 可确认成功，避免把失败后的 WAIT 误当完成。

无电流响应或不收敛时不要长期放任电压递增；当前路径没有覆盖所有无响应情况的总超时。按第 6 节停止并记录原因。弱电阶段不执行 Rs。

### 5.3 D 轴完整扫描

1. Rs 成功、状态 WAIT、轴 INJECT_D 后，写 Ud_amp=100、Uq_amp=0。
2. 写 Experiment.state=NEXT_I（旧枚举 4）。等到 INJECT_COLLECT、Imax≈3 A、inj.State=0，再写一次 Experiment.inj.State=1。
3. 算法自动完成当前点的 3 次重复。Id 到 +Imax 切负 Ud，到 −Imax 切正 Ud。
4. 点完成后 Imax 自动进入约 4 A，重新等待人工放行；确认轴仍为 D、状态稳定在 INJECT_COLLECT、Imax 已推进且 State=0，再写 State=1。依次至 12 A。
5. 最后点完成后自动拟合 ad0/add，再切 Q 轴，保存 D 结果和拟合质量。

同一点内部重复不需要人工逐次放行。PROCESS 期间 State 也可能短暂为 0，不能仅见一次 0 就判定已到下一点；同时观察 Imax、mode、state。点与点之间不要写 Stop=1，Stop 会复位整次辨识。

### 5.4 Q 轴完整扫描

1. 自动切到 inj.mode=INJECT_Q（1），两轴幅值清零，电流目标回到起点。
2. 等到 INJECT_COLLECT，写 Ud_amp=0、Uq_amp=100，再写 inj.State=1。
3. Iq 到 ±Imax 切 Uq 极性；每点自动重复 3 次，下一个电流点再人工放行。完整扫描 3～12 A。
4. 自动拟合 aq0/aqq，切到 DQ，保存 Q 数据。

无需重发 EST_RS/NEXT_I，也不用每个点重写幅值；轴切换清零后才需重新设置电压。

### 5.5 DQ 交叉注入与完成

1. 自动到 inj.mode=INJECT_DQ（2），等待 INJECT_COLLECT。2026-10-13 起，Imax 使用独立变量 `Identification_DqCurrent_A`，上电默认 9 A，不再取 start_I。
2. 首次 DQ 放行前，直接在上位机按 A 写 `Identification_DqCurrent_A`（例如 10.0，也可小数），确认 Imax/IDmax/IQmax 已更新；写 Ud_amp=100、Uq_amp=100，再写一次 inj.State=1。
3. Ud 看 Id 到 ±Imax，Uq 看 Iq 到 ±Imax，两轴独立翻转；采样窗口以 d 轴电压翻转边沿计数。
4. DQ 在独立目标电流点自动重复 repeat_times 次，随后拟合 adq，不扫描完整 3～12 A。首次注入后整组重复锁定该电流；期间修改新变量留待下一轮完整辨识生效，暂停/恢复不解除锁定。9 A 表示两轴分别在 ±9 A 翻转，不是合成电流 9 A。
5. 成功时 Complete=1、Experiment.state=DONE、Identification_State=7、Error=0，自动请求 IDLE、停机。结果在 IDLE 锁存。
6. IDLE 主循环后台重建/提交 MTPA。等待 RebuildPending=0，确认 RebuildFailed=0，之后再使用辨识模型。2026-10-12修订：新模型失败时旧表数组保留供诊断，但活动表有效性撤销；后续SPEED采用Id=|Iq|回退，`Mtpa_LinearFallback=1`可观察。回退电感为配置标称值，不代表辨识成功。表无效时热INIT仍会被原就绪检查拒绝，勿强行改状态释放Stop。

重新进入 IDENTIFY 会初始化镜像并清除本次结果，先导出数据。

独立的 `Identification_DqCurrent_A` 配置在 INIT、重新进入 IDENTIFY 时保留；掉电恢复 9 A。0、负数、非有限值、超 Q24 表示范围及量化为 0 的值会拒绝、回填上次接受值并计无效输入。它不改变 D/Q 扫描起止值、Rs 流程或保护限值；内核仍只保存 Q24 数值。新固件必须加载同次生成的 A2L 才能按该变量名调参。

## 6. 停止与重来

**当前 Experiment.inj.State=0 可暂停辨识注入/采集，但不是硬件急停。** 新点首次放行后，同一点内部重复自动执行；采集中写 0 保留历史样本，恢复 1 后继续。DQ 暂停不会解除本轮电流锁定。若启用了 HFI，暂停辨识不关闭额外 HFI 注入。暂停后的采样物理连续性需重新考虑；完整操作语义以 `IQmath_Mode_Operation_Guide.md` 第 8 节为准。

需立即停止使用原硬件急停；软件正常停机写 Stop=1 并置 Foc_Mode=IDLE，核实主输出禁止。不要靠清幅值、写 state=WAIT、改 inj.mode 或仅清 inj.State 停机。上位机失联由硬件急停处理。

Stop/保护引发 Reset 后内部 Rs、样本、拟合和点进度清除，不是保存进度暂停。重新辨识：保持停机排除原因、保存诊断，IDLE→IDENTIFY，重写参数，从 Rs 开始。当前没有掉电续辨识流程。

## 7. 状态、错误及哪些变量可写

两套状态数值有区别，优先使用英文枚举：

| 状态 | Experiment.state（操作） | Identification_State（观察） |
|---|---:|---:|
| WAIT | 0 | 0 |
| EST_RS | 1 | 1 |
| PROCESS | 2 | 3 |
| INJECT_COLLECT | 3 | 2 |
| NEXT_I / NEXT_CURRENT | 4 | 4 |
| LLS | 5 | 5 |
| PENDING | 6 | 6 |
| DONE | 7 | 7 |
| FAILED | 映射为 WAIT | 8 |

Experiment.state 接受 Rs 启动、成功后首次 NEXT_I 命令；其他阶段算法自动推进，不支持靠任意写状态绕过前置步骤。Identification_State 只是观察值。

| Identification_Error | 名称 | 含义 |
|---:|---|---|
| 0 | NO_ERROR | 无辨识错误，不代表硬件无保护 |
| 1 | INVALID_CONFIG | 配置无效 |
| 2 | SAMPLE_OVERFLOW | 枚举预留；当前容量满转 PROCESS，不保证触发它 |
| 3 | RS_NOT_CONVERGED | Rs 超限或未收敛路径 |
| 4 | SINGULAR_LLS | 拟合/求解数据不足或病态 |

FAILED 自动请求 IDLE，旧 Experiment.state 显示 WAIT，务必同时看 Identification_State=8 和 Error。Error/结果在 IDLE 保留，重新进入 IDENTIFY 清零。

生效输入：start_I/final_I/step_dir、独立 `Identification_DqCurrent_A`、repeat_times/max_steps/sample_capacity/wait_edges、inj.Ud_amp/Uq_amp、每点首次放行 inj.State、条件满足时 state=EST_RS/NEXT_I。每周期读取，在 WAIT 或等待新点放行时设置；采集中不要修改范围、方向或容量。DQ 电流首次放行后按第 5.5 节锁定。

inj.mode/Imax/IDmax/IQmax、Ts、Rs_est、Identification_Voltage、LLS 系数是观察镜像，写入不能等价修改内核。写 Rs_est 不能跳过 Rs；Ts 未接成任意在线采样周期设置。Identification_Voltage 显示当前 D/Q 幅值较大绝对值，不是 Rs 电压或唯一注入控制。

## 8. 结果保存与浮点版差异

A2L 的电压、电流、磁链及模型系数仍是浮点物理值，内核 Q24。保存 Rs、五个系数、D/Q/DQ 的 J/R2、波形及参数，同工况对照旧浮点。

- Experiment.results[] 为 D/Q 共用镜像，Q 同索引会覆盖 D，分别导出。需要两轴完整原始结果时通过调试器导出内核 d_results/q_results 并按基值换算，不能把 Q24 整数直接当 A/Wb。
- Ud_buf/Id_buf/Uq_buf/Iq_buf/psi_d_buf/psi_q_buf 随最新样本更新；仅当前 pos 范围有效，未填区可能保留上一轮数据。重复会复用缓存，不自动归档整次实验。
- 操作框架保留，但不是逐样本完全等价移植：当前单轴选去均值后绝对峰值；电压采样与旧版保存上一周期电压的对齐方式不同；DQ 当前按完整采集次数重复，旧版曾按样本增加 repeat_count。
- 当前 LLS.D.J/Q.J 为平均残差平方换算值，旧版为残差平方和，不能仅按同名字段直接比较。DQ.J 为累计残差平方和，不同样本数也影响指标。
- 当前 inj.State 放行锁存与旧版运行中可清 State 有差别，停止按第 6 节。
- 新 MTPA 在 IDLE 主循环后台重建，旧版在控制路径内重建，等待提交成功后使用。

这些差别应结合相同输入波形和窗口验证，主机测试不能代替实际电机拟合质量。

## 9. 本次记录

已核对 dd50a5c 状态机及构建任务；2026-10-08 从当前 ELF 生成 A2L，报告 Contents already match。用户提供 Flash 日志确认下载校验并执行 go。下面尚待目标板填写。

| 项目 | 实测值/结论 |
|---|---|
| 板号、弱电电源、功率级隔离方式 | 待填写 |
| ELF/A2L、源码提交、烧录日志 | 待填写 |
| Startup_Status / TableValid / 自检 | 待填写 |
| Stop / Mode / POEN / 实际驱动状态 | 待填写 |
| 控制频率、最大周期、负载 | 待填写 |
| 零偏、电流噪声、母线读数 | 待填写 |
| 通信、英文枚举、参数转换读回 | 待填写 |
| 保护标志、数值诊断初值及增量 | 待填写 |
| Rs、D/Q/DQ 参数、系数及 J/R2 | 仅弱电不执行 |
| 辨识后 MTPA 重建结果 | 实际辨识后填写 |

代码依据：Application/Src/initialization_iq.c、main_int_iq.c、fixed_control_gateway.c、foc_iq.c、identification_iq.c；Middleware/Hardware_Interface/Src/hardware_interface.c；.vscode/tasks.json、Tools/flash_iqmath.jlink。累计测试记录见 [IQmath_FixedPoint_Migration_Test.md](IQmath_FixedPoint_Migration_Test.md)。
