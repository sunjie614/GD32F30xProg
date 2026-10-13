# IQmath 全定点迁移与测试记录

最新记录见第21节（2026-10-12 运行中INIT退出恢复ADC及失败输出门控），取代第20节的INIT缺口未修复结论。第20节辨识升压/扫描终点、MTPA线性回退、Wc上限3990继续有效。第19节正反增益对称继续有效，其4000上限被第20节替代。第18节运行中状态保留、第17节停机抢占及Flag可写恢复继续有效。第16节旧顺序测试未覆盖第17节抢占和第21节INIT失败恢复；此前各节保留累计历史，不覆盖最新结论。当前操作以最新累计记录和模式操作指南为准。

## 1. 基线与构建信息

| 项目 | 当前记录 |
|---|---|
| 本轮基线提交 | `6c1ac9755a702a31f25e5db8e44f8a8a62f2466d` |
| 开发分支 | `测试2025-12-26` |
| 编译器 | Arm GNU Toolchain 14.2.Rel1，GCC 14.2.1 |
| MCU | GD32F303VCT6，Cortex-M4F |
| 默认数值后端 | `IQMATH` |
| 参考后端 | `FLOAT_REF` |
| IQMATH ABI | Cortex-M4、Thumb-2、`-mfloat-abi=softfp` |
| 主控制频率 | 定时器配置值为 5 kHz、周期 200 us；新增 DWT 实测量，上板读取待填写 |
| 速度环频率 | 500 Hz，周期 2 ms |
| `IQmathLib.h` SHA-256 | `896A09E33B72C7F14BB7C5F6E9147FD8D1F9BAE8BFD816A27069B7C900607E08` |
| `IQmathLib-cm3.lib` SHA-256 | `507F3030752DA70826CB46861201D76D241E177DD5654DC3B52E6B18C20AE0B2` |

说明：`120 MHz / 12000 / 2 = 5 kHz` 是当前代码推导值，中心对齐且重复计数器为 1。它不是示波器实测结论。本轮未修改定时器参数；上板后读取 `MainInt_FrequencyHz`，并用示波器/逻辑分析仪复核。

## 2. 数值契约

默认内部格式为有符号 Q24，原始值与物理值的关系为 `raw = pu * 2^24`。除 CAN/CCP/A2L、ADC/位置输入和 PWM 输出边界外，默认固件控制状态不得使用浮点。

| 信号类别 | 物理基值 | 设计工作区间（非存储上限） | 格式 | 边界行为 |
|---|---:|---:|---|---|
| 相电流、dq 电流 | 30 A | -1.0 至 1.0 pu | Q24 | ADC 物理值进入中断时转换 |
| dq/alpha-beta 电压 | 800 V | -1.0 至 1.0 pu | Q24 | PWM/A2L 输出时转换 |
| 机械速度 | 1800 rpm | -1.0 至 1.0 pu | Q24 | 位置接口和 A2L 边界转换 |
| 电角度 | 1 转 | 0 至 1.0 pu | Q24 | `1 pu = 2*pi`，使用 IQmath PU 三角函数 |
| PWM 占空比 | 1.0 | 0 至 1.0 | Q24 | 输出前限幅并转换为 float 边界结构 |
| PI 积分与输出 | 按对应电流/电压基值 | 按配置上下限 | Q24 | 增益写入时转换为离散 Q24 系数 |
| 辨识累计量 | 对应 Q24 乘积 | 64 位范围 | `int64_t` | 512 点后缩放回 Q24，检查除零和缩窄范围；FLOAT_REF 对应使用 double 参考累加器 |

定点数学统一由 `mc_math.h` 调用 IQmath。加减采用饱和；乘法使用 `_IQ24rmpy` 完成带舍入和饱和的 Q48 到 Q24 缩窄；除法、sinPU、cosPU、atan2PU 和 sqrt 使用随附 IQmath 库。浮点边界转 Q24 采用就近舍入并在转换前检查范围；除零、无效输入和饱和分别累计诊断计数。

表中电流、电压、速度的 ±1 pu 不是输入截断。32 位有符号 Q24 可表示 `[-128,128-2^-24]` pu；例如 45 A/30 A=1.5 pu 可以转换和记录，但会触发另行配置的相电流过流保护。基值、表示范围、传感器实际量程和保护/控制限幅是不同概念。

## 3. 软件测试记录

| 编号 | 测试步骤 | 预期 | 当前结果 | 状态 |
|---|---|---|---|---|
| SW-01 | 配置 `IQMATH` 后端 | Cortex-M4 softfp，链接 CM3 IQmath 库 | 配置成功 | 通过 |
| SW-02 | 编译全部 64 个 IQMATH 翻译单元 | 无编译错误 | 全部通过 | 通过 |
| SW-03 | 归档并链接最终 ELF/HEX | 无未定义符号、无 ABI 冲突 | ELF/HEX 已生成 | 通过 |
| SW-04 | 检查 ELF ARM 属性 | Cortex-M4、VFPv4-D16、无 hard-float 参数 ABI 标记 | 符合 | 通过 |
| SW-05 | 检查 IQmath 符号 | 存在 rmpy/div/sinPU/cosPU/atan2PU/sqrt/toF | 均存在 | 通过 |
| SW-06 | 检查旧浮点算法符号 | 默认 ELF 不含 CMSIS f32 三角、旧 MTPA/辨识、旧 LESO/HFI | 未发现 | 通过 |
| SW-07 | 逐对象反汇编 FOC、辨识、MTPA、Motor、Protect、滤波、PI/PLL/变换、LESO/HFI/Flying | 不含 VFP 浮点算术和 `__aeabi_*` 浮点调用 | 13 个核心对象全部通过 | 通过 |
| SW-08 | 生成并过滤 A2L | 原 FOC 物理浮点镜像存在，模式显示英文枚举 | `Foc_Mode` 为 `UBYTE FocMode_t 0..5`，枚举表完整 | 通过 |
| SW-09 | 编译 FLOAT_REF 参考后端 | 原浮点路径不受 IQMATH 文件影响 | 64 个翻译单元构建和链接通过 | 通过 |
| SW-10 | 运行固定算法 FLOAT_REF/Q24 主机测试 | 变换、PI、MTPA、Motor、Protect 均通过且无未解释数值诊断 | 两个后端均通过 | 通过 |
| SW-11 | 运行 FOC Q24 状态回归 | Reset 释放、VF、IDENTIFY、非法模式、后台 MTPA 提交均符合状态约束 | 通过 | 通过 |
| SW-12 | 运行辨识模拟工况 FLOAT_REF/Q24 对照 | 状态机到 DONE；五个系数偏差均不超过 1% | 最大相对偏差约 0.043% | 通过 |
| SW-13 | `cppcheck` 检查 IQ 应用、网关、工具和无位置模块 | warning/performance/portability 无未处理问题 | 通过 | 通过 |
| SW-14 | 运行目标板自检 | 四类 failure 均为 0 | 待上板读取 | 待测 |
| SW-15 | 本轮上电路径修复后重跑 IQMATH/FLOAT_REF 编译、13 个核心对象审计和 8 组主机回归 | 全部通过；80 V 母线中性比较值、120 V 正常调制；IDLE/非法模式请求硬件停机 | 全部通过 | 通过 |
| SW-16 | 从本轮 IQMATH ELF 生成 A2L | `Foc_Mode` 英文枚举、`Stop`、启动诊断可搜索 | `Foc_Mode` 使用 `FocMode_t`；`Stop`、`FixedStartup_Status`、`FixedBreakIrqArmed` 均存在 | 通过 |
| SW-17 | 原版辨识 A2L 操作兼容回归 | `Experiment.*` 物理量可驱动 Q24；每个新电流点人工放行；D→Q→DQ 完整；失败退出 | FLOAT_REF/Q24 模拟、网关转换和失败停机判据均通过 | 通过 |

当前 IQMATH 固件资源：Flash 81,228 B（30.99%），RAM 35,208 B（71.63%）。FLOAT_REF 固件资源：Flash 45,464 B（17.34%），RAM 20,520 B（41.75%）。RAM 已包含链接脚本预留的 1 KiB heap 和 2 KiB stack，也包含固定长度辨识采样缓冲区、后台 MTPA 暂存表及旧 A2L 浮点镜像；IQMATH 辨识累计使用 64 位整数。性能只记录，不设提速通过门槛。

主机对照的 MTPA 第 25/50 点最大满量程误差约 0.012%；模拟辨识的 `ad0/add/aq0/aqq/adq` 最大相对偏差约 0.043%。这些结果验证软件定点路径和参考路径的一致性，不代替目标电机实测。

静态栈帧记录：`Main_Int_Handler` 128 B、`FixedControl_Step` 272 B、`FocIq_Run` 184 B、`IdentificationIq_Run` 176 B、`MtpaIq_BuildTable` 176 B、`HfiIq_Run` 72 B。`.su` 只记录单函数静态帧，不能代替完整调用链或中断嵌套测量；首次上板仍需做栈水位检查。`FixedControlState_t` 为 17,136 B，其中包含活动 MTPA 表和后台暂存表。

## 4. 已迁移的软件边界

| 模块 | 定点实现 | 状态接口 | 当前结论 |
|---|---|---|---|
| FOC 主链 | Clarke/Park、PI、反变换、SVPWM、速度斜坡 | `FocIq_Init/Run/Reset` | 已接入 IQMATH 中断路径；待上板对照 |
| 模式控制 | `IDLE/VF_MODE/IF_MODE/SPEED/STARTUP/IDENTIFY` | 状态包含模式、VF/IF 相位和启动计数 | A2L 英文枚举已恢复 |
| 分段速度 PI | Kp 四段状态机、Ki 确认与倍增 | `SpeedPidScheduleIq_*` | 已接入速度环；待轨迹验收 |
| MTPA | 模型、幂次、转矩、磁链扫描、二分、黄金分割、51 点表、按 Iq 排序和插值 | `MtpaIq_Init/BuildTable/Interpolate/Reset` | 默认启动建表；辨识后在 IDLE 的主循环后台建暂存表，再短临界区提交 |
| 辨识 | Rs、D/Q/DQ 注入、边沿等待、512 点磁链积分、多电流点、重复平均、归一化 LLS、SSR/R2、adq | `IdentificationIq_Init/Start/Run/Reset` | Q24 高次基函数先归一化避免下溢；严禁在未限流上板前直接运行 |
| 无位置 | LESO、PLL、HFI 解调/注入、速度滞环切换、Flying 延时 | 各模块独立 `Init/Run/Reset` | 已接入；参数和切换瞬态待上板标定 |
| Motor/Protect | 位置/速度估算及电流、母线、温度、硬件故障保护 | `MotorIq_*`、`ProtectIq_*` 显式状态 | 旧 API 仅作边界兼容包装 |
| A2L 网关 | 物理 float 与 PU Q24 双向转换 | 每周期生成输入快照、输出后更新镜像 | 核心不引用 A2L 全局变量 |

算法核心允许文件内 `static` 辅助函数和 `static const` 查表，但不允许函数内可变 `static` 或直接读取全局调试变量。唯一顶层控制实例位于 `main_int_iq.c` 的中断集成层。

## 5. 构建步骤

```powershell
powershell -ExecutionPolicy Bypass -File Tools/BuildSequential.ps1 -Backend IQMATH
```

浮点参考后端：

```powershell
powershell -ExecutionPolicy Bypass -File Tools/BuildSequential.ps1 -Backend FLOAT_REF
```

本机 WinLibs Ninja 可能无法正确回收 ARM GCC 子进程，因此提供顺序构建脚本；它使用 CMake 生成的原始编译、归档和链接命令，不清理或删除源码。

一键运行八组主机回归：

```powershell
powershell -ExecutionPolicy Bypass -File Tools/RunFixedHostTests.ps1
```

## 6. A2L/CCP 回归步骤

1. 烧录 IQMATH 固件并加载对应 ELF 生成的 A2L。
2. 按原变量名搜索 `Foc_Mode`、`Foc_Speed_Ref`、`Foc_Pid_Speed_Handler.Kp`、`Foc_Idq_Ref.d` 和 `Foc_Udq_Ref.q`。
3. 保持停机，分别写入速度参考和 PI 参数；确认读回值保持物理单位。
4. 观察一个控制周期后定点上下文和输出镜像已更新。
5. 展开 `Foc_Mode`，确认显示 `IDLE/VF_MODE/IF_MODE/SPEED/STARTUP/IDENTIFY`，并逐项写入后读回。
6. 执行 Reset、模式切换和掉电重启，确认镜像与内部状态同步。
7. 读取 `McMath_Diagnostics.*` 和 `FixedControl_SelfTest.*`；正常工况不应出现未解释的计数增长。

实际上位机软件、A2L 文件版本、操作者、日期和结果：**待上板填写**。

## 7. 上板验收矩阵

| 测试 | 操作 | 验收标准 | 实际结果 |
|---|---|---|---|
| 控制频率 | 读取 `MainInt_PeriodTicks/MainInt_FrequencyHz`，并用 GPIO/示波器复核 | 约 5 kHz；以实测值统一离散系数 | 待测 |
| 坐标变换/SVPWM | 固定角度与电流/电压向量，对照 FLOAT_REF | 满量程误差不超过 0.1% | 待测 |
| 角度 | 正反转覆盖过零点 | 电角度误差不超过 0.5 度 | 待测 |
| 闭环轨迹 | 空载和带载加减速、正反转 | 关键轨迹误差不超过 0.5% | 待测 |
| 保护 | 限流、母线边界、温度和传感异常 | 正确置位且 PWM 回到安全占空比 | 待测 |
| 无传感 | LESO、HFI、切换及飞车工况 | 无异常跳变，诊断计数可解释 | 待测 |
| 辨识 | 运行固定注入和 512 点累计 | 相对 FLOAT_REF 结果误差不超过 1% | 待测 |
| 性能 | 读取 `MainInt_CycleTicks/MainInt_MaxCycleTicks/MainInt_LoadPermille` | 记录结果；不得超过实测控制周期 | 待测 |

## 8. 首次上板操作顺序

1. 功率级与电机先断开，仅给控制板限流供电；加载本次 IQMATH ELF 生成的 A2L，保持 `Stop = 1`、`Foc_Mode = IDLE`、`Foc_Reset = true`。
2. 确认 `FixedStartup_Status = 3`、`FixedControl_SelfTest` 四项均为 0；记录 `McMath_Diagnostics` 初值。若启动状态为 1/2，停止试验并用 SWD 读取故障原因。
3. 核对电流零点、母线电压、实测角度和速度方向；比较寄存器应为 50%，但**50% 本身不是安全输出**，必须另行确认 TIMER0 `MOE=0`、六路驱动均未有效导通。
4. 记录 `MainInt_FrequencyHz`、最大周期和负载；若不是约 5 kHz，停止闭环测试并按实测周期重新生成离散系数。
5. 先做低电压 `VF_MODE`，再做限流 `IF_MODE`，确认相序、角度方向和 SVPWM。
6. 用传感器角度、空载低速进入 `SPEED`，验证 STARTUP 保持、速度斜坡和 PI 输出。
7. 单独验证 HFI、LESO、滞环切换和 Flying；每项通过后再组合。
8. 最后才允许进入 `IDENTIFY`；设置硬件限流与急停，逐级提高注入电压，保存 D/Q/DQ 系数、SSR/R2 和 FLOAT_REF 对照结果。

## 9. 已知限制与放行条件

- 尚未烧录目标板，因此闭环精度、无传感切换、保护触发和辨识误差不能标记为通过。
- IQMATH 启动路径现在在开启 ADC 中断前执行电流零偏校准、初始母线保护检查、保护标志复位，并强制保持 Stop；仍需在目标板确认校准时电机无电流且采样值稳定。
- MTPA、LESO/HFI/Flying 和完整辨识路径已经定点实现，但尚未获得目标板波形和 FLOAT_REF 数值误差数据；不能把“编译通过”表述为“算法性能等价”。
- `MainInt_ControlState.*` 会被 A2L 工具发现，其中的 Q24 原始值仅供底层诊断；日常标定继续使用 `Foc_*` 浮点物理镜像，避免把 Q24 原始整数误当物理值写入。
- 首次上板必须保持 `Foc_Mode = IDLE`，确认自检、母线、电流零偏、角度、PWM 50% 安全输出和诊断计数后再逐级使能。
- 所有待测项填写并满足阈值后，才能将本记录结论改为“完成验收”。

## 10. 上电前静态安全复核（未代替台架实测）

本轮以 `FLOAT_REF` 的 `foc.c`、`main_int.c` 及两套后端共用的 `tim.c`、`hardware_interface.c` 为硬件行为基准。旧版与定点版三相 SVPWM 的扇区判定、T1/T2 计算、ABC 比较值映射一致；TIMER0 通道低有效、互补输出、2 us 死区、PE15 低有效 Break、`MOE` 默认关闭以及 `Set_PWM_Compare()` 的 CH0/1/2/3 对应关系没有改动。仅静态核对，实际栅极极性和死区仍需在功率级断开时用示波器确认。

本轮发现并修正：

| 项目 | 旧版基准/问题 | 定点版处理 | 验证状态 |
|---|---|---|---|
| 软件停机与 Break IRQ | IQ 启动路径会开启 Break IRQ；旧 ISR 会将软件 Break 误记为 `Hardware_Fault` | IQ ISR 只把非软件 Break 或 PE15 低电平判为硬件故障；停机期软件 Break 只触发一次，FLOAT_REF 保持原逻辑 | 编译/静态检查通过；目标板待测 |
| IDLE 与硬件停机 | 旧 `MainInt_Check_ProtectFlag()` 使用 `Foc_Get_ResetFlag()`，在 IDLE 强制 `Stop=1`；原 IQ 仅复位算法，未同步停机 | IQ 中断和主循环在 IDLE/非法模式先保持 `Stop=1`，模式在本周期变成 IDLE 时立即触发软件 Break | 编译/静态检查通过；目标板待测 |
| ADC 电流零偏 | 旧校准快速重复读取同一组 ADC 寄存器，不能证明是新采样 | IQ 启动时等待 128 组独立 EOIC，单次超时 10 ms；超时或超出 12 位 ADC 范围则不开放控制中断 | 编译通过；目标板待测 |
| 初始化失败 | 原 IQ 主函数忽略算法初始化返回值 | IQmath 自检、MTPA 表有效性及 ADC 校准失败均阻止后续启动；`FixedStartup_Status=1/2/3` 分别为算法失败/ADC 失败/完成 | 软件检查通过；目标板待测 |
| 低母线调制边界 | 旧 `Adc_Get_VoltageBusInv()` 在低于约 100 V 时让 SVPWM 回到中性值；新代码曾用 20 V | 定点 SVPWM 低母线阈值恢复为 100 V 对应的 PU 值，并加入 80 V 中性输出主机回归 | 主机测试通过；目标板待测 |
| 母线后充电 | IQ 初始化只在启动瞬间母线高于 200 V 时开启 Break IRQ | 母线后续升过 200 V 时也只开启一次；硬件故障 ISR 关闭后不自动重开 | 编译/静态检查通过；目标板待测 |
| PE4 外部停机 | 旧 EXTI4 在 PE4 下降沿触发 Break，但恢复只查 PE15，可能在 PE4 持续低电平时重开 `MOE` | IQ 恢复条件增加 PE4 必须为高电平；若仍为低电平，再次保持 Stop 并触发软件 Break | 编译/静态检查通过；PE4 实际接线/电平待测 |
| CPU 异常 | 旧异常处理无限循环，可能留下最后一次 PWM 比较值 | IQ 的 NMI/HardFault/MemManage/BusFault/UsageFault 入口直接清除 TIMER0 `MOE` 并置 Stop | 编译/静态检查通过；故障注入待测 |

`Stop` 在旧版和定点版中均初始为 1，`Foc_Mode` 选择本身**不会**自动将 `Stop` 改为 0。A2L 可找到 `Stop`，但它是 `MEASUREMENT`，因此“上位机能否按原操作写入”及写入时机需在断开功率级状态下确认。`FixedStartup_Status` 若为 1 或 2，CAN 尚未初始化，需用 SWD/调试器读取。**严禁仅凭 50% 比较值或模式为 IDLE 判断功率级安全；使能前必须实测 `MOE`、PE15、六路栅极、PE14 制动输出和继电器。**

仍未验证：目标板 ADC 注入触发是否持续到达、128 组零偏统计与原电流方向是否一致、上位机写 `Stop` 的行为、Break 锁存/恢复、紧急停机路径、ADC ISR 最坏执行时间与栈水位。上述项目通过前，结论维持“不可放行带电机/高压上电”；只可在功率级断开、限压限流的台架条件下逐步测量。

## 11. 保护逻辑与旧浮点实现的差分验证

本轮新增 `Tests/protect_legacy_parity_host_test.c`，分别链接真实旧 `protect.c` 与 IQMATH 的 `protect_iq_gateway.c`/`protect_iq_core.c`，向公开 `Protect_*` 接口输入相同的物理量。首次运行时 IQMATH 在“间歇过流累计”和“清标志后保留计数”两项失败；修正这两处后，两套可执行测试均通过。它验证的是保护函数的公开接口语义，不是仅比较两份定点代码，也不等于目标板硬件保护已验证。

| 保护项 | 旧浮点公开接口语义 | IQMATH 对照结果 |
|---|---|---|
| 瞬时过流 | 任一相严格大于 30 A 即锁存 | 30.0 A 不触发、30.1 A 触发，主机通过 |
| 平均过流 | 任一相严格大于 27 A 的采样累计至第 11 次；正常采样不清计数 | 交替输入 27.5 A/0 A，第 11 次高采样触发，主机通过 |
| 保护复位 | 清标志但保留平均过流计数 | 5 次高采样→清标志→再 6 次高采样触发，主机通过 |
| 母线过压/欠压 | 严格大于 720 V / 严格小于 20 V | 720/720.1 V 和 20/19.9 V 边界向量通过 |
| 过温/风扇 | 严格大于 80 ℃ 锁存；风扇大于 32 ℃ 开、低于 28.8 ℃ 关 | 80/80.1 ℃及 33→30→28 ℃向量通过 |
| 硬件故障标志 | `Protect_HardWareFault(false)` 锁存 | 公开接口主机通过；实际 Break ISR 触发条件与旧版不同，需上板验证 |

代码链路复核：两套固件均在主控制中断采样相电流、检查母线，在主循环检查温度；保护标志被 `Peripheral_Update_Break()` 读取并置 `Stop`，软件 Break 关闭 TIMER0 主输出。定点版为修正原接入差异还增加了 IDLE 停机、PE4 低电平拒绝重启，以及软件/硬件 Break 区分，因此**整条硬件保护链不是逐指令等同旧版**。

基准限制：当前仓库的 `FLOAT_REF` 将 `MainInt_State` 初始化为 `RUNNING`，而 `Initialization_Modules()` 仅在 `INIT` 分支调用；按此源码，该浮点启动路径不会执行保护参数初始化。必须找出此前实际上板验证所用的固件/提交，核对其启动状态与参数初始化，然后才能声称“与旧板行为完全一致”。实际 `Stop`、TIMER0 `MOE`、PE4/PE15、故障锁存与复位时序，仍需在功率级断开时做故障注入和示波器测试。

## 12. 串口监测通道与辨识调参（2026-09-29）

用户确认原浮点方案的 100 V 注入和 3～12 A 扫描值已经过实验验证；这确认的是原方案参数，不等于当前 IQMATH 固件和保护链已完成目标板等价验证。定点辨识的 `max_steps` 上限恢复为原版的 20，起点 3 A、终点 12 A、步长 1 A 实际只扫描 10 个电流点，上限改变不会增加默认扫描点。

`Buffer` 是 USART 的 VOFA JustFloat 帧，不是 CAN/CCP/A2L 变量采集。当前 IQMATH 仍发送 18 个数据 float 加 1 个帧尾；通道 3 的估计转速单独使用 10 ms 时间常数的一阶低通（配置的 5 kHz 下约 16 Hz），滤波状态由中断集成层持有，滤波值只送串口，不回传 `FixedControl_Step` 或 LESO/HFI/PLL。其余控制速度及角度保持原始估计值。

| 下标 | 数据 | 单位/解释 |
|---:|---|---|
| 0 | 位置传感器实测角度 | rad |
| 1 | 内部无位置估计角度 | rad |
| 2 | 位置传感器实测转速 | rpm |
| 3 | 仅串口显示的一阶低通估计转速 | rpm |
| 4 | 实测减估计的最短角度差 | degree，恢复旧通道定义 |
| 5 | ADC 边界母线浮点电压 | V |
| 6 | 同一母线样本进入 Q24 后回转的电压 | V |
| 7 | 同一母线样本的 Q24 原始整数，以 float 数值发送 | raw Q24 count，非 V |
| 9 | ADC 边界 A 相浮点电流 | A，恢复旧通道 |
| 10 | 同一 A 相样本进入 Q24 后回转的电流 | A |
| 11 | 同一 A 相样本的 Q24 原始整数，以 float 数值发送 | raw Q24 count，非 A |
| 15 | 定点控制器最终 d 轴电压指令 | V；旧版在高频注入前记录，新版为注入后值，二者不可直接做逐点等价对照 |
| 16、17 | d、q 轴反馈电流 | A |
| 8、12～14 | 未使用 | 初始化为 0 |

定浮点输入转换核对公式：`Bus_Q24_V ≈ Bus_Float_V`、`PhaseA_Q24_A ≈ PhaseA_Float_A`；原始 Q24 整数应分别约为 `(Bus_Float_V/800)*2^24` 和 `(PhaseA_Float_A/30)*2^24`。同一样本正常范围内的差别主要是 Q24 舍入及 float 表示误差；串口丢帧或错位必须先排除。18 float + 帧尾共 76 byte，在 5 kHz 下有 3.04 Mbit/s 净数据、约 3.8 Mbit/s 的 8N1 线路速率，当前 USART 配置为 4 Mbaud；队列满会丢帧，因此不能把串口波形当作无遗漏的 5 kHz 采样记录。

辨识已恢复原版 `Experiment.*` 人工操作接口。进入 `IDENTIFY` 时，网关把镜像初始化为 3～12 A、步长方向 +1、每点重复 3 次、512 点缓存、等待 3 个边沿，并保持 `Ud_amp/Uq_amp=0`。上位机写入的 A、V、s 浮点物理量在控制周期开始时转换成 Q24 快照；定点状态、采样缓冲和拟合结果在周期结束时转换回浮点镜像。算法核心不读取 `Experiment` 全局对象。`Identification_Voltage` 仅报告当前 D/Q 幅值的较大绝对值，不再作为人工辨识的唯一控制输入。

| 阶段 | 上位机操作 | 自动行为 | 阶段结束 |
|---|---|---|---|
| Rs | `Experiment.state=EST_RS`，确认释放 Reset/Stop 的时序 | 定点 Rs 电压递增、滤波与收敛判断 | 回到 `WAIT`，`Rs_est` 更新，D/Q 幅值为 0 |
| D 扫描启动 | 写 `start_I/final_I/step_dir`，写 `Ud_amp`，保持 `Uq_amp=0`；先写 `state=NEXT_I`，看到 `INJECT_COLLECT` 后写 `inj.State=1` | 每个电流点内部的重复次数自动完成 | 每完成一个电流点自动推进 `Imax`，但下一个点再次等待 `inj.State=1` |
| Q 扫描 | 状态机自动切到 `inj.mode=INJECT_Q` 并清零幅值；写 `Ud_amp=0`、`Uq_amp`，再写 `inj.State=1` | 完整扫描同一电流范围并拟合 `aq0/aqq` | 自动进入 DQ 准备 |
| DQ 注入 | 看到 `inj.mode=INJECT_DQ` 后写 `Ud_amp`、`Uq_amp`，再写 `inj.State=1` | `Ud` 按 `Id` 到达 `±Imax` 独立翻转，`Uq` 按 `Iq` 独立翻转；采样边沿以 D 轴为准 | 拟合 `adq`，进入 `DONE` 并请求 `IDLE` |

人工放行规则与旧浮点代码一致：同一电流点内的重复采样自动继续；切换到下一个电流点后必须重新写一次 `Experiment.inj.State=1`。D、Q、DQ 阶段切换时两个电压幅值都会清零，避免沿用上一阶段电压。辨识进入 `FAILED` 时本周期请求 `IDLE`，上层停机逻辑据此置 `Stop=1`；`Identification_Error` 保留失败原因。

`MainInt_ControlState.foc.identification.config.*` 仍是内部 Q24 原始状态，只用于底层诊断，禁止把 A/V 物理值直接写入这些字段。日常操作只写 `Experiment.*` 浮点镜像。

| 软件验证 | 当前结果 | 上板结果 |
|---|---|---|
| 旧模式停止、ADC 物理值→Q24 输入→物理值回转主机测试 | 通过 | 待测 |
| 串口估计速度滤波和角度差主机测试 | 通过 | 待测 |
| IQMATH/FLOAT_REF 构建与 IQMATH 核心对象定点审计 | 通过 | 待测 |
| 新 IQMATH A2L 生成 | 通过 | 上位机读取待测 |
| 原版人工辨识网关与完整 D→Q→DQ 状态机主机模拟 | 通过 | 待测 |
| 辨识失败请求 `IDLE` 并触发停机判据 | 通过 | 待测 |

## 13. 弱电 CAN/CCP CONNECT 超时修复（2026-10-08）

现象：更换控制板后 CAN 通道显示 UP，但 CCP CONNECT 超时、上位机接收计数为 0；用户确认同一块板烧其他程序通信正常。本轮以迁移前浮点提交 `22e6d80` 和更早的 `aa5c37d` 为通信基准，对比当前修改前 `dd50a5c`。CAN 驱动、CCP 协议核心、CRO/DTO ID、站地址和 CAN 接收中断均保持原实现，不能通过重复恢复这些文件解决新增的控制中断开销。

### 13.1 原因证据及修复边界

修改前默认 IDLE/Stop 的每次控制中断都会调用 `FocIq_Reset -> IdentificationIq_Reset -> IdentificationIq_Init`。原 ELF 确认每次逐字节清空 13,032 字节，其中六个 512 点采样数组占 12,288 字节。配置为 120 MHz、5 kHz 时每周期预算为 24,000 个时钟；该重复大块清零会严重超出预算，可能阻止主循环执行 CCP 命令解析和回复。实际目标板是否还存在其他超时原因，仍需读取运行状态确认。

| 修改 | 保持的行为 | 本轮软件结果 |
|---|---|---|
| 将小控制复位拆为 `foc_iq_reset_control` | 停止期间 PI、斜坡、相位和观测器小状态仍每周期复位；PWM 仍为原来的中性输出 | 连续 IDLE、VF/IF/SPEED/IDENTIFY 停止回归通过 |
| 辨识流程只在停止入口或模式变化时逻辑复位 | 重新进入 IDENTIFY 仍初始化；释放 Reset 后自动后端启动，人工后端保留 WAIT/人工放行操作 | 模式切换和 Reset 释放回归通过 |
| `IdentificationIq_Reset` 不再调用完整 Init | 保留配置；清系数、D/Q 结果及 valid、有效计数、累计和全部运行标志；旧采样存储被判为无效 | 新 ELF 仅清 4 + 536 + 144 = 684 字节元数据，不再清六个采样数组 |
| IDENTIFY 在 Reset 时不调用辨识 Run | 输出不注入、不会被启动请求提前放行，也不重复复位 | 持续 Stop 下启动请求无效回归通过 |
| CCP 初始化移到 NVIC/ADC 中断使能之前 | 仍在 CAN 初始化之后；不改安全启动检查、GPIO、PWM 和保护顺序 | 源码和 ELF 调用顺序检查通过 |

`Init` 仍完整清零，包括采样存储；`Reset` 是逻辑复位。首次新采样显式从零磁链开始，先覆盖电压、电流、磁链，再增加 `sample_count`；处理只访问本次有效样本。结果点的 `valid` 及所有拟合累计均清除，不能把旧缓存当作新结果。本轮未修改 CAN 波特率/引脚/ID、PWM 极性/死区、MOE/Break 导通与保护条件、ADC 校准逻辑、控制频率或原浮点算法文件，未删除源文件或改写 Git 历史。

### 13.2 可重复的软件检查与实际结果

在项目根目录依次运行：

```powershell
powershell -ExecutionPolicy Bypass -File Tools\RunFixedHostTests.ps1
powershell -ExecutionPolicy Bypass -File Tools\BuildSequential.ps1 -Backend IQMATH
powershell -ExecutionPolicy Bypass -File Tools\BuildSequential.ps1 -Backend FLOAT_REF
powershell -ExecutionPolicy Bypass -File Tools\GenerateA2L.ps1
```

| 检查 | 实际结果 | 判定 |
|---|---|---|
| 9 项主机测试：基础算法两后端、保护两实现、FOC、模式网关、监测、辨识两后端 | 全部通过 | PASS |
| 连续 IDLE 64 周期；VF/IF/SPEED/IDENTIFY 各 8 个停止周期 | PWM 中性、小控制状态逐周期复位，辨识元数据不反复清零 | PASS |
| 污染六个采样数组及所有运行状态后 Reset | 配置和采样存储保持；元数据与正常 Init 的逻辑初值一致；首次样本不引用旧磁链 | PASS |
| 无效配置及辨识重新运行 | 五类无效配置保持 FAILED；复位后完整人工 D→Q→DQ 流程完成 | PASS |
| 负对照：通过编译器标准输入编译 Git 中修改前的辨识实现，使用新测试 | 按预期退出 1：`FAIL reset erased sample payload array 0`；未覆盖源码 | PASS：测试能捕获原问题 |
| IQMATH 构建及 13 个定点核心对象审计 | 编译/链接成功，无禁用的浮点算术指令或浮点运行库调用 | PASS |
| FLOAT_REF 构建 | 编译/链接成功，原浮点源未修改 | PASS |
| ELF 启动调用顺序 | CAN 初始化 → CCP 初始化 → NVIC/ADC 中断使能 | PASS |
| ELF Reset/栈记录 | Reset 不调用 Init；只清 684 字节；Reset 自身静态栈为 16 B | PASS；非整个中断最坏栈占用 |
| A2L | 从新 ELF 生成，模式枚举六项及诊断地址保留 | PASS |

Q24 主机测试使用 `Tests/iqmath_mock`，用于确定性的数值和状态回归；真实 IQmath 库已通过目标固件链接检查，但实际板上库自检、实时周期及 CAN 通信仍待测。主机测试没有被当作硬件实测结果。

链接区域占用：IQMATH Flash **81,196 B**、RAM **35,208 B**；FLOAT_REF Flash **45,464 B**、RAM **20,520 B**。这不是运行时栈水位或中断周期的实测结果。

本次产物 SHA-256（之后重新编译或生成会更新）：

- IQMATH ELF：`86AE96ABA70C3E80C8F46F19F08AE787CA7D385EEFD9252EEC9AF9495E02C9A6`
- IQMATH A2L：`29CFAFB0D4080E7B769C30FEFDCBAB3E4A2A429134A81698FD1F7C52F03A1EC8`

### 13.3 重新烧写后的弱电检查

1. 保持功率级断开、`Stop=1`、`Foc_Mode=IDLE`；点击 VS Code Flash 烧写新 `build-iqmath-softfp/GD32F30xProg1230.elf`。
2. PiSnoop 离线后重新加载同目录新 `GD32F30xProg1230.a2l`，再连接；保持原 1 Mbps、CRO=2、DTO=1、station=0。
3. 确认 CCP CONNECT 成功且 RX 计数增加，不仅是 CAN 通道显示 UP。读取 `FixedStartup_Status`，应为 3。
4. 读取 `MainInt_CycleTicks` 和 `MainInt_MaxCycleTicks`；在 120 MHz 下除以 120 得到微秒，正常周期应小于 200 us，并留出主循环处理余量。超时会拖长实际入口间隔，不应仅用 `MainInt_FrequencyHz` 或负载千分比判断。
5. 若仍超时，用 SWD 检查启动状态、PC/IPSR 和 CAN 接收队列 head/tail，区分启动失败、异常停机、接收失败和主循环饥饿；不要仅通过释放 Stop 或增加母线电压尝试恢复通信。

当前 ELF 的诊断地址（重编译后须与对应 A2L/nm 再核对）：`FixedStartup_Status=0x20007780`（16 位），`MainInt_MaxCycleTicks=0x20000124`、`MainInt_CycleTicks=0x20000128`（32 位）。

**目标板 CONNECT、实际控制中断耗时及最坏栈水位：待用户重新烧写后记录，未声称已经测得通过。** 停止时 LESO/HFI 仍按原定点路径计算，辨识后同步 MTPA 重建也可能暂时阻塞主循环；本次没有扩展修改这些算法行为。

## 14. 旧 A2L 调试映射恢复与必须告知的逻辑差异（2026-10-08）

用户截图中的 `??` 经逐名核实：其中很多旧符号不在 IQMATH ELF/A2L 中，而不是简单的 CAN 故障或 A2L 未刷新。IQMATH 没有链接旧浮点 `motor.c/protect.c/main_int.c/leso.c/hf_injection.c/sensorless_interface.c`，原静态/全局调试变量未全部被物理镜像网关替代。此前“保留旧调试体验”的目标尚未完全实现。

### 14.1 本轮实际恢复的映射

| 原变量/类别 | 当前单位与读写作用 | 生效/刷新时机 |
|---|---|---|
| `Motor_Position` | 编码器原始计数，float 观察；不是角度 | 每次位置输入 |
| `Motor_Theta_Elec/Mech`、`Motor_Speed` | rad/rpm 观察镜像 | 位置、角度、速度公开接口 |
| `Motor_Position_Offset`、`motor_param.Position_Offset` | 可写整数编码器零偏，两入口驱动真实 config | 下一次位置输入；同时写时结构体入口优先 |
| `Motor_Position_Scale/Pn/Initialized/SampleFreq/Speed_Prescaler`、其他 `motor_param.*` | 初始化/有效配置快照；除零偏外不是本轮支持的在线调参入口 | 不支持字段写入会在位置边界读回有效值 |
| `Protect_CurrentMax` | 可写 A，相电流瞬时阈值及 90% 累计阈值实际使用 | 下一次相电流保护调用 |
| `Protect_BusVolRate/BusVolFluc` | 可写 V，真实过压阈值为额定值加波动量 | 下一次母线保护调用 |
| `Protect_TempMax` | 可写 ℃，驱动真实温度配置 | 温度/风扇保护调用 |
| `Flag` | 真实保护锁存的观察镜像，不是直接清故障命令 | 保护调用/取标志时同步；写镜像不解除内部故障 |
| `Foc_Ramp_Speed_Handler.slope` | 可写 rpm/s，换算成真实速度环每步 Q24 增量；0 表示冻结斜坡 | 下一次控制周期参数快照 |
| `Sensorless_ThetaEst/SpeedEst/ThetaErr/SpeedErr` | 当前被选中的定点估算角度 rad、速度 rpm、实测减估计的最短角误差 degree、速度误差 rpm | 新增观察镜像每 50 控制周期发布，配置 5 kHz 时为 100 Hz |
| `Leso_Theta/Speed/Theta_Err/Speed_Err`、`Hfi_Theta/Speed/Theta_Err/Speed_Err` | rad/rpm/degree/rpm 观察镜像 | 同上；不作为写入算法的参数 |
| `Leso_CurEst/Current`、`Hfi_IClarkFdbk/Resp/Filt` | A，真实定点电流估计、采样或滤波状态的物理镜像 | 同上 |
| `Leso_EmfEst/EmfEst_dq/Emf_Filtered/Emf_Slow_Filtered` | V，真实定点反电势及滤波状态 | 同上 |
| `Leso_Voltage` | 当前 FOC 最终电压参考 V；不是上一周期已送入 LESO 的输入缓存 | 同上；这一时序差异不能冒充原输入监测 |
| `MTPA_Inductor.d/q` | H，实际使用的动态电感观察镜像 | 同上；原 `Foc_Inductor.d/q` 保持每控制周期更新 |
| `mtpa_table._n_.T_req/Psi_s/gamma/Id/Iq/Ld/Lq/valid` | Nm/Wb/rad/A/A/H/H/bool，只读观察；按目标转矩升序排列以保留旧索引含义 | 初始化、后台表提交后；不在 ADC ISR 或关中断临界区转换整表 |

新增镜像不会反馈进核心算法。原 FOC 物理镜像和串口监测仍按原配置更新，新增监测的 100 Hz 刷新不是将控制频率降到 100 Hz。100 Hz 仅降低平均开销，仍必须测量包含镜像发布周期的 `MainInt_MaxCycleTicks`，不能以平均频率推断最坏中断已小于 200 us。

### 14.2 本轮新增且明确告知的边界行为

- 零偏当前只接受 `0..Position_Scale` 内的整数计数；分数、NaN/Inf、负值及越界写入拒绝，并读回真实生效值。旧浮点零偏可容纳分数，这项限制不是完全等价。
- 改零偏时同步平移速度差分历史，避免把零点校准当成真实转动产生速度尖峰；真实运动仍正常计算速度。
- 保护阈值与速度斜坡非法写入拒绝并读回上次有效值；不清除故障或平均过流累计。保护判据、20 V 欠压、风扇滞环未改。
- `Flag` 是观察镜像，写 0 不直接解除定点核心锁存；原浮点直接写内部静态 Flag 的方式不适用于该镜像。软件清故障仍由既有 `Protect_Reset_Flag()` 路径完成，后续恢复顶层操作命令时须明确其入口和停机条件。
- 其他 `motor_param.*` 字段当前只读，尤其极对数、旋变极对数、刻度、`theta_factor` 不能认为在线写入已经生效。
- 显示用最短角误差恰好半周时，新定点约定保留 +180 degree，旧 `wrap(ref-est+PI)-PI` 为 -180 degree；这是边界显示差异，不改变控制。
- 电机 core 此前已将角度归一化，并在位置输入时计算速度；旧浮点 setter 未主动归一化、速度更新位于 Getter 调用。恢复镜像没有把这些既有迁移差异改回旧实现。

### 14.3 无位置路径已确认的差异和未恢复项

以下不是本轮新增算法选择，而是之前迁移已经存在、此次审计必须说明的问题：

| 项目 | 原浮点结构 | 当前定点结构/缺口 |
|---|---|---|
| 顶层控制 | `MainInt_State` 的 INIT/RUNNING/SENSORLESS/EXIT，加 `Sensorless_Enabled` | 当前由 `Foc_Mode/Stop/MainInt_UseRealTheta/Sensorless_Method` 管理；两个旧命令尚未恢复，不能简单挂同名空变量 |
| 方法取值 | 旧枚举 FLYING=0、LES_OBSERVER=1、HF_INJECTION=2 | 当前位标志 0全关、1 LESO、2 HFI、3两者、4飞车位；旧0含义改变，且没有原英文枚举表 |
| PLL | 旧 `Sensorless_Theta_PID` 统一融合 PLL，包含速度相关带宽调整及 Kp/Ki 上限 | 当前 LESO/HFI 各自独立 PLL；旧统一 PLL 调参没有对应的等价写入对象 |
| 估算反馈切换 | 旧切换同时参考目标速度、斜坡速度和反馈速度 | 当前主要依据 LESO 估计速度与固定滞环；判据不是逐逻辑等价 |
| HFI 注入 | 旧接口切换与 Hfi_Enabled 关联 | 当前 HFI 方法位开启且非 Reset 就叠加注入，即使仍使用传感器反馈；默认 40 V，不能视为纯后台观察 |
| LESO 带宽/电感 | 旧 Gain/Factor/Wc_Min/Max 动态路径及动态电感接口 | 当前配置主要由默认离散系数初始化；旧带宽和 PLL 参数物理网关未完整接回 |
| 飞车 | 原飞车启动流程 | 当前 `FlyingIq_Run` 仅延迟计数并置 completed，FOC 未用完成标志完成原飞车控制流程；不能声称等价完成 |

尚未恢复的核心控制/调参名包括：`Sensorless_Enabled`、`MainInt_State`、`Sensorless_Theta_PID.*`、`Sensorless_PLL_Wc`、`PLLKPMAX/PLLKIMAX`、`Leso_Theta_PID.*`、`Hfi_Theta_Pid.*`、`Leso_Wc/Gain/Factor/Wc_Min/Wc_Max`、旧切换阈值及相关使能命令。原函数内部临时拟合量/计数如 `test_cnt/stay/hfi_reverse_cnt/leso_reverse_cnt` 也不能没有真实对应状态就随便补值。需要选择以旧统一 PLL/切换算法为基准恢复，还是明确保留当前架构；本轮没有擅自做这一选择。

当前接口仅说明真实代码作用：`Sensorless_Method=1/2/3` 选择估算器，`MainInt_UseRealTheta=0` 才选择估算反馈；为1则仍用传感器。Method=0时即使 UseRealTheta=0也回退传感器。弱电 Stop=1 下 PLL 处于 Reset，不代表算法能在无电机反馈下验证估速收敛。`Foc_Reset` 是 Stop/IDLE 自动生成的状态，不应当人工启停命令。由于无位置架构仍存在上述不等价项，不建议按旧调参界面直接放行无位置闭环。

之前 Stop/MOE/Break、PE4 恢复条件、异常停机和 ADC 零偏初始化的改变也不是单纯“浮点改定点”，详见第10、11节；本轮未继续改这些硬件逻辑。

### 14.4 本轮软件结果与使用步骤

- 13 项主机回归全通过，包括两个数值后端的电机网关、实际保护调参、只读观测/表镜像以及原有算法/保护/辨识测试；Q24 主机测试使用 mock，不冒充上板验证。
- IQMATH 最终构建、13个核心对象无浮点算术审计及新 A2L 生成通过；21 个重点原名抽查均在 A2L 中。
- 当前 IQMATH 链接占用 Flash 84,980 B、RAM 37,152 B；额外接口占用和镜像发布最坏耗时仍需在板上核对。
- 新增符号改变了 RAM 地址；旧 ELF/A2L 和第13节旧诊断地址不能混用。当前生成版本 `FixedStartup_Status=0x20007E7A`、`MainInt_MaxCycleTicks=0x2000012C`、`MainInt_CycleTicks=0x20000130`，重编译后仍需复核。
- 重新烧写后必须离线重新加载 `build-iqmath-softfp/GD32F30xProg1230.a2l`，再读取恢复的名字。监测变量只读；可写参数只使用本节明确列出的入口，并保持 Stop=1进行弱电读写回归。

本轮没有删除源文件、改写 Git 历史或烧写目标硬件。完整旧无位置调试操作兼容仍未完成，后续需按确认的算法方案继续。

## 15. 原浮点实际控制结构恢复（2026-10-08）

### 15.1 基准、原则与范围

- 工作树基准 HEAD `dd50a5c`，纯浮点算法基准 `22e6d80`；保留原浮点源并直接用于对照，不只互比两个新后端。
- 恢复原活跃公式、模块顺序、PLL/模式和人工操作；不将原被注释的函数误算成正在执行的功能。
- 状态由调用方持有，公共控制和观测器提供 Init/Run（或分阶段接口）/Reset。原隐藏静态积分器、相位和采样记忆改为结构体字段。
- 本轮不修改 CAN/CCP 驱动、PWM 极性/导通映射、MOE/Break 或 ADC 校准代码；不删除文件、重写 Git 历史或自动烧板。
- 详细操作入口：[各模式操作与参数](IQmath_Mode_Operation_Guide.md)。旧章节中的位标志模式、双 PLL 调参和缺失镜像描述均为历史。

### 15.2 实现记录

| 模块 | 本轮实现 | 对照/注意点 |
|---|---|---|
| 公共无位置接口 | SensorlessIq 四阶段 FilterCurrent→UpdatePosition→Calculate→InjectVoltage；单公共 PLL 输出电气 rad/s，PU 电角积分、10点平均及原二阶速度滤波 | 64位乘积残差保留长期积分低位；LESO/HFI 局部 PLL 不执行 |
| LESO | 原动态 Gain×Factor×We 带宽及上下限、动态 MTPA Lq（最小1mH）、电流/扰动模型和归一化投影鉴相 | 沿用原有带符号速度规则；不是 atan2 角差替代模型 |
| HFI | 原两点基波/高频差分/极性解调、原物理低信号阈值；仅电压注入时翻转极性 | 去除迁移擅加的响应 IIR；Sensorless_Enabled/Reset/局部 Enabled 分层门控 |
| VF/IF | 共享零起点锯齿计数、各自复位边沿/保持相位、π/2冻结判据 | 普通模式切换不额外清所有 PI/斜坡；use_sensor 时 offset=0 |
| SPEED/STARTUP | 原准备触发、只清速度PI/斜坡的准备阶段、主控制频率分段增益更新、分频速度环、斜坡上下限及 q 电压限幅 | SPEED Stop 清目标，需要解除后再写给定；STARTUP 只是持续5V准备分支 |
| MTPA | 联合Id/Ld/Lq线性插值、末区间外推与原更新时机；保留旧Id查询wrapper | 64位安全外推；拒绝无效表；不复制旧NULL输出指针解引用 |
| 辨识 | 前采样区间电压、flux[0]=0、正峰/有符号电流、每周期人工注入门控；D→Q→DQ、默认两轴幅值0、每新电流点放行 | Q48矩阵乘积累计、归一化矩阵、不可表示/病态拟合失败；SSR总和物理显示恢复 |
| A2L | 原 Sensorless_Enabled/Method 枚举、MainInt_State、统一PLL、LESO/HFI参数、斜坡、状态/模型/Rs名字和物理镜像 | 写入是否生效和刷新频率均在操作文档分类；PID非法输入成对回退 |

### 15.3 自动测试结果

17 项宿主编译/回归（`Tools/RunFixedHostTests.ps1`）通过；数值宿主后端使用 IQmath mock，真实目标库链接/反汇编另行检查。以下为指定测试向量的实际结果，不推断任意工况或目标板已通过。

| 测试 | 预期 | 实际记录 |
|---|---|---|
| 公共 PLL：直接调用旧浮点源，20,000周期闭环 | 电角差≤0.5° | 最大0.005888°，转速差0.016602rpm，电omega差0.003937rad/s |
| HFI 旧源电流/投影/门控 | 物理公式和注入极性一致 | 电流差0.000003A，投影差0.000019；停机/禁用不翻注入极性 |
| LESO 旧源动态模型 | 电流/EMF与投影一致 | 电流差0.000027A，EMF差0.000363V，投影差0.000180；0/负Lq按原1mH处理 |
| 49组策略及Flying | 原判据/窗口/实际计数路径 | PASS；原FLYING=0位判断恒假，Flying仅8周期等待 |
| FOC旧PI/相位参考 | 条件积分、启动、相位/保持、分频一致 | PASS；用真实旧Pid_Update及旧相位参考，不测试新增双PLL自洽 |
| MTPA旧优化器51点及实际旧插值器 | 控制量≤0.5%参考量级，插值公式一致 | 最大Id差0.03335A、Iq差0.01698A；Ld差0.242mH/0.2152%，Lq差0.00997mH/0.02974%；lookup PASS |
| 辨识旧采集/峰值/暂停直接对照 | 上一电压区间、首点0、正峰、暂停保留状态 | D/Q/DQ数组、终止窗口、正负不对称峰与暂停/恢复 PASS |
| 完整辨识双后端，4716周期 | 五参数相对差≤1% | FLOAT=(0.09540835,21.97333717,0.20138133,0.51701158,0.22925439)；Q24=(0.09540790,21.97310066,0.20137316,0.51700509,0.22957969)，最大约0.142% |
| 小磁链/64位矩阵 | Q轴系数(.5,5)、D轴(.2,2)，DQ系数10误差≤1% | PASS；DQ=10.0004；大整数商避免左移溢出，异常累计/系数显式失败 |
| 旧物理参数/A2L门控/保护 | 写入真实作用、非法回退、核心不读float镜像 | PASS；包含MainInt状态、总使能、斜坡、电机零偏、保护及停止逻辑 |

### 15.4 仍需保留/说明的差异与限制

1. 原 FLYING=0 被当位标志、局部PLL调用注释、Hfi_InjFreq不驱动每拍翻转波形，都是原代码事实，未包装成完整飞车功能。
2. 保护 Reset 在当前周期更早用于滤波/公共PLL，不恢复原延迟一拍的响应；INIT 在停机背景重置算法，EXIT不清保护/禁Break。未改硬件门极逻辑。
3. MTPA搜索停止条件仍是定点有限迭代，不是旧浮点容差/最后10候选的逐位替换；保留Iq排序和无效表拒绝。上述误差只覆盖当前51点模型。
4. 原DQ的重复计数按样本递增，当前按完整采集轮次；旧Q.R²写到D.R²和NULL解引用不复现。人工暂停保留原last_Vd/Vq，但暂停样本不能当连续实验拼接，需整点重采。
5. 旧1e25积分/状态限幅不能用Q24表示，按明确表示范围/饱和诊断处理；这不是把实际电流保护阈值放宽到约3840A。
6. Handler速度PI的Kp/Ki当前为可写基值，真实调度值在新增Foc_Pid_Speed_Effective物理镜像；避免反馈放大后的值作为下一拍基值。额外观察别名/辨识过程约100Hz。
7. 严格“零隐式全局引用”尚有技术债：McMath_Diagnostics仍是数值层公共诊断计数，不参与控制输出决策，但不满足字面上的完全无全局写入。不能以全部控制状态已结构体化宣称此条件全部验收；后续需显式传入诊断上下文。
8. 完成宿主回归/目标编译不等于没有致命bug或功率级安全证明。控制ISR实测耗时/栈余量、真实TI库数值、CAN/A2L读写与模式运行结果仍未在目标板代填。

本轮产物、地址与占用以下面最后一次构建记录为准；所有旧地址仅是历史数据。重新Build/Flash后加载同ELF生成的A2L；当前工作树尚未提交/push。

### 15.5 最后一次构建与配套 A2L

| 项目 | 本轮记录 |
|---|---|
| IQMATH | 68编译单元，softfp；Flash 94,308 B / 256 KB，RAM 37,976 B / 48 KB |
| FLOAT_REF | 64编译单元，hard-float；Flash 45,464 B，RAM 20,520 B；构建通过 |
| 算法审计 | 14个核心对象无浮点算术指令/运行库调用，通过；边界转换和测试浮点不算核心算法 |
| 配套产物 | build-iqmath-softfp/GD32F30xProg1230.elf、.hex、.a2l；68个对象均比对应源文件新 |
| ELF SHA-256 | 2DC41F965DEF63910D29D0C42FAB5B4FD601E7E04A2821A764FA12B4D6820EF7 |
| HEX SHA-256 | BFE16D955C1DBED7F2C5834E46F0F1B888956F509DF4074DE6C4CAE156120DAD |
| A2L SHA-256 | E148DA793F7E87803B2441B02E10A4FDA3ED1CC6D1C99094430E33D6BD327FB8 |
| IQmath校验 | 头文件896A09E33B72C7F14BB7C5F6E9147FD8D1F9BAE8BFD816A27069B7C900607E08；库507F3030752DA70826CB46861201D76D241E177DD5654DC3B52E6B18C20AE0B2，未更换 |
| A2L检查 | 16个关键名抽查通过；FocMode_t六模式、sensorless_method_t原0/1/2、DeviceStateEnum_t四状态的英文COMPU_VTAB均存在 |
| 编译栈报告 | Main_Int_Handler 152B，FixedControl_Step 344B，FocIq_Run 240B，MtpaIq_BuildTable 176B；仅单函数静态栈，不能当含IQmath/中断嵌套的峰值实测 |
| 目标板中断/栈 | 待填写实际DWT/高水位结果，不用宿主耗时推断5kHz中断有足够余量 |

本次地址仅供对应上述ELF的诊断：FixedStartup_Status=0x200081B6（16位）；MainInt_MaxCycleTicks=0x200001B4、MainInt_CycleTicks=0x200001B8（32位）；MainInt_State=0x20000129、Sensorless_Enabled=0x200081B3、Sensorless_Method=0x200081B2（本次均8位，编译采用short-enums）。优先加载A2L按名字访问，不根据旧表猜大小/地址；重编译后全部地址重新确认。

全套17测试在最终源码及2A/反向步长新增回归之后再次通过。独立集成复核未发现正常流程新增阻断问题；这不是零bug承诺。没有提交构建产物、没有Git提交/push、没有烧写硬件。

## 16. INIT和PE4恢复及串口显示滤波核查（2026-10-08）

### 16.1 本轮范围与保留边界

- 按用户明确要求，Stop=0恢复条件重新只检查PE15高电平；保留原PE4下降沿EXTI4停机、PE15硬件BRKIN、Stop和保护锁存。未改Timer输出极性、死区、CCR映射、控制频率或EXIT行为。
- INIT恢复重新校准ADC电流零偏和清旧保护故障。先Stop=1、IDLE、直接禁MOE，再暂禁ADC控制IRQ，重建算法并使用已有128组新EOIC校准；算法/校准失败Status=1/2、EXIT，不清故障、不恢复ADC IRQ。
- 成功INIT按新鲜母线值严格>200V重新启BRK中断，不依赖旧FixedBreakIrqArmed镜像；低母线启动新arming epoch，允许之后充电时正常arming。保留在线保护阈值与平均过流累计，不复制整个旧Initialization_Modules的参数重置。
- 新增集成层FixedHardwareFaultEvents：实际BRK故障分支记录事件；INIT只在本次无新事件时于短临界区清旧锁存/重新arming。构表或校准期间的新瞬态故障不得因PE15恢复高而丢失；仍存在的PE15/母线/温度异常会重新锁存。成功后仍Stop=1、IDLE，不自动恢复MOE。
- 校准必须在真实相电流为零且前端稳定时执行；MOE关闭不代表运动电机已经无残余电流。CAN接收IRQ保持，但构表/校准期间主循环CCP回复可能延迟。
- Buffer通道3额外一阶显示低通τ=10ms，5kHz时α=1/51、截止约15.8Hz，已初始化阶跃95%约30.4ms/99%约46.6ms；首样本直通。返回值只进Buffer，不进FOC/LESO/HFI/PLL。内部公共PLL另有原10点平均和二阶10Hz滤波，参与控制，未修改。

### 16.2 软件验证记录

本轮新增真实hardware_interface.c两后端停机/恢复测试，以及真实main_int_iq.c的INIT/监测隔离测试。最终完整20项宿主回归通过；代码冻结后的两后端重新编译、14个算法对象审计和配套A2L生成通过。

| 检查 | 实际结果 | 状态 |
|---|---|---|
| 真实Stop/恢复函数，两数值后端 | 冷启动Stop、100次持续Stop、PE4/PE15四组电平、故障锁存拒绝恢复、BRK IRQ开启不请求MOE；FLOAT_REF恢复函数预处理语句与22e6d80一致 | PASS |
| 真实INIT分支 | Ready三类失败、校准失败均不清锁存/不恢复IRQ；成功重校、200V严格边界、旧fault后重arm、清后母线/PE15/温度重新锁存、保持Stop/IDLE/MOE禁用 | PASS |
| INIT中瞬态fault | 构表/校准期间新事件及uint32单次回绕均保留故障、不重arm；短临界区与原PRIMASK保持 | PASS |
| 显示速度隔离 | 两种显示alpha下真实Main_Int_Handler送入控制的快照和完整控制状态相同，只有Buffer3不同；默认alpha=1/51、首样本直通、152拍即30.4ms达95% | PASS |
| 负对照 | 仅内存中去掉事件guard或故意把显示速度回传控制，两组均被新测试捕获；未覆盖源文件 | PASS |
| 背景arming竞态复核 | 外侧读取Udc后，临界区内重新判断Armed，故障ISR禁IRQ动作不会被后台覆盖 | 源码复核通过 |
| IQMATH最终构建 | 68单元，softfp；Flash94516B，RAM37984B；全部对象比对应源新 | PASS |
| FLOAT_REF最终构建 | 64单元，hard-float；Flash45464B，RAM20520B；IQ新增fault计数不进入浮点分支 | PASS |
| A2L | 本轮ELF重新生成，Stop/Foc_Mode/MainInt_State/FixedStartup_Status/Adc_VoltageBus/Foc_BusVoltage及新增FixedHardwareFaultEvents存在 | PASS |

本轮IQMATH产物SHA-256：ELF `12C015ABA0F7B1421FFF6F929CEFF55D21236DEEBEE3FEFA6D390B6679AB238F`；HEX `E99753437A76F09ACEE6EA8A8A97AEEC5A0E2B7C71B6AC9828F525677828EA9C`；A2L `2440EAC00B6256266D3B1D7E6C5EBED24B0EF103660AC4FFEEC747EF21CDD017`。重新生成A2L的时间戳也会改变文件校验值，按同ELF地址配套判断，不使用上轮地址。单函数编译栈帧MainInt_ControlBackground40B、Main_Int_Handler152B、后台arming16B，不是峰值栈实测。

宿主外设mock只验证代码条件/寄存器请求，不能作为真实门极波形或强电安全证明；当前独立复核未发现本轮新增软件阻断问题，不作零bug承诺。INIT保留128组新样本校准、后台执行、新fault防误清及在线保护参数，不宣称整个旧INIT逐指令等价。软件BRK一次、故障分类、异常关输出、EXIT不清故障等其它差异未撤销。真实IRQ耗时、栈水位和门极/Break实测仍未代填。本轮未烧写、未Git提交/push、未删除源码或Git历史。

## 17. 停机抢占修复、Flag写入恢复与回归（2026-10-12）

### 17.1 对照与问题复现

对照原纯浮点提交`22e6d80`，工作分支HEAD仍为`dd50a5c`，本轮包含此前未提交的迁移修改。用户要求恢复浮点停机效果并测试。核查发现以下两个问题；之前20项顺序测试虽通过，未覆盖这些交错，旧“未发现阻断”结论不能替代本轮结果。

1. 后台恢复分支清Software_BRK后被ADC过流处理抢占；ADC关MOE并置Software_BRK，返回后台又使能MOE。IQ版本之后因Software_BRK已为true不再产生BRKG，模拟结果为故障锁存、Stop=1但MOE仍为1，连续三次停止检查仍如此。相同交错下原浮点下一次检查重新产生BRKG、关闭MOE。原ELF反汇编确认存在该指令窗口。
2. 主ADC处理中先保存stop=false，算法运行中PE4中断置Stop=1，算法结束又回写旧false。当母线尚未超过200V、BRK IRQ未使能时，PE4事件可能没有额外故障位，随后恢复分支可重新使能。真实Main_Int_Handler宿主复现确认停机请求丢失。

### 17.2 本轮实现与保留差异

- IQ `Peripheral_Update_Break()`在每次停止检查直接关闭MOE，不再把“已发过一次BRKG”当作输出必然关闭的证明。仍只首次产生软件BRKG，避免恢复重复高优先级中断造成通信/主循环饥饿；恢复的是原持续关输出效果，不是复制旧重复IRQ实现。
- 故障、Stop、PE15判定及MOE操作放入保留调用者PRIMASK的短临界区。解屏后重新读Stop；不清真实中断pending，不在恢复分支写旧Stop=0。PE15低在该次调用立即关输出，PE4仍只保留原下降沿停机事件。
- 算法结束仅允许新增Stop请求，再调用当前保护检查；不回写算法开始前的旧stop。覆盖PE4、硬件故障及模式退出发生在算法执行期间的情况。
- `Flag`恢复为A2L可写的唯一有效故障锁存，写0清位，写有效组合位影响保护返回值。每次核心检测只计算本次新增位，网关原子OR到Flag；不会由旧镜像复活已清故障或覆盖嵌套ISR新故障。核心状态仍由调用者显式传入。
- 写Flag=0不清Stop、不清平均过流累计，不重校ADC，也不重新开启被故障ISR关闭的Break IRQ。持续电流/电压/温度异常在下次对应检测重锁；PE15低仍阻止恢复。需要ADC重校及重新武装Break时仍使用成功INIT流程。
- 未更改保护阈值、门极极性、死区、PWM/CCR映射、实际控制频率或算法系数；FLOAT_REF硬件停止分支保留原行为。INIT后台/128组校准、保留在线阈值、新故障不误清，EXIT不清故障等已记录差异仍保留。

### 17.3 测试方法与实际结果

执行`Tools/RunFixedHostTests.ps1`，扩展现有测试后完整20项通过。使用真实保护核心/网关、真实硬件接口或真实主中断实现；外设和抢占副作用由宿主模拟，不等同于目标板强制故障实测。

| 验证内容 | 步骤与预期 | 实际结果 |
|---|---|---|
| 持续Stop关输出 | 已经Software_BRK=true时模拟MOE意外为1；下一Stop检查必须关断，重复100次 | 修复前断言失败；修复后PASS，且不重复发BRKG |
| 恢复抢占 | 在进入临界区前、PE15读取后、MOE使能前分别插入ADC过流或PE4事件；事件后Stop保持1、MOE为0 | 六种交错及各后三周期PASS |
| PRIMASK与pending | 调用者PRIMASK为0/1均正确保留；已屏蔽情况下事件挂起，调用者解屏后执行停止 | PASS，不丢挂起事件、不擅自解屏 |
| 算法期间异步Stop | 未武装BRK的100V情形下注入PE4停止、另测硬件故障及IDLE/EXIT模式请求；后四周期reset必须为true | 修复前17条断言失败；修复后全部PASS |
| Flag物理接口 | 写0、有效组合位、各类持续异常重新置位、正常样本不恢复旧位、清位不清累计 | PASS |
| Flag抢占交错 | 真实电压核心检测后插入硬件故障、CCP清零、清零后硬件故障；新位保留，已清旧位不复活 | PASS |
| 原保护向量 | 真实原protect.c及IQ gateway/core分别执行相同过流/电压/温度/锁存向量 | 两后端PASS；严格边界存在Q24微小量化差异 |
| 原其它回归 | FOC模式、MTPA、辨识、统一PLL/HFI/LESO、INIT新故障保护及串口滤波隔离 | 全部PASS |
| 定点对象审计 | 14个算法对象检查浮点算术指令/运行库调用 | PASS |
| ARM代码复核 | 真实ELF恢复函数先CPSID再判定/写MOE，按原PRIMASK恢复后读取Stop；Flag合并为LDREXB/STREXB | PASS，无未解析符号 |

这里的抢占覆盖ADC/EXTI/BRK等可屏蔽中断；不能解释成穷举了NMI、全部异步硬件电平变化或实际功率器件响应。目标板中断最坏耗时、峰值栈和真实门极波形仍没有本轮实测值。

### 17.4 构建与交付记录

| 项目 | 实际结果 |
|---|---|
| IQMATH | 68编译单元，softfp；Flash 94,756 B，RAM 37,984 B；源文件均不晚于对应对象 |
| FLOAT_REF | 64编译单元，hard-float；Flash 45,464 B，RAM 20,520 B |
| 配套产物 | build-iqmath-softfp/GD32F30xProg1230.elf、.hex、.a2l均已重新生成 |
| ELF SHA-256 | A29722AB0633FC5626260B120B98EC0A0D372FBEEC32428AD3A2F2C4516EB5F7 |
| HEX SHA-256 | 75C635195001697B1C2BDAD8B5ADE4E661A95A766020203AE88FF69489D318A5 |
| A2L SHA-256 | 7F2918662C3F10A076E3F69AD59707A734C73E088E995E0C739ECD6B746D352B |
| A2L抽查 | Flag、Stop、Foc_Mode、MainInt_State、FixedStartup_Status、FixedBreakIrqArmed、FixedHardwareFaultEvents均存在，枚举仍保留 |
| 单函数静态栈 | Main_Int_Handler 144B，MainInt_ControlBackground 40B；Protect_PhaseCurrent 152B、Protect_BusVoltage 136B、Protect_HardWareFault 104B、Protect_Temperature 112B；不是含嵌套中断的峰值实测 |

本轮已完成软件修复、回归、构建和同ELF的A2L生成，没有烧写目标板、提交/push Git或删除用户源码/远程历史。下一步由用户Flash本轮IQ固件并重新加载配套A2L；先确认停止时MOE/POEN关闭及停机事件保持，再进行后续电机工况。若编码器仍未连接，不能按默认有感SPEED闭环启动；采样正常不等于位置反馈已经有效。

## 18. 运行中INIT状态保留与积分限幅（2026-10-12）

### 18.1 对照发现与本轮范围

再次对照`22e6d80`真实浮点源码，发现原运行中INIT不会清总无位置使能/角度补偿，也不调用MTPA初始化建表；上一版定点却复用了完整冷启动Init。结果是丢失已辨识模型，并可能留下`Sensorless_Enabled=0、MainInt_UseRealTheta=0`的不更新估计反馈组合。用户要求恢复这两项原行为，本轮实现限定于冷/热初始化分离及配套回归；没有更改PWM、保护阈值、门极导通逻辑、控制周期或LESO数学限幅。

- `FixedControl_Init`保持完整冷启动：不读取调用方未初始化存储，首次上电使用默认模型建表。
- `FixedControl_Reinitialize/FocIq_Reinitialize/SensorlessGateway_Reinitialize`用于运行中INIT，原位初始化，不将整个状态或MTPA表复制到2KiB目标栈上。
- 保留总使能、角度补偿、角度来源、原有效方法/局部使能、共享估计角速、10拍均值累计、LESO电流/扰动/EMF、HFI解调历史/极性及飞车历史。原INIT会重置的公共PID、速度IIR及参数仍重置；公共PLL积分清零，默认积分限幅恢复700rad/s。
- 保留已接受MTPA参数、整表和动态电感；保留辨识结果及`Experiment`镜像。未提交的后台候选事务取消；后台建表过程中若收到INIT，提交前检查阻止发布候选。此候选取消是新后台架构的明确策略，旧同步建表没有对应事务。
- 保留历史DONE/FAILED后，仅在当前模式为IDENTIFY时才因辨识完成/失败自动退IDLE，避免历史结果强制退出其它模式。
- 有效待处理物理写入先读取，NaN/越界写入按先前接受值回退，不能用无效值污染热INIT的新缓存。已接受表原本无效则继续无效，由Ready检查阻止后续启动，不偷偷换回默认表。
- 仍保持INIT期间Stop=1、IDLE、MOE关闭，保留ADC新样本校准和新故障防误清。保留的是运行中状态，不是增加断电存储；重新上电仍加载默认模型。

### 18.2 积分限幅与Gain结论

| 项目 | 旧浮点 | 当前定点 | 本轮处理 |
|---|---|---|---|
| 活跃公共PLL积分 | 默认±700电角rad/s | 默认约±700电角rad/s，内部0.7Q24 | 保持；额外测试条件积分、双向积分限幅、输出限幅、释放与Reset |
| 公共PLL输出 | 默认±700电角rad/s | 默认约±700电角rad/s | 保持；与积分限幅为两个独立限制 |
| LESO电流估计 | 数值限幅±1e25 | 约±3840A，INT32_MAX/2^24×30A | 不同，本轮未改；不代表允许电机电流 |
| LESO扰动积分 | 数值限幅±1e25，原f1单位A/s | 约±1.92e7A/s，内部f1×Ts/30A | 不同，本轮未改；不能误称为PLL积分限幅 |
| Leso_Gain | 默认8、Factor1.9 | 默认8、Factor1.9 | 不变，非IQmath库自动配置 |

原公共`Pid_Update`和定点活跃PLL均以P+I决定是否积分，再夹紧积分，最后夹紧P+I+D输出。局部HFI/LESO PLL在原实际链中已不执行。带宽仍按`Wc=clamp(Gain×1.9×估计rpm×2π×2/60,600,4000)`；Gain8时100/300/1000/1800rpm对应约600/955/3183/4000rad/s，负速按旧有符号规则取600。

### 18.3 验证项目与结果

| 检查 | 预期/覆盖 | 结果 |
|---|---|---|
| 公共PLL边界对照 | 10组向量直接对照真实旧Pid_Update；±700及修改为±250积分上限、条件冻结、回退、Reset | 独立编译运行PASS，容差0.0002rad/s |
| 冷/热INIT真实gateway | 毒化存储后冷启动；真实非默认MTPA表、多次热INIT、历史状态保持、非法写回退 | PASS；将热API替换为冷API的内存负对照按预期case33失败 |
| 热INIT后续周期 | 连续IDLE保留结果/模型，历史DONE/FAILED不强制退出后续SPEED | 独立实际gateway/core诊断及固化回归均PASS；DONE首次SPEED调用直接reset=false |
| 主循环实际INIT | 使用热API；建表中INIT不提交；正常提交、停止/故障时序原回归 | PASS；完整20项宿主回归重新编译运行通过 |
| FLOAT_REF构建 | 未改原浮点执行文件，独立hard-float编译 | PASS，64单元；Flash45464B/RAM20520B |
| IQMATH/A2L/对象审计 | 同源softfp构建和配套符号、无浮点核心、初始化栈帧 | PASS；68单元、14算法对象审计通过；配套A2L已生成，具体旧镜像限制见18.4 |

独立真实MTPA诊断先构建非默认ad0=9模型，热INIT前后accepted状态逐字节一致，动态电感和DONE/采样计数/镜像结果保持；候选事务取消，5次IDLE后直接进入SPEED成功；无效accepted表不被默认替代。该测试与宿主回归仍非实际TI库/目标硬件测量。

### 18.4 未在本轮修复的审计问题

- Gain约大于67.37时`Gain×1.9`中间Q24量饱和，网关却仍接受Gain<128；Gain100、50rpm复现Wc1989.675→1340.408。默认8没有此问题，但不能外推整个调参范围。
- 某些边界模型MTPA搜索发生饱和后仍`table_valid=1`，速度环也未检查每次lookup.valid。模型有效性检查仍不充分；建表失败继续保留旧表不是新模型已经成功应用。
- LESO限幅和部分仅发布的调试镜像写入行为仍不等同原浮点；MTPA优化停止条件、当前周期Reset等既有差异未撤销。
- A2L额外抽查发现既有导出限制：`Leso_Int_limit`无ELF保留符号、生成地址0，不能作为有效观察/调参入口；`Mtpa_TableValid`虽在ELF中，但被`A2LFilter.py`的`.*Table.*`规则过滤。可观察核心`MainInt_ControlState.foc.mtpa.table_valid`，不可把`table_valid=1`当作所有数值检查通过。本轮没有扩展修改导出脚本或LESO镜像。

本轮只恢复所要求的INIT状态/模型保留，没有因此宣称整体算法和原浮点逐点一致，也没有据宿主测试承诺目标电机闭环安全。

### 18.5 最终构建交付

| 项目 | 实际结果 |
|---|---|
| IQMATH资源 | Flash96,552B，RAM38,072B；Cortex-M4 softfp，68源文件均不晚于对象 |
| ELF SHA-256 | D83027D8C028D0EF96F97A4430803211FDC55E103F7DE116A2CC6F54437EBBD1 |
| HEX SHA-256 | 8934907497CEDBA8236A554751D042C0557347F490F897847A2FBBB9C143FCE8 |
| A2L SHA-256 | 13ACADA338DB3FE1F8E2B1EBF7CF8292F2E40996CFD39E7D8813122A3E0BCB68 |
| A2L有效地址抽查 | Sensorless_Enabled、Sensorless_ThetAdj、MainInt_UseRealTheta、Sensorless_Theta_PID.IntegralLimit、Leso_Gain、Mtpa_RebuildFailed、MainInt_State均有有效地址 |
| 单函数静态栈帧 | FocIq_Reinitialize56B、FixedControl_Reinitialize96B、SensorlessGateway_Reinitialize256B、MainInt_ControlBackground40B；不是含调用链/中断嵌套的峰值栈 |
| 其它检查 | 新热接口存在于最终ELF，无未解析符号；git diff --check通过；参考浮点相关文件对22e6d80无修改 |

产物位于`build-iqmath-softfp/GD32F30xProg1230.{elf,hex,a2l}`，本轮未烧写、未提交/push、未删除源码或修改Git历史。烧录时使用这次ELF及配套A2L，旧观察地址不能混用。完整20项宿主回归与上述资源/栈检查不等于目标板栅极、最坏中断耗时或真实电机闭环测试。

## 19. 正反转增益对称修正（2026-10-12）

### 19.1 请求、实现与明确保留项

用户明确要求将反向增益修复为与正转一致。本轮只修改 IQMATH 实际执行路径中的两处调度：

- `LesoIq_Run` 使用 `McMath_Abs(input->speed_pu)` 计算 Wc；Gain、Factor、Wc上下限、离散系数公式及更新顺序不变。
- `SensorlessIq_UpdatePosition` 使用 `McMath_Abs(state->speed_pu)` 判断500rpm阈值并计算超过阈值的增益增量；原正向调度公式、上限及下一拍生效时序不变。
- 原始估计速度/电角速度、角度积分方向、反电势 `E=-Lq*f1` 和负速鉴相误差翻转仍使用有符号量，不能把整个速度状态改成绝对值。
- 未改HFI注入、观测器选择/启停阈值、反馈来源、保护/Stop/PWM/硬件导通逻辑、控制频率、A2L变量名或单位。
- 旧 `leso.c` 和 `sensorless_interface.c` 未改，FLOAT_REF保持原实验参考；因此本轮负向增益是授权的行为修正，不再宣称该处与原浮点逐点一致。

默认Gain8、Factor1.9、极对数2、Wc限值600/4000时的预期物理值（允许Q24量化误差）：

| 估计转速rpm | LESO Wc rad/s | 公共PLL Kp | 公共PLL Ki |
|---|---:|---:|---:|
| 0 / ±100 | 600 | 50 | 625 |
| ±500 | 约1592 | 50 | 625 |
| ±600 | 约1910 | 90 | 2025 |
| ±700 | 约2228 | 130 | 4225 |
| ±1800 | 4000 | 130 | 4225 |

### 19.2 验证与交付

执行 `Tools/RunFixedHostTests.ps1` 完整20项宿主回归通过。扩展 `Tests/sensorless_legacy_parity_host_test.c`，保留原20000拍公共PLL、6000拍LESO、HFI、策略和双向积分限幅测试，未删除负向轨迹或放宽原容差。正向仍直接对照保留的原浮点源码；负向仅在测试参考适配器中显式修正调度为绝对值，实际浮点生产文件不改。

| 测试 | 步骤/预期 | 实际结果 |
|---|---|---|
| 同幅值增益 | 8对输入：0、±100、±500阈值两侧1个Q24最低位/等值、±600、±1800、±4000rpm | LESO Wc、PLL Kp/Ki定点原始值逐位一致；低档/高档上限与物理期望一致 |
| 方向隔离 | 相同观测器状态、电压、电流，仅速度改符号 | EMF及带宽不变，非零速度的鉴相误差正确反号，速度本体保持负值 |
| 连续跨零 | 12步持久状态正/负镜像轨迹，包括降速、过零、加速和再反向 | 记忆连续，双向增益相等；PLL当前输出仍用上一拍增益，不提前调度 |
| 公共PLL数值 | 原浮点加明确的负向调度修正参考 | 最大角差0.005355°，转速差0.016602rpm，角速度差0.003937rad/s；PASS |
| LESO数值 | 原浮点加明确的负向带宽修正参考 | 最大电流差0.000021A，EMF差0.000363V，归一化鉴相差0.000180；PASS |
| HFI数值 | 未改原参考与定点实现 | 最大电流差0.000003A、鉴相差0.000019；PASS |
| 回退负对照 | 分别仅在内存编译中撤回LESO abs或PLL abs，不改实际源文件 | 两者分别触发Wc/Kp断言失败，证明回归可发现任一修复丢失 |
| 其它回归 | 保护/Stop抢占、FOC/INIT、网关、辨识、MTPA、串口滤波 | 完整20项通过；无新增除零/无效输入 |

宿主计算使用Q24数学mock，不是实际TI库运行或目标电机动态验证。相同速度幅值的增益一致，不代表原来正向参数对所有反向工况都已验证稳定。

| 构建/交付 | 实际结果 |
|---|---|
| 软件版本 | 分支`测试2025-12-26`、HEAD `dd50a5c` 加当前工作区修改（未提交） |
| IQMATH | Cortex-M4 softfp，68单元，Flash96,616B、RAM38,072B；相对第18节Flash增加64B、RAM不变 |
| FLOAT_REF | 原参考文件未改，hard-float重新构建通过，64单元、Flash45,464B、RAM20,520B |
| 固件检查 | 14算法对象无浮点算术/运行库调用；68对象均不早于源文件，ELF无未解析符号 |
| A2L | 从本次ELF重新生成；Leso_Gain/Wc/We、Sensorless_SpeedEst、Sensorless_Theta_PID.Kp/Ki均为有效地址的物理浮点镜像 |
| ELF SHA-256 | EBBA276A7141F873C735561689ED250C46B559283309A4004321EB622C817DBA |
| HEX SHA-256 | A2D538CA9EEE4FF894CAC647CFFA2386292E08C2A6CE1A92E0BADA5B5EA6B45F |
| A2L SHA-256 | BA6C890C3E49A84EE190E99CB098416C254FE1503605F768E95DB2050E33CDFE |

产物仍为 `build-iqmath-softfp/GD32F30xProg1230.{elf,hex,a2l}`，VS Code Build任务不需更改。本轮没有烧录板卡、提交/push或更改Git历史；使用新固件时加载这次配套A2L。

### 19.3 未包含在本次修复内的已知问题

- 估计速度为10拍均值及10Hz二阶滤波后的速度，过零可能滞后；反电势鉴相符号仍依赖其方向，本轮不改方向来源。
- 观测器选择和启停条件不完全一致。已用旧/新真实源码复现：目标-600rpm、斜坡+600rpm、反馈±100rpm、原HFI关/LESO开时，公共PLL选择HFI但HFI仍关，使用旧误差。此原有边界未在本次增益修正中改动。
- 第18.4节的大Gain中间饱和、MTPA边界及调试导出限制继续保留；这次对称调度不扩大已验证调参范围。
- 公共PLL低速分支只恢复Kp/Ki，`Sensorless_PLL_Wc`镜像保留历史值，与原正向行为相同；应以实际Kp/Ki判断有效增益。

本轮不承诺目标板闭环稳定性；提高反向带宽到正向水平不等于消除全部反转风险。

## 20. 辨识等价修复、MTPA回退及Wc硬上限（2026-10-12）

### 20.1 授权边界与原浮点INIT回答

用户要求恢复Rs预测电流升压条件、修复扫描漏终点、MTPA无效或超表范围时采用线性分配，并限定Wc≤3990；明确确认反转也采用`Id=|Iq|`，Iq保留方向。此次不修改原FLOAT_REF源码、硬件PWM/保护/INIT链、无位置误差选择策略或显示滤波。

原浮点`MainInt_Initialization`在ADC主中断内调用，2000次紧循环校准没有失败返回分支，也没有关闭ADC NVIC；正常返回后后续ADC中断继续执行采样、电流/母线软件保护和控制。当前IQ后台INIT则先关闭ADC IRQ，算法检查或128组新样本校准失败后保持关闭。这是新的失败路径，不能称与原浮点完全一致。

前一轮已复现：校准失败Status=2、MainInt_State=EXIT、ADC IRQ关闭，但算法Ready仍为1；CAN改回RUNNING/IF、清Flag并Stop=0，PE15正常时可恢复MOE而ADC IRQ仍关闭。本轮用户只询问原浮点行为，未授权改此链，故该门控缺口仍未修复。即使本节四项测试通过，也不能作为所有带电实验放行结论。

### 20.2 辨识数值与操作契约

- `Experiment.start_I/final_I/step_dir`继续为原整数安培输入。网关同时传入显式整数扫描配置；步点在整数域推进、比较，再逐点转为Q24，不用独立量化的起点/终点/步长累加决定是否存在末点。
- 默认3→12/+1以及12→3/−1各10点，3→22/+1受20点容量限制为20点；3→12/+2为3/5/7/9/11，不强行增加不可整步到达的12。不是通过放宽任意epsilon吞入越界点。
- 通用非整数Q24调用仍按其实际代表值严格比较，不猜测上层原物理整数；`integer_current_scan`明确区分两种契约，原A2L默认启用整数模式。
- Rs以原`Estimate_Rs`为基准恢复预测电流判断：未收敛时仅`|I_last+ΔV/Rs_new|<8 A`才更新上一点并加压；保持原15000计数后第15001拍处理、收敛阈值和估计步数的原判断顺序。
- 仍不把Rs超流/超步数当作成功辨识：原函数done_flag=2/4也返回true，定点仍FAILED并请求停机；显式状态、除零/无效数值保护保留。DQ按完整周期计repeat_count的现有行为未修改，不宣称所有辨识细节逐行等价。

### 20.3 MTPA与Wc变更契约

- 无效模型/表、无效插值区间或`|Iq|`超出有效表范围不再沿旧末区间外推，改为`Id=|Iq|`；q轴速度PI输出方向与原限幅保持。
- 建表的饱和、除零、非法数值/点或无解通过调用栈内独立错误位跟踪，不依赖共享诊断计数前后差值。失败候选撤销活动表有效性，旧表数据保留供追溯；成功候选仍在IDLE后台原提交点发布。候选错误可观察`MainInt_ControlState.foc.mtpa_staging.build_errors`：1非法输入、2饱和、4除零、8无解、16无效点（位标志可组合）。
- 回退电感使用`FocIqConfig.mtpa_fallback_inductance_dq_pu`，默认标称Ld=0.1 H、Lq=0.016 H；非法非正配置回到该默认，不能将回退L称为已辨识参数。新增只读网关镜像`Mtpa_LinearFallback`，每控制周期发布，SPEED回退为1、离开SPEED/Reset/启动准备为0；写镜像不改变算法。
- 当前冷启动/热INIT就绪条件仍要求有效表，运行期回退不等于撤销此条件。表无效后执行INIT仍会拒绝就绪；本轮不修改硬件初始化门控。
- 线性回退意味着dq电流幅值为`√2×|Iq|`，例如Iq=24 A时约33.94 A。本轮不另加总电流限幅，不修改原保护阈值；回退不是允许扩大实际电流的证明。
- 默认LESO下/上限由600/4000改为600/3990 rad/s。在线Min/Max必须有限且0<Min≤Max≤3990；非法整对回滚到最后接受值并计诊断，不把写入4200视作合法再执行。
- 核心另有Wc×Ts硬上限，IQ24向下量化，避免直接配置或转换舍入超过3990。反向abs调度、鉴相符号、PLL参数及控制周期均保持；原FLOAT_REF保留4000，因此这项是明确授权差异。

### 20.4 回归与构建记录

`Tools/RunFixedHostTests.ps1`在最终源码上重新编译执行完整20项，全部PASS。原FLOAT_REF执行文件对`22e6d80`无修改；未删除旧测试或放宽原MTPA/LESO/PLL对照容差。合成辨识fixture为适应恢复的预测升压条件，Rs测试保持1000拍、目标0.25 pu，生产默认仍15000拍/8 A/1 V；该调整已在测试注释中明示。

| 检查 | 实际步骤与结果 |
|---|---|
| Rs真实原函数对照 | 原`Estimate_Rs`与IQ运行150010拍相同电流输入，逐拍核对电压、结束时刻；8.2 A未收敛平台两版均保持约6 V；PASS |
| Rs边界/异常 | 超20步、超流FAILED且零输出；微小ΔI重建基线；Q24 raw55/56边界；PASS。阈值改显式整数ceil，避免mock四舍五入与TI常量截断差异 |
| 扫描与人工放行 | 正反10点、正反20点、step±2非整除、max_steps截断、零步/方向/整数极值、失败零输出；另真实FixedControl_Step网关5组扫描确认旧Experiment输入生效；PASS |
| 完整合成辨识 | FLOAT与Q24都在7318周期完成，五系数最差相对差约0.143%，低于1%；原Q48拟合测试PASS |
| MTPA正常表 | 旧优化器对照容差不变：Ld最大相对差0.215194%、Lq 0.0297442%；默认数值测试PASS |
| MTPA拒绝/回退 | 正反有效端点及外侧1LSB、无效区间/L、饱和/无解候选、成功重建恢复；真实FOC保留原±24 A q限幅、Id=abs(Iq)、配置非零电感；PASS |
| MTPA诊断隔离 | 独立诊断在真实建表的每次CosPu模拟无关IRQ全局计数更新，各累计143144次；默认表仍有效、本地build_errors=0；PASS |
| 回退A2L镜像 | 真实网关发布1；写镜像0不影响算法，下一拍恢复1；退出SPEED为0；PASS |
| Wc上下限 | 14组在线写入/回滚、12组正反直接配置和200拍零输入衰减；上限raw13388218为3990×Ts的精确向下量化；PASS |
| 未改路径回归 | 保护/Stop抢占、公共PLL/HFI/LESO、热INIT保留、显示滤波隔离等原测试全部PASS；不代表已覆盖20.1失败恢复缺口 |

| 构建/交付 | 实际结果 |
|---|---|
| 软件版本 | 分支`测试2025-12-26`，HEAD `dd50a5c`加工作区修改，未提交 |
| IQMATH | Cortex-M4 softfp，68单元，Flash100,280 B、RAM38,120 B；较第19节增加3664 B/48 B |
| FLOAT_REF | hard-float重新构建通过，64单元，Flash45,464 B、RAM20,520 B |
| 对象与链接 | 14个算法对象无浮点算术/运行库调用；最终ELF无未解析符号；最终构建无新增编译警告 |
| 单函数静态栈帧 | FocIq_Run 240 B、MtpaIq_BuildTable 184 B、identification_run_rs 32 B；不是调用链/嵌套中断峰值实测 |
| A2L | 同ELF重新生成；Mtpa_LinearFallback有效地址0x20004848，Leso_Wc_Max仍FLOAT32物理值；Experiment.start_I/step_dir仍SLONG旧名 |
| ELF SHA-256 | B0489F7D0C93EABA06C67ECE1DD8C622E715CA8DBC0F75E600C1E7824A80276F |
| HEX SHA-256 | 88787C3F4AC65B3BAFFBD6423FF9B891E67D4544D8488571263F0DD44F0E502C |
| A2L SHA-256 | DE29C025EC30FF0433E76A0F7497887E41006DD5305E46946227DEF2A13050CB |

产物仍位于`build-iqmath-softfp/GD32F30xProg1230.{elf,hex,a2l}`。本轮未烧录、未提交/push、未删除用户文件或Git远程历史；原有工作区修改保留。加载新固件须配套新A2L，不能混用旧地址。

### 20.5 仍未修复/未验证事项

- 20.1所列运行中INIT失败门控缺口未修复；本轮明确没有授权修改硬件启停链，不能强行改状态绕过失败。
- 旧无位置误差来源选择与观测器启停条件不一致、反馈选择与Sensorless_Enabled独立、HFI使能时即使用编码器仍注入等旧行为保持。
- 大Gain中间乘积饱和、LESO积分可表示范围、DQ完整周期重复方式、参考与当前Reset一拍时序差异等此前已记录限制未因本轮四项修复消失。
- 主机Q24数学mock不等同实际TI库/ADC/PWM/电机；目标板最坏中断耗时、峰值栈、闭环全工况不在本轮实际测试结果内。

## 21. 运行中INIT恢复ADC与失败门控（2026-10-12）

### 21.1 请求与实现

用户要求定点保持原浮点“INIT结束后继续ADC控制和软件保护”的行为。原浮点在ADC中断内执行INIT，没有关ADC NVIC或校准失败返回路径；本轮对齐结束后的中断连续性，不将校准失败伪装成成功，也不更改128组新样本的校准方法。

- 运行中INIT仍在后台以Stop=1、IDLE、MOE关闭、暂时禁止ADC IRQ进行，CAN/硬件故障中断仍可运行。
- 所有退出路径统一清EOIC/ADC NVIC挂起位、以原3/0优先级恢复ADC IRQ使能。初始化间隔不参与DWT采样周期估计。
- 成功：Status=3、MainInt_State=RUNNING，继续控制/软件保护；保持Stop=1、IDLE，不自动启机。
- 失败：Status=1/2、EXIT、Stop=1、IDLE；后续ADC采样中断仍运行相电流/母线检查、位置和观测发送，但跳过未就绪算法，输出中性占空比参考且主输出禁止。若硬件ADC本身不产生新转换，恢复NVIC使能不能保证实际采样恢复，输出仍被锁住。
- 集成层新增独立`MainInt_InitOutputBlocked`门控。实际`Peripheral_Update_Break`在原短临界区内检查它，防止Stop/Flag/模式/公开Status写入绕过初始化失败；成功INIT重试才恢复资格。该内部观察值不能作为手动控制输入。
- 原Stop/PE15/BKIN、PWM极性/死区、保护阈值、故障epoch防误清、Break单次触发均保持。原浮点生产路径和首次启动失败流程不变。

### 21.2 验证步骤与结果

| 项目 | 覆盖与预期 | 结果 |
|---|---|---|
| 初始化失败ADC恢复 | 算法未初始化、表无效、自检失败、校准失败四类；均重新启用ADC，清旧pending，输出禁止 | 针对性测试PASS |
| 失败后的软件保护 | 用真实Main_Int_Handler，模拟新的相电流/母线故障；采样保护调用继续，错误核心不执行 | 针对性测试PASS（外设/保护边界为mock） |
| 普通CAN命令不能绕过 | 失败后写RUNNING、IF、Flag=0、Stop=0、Status=3，门控仍阻止PWM | 集成边界及真实hardware_interface测试PASS |
| 成功重试 | 不清私有门控、不重建测试实例，排除失败后INIT成功；保持停机，后续显式请求运行恢复控制 | 针对性测试PASS |
| 中断/输出边界 | 保留原PRIMASK；实际硬件恢复函数在mask=0/1下均强制关闭残留MOE；允许条件恢复后原PE15规则有效 | 针对性测试PASS |

最终重新执行`Tools/RunFixedHostTests.ps1`完整20项全部PASS，包含未改变的保护、无位置、MTPA、辨识、显示滤波隔离及上述新增INIT失败恢复用例。真实目标ELF反汇编确认：`Peripheral_Update_Break`在CPSID后调用`MainInt_InitBlocksOutput`，阻断时进入关MOE分支；通过后才按原PE15条件恢复输出，保留调用方PRIMASK。

### 21.3 构建交付与保留边界

| 项目 | 实际结果 |
|---|---|
| 软件版本 | 分支`测试2025-12-26`、HEAD `dd50a5c`加工作区修改，未提交 |
| IQMATH | Cortex-M4 softfp，68单元；Flash100,588 B、RAM38,128 B；相对第20节增加308 B/8 B |
| FLOAT_REF | hard-float构建通过，64单元；Flash45,464 B、RAM20,520 B不变 |
| 代码审计 | 14定点算法对象无浮点算术/运行库调用；ELF无未解析符号；68个源均不晚于对应对象；git diff --check通过 |
| 原硬件/浮点对照 | main_int.c、TIM0配置及GPIO对22e6d80无修改；共享hardware_interface新增条件仅编入IQMATH分支 |
| 单函数静态栈 | Main_Int_Handler176 B，MainInt_ControlBackground48 B；非含调用链/嵌套中断的峰值实测 |
| A2L | 同ELF重新生成；MainInt_InitOutputBlocked在0x20000000（合法RAM地址）、FixedStartup_Status在0x2000824A、MainInt_State枚举保留 |
| ELF SHA-256 | 1F86630BE62CB8D9879B4157586F7222E41FAA4D69BB55651E137142430BEACA |
| HEX SHA-256 | 95613F6BA1223234FF14A96CF4FF7E46C17C36B9DD1CA0BA1F952BDC34D43734 |
| A2L SHA-256 | 2D0537CBD06EFB709F1C5A3CA1559A426047736EA230DA62AC88E40729D50E59 |

本轮已修复第20.1节指出的运行中INIT失败后输出恢复缺口，不再保留“ADC被软件一直关闭但PWM可恢复”的路径。没有修改其它已记录的无位置策略边界、大Gain数值限制、校准方法或首次启动失败流程，也不声称整个INIT实现与旧浮点逐条相同。新的内部门控可观察但不是用户控制入口；任意改写内部RAM不属于支持的调试操作。

产物仍为`build-iqmath-softfp/GD32F30xProg1230.{elf,hex,a2l}`；未烧录、未提交/push、未删除用户文件或Git历史。上述宿主检查不构成真实ADC、栅极波形或电机实验保证。

## 22. 独立 DQ 联合注入电流（2026-10-13）

按用户要求新增上位机可写物理变量 `Identification_DqCurrent_A`，上电默认 9 A（float）；网关转换为 `IdentificationIqConfig_t.dq_current_pu`。d/q 两轴分别使用 ±该值作为辨识方波换向阈值，不代表合成电流幅值。D/Q 单轴的 start_I/final_I/step_dir、Rs、HFI、保护、硬件启停和 FLOAT_REF 源码均未修改。

生效规则：在 DQ 首次注入前，包括已经进入 INJECT_COLLECT 但 inj.State=0 的等待期，每周期刷新目标和 Imax/IDmax/IQmax 镜像。首次放行时锁定整组 repeat_times 采集的电流；暂停、恢复和自动重复不解锁。运行中接受的新配置留待下一轮完整辨识使用，避免混合不同电流的样本。完整 Reset 清锁定；INIT/重新进入 IDENTIFY 保留设定，掉电恢复默认 9 A，无自动 Flash 保存。

网关拒绝非有限、非正、超过 Q24 表示范围或量化成 0 的值，回填上次接受值并增加 invalid_input_count。数值范围校验不是硬件额定电流校验，原保护阈值保持不变。

| 检查/步骤 | 预期 | 实际结果 |
|---|---|---|
| 核心默认、手动/自动 DQ 进入 | 目标 9 A，独立于单轴扫描 | FLOAT/Q24 宿主测试通过 |
| DQ 首次等待中写 9.5 / 10 A | 下个 Step 更新目标及 Imax/IDmax/IQmax | 实际网关宿主测试通过 |
| 首次注入后写另一电流，暂停/恢复及自动重复 | 本轮阈值不变，下轮读取新配置 | 两数值后端核心及网关测试通过 |
| 写 0、−1、NaN、±Inf、3840 A、1e30 A、1e−20 A | 拒绝、回填上次接受值、诊断计数增加 | 网关测试通过 |
| INIT / 完整 Reset / 重新进入 / 冷启动 | INIT 保留设定；完整 Reset/重新进入清批次锁定；冷启动回 9 A | 核心和网关测试通过 |
| 原 D/Q 双向扫描、旧辨识波形对照 | 单轴端点/步进不变 | 全套宿主测试通过；旧 DQ 对照夹具显式固定原电流，未更换黄金输入 |
| `Tools/RunFixedHostTests.ps1` | 重新编译并运行全部测试 | 全部 20 项通过，含新增用例及保护/无位置/显示隔离回归 |
| `Tools/BuildSequential.ps1 -Backend IQMATH` | 生成 ELF/HEX，核心无浮点指令 | 68 个源文件编译通过，14 个算法对象审计通过 |
| `Tools/GenerateA2L.ps1` | 新变量与同次 ELF 地址一致 | FLOAT32_IEEE，地址 0x2000000C；ELF .data 初值 0x41100000 = 9.0F |
| Flash / 静态 RAM | 记录资源 | 100812 B / 38144 B；栈峰值及中断耗时本轮未测 |

构建为 IQMATH / Debug / Cortex-M4 softfp，基于分支 `测试2025-12-26`、提交 `08cd043` 加本轮未提交修改；目标编译器 Arm GNU 14.2.1，宿主 GCC 15.1.0。新变量和结构体字段使部分旧变量地址改变，烧录后必须重新加载本次 A2L；不能沿用旧地址。上述宿主测试使用 IQmath mock，不等同于目标板电机实测。本轮未操作硬件、未烧录、未提交/push；用户已有 GD32F30x.snw 修改保持不动。

| 产物 | SHA-256 |
|---|---|
| ELF | 27AEE1CCAC45E33A96848606145F38E80D50BC46423B7778DDEA90ABDF5D87FF |
| HEX | 1D9980E9CA5EE43DB60188F34FD7ABE930FA2E7DC247F6B4FB19089CFE685379 |
| A2L | 249F6112A316C5060F6AE8E4DD838AF6EDCAA3474ADBA411473E817932F78F43 |
