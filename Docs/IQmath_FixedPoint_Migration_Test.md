# IQmath 全定点迁移与测试记录

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

| 信号类别 | 物理基值 | 内部范围 | 格式 | 边界行为 |
|---|---:|---:|---|---|
| 相电流、dq 电流 | 30 A | -1.0 至 1.0 pu | Q24 | ADC 物理值进入中断时转换 |
| dq/alpha-beta 电压 | 800 V | -1.0 至 1.0 pu | Q24 | PWM/A2L 输出时转换 |
| 机械速度 | 1800 rpm | -1.0 至 1.0 pu | Q24 | 位置接口和 A2L 边界转换 |
| 电角度 | 1 转 | 0 至 1.0 pu | Q24 | `1 pu = 2*pi`，使用 IQmath PU 三角函数 |
| PWM 占空比 | 1.0 | 0 至 1.0 | Q24 | 输出前限幅并转换为 float 边界结构 |
| PI 积分与输出 | 按对应电流/电压基值 | 按配置上下限 | Q24 | 增益写入时转换为离散 Q24 系数 |
| 辨识累计量 | 对应 Q24 乘积 | 64 位范围 | `int64_t` | 512 点后缩放回 Q24，检查除零和缩窄范围；FLOAT_REF 对应使用 double 参考累加器 |

定点数学统一由 `mc_math.h` 调用 IQmath。加减采用饱和；乘法使用 `_IQ24rmpy` 完成带舍入和饱和的 Q48 到 Q24 缩窄；除法、sinPU、cosPU、atan2PU 和 sqrt 使用随附 IQmath 库。浮点边界转 Q24 采用就近舍入并在转换前检查范围；除零、无效输入和饱和分别累计诊断计数。

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
