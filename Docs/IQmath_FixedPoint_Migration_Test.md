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
| SW-02 | 编译全部 63 个 IQMATH 翻译单元 | 无编译错误 | 全部通过 | 通过 |
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

当前 IQMATH 固件资源：Flash 76,024 B（29.00%），RAM 35,088 B（71.39%）。FLOAT_REF 固件资源：Flash 45,464 B（17.34%），RAM 20,520 B（41.75%）。RAM 已包含链接脚本预留的 1 KiB heap 和 2 KiB stack，也包含固定长度辨识采样缓冲区、后台 MTPA 暂存表及旧 A2L 浮点镜像；IQMATH 辨识累计使用 64 位整数。性能只记录，不设提速通过门槛。

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

一键运行五组主机回归：

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

1. 加载本次 IQMATH ELF 生成的 A2L，保持 `Foc_Mode = IDLE`、`Foc_Reset = true`。
2. 确认 `FixedControl_SelfTest` 四项均为 0；记录 `McMath_Diagnostics` 初值。
3. 核对电流零点、母线电压、实测角度和速度方向；PWM 应保持 50% 安全占空比。
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
