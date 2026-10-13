# IQMATH 各模式操作、参数与观测变量

核对日期：2026-10-12。本文面向本轮 IQMATH 工作树修改后的固件，说明代码里真正执行的操作；不是目标板实测通过或功率级安全证明。

最新修订：运行中INIT成功/失败退出均恢复ADC中断使能，正常成功后继续控制和软件保护；失败时保留采样保护路径并禁止PWM，不能用普通模式/Stop/Flag写入绕过。前一轮Rs升压、整安培扫描、MTPA线性回退、Wc≤3990继续有效。具体测试和剩余限制见迁移记录第21节；不将局部回归当作全部带电工况的安全证明。

源码修改后先 Build，再 Flash；上位机加载同一 ELF 生成的 `build-iqmath-softfp/GD32F30xProg1230.a2l`。A2L 按钮只从已有 ELF 生成地址，不会先编译。旧的 [辨识与弱电指南](IQmath_Identification_Operation_Guide.md) 保留历史记录；涉及停止复位、无传感使能或新恢复接口时，以本文和对应固件为准。

本文的 A/V/rpm/rad 参数仍在上位机按物理 float 输入。每周期网关取快照并转换为内核定点量，内核结果再转换为物理观察值；不要求上位机输入 Q24 整数。项目基值是电流 30 A、电压 800 V、速度 1800 rpm、磁链 1 Wb、转矩 50 Nm，不是 IQmath 库强制统一的常量。例如 3 A→0.1 pu→Q24 raw 约 1677722；100 V→0.125 pu→raw 2097152。

## 1. 先分清：选择模式、允许运行、观察估计

| 变量 | 方向 | 实际作用 |
|---|---|---|
| `Foc_Mode` | 输入，算法也会回写 | 选择下面 6 个模式；异常值回 IDLE；辨识完成/失败会请求 IDLE |
| `Stop` | 输入及保护输出 | `1` 停机；`0` 只请求恢复，仍受保护锁存和实际硬件输入限制 |
| `Foc_Reset` | 实际状态输出 | 控制周期根据 Stop/IDLE 等状态更新；不是独立启动按钮，不用手动写 0 来开机 |
| `MainInt_State` | 输入及状态输出 | 主中断层运行状态，正常运行设 RUNNING；不是 `Foc_Mode` 的替代品 |
| `MainInt_UseRealTheta` | 输入 | `1` 使用位置传感器角度/转速；`0` 请求使用统一无传感估计。VF/IF 的开环相位还有本模式单独规则 |
| `Sensorless_Enabled` | 输入 | 无传感统一 PLL、HFI 电流处理和电压注入的总使能；只是选择 Method 不能代替它。局部 Calculate 的门控见第 7 节 |
| `Sensorless_Method` | 输入及策略输出 | 原枚举 `FLYING=0`、`LES_OBSERVER=1`、`HF_INJECTION=2`；策略运行时可能自动改写，不再按旧定点位标志 1/2/4 操作 |
| `Sensorless_UsingHfi` | 输出 | 当前策略采用 HFI 路径，不能当使能输入 |
| `Sensorless_ThetaEst / Sensorless_SpeedEst` | 输出 | PLL 本体估计角 rad、估计转速 rpm；ThetaEst 不含 ThetAdj。有估计值不表示 FOC 已切到它做反馈 |
| `Foc_Theta / Foc_Speed_Fdbk` | 输出 | FOC 实际采用的控制角度和反馈转速；判断闭环反馈来源时同时看这两个 |

只想把估计和真实位置放一起观察：保持 `MainInt_UseRealTheta=1`，按第 7 节使能无传感算法。它仍可能注入 HFI 电压/处理电流，不能把“只观察”理解成完全不影响电机激励。要真正无位置闭环，还要将 `MainInt_UseRealTheta` 设为 0，并确认对应模式没有采用 VF/IF 自身的开环相位。

## 2. 上电初始化与启动顺序

### 2.1 自动完成的初始化

上电程序自动初始化保护、电机位置/速度、Buffer、定点控制状态和 MTPA 表，再初始化硬件、ADC 零偏、CAN/CCP。CCP 初始化在开放 ADC 控制中断前完成。

| 观察变量 | 含义 |
|---|---|
| `FixedStartup_Status` | `0` 未完成；`1` 算法初始化失败；`2` ADC 校准失败；`3` 初始化完成。不手动写该值来推进启动 |
| `MainInt_InitOutputBlocked` | 内部初始化输出门控的观察值，勿写。运行中INIT失败保持1；成功重试才清0。它不是手动启动命令，也不代表PE15/Flag等其它条件已经满足 |
| `FixedControl_SelfTest.*_failures` | 转换、乘除、三角函数、开方自检计数，正常为 0 |
| `Mtpa_TableValid` | 启动自动生成的 MTPA 表是否有效；无需手动初始化 |
| `Motor_Initialized` | 位置/转速处理已初始化，观察值 |
| `MainInt_FrequencyHz` | 实际中断计时推算频率，配置目标约 5000 Hz |
| `Foc_Current_Ts / Foc_Current_Freq / Foc_Speed_Ts / Foc_Speed_Freq` | 初始化时基观察约 0.0002 s / 5000 Hz / 0.002 s / 500 Hz；写这些镜像不会修改硬件定时器 |
| `MainInt_CycleTicks / MaxCycleTicks / PeriodTicks / LoadPermille` | 当前/最大执行时间、入口间隔、负载千分比；断点暂停的数据不能作周期性能结论 |
| `McMath_Diagnostics.*_count` | 饱和、除零和无效输入诊断；记录增量，不靠写零隐藏持续异常 |

MTPA 先在初始化阶段建表。辨识成功后只在 IDLE 主循环后台重建并提交，随后更新旧名 `mtpa_table[]` 物理镜像；完整表转换不放进 ADC 中断。

### 2.2 每次正常操作

1. 先设 `Stop=1`、`Foc_Mode=IDLE`，确认真实主输出禁止状态和硬件条件。
2. 正常模式操作保持 `MainInt_State=RUNNING`；有传感基线使用 `Sensorless_Enabled=0` 且 `MainInt_UseRealTheta=1`，不是只关 Enabled。不要同时让 SENSORLESS 状态自动改模式，又手动选择 VF/IF/IDENTIFY。
3. 保持停机，选择目标 `Foc_Mode`，等待模式入口处理完成，然后写该模式的输入参数。IDENTIFY 尤其要先进入，再改 `Experiment.*`，否则进入时默认初始化会覆盖先前写入。
4. 核对真实供电、位置基准、采样和保护原因已符合原实验条件。`Stop=0` 后观察实际 `Foc_Reset=0`，确认没有立即重回 Stop。
5. 按本模式给定和状态执行。软件正常结束：`Stop=1`，再返回 IDLE；紧急情况使用原硬件急停，不依赖上位机通信。

仅弱电时母线接近 0 V，欠压保护和 Stop 保持属于预期，能检查通信、采样、枚举、镜像与参数转换，不能据此验证真实电机控制或辨识效果。不要为通信或推进状态屏蔽保护、伪造母线读数。

### 2.3 停止与 Reset 的含义

停止期间 PWM 参考保持中性占空比；参与当前模式的 PI/斜坡/相位按原模式自己的 Reset 分支处理，不主动清空 IDLE 下未执行的模块，也不把普通模式切换变成全部状态重置。具体：IF 复位参与的电流 PI；SPEED 复位速度/电流 PI、斜坡等；VF 复位共享相位计数，仍保留原 held phase/扫描规则；IDLE 不主动清这些未执行状态。辨识状态只在停止入口/模式变化等必要时复位，采样大数组保留旧内容，通过计数和 valid 标志使旧数据失效，新采样从第 0 点覆盖。

`Foc_Tcm.a/b/c=0.5` 不等于开关管全部关断；真正停机还要确认 TIMER0 主输出禁止和实际驱动引脚，本文不改变原硬件极性定义。

Stop 不是辨识“保留进度暂停”。停止入口会使 Rs、点进度、拟合有效性失效；再次正式辨识按 IDLE→IDENTIFY 从头准备。已经自动完成并回 IDLE 的浮点结果镜像会锁存，应先保存后重新进入 IDENTIFY。

## 3. 模式总表

| `Foc_Mode` | 值 | 算法做什么 | 主要输入 |
|---|---:|---|---|
| IDLE | 0 | 请求硬件 Stop、零基础电压；不主动清未执行模式的 PI/相位；允许后台 MTPA 重建 | 不作为带电输出模式 |
| VF_MODE | 1 | 给定 d/q 电压，使用相位发生器；不是电流闭环 | `Foc_VfParam.*`、`Foc_Sweep` |
| IF_MODE | 2 | 给定 d/q 电流，电流 PI 闭环；角度可用相位发生器或反馈来源 | `Foc_IfParam.*`、电流 PI、`Foc_Sweep` |
| SPEED | 3 | 启动准备→速度斜坡→速度 PI→MTPA 电流分配→电流 PI | `Foc_Speed_Ref`、斜坡、速度/电流 PI、反馈选择 |
| STARTUP | 4 | 固定 d 轴启动准备电压，q 轴基础给定为 0 | 编译参数 `SENSORLESS_STARTUP_UD`、反馈选择 |
| IDENTIFY | 5 | Rs→完整 D 扫描/拟合→完整 Q 扫描/拟合→单点 DQ/拟合 | `Experiment.*` 和每点人工放行 |

以上为模式基础输出；无传感/HFI 使能且允许运行时，可能附加 HFI d 轴电压。普通 VF/IF/辨识不需要它时应关闭 `Sensorless_Enabled`，不要只改 `Sensorless_Method`。

## 4. VF_MODE：电压/频率模式

### 4.1 真正生效的输入

| 变量 | 单位/默认 | 方向与作用 |
|---|---|---|
| `Foc_VfParam.vol_ref.d` | V，默认 0 | d 轴基础电压输入 |
| `Foc_VfParam.vol_ref.q` | V，默认 0 | q 轴基础电压输入 |
| `Foc_VfParam.freq` | 电角频率 Hz，默认 0 | 相位每周期步进 `freq × Ts` pu；不是 rpm |
| `Foc_VfParam.offset` | rad，默认 0 | 相位偏置；不是位置传感器零偏计数 |
| `Foc_Sweep` | bool，默认 1 | 开环相位扫描请求。置 0 不是立即 Stop，按相位扫描/回绕规则冻结参考相位 |
| `Foc_Vf_Vd / Vq / Frequency / Offset` | 与上述同单位 | 只显示结构体中实际输入，写这些平铺别名不会驱动控制 |

### 4.2 操作

1. Stop=1，MainInt_State=RUNNING，Sensorless_Enabled=0、MainInt_UseRealTheta=1，选择 VF_MODE。
2. 在 `Foc_VfParam` 结构体中写原实验使用的电压、频率、偏置；设 Foc_Sweep=1 要求继续扫描。
3. 核对 Foc_Vf_* 别名读回一致。满足原实验条件后释放 Stop。
4. 观察 Foc_Theta、Foc_Udq_Ref、Foc_Idq_Fdbk、母线和保护；电流不能靠 VF 自动限制在给定值。
5. 结束写 Stop=1→IDLE，不用 Foc_Sweep=0 代替停机。

参数通过浮点物理镜像转换为内部 Q24；电压除以 800 V 基值，角度除以 2π，频率转换为每周期 pu 步进。停止时电压镜像可能仍显示设定值，不表示主输出正在导通。

## 5. IF_MODE：电流/频率模式

| 变量 | 单位/默认 | 方向与作用 |
|---|---|---|
| `Foc_IfParam.cur_ref.d / q` | A，默认 0 | d/q 电流给定，真正输入 |
| `Foc_IfParam.freq` | 电角 Hz，默认 0 | 仅使用本模式相位发生器时生效 |
| `Foc_IfParam.offset` | rad，默认 0 | 本模式发生器角度偏置，使用反馈角度时不作为传感器零点修正 |
| `Foc_IfParam.use_sensor` | bool，默认 0 | 0=本模式开环发生器；1=使用 FOC 选出的反馈角度。要明确使用真实传感器，还需 MainInt_UseRealTheta=1 |
| `Foc_Sweep` | bool | 开环发生器扫描请求；use_sensor=1 时不控制反馈角度 |
| `Foc_If_Id / Iq / Frequency / Offset / UseSensor` | 对应同单位 | 只读有效输入别名，不是另一组独立控制寄存器 |
| `Foc_Pid_CurD_Handler.* / CurQ_Handler.*` | 物理 PI 参数 | 电流环增益、输出电压限幅、积分限幅；Reset 为公共状态观察，不是独立复位输入 |

操作：Stop=1，MainInt_State=RUNNING，以 Sensorless_Enabled=0、MainInt_UseRealTheta=1 为有传感基线→选 IF_MODE→写结构体电流给定及角度来源→核对电流 PI→满足原实验条件后 Stop=0。先观察 Id/Iq 反馈及电压给定是否异常，再按原工况调整给定；停止仍用 Stop，不靠把某轴给定改成 0 代替。

开环电流模式：use_sensor=0，按 freq/offset/Foc_Sweep 生成旋转角。传感器电流闭环：use_sensor=1 且 MainInt_UseRealTheta=1。无传感电流闭环：use_sensor=1 且 MainInt_UseRealTheta=0，同时无传感总使能和估计链正常。use_sensor=1 时，网关按原行为将本模式 offset 回填为 0。

## 6. SPEED 与 STARTUP

### 6.1 SPEED 输入与观察

| 变量 | 单位/默认 | 实际作用 |
|---|---|---|
| `Foc_Speed_Ref` | rpm，默认 0 | 速度目标输入，正/负按原方向定义；SPEED 保持 Reset 时每周期清为 0，应在释放 Stop 后再给目标 |
| `Foc_Ramp_Speed_Handler.slope` | rpm/s，默认 250 | 真正斜坡输入；非法值退回已接受值。代码 250 与旧注释“100”不同，以数值为准 |
| `Foc_Ramp_Speed_Handler.limit_min / limit_max` | rpm，默认 −1800/+1800 | 已接通的斜坡限幅输入；必须有限且 Min≤Max，非法值回填 |
| `Foc_Ramp_Speed_Handler.value / target / Ts` | rpm/rpm/s | 输出镜像；实际斜坡、目标及固定速度环周期，不作为强写内核状态的入口 |
| `Foc_Pid_Speed_Effective.*` | 物理 PI 状态 | 实际自适应增益和 PI 状态的只读镜像；不要把其放大值重新写入 Handler 当基础增益 |
| `Foc_Speed_Ramp` | rpm | 斜坡后的目标输出；写它不会绕过斜坡 |
| `Foc_Speed_Fdbk` | rpm | 实际控制反馈输出 |
| `Foc_StartupPrepareRequest` | bool 命令 | SPEED 正常运行时写 1 请求启动准备，网关采样后清零；Stop 中会取消，不是持续使能 |
| `Foc_StartupActive` | bool | SPEED 内部启动准备状态输出；与独立 STARTUP 模式不同 |
| `Foc_Id_Ref / Foc_Iq_Ref`、`Foc_Idq_Ref.*` | A | 速度环/MTPA 产生的电流输出，不是 SPEED 模式的手动电流输入 |
| `Foc_Inductor.* / MTPA_Inductor.*` | H | 当前电感观察镜像 |
| `Mtpa_LinearFallback` | bool，输出 | 1表示当前SPEED电流分配采用`Id=|Iq|`回退；0表示未使用回退，不能据此单独判断表有效或正在运行。写该值不控制算法 |
| `Mtpa_RebuildPending / RebuildFailed / TableValid` | bool | 辨识模型重建状态，不能靠写 1 让模型生效 |

斜坡限幅默认 ±1800 rpm，可通过上述 limit_min/max 调整；这不是允许给任意超范围目标的安全证明，目标和电流/电压限幅仍按原实验要求设置。

表无效、有效区间缺失或`|Iq|`超出表范围时，SPEED不再外推：保留原速度PI限幅后的有符号Iq，取Id=|Iq|。回退电感使用配置中的标称值，默认Ld=0.1 H、Lq=0.016 H，不是新辨识结果。新模型建表失败会保留旧表数据供诊断但撤销其有效性，不能悄悄继续使用旧表；新模型成功提交后恢复查表。原逻辑没有新增dq幅值限幅，因此Iq=24 A时Id也为24 A，合成约33.94 A，可能触发现有30 A相电流保护。不要把线性回退理解为允许扩大电流范围。

回退针对运行期SPEED分配；首次启动/运行中INIT的`MainInt_ControlReady()`仍要求表有效。本轮没有绕过该就绪检查，故表无效时执行INIT仍会失败。运行中失败现会恢复ADC中断使能，但跳过未就绪核心、保持PWM禁止；不能靠改状态或Stop强行恢复。

在 SPEED 路径中检测到 Stop/Reset→Run 释放后，启动准备自动执行，也可由 Foc_StartupPrepareRequest 请求：基础 Ud=5 V、Uq=0，默认持续 0.15 s，约 750 个 5 kHz 周期；然后运行速度环。单纯切换 Foc_Mode 不强制重新准备，推荐按停机→进入 SPEED→释放的顺序。速度环按 10 分频更新，配置目标 500 Hz；电流环配置目标 5 kHz。不能把 Foc_StartupActive 当“电机已启动成功”证明，它只是软件阶段标志。

有传感速度闭环：保持 Sensorless_Enabled=0、MainInt_UseRealTheta=1。要观察无传感估计但仍使用真实反馈：Enabled=1、UseRealTheta=1。无传感闭环见第 7 节，不只改 Method。

操作：停机→RUNNING→选 SPEED→核对反馈来源、斜坡、PI/限幅→满足原实验条件后释放 Stop→按原给定流程写 Foc_Speed_Ref→观察启动阶段、Foc_Speed_Ramp/反馈、电流与保护。停止后再次释放仍重新启动准备，不是沿旧积分接续运行。

### 6.2 PI 字段哪些能改

| 字段 | 方向 | 物理意义 |
|---|---|---|
| `Kp / Ki / Kd` | 输入 | 连续形式物理增益，网关按固定 Ts 生成离散参数；不要先在上位机再乘一次 Ts |
| `MaxOutput / MinOutput` | 输入 | 速度 PI 输出 A；d/q 电流 PI 输出 V |
| `IntegralLimit` | 输入 | 速度 PI 积分 A；电流 PI 积分 V |
| `Reset` | 输出 | 公共 Foc_Reset 状态回填，不是独立可写 PI 复位输入；写它不能代替 Stop，也不会单独门控 PI |
| `integral / previous_error / output` | 输出 | 真实算法状态镜像；不是可写积分预置或输出命令 |
| `Ts` | 输出 | 固定构建时基：速度约 0.002 s，电流约 0.0002 s；写该字段不会修改硬件中断频率 |

默认速度 PI Kp=0.016、Ki=0.060、Kd=0，输出限幅 ±24 A。默认 d 轴 Kp=73.8274273、Ki=408.40704496，输出限幅 ±300 V；q 轴 Kp=27.646015、Ki=408.40704496，输出限幅 ±400 V；两电流环 Kd=0。这些是代码默认数值，沿用原实验参数/限幅做调试，不将表中最大值当首次测试给定。

速度 PI 还有原分段自适应增益调度。这里保留一项明确的镜像语义差别：旧 Handler.Kp/Ki 显示被放大的实际增益；当前 Handler.Kp/Ki 是可写基值，实际物理增益看 `Foc_Pid_Speed_Effective.Kp/Ki`。若把放大后的观察值再当基值输入，会重复放大。PID 的非法/非有限增益、倒置输出限幅、负积分限幅会被拒绝并回填接受值。

### 6.3 独立 STARTUP

`Foc_Mode=STARTUP` 只给固定 Ud=`SENSORLESS_STARTUP_UD`（当前 5 V）、基础 Uq=0。它没有 SPEED 内部 0.15 s 自动结束规则，也不等同完整飞车转速搜索。保持该模式会持续请求该基础电压，需操作人员按原实验流程停止或切换。

它没有一组独立 A2L 启动电压/时间输入；`Foc_VfParam`、`Foc_Speed_Ref` 不能用于修改此模式的基础 Ud。常规有传感准备仍用 Sensorless_Enabled=0、MainInt_UseRealTheta=1。需改这两个编译参数时重新编译并核对；HFI 使能时仍可能附加注入。

## 7. 无传感：总使能、统一 PLL 与实际闭环

### 7.1 三种使用方式

| 目的 | Sensorless_Enabled | MainInt_UseRealTheta | 说明 |
|---|---:|---:|---|
| 正常有传感控制、不使用统一估计/注入 | 0 | 1 | 反馈来自位置传感器；局部 Calculate 可能仍执行，见下文 |
| 对照观察估计，仍用真实反馈 | 1 | 1 | 估计值可观察；若策略启 HFI，仍有附加激励 |
| 无传感闭环 | 1 | 0 | FOC 使用统一估计角度/转速；须在原允许工况确认估计链正常 |

Enabled=0、UseRealTheta=0 不是有效的无位置闭环准备，此时可能选到保留的估计值。停止期间估计链保持/复位值不能代表旋转跟踪准确；仅弱电无真实电机激励也不能验证 LESO/HFI 效果。

与旧浮点实际代码一致，`Sensorless_Calculate` 只由 Reset 拦截，没有用 Sensorless_Enabled 拦截全部局部计算。因此 Enabled=0 且未停止时，LESO/HFI 的策略和局部观测仍可能执行；它关闭的是统一位置 PLL、HFI 电流预处理和电压注入，不能解释为“所有无传感函数都不调用”。

### 7.2 策略与使能变量

| 变量 | 方向/单位 | 真正作用 |
|---|---|---|
| `Sensorless_Method` | 原英文枚举输入/输出 | FLYING=0、LES_OBSERVER=1、HF_INJECTION=2。可给初始/保持区策略，运行策略可能自动覆盖 |
| `Leso_Enabled / Hfi_Enabled` | bool 输入/输出 | 可写对应算法的实际状态，但策略 Calculate 会按速度门槛改写；关闭统一估计/注入用 Sensorless_Enabled，停全部控制用 Stop |
| `Sensorless_Switch_Speed` | rpm 输入 | 默认切换速度 400 rpm |
| `Sensorless_Threshold_Leso / Threshold_Hfi` | rpm 输入 | 默认 340/460 rpm；与 Switch_Speed 是独立已接通寄存器，修改 Switch 不会自动重算二者 |
| `Sensorless_SpeedTarget / SpeedRef / SpeedFdbk` | rpm 输出 | 目标速度、斜坡速度、实际闭环反馈，参与策略判断，不是额外独立目标输入 |
| `Sensorless_Reset` | bool 输出 | 实际停止/复位状态；不是绕过 Stop 的开关 |
| `Flying_Enabled / Flying_Completed` | bool 输出 | 延迟阶段观察，不能解释为已找准电机角度或速度 |

原策略依赖目标速度、斜坡速度和反馈速度，不只由转速单值选 Method；目标在默认 400±10 rpm 保持区时，保留当前 Method。LESO/HFI 启用窗口可重叠，`Method` 表示统一 PLL 当前采用的误差信号，两个 Enabled 不一定互斥。

FLYING=0 是历史枚举值：原代码 `Sensorless_Method & FLYING` 恒为 0，不能据此进入飞车模式；恢复接口也不会把它包装成新的真实搜索算法。原 Flying 与当前 Flying 路径都是周期计数延迟，不包含转速扫描、转向识别或带速角度捕获。

### 7.3 统一 PLL 参数与观察

本轮恢复原接口层统一 PLL：LESO/HFI 提供误差与观测量，统一 PLL 输出估计角度/速度，再反馈给对应观测算法。不是两套独立 PLL 任意选一个输出就宣称与原版等价。

| 变量 | 方向 | 作用/限制 |
|---|---|---|
| `Sensorless_Theta_PID.Kp / Ki / Kd` | 输入及调度输出 | 统一 PLL 物理增益。Kp/Ki 会在位置更新后按速度幅值自适应规则覆盖，手动写入不保证一直保留 |
| `Sensorless_Theta_PID.MaxOutput / MinOutput / IntegralLimit` | 输入 | 统一 PLL 电角速度/积分限幅，物理值 rad/s；默认约 ±700 |
| `Sensorless_Theta_PID.Reset` | 输出 | 公共实际停止/复位状态，不是独立 PLL 复位输入，不是硬件停机命令 |
| `Sensorless_Theta_PID.Ts` | 输出 | 固定主控制周期，不能通过写 Ts 改中断频率 |
| `Sensorless_Theta_PID.integral / previous_error / output` | 输出 | PLL 积分、归一化误差、输出角速度；非用于强写控制状态 |
| `PLLKPMAX / PLLKIMAX` | 输入 | 自适应 Kp/Ki 上限，默认 130/4225 |
| `Sensorless_PLL_Wc` | 输出 | 默认基值/动态带宽观察，单独写它不会成为固定 PLL 带宽输入 |
| `Sensorless_ThetAdj` | rad 输入 | 加到 FOC/串口位置结果的角度补偿，区别于 PLL 本体角度和编码器零偏 |
| `Sensorless_ThetaEst / SpeedEst` | rad / rpm 输出 | ThetaEst 是统一 PLL 本体角，不含 ThetAdj；内部速度滤波与第 10 节串口显示滤波不同 |
| `Sensorless_ThetaErr / SpeedErr` | deg / rpm 输出 | 对真实传感器的估计误差；A2L 角差按 PLL 本体角计算，为实测减本体估计的短角差，不含 ThetAdj |

需调整 PLL 时先记录全部参数和状态，不靠清 Reset/写 estimate 来伪装收敛。自适应覆盖规则、滤波离散实现、定点舍入等仍要做同工况对照，恢复旧名字和框架不代表逐样本完全相同。

2026-10-12 按用户要求将 IQMATH 自适应调度改为正反转对称：当估计速度幅值 `|speed|>500 rpm` 时，Wc=25+(|speed|−500)×0.2，Kp=2Wc、Ki=Wc²，再受 PLLKPMAX/PLLKIMAX 上限约束；其他情况回 Kp=50、Ki=625。默认 ±600rpm 对应约 Kp=90、Ki=2025，±700rpm 及以上按默认上限130/4225。只改变增益调度使用的速度，不改变实际速度/角度方向或鉴相负号。旧 FLOAT_REF 仍保留带符号速度规则，负速增益不再是本版本与旧版逐点相等的项目。`Sensorless_PLL_Wc` 镜像在低速分支仍按原代码保留历史值，当前有效增益以 `Sensorless_Theta_PID.Kp/Ki` 为准。

### 7.4 LESO/HFI 的参数和兼容对象

| 变量 | 方向/默认 | 实际作用 |
|---|---|---|
| `Leso_Gain` | 输入，8 | LESO 自适应带宽系数 |
| `Leso_Wc_Min / Wc_Max` | 输入，600/3990 | 带宽 rad/s 下/上限；必须有限、0<Min≤Max≤3990，否则整对回填原接受值并计无效输入；核心另有不可绕过的3990上限 |
| `Leso_Rs` | Ω 输入，0.65 | LESO 电阻模型参数，与辨识 Rs_est 观察字段区分 |
| `Leso_Factor` | 输出，1.9 | 原设置电感流程每周期会固定为 1.9，写入不持续生效 |
| `Leso_Wc / We / Beta1 / Beta2` | 输出 | 当前带宽、电角速度和观测增益，不通过手动强写设置 |
| `Leso_Ld / Lq / InvLd / InvLq` | H / H⁻¹ 输出 | 动态 MTPA 电感及倒数，最小按 0.001 H 显示/使用边界处理；不是直接在线写电感的接口 |
| `Leso_Int_limit` | 非有效调参入口；本轮A2L地址0 | 源码镜像意图表示约3840A，但未保留ELF符号，不可据该项读写；核心限幅见第13节。不是真实电机电流保护阈值 |
| `Hfi_InjVolt` | V 输入，40 | 真正 HFI d 轴注入幅值；只在总使能、HFI 局部使能及未停止条件同时允许时附加 |
| `Hfi_InjFreq` | Hz 兼容字段，500 | 当前没有作为波形频率输入。实际符号每个调用周期翻转，理想方波频率约控制频率的一半；不能以该字段的 500 判定实测频率 |
| `Hfi_InjectSign` | bool 输出 | 本次 HFI 注入极性 |
| `Leso_Error / Hfi_Error` | 无量纲输出 | 给统一 PLL 的归一化投影误差，不是对传感器的 deg 角差 |
| `Leso_Theta_PID.* / Hfi_Theta_Pid.*` | 保留局部 PID 兼容对象 | 原主动代码中局部 PLL 调用已注释；本轮位置使用统一 PLL。写这些局部对象不调整实际统一 PLL |

HFI 与辨识方波是两套不同激励：Hfi_InjVolt 不修改 Experiment.inj.Ud_amp/Uq_amp。辨识时关闭无传感总使能，避免混入额外 HFI；正常控制如不需要 HFI，也不能只写 Method=LESO 就假定注入已关闭，策略仍会判断局部 Enabled。

## 8. IDENTIFY：保持原上位机操作

用户已确认原硬件上 100 V 注入、3～12 A 扫描经过验证。以下使用这一已验证流程举例，不否定旧实验结果；本轮定点实现仍需对照记录。

### 8.1 参数表：写这些才真正生效

| 变量 | 默认/单位 | 输入/输出及行为 |
|---|---|---|
| `Experiment.start_I / final_I` | 3 / 12，整数 A | 起止电流输入，进入 IDENTIFY 后再设置 |
| `Experiment.step_dir` | +1，整数 A | 原整数步长输入：+2 每步增加 2 A，−1 每步减少 1 A；0/超出表示范围会回退 ±1 并计无效输入；下一步越过终点则结束，不采越界点 |
| `Experiment.repeat_times` | 3 | 每个点重复次数输入，限 1～3 |
| `Experiment.max_steps` | 20 | 最大扫描点数输入，限 1～20；不是要求一定跑 20 点 |
| `Experiment.sample_capacity` | 512 | 采样容量输入，限 2～512 |
| `Experiment.wait_edges` | 3 | 采集前等待边沿数输入，限 0～100 |
| `Experiment.inj.Ud_amp / Uq_amp` | 两者默认 0 V | 两轴方波幅值输入；每次切 Q、DQ 前自动清零，需要手动重写 |
| `Experiment.inj.State` | 默认 0 | 新电流点首次放行输入，点内重复自动执行；采集中写 0 可暂停该次注入输出/采集，按旧逻辑保留采样状态，不是硬件 Stop |
| `Experiment.state` | WAIT | 有效人工命令为 WAIT 中的 EST_RS，以及 Rs 成功后的首次 NEXT_I；其余状态主要观察、由算法推进 |
| `Experiment.inject_amp` | 100 V | 遗留字段，不是当前两轴控制输入；用 inj.Ud_amp/Uq_amp |
| `Experiment.Ts / Rs_est` | s / Ω | 实际时基/Rs 观察；写 Rs_est 不会跳过 Rs 步骤，写 Ts 不会改中断 |
| `Experiment.inj.mode / Imax / IDmax / IQmax` | 轴枚举 / A | 当前轴和阈值输出，不能靠手动改 mode 强行跳轴 |
| `Experiment.inj.Vd / Vq / inj_state_d / inj_state_q` | V / ±1 | 当前输出电压和极性观察 |
| `Experiment.pos / edge_count / repeat_count / step_index` | 计数 | 当前有效采样数/边沿/重复/电流点索引输出 |
| `Experiment.sum_max_psi / sum_max_I / cq_Sxx / cq_Sxy / cq_N / sum_id / sum_id2 / sum_iq / sum_iq2 / sum_eps_id2 / sum_eps_iq2 / count_id / count_iq / edge_idx[] / last_Vd / last_Vq` | 过程观察输出 | 以约 100 Hz 发布当前状态/累计物理镜像；不可写入调算法。cq_N 因旧 uint8 容量封顶 255，完整 DQ 样本数看 count_id/count_iq；edge_idx[1]在 PROCESS 发布采样窗口终点 |
| `I_filtered / I_last / V_last / V_now / done_flag` | A/A/V/V/bool 输出 | 原 Rs 内部过程名字的实际状态镜像，约 100 Hz；done_flag 表示 Rs 完成，不等于整个模型辨识完成 |
| `a_d / b_d / a_q / b_q / c_coeff` | 物理模型系数输出 | 已提交 MTPA 模型参数；在启动/后台提交时刷新，不是在线修改正在生成表格的命令 |
| `Experiment.Initialized / Complete` | bool | 实验已初始化/全部成功完成输出；不手动写 1 让算法完成 |
| `Identification_State / Error / Rs / Voltage` | 状态/错误/Ω/V | 定点状态及物理观察；Voltage 是两轴方波幅值最大绝对值，不是 Rs 输出命令 |

选择 IDENTIFY 不会自动给 100 V 方波：`Experiment.inj.Ud_amp/Uq_amp` 初始均为 0。内核默认参数、遗留 inject_amp 或 Identification_Voltage 初始化曾为 100，不等于两轴电压已放行。

### 8.2 具体步骤

1. Stop=1，MainInt_State=RUNNING，MainInt_UseRealTheta=1，Sensorless_Enabled=0；按原方案完成位置对齐和相序准备，选 Foc_Mode=IDENTIFY。
2. 等 Initialized=1、state=WAIT、inj.mode=INJECT_D，再写 start_I=3、final_I=12、step_dir=1、repeat_times=3、max_steps=20、sample_capacity=512、wait_edges=3；保持两轴幅值和 inj.State 为 0。
3. 原供电/保护条件满足后释放 Stop，确认 Foc_Reset=0。写 `Experiment.state=EST_RS`：Rs 使用自动逐步增压，基础 q 电压为 0，无需写 inj.State=1。默认从 d 轴 1 V 开始，正常响应时约每 3 s 增加 1 V；没有覆盖所有无响应情况的总超时，不长期放任其升压。
4. Rs 成功后回 WAIT，Rs_est 更新；可观察内部 rs_complete=1 与 Error=0，避免把失败回 WAIT 当成功。
5. D 扫描：写 Ud_amp=100、Uq_amp=0，再写 `state=NEXT_I`。等 state=INJECT_COLLECT、inj.mode=D、Imax≈3 A、inj.State=0，写一次 `inj.State=1`。
6. 同一点自动重复 3 次。Id 到 +Imax 切负 Ud，到 −Imax 切正 Ud。点完成后 Imax 自动推进至 4 A，但等待下一次人工 `inj.State=1`；依次至 12 A。不是手动改 Imax，也不用重写幅值。
7. D 全部完成自动拟合 ad0/add，然后切 Q；两轴幅值被清零。等 INJECT_Q/INJECT_COLLECT，写 Ud_amp=0、Uq_amp=100，按每个新电流点放行，Iq 决定 Uq 极性。自动完整扫描，再拟合 aq0/aqq。
8. 自动切 DQ，幅值再次清零，Imax 回起点。写 Ud_amp=100、Uq_amp=100，放行一次。Ud 根据 Id、Uq 根据 Iq 独立翻转；采集边沿窗口依据 d 轴。当前 DQ 只在 start_I 一个点重复 repeat_times 次，不再扫描完整 3～12 A。
9. 完成自动拟合 adq，Complete=1、定点 State=DONE、Error=0，请求 IDLE+停机。保存五个系数、Rs、各轴波形和 J/R2，再等待 IDLE 后台 MTPA 提交完成。拟合完成不等于新表有效：建表失败会撤销活动表有效性，后续SPEED使用线性回退；不要把旧表数组中仍留存的数据当作新模型已接受。

扫描调度使用原整数安培值，不再累加Q24步长：3→12/+1和12→3/−1均10点；3→22/+1为20点；3→12/+2只到11 A，不强行补终点。Rs恢复原预测电流升压条件，未收敛时仅预计下一电流幅值低于8 A才继续加1 V；超流/超步数仍明确FAILED，不冒充辨识成功。

每点内部重复不手动放行；每个新电流点放行一次。PROCESS 时 State 可能短暂为 0，要同时看轴、Imax 和 state，不能看到一次 0 就重发启动。点之间不要 Stop=1，否则辨识进度复位。

### 8.3 停止与结果

正在 INJECT_COLLECT 时，写 `inj.State=0` 已恢复原手动行为：网关采样到该写入的本周期，辨识 Ud/Uq 输出立即置 0，暂停采集推进，采样数、极性、边沿和历史电压状态保留；再写 1 可继续当前点。上位机/CCP 写入到采样仍存在通信延迟，这不是硬件急停。

这也保留了原流程的限制：暂停时内核历史电压（对应旧 last_Vd/last_Vq）不随输出清零，恢复时不能把停顿后的样本拼接当作连续方波实验；未同步的同名浮点镜像不作为判断依据。正式模型辨识不建议采到一半暂停后继续拟合，应重采整个电流点；当前没有单独“作废并重采该点”操作命令时，按 Stop→IDLE→IDENTIFY 重新准备流程处理。正常整机停止仍用 Stop=1→IDLE，紧急使用原硬件急停。修改 inj.mode、state=WAIT、清幅值等都不能代替停机链。

`Experiment.LLS.ad0/add/aq0/aqq/adq`、`LLS.D/Q/DQ.J/R2` 是物理 float 结果镜像，写入不驱动 MTPA。`Experiment.results[]` 的 D/Q 共用镜像会覆盖同索引，需分轴保存。Ud/Id/Uq/Iq/psi_d/psi_q 六组 buf 只有 `[0, pos)` 当前有效，未写区保留旧数据不是本轮结果；采样重复会复用缓存。

内部辨识状态与旧操作状态数值不完全相同：旧 PROCESS=2、INJECT_COLLECT=3；定点两者分别 3、2。优先用英文枚举。FAILED 的旧 Experiment.state 显示 WAIT，但 `Identification_State=8` 和 Error 保留。

Error：0 无辨识错误；1 INVALID_CONFIG；2 SAMPLE_OVERFLOW 枚举预留、当前满容量转 PROCESS；3 RS_NOT_CONVERGED；4 SINGULAR_LLS。失败回 IDLE 停机，先保存诊断，不靠任意改 state 跳过失败。

本轮已恢复上一采样区间电压用于 Ud/Uq 缓存和后续磁链积分，第 0 个磁链点按原流程置 0；不是用本周期刚算出的电压去匹配本次 ADC 样本。

本轮已恢复前区间电压对齐、首点磁链为 0、去均值后的正峰和有符号电流、人工暂停，以及 LLS.D/Q.J 的残差总和显示。拟合内部使用归一化矩阵和 Q48 乘积累计，拒绝病态/不可表示结果，不照搬旧 Q.R2 写错 D.R2 的缺陷。仍有差异：旧 DQ repeat_count 按采样点递增，当前按完整采集轮次重复；默认上限/错误管理和 IDLE 后台重建也不是逐状态逐位等价。旧指南关于不能暂停/未对齐的描述仅适用于前轮固件。

## 9. 主状态、保护、位置零偏和旧观测窗口

### 9.1 `MainInt_State`

| 状态/值 | 操作与当前行为 | 与旧版差别 |
|---|---|---|
| RUNNING / 1 | 正常执行手动选定的 Foc_Mode | 常规操作用它 |
| SENSORLESS / 2 | 每周期自动请求 SPEED，手动模式会被覆盖 | 旧 FLYING=0 位测试恒假，原分支实际也是 SPEED，不声称已增加真正飞车搜索 |
| EXIT / 3 | 强制 Stop=1、IDLE | 不按旧代码另外清保护锁存/禁用 Break IRQ，以免把退出当故障绕过 |
| INIT / 0 | 强制 Stop=1、IDLE并关闭MOE；后台暂禁ADC控制IRQ，重初始化算法、重新校准电流零偏，成功后清旧故障并按新采样Udc>200V重新启BRK中断，回RUNNING | 算法/校准失败为Status=1/2、EXIT且不清故障；INIT期间新BRK故障保留。成功也不自动放行PWM；保留128组新采样校准、在线保护阈值和平均过流累计，不是把整个旧Modules逐指令复制 |

模式操作与状态操作不要混用：比如 SENSORLESS 下选 VF_MODE/IDENTIFY 可能下一周期回 SPEED。SENSORLESS 状态只自动请求 SPEED，并不替操作人员设置 Sensorless_Enabled=1 或 MainInt_UseRealTheta=0；仍须区分第 7 节总使能与反馈选择。真正需要完整硬件初始化按原上电/复位流程执行。

### 9.2 保护与故障复位

| 变量 | 方向/单位 | 作用 |
|---|---|---|
| `Protect_BusVolRate / BusVolFluc` | V 输入 | 母线基准/上限波动参数，当前默认 580/140；欠压底限单独为 20 V，不等于 Rate−Fluc |
| `Protect_CurrentMax` | A 输入 | 实际电流保护阈值，默认 30 |
| `Protect_TempMax` | °C 输入 | 温度阈值，默认 80 |
| `Flag` | 可读写故障锁存 | 恢复原浮点操作：写 0 清故障位，写有效组合位直接生效；不会清 Stop、重校 ADC 或重新开启已禁用的 Break IRQ |
| `FixedBreakIrqArmed` | bool 输出 | 是否开启过 Break IRQ；从未超过 200 V 时为 0 可能属预期，不是栅极安全状态 |
| `Stop / Software_BRK` | 停机请求/执行观察 | Stop 可被保护再次置 1，不能仅因通信写入成功判断允许运行 |

`Flag` 现为唯一有效故障锁存，恢复通过 A2L 直接写 0 清历史故障的行为。操作时保持 Stop=1，先排除原因，再清 Flag；持续过流、母线越界或过温会在下一次对应保护检测重新置位。清 Flag 不清累计过流计数，也不等于重新初始化或允许运行。PE15仍低时，即使Flag=0也不能恢复MOE；已由硬件故障ISR关闭的Break IRQ不会因写Flag=0重新开启。

需要重校零偏并重新武装保护时，确认真实相电流为零，在停机状态写MainInt_State=INIT。等待初始化完成后重新核对模式、在线保护阈值和故障；仍保持Stop=1、IDLE，不自动恢复驱动。INIT按Udc>200V重新开启Break IRQ；PE15异常、母线/温度越界会再次锁存，INIT期间新发生的BRK故障也不会被清除。EXIT仍不清锁存。运行中INIT不再重新生成默认MTPA表；初始化和校准期间CAN接收IRQ仍可达，但CCP回复可能暂时延迟，不反复发送INIT或释放Stop排查。

运行中INIT与首次上电初始化分开：保留`Sensorless_Enabled`、`Sensorless_ThetAdj`、`MainInt_UseRealTheta`、有效的Method/局部使能及原浮点保留的观测历史；按原初始化恢复公共PLL的Kp/Ki/积分限幅等参数，并将PLL积分清零。保留已接受的MTPA五参数、表和当前动态电感、辨识结果，不静默换回默认电机模型。未提交的后台MTPA候选事务取消；已提交模型保持。冷启动仍使用默认参数建表。INIT成功不等于自动启动，不应在初始化期间同时改参数/模式或释放Stop。

运行中INIT结束后的行为已修复：成功时Status=3、RUNNING、IDLE、Stop=1，恢复ADC中断并继续正常控制/保护，仍需显式运行请求。失败时Status=1或2、EXIT、IDLE、Stop=1，同样清旧挂起标志并恢复ADC中断使能；后续有新ADC转换时继续相电流/母线采样和保护，但不执行未就绪算法，PWM参考为中性且主输出禁止。独立内部门控参与硬件恢复判定，写RUNNING、Flag=0、Stop=0或伪写Status=3都不能解除它。排除原因后重新INIT成功才恢复运行资格。若ADC硬件本身不再产生转换，中断使能不等于实际采样已经恢复，故仍保持禁止输出。

此修改针对运行中INIT；首次上电初始化失败仍保持原停止启动流程。本轮继续使用128组新ADC样本的超时校准，不恢复旧2000次紧循环读取；未改保护阈值、GPIO极性或定时器输出配置。

PE4保留原下降沿EXTI4停机，不要求其持续高电平才允许恢复。恢复仍受Stop、Flag及PE15高电平约束。2026-10-12修复两处并发问题：停机期间每次直接清MOE（首次仍触发软件Break，避免连续BRK中断）；恢复判定和MOE使能在短PRIMASK临界区内完成；算法结束后不再回写开始时的Stop旧值，改为检查当前停机请求。恢复的是原浮点持续停机的效果，同时补上原恢复分支的抢占窗口。BRK故障分类、EXIT和异常处理等已记录差异仍存在，不能称整个硬件链逐指令等价。

Flag 是位组合：1=Over_Avg_Current，2=Over_Max_Current，4=Over_Voltage，8=Low_Voltage，16=Hardware_Fault，32=Over_Heat，可同时出现多位。弱电常见 8，并不代表 CAN 无法初始化。

### 9.3 位置零偏

`motor_param.Position_Offset` 与 `Motor_Position_Offset` 为生效输入，单位是传感器原始计数，不是 rad。写合法整计数 0～modulus−1；二者同时修改时结构体输入优先，非法值回填接受值。不要在转动/带电情况下试探零偏。

`motor_param` 其他字段如 Rs/Ld/Lq/Pn、`Motor_Position_Scale/Pn/SampleFreq/Speed_Prescaler` 当前是初始化快照/观察，不因在 A2L 写入就自动改变算法。`Motor_Position` 是原始位置计数 float 镜像，`Motor_Theta_Elec/Mech` 为 rad，`Motor_Speed` 为 rpm。

### 9.4 旧名观测镜像

以下为观察，不作为控制输入；写入会被后续发布覆盖，不能预置核心状态。

| 变量 | 单位与来源 |
|---|---|
| `Leso_Theta / Speed / Theta_Err / Speed_Err` | rad/rpm/deg/rpm；对应实际统一估计接口和对传感器误差 |
| `Leso_CurEst / Current` | αβ 两分量 A；估计电流/实际提供给 LESO 的电流快照 |
| `Leso_Voltage / EmfEst` | αβ 两分量 V；实际提供给 LESO 的电压快照/估计 EMF |
| `Leso_EmfEst_dq / Emf_Filtered / Emf_Slow_Filtered` | d/q 两分量 V；兼容视图，当前主动 LESO 路径不持续更新这些扩展量，不能将其零值/保持值当动态滤波结果 |
| `Hfi_Theta / Speed / Theta_Err / Speed_Err` | rad/rpm/deg/rpm |
| `Hfi_IClarkFdbk / IClarkResp / IClarkFilt` | αβ 两分量 A；反馈/高频响应/基波滤波电流 |
| `mtpa_table[i].T_req / Psi_s / gamma / Id / Iq / Ld / Lq / valid` | Nm/Wb/rad/A/A/H/H/bool；按给定转矩排序的物理表，核心查表仍保持自己的 Iq 排序 |

旧观察别名采用较低频发布，默认约 100 Hz，不能按其两次变化间隔反推控制中断周期。FOC 主要反馈和串口更新周期另按自身路径。Leso_Voltage 现在取实际输入快照，禁用期间可能保留先前电压；不能把保持值当当前 PWM 指令。统一 PLL 会回填 Leso_Theta/Speed 和 Hfi_Theta；旧主动路径没有更新 Hfi_Speed 的局部 PLL，因而 Hfi_Speed 可能保持 0，真正估计转速看 Sensorless_SpeedEst。

## 10. Buffer/串口每个通道是什么

帧内仍是 18 个 IEEE float 数据加 JustFloat 帧尾，上位机显示物理值。只有专门的 Q24 raw 通道把整数数值强转 float 打包；不是直接发 32 位 Q24 二进制。

| 下标 | 当前内容 | 单位 | 定点/浮点边界 |
|---:|---|---|---|
| 0 | 实测电角度 | rad | 位置处理输出物理 float |
| 1 | 估计电角度，含 ThetAdj | rad | 定点统一 PLL 本体角加补偿后回转物理 float；区别于 A2L Sensorless_ThetaEst |
| 2 | 实测机械转速 | rpm | 位置处理输出物理 float |
| 3 | 估计转速，仅串口显示低通后 | rpm | 定点结果回转后，再作显示滤波，绝不作为内部无传感/速度环反馈 |
| 4 | 实测减通道 1 的短角差 | deg | 串口监测计算，包含 ThetAdj 的作用；区别于 A2L 本体角误差 Sensorless_ThetaErr |
| 5 | 本周期母线物理样本 | V | ADC 边界 float 样本 |
| 6 | 同一母线转 Q24 再回转物理值 | V | 检查转换；不是重新采一次 ADC |
| 7 | 同一母线 Q24 raw 数值 | count | `(V/800)×2^24`，整数数值装 float |
| 8 | 当前未写入的保留通道 | — | 初始化为 0，不视作旧观测数据 |
| 9 | 本周期 A 相电流物理样本 | A | ADC 边界 float 样本 |
| 10 | 同一 A 相转 Q24 再回转物理值 | A | 检查转换；不是两套独立电流采样 |
| 11 | 同一 A 相 Q24 raw 数值 | count | `(A/30)×2^24`，整数数值装 float |
| 12 | 当前未写入的保留通道 | — | 初始化为 0 |
| 13 | 当前未写入的保留通道 | — | 初始化为 0 |
| 14 | 当前未写入的保留通道 | — | 初始化为 0 |
| 15 | 最终 d 轴电压指令 | V | 定点结果回转物理 float，包含实际路径叠加的注入 |
| 16 | d 轴反馈电流 | A | 定点 Park 结果回转物理 float |
| 17 | q 轴反馈电流 | A | 同上 |

比较 5≈6、9≈10，以同一输入判断定浮点转换；零输入不能证明全量程正确。Q24 raw 装 float 时低位可能失真，精确整数比较用 A2L 中实际整数字段。

通道3的额外显示低通默认τ=10ms，Ts=200us时α=Ts/(τ+Ts)=1/51≈0.01960784，递推为y+=α×(x-y)。按5kHz连续更新，截止约15.8Hz；已初始化后的阶跃95%约30.4ms、99%约46.6ms，首个有效样本直接初始化输出。该函数在FixedControl_Step完成之后调用，返回值只写Buffer通道3，不改变CAN/A2L原估计速度、内部PLL速度滤波或闭环反馈。算法本来就有的统一PLL内部10点平均和二阶10Hz速度滤波是另一件事，不要混为一谈。

Buffer_Send 按配置每个控制周期请求发送，DMA 忙会丢帧，不能保证 5 kHz 每个样本都送达。改 A2L 变量不会改变上述通道配置；需要调整通道、滤波或发送分频时修改源码、重新 Build。

与旧浮点相比：0～4、9、15～17 主要用途保留；5～7 和 10～11 被用于转换检查，不再是旧 LESO 内部电流通道。要看 LESO 估计/反馈电流，使用第 9.4 节旧名 A2L 镜像。

## 11. 与原浮点的差别速查

| 项目 | 保留/恢复的部分 | 必须知道的差别 |
|---|---|---|
| 模式与操作命名 | 原Foc_Mode英文枚举、VF/IF结构体输入、SPEED给定、主状态；INIT重校ADC/清旧故障、PE4恢复条件已恢复 | INIT保留失败停机及新故障防误清；EXIT不复制旧清保护/禁Break行为；其它明确差异仍存在 |
| 计算表示 | 基础控制/统一 PLL/观测/辨识使用定点状态与显式接口 | Q24 舍入、饱和、表示范围与浮点不同；不是更换 typedef 就完成 |
| PI、相位、控制顺序 | 按旧主动路径的条件积分、局部 Reset、共享相位/冻结、统一 PLL 顺序恢复 | 当前保护先于 Filter/PLL 更新 reset，故障可更早抑制；未复制旧上一周期 Reset 延迟；不保证各波形逐点完全相同 |
| A2L 调参 | 主要旧名字及物理单位镜像 | 输出/未同步兼容字段并非可写生效输入；地址随 ELF 改变，必须配套 A2L |
| 观测显示 | 原 LESO/HFI/统一估计名字 | 额外观察别名约 100 Hz；局部 Hfi_Speed 和未运行扩展量可能保持值 |
| 辨识 | 人工 Rs→D→Q→DQ、每新电流点放行、默认两轴幅值 0，State=0 暂停已恢复 | 暂停保持历史采样不能任意拼接；采样/统计定义等差别仍存在，不宣称完整数值等价 |
| MTPA | 启动定点建表、辨识系数重建、有效范围内Id/Ld/Lq插值；无效/超范围采用Id=\|Iq\| | 优化器停止条件仍与旧版不同；额外拒绝数值异常模型，新模型失败撤销旧表有效性，IDLE后台提交保留 |
| 串口 | 物理 float 帧及主要通道 | 新增转换对照通道、串口估计速度显示低通；不影响内部控制，部分旧通道已改用途 |

保护时序的差异是明确保留的边界：旧 Main_Int 在位置估计之后才传递本周期保护 Reset，位置更新可能先用上一周期 Reset；当前提前检查保护，本周期即可抑制相应 Filter/PLL。没有为了追求逐周期对照而延后停机响应，硬件驱动极性/导通定义不从这一算法差异推断。

MTPA仅对有效表范围内的Id/Ld/Lq做区间插值。未找到匹配区间、无效点或无效表返回`valid=false`，SPEED据此采用`Id=|Iq|`和配置标称电感，不执行末区间外推，也不把查表失败自动改成故障停机。建表使用调用方局部错误状态，饱和、除零、非法数值、无解或无效点均使表无效；不借全局诊断计数差值判断，避免后台与控制中断相互污染。后台失败保留旧数组内容但`table_valid=false`、`Mtpa_RebuildFailed=1`；新表成功才重新置有效。`Mtpa_LinearFallback`明确报告是否正在回退。

MTPA 建表仍不是逐点完全等价：当前二分最多 20 步、黄金分割 24 步并取最终区间中点；旧浮点按 1e-5/1e-4 容差退出且最后还检查 10 个候选点。本轮 51 点宿主对照的最大绝对偏差记录如下，仅说明该组模型/向量，不替代实测或任意参数保证：

| 输出 | 最大绝对偏差 | 最大相对偏差（本次记录） |
|---|---:|---:|
| Id | 0.03335 A | 未据此单独作相对验收结论 |
| Iq | 0.01698 A | 未据此单独作相对验收结论 |
| Ld | 0.242 mH | 0.2152% |
| Lq | 0.00997 mH | 0.02974% |

## 12. 哪些“名字能搜到”但不应直接写

1. 内核 `*_pu`、PI 离散步长、累加器、数组和 `MainInt_ControlState.*` 是定点/算法状态；不是按 A/V/rpm 写入的调参接口。
2. `Foc_Idq_Ref/Foc_Udq_Ref/Foc_Tcm`、估计值、LLS 系数、MTPA 表和观察误差是输出。上位机允许内存写入，不代表控制每周期会读取它。
3. VF/IF 真正输入是 `Foc_VfParam/Foc_IfParam` 结构体；平铺别名只观察。SPEED 目标用 Foc_Speed_Ref，不用 Foc_Speed_Ramp。
4. PID `Ts`、电流/速度频率镜像和 Experiment.Ts 为固定时基观察；修改它们不会改变定时器配置。
5. 无传感总使能、角度来源、Method/局部 Enabled 是不同层级，不能单写某一个就宣称已经无位置闭环。

本文依据：`initialization_iq.c`、`main_int_iq.c/main_int_iq_mode.c`、`fixed_control_gateway.c`、`fixed_debug_gateway.c`、`sensorless_iq_gateway.c/sensorless_iq.c`、`foc_iq.c`、`identification_iq.c`、电机/保护网关，以及旧 `main_int.c/foc.c/sensorless_interface.c` 实际执行路径。修改后必须按最终生成 ELF/A2L 核对名称、类型和读写方向；不混用前轮固件与本轮文档。

## 13. 无位置积分限幅与 LESO 带宽（2026-10-12）

当前实际角度跟踪执行的是统一 PLL。`Sensorless_Theta_PID.IntegralLimit=700` 表示积分项限制在 ±700 **电角 rad/s**，不是 rpm；`MinOutput=-700`、`MaxOutput=700` 是另外的总输出限幅。原浮点和当前定点均先以 P+I 判断是否允许积分，再单独夹紧积分，最后夹紧总输出。定点内部基值为1000rad/s，所以默认积分上限为约0.7 Q24，存在不足一个Q24最低位的量化差。这个输入可通过物理镜像修改，运行中 INIT 按原参数初始化流程恢复默认700。

不要将 `Leso_Int_limit` 和上述 PLL 限幅混为一谈：

| 状态 | 定点定义 | 当前表示/限幅范围约值 | 原浮点数值上限 |
|---|---|---|---|
| PLL积分 | 积分电角速度 / 1000rad/s | 默认 ±700rad/s，可写物理镜像 | ±700rad/s |
| LESO电流估计 | Iest / 30A | ±3840A（实际略小于） | ±1e25，作用于电流估计 |
| LESO扰动积分 | 旧leso_f1 × Ts / 30A | Ts=200us时约 ±1.92e7A/s | ±1e25，作用于扰动积分 |

`Leso_Int_limit` 的源码定义仅意图表示电流估计上限，不是生效的可调输入；本轮ELF未保留该符号，A2L地址为0，所以也不能将它当作可用观察量。它不是电机允许电流，更不是过流保护设置。实际相电流保护仍由保护模块决定。当前定点 LESO 的数值上限与浮点1e25不相同，本轮没有改动这些限幅。局部 `Leso_Theta_PID/Hfi_Theta_Pid` 不执行角度跟踪，不能用它们的限幅判断实际统一 PLL。

`Leso_Gain` 是项目手动配置的无量纲带宽系数，默认8，不是IQmath库自动决定的值。2026-10-12 按用户要求，仅带宽调度改用估计转速的绝对值，电角速度及鉴相仍保留方向：

```text
We = 估计机械转速rpm × 2π × 极对数 / 60
Wc = clamp(Leso_Gain × 1.9 × |We|, Leso_Wc_Min, Leso_Wc_Max)
Beta1 = 2 × Wc
Beta2 = Wc × Wc
```

默认极对数2、Gain8、Min600/Max3990时，未限幅Wc≈3.18348×|rpm|；±100rpm对应600、±300rpm约955、±600rpm约1910、±1000rpm约3183、约±1253.35rpm达到3990，±1800rpm也保持3990，单位均为rad/s（定点实现向下量化硬上限）。正反转调度公式保持，仅按用户要求将原4000上限降至3990；原FLOAT_REF源码仍保留4000。这不意味着反转切换/估计速度过零滞后问题也已修复。

已知未修复的调参边界：当前接受Gain<128，但先计算Gain×1.9，超过约67.37时中间量会饱和；Gain100、50rpm复现Wc由浮点约1989.675变为定点1340.408。默认Gain8不触发，不能据网关接受写入就判定整个调参范围有效。不要通过放大积分/状态上限掩盖该中间运算问题。
