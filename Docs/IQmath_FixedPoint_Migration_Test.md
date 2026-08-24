# IQmath 全定点迁移与测试记录

## 1. 基线与构建信息

| 项目 | 当前记录 |
|---|---|
| 基线提交 | `22e6d80e80b35386c064f69b771fb12ec1044290` |
| 开发分支 | `测试2025-12-26` |
| 编译器 | Arm GNU Toolchain 14.2.Rel1，GCC 14.2.1 |
| MCU | GD32F303VCT6，Cortex-M4F |
| 默认数值后端 | `IQMATH` |
| 参考后端 | `FLOAT_REF` |
| IQMATH ABI | Cortex-M4、Thumb-2、`-mfloat-abi=softfp` |
| 主控制频率 | 代码确定为 5 kHz，周期 200 us；上板测量待填写 |
| 速度环频率 | 500 Hz，周期 2 ms |
| `IQmathLib.h` SHA-256 | `896A09E33B72C7F14BB7C5F6E9147FD8D1F9BAE8BFD816A27069B7C900607E08` |
| `IQmathLib-cm3.lib` SHA-256 | `507F3030752DA70826CB46861201D76D241E177DD5654DC3B52E6B18C20AE0B2` |

说明：代码注释原先将 5 kHz/200 us 错写为 10 kHz/100 us，已只修正注释，未修改定时器行为。

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
| 辨识累计量 | 对应 Q24 乘积 | 64 位范围 | `int64_t` | 512 点后缩放回 Q24，检查除零和缩窄范围 |

定点数学统一由 `mc_math.h` 调用 IQmath。加减和缩窄采用饱和；乘、除、sinPU、cosPU、atan2PU 和 sqrt 使用随附 IQmath 库；除零、无效输入和饱和分别累计诊断计数。

## 3. 软件测试记录

| 编号 | 测试步骤 | 预期 | 当前结果 | 状态 |
|---|---|---|---|---|
| SW-01 | 配置 `IQMATH` 后端 | Cortex-M4 softfp，链接 CM3 IQmath 库 | 配置成功 | 通过 |
| SW-02 | 编译全部 50 个 IQMATH 翻译单元 | 无编译错误 | 全部通过 | 通过 |
| SW-03 | 归档并链接最终 ELF/HEX | 无未定义符号、无 ABI 冲突 | ELF/HEX 已生成 | 通过 |
| SW-04 | 检查 ELF ARM 属性 | Cortex-M4、VFPv4-D16、无 hard-float 参数 ABI 标记 | 符合 | 通过 |
| SW-05 | 检查 IQmath 符号 | 存在 mpy/sinPU/cosPU/atan2PU/sqrt | 均存在 | 通过 |
| SW-06 | 检查旧浮点算法符号 | 默认 ELF 不含 CMSIS f32 三角、旧 MTPA/辨识、旧 LESO/HFI | 未发现 | 通过 |
| SW-07 | 反汇编定点核心函数 | 不含 VFP 浮点算术和 `__aeabi_f*` 算术调用 | 审计脚本通过 | 通过 |
| SW-08 | 生成并过滤 A2L | 原 FOC 变量名和物理浮点镜像存在 | 已生成，关键变量存在 | 通过 |
| SW-09 | 目标链接自检程序 | 与 IQmath CM3 库成功链接 | 45,348 B 独立 ELF | 通过 |
| SW-10 | 运行目标板自检 | 四类 failure 均为 0 | 待上板读取 | 待测 |

当前完整固件资源：Flash 41,228 B（15.73%），RAM 18,072 B（36.77%）。RAM 包含为保持原 A2L 变量名而保留的浮点辨识镜像缓冲区；算法累计仍使用 64 位定点。性能只记录，不设提速通过门槛。

### 构建步骤

```powershell
powershell -ExecutionPolicy Bypass -File Tools/BuildSequential.ps1 -Backend IQMATH
```

浮点参考后端：

```powershell
powershell -ExecutionPolicy Bypass -File Tools/BuildSequential.ps1 -Backend FLOAT_REF
```

本机 WinLibs Ninja 可能无法正确回收 ARM GCC 子进程，因此提供顺序构建脚本；它使用 CMake 生成的原始编译、归档和链接命令，不清理或删除源码。

## 4. A2L/CCP 回归步骤

1. 烧录 IQMATH 固件并加载对应 ELF 生成的 A2L。
2. 按原变量名搜索 `Foc_Mode`、`Foc_Speed_Ref`、`Foc_Pid_Speed_Handler.Kp`、`Foc_Idq_Ref.d` 和 `Foc_Udq_Ref.q`。
3. 保持停机，分别写入速度参考和 PI 参数；确认读回值保持物理单位。
4. 观察一个控制周期后定点上下文和输出镜像已更新。
5. 执行 Reset、模式切换和掉电重启，确认镜像与内部状态同步。
6. 读取 `McMath_Diagnostics.*` 和 `FixedControl_SelfTest.*`；正常工况不应出现未解释的计数增长。

实际上位机软件、A2L 文件版本、操作者、日期和结果：**待上板填写**。

## 5. 上板验收矩阵

| 测试 | 操作 | 验收标准 | 实际结果 |
|---|---|---|---|
| 控制频率 | GPIO 翻转或中断计数测量 ADC 主中断 | 5 kHz，允许测量误差 | 待测 |
| 坐标变换/SVPWM | 固定角度与电流/电压向量，对照 FLOAT_REF | 满量程误差不超过 0.1% | 待测 |
| 角度 | 正反转覆盖过零点 | 电角度误差不超过 0.5 度 | 待测 |
| 闭环轨迹 | 空载和带载加减速、正反转 | 关键轨迹误差不超过 0.5% | 待测 |
| 保护 | 限流、母线边界、温度和传感异常 | 正确置位且 PWM 回到安全占空比 | 待测 |
| 无传感 | LESO、HFI、切换及飞车工况 | 无异常跳变，诊断计数可解释 | 待测 |
| 辨识 | 运行固定注入和 512 点累计 | 相对 FLOAT_REF 结果误差不超过 1% | 待测 |
| 性能 | 读取主中断计时并记录最大值 | 记录结果；不得超过 200 us 周期 | 待测 |

## 6. 已知限制与放行条件

- 尚未烧录目标板，因此闭环精度、无传感切换、保护触发和辨识误差不能标记为通过。
- 当前定点 MTPA 使用有界定点律，LESO/HFI 和辨识采用固定运行实现；必须以上板对照结果决定参数，不允许仅凭成功编译放行运行。
- 首次上板必须保持 `Foc_Mode = IDLE`，确认自检、母线、电流零偏、角度、PWM 50% 安全输出和诊断计数后再逐级使能。
- 所有待测项填写并满足阈值后，才能将本记录结论改为“完成验收”。
