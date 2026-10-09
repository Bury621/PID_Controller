# 软件实现的 PID 控制器

这是一个使用 C 语言实现的轻量级 PID 控制器。代码不依赖动态内存分配，所有状态都保存在调用者提供的 `PIDController` 结构体中，适合在嵌入式系统、实时控制程序以及其他需要固定采样周期的场景中使用。

## 功能特点

- 支持完整的比例、积分和微分控制。
- 积分项采用梯形法计算，并通过对积分累加值限幅实现抗积分饱和。
- 微分项基于测量值计算，而不是基于误差计算，可避免设定值突变造成的微分冲击。
- 微分项带一阶低通滤波，可抑制测量噪声。
- 支持控制器输出限幅和积分器限幅。
- 支持可选的误差死区。
- 核心实现仅包含 `PID.c` 和 `PID.h`，不依赖外部库。

## 文件说明

| 文件 | 说明 |
| --- | --- |
| `PID.h` | PID 控制器结构体及公开接口 |
| `PID.c` | PID 控制器算法实现 |
| `PID_Test.c` | 一阶系统仿真示例 |
| `PID Controller Implementation in Software.pdf` | PID 软件实现参考资料 |

## 算法说明

每个采样周期内，控制器根据设定值 `setpoint` 和测量值 `measurement` 计算控制输出。

误差定义为：

```text
error = setpoint - measurement
```

比例项为：

```text
proportional = Kp * error
```

积分项使用梯形法更新：

```text
integrator += 0.5 * Ki * T * (error + prevError)
```

更新后会使用 `limMinInt` 和 `limMaxInt` 对积分项限幅，以减小积分饱和对系统的影响。

微分项采用“对测量值求导”的形式，并使用一阶低通滤波：

```text
differentiator =
    -(2 * Kd * (measurement - prevMeasurement)
      + (2 * tau - T) * differentiator)
    / (2 * tau + T)
```

这种写法不会因为设定值突然变化而产生过大的微分冲击。最终输出为：

```text
out = proportional + integrator + differentiator
```

输出最终会被限制在 `limMin` 和 `limMax` 之间。

当启用死区且误差满足 `-deadband < error < deadband` 时，控制器会清零积分项、微分项和输出，并直接返回 `0.0f`。如果系统不需要死区，应将 `deadband` 设置为 `0.0f`。

## 快速开始

### 1. 添加源文件

将 `PID.c` 和 `PID.h` 加入工程，然后在需要使用控制器的源文件中包含头文件：

```c
#include PID.h
```

### 2. 配置并初始化控制器

推荐先配置控制器参数，再调用 `PIDController_Init()` 清除运行时状态：

```c
#include PID.h

int main(void)
{
    PIDController pid = {
        .Kp = 2.0f,
        .Ki = 0.5f,
        .Kd = 0.25f,

        .tau = 0.02f,

        .limMin = -10.0f,
        .limMax = 10.0f,
        .limMinInt = -5.0f,
        .limMaxInt = 5.0f,

        .T = 0.01f
    };

    PIDController_Init(&pid);

    /* 当前实现会在 Init 中将 deadband 清零，因此需要在 Init 后设置。 */
    pid.deadband = 0.0f;

    return 0;
}
```

### 3. 周期性更新

控制器需要按照固定的采样周期调用。`measurement` 是系统当前测量值，返回值是新的控制输出：

```c
float setpoint = 1.0f;
float measurement = ReadMeasurement();
float control_output = PIDController_Update(&pid, setpoint, measurement);

ActuatorWrite(control_output);
```

典型控制循环如下：

```c
while (1) {
    WaitForNextSample();

    float measurement = ReadMeasurement();
    float control_output = PIDController_Update(&pid, setpoint, measurement);

    ActuatorWrite(control_output);
}
```

结构体中的变量使用 `float`，如果目标平台没有硬件浮点单元，软件浮点运算会增加 CPU 开销。

## 参数说明

| 参数 | 说明 |
| --- | --- |
| `Kp` | 比例增益 |
| `Ki` | 积分增益 |
| `Kd` | 微分增益 |
| `tau` | 微分项低通滤波时间常数，单位为秒，通常应大于或等于 `0` |
| `deadband` | 误差死区，单位为测量值单位，建议不小于 `0`；设为 `0` 表示关闭 |
| `T` | 采样周期，单位为秒，必须大于 `0` |
| `limMin` | 控制器输出下限 |
| `limMax` | 控制器输出上限 |
| `limMinInt` | 积分项下限 |
| `limMaxInt` | 积分项上限 |

结构体中的 `integrator`、`prevError`、`differentiator`、`prevMeasurement` 和 `out` 由控制器内部维护，不需要在控制循环中手动修改。

## API

### `void PIDController_Init(PIDController *pid)`

清除控制器的积分器、历史误差、微分项、历史测量值和输出，并将 `deadband` 设置为 `0.0f`。

该函数不会修改 `Kp`、`Ki`、`Kd`、`tau`、输出限幅、积分限幅和采样周期。

### `float PIDController_Update(PIDController *pid, float setpoint, float measurement)`

根据设定值和当前测量值计算下一个控制输出，并更新控制器内部状态。函数返回限幅后的控制输出。

## 编译和运行示例

可以使用 GCC 或 Clang 编译：

```bash
gcc PID.c PID_Test.c -o pid_test -lm
./pid_test
```

Windows 下如果生成 `.exe` 文件：

```powershell
gcc PID.c PID_Test.c -o pid_test.exe -lm
.\pid_test.exe
```

使用 MSVC 时：

```bat
cl PID.c PID_Test.c /Fe:pid_test.exe
pid_test.exe
```

示例程序使用 10 ms 采样周期、持续运行 4 秒，输出格式为：

```text
Time (s)        System Output   ControllerOutput
```

输出数据可以重定向到文件后，使用 Excel、Python 或 MATLAB 绘制响应曲线。

> 注意：当前 `PID_Test.c` 仍使用旧字段顺序的聚合初始化，尚未为新增的 `deadband` 字段留出位置。直接编译示例时，部分参数可能发生错位。实际使用时请采用本文“配置并初始化控制器”中的指定成员初始化方式，或先更新示例程序的初始化代码。

## 调参建议

1. 先将 `Ki` 和 `Kd` 设为 `0`，逐步增大 `Kp`，直到系统获得合适的响应速度，同时保留一定稳定裕量。
2. 适度增加 `Ki` 以消除稳态误差，同时观察积分限幅和输出限幅是否合理。
3. 最后加入 `Kd` 抑制超调和振荡，并根据测量噪声设置合适的 `tau`。
4. 确保实际控制循环的调用周期与 `T` 一致。采样周期不稳定会直接影响积分和微分计算。
5. 如果执行器存在饱和或机械限制，应正确设置输出限幅和积分限幅。

## 注意事项

- `PIDController_Init()` 应在控制器开始工作前调用，不要在运行过程中随意调用，否则会清除控制器状态。
- `T` 不能为 `0`。`tau` 和 `T` 的具体取值应结合采样频率及测量噪声确定。
- 控制器不会自动处理多线程并发访问。如果多个线程可能同时更新同一个控制器，需要由调用方加锁或保证串行调用。
- 当前实现假设调用方按照固定周期提供测量值，并在外部完成执行器安全限制、传感器异常处理和故障保护。

## 许可证

本项目使用 MIT License，详见 [LICENSE](LICENSE)。
