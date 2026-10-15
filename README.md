# 具身智能巡检车 · STM32 下位机

面向 ROS2 巡检车的 STM32F407VET6 下位机程序，采用四轮独立驱动、独立转向结构。驱动使用达妙 3519 电机，转向使用 PWM 舵机，航向传感器为 HWT101CT（TTL）。

## 当前进度

已实现串口 DMA 收发、上下位机协议、CAN 通信、电机与陀螺仪反馈解析、四轮逆运动学及连续运动规划，并提供电脑端联调和无硬件测试工具。

**当前主程序仅运行通信和运动计算预览，不使能驱动电机、不发送速度目标、不启动舵机 PWM。接入上位机或发送速度帧不会让车辆行驶。**

舵机 PWM 驱动、执行数据生成、启停管理和单轮标定接口已编写，实际执行流程尚未接通；方向、中位及整车运行效果仍需硬件联调。启停和标定串口扩展保留为草案。

## 硬件接口

| 用途 | 外设 | STM32 引脚 | 配置 |
|---|---|---|---|
| ROS2 上位机 | USART2 | TX：PA2，RX：PA3 | 115200、8N1 |
| 蓝牙调试 | UART5 | TX：PC12，RX：PD2 | 115200、8N1；当前输出诊断信息 |
| HWT101CT | USART6 | TX：PC6，RX：PC7 | 115200、8N1；传感器输出 50Hz |
| 达妙 3519 | CAN1 | TX：PB9，RX：PB8 | 1Mbps，经 CAN 收发器连接总线 |
| 四路转向舵机 | TIM3 PWM | 左前 PB4、右前 PB5、左后 PB0、右后 PB1 | 50Hz |

- 四轮顺序统一为：左前、右前、左后、右后。
- 电机 CAN ID 为 `1、2、3、4`，反馈 Master ID 为 `0x11、0x12、0x13、0x14`，使用速度模式 `CTRL_MODE=3`；需在实物驱动器中对应设置。
- 轮径 125mm，转向轴心左右间距 285mm、前后间距 395mm；电机输出轴直接连接车轮。
- 舵机信号由 STM32 直接输出，使用独立合适电源并共地；USART3 空闲，不使用 SD 卡。
- 详细接线、电平及标定约定见 [项目关键信息](项目关键信息.md)。

## 代码结构

| 路径 | 内容 |
|---|---|
| `Core/`、`Drivers/` | CubeMX 初始化、启动流程和 STM32 HAL 驱动 |
| `user/inspection_app.*` | 应用入口，调度接收、设备查询、计算预览和反馈 |
| `user/inspection_config.h` | 底盘尺寸、运动限制、频率和超时等参数 |
| `user/uart_dma.*`、`user/can_link.*` | 串口 DMA 与 CAN 底层收发 |
| `user/host_protocol.*` | 上下位机协议解析和反馈打包 |
| `user/dm3519.*`、`user/hwt101ct.*` | 驱动电机与陀螺仪协议 |
| `user/swerve_kinematics.*`、`motion_limits.*`、`chassis_planner.*` | 运动学、限速和连续运动规划 |
| `user/chassis_output.*`、`servo_calibration.*`、`servo_pwm.*` | 执行数据生成、舵角映射及 PWM 驱动 |
| `user/drive_supervisor.*`、`single_wheel_test.*` | 启停、故障恢复和单轮标定内部接口 |
| `user/tools/`、`user/tests/` | 电脑端联调工具和无硬件测试 |
| `STM32F407.ioc` | CubeMX 配置文件 |

## 上下位机通信

上位机按 100Hz 下发车体速度 `vx、vy、wz`；STM32 分别按 50Hz 上传底盘、陀螺仪反馈，按 1Hz 上传通信诊断。坐标为 X 向前、Y 向左、Z 向上，逆时针为正；单位使用 m/s、rad/s 和 rad。

帧使用 `AA 55` 帧头、小端编码和 CRC16/CCITT-FALSE 校验。运动指令 200ms 未更新时，规划轮速逐步降至零并保持转角；设备数据 100ms 未更新时失效。上位机必须依据有效标记使用反馈，目标舵角不能当作实测舵角。

完整帧格式见 [上下位机通信协议](上下位机通信协议.md)，Python 参考实现见 [host_protocol.py](user/tools/host_protocol.py)。

## 编译与联调

固件使用 CMake、Ninja 和 Arm GNU Toolchain。将工具加入 PATH 后，在项目根目录运行：

```sh
cmake --preset Debug
cmake --build --preset Debug
```

生成文件位于 `build/Debug/`。自写功能集中在 `user/`；修改 CubeMX 生成文件时，应放在对应的 `USER CODE` 区域。

电脑端无硬件协议演示仅需 Python 3：

```sh
python user/tools/serial_console.py demo --seconds 1 --vx 0.1
```

真实串口监听需先安装 `pyserial`，将 `COM5` 替换为实际端口：

```sh
python -m pip install pyserial
python user/tools/serial_console.py monitor --port COM5
```

Windows PowerShell 下，安装电脑端 GCC 后可运行 C 测试及运动指令回放：

```powershell
./user/tests/run.ps1
./user/tools/simulate.ps1
```

回放结果写入 `build/simulation/`，仅表示控制算法计算结果，不代表实车运动或动力学仿真。
