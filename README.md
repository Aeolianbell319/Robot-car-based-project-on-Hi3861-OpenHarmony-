# Hi3861 智能 WiFi 小车 (Smart WiFi Robot Car)

[![OpenHarmony](https://img.shields.io/badge/OpenHarmony-Hi3861-blue.svg)](https://www.openharmony.cn/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

## 📖 项目简介

这是一个基于 **Hi3861** 开发板（OpenHarmony 操作系统）的全栈智能小车项目。项目集成了超声波避障、红外循迹、OLED 状态显示以及 WiFi UDP 远程控制等多种功能。

项目包含两大部分：
1.  **嵌入式固件**：运行在 Hi3861 开发板上的 C 语言程序，负责底层硬件控制和传感器数据处理。
2.  **上位机软件**：运行在 Windows 上的 C# WinForms 应用程序，用于通过 WiFi 发送控制指令。

## ✨ 功能特性

本项目支持通过板载按键（GPIO5）在以下四种模式间循环切换：

1.  **停止模式 (Stop Mode)**
    *   小车保持静止，等待指令或模式切换。
    *   OLED 屏幕显示当前状态。

2.  **循迹模式 (Trace Mode)**
    *   利用双路红外传感器（GPIO11/GPIO12）自动识别黑线路径。
    *   实现自动沿着黑线行驶，偏离时自动修正方向。

3.  **避障模式 (Avoidance Mode)**
    *   结合 **HC-SR04** 超声波传感器和 **SG90** 舵机。
    *   自动检测前方障碍物距离。
    *   当距离过近时，自动停车、后退并转向，寻找无障碍路径。

4.  **遥控模式 (Remote Control Mode)**
    *   启动 UDP 服务器（端口 50001）。
    *   接收来自 Windows 上位机的 JSON 指令。
    *   支持前进、后退、左转、右转、停止等实时控制。

## 🛠️ 硬件清单与连线 (Pin Map)

请按照下表连接各硬件模块：

| 模块 | 功能 | Hi3861 引脚 | 备注 |
| :--- | :--- | :--- | :--- |
| **L9110S 电机驱动** | 左轮前进 (PWM) | GPIO 0 | 复用为 PWM3 |
| | 左轮后退 (PWM) | GPIO 1 | 复用为 PWM4 |
| | 右轮前进 (PWM) | GPIO 9 | 复用为 PWM0 |
| | 右轮后退 (PWM) | GPIO 10 | 复用为 PWM1 |
| **HC-SR04 超声波** | 触发信号 (Trig) | GPIO 7 | 输出模式 |
| | 回响信号 (Echo) | GPIO 8 | 输入模式 |
| **SG90 舵机** | PWM 信号 | GPIO 2 | 产生 20ms 周期脉冲 |
| **红外循迹模块** | 左侧传感器 | GPIO 11 | 输入模式 |
| | 右侧传感器 | GPIO 12 | 输入模式 |
| **SSD1306 OLED** | I2C 数据 (SDA) | GPIO 13 | I2C0 |
| | I2C 时钟 (SCL) | GPIO 14 | I2C0 |
| **板载按键** | 模式切换 | GPIO 5 | ADC 采样检测 |

## 📂 目录结构说明

```
Hi3861_Robot_Car/
├── Robot_Car/                  # 嵌入式端源码 (C语言)
│   ├── BUILD.gn                # 编译构建脚本
│   ├── robot_control.c         # 主控制逻辑与状态机
│   ├── udp_control.c           # UDP 服务端与 JSON 解析
│   ├── robot_l9110s.c          # L9110S 电机驱动 (PWM)
│   ├── robot_hcsr04.c          # HC-SR04 超声波测距驱动
│   ├── robot_sg90.c            # SG90 舵机驱动
│   ├── trace_model.c           # 红外循迹逻辑
│   ├── ssd1306_test.c          # OLED 显示界面逻辑
│   └── ssd1306/                # SSD1306 屏幕底层驱动库
│
└── 小车控制程序/                # 上位机端源码 (C#)
    └── WindowsFormsApplication1/
        ├── Form1.cs            # 界面逻辑与 UDP 发送
        ├── CtrlJson.cs         # JSON 数据实体类
        └── ...
```

## 🚀 快速开始

### 1. 嵌入式端 (Hi3861)

1.  **环境搭建**：安装 DevEco Device Tool 和 OpenHarmony 编译环境。
2.  **导入代码**：将 `Robot_Car` 文件夹复制到 OpenHarmony 源码的 `applications/sample/wifi-iot/app/` 目录下。
3.  **配置构建**：修改 `applications/sample/wifi-iot/app/BUILD.gn`，添加 `"Robot_Car:robot_car"` 到 `features` 列表。
4.  **编译烧录**：使用 DevEco Device Tool 编译项目并烧录到 Hi3861 开发板。
5.  **配网连接**：确保开发板连接到与电脑相同的 WiFi 网络（代码中需配置 WiFi SSID 和密码，或使用配网功能）。

### 2. 上位机端 (Windows)

1.  **打开项目**：使用 Visual Studio 打开 `小车控制程序/WindowsFormsApplication1/WindowsFormsApplication1.sln`。
2.  **编译运行**：点击“启动”运行程序。
3.  **连接控制**：
    *   在文本框中输入小车的 IP 地址（可通过串口日志查看）。
    *   点击控制按钮（前进、后退等）发送指令。

## 📡 通信协议说明

上位机与小车之间使用 **UDP** 协议通信，目标端口为 **50001**。
数据格式为 **JSON** 字符串。

**指令格式：**

```json
{
    "cmd": "指令内容"
}
```

**支持的指令列表：**

| 指令 (cmd) | 描述 | 行为 |
| :--- | :--- | :--- |
| `forward` | 前进 | 两个电机正转 |
| `backward` | 后退 | 两个电机反转 |
| `left` | 左转 | 左轮反转，右轮正转（原地左旋） |
| `right` | 右转 | 左轮正转，右轮反转（原地右旋） |
| `stop` | 停止 | 所有电机停止 |

## 📄 许可证

本项目采用 [MIT License](LICENSE) 许可证。

