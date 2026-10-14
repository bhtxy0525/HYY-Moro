# HYY-Moro
# HYY-Moro

HYY-Moro 机器人开发资料仓库，提供机器人模型、环境配置、API 文档、使用方法和示例程序，支持真机开发与 MuJoCo 动力学仿真。

> 仓库持续建设中。已提供的内容见下方目录，尚未完成的功能列于开发计划。

## 机器人概述

HYY-Moro 是一款轮式人形机器人，由全向移动底盘、躯干、双臂及头部组成，可用于双臂操作、移动操作、遥操作和柔顺控制等研究。

| 部分 | 自由度 |
|---|---:|
| 移动底盘 | 平面运动：x、y、yaw |
| 躯干 | 3 |
| 左臂 | 7 |
| 右臂 | 7 |
| 头部 | 2 |

双臂末端配备六维力/力矩传感器。上肢支持关节位置指令控制，底盘支持平面速度指令控制。具体接口及使用条件见 API 文档。

<!-- 上传图片后，取消下一行注释并修改路径 -->
<!-- ![HYY-Moro](docs/assets/moro.png) -->

## 仓库内容

- **机器人模型**：URDF、MuJoCo MJCF，以及外观和碰撞网格。
- **环境配置**：Ubuntu、HYY SDK、ROS2 和 MuJoCo 的安装与配置方法。
- **API 文档**：控制接口、参数说明、返回值与调用示例。
- **使用手册**：机器人连接、启动、运动控制、传感器读取与停机流程。
- **示例程序**：C++、Python、ROS2 和 MuJoCo 示例。
- **联合调试**：真机数据记录、动力学回放与真机—仿真结果对比。

上述内容按开发进度逐步提供。

## 目录结构

```text
hyy_moro/
├── README.md
├── CHANGELOG.md
├── models/
│   ├── meshes/
│   │   ├── visual/
│   │   └── collision/
│   ├── urdf/
│   └── mjcf/
├── environment/
├── docs/
│   ├── assets/
│   ├── installation/
│   ├── api/
│   ├── usage/
│   └── modeling/
├── examples/
│   ├── cpp/
│   ├── python/
│   ├── ros2/
│   └── mujoco/
├── config/
├── scripts/
└── tools/
```

## 环境配置

真机开发与仿真可以分别配置，无需安装全部组件。

| 使用场景 | 主要依赖 |
|---|---|
| C++ 真机开发 | Ubuntu、CMake、C++ 编译器、HYY SDK |
| ROS2 底盘控制 | ROS2、底盘通信与控制程序 |
| MuJoCo 仿真 | Python、MuJoCo、NumPy |
| 运动学与动力学算法开发 | 按示例需要安装 Eigen、Pinocchio 等依赖 |

已验证的软件版本将在安装文档中列出。

- [环境配置说明](environment/README.md)
- [HYY SDK 安装](docs/installation/hyy_sdk.md)
- [ROS2 环境配置](docs/installation/ros2.md)
- [MuJoCo 环境配置](docs/installation/mujoco.md)
- [网络配置](docs/installation/network.md)

## 快速开始

### 1. 获取仓库

```bash
git clone https://github.com/<用户名或组织名>/hyy_moro.git
cd hyy_moro
```

### 2. 配置环境

根据使用场景，按照环境配置文档安装依赖。真机开发还需要安装匹配版本的 HYY SDK，并配置控制器通信。

### 3. 查看机器人模型

完成 MuJoCo 环境安装后，可加载场景文件：

```bash
python -m mujoco.viewer --mjcf=models/mjcf/moro_scene.xml
```

其中，`moro_scene.xml` 包含机器人模型、地板和灯光配置。模型文件名以实际目录为准。

### 4. 运行示例

各示例目录提供对应的编译、配置及运行说明：

- [C++ 示例](examples/cpp/)
- [Python 示例](examples/python/)
- [ROS2 示例](examples/ros2/)
- [MuJoCo 示例](examples/mujoco/)

## 机器人模型

### URDF

用于机器人结构描述、运动学计算，以及 ROS2、RViz 等工具集成。

### MuJoCo MJCF

用于动力学仿真，配置连杆惯性、关节特性、执行器、碰撞和接触参数。

模型采用以下单位：

| 物理量 | 单位 |
|---|---|
| 长度 | m |
| 质量 | kg |
| 角度 | rad |
| 时间 | s |
| 力 | N |
| 力矩 | N·m |

关节名称、顺序、零位和坐标系见[模型约定](docs/modeling/conventions.md)。参数来源和模型验证结果见[动力学说明](docs/modeling/dynamics.md)及[验证说明](docs/modeling/validation.md)。

## API 文档

API 文档按功能组织：

- 机器人初始化与状态读取
- 关节运动控制
- 笛卡尔运动控制
- 底盘速度控制
- 力/力矩及其他传感器读取
- 数据记录
- 错误码与异常处理

每个接口说明包含函数声明、参数单位、坐标系、返回值、调用条件及示例。

[查看 API 文档](docs/api/overview.md)

## 使用手册

- [启动与停机](docs/usage/startup.md)
- [关节控制](docs/usage/joint_control.md)
- [笛卡尔控制](docs/usage/cartesian_control.md)
- [移动底盘](docs/usage/mobile_base.md)
- [传感器使用](docs/usage/sensors.md)
- [MuJoCo 仿真与联合调试](docs/usage/mujoco.md)
- [常见问题](docs/troubleshooting.md)

## 真机与 MuJoCo 联合调试

联合调试功能计划支持：

1. 将运动指令同步提供给真机和 MuJoCo 控制器。
2. 通过动力学积分生成仿真运动。
3. 记录真机与仿真的关节位置、速度及相关力学数据。
4. 按时间戳对齐，分析跟踪误差与动力学响应差异。
5. 提供动作回放、网页可视化和一键启动功能。

模型精度及适用范围以验证文档为准。

## 开发计划

- [ ] 整理 URDF 与三维网格
- [ ] 完善 MuJoCo 动力学模型
- [ ] 编写环境配置文档
- [ ] 整理核心 API 文档
- [ ] 提供基础控制与传感器示例
- [ ] 实现运动数据记录与动力学回放
- [ ] 完成真机与仿真结果对比
- [ ] 实现实时联合调试
- [ ] 增加网页可视化与一键启动
