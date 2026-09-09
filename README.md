# AIDoLamp - 基于多模态交互与机械臂追踪的智能台灯

围绕“视觉感知—智能决策—机械执行”组织智能台灯项目，包含**六轴机械臂解析 IK 与关节轨迹算法演示**，以及树莓派 5 + STM32 多模态交互原型。

**六轴解析 IK · 多解选解 · 人脸降维求解 · 书本照射姿态 · 关节限位 · 五次多项式轨迹 · 双摄视觉感知**

## 项目组成

| 组成 | 内容 | 当前状态 |
|------|------|----------|
| [六轴运动算法](src/motion6dof/) | FK / 几何解析 IK、分支选解、工作空间投影、照射姿态规划与六关节轨迹 | 独立可运行的软件演示，已做数值测试，尚未接入硬件 |
| [多模态交互原型](src/main.py) | 双 CSI 摄像头、YOLOv8、MediaPipe、语音交互、四模式状态机与 UART | 原四轴实物配套代码，照片展示此版本 |
| [视觉优化资料](training/yolo/identify_pi/) | YOLOv8n 训练、ONNX 导出、量化与 ONNX Runtime 推理实验 | 独立开发资料；主程序默认使用 `best.pt` |

## 六轴运动学与关节轨迹

- **运动学与姿态规划**：理想球腕六轴模型，解析解耦腕心位置与末端朝向；枚举基座、肘部、腕部解，并以 FK 回代检查结果。
- **任务降维与全关节协调**：人脸场景保持前三关节，通过腕部瞄准；书本场景设置照射高度，通过六关节 IK 协调光源位置与姿态。
- **可达性与选解**：支持几何工作空间投影、关节限位检查，以及根据当前角度选择邻近 IK 分支；不满足目标的结果会被拒绝。
- **轨迹控制**：五次多项式关节插值，输出时间戳、位置、速度、加速度；支持步数、时长和关节速度上限，端点速度与加速度为零。

![六轴运动算法演示](docs/six-axis-demo.png)

在仓库根目录运行，基础演示仅需 NumPy，不需要摄像头、模型权重或串口设备：

```bash
python -m pip install numpy
python -m src.motion6dof
python -m src.motion6dof --output output/six-axis-demo.json
python -m unittest discover -s tests -v
```

演示使用合成的书本与人脸目标，输出六关节角、轨迹及 FK 校验残差。模型定义、公式、测试范围和可选绘图命令见[六轴算法说明](docs/six-axis-motion.md)。六轴模块为新增的软件扩展，未开展六轴真机验证；下方介绍的是原四轴实物系统。

## 多模态交互实物原型

<p align="center">
  <img src="media/side-view.jpg" width="380" alt="AIDoLamp 侧视图"/>
  <img src="media/front-view.jpg" width="380" alt="AIDoLamp 正视图"/>
</p>

基于 **树莓派5 + STM32F407ZGT6** 的多模态交互智能台灯，围绕“视觉感知—模式决策—机械执行”组织目标检测、几何法逆运动学、关节空间插值与串口控制，结合 MediaPipe 手势/坐姿识别和 DeepSeek 语音交互，提供待机、普通、互动、写作四种工作模式。

该实物版本采用底座、肩、肘、腕四个关节；实现入口与版本边界见[四轴原型实现说明](docs/motion-control.md)。

## 实物展示

<p align="center">
  <img src="media/front-view.jpg" width="600" alt="AIDoLamp 实物"/>
  <br/>
  <sub>需要观看作品演示视频，可联系作者提供。</sub>
</p>

## 技术栈

| 层级 | 组件 | 职责 |
|------|------|------|
| 上位机 | 树莓派5（8GB） | 视觉处理、AI 推理、多线程任务调度 |
| 下位机 | STM32F407ZGT6 | 舵机 PWM 控制、超声波测距、OLED 显示、灯光调节 |
| 机械臂 | 自研4轴（4 舵机） | 底座旋转 + 肩/肘/腕关节，3D 打印灯罩 |
| 视觉 | 双 CSI 摄像头 | cam1（底座）：手势/坐姿；cam2（灯罩）：YOLO 检测 |
| 通信 | UART 115200bps | 树莓派 ↔ STM32 双向通信 |
| 传感器 | TSL2591 + 超声波 + 可编程 LED | 环境光感知、距离检测、自适应调光 |

## 系统架构

```
树莓派5（视觉 & AI 大脑）                    STM32F407（实时控制）
  ├── cam1: MediaPipe 手势识别                 ├── PCA9685 驱动 4路舵机 PWM
  ├── cam1: MediaPipe 坐姿检测                 ├── 超声波测距 → UART 上报
  ├── cam2: YOLOv8 人脸/书本检测               ├── TSL2591 环境光采集 → 自适应调光
  ├── 逆运动学解算 → UART 发送舵机角度          ├── OLED 模式状态显示
  ├── DeepSeek API 语音对话                    ├── PWM 灯光亮度/色温控制
  └── 百度语音 ASR/TTS                         └── 串口命令解析与分发
```

## 四种工作模式

| 模式 | 触发方式 | 功能 |
|------|----------|------|
| **待机模式** | 默认 / 手势切换 | 超声波持续监测，距离突变 >10cm 自动唤醒进入普通模式 |
| **普通模式** | 超声波触发 | 手势控制灯光亮度/色温、舵机方向/高度（10种手势，5指张开3秒激活） |
| **互动模式** | 手势切换 | 多线程：YOLOv8 人脸检测 + 机械臂跟随 + "你好悠悠"语音唤醒 + DeepSeek 对话 + 天气查询 |
| **写作模式** | 手势切换 | 多线程：YOLOv8 书本检测 + 机械臂跟随 + 坐姿检测语音提醒 + 环境光自适应调光 |

## 手势交互设计

系统支持 10 种手势，采用三级状态流转：**未激活 → 激活 → 模式控制**

| 手势编号 | 名称 | 功能 |
|----------|------|------|
| 1 | activate（五指张开） | 激活手势系统（需持续3秒） |
| 3 | exit | 退出当前模式 / 关闭激活状态 |
| 5 | mode1 | 进入待机模式 |
| 6 | mode2 | 进入普通模式 |
| 7 | mode3 | 进入互动模式 |
| 8 | mode4 | 进入写作模式 |
| 10 | up（拇指向上） | 增强光照 / 舵机向上 |
| 2 | down（拇指向下） | 减弱光照 / 舵机向下 |
| 9 | right | 灯身向右 / 色温调节 |
| 4 | left | 灯身向左 / 色温调节 |

防误触机制：激活手势需持续 3 秒，功能手势需持续 1 秒，操作间隔 1 秒防抖。

## 原型的运动控制与视觉感知

| 技术方向 | 当前实现 | 代码入口 |
|----------|----------|----------|
| 运动学与照射姿态 | 建立四轴连杆模型（L1/L2/L3/L4 = 8/24/24/8 cm）；人脸跟踪固定肩肘、求解底座与腕部；书本跟踪通过平面几何与余弦定理协调四关节 | [object_tracking.py](src/object_tracking.py) |
| 可达性与关节约束 | 修正超出连杆可达范围的腕部目标，裁剪关节角度；书本照射目标结合高度偏移与距离参数生成 | [object_tracking.py](src/object_tracking.py) |
| 关节轨迹控制 | 主程序通过线性关节插值发送四路角度，以 `duration` 和 `steps` 调节发送节奏；运动学模块另提供余弦插值工具 | [serial_comm.py](src/serial_comm.py) |
| 视觉引导跟踪 | 人脸/书本检测 → 坐标估计 → IK → 角度单位转换 → 插值 → UART → STM32 舵机指令处理 | [main.py](src/main.py)、[contact.c](firmware/Core/Src/contact.c) |
| 双摄感知与调度 | 摄像头 1 用于手势/坐姿，摄像头 0 用于目标检测；以四模式状态机和工作线程组织感知、跟踪与语音任务 | [main.py](src/main.py)、[csi1.py](src/csi1.py) |
| 推理优化资料 | YOLOv8n 训练、ONNX 导出、量化与 ONNX Runtime 推理实验；主程序默认加载 `best.pt` | [树莓派 YOLO 开发资料](training/yolo/identify_pi/) |

手势识别采用 MediaPipe 21 个关键点、63 维特征与随机森林分类器；坐姿检测采用 MediaPipe Pose + FaceMesh；语音交互接入百度 ASR/TTS、DeepSeek 与和风天气，唤醒词为“你好悠悠”。

ONNX Runtime 实验属于 YOLO 目标检测模块。坐标估计、串口角度状态与跟踪节奏的详细说明见[四轴原型实现说明](docs/motion-control.md#实现范围与验证边界)。

## 目录结构

```
AIDoLamp_end/
├── src/                          # 上位机源代码（Python，运行于树莓派5）
│   ├── main.py                   # 主程序入口 & 四模式状态机
│   ├── gesture_class.py          # 手势识别（MediaPipe + RandomForest）
│   ├── posture_class.py          # 坐姿检测（MediaPipe Pose + FaceMesh）
│   ├── csi1.py                   # 综合检测系统（手势+坐姿联合，写作模式用）
│   ├── yolo_class.py             # YOLOv8 目标检测（人脸/书本）
│   ├── object_tracking.py        # 机械臂逆运动学 & 目标追踪
│   ├── motion6dof/               # 独立六轴算法演示（不接入原型串口）
│   │   ├── kinematics.py        # 六轴 FK / 解析 IK / 工作空间投影
│   │   ├── trajectory.py        # 六关节五次插值与速度约束
│   │   ├── tracking.py          # 人脸 / 书本照射姿态规划
│   │   └── __main__.py          # 无硬件演示、JSON 与可选绘图
│   ├── serial_comm.py            # UART 串口通信（舵机/灯光/模式控制）
│   └── voice_class.py            # 语音助手（百度ASR/TTS + DeepSeek + 天气）
├── firmware/                     # 下位机源代码（C，STM32F407ZGT6）
│   ├── Core/
│   │   ├── Inc/                  # 头文件
│   │   └── Src/                  # 源文件
│   │       ├── main.c            # STM32 主程序
│   │       ├── contact.c         # 串口命令解析与执行（模式/舵机/灯光）
│   │       ├── chuankou.c        # UART DMA 通信驱动
│   │       ├── PCA9685.c         # PCA9685 舵机驱动（I2C）
│   │       ├── TSL.c             # TSL2591 光照传感器 + 超声波测距
│   │       ├── oled.c            # OLED 显示驱动
│   │       └── ...               # HAL 外设初始化
│   ├── ee.ioc                    # STM32CubeMX 工程配置
│   └── STM32F407ZGTX_FLASH.ld   # 链接脚本
├── training/                     # 各子模块开发 & 训练资料
│   ├── gesture/                  # 手势识别训练（3个版本：pc/pi/重构版）
│   │   ├── mediaPipe_Hands_pc/   # PC 版训练流水线（特征提取→模型训练）
│   │   ├── mediaPipe_Hands_pi/   # 树莓派版
│   │   └── mediaPipe_Hands_重构版本/  # 重构后的模块化版本
│   ├── yolo/                     # YOLOv8 目标检测训练
│   │   ├── identify_pc/          # PC 端训练 & 推理脚本
│   │   ├── identify_pi/          # 树莓派端优化版本
│   │   └── identify_train/       # 独立训练环境
│   ├── robotic-arm/              # 机械臂 IK 算法开发
│   │   ├── Robotic arm linux/    # Linux/树莓派版
│   │   └── Robotic arm window/   # Windows 调试版
│   ├── voice/                    # 语音助手独立开发
│   └── pose/                     # 坐姿检测（LSTM + 规则两种方案）
├── models/                       # 训练好的模型权重（本地保留，不提交git）
│   ├── gesture_model.pkl         # 手势分类模型（RandomForest）
│   └── best.pt                   # YOLOv8 自训练权重（4类：person_a/person_b/person_c/book）
├── docs/                         # 项目文档 & 训练结果可视化
│   ├── six-axis-motion.md        # 六轴模型、算法与验证范围
│   ├── six-axis-demo.png         # 合成目标姿态与轨迹示意
│   ├── motion-control.md         # 四轴原型实现说明
│   ├── gesture_confusion_matrix.png  # 手势识别混淆矩阵
│   ├── gesture_feature_importance.png # 手势特征重要性
│   └── yolo_results.png          # YOLOv8 训练曲线
├── media/                        # 演示媒体
│   ├── side-view.jpg             # 实物侧视图
│   └── front-view.jpg            # 实物正视图
├── .env.example                  # API 密钥配置模板
├── tests/test_motion6dof.py      # 六轴数值与任务规划测试
├── .gitignore
├── requirements.txt              # Python 依赖
└── README.md
```

## 硬件清单

| 硬件 | 型号/规格 | 用途 |
|------|-----------|------|
| 上位机 | 树莓派5（8GB） | 视觉处理 & AI 推理 |
| 下位机 | STM32F407ZGT6 | 实时控制 & 外设管理 |
| 摄像头 | CSI 摄像头 x2 | cam1 底座（手势/坐姿）+ cam2 灯罩（YOLO） |
| 舵机驱动 | PCA9685（I2C） | 4路舵机 PWM 生成 |
| 舵机 | x4 | 底座(-135~135°) / 肩(-90~90°) / 肘(0~150°) / 腕(-90~40°) |
| 光照传感器 | TSL2591（I2C） | 环境光检测，写作模式自适应调光 |
| 超声波模块 | HC-SR04 | 距离检测，待机→普通模式触发 |
| OLED 屏 | 128x64（I2C） | 当前模式与状态显示 |
| LED 灯 | 可编程 PWM 控制 | 灯光亮度/色温调节 |
| 语音模块 | USB 麦克风 + USB 扬声器 | 语音交互 |
| 灯罩 | 3D 打印（自主设计） | 末端执行器，内置 cam2 |

## 下位机固件说明

固件基于 STM32CubeMX + HAL 库开发，核心模块：

| 文件 | 功能 |
|------|------|
| `contact.c` | 串口命令分发：MODE（模式切换）、Alldro（全舵机控制）、LIGHT（灯光调节）等 |
| `PCA9685.c` | I2C 驱动 PCA9685，控制 4 路舵机 PWM（支持 180°/270° 舵机） |
| `TSL.c` | TSL2591 光照采集 + 超声波触发/测距 + PWM 自适应调光 |
| `oled.c` | I2C 驱动 OLED 屏，显示当前模式（待机/陪伴/互动/写作） |
| `chuankou.c` | UART6 DMA 收发 + 命令解析器（支持最多6参数） |

> 编译需要 STM32CubeIDE。打开 `firmware/ee.ioc` 用 CubeMX 生成 HAL 驱动后即可编译。

## 四轴原型运行

### 环境配置

```bash
# 安装 Python 依赖（树莓派环境）
pip install -r requirements.txt

# 树莓派专用库（系统级安装）
sudo apt install python3-picamera2 python3-libcamera

# 配置 API 密钥
cp .env.example .env
# 编辑 .env 填入你的 百度/DeepSeek/天气 API Key
```

### 运行

```bash
# 需要在树莓派上运行，并连接 STM32 下位机
cd src/
python main.py
```

> **前置条件**：`gesture_model.pkl`（手势分类模型）和 `best.pt`（YOLOv8 权重）需要提前放到运行目录。

## License

本项目为课程/竞赛实践作品，仅供学习参考。
