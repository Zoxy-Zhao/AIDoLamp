# 四轴实物原型：运动控制与视觉实现说明

本文描述 `src/main.py` 及其四轴原型配套模块，便于从运动学、轨迹执行和视觉调度三个方向阅读项目。新增的独立六轴算法演示见 [six-axis-motion.md](six-axis-motion.md)。代码路径说明不等同于真机验证报告。

## 运动学与照射姿态

[ObjectTracker](../src/object_tracking.py) 使用底座、肩、肘、腕四个关节，连杆参数为 8、24、24、8 cm，内部角度单位为弧度。

- 人脸跟踪：`calculate_face_ik` 固定肩部 60°、肘部 30°，计算底座朝向及腕部角度，减少求解变量。
- 书本跟踪：`calculate_book_ik` 结合目标上方偏移、朝基座方向的偏移及 40–50 cm 距离参数构造照射目标，将肩肘求解转化为平面几何问题，以余弦定理计算关节角。
- 可达范围：对腕部目标进行连杆长度范围修正，再对关节角执行限位裁剪。该处理不保证限位后的末端精确满足原始位置和朝向。
- 正运动学：提供人脸和书本场景的关节位置计算及绘图工具。书本绘图接口允许直接传入目标末端位置，验证实际 FK 误差时应使用关节角独立计算末端位置。

四轴原型的 `ObjectTracker` 不包含六个独立关节的运动学模型；六轴算法位于独立的 `src/motion6dof/` 中。串口解析器可容纳的参数数量不代表机械臂自由度。

## 从视觉结果到舵机执行

```text
摄像头 0 图像
    → YOLO 人脸 / 书本检测
    → 目标坐标估计
    → 场景对应的几何 IK
    → 弧度转换为角度
    → 四关节线性插值
    → UART Alldro 指令
    → STM32 命令解析
    → PCA9685 四路舵机控制
```

[main.py](../src/main.py) 分别调用 `track_face` 与 `track_book`，随后调用 [SerialComm.send_servos_smooth](../src/serial_comm.py)。该接口以 `duration / steps` 设置发送间隔，从已记录的角度向目标角度进行线性插值，并发送 `Alldro` 加四个角度的命令。[contact.c](../firmware/Core/Src/contact.c) 将四个参数分发到舵机通道 0–3。

`ObjectTracker.generate_smooth_trajectory` 另外提供余弦插值，但主程序中的跟踪链路没有调用该工具。当前执行链路不应描述为已接入余弦速度曲线、加速度约束或停稳检测。

## 双摄与推理模块

| 模块 | 当前代码路径 | 实现内容 |
|------|--------------|----------|
| 目标检测 | [yolo_class.py](../src/yolo_class.py) | Ultralytics YOLO，默认加载 `best.pt`，使用摄像头 0 |
| 手势识别 | [gesture_class.py](../src/gesture_class.py) | MediaPipe 关键点 + `gesture_model.pkl` 分类器，使用摄像头 1 |
| 手势与坐姿联合处理 | [csi1.py](../src/csi1.py) | 写作模式中组织摄像头 1 的手势和姿态分析 |
| 模式和线程组织 | [main.py](../src/main.py) | 四模式状态切换、视觉跟踪线程与语音任务 |
| YOLOv8n 训练 | [train.py](../training/yolo/identify_pi/train.py) | 以 `yolov8n.pt` 初始化训练 |
| ONNX 推理实验 | [main_onnx_pi.py](../training/yolo/identify_pi/main_onnx_pi.py) | ONNX Runtime CPU 推理会话 |
| 导出与量化工具 | [convert_to_onnx.py](../training/yolo/identify_pi/convert_to_onnx.py) | 输入尺寸配置、模型导出与量化工具 |

双 CSI 摄像头承担不同感知任务，此处的“双摄”不表示已经实现双目立体测距。ONNX 推理实验也不表示主程序或手势分类器已经切换到该后端。

## 实现范围与验证边界

- **关节数量**：本页所述原型 IK、上位机角度接口及下位机执行分支对应四轴。新增六轴算法尚未接入该执行链路。
- **位置估计**：[yolo_class.py](../src/yolo_class.py) 结合视场角、距离输入或物体尺寸估计生成坐标，当前跟踪路径未包含完整的标定外参变换。
- **关节状态**：`current_angles` 保存已发送的插值角度，不能用作实际落位误差或停稳状态的测量依据。
- **跟踪节奏**：主程序的跟踪线程包含 3 秒插值调用及随后 3 秒等待；模型推理延迟不能直接代表机械臂跟踪周期。
- **性能口径**：模型推理耗时与包含图像采集、模式调度及执行的端到端延迟应分别统计；本文未报告性能基准数据。
- **验证范围**：本文根据源代码核对编写，没有开展本轮树莓派、双摄和 STM32 真机联调。
