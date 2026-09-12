# 能力与简历对应

| 简历能力 | 代码入口 | 对应验证 |
|---|---|---|
| 四关节几何 FK/IK、照射姿态 | `src/motion4dof/model.py`、`src/object_tracking.py` | 300 组往返、目标光轴对准、限位拒绝 |
| 关节轨迹与速度调节 | `src/motion4dof/model.py`、`src/serial_comm.py` | 五次插值、端点、速度约束、短写中断 |
| 四关节＋独立夹爪 | `src/motion4dof/protocol.py`、`firmware/Core/Inc/lamp_motion.h`、`contact.c` | 四角度帧与夹爪帧隔离，C 运动分发一致性 |
| 双摄视觉感知 | `src/yolo_class.py`、`src/csi1.py`、`src/gesture_class.py` | 部署入口与模型接口 |
| 四模式多线程调度 | `src/main.py` | 待机/普通/互动/写作入口 |
| ONNX 部署实验 | `training/yolo/identify_pi/main_onnx_pi.py`、`convert_to_onnx.py` | 属于 YOLO 部署工具，不等于手势分类器已经使用 ONNX |

当前台灯为四个运动关节加夹爪，共五个执行通道；夹爪与运动关节分别描述。完整测试与软硬件边界见 [四关节说明](four-axis-motion.md)。
