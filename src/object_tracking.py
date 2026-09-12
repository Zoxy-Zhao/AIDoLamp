"""Four-joint tracking API; gripper is controlled independently by SerialCommunicator."""
import numpy as np
try:
    from .motion4dof import Arm4, trajectory
except ImportError:
    from motion4dof import Arm4, trajectory

class ObjectTracker:
    def __init__(self):
        self.arm = Arm4()

    def calculate_face_ik(self, target_pos):
        q = self.arm.face(target_pos)
        return np.radians(q[0]), np.radians(q[3]), self.arm.forward(q)[0]

    def face_forward_kinematics(self, base_angle, wrist_angle):
        return self.arm.points([np.degrees(base_angle),60,30,np.degrees(wrist_angle)])

    def calculate_book_ik(self, target_pos, distance=30):
        q = self.arm.book(target_pos, distance)
        return (*np.radians(q), self.arm.forward(q)[0])

    def book_forward_kinematics(self, base_angle, shoulder_angle, elbow_angle, wrist_angle, end_effector_pos=None):
        # Always compute actual FK; never replace the endpoint with a requested target.
        return self.arm.points(np.degrees([base_angle,shoulder_angle,elbow_angle,wrist_angle]))

    def generate_smooth_trajectory(self, start_angles, end_angles, steps=20):
        _, q = trajectory(self.arm,np.degrees(start_angles),np.degrees(end_angles),steps=steps)
        return [tuple(row) for row in np.radians(q)]

    def angles_to_degrees(self, *angles):
        return np.degrees(angles).tolist()

    def track_face(self, face_coordinates):
        return self.calculate_face_ik(face_coordinates)[:2]

    def track_book(self, book_coordinates, distance=30):
        return self.calculate_book_ik(book_coordinates,distance)[:4]

    def plot_robotic_arm(self, joint_positions, target_pos=None, end_effector_pos=None, title="机械臂姿态可视化"):
        import matplotlib.pyplot as plt
        # 转换为numpy数组方便处理
        positions = np.array(joint_positions)
        
        # 提取各关节坐标
        base = positions[0]
        shoulder = positions[1]
        elbow = positions[2]
        wrist = positions[3]
        end = positions[4]
        
        # 创建3D图形对象
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制连杆结构
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2],
                'o-', markersize=8, linewidth=3, color='#2E86C1')
        
        # 绘制各关节标记
        joint_labels = ['底座', '肩部', '肘部', '腕部', '末端']
        for i, (x, y, z) in enumerate(positions):
            ax.scatter(x, y, z, s=100, label=joint_labels[i])
        
        # 绘制摄像头位置（末端执行器中心）
        if end_effector_pos is not None:
            ax.scatter(end_effector_pos[0], end_effector_pos[1], end_effector_pos[2], 
                       color='blue', marker='^', s=100, label='摄像头')
        
        # 添加目标点显示
        if target_pos is not None:
            ax.scatter(target_pos[0], target_pos[1], target_pos[2],
                       s=150, c='r', marker='*', label='目标')
            
        # 绘制从末端到目标的连线(红色实线)
        if end_effector_pos is not None and target_pos is not None:
            ax.plot([end_effector_pos[0], target_pos[0]],
                    [end_effector_pos[1], target_pos[1]],
                    [end_effector_pos[2], target_pos[2]],
                    '-', color='red', linewidth=2)
            # 绘制腕部到末端的连线(蓝色实线)
            wrist = positions[3]  # 从joint_positions中提取腕部位置
            ax.plot([wrist[0], end_effector_pos[0]],
                    [wrist[1], end_effector_pos[1]],
                    [wrist[2], end_effector_pos[2]],
                    '-', color='blue', linewidth=2)
                
            # 显示距离
            distance = np.linalg.norm(np.array(end_effector_pos) - np.array(target_pos))
            mid_point = (np.array(end_effector_pos) + np.array(target_pos)) / 2
            ax.text(mid_point[0], mid_point[1], mid_point[2], 
                    f'{distance:.1f}cm', color='red', fontsize=10)
        
        # 绘制机械臂坐标系
        origin = np.zeros(3)
        ax.quiver(origin[0], origin[1], origin[2], 15, 0, 0, color='r', arrow_length_ratio=0.1, label='X轴')
        ax.quiver(origin[0], origin[1], origin[2], 0, 15, 0, color='g', arrow_length_ratio=0.1, label='Y轴')
        ax.quiver(origin[0], origin[1], origin[2], 0, 0, 15, color='b', arrow_length_ratio=0.1, label='Z轴')
        
        # 设置坐标轴参数
        max_range = max(L2, L3) * 2
        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Y (cm)')
        ax.set_zlabel('Z (cm)')
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([0, L1 + L2 + L3 + L4 + 10])
        
        # 添加桌面平面（z=0）
        xx, yy = np.meshgrid(np.linspace(-max_range, max_range, 2), 
                             np.linspace(-max_range, max_range, 2))
        zz = np.zeros(xx.shape)
        ax.plot_surface(xx, yy, zz, alpha=0.2, color='gray')
        
        # 设置观察视角（俯仰角，方位角）
        ax.view_init(elev=30, azim=45)
        
        # 添加图例和标题
        plt.legend(loc='upper left', bbox_to_anchor=(0.9, 0.9))
        plt.title(f"{title}\n底座到末端长度：{np.linalg.norm(end - base):.1f}cm")
        plt.tight_layout()
        plt.show()
