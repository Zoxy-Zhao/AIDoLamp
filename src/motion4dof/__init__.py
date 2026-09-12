"""Four positioning joints plus an independent gripper (five actuator channels)."""
from .model import Arm4, MotionError, trajectory
from .protocol import arm_command, gripper_command, SimReceiver
