import sys
sys.path.append(".")

from robot.aubo_driver import AuboRobot
from control.action_decoder import ActionDecoder
from vla.fake_vla import FakeVLA


robot = AuboRobot()

robot.connect()


# get current robot pose
current_pose = robot.get_tcp_pose()

print("Current TCP:")
print(current_pose)


# VLA inference
model = FakeVLA()

action = model.predict()

print("VLA action:")
print(action.vector())


# decode action
decoder = ActionDecoder()

target_pose = decoder.delta_to_pose(
    current_pose,
    action.arm
)


print("Target pose:")
print(target_pose)


# execute
robot.move_pose(target_pose)

print("Motion command sent")
