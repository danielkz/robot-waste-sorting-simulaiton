import pybullet as p
import pybullet_data
import time
import math

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# 加载多个目标 cube
target_positions = [[0.6, 0.1, 0.02], [0.4, -0.1, 0.02], [0.55, 0, 0.02]]
cube_ids = []
for pos in target_positions:
    cube_ids.append(p.loadURDF("cube_small.urdf", pos))

# 初始模拟几帧
for _ in range(100):
    p.stepSimulation()
    time.sleep(1. / 240)

# 获取 Panda 末端当前位姿
end_effector_index = 11
ee_state = p.getLinkState(panda, end_effector_index)
ee_pos = ee_state[0]

# 找出最近的 cube
def euclidean(p1, p2):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))

closest_cube = min(cube_ids, key=lambda cid: euclidean(p.getBasePositionAndOrientation(cid)[0], ee_pos))
target_pos = p.getBasePositionAndOrientation(closest_cube)[0]

# 设定抓取点（上方 + 抓取 + 提升 + 放置）
above = [target_pos[0], target_pos[1], target_pos[2] + 0.2]
grasp = [target_pos[0], target_pos[1], target_pos[2] + 0.01]
place = [0.2, 0.0, 0.1]

# 指定 orientation（夹爪朝下）
orn = p.getQuaternionFromEuler([0, -math.pi, 0])

# 移动到上方、抓取点、放置点时都加上 orientation
def move_to(pos, orn=orn, steps=100):
    joint_angles = p.calculateInverseKinematics(panda, end_effector_index, pos, orn)
    for _ in range(steps):
        for j in range(7):
            p.setJointMotorControl2(panda, j, p.POSITION_CONTROL, joint_angles[j])
        p.stepSimulation()
        time.sleep(1./240)


def control_gripper(close=True):
    val = 0.01 if close else 0.04
    p.setJointMotorControl2(panda, 9, p.POSITION_CONTROL, targetPosition=val, force=10)
    p.setJointMotorControl2(panda, 10, p.POSITION_CONTROL, targetPosition=val, force=10)
    for _ in range(60):
        p.stepSimulation()
        time.sleep(1./240)

# 自动流程：上方 → 抓 → 抬 → 放
move_to(above)
move_to(grasp)
control_gripper(close=True)
move_to(above)
move_to(place)
control_gripper(close=False)

# 停在最后姿势
while True:
    p.stepSimulation()
    time.sleep(1./240)


