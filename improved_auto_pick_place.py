import pybullet as p
import pybullet_data
import time, math

# —— 初始化 —— #
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# 末端 Link
ee_index = 11

# 生成小方块
cube_positions = [[0.6, 0.1, 0.02],
                  [0.5, -0.1, 0.02],
                  [0.55, 0.0, 0.02]]
cube_ids = [p.loadURDF("cube_small.urdf", pos) for pos in cube_positions]

# 等待物理稳定
for _ in range(200):
    p.stepSimulation()
    time.sleep(1/240)

# 距离函数
def dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

# 动态找最近的 cube
def get_closest_cube():
    ee_pos = p.getLinkState(panda, ee_index)[0]
    return min(cube_ids,
               key=lambda cid: dist(p.getBasePositionAndOrientation(cid)[0], ee_pos))

# 简化的运动函数（只用位置 IK，不带 null-space）
def move_to(pos, steps=240):
    ik_sol = p.calculateInverseKinematics(
        bodyUniqueId=panda,
        endEffectorLinkIndex=ee_index,
        targetPosition=pos,
        # 如果要加姿态约束，可以传入 targetOrientation
    )
    # 前 7 根 Revolute 关节的索引就是 0~6
    for _ in range(steps):
        for j in range(7):
            p.setJointMotorControl2(
                bodyIndex=panda,
                jointIndex=j,
                controlMode=p.POSITION_CONTROL,
                targetPosition=ik_sol[j],
                force=200
            )
        p.stepSimulation()
        time.sleep(1/240)

# 抓取流程
closest = get_closest_cube()
pos = p.getBasePositionAndOrientation(closest)[0]
above = [pos[0], pos[1], pos[2] + 0.15]
grasp = [pos[0], pos[1], pos[2] + 0.015]
place = [0.2, -0.2, 0.1]

# 可视化
p.addUserDebugLine(grasp, [grasp[0], grasp[1], grasp[2]+0.1], [1,0,0], 3, 5)

# 执行
print("Move above"); move_to(above)
print("Down to grasp"); move_to(grasp)
# ……然后按原来的 control_gripper、move_to(above)、move_to(place)、control_gripper(False)

# 保持
while True:
    p.stepSimulation()
    time.sleep(1/240)
