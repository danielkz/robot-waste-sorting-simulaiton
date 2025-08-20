import pybullet as p
import pybullet_data
import time
import math
import random

# —— 仿真初始化 —— #
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# —— 收集可动关节（7 臂 + 2 指） —— #
dof_indices = []
for i in range(p.getNumJoints(panda)):
    jt = p.getJointInfo(panda, i)[2]
    if jt in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
        dof_indices.append(i)
# 去重保序
dof_indices = list(dict.fromkeys(dof_indices))
arm_joints = dof_indices[:7]
finger_joints = dof_indices[7:]

# —— Null‐space IK 参数准备 —— #
lower_limits, upper_limits, joint_ranges, joint_damping = [], [], [], []
for j in dof_indices:
    info = p.getJointInfo(panda, j)
    ll, ul = info[8], info[9]
    lower_limits.append(ll)
    upper_limits.append(ul)
    joint_ranges.append(ul - ll)
    joint_damping.append(0.1)

ee_index        = 11  # 末端链接
down_orientation = p.getQuaternionFromEuler([0, -math.pi, 0])

# —— 夹爪控制 —— #
def control_gripper(open=True):
    # open=True → 张开到上限；open=False → 闭合到下限
    for j in finger_joints:
        info = p.getJointInfo(panda, j)
        ll, ul = info[8], info[9]
        tgt = ul if open else ll
        p.setJointMotorControl2(panda, j, p.POSITION_CONTROL, targetPosition=tgt, force=50)
    # 等一会儿
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1/240.)

# —— 运动到指定末端位置（带 Null‐space） —— #
def move_to(target_pos, target_orn=down_orientation, steps=240):
    rest_poses = [p.getJointState(panda, j)[0] for j in dof_indices]
    ik_sol = p.calculateInverseKinematics(
        panda, ee_index,
        target_pos, target_orn,
        lowerLimits=lower_limits,
        upperLimits=upper_limits,
        jointRanges=joint_ranges,
        restPoses=rest_poses,
        jointDamping=joint_damping
    )
    # 前 7 个是机械臂
    for _ in range(steps):
        for idx, j in enumerate(arm_joints):
            p.setJointMotorControl2(panda, j, p.POSITION_CONTROL,
                                    targetPosition=ik_sol[idx],
                                    force=200, maxVelocity=1.5)
        p.stepSimulation()
        time.sleep(1/240.)

# —— 随机生成小方块 —— #
cube_ids = []
for layer in range(4):
    z = 0.02 + layer * 0.03
    for _ in range(20):
        pos = [
            1.3 + random.uniform(-0.05, 0.05),
            0.0 + random.uniform(-0.05, 0.05),
            z
        ]
        cube_ids.append(p.loadURDF("cube_small.urdf", pos))

# 等待稳定
for _ in range(200):
    p.stepSimulation()
    time.sleep(1/240.)

# —— 开始抓取流程 —— #
# 1. 打开夹爪
control_gripper(open=True)

# 2. 选最近方块
def dist(a, b):
    return math.dist(a, b)

def get_closest_cube():
    ee_pos = p.getLinkState(panda, ee_index)[0]
    return min(cube_ids,
               key=lambda cid: dist(p.getBasePositionAndOrientation(cid)[0], ee_pos))


closest = get_closest_cube()
pos = p.getBasePositionAndOrientation(closest)[0]
above = [pos[0], pos[1], pos[2] + 0.15]
grasp = [pos[0], pos[1], pos[2] + 0.015]
place = [0.2, -0.2, 0.1]

# 可動関節だけ抽出（REVOLUTE＝0 または PRISMATIC＝1）
dof_indices = []
for i in range(p.getNumJoints(panda)):
    info = p.getJointInfo(panda, i)
    joint_type = info[2]
    if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
        dof_indices.append(i)

# 各関節パラメータをリスト化
lower_limits = []
upper_limits = []
joint_ranges = []
joint_damping = []

for j in dof_indices:
    info = p.getJointInfo(panda, j)
    ll = info[8]
    ul = info[9]
    lower_limits.append(ll)
    upper_limits.append(ul)
    joint_ranges.append(ul - ll)
    joint_damping.append(0.1)  # 一般的な値


# 3. 抓取动作
print("[INFO] 移到上方")
move_to(above)
print("[INFO] 下探抓取")
move_to(grasp)

print("[INFO] 闭合夹爪并附加约束")
control_gripper(open=False)
# ↳ 增加刚性约束，确保方块随夹爪一起动
cid = p.createConstraint(
    parentBodyUniqueId=panda,
    parentLinkIndex=ee_index,
    childBodyUniqueId=closest,
    childLinkIndex=-1,
    jointType=p.JOINT_FIXED,
    jointAxis=[0,0,0],
    parentFramePosition=[0,0,0],
    childFramePosition=[0,0,0]
)

print("[INFO] 抬起搬运")
move_to(above)

print("[INFO] 移动放置点")
move_to(place)

print("[INFO] 删除约束并放开夹爪")
p.removeConstraint(cid)
control_gripper(open=True)

# —— 保持仿真 —— #
try:
    while True:
        p.stepSimulation()
        time.sleep(1/240.)
except KeyboardInterrupt:
    p.disconnect()
