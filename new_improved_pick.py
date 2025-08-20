import pybullet as p
import pybullet_data
import time
import math

# —— 仿真初始化 —— #
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# —— 关节索引与类型检测 —— #
# 收集所有 Revolute/Prismatic 关节索引（总共 9 DOF），并去重
dof_indices = []
for i in range(p.getNumJoints(panda)):
    jt = p.getJointInfo(panda, i)[2]
    if jt in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
        dof_indices.append(i)
dof_indices = list(dict.fromkeys(dof_indices))

# 前 7 根是机械臂，后 2 根是夹爪
arm_joints = dof_indices[:7]   # [0,1,2,3,4,5,6]
finger_joints = dof_indices[7:]   # [9,10]

for j in arm_joints:
    p.setJointMotorControl2(
        bodyIndex = panda,
        jointIndex=j,
        controlMode=p.POSITION_CONTROL,
        targetPosition=0.0,
        force=200
    )
for _ in range(200):
    p.stepSimulation()
    time.sleep(1/240.)

# 打印确认
print("All DOF indices:", dof_indices)
print("Arm joints   :", arm_joints)
print("Gripper joints:", finger_joints)

# —— Null‐space 参数（长度=9）—— #
lower_limits, upper_limits, joint_ranges, joint_damping = [], [], [], []

for j in dof_indices:
    info = p.getJointInfo(panda, j)
    ll, ul = info[8], info[9]
    lower_limits.append(ll)
    upper_limits.append(ul)
    joint_ranges.append(ul - ll)
    joint_damping.append(0.1)

# 末端 link index 及“朝下”姿态
ee_index = 11
down_orientation = p.getQuaternionFromEuler([0, -math.pi, 0])

# —— 加载方块目标 —— #
cube_positions = [[0.6,  0.1, 0.02],
                  [0.5, -0.1, 0.02],
                  [0.55, 0.0, 0.02]]
cube_ids = [p.loadURDF("cube_small.urdf", pos) for pos in cube_positions]

# 等待物理稳定
for _ in range(200):
    p.stepSimulation()
    time.sleep(1/240.)

# 距离函数
def dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


# 动态选最近方块
def get_closest_cube():
    ee_pos = p.getLinkState(panda, ee_index)[0]
    return min(cube_ids, key=lambda cid: dist(p.getBasePositionAndOrientation(cid)[0], ee_pos))

# —— 带动态 null‐space 的运动函数 —— #
def move_to(target_pos, target_orn=down_orientation, steps=240):
    # 每次都把当前各 DOF 角度当 restPoses，避免把基座钉死
    rest_poses = [p.getJointState(panda, j)[0] for j in dof_indices]
    # 计算 IK（位置+姿态＋null-space＋阻尼）
    ik_sol = p.calculateInverseKinematics(
        bodyUniqueId=panda,
        endEffectorLinkIndex=ee_index,
        targetPosition=target_pos,
        targetOrientation=target_orn,
        lowerLimits=lower_limits,
        upperLimits=upper_limits,
        jointRanges=joint_ranges,
        restPoses=rest_poses,
        jointDamping=joint_damping,
    )
    # 前 7 个解映射给机械臂关节
    for _ in range(steps):
        for idx, j in enumerate(arm_joints):
            p.setJointMotorControl2(
                bodyIndex=panda,
                jointIndex=j,
                controlMode=p.POSITION_CONTROL,
                targetPosition=ik_sol[idx],
                force=200,
                maxVelocity=1.5
            )
        p.stepSimulation()
        time.sleep(1/240.)

# 夹爪开合（Prismatic）
def control_gripper(close=True):
    target = 0.01 if close else 0.04
    for j in finger_joints:
        p.setJointMotorControl2(
            bodyIndex=panda,
            jointIndex=j,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target,
            force=50
        )
    # 等夹爪运动到位
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1/240.)

# —— 抓取流程 —— #
# 1. 选最近方块，计算上方／抓取位置
closest = get_closest_cube()
pos = p.getBasePositionAndOrientation(closest)[0]
above = [pos[0], pos[1], pos[2] + 0.15]
grasp = [pos[0], pos[1], pos[2] + 0.015]
place = [0.2, -0.2, 0.1]

# 2. 可视化抓取方向
p.addUserDebugLine(grasp, [grasp[0], grasp[1], grasp[2]+0.1], [1,0,0], 3, 5)

# 3. 执行动作序列
print("[INFO] 移到上方")
move_to(above)
print("[INFO] 下探抓取")
move_to(grasp)
print("[INFO] 合拢抓爪")
control_gripper(True)
print("[INFO] 抬起搬运")
move_to(above)
print("[INFO] 移动放置点")
move_to(place)
print("[INFO] 张开抓爪放置")
control_gripper(False)

# —— 保持仿真 —— #
try:
    while True:
        p.stepSimulation()
        time.sleep(1/240.)
except KeyboardInterrupt:
    p.disconnect()
