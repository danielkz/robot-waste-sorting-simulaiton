import pybullet as p
import pybullet_data
import time
import math
import random

# —— 参数 & 数据结构初始化 —— #
def init_robot():
    # 1) 连接 GUI，加载地面和平面模型
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")

    # 2) 加载 Panda（固定底座）
    panda_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

    # 3) 一次性提取所有可动关节
    dof = [
        i for i in range(p.getNumJoints(panda_id))
        if p.getJointInfo(panda_id, i)[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)
    ]
    arm_joints    = dof[:7]   # 前 7 个是机械臂
    finger_joints = dof[7:]   # 后 2 个是手爪

    # 4) 关节极限 & 范围 & 阻尼
    lowers  = [p.getJointInfo(panda_id, j)[8] for j in dof]
    uppers  = [p.getJointInfo(panda_id, j)[9] for j in dof]
    ranges  = [u - l for l, u in zip(lowers, uppers)]
    damping = [0.1] * len(dof)

    # 末端连杆、默认朝下姿态
    ee_link = 11
    down_q  = p.getQuaternionFromEuler([0, -math.pi, 0])

    return {
        "panda":       panda_id,
        "dof":         dof,
        "arm_joints":  arm_joints,
        "finger_joints": finger_joints,
        "lower":       lowers,
        "upper":       uppers,
        "ranges":      ranges,
        "damping":     damping,
        "ee_link":     ee_link,
        "down_ori":    down_q
    }

# —— 夹爪开合函数 —— #
def control_gripper(panda, finger_joints, open=True):
    for j in finger_joints:
        ll, ul = p.getJointInfo(panda, j)[8], p.getJointInfo(panda, j)[9]
        tgt = ul if open else ll
        p.setJointMotorControl2(panda, j, p.POSITION_CONTROL,
                                targetPosition=tgt, force=50)
    # 等抓爪动作完成
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1/240.)

# —— Null‐space IK 运动函数 —— #
def move_to(params, target_pos, target_ori=None, steps=240):
    if target_ori is None:
        target_ori = params["down_ori"]
    # 休息位
    rest_poses = [p.getJointState(params["panda"], j)[0] for j in params["dof"]]
    ik = p.calculateInverseKinematics(
        params["panda"], params["ee_link"],
        target_pos, target_ori,
        lowerLimits=params["lower"],
        upperLimits=params["upper"],
        jointRanges=params["ranges"],
        restPoses=rest_poses,
        jointDamping=params["damping"]
    )
    # 只控制臂部 7 关节
    for _ in range(steps):
        for idx, j in enumerate(params["arm_joints"]):
            p.setJointMotorControl2(params["panda"], j, p.POSITION_CONTROL,
                                    targetPosition=ik[idx],
                                    force=200, maxVelocity=1.5)
        p.stepSimulation()
        time.sleep(1/240.)

# —— 随机生成小方块 —— #
def spawn_cubes():
    ids = []
    for layer in range(4):
        z = 0.02 + layer * 0.03
        for _ in range(20):
            pos = [1.3 + random.uniform(-0.05, 0.05),
                   0.0 + random.uniform(-0.05, 0.05), z]
            ids.append(p.loadURDF("cube_small.urdf", pos))
    # 等稳定
    for _ in range(200):
        p.stepSimulation()
        time.sleep(1/240.)
    return ids

# —— 主流程 —— #
def main():
    params = init_robot()
    cubes  = spawn_cubes()

    # 1) 张开手爪
    control_gripper(params["panda"], params["finger_joints"], open=True)

    # 2) 选最近方块
    def dist(a, b): return math.dist(a, b)
    def closest_cube():
        ee_pos = p.getLinkState(params["panda"], params["ee_link"])[0]
        return min(cubes,
                   key=lambda cid: dist(p.getBasePositionAndOrientation(cid)[0], ee_pos))

    blk = closest_cube()
    pos = p.getBasePositionAndOrientation(blk)[0]
    above = [pos[0], pos[1], pos[2] + 0.15]
    grasp = [pos[0], pos[1], pos[2] + 0.015]
    place = [0.2, -0.2, 0.1]

    # 3) 抓取－搬运－放下
    print("[INFO] 移到上方")
    move_to(params, above)
    print("[INFO] 下探抓取")
    move_to(params, grasp)

    print("[INFO] 闭爪＋加约束")
    control_gripper(params["panda"], params["finger_joints"], open=False)
    cid = p.createConstraint(
        parentBodyUniqueId=params["panda"],
        parentLinkIndex=params["ee_link"],
        childBodyUniqueId=blk,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0,0,0],
        parentFramePosition=[0,0,0],
        childFramePosition=[0,0,0]
    )

    print("[INFO] 抬起＋搬运")
    move_to(params, above)
    move_to(params, place)

    print("[INFO] 放下＋开爪")
    p.removeConstraint(cid)
    control_gripper(params["panda"], params["finger_joints"], open=True)

    # —— 无限循环，直到你按 Ctrl+C —— #
    try:
        while True:
            p.stepSimulation()
            time.sleep(1/240.)
    except KeyboardInterrupt:
        print("\n[INFO] 已中断，断开连接。")
    finally:
        p.disconnect()

if __name__ == "__main__":
    main()
