import pybullet as p
import pybullet_data
import time
import math
import random
import os

# —— 参数 & 数据结构初始化 —— #
def init_robot():
    # 1) 连接 GUI，加载地面和平面模型
    p.connect(p.GUI) # 连接 PyBullet 的图形用户界面。p.GUI 表示启动一个可视化窗口，方便观察模拟过程。
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) # 设置 PyBullet 寻找模型文件的额外路径。pybullet_data.getDataPath() 返回 PyBullet 官方提供的一些通用模型（如 plane.urdf, cube_small.urdf, franka_panda/panda.urdf）所在的路径。
    p.setGravity(0, 0, -9.8) # 设置重力，这里是地球上的重力加速度，方向沿着 Z 轴负方向。
    p.loadURDF("plane.urdf") # 加载一个简单的平面模型作为地面。

    # 2) 加载 Panda（固定底座）
    panda_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True) # 加载 Franka Panda 机器人模型。
    # useFixedBase=True 意味着机器人的底座（基座）是固定的，不会受到重力或其他力的影响而移动。这在很多工业机器人模拟中是常见的，因为它通常被固定在工作台上。

    # 3) 一次性提取所有可动关节
    dof = [
        i for i in range(p.getNumJoints(panda_id)) # 遍历机器人所有关节的索引。
        if p.getJointInfo(panda_id, i)[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC) # 检查关节类型。
        # p.getJointInfo(panda_id, i) 返回一个包含关节信息的元组，索引 2 表示关节类型。
        # p.JOINT_REVOLUTE 是旋转关节（如机械臂的各个轴）。
        # p.JOINT_PRISMATIC 是平移关节（如一些直线滑轨或伸缩臂，Panda 机器人中通常没有这类关节，但这里是通用写法）。
        # dof (Degrees Of Freedom) 将包含所有可动关节的索引。
    ]
    arm_joints    = dof[:7]    # 前 7 个是机械臂的关节。Panda 机器人有 7 个自由度（DOF）的机械臂。
    finger_joints = dof[7:]    # 后 2 个是手爪的关节。Panda 夹爪通常由两个关节控制开合。

    # 4) 关节极限 & 范围 & 阻尼
    lowers  = [p.getJointInfo(panda_id, j)[8] for j in dof] # 获取每个可动关节的下限（最小关节角度/位置）。
    uppers  = [p.getJointInfo(panda_id, j)[9] for j in dof] # 获取每个可动关节的上限（最大关节角度/位置）。
    ranges  = [u - l for l, u in zip(lowers, uppers)] # 计算每个关节的运动范围（上限 - 下限）。
    damping = [0.1] * len(dof) # 设置每个关节的阻尼系数。阻尼可以帮助稳定关节运动，防止过度振荡。

    # 末端连杆、默认朝下姿态
    ee_link = 11 # 末端执行器（End-Effector）的连杆索引。在 Panda 机器人模型中，索引 11 通常是夹爪的连接点。
    down_q  = p.getQuaternionFromEuler([0, -math.pi, 0]) # 定义一个四元数，表示末端执行器朝下的姿态。
    # p.getQuaternionFromEuler([roll, pitch, yaw]) 将欧拉角转换为四元数。
    # [0, -math.pi, 0] 意味着绕 Y 轴旋转 -180 度（-π 弧度），使末端执行器指向下方。

    return { # 返回一个字典，包含了所有初始化后的机器人参数，方便后续函数调用。
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
    for j in finger_joints: # 遍历夹爪的每个关节。
        ll, ul = p.getJointInfo(panda, j)[8], p.getJointInfo(panda, j)[9] # 获取夹爪关节的下限和上限。
        tgt = ul if open else ll # 如果 open 为 True，目标位置设为上限（张开）；否则设为下限（闭合）。
        p.setJointMotorControl2(panda, j, p.POSITION_CONTROL, # 控制关节。
                                 targetPosition=tgt, force=50) # 设置目标位置和施加的力。POSITION_CONTROL 模式会尝试将关节移动到目标位置。
    # 等抓爪动作完成
    for _ in range(100): # 循环 100 次，让模拟器进行步进，给夹爪时间完成动作。
        p.stepSimulation() # 模拟器向前推进一步。
        time.sleep(1/240.) # 暂停一小段时间，以 240Hz 的频率进行可视化。

# —— Null‐space IK 运动函数 —— #
def move_to(params, target_pos, target_ori=None, steps=240):
    if target_ori is None: # 如果没有指定目标姿态，则使用默认的朝下姿态。
        target_ori = params["down_ori"]
    # 休息位
    rest_poses = [p.getJointState(params["panda"], j)[0] for j in params["dof"]] # 获取当前所有关节的姿态作为“休息位”。
    # 休息位 (restPoses) 在逆运动学计算中用于解决冗余自由度问题。当末端执行器达到目标位置和姿态时，机械臂可能有多种关节配置。restPoses 会引导 IK 求解器找到一个尽可能接近这个“休息位”的配置。
    ik = p.calculateInverseKinematics( # 计算逆运动学。
        params["panda"], params["ee_link"], # 机器人 ID 和末端执行器连杆 ID。
        target_pos, target_ori, # 目标位置和姿态（四元数）。
        lowerLimits=params["lower"], # 关节下限。
        upperLimits=params["upper"], # 关节上限。
        jointRanges=params["ranges"], # 关节运动范围。
        restPoses=rest_poses, # 休息位。
        jointDamping=params["damping"] # 关节阻尼。
    )
    # 只控制臂部 7 关节
    for _ in range(steps): # 循环 steps 次，逐步将机器人移动到目标位置。
        for idx, j in enumerate(params["arm_joints"]): # 遍历机械臂的 7 个关节。
            p.setJointMotorControl2(params["panda"], j, p.POSITION_CONTROL,
                                    targetPosition=ik[idx], # 将 IK 计算出的目标关节角度设置为控制目标。
                                    force=200, maxVelocity=1.5) # 施加力和最大速度，使关节平滑移动。
        p.stepSimulation()
        time.sleep(1/240.)

def spawn_mobile_phone(pos=[0, 0, 0.1], ori=[0, 0, 0, 1]):
    # 获取当前脚本的目录
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # 构建到 mobile_phone_description 文件夹的路径
    # 假设 mobile_phone_description 文件夹与您的 Python 脚本在同一层
    model_path = os.path.join(current_dir, "mobile_phone_description")

    # PyBullet 需要知道 'package://' 指向哪里
    p.setAdditionalSearchPath(model_path) # 将模型的根目录添加到搜索路径

    # 加载 URDF
    phone_id = p.loadURDF("mobile_phone_description/urdf/mobile_phone.urdf",
                          basePosition=pos,
                          baseOrientation=ori,
                          useFixedBase=False) # 垃圾应该是不固定的

    return phone_id


# —— 随机生成小方块 —— #
def spawn_cubes():
    ids = []
    for layer in range(4): # 生成 4 层。
        z = 0.02 + layer * 0.03 # 根据层数计算 Z 坐标，模拟堆叠。
        for _ in range(20): # 每层生成 20 个方块。
            pos = [1.3 + random.uniform(-0.05, 0.05), # 在 X 轴 1.3 附近随机分布。
                   0.0 + random.uniform(-0.05, 0.05), z] # 在 Y 轴 0.0 附近随机分布。
            ids.append(p.loadURDF("cube_small.urdf", pos)) # 加载小方块模型并设置位置。
    # 等稳定
    for _ in range(200): # 循环 200 次，让方块在重力作用下稳定下来（例如，从高处掉落）。
        p.stepSimulation()
        time.sleep(1/240.)
    return ids

# —— 主流程 —— #
def main():
    params = init_robot() # 初始化机器人参数。
    cubes  = spawn_cubes() # 生成小方块。

    # 1) 张开手爪
    control_gripper(params["panda"], params["finger_joints"], open=True) # 张开夹爪，准备抓取。

    # 2) 选最近方块
    def dist(a, b): return math.dist(a, b) # 计算两个三维点之间的距离。
    def closest_cube():
        ee_pos = p.getLinkState(params["panda"], params["ee_link"])[0] # 获取末端执行器的当前位置。
        return min(cubes,
                   key=lambda cid: dist(p.getBasePositionAndOrientation(cid)[0], ee_pos)) # 从所有方块中找到距离末端执行器最近的那个。

    blk = closest_cube() # 选定要抓取的方块。
    pos = p.getBasePositionAndOrientation(blk)[0] # 获取选定方块的当前位置。
    above = [pos[0], pos[1], pos[2] + 0.15] # 定义一个在方块上方 0.15 米的位置，作为抓取前的预备位置。
    grasp = [pos[0], pos[1], pos[2] + 0.015] # 定义抓取位置，略高于方块，以便夹爪能够闭合。
    place = [0.2, -0.2, 0.1] # 定义放置位置。

    # 3) 抓取－搬运－放下
    print("[INFO] 移到上方")
    move_to(params, above) # 机器人移动到方块上方。
    print("[INFO] 下探抓取")
    move_to(params, grasp) # 机器人下探到抓取位置。

    print("[INFO] 闭爪＋加约束")
    control_gripper(params["panda"], params["finger_joints"], open=False) # 闭合夹爪。
    cid = p.createConstraint( # 创建一个固定约束 (fixed constraint)。
        parentBodyUniqueId=params["panda"], # 父物体是机器人。
        parentLinkIndex=params["ee_link"], # 父连杆是末端执行器。
        childBodyUniqueId=blk, # 子物体是被抓取的方块。
        childLinkIndex=-1, # -1 表示子物体的基座（整个物体）。
        jointType=p.JOINT_FIXED, # 约束类型是固定，即被抓取的物体会像被“粘”在末端执行器上一样。
        jointAxis=[0,0,0], # 轴，固定约束通常不关心轴，但需要占位。
        parentFramePosition=[0,0,0], # 子物体相对于父连杆的偏移。
        childFramePosition=[0,0,0] # 父连杆相对于子物体的偏移。这里设为 [0,0,0] 意味着在当前接触点直接固定。
    )
    # 这个约束是模拟“成功抓取”的关键。它让被抓取的物体与机器人的末端执行器一起运动，而不会掉落。

    print("[INFO] 抬起＋搬运")
    move_to(params, above) # 抬起机器人。
    move_to(params, place) # 搬运到放置位置。

    print("[INFO] 放下＋开爪")
    p.removeConstraint(cid) # 移除之前添加的固定约束，被抓取的方块将重新受到重力等物理效应的影响。
    control_gripper(params["panda"], params["finger_joints"], open=True) # 张开夹爪，释放物体。

    # —— 无限循环，直到你按 Ctrl+C —— #
    try:
        while True:
            p.stepSimulation()
            time.sleep(1/240.)
    except KeyboardInterrupt: # 捕获键盘中断异常 (Ctrl+C)。
        print("\n[INFO] 已中断，断开连接。")
    finally: # 无论是否发生异常，最后都会执行这里的代码。
        p.disconnect() # 断开 PyBullet 连接，关闭模拟器窗口。

if __name__ == "__main__":
    main() # 当脚本作为主程序运行时，执行 main 函数。