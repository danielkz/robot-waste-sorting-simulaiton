import pybullet as p
import pybullet_data
import time
import math
import random
import os


# —— パラメータ & データ構造初期化 —— #
def init_robot():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")

    panda_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

    dof = [
        i for i in range(p.getNumJoints(panda_id))
        if p.getJointInfo(panda_id, i)[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)
    ]
    arm_joints = dof[:7]
    finger_joints = dof[7:]

    lowers = [p.getJointInfo(panda_id, j)[8] for j in dof]
    uppers = [p.getJointInfo(panda_id, j)[9] for j in dof]
    ranges = [u - l for l, u in zip(lowers, uppers)]
    damping = [0.1] * len(dof)

    ee_link = 11
    down_q = p.getQuaternionFromEuler([0, -math.pi, 0])

    return {
        "panda": panda_id,
        "dof": dof,
        "arm_joints": arm_joints,
        "finger_joints": finger_joints,
        "lower": lowers,
        "upper": uppers,
        "ranges": ranges,
        "damping": damping,
        "ee_link": ee_link,
        "down_ori": down_q
    }


# —— 夾爪開合函數 —— #
def control_gripper(panda, finger_joints, open=True):
    for j in finger_joints:
        ll, ul = p.getJointInfo(panda, j)[8], p.getJointInfo(panda, j)[9]
        tgt = ul if open else ll
        p.setJointMotorControl2(panda, j, p.POSITION_CONTROL,
                                targetPosition=tgt, force=50)
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1 / 240.)


# —— Null‐space IK 運動函數 —— #
def move_to(params, target_pos, target_ori=None, steps=240):
    if target_ori is None:
        target_ori = params["down_ori"]
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
    for _ in range(steps):
        for idx, j in enumerate(params["arm_joints"]):
            p.setJointMotorControl2(params["panda"], j, p.POSITION_CONTROL,
                                    targetPosition=ik[idx],
                                    force=200, maxVelocity=1.5)
        p.stepSimulation()
        time.sleep(1 / 240.)


# ==================== 新しい、より堅牢な関数 ====================
def spawn_waste_pile(num_cubes=40, include_phone=True):
    """
    指定された数の立方体と、必要に応じて1つのスマートフォンをスポーンし、
    物理エンジンで安定させる。
    """
    object_ids = []

    current_script_dir = os.path.dirname(os.path.abspath(__file__))

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    cube_urdf_path = "cube.urdf"

    phone_urdf_path_absolute = os.path.join(current_script_dir,
                                            "mobile_models",
                                            "urdf",
                                            "mobile_phone.urdf")

    # 立方体をランダムに配置
    print(f"[INFO] Spawning {num_cubes} cubes...")
    for i in range(num_cubes):
        # 立方体の生成範囲を狭くして、一箇所に集まるようにする
        pos_x = 1.3 + random.uniform(-0.1, 0.1)
        pos_y = 0.0 + random.uniform(-0.1, 0.1)
        pos_z = 0.5 + i * 0.05
        ori = p.getQuaternionFromEuler([0, 0, random.uniform(-math.pi, math.pi)])

        try:
            # globalScalingを0.05に設定
            obj_id = p.loadURDF(cube_urdf_path,
                                basePosition=[pos_x, pos_y, pos_z],
                                baseOrientation=ori,
                                useFixedBase=False,
                                globalScaling=0.05)
            object_ids.append(obj_id)
        except p.error as e:
            print(f"!!!!!!!!!! PyBullet Error !!!!!!!!!!!")
            print(f"Failed to load URDF: {cube_urdf_path}")
            print(f"Error message: {e}")
            print(f"Please check if the file exists in the pybullet_data directory.")
            print(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            return [], None

    # スマートフォンを配置
    target_phone_id = None
    if include_phone:
        print(f"[INFO] Spawning 1 phone...")
        # スマートフォンの生成範囲を立方体と同じにする
        pos_x = 1.3 + random.uniform(-0.1, 0.1)
        pos_y = 0.0 + random.uniform(-0.1, 0.1)
        # 立方体の高さに合わせて配置を調整
        pos_z = 0.5 + num_cubes * 0.0025
        ori = p.getQuaternionFromEuler(
            [random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5), random.uniform(-math.pi, math.pi)])

        try:
            target_phone_id = p.loadURDF(phone_urdf_path_absolute,
                                         basePosition=[pos_x, pos_y, pos_z],
                                         baseOrientation=ori,
                                         useFixedBase=False)
            object_ids.append(target_phone_id)
        except p.error as e:
            print(f"!!!!!!!!!! PyBullet Error !!!!!!!!!!!")
            print(f"Failed to load URDF: {phone_urdf_path_absolute}")
            print(f"Error message: {e}")
            print(f"Please check if the file exists at this path and if the URDF is valid.")
            print(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            return [], None

    print("[INFO] Waiting for objects to settle...")
    for _ in range(500):
        p.stepSimulation()
        time.sleep(1 / 240.)
    print("[INFO] Objects settled.")

    return object_ids, target_phone_id


# —— 主流程 —— #
def main():
    params = init_robot()

    all_objects, target_phone_id = spawn_waste_pile(num_cubes=40, include_phone=True)

    if not target_phone_id:
        print("Error: The target phone object was not loaded. Exiting.")
        p.disconnect()
        return

    control_gripper(params["panda"], params["finger_joints"], open=True)

    phone_pos, phone_ori = p.getBasePositionAndOrientation(target_phone_id)
    print(f"[INFO] Target phone position: {phone_pos}")

    above_pos = [phone_pos[0], phone_pos[1], phone_pos[2] + 0.15]
    grasp_pos = [phone_pos[0], phone_pos[1], phone_pos[2] + 0.015]

    place_pos = [0.2, -0.2, 0.1]

    print(f"[INFO] Moving above the phone (ID: {target_phone_id})...")
    move_to(params, above_pos)
    print("[INFO] Descending to grasp position...")
    move_to(params, grasp_pos)

    print("[INFO] Closing gripper and creating constraint...")
    control_gripper(params["panda"], params["finger_joints"], open=False)

    cid = p.createConstraint(
        parentBodyUniqueId=params["panda"],
        parentLinkIndex=params["ee_link"],
        childBodyUniqueId=target_phone_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0]
    )

    print("[INFO] Lifting and transporting...")
    move_to(params, above_pos)
    move_to(params, place_pos)

    print("[INFO] Releasing and opening gripper...")
    p.removeConstraint(cid)
    control_gripper(params["panda"], params["finger_joints"], open=True)

    try:
        while True:
            p.stepSimulation()
            time.sleep(1 / 240.)
    except KeyboardInterrupt:
        print("\n[INFO] Simulation interrupted. Disconnecting.")
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()