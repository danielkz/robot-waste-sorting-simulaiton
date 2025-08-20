"""
Franka Panda が 20 個の白ブロックと 1 個の赤ブロックを生成し、
カメラで赤のみ検出 → IK で下降 → 摩擦グリップで把持 → 移載する最小デモ
"""

import pybullet as p, pybullet_data as pd
import numpy as np, cv2, math, time, random

# ---------------- 1. シミュレーション初期化 ----------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

arm = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
dof = [i for i in range(p.getNumJoints(arm))
       if p.getJointInfo(arm, i)[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)]
arm_j, finger_j = dof[:7], dof[7:]
ee_index = 11
down_orn = p.getQuaternionFromEuler([0, -math.pi, 0])

# ---- IK パラメータ
ll, ul, jr, jd = [], [], [], []
for j in dof:
    info = p.getJointInfo(arm, j)
    ll.append(info[8]); ul.append(info[9])
    jr.append(ul[-1] - ll[-1]); jd.append(0.1)

# ---------------- 2. グリッパ制御 ----------------
def grip(gap):                # gap=0.0 閉、0.04 開（m）
    for j in finger_j:
        p.setJointMotorControl2(arm, j, p.POSITION_CONTROL,
                                targetPosition=gap/2, force=120)
    for _ in range(120):      # ホールド待ち
        p.stepSimulation(); time.sleep(1/480)

# ---------------- 3. ゆっくり IK 移動 ----------------
def move(pos, orn=down_orn, steps=480, vel=0.6):
    rest = [p.getJointState(arm, j)[0] for j in dof]
    sol = p.calculateInverseKinematics(
        arm, ee_index, pos, orn, ll, ul, jr, rest, jd)
    for _ in range(steps):
        for idx, j in enumerate(arm_j):
            p.setJointMotorControl2(arm, j, p.POSITION_CONTROL,
                                    sol[idx], force=200, maxVelocity=vel)
        p.stepSimulation(); time.sleep(1/480)

# ---------------- 4. ブロック生成 ----------------
box_half = [0.04, 0.02, 0.015]
col_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=box_half)
vis_white = p.createVisualShape(p.GEOM_BOX, halfExtents=box_half,
                                rgbaColor=[0.9, 0.9, 0.9, 1])
vis_red   = p.createVisualShape(p.GEOM_BOX, halfExtents=box_half,
                                rgbaColor=[0.9, 0.1, 0.1, 1])

white_ids = []
for _ in range(20):
    pos = [1.0 + random.uniform(-0.05, 0.05),
           0.0 + random.uniform(-0.05, 0.05), 0.03]
    white_ids.append(p.createMultiBody(0.02, col_shape, vis_white, pos))

red_pos = [1.0 + random.uniform(-0.05, 0.05),
           0.0 + random.uniform(-0.05, 0.05), 0.03]
red_id  = p.createMultiBody(0.02, col_shape, vis_red,  red_pos)

# 指とブロックの摩擦係数を上げる
for fid in finger_j:
    p.changeDynamics(arm, fid, lateralFriction=1.2)
p.changeDynamics(red_id, -1, lateralFriction=1.0)

for _ in range(240): p.stepSimulation()     # 落下待ち

# ---------------- 5. カメラ設定 ----------------
W = H = 400
eye, tgt = [1.0, 0, 1.1], [1.0, 0, 0]
view = np.array(p.computeViewMatrix(eye, tgt, [0,1,0])).reshape(4,4)
proj = np.array(p.computeProjectionMatrixFOV(60, 1, 0.1, 2)).reshape(4,4)
invPV = np.linalg.inv(proj @ view)

# ---------------- 6. メインループ（赤 1 個だけ把持） ----------------
while True:
    _, _, rgb, _, _ = p.getCameraImage(W, H, view, proj,
                                       renderer=p.ER_BULLET_HARDWARE_OPENGL)
    img = np.reshape(rgb, (H, W, 4))[:, :, :3]

    hsv  = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (0,120,70), (10,255,255))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        c  = max(contours, key=cv2.contourArea)
        M  = cv2.moments(c); cx = int(M['m10']/M['m00']); cy = int(M['m01']/M['m00'])

        # --- ピクセル → ワールド（床 z=0.03） ---
        ndc = np.array([(cx/W-0.5)*2, -(cy/H-0.5)*2, 1, 1])
        ws  = invPV @ ndc; ws /= ws[3]
        wx, wy = ws[0], ws[1]

        above  = [wx, wy, 0.15]
        grasp  = [wx, wy, 0.03]
        place  = [0.4, -0.3, 0.15]

        # 把持シーケンス（剛性拘束なし）
        grip(0.05); move(above); move(grasp)
        grip(0.00)               # 指を完全閉
        time.sleep(0.5)          # 摩擦で保持させる待ち
        move(above); move(place)
        grip(0.05)               # 放す
        break

    cv2.imshow("camera", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    if cv2.waitKey(1) == 27: break   # Esc で中断

cv2.destroyAllWindows()
while True:
    p.stepSimulation(); time.sleep(1/240)


