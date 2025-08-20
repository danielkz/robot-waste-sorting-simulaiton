import pybullet as p, pybullet_data
from PIL import Image

# 1. 初期化
p.connect(p.GUI)                         # p.DIRECT でも可
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

# 2. Panda ロボット読込
robot_uid = p.loadURDF(
    "franka_panda/panda.urdf",
    useFixedBase=True,
    flags=p.URDF_USE_SELF_COLLISION
)

# 3. エンドエフェクタ (link 11) の位置を取得
EEF_LINK = 11                            # Franka Hand のルートリンク
pos, _ = p.getLinkState(robot_uid, EEF_LINK)[:2]

# 4. カメラ設定（側面 = yaw 90°, pitch や distance は適宜調整）
W, H = 640, 480
view = p.computeViewMatrixFromYawPitchRoll(
    cameraTargetPosition=pos,
    distance=0.25,                       # エンドエフェクタが画面いっぱいになる距離
    yaw=90,                              # +y 方向からの側面
    pitch=-10,                           # 少し俯瞰
    roll=0,
    upAxisIndex=2
)
proj = p.computeProjectionMatrixFOV(
    fov=45,                              # 狭めてクローズアップ
    aspect=W/H,
    nearVal=0.01,
    farVal=1.0
)

# 5. 画像取得・保存
rgb = p.getCameraImage(W, H, view, proj)[2]
Image.fromarray(rgb).save("eef_side.png")
print("Saved: eef_side.png")

# 必要なら p.disconnect()

