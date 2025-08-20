import pybullet as p, pybullet_data, os

# 0. 物理サーバへ接続
p.connect(p.GUI)                                      # または p.DIRECT

# 1. 検索パスを登録（順序は任意）
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setAdditionalSearchPath(
    r"D:\Wasedaproject\external"
)

# 2. ロボット本体
panda_uid = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# 3. 真空グリッパ
vac_uid = p.loadURDF(os.path.join("urdf", "epick_gripper.urdf"), useFixedBase=False)

# 4. 固定拘束
EEF_LINK = 11
p.createConstraint(panda_uid, EEF_LINK, vac_uid, -1,
                   p.JOINT_FIXED, [0,0,0], [0,0,0], [0,0,0])


