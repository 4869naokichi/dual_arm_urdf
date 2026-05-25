"""
Dual-arm robot URDF generator.

使い方:
    1. 下の DIMENSIONS セクションの数値を CAD 実測値で更新
    2. python generate_urdf.py を実行
    3. dual_arm_robot.urdf が出力される
    4. urdf-viz dual_arm_robot.urdf で確認

座標系: ROS 標準 (x=前, y=左, z=上), 単位: m / rad
ホームポーズ: 両腕は体側で下方向に伸びた状態 (Z- 方向)
"""

from pathlib import Path

# ====================================================================
# DIMENSIONS  --- CAD 実測値に書き換えて再生成
# ====================================================================

# --- 中央柱 (アルミフレーム) ---
# 制約: COLUMN_LOWER_LEN + COLUMN_UPPER_LEN = 0.499 (柱実測 499 mm)
COLUMN_LOWER_LEN     = 0.2615  # 床(柱底) から 腰回転軸 まで [実測]
COLUMN_UPPER_LEN     = 0.2375  # 腰回転軸 から 柱の頂上(首) まで [実測]
COLUMN_SIZE          = 0.040   # 角アルミ断面 (□40) [実測]
# (waist_link は torso_link にマージ済み、ハウジング寸法は不要)

# --- 肩のブラケット (柱頂上から左右に伸びる水平バー) ---
SHOULDER_BRACKET_HALFWIDTH = 0.0715  # 柱中心 → 片肩J1軸のY [実測]
SHOULDER_BRACKET_Z         = -0.0275 # 柱頂上 → 肩J1軸のZ (負=下) [実測]

# --- 腕 (片側ぶん、左右対称に適用) ---
# J1 = 肩 PITCH  (axis Y, 腕を前後に振る)         [XM540]
# J2 = 肩 ROLL   (axis X, 腕を体側から外側へ開く) [XM540]
J1_TO_J2          = 0.06725  # J1 → J2 の Z オフセット [実測]
# J3 = 上腕 YAW  (axis Z, 上腕の長軸まわり回旋)   [XM540]
J2_TO_J3          = 0.0825   # J2 → J3 の Z オフセット [実測]
# J4 = 肘 PITCH  (axis Y, 肘の屈伸)               [XM430]
UPPER_ARM_LEN     = 0.041    # J3 → J4 [実測]
# J5 = 前腕 YAW  (axis Z, 前腕の長軸まわり回旋)   [XM430]
J4_TO_J5          = 0.064    # [実測]
# J6 = 手首 PITCH (axis Y)                        [XM430]
FOREARM_LEN       = 0.0405   # J5 → J6 [実測]
# J7 = 手首 ROLL (axis Z, 手先の長軸まわり回旋)   [XM430]
J6_TO_J7          = 0.028    # [実測]

# --- グリッパ (並行 1 軸、両指は mimic で対称に直進開閉) ---
GRIPPER_BASE_LEN   = 0.090   # J7軸 → 指の並進高さ Z [実測]
GRIPPER_FINGER_LEN = 0.085   # 指の長さ (並進高さ → 指先 = TCP位置) [実測]
GRIPPER_HALF_OPEN  = 0.037   # 片指の最大ストローク [実測]、両側合計74mm
GRIPPER_REST_Y     = 0.01836 # 全閉時の指中心Y [実測]

# --- 頭 (パン・チルト) ---
NECK_BASE_HEIGHT  = 0.043    # 柱頂上 → パン軸 Z [実測]
PAN_TO_TILT_X     = 0.024    # パン軸 → チルト軸 X (前方) [実測]
PAN_TO_TILT_Z     = 0.0195   # パン軸 → チルト軸 Z (上方) [実測]
# TILT_TO_CAMERA: チルト軸→センサ面の距離。camera_link 原点 = カメラSTL原点 = センサ面
# なので実測値そのまま。
TILT_TO_CAMERA    = 0.05625  # チルト軸 → カメラセンサ面 X 前方 [実測]

# --- 関節可動範囲 (rad) ---
ARM_JOINT_LIMIT   = 2.6      # ~±150 deg, Dynamixel デフォルト相当
WAIST_LIMIT       = 3.0
NECK_PAN_LIMIT    = 2.0
NECK_TILT_LIMIT   = 1.0

# --- 動的パラメータ (おおまかな初期値、ROS 側でチューニング) ---
EFFORT_LIMIT      = 6.0      # N·m  (XM540 ~10 Nm, XM430 ~3 Nm の中間値)
VELOCITY_LIMIT    = 3.0      # rad/s

# --- Dynamixel ハウジング寸法 (可視化用) ---
XM540 = (0.0335, 0.0585, 0.0440)  # (x, y, z) 軸方向は親フレームに従う
XM430 = (0.0285, 0.0465, 0.0340)

# ====================================================================
# 以下はジェネレータ本体。通常編集不要。
# ====================================================================

INDENT = "  "


def _fmt(v):
    if isinstance(v, float):
        return f"{v:.6g}"
    return str(v)


def _xyz(t):
    return " ".join(_fmt(x) for x in t)


def _inertia_box(mass, size):
    sx, sy, sz = size
    ixx = mass * (sy * sy + sz * sz) / 12.0
    iyy = mass * (sx * sx + sz * sz) / 12.0
    izz = mass * (sx * sx + sy * sy) / 12.0
    return ixx, iyy, izz


def _inertia_cyl(mass, radius, length):
    ixx = mass * (3 * radius * radius + length * length) / 12.0
    iyy = ixx
    izz = mass * radius * radius / 2.0
    return ixx, iyy, izz


def link_box(name, size, origin_xyz=(0, 0, 0), origin_rpy=(0, 0, 0),
             color="black", mass=0.1):
    ixx, iyy, izz = _inertia_box(mass, size)
    return f"""  <link name="{name}">
    <visual>
      <origin xyz="{_xyz(origin_xyz)}" rpy="{_xyz(origin_rpy)}"/>
      <geometry><box size="{_xyz(size)}"/></geometry>
      <material name="{color}"/>
    </visual>
    <collision>
      <origin xyz="{_xyz(origin_xyz)}" rpy="{_xyz(origin_rpy)}"/>
      <geometry><box size="{_xyz(size)}"/></geometry>
    </collision>
    <inertial>
      <mass value="{_fmt(mass)}"/>
      <origin xyz="{_xyz(origin_xyz)}" rpy="{_xyz(origin_rpy)}"/>
      <inertia ixx="{_fmt(ixx)}" iyy="{_fmt(iyy)}" izz="{_fmt(izz)}" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
"""


def link_cyl(name, radius, length, origin_xyz=(0, 0, 0), origin_rpy=(0, 0, 0),
             color="black", mass=0.1):
    ixx, iyy, izz = _inertia_cyl(mass, radius, length)
    return f"""  <link name="{name}">
    <visual>
      <origin xyz="{_xyz(origin_xyz)}" rpy="{_xyz(origin_rpy)}"/>
      <geometry><cylinder radius="{_fmt(radius)}" length="{_fmt(length)}"/></geometry>
      <material name="{color}"/>
    </visual>
    <collision>
      <origin xyz="{_xyz(origin_xyz)}" rpy="{_xyz(origin_rpy)}"/>
      <geometry><cylinder radius="{_fmt(radius)}" length="{_fmt(length)}"/></geometry>
    </collision>
    <inertial>
      <mass value="{_fmt(mass)}"/>
      <origin xyz="{_xyz(origin_xyz)}" rpy="{_xyz(origin_rpy)}"/>
      <inertia ixx="{_fmt(ixx)}" iyy="{_fmt(iyy)}" izz="{_fmt(izz)}" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
"""


def link_empty(name):
    """質量0の純フレーム (TCP やセンサ取り付け基準など)"""
    return f"""  <link name="{name}"/>
"""


def link_mesh(name, mesh_file, origin_xyz=(0, 0, 0), origin_rpy=(0, 0, 0),
              scale=(0.001, 0.001, 0.001), color="black", mass=0.1, bbox=None):
    """メッシュSTL を visual/collision に使うリンク。
    bbox=(sx,sy,sz) [m] を渡すと box 近似で慣性を計算。"""
    if bbox:
        sx, sy, sz = bbox
        ixx = mass * (sy*sy + sz*sz) / 12.0
        iyy = mass * (sx*sx + sz*sz) / 12.0
        izz = mass * (sx*sx + sy*sy) / 12.0
    else:
        ixx = iyy = izz = mass * 0.001
    sxyz = " ".join(_fmt(x) for x in scale)
    return f"""  <link name="{name}">
    <visual>
      <origin xyz="{_xyz(origin_xyz)}" rpy="{_xyz(origin_rpy)}"/>
      <geometry><mesh filename="meshes/{mesh_file}" scale="{sxyz}"/></geometry>
      <material name="{color}"/>
    </visual>
    <collision>
      <origin xyz="{_xyz(origin_xyz)}" rpy="{_xyz(origin_rpy)}"/>
      <geometry><mesh filename="meshes/{mesh_file}" scale="{sxyz}"/></geometry>
    </collision>
    <inertial>
      <mass value="{_fmt(mass)}"/>
      <origin xyz="{_xyz(origin_xyz)}" rpy="{_xyz(origin_rpy)}"/>
      <inertia ixx="{_fmt(ixx)}" iyy="{_fmt(iyy)}" izz="{_fmt(izz)}" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
"""


def joint(name, jtype, parent, child, xyz=(0, 0, 0), rpy=(0, 0, 0),
          axis=(0, 0, 1), lower=-ARM_JOINT_LIMIT, upper=ARM_JOINT_LIMIT,
          effort=EFFORT_LIMIT, velocity=VELOCITY_LIMIT, mimic=None):
    parts = [f'  <joint name="{name}" type="{jtype}">']
    parts.append(f'    <parent link="{parent}"/>')
    parts.append(f'    <child link="{child}"/>')
    parts.append(f'    <origin xyz="{_xyz(xyz)}" rpy="{_xyz(rpy)}"/>')
    if jtype in ("revolute", "prismatic", "continuous"):
        parts.append(f'    <axis xyz="{_xyz(axis)}"/>')
        if jtype != "continuous":
            parts.append(
                f'    <limit lower="{_fmt(lower)}" upper="{_fmt(upper)}" '
                f'effort="{_fmt(effort)}" velocity="{_fmt(velocity)}"/>'
            )
    if mimic is not None:
        m_joint, m_multiplier, m_offset = mimic
        parts.append(
            f'    <mimic joint="{m_joint}" multiplier="{_fmt(m_multiplier)}" offset="{_fmt(m_offset)}"/>'
        )
    parts.append("  </joint>")
    return "\n".join(parts) + "\n"


def materials():
    return """  <material name="black">  <color rgba="0.10 0.10 0.10 1.0"/></material>
  <material name="green">  <color rgba="0.05 0.65 0.25 1.0"/></material>
  <material name="gray">   <color rgba="0.55 0.55 0.55 1.0"/></material>
  <material name="silver"> <color rgba="0.80 0.80 0.82 1.0"/></material>
  <material name="orange"> <color rgba="0.95 0.55 0.10 1.0"/></material>
  <material name="blue">   <color rgba="0.10 0.30 0.85 1.0"/></material>
"""


# --------------------------------------------------------------------
# アーム生成 (side = "L" or "R", y_sign = +1 (左) or -1 (右))
# --------------------------------------------------------------------
def build_arm(side, y_sign):
    out = []
    p = f"{side}_"  # プレフィクス

    # 肩のマウントブラケット (柱から左右に張り出すバー、固定)
    out.append(link_box(
        f"{p}shoulder_mount",
        size=(0.040, 2 * SHOULDER_BRACKET_HALFWIDTH * 0 + 0.040, 0.030),
        color="black", mass=0.05))
    out.append(joint(
        f"{p}shoulder_mount_fixed", "fixed",
        parent="torso_link", child=f"{p}shoulder_mount",
        xyz=(0, y_sign * SHOULDER_BRACKET_HALFWIDTH, COLUMN_UPPER_LEN + SHOULDER_BRACKET_Z)))

    # J1: 肩 PITCH (axis Y)  [XM540]
    out.append(link_box(f"{p}link1", size=XM540, color="black", mass=0.165))
    out.append(joint(
        f"{p}joint1", "revolute",
        parent=f"{p}shoulder_mount", child=f"{p}link1",
        xyz=(0, 0, 0), axis=(0, 1, 0)))

    # J2: 肩 ROLL (axis X)  [XM540]
    # J1→J2 は外側 (Y方向) オフセット。J1のブラケットが横に伸びてJ2を保持する構造。
    out.append(link_box(f"{p}link2", size=XM540, color="black", mass=0.165))
    out.append(joint(
        f"{p}joint2", "revolute",
        parent=f"{p}link1", child=f"{p}link2",
        xyz=(0, y_sign * J1_TO_J2, 0), axis=(1, 0, 0)))

    # J3: 上腕 YAW (axis Z, 上腕長軸)  [XM540]
    out.append(link_cyl(f"{p}link3", radius=0.022, length=UPPER_ARM_LEN,
                       origin_xyz=(0, 0, -UPPER_ARM_LEN / 2),
                       color="green", mass=0.20))
    out.append(joint(
        f"{p}joint3", "revolute",
        parent=f"{p}link2", child=f"{p}link3",
        xyz=(0, 0, -J2_TO_J3), axis=(0, 0, 1)))

    # J4: 肘 PITCH (axis Y)  [XM430]
    out.append(link_box(f"{p}link4", size=XM430, color="black", mass=0.082))
    out.append(joint(
        f"{p}joint4", "revolute",
        parent=f"{p}link3", child=f"{p}link4",
        xyz=(0, 0, -UPPER_ARM_LEN), axis=(0, 1, 0)))

    # J5: 前腕 YAW (axis Z, 前腕長軸)  [XM430]
    out.append(link_cyl(f"{p}link5", radius=0.018, length=FOREARM_LEN,
                       origin_xyz=(0, 0, -FOREARM_LEN / 2),
                       color="green", mass=0.15))
    out.append(joint(
        f"{p}joint5", "revolute",
        parent=f"{p}link4", child=f"{p}link5",
        xyz=(0, 0, -J4_TO_J5), axis=(0, 0, 1)))

    # J6: 手首 PITCH (axis Y)  [XM430]
    out.append(link_box(f"{p}link6", size=XM430, color="black", mass=0.082))
    out.append(joint(
        f"{p}joint6", "revolute",
        parent=f"{p}link5", child=f"{p}link6",
        xyz=(0, 0, -FOREARM_LEN), axis=(0, 1, 0)))

    # J7: 手首 ROLL (axis Z, 手先長軸)  [XM430]
    out.append(link_box(f"{p}link7", size=XM430, color="black", mass=0.082))
    out.append(joint(
        f"{p}joint7", "revolute",
        parent=f"{p}link6", child=f"{p}link7",
        xyz=(0, 0, -J6_TO_J7), axis=(0, 0, 1)))

    # グリッパ本体 (リンク7の先端に直接固定、フレーム原点はJ7軸)
    out.append(link_box(f"{p}gripper_base", size=(0.040, 0.060, GRIPPER_BASE_LEN),
                       origin_xyz=(0, 0, -GRIPPER_BASE_LEN / 2),
                       color="green", mass=0.05))
    out.append(joint(
        f"{p}gripper_base_fixed", "fixed",
        parent=f"{p}link7", child=f"{p}gripper_base",
        xyz=(0, 0, 0)))

    # 左指 (prismatic, +Y 方向に並進で開く)
    out.append(link_box(f"{p}finger_left", size=(0.010, 0.012, GRIPPER_FINGER_LEN),
                       origin_xyz=(0, 0, -GRIPPER_FINGER_LEN / 2),
                       color="silver", mass=0.01))
    out.append(joint(
        f"{p}gripper_joint", "prismatic",
        parent=f"{p}gripper_base", child=f"{p}finger_left",
        xyz=(0, GRIPPER_REST_Y, -GRIPPER_BASE_LEN), axis=(0, 1, 0),
        lower=0.0, upper=GRIPPER_HALF_OPEN,
        effort=20.0, velocity=0.1))

    # 右指 (mimic で対称、-Y 方向に並進)
    out.append(link_box(f"{p}finger_right", size=(0.010, 0.012, GRIPPER_FINGER_LEN),
                       origin_xyz=(0, 0, -GRIPPER_FINGER_LEN / 2),
                       color="silver", mass=0.01))
    out.append(joint(
        f"{p}gripper_mimic", "prismatic",
        parent=f"{p}gripper_base", child=f"{p}finger_right",
        xyz=(0, -GRIPPER_REST_Y, -GRIPPER_BASE_LEN), axis=(0, 1, 0),
        lower=-GRIPPER_HALF_OPEN, upper=0.0,
        effort=20.0, velocity=0.1,
        mimic=(f"{p}gripper_joint", -1.0, 0.0)))

    # ツールセンタポイント (TCP) - 空フレーム
    out.append(link_empty(f"{p}tcp"))
    out.append(joint(
        f"{p}tcp_fixed", "fixed",
        parent=f"{p}gripper_base", child=f"{p}tcp",
        xyz=(0, 0, -GRIPPER_BASE_LEN - GRIPPER_FINGER_LEN)))

    return "".join(out)


# --------------------------------------------------------------------
# 全体組み立て
# --------------------------------------------------------------------
def build_robot():
    out = ['<?xml version="1.0"?>\n', '<robot name="dual_arm_robot">\n\n']
    out.append(materials())
    out.append("\n")

    # base_footprint: 床面の基準フレーム (Z=0)
    out.append(link_empty("base_footprint"))

    # base_link: 柱の下半分 (床から腰軸まで、固定)
    # STL: メッシュ原点 = 床中心 (Fusionで原点合わせ済み)
    out.append(link_mesh(
        "base_link", mesh_file="base_link.stl",
        bbox=(0.088, 0.087, 0.273),  # 慣性近似用
        color="black", mass=2.0))
    out.append(joint(
        "base_footprint_to_base", "fixed",
        parent="base_footprint", child="base_link"))

    # torso_link: 腰回転軸より上の回転する全部 (ハウジング + 上柱 + 肩マウントバー)
    # 原点 = 腰回転軸 (Z=0 がこのリンク内では腰軸の高さ)
    out.append(link_mesh(
        "torso_link", mesh_file="torso_link.stl",
        bbox=(0.087, 0.166, 0.2849),
        color="black", mass=1.8))

    # 腰関節: Z 軸まわり回転 (base_link → torso_link 直結)
    out.append(joint(
        "waist_joint", "revolute",
        parent="base_link", child="torso_link",
        xyz=(0, 0, COLUMN_LOWER_LEN), axis=(0, 0, 1),
        lower=-WAIST_LIMIT, upper=WAIST_LIMIT,
        effort=10.0, velocity=2.0))

    # 頭 (パン・チルト + カメラ)
    out.append(link_mesh(
        "neck_pan_link", mesh_file="neck_pan_link.stl",
        bbox=(0.0465, 0.0424, 0.0338),
        color="black", mass=0.10))
    out.append(joint(
        "neck_pan_joint", "revolute",
        parent="torso_link", child="neck_pan_link",
        xyz=(0, 0, COLUMN_UPPER_LEN + NECK_BASE_HEIGHT),
        axis=(0, 0, 1),
        lower=-NECK_PAN_LIMIT, upper=NECK_PAN_LIMIT,
        effort=3.0, velocity=3.0))

    out.append(link_mesh(
        "neck_tilt_link", mesh_file="neck_tilt_link.stl",
        bbox=(0.038, 0.065, 0.025),
        color="black", mass=0.10))
    out.append(joint(
        "neck_tilt_joint", "revolute",
        parent="neck_pan_link", child="neck_tilt_link",
        xyz=(PAN_TO_TILT_X, 0, PAN_TO_TILT_Z), axis=(0, 1, 0),
        lower=-NECK_TILT_LIMIT, upper=NECK_TILT_LIMIT,
        effort=3.0, velocity=3.0))

    # カメラ本体 (Realsense 等)
    # camera_link フレーム = カメラ本体中心
    out.append(link_mesh(
        "camera_link", mesh_file="camera_link.stl",
        bbox=(0.025, 0.090, 0.025),
        color="gray", mass=0.05))
    out.append(joint(
        "camera_fixed", "fixed",
        parent="neck_tilt_link", child="camera_link",
        xyz=(TILT_TO_CAMERA, 0, 0)))

    # 光学フレーム (ROS 規約: z=前方, x=右, y=下)
    out.append(link_empty("camera_optical_frame"))
    out.append(joint(
        "camera_optical_fixed", "fixed",
        parent="camera_link", child="camera_optical_frame",
        xyz=(0, 0, 0),
        rpy=(-1.5707963, 0.0, -1.5707963)))

    # 左右の腕
    out.append("\n  <!-- ===== LEFT ARM ===== -->\n")
    out.append(build_arm("L", +1))
    out.append("\n  <!-- ===== RIGHT ARM ===== -->\n")
    out.append(build_arm("R", -1))

    out.append("\n</robot>\n")
    return "".join(out)


def main():
    urdf = build_robot()
    out_path = Path(__file__).parent / "dual_arm_robot.urdf"
    out_path.write_text(urdf, encoding="utf-8")
    print(f"Wrote {out_path} ({len(urdf):,} bytes)")
    # 簡単な検算
    total = COLUMN_LOWER_LEN + COLUMN_UPPER_LEN
    print(f"Column total length: {total*1000:.1f} mm (lower {COLUMN_LOWER_LEN*1000:.1f} + upper {COLUMN_UPPER_LEN*1000:.1f})")


if __name__ == "__main__":
    main()
