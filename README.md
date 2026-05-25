# Dual-Arm Robot URDF

7+1軸 × 2 + 腰1 + 頭パン・チルト2 = **計19 DOF** の双腕ロボット URDF。

```
ファイル構成
dual_arm_urdf/
├── generate_urdf.py        # 寸法パラメータから URDF を生成 (uv run で実行)
├── dual_arm_robot.urdf     # 生成された URDF (これを ROS 担当に渡す)
├── meshes/                 # STL を後で置く場所 (空)
├── tools/
│   └── urdf-viz.exe        # Windows用 URDF ビューア (単体exe)
└── README.md
```

---

## 1. urdf-viz で表示確認

PowerShell から:

```powershell
cd c:\Users\nikud\dual_arm_urdf
.\tools\urdf-viz.exe .\dual_arm_robot.urdf
```

urdf-viz は **GUI スライダ無し**、すべてキーボード/マウス操作:

| 操作 | 動作 |
|---|---|
| 左ドラッグ | 視点回転 |
| 右ドラッグ | 視点パン |
| ホイール | ズーム |
| `o` / `p` | 関節を選択（ID +1 / -1） |
| `;` / `:` | IK ターゲットを選択 |
| `↑` / `↓` | 選択中の関節を ±`move-joint-diff-unit` 動かす |
| `Ctrl + 左ドラッグ` | 選択中の関節を**連続で**動かす（prismatic でもスムーズ） |
| `Shift + ドラッグ` / `Shift+Ctrl + ドラッグ` | IK |
| `l` | URDF をリロード |
| `r` | ランダム姿勢 |
| `z` | リセット |
| `c` | visual / collision 切替 |
| `f` | リンクフレーム表示トグル |
| `n` | リンク名表示トグル |
| `m` | メニュー表示トグル |

### キー操作の step サイズに注意
矢印キーの 1 押しは `--move-joint-diff-unit`（既定 0.1）。
これは revolute (rad) には適切だが、prismatic (m) では **100 mm/press** となり
グリッパが一発で上限張りつき → 動かないように見える。

```powershell
# prismatic フレンドリーな step (5 mm/press, 0.29°/press)
.\tools\urdf-viz.exe .\dual_arm_robot.urdf --move-joint-diff-unit 0.005
```

ただし **Ctrl+左ドラッグ** はこの設定に依存せず連続で動かせるので、prismatic 関節は
こちらが楽。

### 自動リロード
URDF ファイルを保存 → urdf-viz が自動で再読み込み。
revolute ↔ prismatic のような関節型変更時は念のため `l` キーで手動リロード。

---

## 2. 寸法の更新ワークフロー

寸法は [generate_urdf.py](generate_urdf.py) の上部 `DIMENSIONS` セクションに集約。

```powershell
# 値を編集 → 再生成
uv run --python 3.12 generate_urdf.py

# urdf-viz は自動でリロードされる (起動したままでOK)
```

### CAD で計測してほしい寸法

下記を Fusion 360 から拾って渡してもらえれば、ジェネレータの値を実測に置き換えます。
**現状は推定値が入っている**ので、実物との見た目のズレはほぼ寸法のせいです。

#### 中央柱 (合計 500 mm の内訳)
| 変数名 | 意味 | 現在値 |
|---|---|---|
| `COLUMN_LOWER_LEN` | 柱底（床）から腰回転軸まで Z | 261.5 mm |
| `COLUMN_UPPER_LEN` | 腰回転軸から柱頂上（首根本）まで Z | 237.5 mm |
| `COLUMN_SIZE` | アルミフレーム断面（□） | 40 mm |

#### 肩マウント
| 変数名 | 意味 | 現在値 |
|---|---|---|
| `SHOULDER_BRACKET_HALFWIDTH` | 柱中心から片側の肩 J1 軸までの Y 距離 | 71.5 mm |
| `SHOULDER_BRACKET_Z` | 柱頂上から肩 J1 軸への Z オフセット（負=下） | -27.5 mm |

#### 片腕（左右同値、ホームポーズ=腕を下に伸ばした状態で Z 軸=腕の長軸）
| 変数名 | 意味 | 現在値 |
|---|---|---|
| `J1_TO_J2` | J1（肩pitch）→ J2（肩roll）軸間 Y オフセット | 67.25 mm |
| `J2_TO_J3_X` | J2（肩roll）→ J3（上腕yaw）軸間 X オフセット（前方+） | 15.5 mm |
| `J2_TO_J3` | J2（肩roll）→ J3（上腕yaw）軸間 Z 距離 | 82.5 mm |
| `UPPER_ARM_LEN` | J3 → J4（肘pitch）軸間 Z 距離 = 上腕長 | 41 mm |
| `J4_TO_J5` | J4 → J5（前腕yaw）軸間 Z 距離 | 64 mm |
| `FOREARM_LEN` | J5 → J6（手首pitch）軸間 Z 距離 = 前腕長 | 40.5 mm |
| `J6_TO_J7` | J6 → J7（手首roll）軸間 Z 距離 | 28 mm |
| `GRIPPER_BASE_LEN` | J7 → 指の付け根 Z | 90 mm |
| `GRIPPER_FINGER_LEN` | 指の長さ | 85 mm |
| `GRIPPER_HALF_OPEN` | 片指の最大ストローク（並行開閉、両側で2倍開く） | 37 mm |
| `GRIPPER_REST_Y` | 全閉時の指中心の Y オフセット（指厚みぶん） | 18.36 mm |

#### 頭
| 変数名 | 意味 | 現在値 |
|---|---|---|
| `NECK_BASE_HEIGHT` | 柱頂上 → パン軸 Z | 20 mm |
| `PAN_TO_TILT` | パン軸 → チルト軸 Z | 40 mm |
| `TILT_TO_CAMERA` | チルト軸 → カメラ光学中心 X（前方） | 30 mm |

#### 関節軸の方向（要確認）
現在は次の前提でセット。**実機と違う場合は教えてください。**
- J1: 肩 PITCH（Y軸まわり、腕を前後に振る）
- J2: 肩 ROLL（X軸まわり、腕を体側から外側へ広げる）
- J3: 上腕 YAW（Z軸=腕長軸まわり、上腕回旋）
- J4: 肘 PITCH（Y軸まわり、肘屈伸）
- J5: 前腕 YAW（Z軸=前腕長軸まわり、前腕回旋）
- J6: 手首 PITCH（Y軸まわり）
- J7: 手首 ROLL（Z軸=手先長軸まわり）

---

## 3. Fusion 360 → STL 戦略

### 大原則
**1 URDF リンク = 1 Fusion コンポーネント = 1 STL ファイル**

URDFのリンクは「一体で動く部品のかたまり」。たとえば「J3（上腕yaw）リンク」には、
J3 のXM540本体 + 上腕パイプ + J4 サーボを取り付けるブラケット、までを全部入れる。
J4 サーボ自体は次の link4 側。

### コンポーネントの原点位置（最重要）
**各コンポーネントの「コンポーネント原点」を、その親側ジョイントの回転軸上に置く。**

これだけ守れば、STL書き出し時に位置オフセットを気にしなくて済む（URDFの
`<visual><origin xyz="0 0 0"/>` のままで OK）。

たとえば link3（上腕）なら:
- コンポーネント原点 = J3（上腕yaw軸）の中心
- そこから -Z 方向（下方向）に上腕本体が伸びる構造
- 一番下に J4（肘）のサーボマウントが付く

### Fusion 360 での具体的なアセンブリ手順

#### 1) ルート設計を新規作成
- 単位: mm（あとで URDF 側で scale="0.001" するか、m で書き出すか選択）
- Z up（Fusion デフォルト）

#### 2) コンポーネント階層を URDF と一致させる
ブラウザツリーで右クリック → "新規コンポーネント" を繰り返す。
全部で **約30コンポーネント**:

```
DualArmRobot/
├── base_link              (柱下半分 + 床ベース)
├── waist_link             (腰XM540ハウジング、回転部)
├── torso_link             (柱上半分 + 肩マウントバー)
├── neck_pan_link          (パン用XM430)
├── neck_tilt_link         (チルト用XM430 + マウント)
├── camera_link            (カメラ本体)
├── L_link1                (左肩pitch XM540 + ブラケット)
├── L_link2                (左肩roll XM540 + ブラケット)
├── L_link3                (左上腕: XM540 + パイプ + 肘マウント)
├── L_link4                (左肘 XM430 + ブラケット)
├── L_link5                (左前腕: XM430 + パイプ + 手首マウント)
├── L_link6                (左手首pitch XM430 + ブラケット)
├── L_link7                (左手首roll XM430)
├── L_gripper_base         (左グリッパ本体・駆動側XM430)
├── L_finger_left          (左グリッパの指1)
├── L_finger_right         (左グリッパの指2)
└── R_... (上記の R 版を左右ミラーで作成、計13個)
```

#### 3) 各コンポーネントの原点を「親側ジョイント軸」に合わせる
- 部品をモデリングしてから、`Modify > 整列(Align)` で
  コンポーネント原点をジョイント軸上に移動
- またはモデリングの最初に「ジョイント軸 = スケッチ原点」として配置

ジョイント軸の対応:
| Component | 原点位置（親側ジョイント） |
|---|---|
| base_link | 床（base_footprint と同位置） |
| waist_link | 腰回転軸の中心（柱底から COLUMN_LOWER_LEN 上） |
| torso_link | 腰回転軸の中心（waist_link と同位置でOK） |
| L_link1 | 左肩 J1（pitch）軸上 |
| L_link2 | 左肩 J2（roll）軸上 |
| L_link3 | 左 J3（上腕yaw）軸上 = 上腕の最上端 |
| L_link4 | 左肘 J4 軸上 = 上腕の最下端 |
| L_link5 | 左 J5（前腕yaw）軸上 = 前腕の最上端 |
| L_link6 | 左手首 J6 軸上 = 前腕の最下端 |
| L_link7 | 左手首 J7 軸上 |
| L_gripper_base | グリッパ本体取付け位置 |
| L_finger_left | 左指の回転軸上 |
| L_finger_right | 右指の回転軸上 |

#### 4) (任意) Fusion ジョイントで動作確認
- "アセンブル > ジョイント" でコンポーネント間を回転ジョイントで接続
- Fusion 内で実際に手で動かして可動範囲・干渉を事前にチェック
- URDF 側ではジョイント情報は使わない（URDF に手書きされる）

#### 5) STL書き出し
各コンポーネントを個別に書き出し:

1. ブラウザでコンポーネントを右クリック → "メッシュとして保存"
2. ダイアログ:
   - **形式**: STL（バイナリ）
   - **単位**: ミリメートル（URDF側で scale="0.001 0.001 0.001"）
     - または メートルでも可（URDFで scale なしまたは "1 1 1"）
   - **リファインメント**: 中（可視化のみなので十分。動力学計算には不要）
   - **構造**: "1ファイル" にチェック
3. 保存先: `dual_arm_urdf/meshes/{コンポーネント名}.stl`
   - ファイル名は URDF のリンク名と完全一致させる（例: `L_link3.stl`）

#### 6) URDF をメッシュ参照に切り替え
[generate_urdf.py](generate_urdf.py) の `link_box` / `link_cyl` 関数を
メッシュ参照版 `link_mesh` に置き換える（STL が揃ってから）。
こちら側で対応するので、STL一式が揃ったら知らせてください。

### 軽量化のコツ
- ネジ・座金・配線などは STL から除外（可視化が重くなる）
- 装飾的な凹凸はリファインメント"中" or "低"で十分
- 1 STL あたり数百 KB ～ 数 MB が目安
- 左右ミラーは Fusion で片側設計 → ミラーコピー → 両側 STL書き出し

### 座標系のミスマッチに注意
- Fusion 360: Z up、ROS と同じ
- 単位だけ注意（Fusion mm vs URDF m）

---

## 4. ROS担当者への引き渡し物

ROS担当には以下を渡せばOK:
- `dual_arm_robot.urdf`（最終版）
- `meshes/*.stl`（STLが揃ったら）
- このREADME（座標系・関節軸の仕様書として）

ROS側でやること（参考、自分はやらない）:
- `package://dual_arm_urdf/meshes/...` 形式のパスへ書き換え
- joint_state_publisher_gui / robot_state_publisher での確認
- MoveIt 設定 / Gazebo シミュレーション用 inertial 値の精緻化
