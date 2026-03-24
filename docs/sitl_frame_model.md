# SITL フレームモデルパラメータの変更方法

## 背景：なぜモデルパラメータを合わせる必要があるか

ArduPilot SITLはデフォルトで以下の物理パラメータを使用する：

```
mass              = 3.0 kg  （ハードコード: SIM_Frame.h line 85）
moment_of_inertia = mass × diagonal_size² から幾何学的に推定
```

Mode 99 LQR は `sysid_params.txt` の値（MASS=2.0 kg 等）を使って設計される。
**このミスマッチが LQR の速度追従精度を低下させ、TILT/FLIP の根本原因となる。**

```
例: 質量50%ずれの影響
  LQR が計算したブレーキモーメント → 3.0 kg ドローンへの効果は設計値の 2/3
  → vel_ref=1.5 m/s でも実速度が 3-4 m/s まで超過
  → LQR が大きなブレーキモーメントを要求 → ピッチ 40°+ → TILT
```

---

## 解決策：カスタム JSON フレームファイル

ArduPilot SITL は `--model "+:path/to/frame.json"` でカスタム物理モデルを読み込める。
JSON ファイルで `mass` と `moment_inertia` を直接指定することで LQR 設計値と完全一致させる。

### JSON ファイルの場所

```
autonomous_drone_sim/configs/ardupilot/quad_2kg.json
```

### JSON ファイルのフォーマット（主要パラメータ）

```json
{
    "mass"           : 2.0,
    "diagonal_size"  : 0.45,
    "moment_inertia" : [0.0347, 0.0458, 0.0977],
    "hoverThrOut"    : 0.5,
    "num_motors"     : 4
}
```

| フィールド | 型 | 説明 | sysid_params.txt の対応 |
|---|---|---|---|
| `mass` | float | 機体総質量 (kg) | `MASS=2.0` |
| `diagonal_size` | float | 対角モーター間距離 (m) = 2 × ARM_LENGTH | `ARM_LENGTH=0.225` → `0.45` |
| `moment_inertia` | [x,y,z] | 慣性モーメント [Ixx, Iyy, Izz] (kg⋅m²) | `IXX, IYY, IZZ` |
| `hoverThrOut` | float | ホバリングスロットル (0-1) | `THROTTLE_HOVER=0.5` |
| `num_motors` | int | モーター数 | 4 |

> **注意:** `moment_inertia` が指定されていない（またはゼロ）場合は、
> `mass × (diagonal_size/2)²` から幾何学的に推定される（精度低い）。

### 利用可能な全フィールド（SIM_Frame.cpp より）

```json
{
    "mass"          : 2.0,        // kg
    "diagonal_size" : 0.45,       // m (motor-to-motor diagonal)
    "moment_inertia": [Ixx, Iyy, Izz],  // kg⋅m²
    "hoverThrOut"   : 0.5,        // 0-1
    "refSpd"        : 10.0,       // m/s (drag estimation reference speed)
    "refAngle"      : 15.0,       // deg (tilt at refSpd)
    "refVoltage"    : 22.2,       // V
    "refCurrent"    : 10.0,       // A
    "refAlt"        : 0,          // m AMSL
    "refTempC"      : 25,         // °C
    "refBatRes"     : 0.02,       // Ω
    "maxVoltage"    : 25.2,       // V (full battery)
    "battCapacityAh": 0,          // Ah (0 = unlimited)
    "propExpo"      : 0.65,       // MOT_THST_EXPO
    "refRotRate"    : 300,        // deg/s (max yaw rate)
    "pwmMin"        : 1000,       // μs
    "pwmMax"        : 2000,       // μs
    "spin_min"      : 0.10,       // MOT_SPIN_MIN
    "spin_max"      : 0.95,       // MOT_SPIN_MAX
    "slew_max"      : 0,          // max motor slew rate (0=disabled)
    "disc_area"     : 0.203,      // m² (total effective prop disc area)
    "mdrag_coef"    : 0.10,       // momentum drag coefficient
    "num_motors"    : 4
}
```

---

## 適用方法

### 1. arducopter バイナリへの渡し方

```bash
# --model の引数を "+" から "+:JSON_PATH" に変更
/path/to/arducopter \
    --model "+:/path/to/quad_2kg.json" \
    --defaults copter.parm,params.parm \
    ...
```

### 2. start_mode99_training.sh では自動適用済み

```bash
# start_mode99_training.sh 内:
FRAME_MODEL="$WORKSPACE_DIR/configs/ardupilot/quad_2kg.json"
...
"$ARDUPILOT_DIR/build/sitl/bin/arducopter" \
    --model "+:$FRAME_MODEL" \
    ...
```

通常のトレーニング起動では自動的に `quad_2kg.json` が読み込まれる。

---

## 動作確認方法

SITL 起動直後のログに以下が出力される：

```
Loaded model params from /path/to/quad_2kg.json
Suggested EK3_DRAG_BCOEF_* = XX.XXX, EK3_DRAG_MCOEF = X.XXX
```

`/tmp/sitl_mode99.log` で確認：

```bash
grep "Loaded model\|Suggested EK3" /tmp/sitl_mode99.log
```

---

## パラメータ変更手順（sysid_params.txt が更新された場合）

1. `sysid_params.txt` の新しい値を確認
2. `configs/ardupilot/quad_2kg.json` の対応フィールドを更新
3. `design_lqr_discrete.py` の質量・慣性値も同じ値に更新して `lqr_gains.txt` を再生成
4. `lqr_gains.txt` を `/tmp/sitl_cowork/` にコピー（または起動スクリプトが自動コピー）
5. トレーニング再起動

```bash
# lqr_gains.txt 再生成
cd /home/yonetoshi27/autonomous_drone_sim
python3 design_lqr_discrete.py

# 確認
cat lqr_gains.txt | head -5
```

---

## 技術メモ

### ArduPilot SITL の物理モデル内部実装

- フレームモデルは `libraries/SITL/SIM_Frame.cpp` で実装
- `Frame::init()` で JSON を読み込み、デフォルト値を上書き
- `Frame::calculate_forces()` で推力・トルクを計算
- 角加速度: `rot_accel = torque / moment_of_inertia` （SIM_Frame.cpp line 688-690）
- 並進加速度: `body_accel = thrust / gross_mass()` （SIM_Frame.cpp line 720）

### diagonal_size の計算

X フレーム（ARM_LENGTH = モーター中心-機体中心距離）の場合：

```
対角モーター間距離 = 2 × ARM_LENGTH = 2 × 0.225 = 0.45 m
```

`moment_inertia` を直接指定する場合、`diagonal_size` は抗力推定のみに使われるため精度への影響は小さい。

### disc_area（プロペラ有効面積）の計算

```
プロペラ直径 d から:
  disc_area = 4 × π × (d/2)²

10インチプロペラ (d=0.254m) の場合:
  disc_area = 4 × π × 0.127² ≈ 0.203 m²
```
