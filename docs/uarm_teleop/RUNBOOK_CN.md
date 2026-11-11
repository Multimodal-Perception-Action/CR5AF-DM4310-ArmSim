# U-Arm Config2 → CR10A 遥操仿真操作手册

本包用于把 **U-Arm Config2** 当作 leader，**Dobot CR10A** 当作 follower。六个机械臂关节严格按索引对应：

`U-J1→CR-J1, U-J2→CR-J2, U-J3→CR-J3, U-J4→CR-J4, U-J5→CR-J5, U-J6→CR-J6`。

统一映射公式：

`q_CR = follower_home + sign * scale * (q_UArm - leader_zero)`

默认 `sign=[+1,-1,+1,+1,+1,+1]`，因为 U-Arm 官方 Dobot CR5 示例对第 2 轴做了反向。**STEP 只能验证关节轴拓扑/索引，不能证明真机编码器或 API 的正方向**，所以接真机前必须先播放 `00_joint_mapping_sweep.json` 逐轴确认。

## 0. 先验证包

```bash
cd uarm_cr10a_teleop_sim
python common/validate_package.py
```

应看到：

```text
OK: XML syntax, 14 meshes, clips, paired clips, and strict 6x one-to-one J-index mapping checks passed.
```

`paired_clips/` 里每个时间点同时写了 `uarm_deg` 和映射后的 `cr10a_deg`，便于直接审计一一对应关系。

---

## 1. MuJoCo

### 安装

```bash
python -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r mujoco/requirements.txt
```

### 场景 1：逐关节映射检查（必须先跑）

```bash
python mujoco/teleop_runtime.py \
  --scene mapping_check \
  --clip clips/00_joint_mapping_sweep.json
```

你应该依次看到：U-Arm J1 只带动 CR10A J1；U-Arm J2 只带动 CR10A J2；一直到 J6。

### 场景 2：桌面 reach

```bash
python mujoco/teleop_runtime.py \
  --scene tabletop \
  --clip clips/01_tabletop_reach.json
```

### 场景 3：pick-place 轨迹预览

```bash
python mujoco/teleop_runtime.py \
  --scene pickplace \
  --clip clips/02_pickplace_motion.json
```

### 场景 4：柔性布料 / fold-like 轨迹

```bash
python mujoco/teleop_runtime.py \
  --scene cloth \
  --clip clips/03_cloth_fold_motion.json
```

该 MuJoCo 场景使用 `flexcomp` 柔性网格；轨迹只是遥操/接触测试片段，不是已经优化好的叠衣策略。

### 场景 5：6 轴联合运动

```bash
python mujoco/teleop_runtime.py \
  --scene tabletop \
  --clip clips/04_full_chain_demo.json
```

### 键盘/终端手动遥操

不传 `--clip`：

```bash
python mujoco/teleop_runtime.py --scene mapping_check
```

终端输入：

```text
j1 +5
j2 -10
home
g 1
quit
```

MuJoCo viewer 内也可以用 `1..6` 选轴，`A/D` 每次减/加 2°，`R` 回零，`G` 切换夹爪，`Q` 退出。

### 接真实 U-Arm leader

终端 A：

```bash
export PYTHONPATH=/ABS/PATH/LeRobot-Anything-U-Arm/src/uarm/scripts/Uarm_teleop/Feetech_servo:$PYTHONPATH
python tools/uarm_feetech_udp_bridge.py --port /dev/ttyUSB0 --dest 127.0.0.1:15000
```

终端 B：

```bash
python mujoco/teleop_runtime.py --scene tabletop --udp --udp-port 15000
```

此时 U-Arm 1..6 轴被读取并通过 UDP 发给仿真，CR10A 通过统一映射跟随。

---

## 2. Isaac Sim 6.x

**不要用系统 Python。** 使用 Isaac Sim 安装目录里的 `python.sh`（Windows 用 `python.bat`）。

### 第一次运行：URDF → USD

Linux：

```bash
cd /PATH/TO/isaac-sim
./python.sh /ABS/PATH/uarm_cr10a_teleop_sim/isaac/import_robots.py
```

完成后会生成：

```text
isaac/usd_assets/manifest.json
```

运行脚本会从 manifest 读取真实 USD 路径，不猜文件名。

### 精确映射模式（先验证）

```bash
./python.sh /ABS/PATH/uarm_cr10a_teleop_sim/isaac/teleop_runtime.py \
  --scene mapping_check \
  --clip /ABS/PATH/uarm_cr10a_teleop_sim/clips/00_joint_mapping_sweep.json \
  --control-mode state
```

`state` 模式直接设置 U-Arm 和 CR10A 的关节状态，适合先验证 J1→J1 … J6→J6 是否完全正确。

### Drive 跟随模式（更接近 follower 在“运动”）

```bash
./python.sh /ABS/PATH/uarm_cr10a_teleop_sim/isaac/teleop_runtime.py \
  --scene tabletop \
  --clip /ABS/PATH/uarm_cr10a_teleop_sim/clips/01_tabletop_reach.json \
  --control-mode drive
```

`drive` 模式：U-Arm 作为 leader 直接显示输入角，CR10A 使用关节 position target 由 PhysX drive 跟随。

其它场景只替换 `--scene` 和 `--clip`：

```text
pickplace + clips/02_pickplace_motion.json
cloth     + clips/03_cloth_fold_motion.json
tabletop  + clips/04_full_chain_demo.json
```

Isaac 的 cloth 当前是刚性薄片 proxy；真正的柔性布料测试请先用 MuJoCo `cloth.xml`。后续若你要 Isaac PhysX cloth/particle cloth，需要再按你的 Isaac 版本单独配置材料、particle system 和 attachment。

### Isaac + 真实 U-Arm

终端 A（普通 Python）：

```bash
python tools/uarm_feetech_udp_bridge.py --dest 127.0.0.1:15000
```

终端 B（Isaac Python）：

```bash
./python.sh /ABS/PATH/uarm_cr10a_teleop_sim/isaac/teleop_runtime.py \
  --scene tabletop --udp --udp-port 15000 --control-mode drive
```

---

## 3. 五组场景/片段

| 场景 | Leader clip | 用途 |
|---|---|---|
| `mapping_check` | `00_joint_mapping_sweep.json` | J1~J6 逐轴验证，必须第一步运行 |
| `tabletop` | `01_tabletop_reach.json` | 桌面 reach/工作空间预览 |
| `pickplace` | `02_pickplace_motion.json` | 抓取时序、夹爪与关节运动联调 |
| `cloth` | `03_cloth_fold_motion.json` | fold-like 轨迹和软体接触测试 |
| `tabletop` | `04_full_chain_demo.json` | 6 轴联合运动和整体映射检查 |

每个 `clips/*.json` 都有同名的 `paired_clips/*.json`，后者显式给出 U-Arm 与 CR10A 对应角度。

---

## 4. 真机接入前必须改的只有三类标定量

编辑：

```text
config/joint_mapping.json
```

关键字段：

```json
"leader_zero_deg": [0,0,0,0,0,0],
"follower_home_deg": [0,0,0,0,0,0],
"sign": [1,-1,1,1,1,1]
```

建议真机流程：先把 CR10A 速度限制到很低；一次只开放一个关节；确认 J1→J1 … J6→J6；若方向相反只改对应 `sign[i]`；然后记录实际 U-Arm 零位和 CR10A home。不要在符号和零位未验证时直接让 6 轴同时运动。

---

## 5. 目前模型能做什么 / 不能做什么

已经能可靠用于：

- U-Arm/CR10A 6R 构型和关节索引验证
- 一一对应 leader-follower joint mapping
- FK/IK 和轨迹预览
- MuJoCo/Isaac 控制链路联调
- U-Arm STS3215 实时 UDP 输入
- 场景/片段播放与后续数据记录扩展

目前不能把仿真的关节扭矩、碰撞力直接当成 CR10A 真机数值，因为 STEP 没有提供完整可信的 link mass / COM / inertia，包内动力学参数只是仿真占位。要做真实力控/力反馈，需要再补厂家或实测动力学参数和实际末端 F/T 传感器模型。
