# 关节角度记录工具

`read_joint_angles.py` —— 在电机**不使能**的情况下，把机器人手动摆成任意姿势，读取并记录 STM32 上报的全部 12 个关节角度。

主要用于：给固定策略采集若干个目标姿态（关键点）。

有两个版本，功能相同，选一个用：

| 版本 | 文件 | 操作方式 |
|---|---|---|
| 命令行 | `read_joint_angles.py` | 终端里打字（本文档第一～十节） |
| **图形界面** | `joint_angle_gui.py` | 鼠标点的窗口（[第十一节](#十一图形界面joint_angle_guipy)） |

**不要同时跑两个** —— 会抢同一个串口。

---

## 一、它做什么

程序常驻运行，做两件互不干扰的事：

| 产出 | 内容 | 写入时机 |
|---|---|---|
| `stream.csv` | 12 个关节角的**连续**记录 | 后台线程按固定频率一直写 |
| `keypoints.json` | 你手动标记的**关键点**，每个姿势一条 | 你在终端按回车时 |

两者写在同一个会话目录里，各占一个文件，不会混在一起。

---

## 二、安全性

**默认模式不会请求电机使能。** `--zero-gain-readback` 是针对已确认下位机 KP/KD 均为 0 的固件增加的显式选项，详见下方。

- 发出的每一帧 `command_flags` 都是 `0` —— bit0（`COMMAND_ENABLE`）始终为 0，使能位是清的
- `kp_scale` 和 `kd_scale` 都发 `0.0`
- 收到首帧前会以 50 Hz 发送全零目标的**去使能探测帧**；收到首帧后，保活帧改为回显当前测量角度

> 首帧探测解决了“上位机等首帧、下位机等命令”时的启动僵局。所有探测帧和保活帧均不使能电机，增益为零。`--no-keepalive` 连首帧探测也不发，完全不发帧；如果下位机只在收到命令后回状态，使用此选项会等到超时。

### 零增益固件的实时读数

如果 STM32 在收到 `command_flags=0` 后停止电机 CAN 反馈，USB 状态帧的序号仍可能继续增长，但关节角停留在停止前的值。**只有实际烧录的 STM32 固件已确认对所有电机使用 KP=0、KD=0 且前馈力矩为 0**，才运行：

```bash
python tools/read_joint_angles.py --port /dev/ttyACM0 --zero-gain-readback
```

此模式收到第一帧后，会每 20 ms 向 STM32 发送 `COMMAND_ENABLE=1`、最新实测目标角、`kp_scale=0`、`kd_scale=0`，促使下位机继续与电机通信并读取新的 CAN 反馈。**上位机的 KP/KD 字段可能被下位机忽略，关键是下位机实际应用的 KP/KD 必须为 0**。退出或遥测丢失时发送去使能帧。此模式不能和 `--no-keepalive` 同用；也不能把它用于仓库中的原版固定增益 STM32 固件，因为那会真正驱动电机。

先在可靠支撑下手动转动单个关节，再按 `p`：对应数值应随手转而变化。若序号增长但角度仍不变，需检查实际烧录固件的电机 CAN 反馈路径；上位机不能从未更新的 STM32 状态包还原新角度。

**本工具不做标定。** 标定在下位机侧完成，这里只是读取下位机已经上报的值。

---

## 三、部署到 Orin Nano

在**你的机器**上、仓库根目录执行：

```bash
scp humanoid_jetson_deploy/serial_link.py                  isaac@<JETSON_IP>:/home/isaac/humanoid_jetson_deploy/
scp humanoid_jetson_deploy/tools/read_joint_angles.py      isaac@<JETSON_IP>:/home/isaac/humanoid_jetson_deploy/tools/
scp humanoid_jetson_deploy/tools/joint_angle_gui.py        isaac@<JETSON_IP>:/home/isaac/humanoid_jetson_deploy/tools/
scp humanoid_jetson_deploy/tools/joint_motion.py           isaac@<JETSON_IP>:/home/isaac/humanoid_jetson_deploy/tools/
scp humanoid_jetson_deploy/tools/read_joint_angles.md      isaac@<JETSON_IP>:/home/isaac/humanoid_jetson_deploy/tools/
scp humanoid_jetson_deploy/tests/test_read_joint_angles.py isaac@<JETSON_IP>:/home/isaac/humanoid_jetson_deploy/tests/
scp humanoid_jetson_deploy/tests/test_joint_angle_gui.py   isaac@<JETSON_IP>:/home/isaac/humanoid_jetson_deploy/tests/
scp humanoid_jetson_deploy/tests/test_joint_motion.py      isaac@<JETSON_IP>:/home/isaac/humanoid_jetson_deploy/tests/
```

> ⚠️ **`serial_link.py` 是唯一被修改的现有文件**（加了 `reader_alive()` 和一个有界写锁）。
> 它向后兼容，`main.py` 不受影响，但如果同伴以后也改了这个文件，合并时注意。

`joint_angle_gui.py` + `joint_motion.py` 是图形界面版（见第十一节），不需要也可以不传。

> **这两个文件必须放在 `humanoid_jetson_deploy/` 里面。**
> 脚本靠 `Path(__file__).resolve().parents[1]` 往上找一级来 import `config`、`protocol`、`motor_test_common`、`serial_link`。放到别的目录会直接 import 报错。
>
> 它们是 untracked 文件，之后在 Jetson 上 `git pull` 不会覆盖或删除。

### 到 Jetson 上先验证

```bash
ssh isaac@<JETSON_IP>
cd ~/humanoid_jetson_deploy
python -m unittest tests.test_read_joint_angles -v
```

17 个测试，不需要接硬件。全过就说明环境没问题。

---

## 四、快速开始

```bash
cd ~/humanoid_jetson_deploy
python tools/read_joint_angles.py --port /dev/ttyACM0
```

启动后会看到：

```
Opening /dev/ttyACM0 (line coding 921600)
Motors stay disabled: every frame this tool sends has command_flags=0
First state packet: sequence=1234 flags=0x0000000F
Session directory: logs/joint_angles/20260924_143012_123456
Continuous stream: stream.csv at 20 Hz
Key points:        keypoints.json

Press Enter to save the current pose as a key point.
  <name>  save it under that name
  p       show the current angles without saving
  q       finish and close the files

[0 saved] name (Enter=auto) >
```

---

## 五、交互命令

光标停在 `name (Enter=auto) >` 时：

| 输入 | 动作 |
|---|---|
| **直接回车** | 采集当前姿势，自动命名 `keypoint_000`、`keypoint_001`… |
| **名字 + 回车** | 采集当前姿势，用你给的名字，如 `crouch` |
| **`p`** 或 `peek` 或 `?` | 只显示当前角度，**不保存**（摆姿势时随手看一眼） |
| **`q`** 或 `quit` 或 `exit` | 收尾退出。Ctrl+C / Ctrl+D 等效 |

每次保存会回显这次采集的详情：

```
[0 saved] name (Enter=auto) > crouch
  saved 'crouch' #0: 100 frames in 0.51s, spread 0.08 deg (stable)
  r_leg_pitch_joint=+0.150  r_leg_roll_joint=+0.000  ...  l_ankle_roll_joint=+0.000
```

- `100 frames in 0.51s` —— 这次采集取了多少帧、花了多久
- `spread 0.08 deg` —— 这段窗口内**任意关节的最大波动**。手没扶稳时这个数会变大
- `(stable)` / `(NOT STABLE)` —— 波动是否在 `--stable-tol-deg` 阈值内

判为不稳定时会额外警告：

```
  warning: the pose was still moving; hold the robot still and retake it
```

**此时这一条已经存进去了**，但它不可信。重新摆稳再采一次即可（重复命名不会覆盖，是新的一条）。

> `p` 和 `q` 是保留字。如果真要有个关键点叫这个名字，改用别的名字。

---

## 六、输出文件

```
logs/joint_angles/<会话名>/
├── keypoints.json     ← 关键点，全部在这一个文件里
└── stream.csv         ← 连续读取，另一个文件
```

`<会话名>` 默认是启动时间戳，可以用 `--session` 固定。

`logs/` 在仓库的 `.gitignore` 里，所以录制产生的文件**不会弄脏 git 工作区**。

### keypoints.json

```json
{
  "host_time_iso": "2026-09-24T14:30:12.001+08:00",
  "port": "/dev/ttyACM0",
  "joint_names": ["r_leg_pitch_joint", "...12 个..."],
  "stable_tol_deg": 0.5,
  "samples": 100,
  "keypoints": [
    {
      "index": 0,
      "name": "crouch",
      "host_time_iso": "2026-09-24T14:30:15.221+08:00",
      "frames": 100,
      "elapsed_s": 0.512,
      "max_spread_deg": 0.08,
      "stable": true,
      "state_sequence": 53120,
      "state_flags": "0x0000000F",
      "joint_position_rad": [0.1500001, 0.0, "...12 个..."],
      "joint_position_deg": [8.594, 0.0, "...12 个..."]
    }
  ]
}
```

**每存一个关键点就整份原子重写一次**（写 `.tmp` 再 `os.replace`），所以中途 Ctrl+C 或断电都不会丢掉已经采到的点。

`stable` 是 `false` 的点建议复查后重采。

### stream.csv

```
host_time_iso,elapsed_s,state_sequence,r_leg_pitch_joint_rad,r_leg_roll_joint_rad,...,l_ankle_roll_joint_rad
2026-09-24T14:30:12.310+08:00,0.100000,1240,0.15000001,0.00000000,...
```

由**独立后台线程**写入 —— 你停在提示符前不动，它照样在录。默认 20 Hz。

---

## 七、命令行参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--port` | `/dev/ttyACM0` | 串口设备 |
| `--baud` | `921600` | 仅设定线路编码；原生 USB CDC 忽略实际波特率 |
| `--out-dir` | `logs/joint_angles` | 会话目录的父目录 |
| `--session` | 时间戳 | 会话目录名，固定它可覆盖同一组 |
| `--samples` | `100` | 每次采集取多少帧做平均 |
| `--stable-tol-deg` | `0.5` | 窗口内波动小于此值即认为已稳定，提前结束采集 |
| `--timeout` | `3.0` | 单次采集的时间上限（秒） |
| `--log-hz` | `20.0` | `stream.csv` 记录频率。`0` = 完全关闭连续记录 |
| `--no-keepalive` | 关 | 打开后一个字节都不发 |
| `--zero-gain-readback` | 关 | 仅供已确认电机实际 KP/KD 为 0 的下位机固件；请求电机使能以持续获取反馈 |

常用组合：

```bash
# 只要关键点，不要连续记录（文件最小）
python tools/read_joint_angles.py --log-hz 0

# 连续记录要全速（STM32 约 200 Hz 上报）
python tools/read_joint_angles.py --log-hz 200

# 要求更严的稳定性（手抖得厉害时）
python tools/read_joint_angles.py --stable-tol-deg 0.2 --samples 200

# 多次会话写进同一目录，方便对比
python tools/read_joint_angles.py --session standing_poses
```

---

## 八、关节顺序（重要）

**保存顺序 = `config.JOINT_NAMES`，右腿在前：**

```
 0  r_leg_pitch_joint      6  l_leg_pitch_joint
 1  r_leg_roll_joint       7  l_leg_roll_joint
 2  r_leg_yaw_joint        8  l_leg_yaw_joint
 3  r_knee_pitch_joint     9  l_knee_pitch_joint
 4  r_ankle_pitch_joint   10  l_ankle_pitch_joint
 5  r_ankle_roll_joint    11  l_ankle_roll_joint
```

这与 `main.py` 加载模型时使用的顺序**完全一致**，也是 ONNX 观测/动作张量的顺序。工具**不做任何重排**。

> ⚠️ 仓库里存在第二套顺序：`motor_test_common.py:30` 的 `STM32_BRIDGE_JOINT_NAMES` 声称协议通道 0..5 是**左腿**。这与 `config.JOINT_NAMES` 矛盾。
>
> 该常量只被电机台架测试（`gentle_joint_direction_test.py`）使用，`main.py` 的生产路径从不引用它，所以本工具和部署路径是一致的。
>
> **线上实际是哪一套，需要在真机上确认一次** —— 机器人摆成默认蜷缩姿时，第 0 个通道读出来应该是 `+0.15`（右腿）。如果是 `-0.15`，说明线上是左腿在前，那本工具和 `main.py` 都会左右腿互换。

### 验证方法

把机器人摆成默认蜷缩姿（就是 `config.py` 里 `Q_DEFAULT` 那个姿势），跑一次 `p`：

- 第 0 个通道 `r_leg_pitch_joint` ≈ **+0.15** → 线上是右腿在前，本工具正确
- 第 0 个通道 ≈ **-0.15** → 线上是左腿在前，需要找同伴确认

单位同时给出弧度和角度，直接对比即可。

---

## 九、典型使用流程

给固定策略采集一组目标姿态：

```bash
cd ~/humanoid_jetson_deploy
python tools/read_joint_angles.py --port /dev/ttyACM0 --session task_a_poses
```

1. 用手把机器人摆成第一个姿势
2. 敲 `p` 看一眼角度对不对（不保存）
3. 摆稳，回车 → 存为 `keypoint_000`（或用 `start` 这样的名字）
4. 摆成第二个姿势，重复
5. 全部采完后敲 `q`

结束时会打印最后一次采集的、可直接粘贴的数组：

```
Last key point, policy-order array (rad), ready to paste:
np.array([
    +0.150000, +0.000000, +0.000000, +0.300000, -0.150000, +0.000000,
    -0.150000, +0.000000, +0.000000, -0.300000, +0.150000, +0.000000,
], dtype=np.float32)
```

之后从 `keypoints.json` 里按名字取任意一个关键点的 `joint_position_rad` 即可。

---

## 十、排障

| 现象 | 原因与处理 |
|---|---|
| `Permission denied: '/dev/ttyACM0'` | 串口权限。`sudo usermod -aG dialout $USER` 后**重新登录**；临时可用 `sudo chmod 666 /dev/ttyACM0` |
| `FAIL: No valid STM32 state packet received` | 下位机没在发数据。检查设备名 `ls /dev/ttyACM*`、USB 线、固件是否在跑 |
| `FAIL: STM32 reports a motor fault: flags=0x...` | 下位机报了故障（`STATE_FAULT`）。先在下位机侧清故障 |
| `FAIL: STM32 encoder-valid flag is missing` | `STATE_ENCODERS_VALID` 没置位，编码器数据不可信 |
| 每次采集都很慢（跑满 `--timeout`） | 姿态一直在动，达不到稳定阈值。扶稳，或放宽 `--stable-tol-deg` |
| 一直提示 `NOT STABLE` | 同上；也可能是 `--samples` 太大导致窗口太长。试试减小 `--samples` |
| 自动编号出现空缺（如 `keypoint_000` 之后直接是 `keypoint_002`） | 正常。自动名用的是**本次会话的全局序号**，中间手动命名的那些占掉了号。序号必定等于该点在此文件中的下标，不会重名 |

---

## 十一、图形界面（`joint_angle_gui.py`）

上面这个命令行工具还有一个**图形界面版**，功能相同——读取、实时显示、采集关键点——但是用鼠标点的窗口，不用记命令。

窗口分两个标签页：

| 标签页 | 做什么 | 是否驱动电机 |
|---|---|---|
| **Record** | 手摆姿势、采集关键点 | **否**（所有帧 `command_flags=0`） |
| **Fixed policy** | 输入目标角度 / 加载关键点 / 顺序回放 | **是**，详见下方 |

产出的 `keypoints.json` 与命令行版**逐字段相同**（共用同一套记录代码），两边可以互相加载。

---

### ⚠️ Fixed policy 标签页会真正驱动电机

这是本工具唯一危险的部分。设计上的防呆：

- **使能总开关**：默认关闭。打开时要**输入 `ENABLE` 字样**确认（不是点按钮，防双击误触）
- **急停按钮常驻**，且绑定 **空格 / Esc**（手在机器人上时够得着键盘）
- **软限位**：输入超出关节限位的角度会被**拒绝**而不是静默钳制——静默钳制会让你以为机器人到了 0.9 rad 而实际到了 0.45
- **状态横幅的颜色有严格含义**：**绿色只表示「电机断电，可以碰」**。使能后是琥珀色，运动中红色
- 底层还有 `joint_motion.py` 的三阶段安全链（关节限位 → 限速 → 实测窗口）、静默饱和检测、遥测丢失/固件卡死故障处理

**所有去使能路径都会让机器人变软**（`kp=kd=0`），站立的机器人会瘫倒。**必须悬空或有人扶稳。**

`commands.csv` 会记录每一帧命令的目标、模式、使能位和来源标签，事后可复盘。

### 状态横幅对照

| 显示 | 含义 |
|---|---|
| `connected - motors OFF (safe to touch)` 绿 | 安全，可以碰 |
| `MOTORS ON - holding, hands clear` 琥珀 | 带电保持中 |
| `MOTORS ON - moving to '<名字>' (step n/m)` 红 | 正在运动 |
| `armed, but the firmware has not reported the motors on` 琥珀 | 已请求使能，**固件尚未确认**——不要据此认为已带电 |
| `EMERGENCY STOP latched` 红 | 急停闭锁，需先 Disable 再按 Clear stop |
| `FAULT: <原因>` 红 | 已自动断电并停止 |

> 显示**从不**根据日志文字推断使能状态：`main.py` 打印 `MOTORS ENABLED` 时串口还没打开，那行字什么也证明不了。判据是固件自己上报的 `STATE_MOTORS_ENABLED` 位加上数据包新鲜度。

### 前置检查

```bash
python -c "import tkinter; tkinter.Tk()"
```

没反应只弹出一个空窗口就是好的（关掉它）。如果报错：

```bash
sudo apt install python3-tk
```

### 先不接机器人，确认窗口能显示

```bash
cd ~/humanoid_jetson_deploy
python tools/joint_angle_gui.py --simulate
```

`--simulate` 用一个内置的合成链路，不读串口、不需要机器人。**建议第一次一定先跑这个**——确认窗口、表格刷新、采集按钮都正常，再去接真机。

### 接真机

```bash
python tools/joint_angle_gui.py --port /dev/ttyACM0
```

电机全程不使能。

### 界面

```
┌─ Joint angle recorder (motors disabled) ─────────────┐
│ connected  state_seq=53120  flags=0x0000000F          │
│ Motors stay disabled: every frame ... command_flags=0 │
│  #   joint                  rad          deg          │
│  0   r_leg_pitch_joint   +0.149812    +8.584          │
│  1   r_leg_roll_joint    +0.000000    +0.000          │
│ ...                                                   │
│ ┌─ last capture ─────────────────────────────────┐    │
│ │ crouch  #0  100 frames / 0.51s  spread 0.08°   │    │
│ └────────────────────────────────────────────────┘    │
│ port: /dev/ttyACM0  crc errors: 0  telemetry: ok      │
│ stream.csv: 240 rows (20 Hz)   key points saved: 2    │
│ session: logs/joint_angles/20260924_143012            │
│ [Capture key point]        [EMERGENCY STOP] (灰化)    │
└───────────────────────────────────────────────────────┘
```

- 角度表每 100 ms 刷新，只读
- 点 **Capture key point** → 输入名字（留空自动编号）→ 采集 → 底部显示结果
- **采集时姿态没扶稳会标红**并提示重采，但**仍然保存**（你可以自己决定弃用）
- 顶部横幅：正常是绿色；收到固件故障标志（`STATE_FAULT`）或编码器数据无效会变红/橙；完全收不到数据包会显示 `NO TELEMETRY`
- 关窗口会先停后台线程、刷盘 CSV、关闭串口，然后才退出

### 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--port` | `/dev/ttyACM0` | 串口 |
| `--baud` | `921600` | 仅线路编码 |
| `--simulate` | 关 | 用内置合成链路，不接硬件 |
| `--out-dir` | `logs/joint_angles` | 会话父目录 |
| `--session` | 时间戳 | 会话目录名 |
| `--samples` | `100` | 每次采集帧数 |
| `--stable-tol-deg` | `0.5` | 稳定阈值 |
| `--timeout` | `3.0` | 单次采集时间上限 |
| `--log-hz` | `20.0` | 流记录频率；`0` 关闭 |

### 排障

| 现象 | 处理 |
|---|---|
| 启动弹「Cannot open the serial port」 | 弹窗里已列出检查步骤：`ls /dev/ttyACM*`、dialout 权限。或先用 `--simulate` 验证界面 |
| 启动弹「No STM32 state packet」 | 下位机必须先上电并在发数据，窗口才能启动。先去查下位机 |
| 窗口出不来 / `TclError` | 没装 `python3-tk`，或 `DISPLAY` 没设（纯 SSH 连的）。需要用显示器或 VNC |
| 弹「Cannot open the serial port」 | 可能有别的程序占着：`main.py`、`read_joint_angles.py`、`hold_standing_pose.py`。pyserial 不加锁，第二个打开者会**静默破坏数据流**而不是报错 |
| 点 Enable 没反应 | 处于急停/故障闭锁。先按 Disable，再按 Clear stop |
| 横幅一直「has not reported the motors on」 | 已发使能帧但固件未确认。**不要据此认为机器人带电了**，也不要伸手 |
| 顶部一直 `NO TELEMETRY` | 串口还在，但数据包断了。查线缆和下位机 |
| 表格是空的 | 同 `NO TELEMETRY` |

### 注意

**不要同时跑命令行版和图形版**——两个程序会同时打开 `/dev/ttyACM0`，互相抢数据包。用其中一个。

---

## 十二、运行测试

```bash
cd ~/humanoid_jetson_deploy
python -m unittest tests.test_read_joint_angles -v    # 命令行工具，17 个
python -m unittest tests.test_joint_angle_gui -v      # 图形界面，16 个
```

17 个测试，全部不需要硬件（状态帧通过真实的 `pack_state` / `FrameDecoder` 编解码后喂给被测代码）。覆盖：

- 关键点与连续记录**写各自的文件**
- 保活帧 `command_flags` **恒为 0**、增益恒为 0；`--no-keepalive` 时一帧不发
- 回车保存、`p` 不保存、`q` 退出
- 姿态不稳时如实报告，而不是静默给出错误的均值
