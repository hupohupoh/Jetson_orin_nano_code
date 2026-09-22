# gpu_vision — Jetson GPU 加速版视觉

目标硬件：**Jetson Orin Nano Super 8GB**（Ampere 1024 CUDA 核）。
视觉预处理（二值化/形态学/透视变换）走 `cv2.cuda`，几何算法
（Hough/LSD/轮廓/连通域）留 CPU——CUDA 无对应实现或收益小。

**cv2.cuda 不可用时自动 CPU 回退**（backend.py 的 HAS_CUDA 探测），回退实现
与 `jetson/` CPU 版逐算子等价——桌面无 GPU 跑本目录 = 验证正确逻辑。

## 文件

| 文件 | 说明 |
|---|---|
| `backend.py` | cv2.cuda 算子后端：找框二值链 / 巡线二值链 / warp / 核缓存 |
| `shape_detector_gpu.py` | 图卡找框（继承 CPU 版，预处理链 GPU 化）|
| `line_detector_gpu.py` | 巡线（process 预处理链 GPU 化）|
| `vision_camera.py` | 相机源：Windows DSHOW / Linux V4L2 / 视频文件 |
| `run_robot.py` | 机器人控制器（协议 V2 LINE_CTRL + 一步前瞻）|
| `probe.py` | 板子到手第一步：CUDA/OpenCV 能力 + 算子计时 |
| `compare_cpu_gpu.py` | CPU 版 vs GPU 版同帧一致性 |
| `requirements-gpu.txt` | Jetson 依赖与装法 |

## Jetson 安装

**板子准备**（在测任何性能数据之前，默认跑低功耗档会测出偏低的帧率）：
```bash
sudo nvpmodel -p --verbose && sudo nvpmodel -m 0 && sudo jetson_clocks
sudo apt install -y v4l-utils
sudo usermod -aG dialout $USER    # 串口免 sudo，重新登录生效
```

**JetPack 7.2**（Ubuntu 24.04 + Python 3.12 + CUDA 13.2）：
```bash
python3 -m venv ~/robocup-venv && source ~/robocup-venv/bin/activate
pip install "numpy<=1.26.4"   # 板子预装 OpenCV 按 numpy 1.x 编译
pip install pyserial
python probe.py               # OpenCV 探测（预装版是否带 CUDA）
```

JP7.2 预装的 OpenCV 4.8.0 和 apt 的 `python3-opencv` **都不带 CUDA**，
`cv2.cuda` 实际走 CPU 回退。想要真 GPU 加速只能源码编译，且 CUDA 13.2 上
OpenCV 4.10/4.13 的 cudev 模块编不过（libcu++ tuple 不兼容，需手改
`cudev/common.hpp` + `cudev/ptr2d/zip.hpp`）。编译时装进 venv 即可
（`-D OPENCV_PYTHON3_INSTALL_PATH=~/robocup-venv/lib/python3.12/site-packages`），
**不需要删系统 OpenCV**——venv 本来就不看 `/usr/lib/python3/dist-packages`。

先按 `probe.py` + `run_robot.py --headless` 打印的 `fps=` 判断是否值得编：
控制环只有 10Hz，全链 ≥15 FPS 就不用折腾。

## 运行

```bash
python run_robot.py               # 相机 + 调试窗口
python run_robot.py --headless    # 板子无显示器（实车标准）
python run_robot.py --video x.mp4 # 视频回放调试
python run_robot.py --width-640   # CPU 紧张时降分辨率
python run_robot.py --no-serial   # 不开串口（纯视觉调试）
```

参数 env 覆盖：`STEP_LEN_CM`（机器人步长）、`PREVIEW_GAIN`（一步前瞻增益）、
`SERIAL_PORT` 等，见 run_robot.py 头注释。

## 板子验收步骤

1. `python probe.py` → CUDA 设备、`cv2.cuda` 可用性、每算子计时表
2. `python compare_cpu_gpu.py` → GPU vs CPU 同帧对比
   （adaptiveThreshold 允许 ≤0.1% 像素差；形态学/阈值应完全一致）
3. `python run_robot.py --headless --no-serial` → 全链帧率（960×540 预处理 <5ms）
4. 接串口实车小跑

## 回退

OpenCV 无 CUDA 时 backend 自动走 CPU 回退（逐算子与 CPU 版等价），程序不崩。
