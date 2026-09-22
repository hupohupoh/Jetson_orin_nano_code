# OpenMV H7 Plus 参数与拍照指南

## 1. 放在一个地方的核心参数

本节把你当前项目最关键、最常用的参数放在一起，便于统一查看和对齐。

### 1.1 你项目当前在用的视觉几何参数
来源文件：line_follow_params.json

- camera.height_cm: 40.0
- camera.pitch_deg: 30.0
- camera.hfov_deg: 70.0
- camera.vfov_deg: 52.0
- base_frame.width: 160
- base_frame.height: 120

说明：
- 这组参数是控制算法里的几何模型参数。
- 如果你后续做了真实相机标定，应优先用标定结果更新 hfov/vfov。

### 1.2 OpenMV H7 Plus 在官方固件中的有用信息（用于初值）

来自 OpenMV 官方仓库（以 OMV4P 板级配置和 OV5640 驱动为主）：

- 板型标识：OMV4P H7 32768 SDRAM
- OV5640 传感器启用
- OV5640 自动对焦启用
- OV5640 PLL 参数：0x64 / 0x13（板级默认配置）
- OV5640 驱动中的有效成像尺寸：2592 x 1944（由 2624 x 1964 扣除 blank/dummy 后）
- OV5640 默认时钟宏：24MHz（若板级未覆盖）

官方参考链接：
- https://github.com/openmv/openmv/blob/main/boards/OPENMV4P/board_config.h
- https://github.com/openmv/openmv/blob/main/boards/OPENMV4P/board_config.mk
- https://github.com/openmv/openmv/blob/main/drivers/sensors/ov5640.c
- https://github.com/openmv/openmv/blob/main/drivers/sensors/sensor_config.h
- https://github.com/openmv/openmv/blob/main/scripts/examples/01-Camera/08-Readout-Control/apriltag_tracking.py

## 2. 你问的重点：OpenMV 里怎么拍照，不是录视频

OpenMV IDE 运行脚本时默认会实时显示帧缓冲，看起来像“录视频”。
但你完全可以把当前帧当照片保存。

有两种方式：

### 2.1 手动拍照（推荐你现在用这个）

使用本仓库脚本：openmv/openmv_capture_photos_manual.py

功能：
- 手动触发，一次一张
- 支持板载按键触发（如果板子支持）
- 支持 IDE 终端按 Enter 触发
- 自动保存到相机存储（优先 SD 卡，其次内部 flash）

步骤：
1. 打开 OpenMV IDE，连接相机。
2. 运行 openmv_capture_photos_manual.py。
3. 触发拍照：按板载键，或者在 IDE 终端按 Enter。
4. 终端出现 saved: 路径即表示该张已保存。

适合你边移动棋盘格边精确控制每一张样本质量。

### 2.2 自动连拍（推荐做标定）

使用本仓库脚本：openmv/openmv_capture_photos.py

功能：
- 自动间隔拍照
- 自动保存到相机存储（优先 SD 卡，其次内部 flash）
- 文件名按编号递增

运行后会在 IDE 终端打印每张照片保存路径。

## 3. 标定拍照建议（高成功率）

- 建议张数：25 到 40 张
- 棋盘格要覆盖画面中心和四角
- 角度要有俯仰、左右倾斜、远近变化
- 分辨率必须与之后算法运行时一致
- 避免虚焦、拖影、过曝、强反光

## 4. 典型流程（最短路径）

1. 用 openmv_capture_photos_manual.py 或 openmv_capture_photos.py 拍 30 张标定图。
2. 导出到电脑，用 OpenCV 标定得到 fx/fy/cx/cy 和畸变。
3. 用标定结果更新 line_follow_params.json 的 hfov/vfov（或直接用 fx/fy 进行几何换算）。
4. 用地面标尺做仿真与实机对齐验证。

完成后，你的仿真相机和真实相机会明显更一致。

## 5. OpenCV 标定脚本（已就绪）

脚本 1：计算内参和畸变
- scripts/calibrate_camera_opencv.py

示例命令：

python scripts/calibrate_camera_opencv.py --images "calib_photos/*.jpg" --cols 9 --rows 6 --square-size-mm 20 --output generated/camera_calibration_result.json --debug-dir generated/calib_debug

输出内容：
- camera_matrix（内参矩阵）
- dist_coeffs（畸变参数）
- fx/fy/cx/cy
- hfov/vfov
- 重投影误差（RMS 和每张误差）

脚本 2：把结果写回 line_follow_params.json
- scripts/apply_calibration_to_line_follow_params.py

示例命令：

python scripts/apply_calibration_to_line_follow_params.py --calib-json generated/camera_calibration_result.json --params-json line_follow_params.json

该脚本会更新：
- camera.hfov_deg
- camera.vfov_deg
- camera.calibration（保存 fx/fy/cx/cy、dist_coeffs、误差和来源）

## 6. Webots 自动调参（黑箱搜索）

脚本：
- scripts/auto_tune_webots_params.py

作用：
- 自动采样一组参数
- 运行 Webots 一小段时间
- 从控制器日志里提取偏差/丢线/转向饱和指标
- 计算综合分数并选最优参数

示例命令（先只评估，不覆盖原参数）：

python scripts/auto_tune_webots_params.py --trials 24 --run-seconds 16 --webots-cmd "webots --batch --mode=fast --stdout --stderr \"{world}\""

示例命令（评估后自动应用最优参数）：

python scripts/auto_tune_webots_params.py --trials 24 --run-seconds 16 --webots-cmd "webots --batch --mode=fast --stdout --stderr \"{world}\"" --apply-best

输出：
- generated/auto_tune_result.json（每轮分数、最优参数、Top5）

备注：
- 该脚本依赖控制器周期日志（th/steer/ex/lost/conf 等字段）进行评分。
- 如果本机 Webots 命令行参数和示例不同，替换 --webots-cmd 即可。