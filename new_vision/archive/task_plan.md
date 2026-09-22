# RoboCup CV / OpenMV / 仿真 — 任务计划

> **注意**：本文件是规划草稿，**不一定反映最新进度**。请以 **`git log`、`OpenMV_flash/`、根目录 `line_follow_params.json`** 为准；细则见同目录 `findings.md` 开头说明。
>
> 导师要求摘要：**大倾角鲁棒**、**低延迟（0.5 s 量级优于 1 s）**、**提前约 20 cm 能稳检**；测试是为了找问题，不是声称没问题。  
> 规则 PDF：**`rule.pdf`**（仓库根目录）；与 QR/视觉相关的正式条款摘录见 **`findings.md`**（本目录）。

---

## 阶段 A — 目录与刷机（已完成）

- [x] 建立 `OpenMV_flash/`，仅含 `main.py`、`protocol_v2.py`、`line_follow_params.json` + `README.md`
- [x] 旧 `CVpart/openmv_flash_package/` 改为指向新目录（见该目录 `README.txt`）

## 阶段 B — Webots 与参数

- [x] Webots 路径：`D:\Webots\msys64\mingw64\bin\webots.exe`（R2025a）；`run_shake_eval.py --run-seconds 12` 冒烟已通过
- [ ] 按需跑完整时长（如 18～25 s）或 `auto_tune_webots_params.py` 做参数搜索
- [ ] 新相机标定已进根目录 `line_follow_params.json`；复核 `scripts/check_camera_consistency.py` → PASS
- [ ] 按需微调仿真侧 ROI / PID / `shake.*`（以短跑对照为主，不必从零重跑全部历史 sweep）
- [ ] 将**确定上场**的键同步到 `OpenMV_flash/line_follow_params.json`（保持 `sim_opencv_distort: false`）

## 阶段 C — 二维码：指标与方案

**物理/相机（当前配置口径）**

- `line_follow_params.json`：`height_cm=40`，`pitch_deg=45`；标定分辨率 320×240；`hfov`/`vfov` 与内参见 `camera.calibration`
- OpenMV：QVGA 流程与 `lens_corr` 已在 `qr_uart_test.py` 使用；真机畸变与仿真 OpenCV 后畸变是两条线，以**真场测试**为准

**工程指标（对应导师质疑）**

| 维度 | 目标 | 验证方法 |
|------|------|----------|
| 倾角 | 斜持仍能稳定解码 | 固定距离，扫 tilt 0°~45°（或规则上限），统计成功率与耗时 |
| 延迟 | 识别到发出指令 **≤0.5 s**（力争；可先量 1 s 再压） | 打日志时间戳或 GPIO；多帧均值 |
| 提前量 | 较「贴到最近」再早 **~20 cm** 仍可检 | 钢卷尺/地贴标记；录视频对帧 |

**技术选项（逐项小步试）**

1. **分辩率与 ROI**：全程 QVGA 或窗口 ROI 放大码区；权衡 fps 与模块像素数
2. **`lens_corr` 强度**：与距离/倾角联合扫参
3. **状态机**：接近标志位 → 提高 QR 帧预算或短暂降巡线频率
4. **备选**：若 QR 仍不稳，评估 AprilTag / 更大数据字库（规则允许前提下）

**风险**：无法在理论上「保证」任意远+任意倾角必解；以统计通过率 + 最坏工况录像为交付。

## 阶段 D — 规则对齐

- [ ] PDF 入场后：摘录 QR/尺寸/距离/禁止事项 → `findings.md`
- [ ] 调整场地标定与评分相关的测试用例

## 阶段 E — Jetson Nano + USB 摄像头视觉方案（进行中）

- [x] 架构规划：QR + 巡线（鸟瞰）+ 红条（HSV）
- [x] QR 模块、巡线模块、USB 测试脚本
- [x] Webots 控制器（共用 jetson_vision 模块）
- [x] 规则 PDF 场地图数据提取验证（3660×2440mm, 6140mm, 350mm, 240mm门）
- [x] 项目架构诊断：两份代码、过时文件、OpenMV 硬伤 → 记录到 findings.md
- [ ] Windows 上验证 USB 摄像头 QR 检测
- [ ] 相机畸变标定 + Webots 巡线仿真验证
- [ ] UART 串口联通 STM32（协议 V2）
- [ ] Jetson Nano 实机部署测试

---

## 当前优先级建议

1. **现在**：Windows 上跑 `usb_cam_qr_test.py` 验证 QR 距离
2. **然后**：相机标定 → 巡线 + 红条模块 → 串口联调
3. **保底**：OpenMV 方案继续可用，QR 参数已调好
