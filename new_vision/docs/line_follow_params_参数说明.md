# line_follow_params.json 参数说明与调参建议

本文档对应配置文件 line_follow_params.json，适用于：
- OpenMV 主程序：`openmv/main_webots_aligned.py`（刷机见 `OpenMV_flash/main.py`）
- Webots 迁移控制器：`Webots/controllers/line_follow_transfer/line_follow_transfer.py`

目标是做到同一套参数在实机和仿真里共用，减少参数漂移。

## 1. 调参顺序建议（先后很重要）

1. threshold：先把黑线分割稳定。
2. roi：再让各层 ROI 都能稳定取到中线。
3. fusion：再调融合后的误差响应速度和抗抖。
4. pid：最后调转向闭环稳定性。
5. steer / lost / webots：做边界行为和速度上限微调。

## 2. 顶层字段

| 字段 | 含义 | 调参建议 |
| --- | --- | --- |
| version | 配置版本号，仅用于标识 | 每次大改参数可以递增版本，便于回滚 |

## 3. base_frame（基准分辨率）

| 字段 | 含义 | 调参建议 |
| --- | --- | --- |
| base_frame.width | ROI 参数的基准宽度（像素） | 当前应与 ROI 设计基准一致（160） |
| base_frame.height | ROI 参数的基准高度（像素） | 当前应与 ROI 设计基准一致（120） |

说明：
- OpenMV 直接按该分辨率使用 ROI 像素坐标。
- Webots 控制器会把 roi.rois_px 按 base_frame 比例映射到当前相机分辨率。

## 4. camera（相机几何模型）

| 字段 | 含义 | 增大时效果 | 减小时效果 | 建议 |
| --- | --- | --- | --- | --- |
| camera.pitch_deg | 相机俯仰角（度） | 估计前方距离更近 | 估计前方距离更远 | 与实际安装角度匹配，优先实测 |
| camera.height_cm | 相机离地高度（厘米） | 同像素对应的地面距离更远 | 更近 | 实测为准，误差会影响距离门控 |
| camera.vfov_deg | 垂直视场角（度） | 距离换算对行号更敏感 | 敏感度下降 | 取相机真实参数 |
| camera.hfov_deg | 水平视场角（度） | 像素偏差换算成厘米更大 | 更小 | 取相机真实参数 |
| camera.row_dist_min_cm | 行距离下限裁剪 | 近处估计更保守 | 近处估计更激进 | 防止异常几何爆点 |
| camera.row_dist_max_cm | 行距离上限裁剪 | 远处估计更保守 | 远处估计截断更早 | 防止远处噪声主导 |

## 5. turn（入弯门控）

| 字段 | 含义 | 增大时效果 | 减小时效果 | 建议 |
| --- | --- | --- | --- | --- |
| turn.start_dist_cm | 开始判断提前入弯的距离阈值 | 更早进入弯道策略 | 更晚进入弯道策略 | 直线误入弯则减小，入弯慢则增大 |
| turn.end_dist_cm | 弯道距离归一化终点 | 门控持续更久 | 门控更短 | 一般小于 start_dist_cm |
| turn.err_cm | 将远端偏差归一化的尺度 | 门控触发更迟钝 | 门控触发更灵敏 | 抖动大就增大，反应慢就减小 |

## 6. roi（采样区域）

### 6.1 roi.rois_px

格式为数组，每项是 [x, y, w, h, weight]：
- x, y：ROI 左上角像素坐标（基于 base_frame）
- w, h：ROI 宽高（像素）
- weight：该 ROI 在融合中的权重

当前三层语义：
- near（近端）：抑制抖动
- mid（中端）：主控
- far（远端）：预判

调参建议：
- 看不见线：先增大对应 ROI 的 h 或下移 y。
- 转弯提前量不足：提高 far 权重或上移 far ROI。
- 直线左右抖：降低 near 权重，适当提高 mid 权重。

### 6.2 其他 roi 字段

| 字段 | 含义 | 调参建议 |
| --- | --- | --- |
| roi.scan_lines_per_roi | 每层 ROI 内扫描行数 | 增大更稳但更慢；建议 4 到 8 |
| roi.min_track_width | 允许的最小线宽（像素） | 线被漏检可减小；误检多可增大 |
| roi.max_track_width | 允许的最大线宽（像素） | 粗线漏检可增大；背景误检可减小 |
| roi.min_valid_lines | ROI 判定有效的最少行数 | 抗噪提升可增大；漏检时减小 |
| roi.width_std_max | 线宽标准差上限，用于置信度衰减 | 抖动大可增大；想更严格可减小 |
| roi.conf_min | ROI 最低置信度阈值 | 漏检时减小；噪声多时增大 |

## 7. threshold（二值阈值）

| 字段 | 含义 | 调参建议 |
| --- | --- | --- |
| threshold.offset | Otsu 基础上附加偏移 | 黑线偏灰时增大；背景误检时减小 |
| threshold.min | 阈值下限 | 光照极暗时保护下限 |
| threshold.max | 阈值上限 | 光照极亮时保护上限 |

经验：
- 先看黑线是否连贯，再看背景是否被吞噬。
- offset 通常是首调参数。

## 8. preprocess（预处理）

| 字段 | 含义 | 调参建议 |
| --- | --- | --- |
| preprocess.use_histeq | 是否做直方图均衡 | 光照不均明显时开启 |
| preprocess.use_binary | 是否在前处理中做二值化 | 一般保持 false，避免过早丢信息 |
| preprocess.use_erode | 是否做腐蚀 | 孤立噪点多时开启 |
| preprocess.erode_iter | 腐蚀迭代次数 | 过大易把细线腐蚀断 |
| preprocess.enable_debug_draw | 是否绘制调试图层 | 调参阶段开，比赛时可关 |

## 9. fusion（误差融合）

| 字段 | 含义 | 增大时效果 | 减小时效果 | 建议 |
| --- | --- | --- | --- | --- |
| fusion.smooth_alpha | 误差一阶平滑系数 | 更稳、更慢 | 更快、更抖 | 抖动大就增大 |
| fusion.curve_gain | 曲率项权重 | 对弯道更敏感 | 对弯道更钝 | 直道误摆就减小 |
| fusion.angle_gain | 角度项权重 | 朝向修正更强 | 更依赖中心偏差 | 角度抖动大就减小 |
| fusion.min_weight | 最小有效总权重 | 过滤更严格 | 更容易输出控制 | 漏检多时减小 |
| fusion.lookahead_gain | 远端前瞻权重 | 入弯更早 | 入弯更晚 | 过冲则减小，入弯迟缓则增大 |

## 10. pid（双模 PID）

| 字段 | 含义 | 调参建议 |
| --- | --- | --- |
| pid.curve_switch_cm | 直道/弯道 PID 切换阈值 | 增大后更常用直道参数 |
| pid.straight.kp | 直道比例项 | 过小跟踪慢，过大抖动 |
| pid.straight.ki | 直道积分项 | 用于消除小偏差，过大易积累震荡 |
| pid.straight.kd | 直道微分项 | 抑制快速变化，过大易噪声放大 |
| pid.curve.kp | 弯道比例项 | 决定弯道转向力度 |
| pid.curve.ki | 弯道积分项 | 建议小于直道，防止弯中积累 |
| pid.curve.kd | 弯道微分项 | 弯道抑振，防止过冲 |
| pid.i_clamp | 积分限幅 | 必须保留，防积分饱和 |

## 11. steer（转向映射）

| 字段 | 含义 | 调参建议 |
| --- | --- | --- |
| steer.sat | 非线性映射饱和值 | 上限越大，极端转向越强 |
| steer.scale | tanh 缩放尺度 | 大则更平缓，小则更激进 |
| steer.deadband | 小误差死区 | 大则更稳但更钝，小则更灵敏 |

## 12. lost（丢线恢复）

| 字段 | 含义 | 调参建议 |
| --- | --- | --- |
| lost.hold_frames | 丢线后保持上一转向的帧数 | 增大可减轻瞬时噪声影响 |
| lost.search_turn | 超时后搜索转向幅度 | 增大可更快找线，但风险是摆头过猛 |

## 13. output（输出与通信）

| 字段 | 含义 | 调参建议 |
| --- | --- | --- |
| output.send_interval_ms | 输出节流周期（毫秒） | 小则控制更新快，大则更省带宽 |
| output.uart.id | UART 端口号 | 与硬件接线一致 |
| output.uart.baud | 串口波特率 | 与下位机配置一致 |
| output.route.go | 直行指令码 | 与接收端协议一致 |
| output.route.left | 左转指令码 | 与接收端协议一致 |
| output.route.right | 右转指令码 | 与接收端协议一致 |
| output.route.slight_left | 微左指令码 | 与接收端协议一致 |

## 14. webots（仿真专用）

| 字段 | 含义 | 调参建议 |
| --- | --- | --- |
| webots.base_speed | Webots 轮速基值 | 太大容易冲出赛道，太小响应慢 |
| webots.steer_to_wheel | 转向量映射到左右轮差速比例 | 太大易过冲，太小转不过弯 |
| webots.camera.width | 仿真相机宽度配置值 | 保持与世界文件一致 |
| webots.camera.height | 仿真相机高度配置值 | 保持与世界文件一致 |
| webots.camera.field_of_view | 仿真相机 FOV 配置值 | 保持与世界文件一致 |

补充：
- 控制器代码支持可选字段 webots.max_speed（当前 JSON 未配置时默认 6.28）。

## 15. 快速排障对照

- 症状：直线抖动大
  - 优先调：fusion.smooth_alpha、pid.straight.kd、steer.scale
- 症状：弯道进不去
  - 优先调：fusion.lookahead_gain、pid.curve.kp、webots.steer_to_wheel
- 症状：弯道过冲
  - 优先调：pid.curve.kd、fusion.curve_gain、steer.sat
- 症状：偶发丢线后大幅摆头
  - 优先调：lost.hold_frames、lost.search_turn、roi.conf_min
- 症状：不同光照下识别不稳
  - 优先调：threshold.offset、threshold.min/max、preprocess.use_histeq

## 16. 建议的调参记录格式

每次改参数建议只改一组，并记录：
- 日期与场地光照
- 修改字段与修改前后值
- 现象变化（直线、入弯、出弯、丢线恢复）
- 是否保留该改动

这样能快速形成可复现的参数演进历史。