# Robocup 项目 AI 交接日志

## 1. 项目背景
- 项目目标是做人形小车在 Webots 场景中的巡线控制，当前聚焦于双线赛道边缘识别与稳定转向。
- 近期问题集中在三类：起步阶段异常偏转、弯道内切/外切、速度提升后的稳定性。
- 用户明确要求按第一性原理做减法：less is more，保留有效归纳偏置，删除叠加复杂结构。

## 2. 当前核心目标
- 在保证稳定识别的前提下提升速度。
- 算法尽量简单可解释，优先保留底部双线和中位线主线。
- 参数以 json 为主，代码默认值仅作兜底，避免两套口径冲突。

## 3. 用户偏好与协作要求
- 用户偏好中文沟通。
- 用户不希望“堆机制”，希望直接、简洁、可解释。
- 用户常切窗口继续对话，所以需要高质量上下文日志，便于下一窗口快速接手。

## 4. 当前控制器状态
- 当前控制器文件是 [Webots/controllers/line_follow_transfer/line_follow_transfer.py](../../Webots/controllers/line_follow_transfer/line_follow_transfer.py)。
- 当前实现已经是简化主线版本，关键函数如下：
- 三分区双线扫描与中位线提取在 [Webots/controllers/line_follow_transfer/line_follow_transfer.py](../../Webots/controllers/line_follow_transfer/line_follow_transfer.py#L318)。
- 底部锁定在 [Webots/controllers/line_follow_transfer/line_follow_transfer.py](../../Webots/controllers/line_follow_transfer/line_follow_transfer.py#L398)。
- PID 在 [Webots/controllers/line_follow_transfer/line_follow_transfer.py](../../Webots/controllers/line_follow_transfer/line_follow_transfer.py#L475)。
- 转向斜率限制在 [Webots/controllers/line_follow_transfer/line_follow_transfer.py](../../Webots/controllers/line_follow_transfer/line_follow_transfer.py#L489)。
- 速度缩放函数在 [Webots/controllers/line_follow_transfer/line_follow_transfer.py](../../Webots/controllers/line_follow_transfer/line_follow_transfer.py#L497)。
- 主循环日志输出在 [Webots/controllers/line_follow_transfer/line_follow_transfer.py](../../Webots/controllers/line_follow_transfer/line_follow_transfer.py#L745)。

## 5. 当前参数状态
- 当前参数主文件为 [line_follow_params.json](../../line_follow_params.json)（仓库根目录，Webots/脚本默认读此路径）。
- 速度已按 1.5 倍目标调整：
- base_speed = 9.42，位置 [line_follow_params.json](../../line_follow_params.json#L195)。
- max_speed = 9.42，位置 [line_follow_params.json](../../line_follow_params.json#L197)。
- speed_min = 7.2，位置 [line_follow_params.json](../../line_follow_params.json#L200)。
- 速度保守项已调松：
- bottom_lock_speed_penalty = 0.12，位置 [line_follow_params.json](../../line_follow_params.json#L103)。
- startup_speed_scale = 0.75，位置 [line_follow_params.json](../../line_follow_params.json#L105)。

## 6. 已完成工作摘要
- 协议文档精简，形成可执行分层口径：
- 视觉主控协议见 [docs/vision_main_protocol_v2.md](../vision_main_protocol_v2.md)。
- 电机执行协议见 [docs/motor_protocol_v2_p1.md](../motor_protocol_v2_p1.md)。
- 控制器经历了复杂版到减法版收敛，保留有效主线：双线中位线、底部锁、平滑 PID、丢线恢复。
- 做过多轮参数调优与一致性检查，脚本在 [scripts/iterate_line_follow_params.py](../../scripts/iterate_line_follow_params.py) 与 [scripts/evaluate_line_follow_log.py](../../scripts/evaluate_line_follow_log.py)。

## 7. 当前已知风险
- 提速会缩小视觉与控制容错边界，急弯/遮挡/阈值抖动时更容易短时丢线。
- 目前编辑器中有 controller 导入告警，这是本机缺 Webots 运行时解析导致，不是算法语法错误。
- 若后续又有人手动覆盖控制器文件，可能把已删掉的拓扑和 cut 补偿层重新带回，需要先核对当前文件状态再动手。

## 8. 下一位 AI agent 建议先做的事
- 第一步先读 [Webots/controllers/line_follow_transfer/line_follow_transfer.py](../../Webots/controllers/line_follow_transfer/line_follow_transfer.py) 与 [line_follow_params.json](../../line_follow_params.json)，确认是否仍是简化版。
- 第二步先看日志字段 conf、lost、bp、lock，再判断是识别问题还是控制参数问题。
- 第三步只调整少量一线参数，不新增复杂逻辑，优先动这几个：
- webots.speed_steer_slowdown。
- webots.speed_lost_scale。
- roi.bottom_lock_blend。
- roi.bottom_lock_min_pair_ratio。
- pid.straight 和 pid.curve 的 kp、kd。

## 9. 给新窗口可直接复制的接手提示词
- 你接手的是 Robocup Webots 巡线项目。先读取 [docs/project/AI_log.md](AI_log.md)、[Webots/controllers/line_follow_transfer/line_follow_transfer.py](../../Webots/controllers/line_follow_transfer/line_follow_transfer.py)、[line_follow_params.json](../../line_follow_params.json)。当前策略是 less is more：保留双线中位线和底部锁主线，不新增复杂结构。优先用参数法解决速度与稳定性问题。修改后请给出具体改动行号、预期影响、回滚方案。

## 10. 其他索引
- 任务与阶段规划见同目录 [task_plan.md](task_plan.md)；研究笔记见 [findings.md](findings.md)。
- 本日志文件是 [AI_log.md](AI_log.md)。
