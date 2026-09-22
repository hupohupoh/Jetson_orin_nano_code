# Vision <-> MainBoard 通讯协议 V2（P1 清爽执行版）

## 1. 这版文档只回答三件事

1. 视觉到底要发什么。
2. 电机组每步怎么用。
3. 哪些内容现在不启用，避免过度实现。

## 2. P1 一句话结论

- 主执行链路：route + conf + lost（离散优先）。
- 建议辅助链路：ex + ang（用于阈值和步态细调）。
- 预留链路：v_cmd + w_cmd（本版可发 0，不作为必需）。

## 3. 物理层

- UART1，115200，8N1。
- Little Endian。

## 4. 统一帧格式

| 字段 | 长度(字节) | 说明 |
|---|---:|---|
| SOF1 | 1 | 0x55 |
| SOF2 | 1 | 0xAA |
| VER | 1 | 当前 0x02 |
| MSG_TYPE | 1 | 消息类型 |
| FLAGS | 1 | ACK_REQ / IS_ACK |
| SEQ | 1 | 0~255 循环 |
| TS_MS | 4 | 毫秒时间戳 |
| LEN | 1 | Payload 长度 |
| PAYLOAD | LEN | 负载 |
| CRC16 | 2 | CRC-16/CCITT-FALSE（VER 到 PAYLOAD） |

CRC 参数：poly=0x1021, init=0xFFFF, xorout=0x0000。

## 5. 消息与发送规则

### 5.1 高频流

| MSG_TYPE | 方向 | 名称 | 周期 |
|---:|---|---|---|
| 0x01 | Vision->Main | HEARTBEAT | 10 Hz |
| 0x02 | Vision->Main | LINE_CTRL | 8~12 Hz（建议 10 Hz） |
| 0x81 | Main->Vision | ROBOT_STATE | 10 Hz |

### 5.2 LINE_CTRL 字段分级（最关键）

| 字段 | 类型 | P1级别 | 电机组用法 |
|---|---|---|---|
| mode_u8 | u8 | 必发 | 步态状态机 |
| route_u8 | u8 | 必发 | 本步方向决策 |
| conf_u8 | u8 | 必发 | 是否降级/停步 |
| lost_u8 | u8 | 必发 | 搜线或停步触发 |
| ex_mm_i16 | i16 | 建议发 | 调试与阈值优化 |
| ang_cdeg_i16 | i16 | 建议发 | 调试与阈值优化 |
| v_cmd_mmps_i16 | i16 | 可选 | 预留，P1 可置 0 |
| w_cmd_mradps_i16 | i16 | 可选 | 预留，P1 可置 0 |

量化建议（降低算力与抖动）：
- ex: 10 mm 一档。
- ang: 1 deg 一档（即 100 cdeg 一档）。
- v_cmd: 20 mm/s 一档。
- w_cmd: 50 mrad/s 一档。
- conf: 5 一档。

## 6. 迈步前采样契约（电机组直接按此实现）

1. 每次迈步前只取最近 1 帧有效 LINE_CTRL。
2. 若一个步周期内收到多帧，可选最近 3 帧 route 多数投票，conf 取中值。
3. 若 lost=1 或 conf<35，本步不前推，只做原地小角度调整或停步。
4. 若 300 ms 内无有效包，进入 SAFE_STOP。

## 7. 事件流（必须可靠）

| MSG_TYPE | 方向 | 名称 | 可靠性 |
|---:|---|---|---|
| 0x10 | Vision->Main | QR_EVENT | ACK + 重发 |
| 0x11 | Vision->Main | OBSTACLE_EVENT | ACK + 重发 |
| 0x12 | Vision->Main | MODE_SWITCH_REQ | ACK + 重发 |
| 0x80 | Main->Vision | ACK/NACK | - |
| 0xE0 | 双向 | ESTOP | ACK + 最高优先级 |

ACK 规则：80 ms 超时重发，最多 3 次。

## 8. 优先级与安全

优先级从高到低：
1. ESTOP
2. OBSTACLE_EVENT
3. QR_EVENT
4. MODE_SWITCH_REQ
5. LINE_CTRL / HEARTBEAT

主控超时建议：
- 300 ms 无有效 HEARTBEAT 或 LINE_CTRL -> SAFE_STOP。
- 800 ms 未恢复 -> LOST_RECOVERY。

## 9. 当前版本“要发的”清单

视觉侧当前必须做到：
1. LINE_CTRL: mode, route, conf, lost（每 8~12 Hz）。
2. HEARTBEAT: 10 Hz。
3. QR_EVENT / OBSTACLE_EVENT: 状态变化触发，ACK 重发。

视觉侧建议做到：
1. LINE_CTRL 附带 ex, ang（调试和优化用）。

视觉侧可暂不启用：
1. v_cmd, w_cmd（可置 0，不影响 P1 比赛流程）。

## 10. 旧协议映射（兼容）

| 旧字符 | 含义 | route_u8 |
|---|---|---:|
| "1" | go | 1 |
| "2" | left | 2 |
| "3" | right | 3 |
| "4" | slight_left | 4 |

## 11. 冻结规则（避免重构）

1. 不改帧结构，不改字段字节布局，不改 CRC。
2. 不改变 route 编码语义（1/2/3/4）。
3. 只允许调整：阈值、量化步长、发送频率、超时阈值。
4. 任意调整必须完成同赛道 3 圈回归。

## 12. 对应配置键

- output.send_interval_ms = 100
- output.protocol.ctrl_hz = 10
- output.protocol.heartbeat_hz = 10
- output.protocol.ack_timeout_ms = 80
- output.protocol.retry_max = 3
- output.protocol.safety_timeout_ms = 300
- output.protocol.step_latch_mode = latest_1_or_recent_3
- output.protocol.ctrl_profile = v2_p1_discrete_first
- output.protocol.continuous_ctrl_enable = false
