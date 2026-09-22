"""USB摄像头 QR 实测 — raw + 2x upscale 双策略"""
import cv2
import time
import numpy as np

# ── 可调参数 ──
CAM_IDX = 0          # 0=内置 1=USB（不确定就试）
CAM_W = 1280
CAM_H = 720
COOLDOWN_MS = 2000

detector = cv2.QRCodeDetector()
last_qr = None
last_qr_n = 0
last_send_ms = 0
fps_t0 = time.time()
fps_n = 0
fps_val = 0.0


def _try_decode(img_gray):
    try:
        data, pts, _ = detector.detectAndDecode(img_gray)
        if pts is not None and data:
            data = data.strip()
            if data in ("1","2","3","4","5","6"):
                return data, pts
    except cv2.error:
        pass
    return None, None


def decode_multi(bgr):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    # S1: raw
    data, pts = _try_decode(gray)
    if data: return data, pts, "raw"

    # S2: 2x upscale
    h, w = gray.shape[:2]
    up = cv2.resize(gray, (w * 2, h * 2), interpolation=cv2.INTER_LANCZOS4)
    data, pts = _try_decode(up)
    if data and pts is not None:
        pts = pts * 0.5
        return data, pts, "upscale"

    return None, None, None


cap = cv2.VideoCapture(CAM_IDX, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

print(f"USB Camera QR Test  ({actual_w}x{actual_h})  raw + 2x upscale")
print("Hold QR code in front of camera. Press ESC to quit.\n")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    data, pts, strategy = decode_multi(frame)
    now = int(time.time() * 1000)

    fps_n += 1
    if fps_n % 30 == 0:
        fps_val = 30 / max(time.time() - fps_t0, 1e-3)
        fps_t0 = time.time()

    if data:
        box = pts.astype(int).reshape(-1, 2)
        xs, ys = box[:, 0], box[:, 1]
        qr_w, qr_h = int(xs.max() - xs.min()), int(ys.max() - ys.min())

        cv2.polylines(frame, [box], True, (0, 255, 0), 2)
        cv2.putText(frame, f"QR={data}  {qr_w}x{qr_h}px  [{strategy}]  FPS={fps_val:.0f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if data == last_qr:
            last_qr_n += 1
        else:
            last_qr = data
            last_qr_n = 1

        if last_qr_n >= 1 and (now - last_send_ms) >= COOLDOWN_MS:
            print(f"  >>> QR={data}  size={qr_w}x{qr_h}px  strategy={strategy}")
            last_send_ms = now
            last_qr_n = 0
    else:
        cv2.putText(frame, f"No QR  FPS={fps_val:.0f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        last_qr = None
        last_qr_n = 0

    cv2.imshow("USB Camera QR Test", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
