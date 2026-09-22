"""相机/视频源 — Windows(DSHOW) / Linux(V4L2) 自动选择 + 文件回放。

相机模式下采集走独立线程常驻 read()，主线程取最新帧：把等待出帧的时间
（MJPEG 720p 约 33ms）藏进处理时间里，主循环不再串行等待。

用法:
    cam = VisionCamera(cam_idx=0, width=1280, height=720)   # 相机（异步）
    cam = VisionCamera(video_path="run.mp4")                 # 视频文件（同步）
    ok, frame = cam.read()
"""
import sys
import time
import threading
import cv2


class VisionCamera:
    def __init__(self, cam_idx=0, width=1280, height=720,
                 video_path=None, width_640=False):
        self.video_path = video_path
        self.cap = None
        self._frame = None
        self._seq = 0
        self._last_seq = -1
        self._stop = False
        self._thread = None
        if video_path:
            self.cap = cv2.VideoCapture(video_path)
            self.actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            return
        # Windows 用 DSHOW，Linux (Jetson) 用 V4L2
        api = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2
        self.cap = cv2.VideoCapture(cam_idx, api)
        if width_640:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        else:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        # 顺序不能反：DSHOW 下若先设 FOURCC 再设分辨率，后者会触发驱动重新协商
        # 并把 MJPG 踢回默认 YUY2（实测 720p 22fps → 10fps）。V4L2 上顺序无影响。
        # 不支持 MJPG 的相机（多为笔记本内置）会忽略，退回默认格式。
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        # V4L2 偶发第一帧无效，先抓几帧唤醒
        for _ in range(3):
            ok, _ = self.cap.read()
            if ok:
                break
        self._thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._thread.start()
        for _ in range(200):        # 等首帧就绪，最长 2s
            if self._frame is not None:
                break
            time.sleep(0.01)

    def _grab_loop(self):
        while not self._stop:
            ok, f = self.cap.read()
            if ok:
                # GIL 保证引用赋值原子，处理线程只会拿到完整帧
                self._frame = f
                self._seq += 1
            else:
                time.sleep(0.005)

    def read(self):
        """取最新帧；返回 (False, None) 表示自上次调用后没有新帧。

        返回的数组由采集线程持有并被下一帧替换，调用方不要原地修改。
        """
        if self._thread is None:
            return self.cap.read()
        seq = self._seq
        if self._frame is None or seq == self._last_seq:
            return False, None
        self._last_seq = seq
        return True, self._frame

    def release(self):
        self._stop = True
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self.cap is not None:
            self.cap.release()

    @property
    def size(self):
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if w <= 0 or h <= 0:
            w, h = 1280, 720
        return w, h

    def is_opened(self):
        return self.cap is not None and self.cap.isOpened()

    @property
    def fourcc(self):
        """驱动实际生效的像素格式（4 字符），用于确认 MJPG 有没有设上。"""
        if self.cap is None:
            return "----"
        v = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))
