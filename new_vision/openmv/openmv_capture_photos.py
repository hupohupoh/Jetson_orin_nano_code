import sensor  # type: ignore
import time
import os
from pyb import LED  # type: ignore

# -------------------- User settings --------------------
PIXFORMAT = sensor.RGB565
FRAMESIZE = sensor.QVGA
TOTAL_IMAGES = 30
CAPTURE_INTERVAL_MS = 700
WARMUP_MS = 2000
JPEG_QUALITY = 95
SUBDIR_NAME = "calib_photos"
# ------------------------------------------------------


def pick_base_dir():
    for base in ("/sd", "/flash"):
        try:
            os.listdir(base)
            return base
        except OSError:
            pass
    raise OSError("Neither /sd nor /flash is available")


def ensure_dir(path):
    parts = [p for p in path.split("/") if p]
    cur = ""
    for p in parts:
        cur += "/" + p
        try:
            os.mkdir(cur)
        except OSError:
            pass


sensor.reset()
sensor.set_pixformat(PIXFORMAT)
sensor.set_framesize(FRAMESIZE)
sensor.skip_frames(time=WARMUP_MS)

# For calibration photos, disable auto controls for consistency.
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()
led_g = LED(2)
led_b = LED(3)

base_dir = pick_base_dir()
save_dir = base_dir + "/" + SUBDIR_NAME
ensure_dir(save_dir)

print("Capture start")
print("save_dir:", save_dir)

last_ms = time.ticks_ms()
count = 0

while count < TOTAL_IMAGES:
    clock.tick()
    img = sensor.snapshot()

    now = time.ticks_ms()
    if time.ticks_diff(now, last_ms) >= CAPTURE_INTERVAL_MS:
        filename = "%s/img_%03d.jpg" % (save_dir, count)
        img.save(filename, quality=JPEG_QUALITY)
        print("saved:", filename)

        led_g.on()
        time.sleep_ms(60)
        led_g.off()

        count += 1
        last_ms = now

    if (count % 5) == 0:
        led_b.toggle()

print("Capture done, total:", TOTAL_IMAGES)
print("FPS:", clock.fps())
