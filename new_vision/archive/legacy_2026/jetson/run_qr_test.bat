@echo off
cd /d "%~dp0.."
echo ============================================
echo   USB Camera QR Test (OpenCV 640x480)
echo   Hold QR code in front of camera
echo   Press ESC to quit
echo ============================================
.venv\Scripts\python jetson\usb_cam_qr_test.py
pause
