Stop the MJPEG camera stream running on the Jetson.

Steps:
1. Read credentials from `.env` (JETSON_HOST, JETSON_USER, JETSON_PASS)
2. SSH into the Jetson using `expect`
3. Kill the stream process: `pkill -f stream_camera.py`
4. Confirm it's stopped: `pgrep -f stream_camera.py` should return nothing
5. Report the result

Use `expect` for SSH (no sshpass on macOS).
