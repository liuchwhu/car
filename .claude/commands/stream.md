Start the MJPEG camera stream on the Jetson and tell me the URL to view it.

Steps:
1. Read credentials from `.env` (JETSON_HOST, JETSON_USER, JETSON_PASS)
2. SCP `scripts/stream_camera.py` to the Jetson home directory
3. SSH into the Jetson and kill any existing stream process: `pkill -f stream_camera.py`
4. Start the stream in background: `nohup python3 -u ~/stream_camera.py > /tmp/stream.log 2>&1 &`
5. Wait 8 seconds, then check `/tmp/stream.log` to confirm it started
6. Tell me the URL: `http://<JETSON_HOST>:8080`

Use `expect` for SSH (no sshpass on macOS). Use `python3` on Jetson (NOT `python`).
