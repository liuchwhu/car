Run the camera test on the Jetson to verify the RealSense D435 is working.

Steps:
1. Read credentials from `.env` (JETSON_HOST, JETSON_USER, JETSON_PASS)
2. SCP `scripts/test_camera.py` to the Jetson home directory
3. SSH into the Jetson using `expect`
4. Run: `python3 ~/test_camera.py`
5. Report results: device info, USB type, frame shapes, depth distances, frame timing

Use `python3` on Jetson (NOT `python`). Use `expect` for SSH.
