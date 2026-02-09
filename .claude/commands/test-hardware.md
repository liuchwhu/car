Run the hardware test on the Jetson to verify steering and throttle.

**IMPORTANT: Make sure the car is on a stand with wheels off the ground before running!**

Steps:
1. Read credentials from `.env` (JETSON_HOST, JETSON_USER, JETSON_PASS)
2. SCP `scripts/test_hardware.py` to the Jetson home directory
3. SSH into the Jetson using `expect`
4. Run: `python3 ~/test_hardware.py`
5. Report results: steering mapping, dead zones, steering under throttle

Use `python3` on Jetson (NOT `python`). Use `expect` for SSH. Set timeout to 120s since the test takes ~30 seconds.
