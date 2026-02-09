Emergency stop — kill all running scripts on the Jetson and send neutral commands to the servo and ESC.

Steps:
1. Read credentials from `.env` (JETSON_HOST, JETSON_USER, JETSON_PASS)
2. SSH into the Jetson using `expect`
3. Kill any running Python scripts: `pkill -f wander_demo; pkill -f stream_camera; pkill -f test_`
4. Send neutral commands to hardware:
   ```
   python3 -c "
   import board, busio
   from adafruit_servokit import ServoKit
   i2c = busio.I2C(board.SCL_1, board.SDA_1)
   kit = ServoKit(channels=16, i2c=i2c)
   kit.servo[0].angle = 75
   kit.continuous_servo[1].throttle = 0.0
   print('STOPPED')
   "
   ```
5. Confirm the car is stopped

Use `expect` for SSH (no sshpass on macOS). Use `python3` on Jetson.
