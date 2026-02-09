#!/usr/bin/env python3
"""MJPEG stream from RealSense D435. View at http://<jetson-ip>:8080

Run on Jetson:  python scripts/stream_camera.py
View on Mac:    open http://192.168.1.233:8080
"""

import time
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

import numpy as np
import cv2
import pyrealsense2 as rs

PORT = 8080
FRAME_LOCK = threading.Lock()
latest_jpeg = None


def camera_loop():
    global latest_jpeg

    print("Starting RealSense pipeline (640x480 @ 30fps)...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    align = rs.align(rs.stream.color)

    # Warm up
    for _ in range(5):
        pipeline.wait_for_frames()

    print("Camera ready. Streaming...")
    colormap = cv2.COLORMAP_JET

    try:
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            rgb = np.asanyarray(color_frame.get_data())
            depth_raw = np.asanyarray(depth_frame.get_data())

            # Colorize depth for display (clip to 3m)
            depth_clipped = np.clip(depth_raw, 0, 3000)
            depth_norm = (depth_clipped / 3000.0 * 255).astype(np.uint8)
            depth_color = cv2.applyColorMap(depth_norm, colormap)

            # Side-by-side: RGB | Depth
            combined = np.hstack([rgb, depth_color])

            # Add center distance text
            h, w = depth_raw.shape
            center_depth = depth_raw[h // 2, w // 2] * depth_frame.get_units()
            cv2.putText(combined, "Center: {:.2f}m".format(center_depth),
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            _, jpeg = cv2.imencode('.jpg', combined, [cv2.IMWRITE_JPEG_QUALITY, 70])

            with FRAME_LOCK:
                latest_jpeg = jpeg.tobytes()

    finally:
        pipeline.stop()


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(b'<html><body style="margin:0;background:#000">'
                             b'<img src="/stream" style="width:100%">'
                             b'</body></html>')
            return

        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    with FRAME_LOCK:
                        frame = latest_jpeg
                    if frame is None:
                        time.sleep(0.03)
                        continue
                    self.wfile.write(b'--frame\r\n'
                                    b'Content-Type: image/jpeg\r\n\r\n')
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
                    time.sleep(0.033)  # ~30fps
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        self.send_error(404)

    def log_message(self, format, *args):
        pass  # Quiet logging


if __name__ == '__main__':
    cam_thread = threading.Thread(target=camera_loop, daemon=True)
    cam_thread.start()

    print("MJPEG server on http://0.0.0.0:{}".format(PORT))
    print("Press Ctrl+C to stop.\n")
    server = HTTPServer(('0.0.0.0', PORT), MJPEGHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
