import cv2
import pyrealsense2 as rs
import numpy as np

# Initialize pipeline and config
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Align depth to color
align_to = rs.stream.color
align = rs.align(align_to)

# Set your area of interest in meters
depth_min = 0.8  # 80 cm
depth_max = 1.0  # 100 cm

try:
    while True:
        # Wait for a new frame
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Convert depth to meters
        depth_meters = depth_image * depth_scale

        # Clip to range of interest (0.8m to 1.0m)
        depth_clipped = np.clip(depth_meters, depth_min, depth_max)

        # Normalize clipped depth to 0–255
        depth_normalized = cv2.normalize(
            depth_clipped, None, 0, 255, cv2.NORM_MINMAX
        )

        # Convert to 8-bit and apply color map
        depth_colormap = cv2.applyColorMap(
            depth_normalized.astype(np.uint8), cv2.COLORMAP_JET
        )

        # Display
        cv2.imshow('Color Image', color_image)
        cv2.imshow(f'Depth Image ({depth_min}–{depth_max} m)', depth_colormap)

        # Exit on ESC
        if cv2.waitKey(1) == 27:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
