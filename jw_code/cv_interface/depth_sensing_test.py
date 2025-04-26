import cv2
import pyrealsense2 as rs
import numpy as np


def pixels_to_coordinates(pixelx, pixely):
    """takes the x and y pixel indices from camera and returns
    the coordinates in ur5 frame of reference"""
    ppm = 671.9
    robotx = (pixely-23.5)/ppm
    roboty = (pixelx-335)/ppm
    return robotx, roboty

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

# Define crop boundaries (top, bottom, left, right)
y1, y2 = 90, 390    # vertical range
x1, x2 = 170, 470   # horizontal range


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
        print(depth_colormap.shape)
        print(depth_normalized)
        print(depth_normalized.shape)

        kernel = np.ones((5,5),np.uint8)
        # bounds = ([],[])
        # mask = cv2.inRange(hsv, bounds[0], bounds[1])
        # mask = cv2.inRange(depth_normalized, 135, 210)
        depth_blurred = cv2.GaussianBlur(depth_normalized, (5, 5), 0)
        # Crop the depth map
        depth_crop = depth_blurred[y1:y2, x1:x2]

        # Crop the color image (if you want to overlay results later)
        color_crop = color_image[y1:y2, x1:x2]
        

        # mask = cv2.inRange(depth_blurred, 0, 122)
        mask = cv2.inRange(depth_crop, 0, 124)
        print(mask)
    
        # closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations = 1)
        # Morphological operations
        kernel = np.ones((5, 5), np.uint8)
        closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel, iterations=2)

        results = []
        # Filter out small areas
        contours, _ = cv2.findContours(opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_copy = []
        for c in contours:
            if cv2.contourArea(c) < 3000:
                cv2.drawContours(opening, [c], -1, 0, -1)  # fill with black
            else:
                contours_copy.append(c)
        
        # contours, _ = cv2.findContours(closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours_copy) > 0:
                c = max(contours_copy, key = cv2.contourArea)
                
                # compute the center of the contour
                M = cv2.moments(c)
                if M["m10"] == 0 or M["m00"] == 0 or M["m01"] == 0 or M["m00"] == 0:
                    break
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # draw the contour and center of the shape on the image
                cv2.drawContours(opening, [c], -1, 0, 2)
                cv2.circle(opening, (cX, cY), 7, 0, -1)
                cv2.putText(opening, 'centroid', (cX - 20, cY - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                theta = 0.5*np.arctan2(2*M["mu11"],M["mu20"]-M["mu02"])
                startX = int(cX - 200 * np.cos(theta)) 
                startY = int(cY - 200 * np.sin(theta))
                endX = int(200 * np.cos(theta) + cX) 
                endY = int(200 * np.sin(theta) + cY)
                cv2.line(opening, (startX, startY), (endX,endY), (0,0,255), 6)
                realX, realY = pixels_to_coordinates(cX, cY)
                results.append(("clear", realX, realY, theta))

        contour_overlay = color_crop.copy()
        cv2.drawContours(contour_overlay, contours_copy, -1, (0, 255, 0), 2)  # green contours

        # 5. Optional: combine mask and color image visually
        # mask_color = cv2.cvtColor(closing, cv2.COLOR_GRAY2BGR)  # convert binary mask to 3-channel
        mask_color = cv2.cvtColor(opening, cv2.COLOR_GRAY2BGR)
        stacked = np.hstack((contour_overlay, mask_color))

        # 6. Show the result
        cv2.imshow('Color Image', color_image)
        cv2.imshow(f'Depth Image ({depth_min}-{depth_max} m)', depth_blurred)
        cv2.imshow("Depth Range Detection", stacked)

        # Display
        # cv2.imshow('Color Image', color_image)
        # cv2.imshow(f'Depth Image ({depth_min}–{depth_max} m)', depth_colormap)

        # Exit on ESC
        if cv2.waitKey(1) == 27:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()