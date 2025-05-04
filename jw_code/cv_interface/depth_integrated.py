import cv2
import pyrealsense2 as rs
import numpy as np
import cv_interface.bottle as bottle
#import bottle

class CVInterface:
    def __init__(self):
        """
        Initialize computer vision interface.
        """
        #self.cap = cv2.VideoCapture(0) # 0 for webcam, 1 for external camera

        # Initialize pipeline and config
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, framerate = 15)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, framerate = 15)

        # Start streaming
        self.profile = self.pipeline.start(config)
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        # Align depth to color
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # Set your area of interest in meters
        self.depth_min = 0.8  # 80 cm
        self.depth_max = 1.0  # 100 cm

        # Define crop boundaries (top, bottom, left, right)
        #y1, y2 = 90, 390    # vertical range
        #x1, x2 = 170, 470   # horizontal range
        
        blue_lower_bound = (80,50,120)
        blue_upper_bound = (110,255,255)
        yellow_lower_bound = (26,20,120)
        yellow_upper_bound = (40,255,255)
        orange_lower_bound = (0,100,120) 
        orange_upper_bound = (20,255,255)

        self.colors = ["blue", "yellow", "orange"] #excludes clear
        self.display_colors = {"blue": (0, 255, 0), "yellow": (0, 250, 250), "orange": (25, 25, 255), "clear": (130, 0, 75)}
        self.color_ranges = {"blue": (blue_lower_bound, blue_upper_bound), 
                             "yellow": (yellow_lower_bound, yellow_upper_bound),
                             "orange": (orange_lower_bound, orange_upper_bound)}
        self.kernel = np.ones((5,5),np.uint8)
        #self.pixels_per_inch = 17
        self.ppm = 671.9
        self.belt_space = (135, 0, 530, 430) #X1, Y1, X2, Y2

    def get_camera_fps(self):
        return self.cap.get(cv2.CAP_PROP_FPS)
    
    def get_camera_resolution(self):
        """
        Get width and height of camera stream
        """
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

        return width, height

    def set_camera_fps(self, fps):
        self.cap.set(cv2.CAP_PROP_FPS, fps)

    def set_camera_resolution(self, width, height):
        """
        Set width and height of camera stream
        """
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def map_cropped_to_original(self, pixelx, pixely):
        left, top, right, bottom = self.belt_space
    
    def pixels_to_coordinates(self, pixelx, pixely):
        """takes the x and y pixel indices from camera and returns
        the coordinates in ur5 frame of reference"""
        robotx = (pixely-23.5)/self.ppm
        #roboty = (pixelx+self.belt_space[0]-335)/self.ppm
        roboty = (pixelx-195)/self.ppm
        return robotx, roboty

    def find_centroid(self, c, display, color):
            # compute the center of the contour
            M = cv2.moments(c)
            if M["m10"] != 0 and M["m00"] != 0 and M["m01"] != 0 and M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # draw the contour and center of the shape on the image
                cv2.drawContours(display, [c], -1, self.display_colors[color], 2)
                cv2.circle(display, (cX, cY), 7, self.display_colors[color], -1)
                
                theta = 0.5*np.arctan2(2*M["mu11"],M["mu20"]-M["mu02"])
                startX = int(cX - 50 * np.cos(theta)) 
                startY = int(cY - 50 * np.sin(theta))
                endX = int(200 * np.cos(theta) + cX) 
                endY = int(200 * np.sin(theta) + cY)
                cv2.line(display, (startX, startY), (cX,cY), self.display_colors[color], 5)
                cv2.putText(display, f"{color} {cv2.contourArea(c)}", (cX - 20, cY - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            return display, cX, cY, theta
    
    def bottle_identification(self, prev_bottle = None, display_only = False):
        """
        use camera to identify bottle location and output a display of where the bottle is
        """
        previous_mask = None
        results = [] #list of identified bottles - (color, centerX, centerY, theta)

        # Wait for a new frame
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        spatial = rs.spatial_filter()
        temporal = rs.temporal_filter()
        hole_filling = rs.hole_filling_filter()

        depth_frame = spatial.process(depth_frame)
        depth_frame = temporal.process(depth_frame)
        depth_frame = hole_filling.process(depth_frame)

        # if not depth_frame or not color_frame:
        #     return

        # Convert frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        left, top, right, bottom = self.belt_space
        depth_image = depth_image[top:bottom, left:right] #crop image
        color_image = color_image[top:bottom, left:right]
        display = color_image.copy()

        if display_only:
            return display

        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        for color in self.colors:
            bounds = self.color_ranges[color]
            mask = cv2.inRange(hsv, bounds[0], bounds[1])
        
            #opening?
            closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations = 1)
            cnts, _ = cv2.findContours(closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if len(cnts) > 0:
                c = max(cnts, key = cv2.contourArea)
                if cv2.contourArea(c) >= 1000: #experiment with minimum area
                    display, X, Y, theta = self.find_centroid(c, display, color)
                    realX, realY = self.pixels_to_coordinates(X, Y)
                    results.append((color, realX, realY, theta, cv2.contourArea(c)))

        if not results:
            # Convert depth to meters

            depth_meters = depth_image * self.depth_scale
            mask_test = ((depth_meters >= 0.8) & (depth_meters < 0.899)).astype(np.uint8)
            mask_test_display = mask_test * 255
            cv2.imshow('Depth > 0.9m Mask', mask_test_display)

            if previous_mask is not None:
                # Keep only pixels that are consistently present
                mask_test_display = cv2.bitwise_and(mask_test_display, previous_mask)

            # Store current mask for next frame
            previous_mask = mask_test_display.copy()

            # results = []
            # Filter out small areas
            contours, _ = cv2.findContours(mask_test_display, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_copy = []
            for c in contours:
                if cv2.contourArea(c) < 3000:
                    cv2.drawContours(mask_test_display, [c], -1, 0, -1)  # fill with black
                else:
                    contours_copy.append(c)

            if len(contours_copy) > 0:
                    c = max(contours_copy, key = cv2.contourArea)
                    
                    # compute the center of the contour
                    display, X, Y, theta = self.find_centroid(c, display, "clear")
                    realX, realY = self.pixels_to_coordinates(X, Y)
                    results.append(("clear", realX, realY, theta, cv2.contourArea(c)))
            

        target = self.best_target(results, prev_bottle)
        if target:
            target = target[:4] 
            return bottle.Bottle(*target), display
        else:
            return None, display
    
    def best_target(self, results, prev_bottle):
        if not results:
            return ()
        elif prev_bottle != None:
            # sort largest to smallest, return largest contour that is within uncertainty distance
            sizes = sorted(results, key=lambda x: x[4])
            un = prev_bottle.get_uncertainty()
            prev_x, prev_y = prev_bottle.get_pos()
            for c in sizes:
                dist = ((c[1]-prev_x)**2 + (c[2]-prev_y)**2)**.5
                if dist < un:
                    return c
            # if no bottles pass, then return largest contour
            return sizes[0]
        else:
            #just return largest contour
            return max(results, key=lambda x: x[4])
        
if __name__=="__main__":
    CV = CVInterface()
    test_bot = None
    #CV.set_camera_resolution(640, 480)
    #CV.set_camera_fps(10)
    while True:
        results, display = CV.bottle_identification()
        if results and test_bot == None:
            test_bot = results
        elif results and test_bot != None:
            results.update(test_bot, 1/15)
            test_bot = results
            print(test_bot)
            if test_bot.get_status() == "ready":
                cv2.imwrite("cv_interface/center.png", display)
                break
        
        # show the image
        cv2.imshow("Output", display)
        #cv2.waitKey(3)

        #press q to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    CV.pipeline.stop()
    cv2.destroyAllWindows()