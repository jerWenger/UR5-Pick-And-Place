# cv_interface.py
import cv2
import math
import pyrealsense2 as rs
import numpy as np
import bottle

class CVInterface:
    def __init__(self):
        """
        Initialize computer vision interface.
        """
        self.cap = cv2.VideoCapture(0) # 0 for webcam, 1 for external camera
        
        # blue_lower_bound = (85,50,50)
        # blue_upper_bound = (140,255,255)
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
        self.belt_space = (140, 0, 530, 430) #X1, Y1, X2, Y2
        # belt area = (140, 0) (530, 430)
        # width = 23", height = 25"
        
    def get(self, key):
        """
        Generic getter for internal data.
        """
        return self._data.get(key, None)
    
    def get_camera_fps(self):
        return self.cap.get(cv2.CAP_PROP_FPS)
    
    def get_camera_resolution(self):
        """
        Get width and height of camera stream
        """
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

        return width, height
        
    def set(self, key, value):
        """
        Generic setter for internal data.
        """
        self._data[key] = value

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
        roboty = (pixelx+self.belt_space[0]-335)/self.ppm
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
            cv2.putText(display, color, (cX - 20, cY - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            theta = 0.5*np.arctan2(2*M["mu11"],M["mu20"]-M["mu02"])
            startX = int(cX - 200 * np.cos(theta)) 
            startY = int(cY - 200 * np.sin(theta))
            endX = int(200 * np.cos(theta) + cX) 
            endY = int(200 * np.sin(theta) + cY)
            cv2.line(display, (startX, startY), (endX,endY), self.display_colors[color], 5)

        return display, cX, cY, theta

    def bottle_identification(self):
        """
        use camera to identify bottle location and output a display of where the bottle is
        """
        ret, image = self.cap.read()
        left, top, right, bottom = self.belt_space
        image = image[top:bottom, left:right] #crop image
        display = image.copy()
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        results = [] #list of identified bottles - (color, centerX, centerY, theta)

        for color in self.colors:
            bounds = self.color_ranges[color]
            mask = cv2.inRange(hsv, bounds[0], bounds[1])
        
            closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations = 1)
            cnts, _ = cv2.findContours(closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if len(cnts) > 0:
                c = max(cnts, key = cv2.contourArea)
                if cv2.contourArea(c) >= 225: #experiment with minimum area
                    display, X, Y, theta = self.find_centroid(c, display, color)
                    realX, realY = self.pixels_to_coordinates(X, Y)
                    results.append((color, realX, realY, theta))

        if not results:
            pass #run depth sensing
        first = self.first_bottle(results)
        if first:
            return bottle.Bottle(*first), display
        else:
            return None, display
    
    def first_bottle(self, results):
        if not results:
            return ()
        else:
            return min(results, key=lambda x: x[2])
        
        
if __name__=="__main__":
    CV = CVInterface()
    CV.set_camera_resolution(640, 480)
    CV.set_camera_fps(10)
    while True:
        results, display = CV.bottle_identification()
        #print(results)
        
        # show the image
        cv2.imshow("Output", display)
        #cv2.waitKey(3)

        #press q to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    CV.cap.release()
    cv2.destroyAllWindows()