# cv_interface.py
import cv2
import math
import numpy as np

class CVInterface:
    def __init__(self):
        """
        Initialize computer vision interface.
        """
        self.cap = cv2.VideoCapture(1) # 0 for webcam, probably 1 for external camera
        
        blue_lower_bound = (85,50,50)
        blue_upper_bound = (140,255,255)

        green_lower_bound = (40,50,50)
        green_upper_bound = (85,255,255)

        self.colors = ["green", "blue"]
        self.display_colors = {"blue": (255, 0, 0), "green": (0, 255, 0)}
        self.color_ranges = {"green":(green_lower_bound, green_upper_bound), "blue":(blue_lower_bound, blue_upper_bound)}
        self.kernel = np.ones((5,5),np.uint8)
        self.pixels_per_inch = 17
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
        
    def aux_function(self, *args, **kwargs):
        """
        Auxiliary function placeholder.
        """
        print("CVInterface auxiliary function called.")

    def bottle_identification(self):
        """
        use camera to identify bottle location and output a display of where the bottle is
        """
        ret, image = self.cap.read()
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
                
                # compute the center of the contour
                M = cv2.moments(c)
                if M["m10"] == 0 or M["m00"] == 0 or M["m01"] == 0 or M["m00"] == 0:
                    break
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
                cv2.line(image, (startX, startY), (endX,endY), (0,0,255), 6)

                results.append((color, cX, cY, theta))

        return results, display
    
    def first_bottle(self, results):
        return min(results, key=lambda x: x[2])
    
    def calculate_velocity(self, center1, center2):
        """
        calculate approximate velocity in m/s of a bottle given (X1, Y1) and (X2, Y2)
        """
        fps = self.get_camera_fps
        dx = center2[0] - center1[0]
        dy = center2[1] - center1[1]
        # add a limit on max velocity to ignore impossibly large jumps
        pixels_per_meter = self.pixels_per_inch*.0254
        v_abs = math.sqrt(dx**2 + dy**2)*fps/pixels_per_meter 
        vx = dx*fps/pixels_per_meter
        vy = dy*fps/pixels_per_meter
        return v_abs, vx, vy
        
if __name__=="__main__":
    CV = CVInterface()
    while True:
        #ret, image = CV.cap.read()

        # visualize it in a cv window
        #cv2.imshow("Original_Image", image)
        cv2.waitKey(3)

        #image = cv2.imread("crunched.jpg")
        #image = identify_clear(image)
        results, display = CV.bottle_identification()
        

        # show the image
        cv2.imshow("Output", display)
        cv2.waitKey(3)

        #press q to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    CV.cap.release()
    cv2.destroyAllWindows()