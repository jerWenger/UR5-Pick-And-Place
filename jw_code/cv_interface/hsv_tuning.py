import cv2
from tkinter import *
import numpy as np
import time

global tk
tk = Tk()
global l_b, u_b
hue_min = Scale(tk, from_ = 0, to = 180, label = 'Hue', orient = HORIZONTAL)
hue_min.pack()
hue_max = Scale(tk, from_ = 0, to = 180, label = 'Hue', orient = HORIZONTAL)
hue_max.pack()
hue_max.set(180)
sat_min = Scale(tk, from_ = 0, to = 255, label = 'Saturation', orient = HORIZONTAL)
sat_min.pack()
sat_max = Scale(tk, from_ = 0, to = 255, label = 'Saturation', orient = HORIZONTAL)
sat_max.pack()
sat_max.set(255)
val_min = Scale(tk, from_ = 0, to = 255, label = 'Value', orient = HORIZONTAL)
val_min.pack()
val_max = Scale(tk, from_ = 0, to = 255, label = 'Value', orient = HORIZONTAL)
val_max.pack()
val_max.set(255)

def main():
    # Open up the webcam
    cap = cv2.VideoCapture(1)
    
    while True:
        tk.update()
        # Read from the webcam 
        ret, frame = cap.read()

        h1 = hue_min.get()
        h2 = hue_max.get()
        s1 = sat_min.get()
        s2 = sat_max.get()
        v1 = val_min.get()
        v2 = val_max.get()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (h1, s1, v1), (h2, s2, v2))
        mask_im = cv2.bitwise_and(frame,frame, mask= mask)
        #print(cap.get(cv2.CAP_PROP_FRAME_WIDTH),cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        #print(mask_im.get(cv2.CAP_PROP_FRAME_WIDTH),mask.get(cv2.CAP_PROP_FRAME_HEIGHT))
        image = cv2.hconcat([frame, mask_im])
       
        cv2.imshow("HSV mask", image)
        cv2.waitKey(3)
        time.sleep(0.02)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print(f"hue ({h1}, {h2}), saturation ({s1}, {s2}), value ({v1}, {v2})")
            break

if __name__=='__main__':
    main()

