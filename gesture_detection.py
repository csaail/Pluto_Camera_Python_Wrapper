#!/usr/bin/env python3

"""
This code is based on gesture detection done in ROS. When the cam module detects a palm, pluto is armed and it takes off. 
It continues to hover in air until a fist is detected. Both the palm and fist detected are saved as jpg images.
"""

from Pluto import pluto
from ctypes import *

from ctypes import POINTER, Structure
from ctypes import c_void_p, c_int, c_char

# import base64
import threading  
import h264decoder  
import numpy as np  
import cv2
import time



global lewei_video_framepyth
frame = b''

my_pluto = pluto()# This commands creates an object of pluto() where you can connect your hardware to. my_pluto is literally your plutodrone



libc = CDLL("libLeweiLib.so")
decoder = h264decoder.H264Decoder()

class lewei_video_frame(Structure):
        _fields_ = [('timestamp',c_int64),
                ('iFrame',c_int32),
                ('size',c_int32),
                ('buf', POINTER(c_char))]

        def __repr__(self):
            return f'Lewei Video Frame Buf: {self.buf}'

CMPFUNC = CFUNCTYPE(None,POINTER(c_void_p),POINTER(lewei_video_frame))

writer= cv2.VideoWriter('gesture_detection.avi', cv2.VideoWriter_fourcc(*'XVID'), 20, (640, 480))

#Loading the xml files
fist_cascade = cv2.CascadeClassifier('fist.xml')
lpalm_cascade = cv2.CascadeClassifier('lpalm.xml')
rpalm_cascade = cv2.CascadeClassifier('rpalm.xml')


class pluto_cam():

    def __init__(self):
        self.cam_running = False
        self.palm_label=""
        self.objDetected = ""
        self.inAir = False
        self.stop = False

    def read_buffer(self, lpParam, pFrame):
   
        frame_event  = threading.Event()
        global count
        ret = 0
        got_picture = 0
        
        pFrame_size = pFrame[0].size

        if (pFrame_size <= 0):
            libc.video_free_frame_ram(pFrame)
            return

        else: 
            data_in = pFrame[0].buf[:pFrame[0].size]

            framedatas = decoder.decode(data_in)
            for framedata in framedatas:
                (frame, w, h, ls) = framedata
                if frame is not None:
                    frame = np.frombuffer(frame, dtype=np.ubyte, count=len(frame))
                    frame = frame.reshape((h, ls//3, 3))
                    frame = frame[:,:w,:]
                    
                    #change color format from bgr to rgb
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    writer.write(frame)

                    # Convert the cv_image to grayscale
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                    # Detect fists in the grayscale frame
                    fists = fist_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(100, 100))

                    # Iterate over the detected fist regions
                    for (x, y, w, h) in fists:
                        # Draw rectangles around the fist regions
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                        self.objDetected = "Fist"
                        # Display the fist label
                        cv2.imwrite(f"fist_detected{time.strftime('%H-%M-%S')}.jpg", frame)  #Save the detected fist frame
                        cv2.putText(frame, 'Fist', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                       

                    # Detect palms in the grayscale frame
                    rpalms = rpalm_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(100, 100))


                    for (x, y, w, h) in rpalms:
                        # Draw rectangles around the palm regions
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        self.objDetected = "Palm"
                        #Arma and Take off the drone
                        my_pluto.arm()
                        my_pluto.take_off()
                        
                        self.stop = False
                        self.inAir = True

                        # Display the palm label
                        cv2.putText(frame, 'Palm', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        cv2.imwrite(f"palm_detected{time.strftime('%H-%M-%S')}.jpg", frame) #Save the detected palm frame

                    if self.objDetected == "Palm":
                        print("Taking off")
                    elif  self.objDetected == "Fist" and self.inAir:
                        self.inAir = False
                        self.stop = True
                        my_pluto.disarm()
                        my_pluto.land()
                    else:
                        print("Nothing detected yet")

                    if self.stop: 
                        cv2.putText(frame, "Landed ", (40, 70), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                    elif self.inAir:
                        cv2.putText(frame, "In Air", (40, 70), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)
                    else:
                        cv2.putText(frame, "Drone Stream", (40, 70), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)

                        
                    cv2.imshow("Palm Detection", frame)
                    cv2.waitKey(1)
            
        return


    def show_cam(self):
        cmp_func = CMPFUNC(self.read_buffer)

        libc.lewei_initialize_stream()
        libc.lewei_set_HDflag(False)

        # while True:
        ret = libc.lewei_start_stream(None,cmp_func)
        # exit()
         
    def start_cam(self):
        self.cam_running = True
        threading.Thread(target=self.show_cam).start()
    
    def stop_cam(self):
        cv2.destroyAllWindows()
        libc.lewei_stop_stream()


my_pluto_cam = pluto_cam()
my_pluto_cam.start_cam()


