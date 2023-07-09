#!/usr/bin/env python3

from Pluto import pluto
from ctypes import *
from ctypes import POINTER, Structure
from ctypes import c_void_p, c_int, c_char

import threading  
import h264decoder   
import numpy as np   
import cv2 
 


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

global frame_data
frame_data = None

haar_file = 'haarcascade_frontalface_default.xml' 

# defining the size of images
(width, height) = (330, 300)	

face_cascade = cv2.CascadeClassifier(haar_file)

def face_detection(im): 
	gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
	faces = face_cascade.detectMultiScale(gray, 1.3, 4) # Detect faces
	for (x, y, w, h) in faces:
		cv2.rectangle(im, (x, y), (x + w, y + h), (255, 0, 0), 2)
		face = gray[y:y + h, x:x + w]
		face_resize = cv2.resize(face, (width, height))

def read_buffer(lpParam, pFrame):
        frame_event  = threading.Event()
        global count
        global frame_data, recording
        ret = 0
        got_picture = 0
        
        pFrame_size = pFrame[0].size

        if (pFrame_size <= 0):
            libc.video_free_frame_ram(pFrame)
            return

        else:
            print("pframe_size: ", pFrame[0].size) 
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
                    frame_data = frame
                    face_detection(frame_data)
                    cv2.imshow('frame', frame_data)
                    cv2.waitKey(1)


        return


class pluto_cam():

    def __init__(self):
        self.cam_running = False
        self.taking_pic = False

    def show_cam(self):
        cmp_func = CMPFUNC(read_buffer)

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


     