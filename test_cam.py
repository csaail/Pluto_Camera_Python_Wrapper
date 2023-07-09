#!/usr/bin/env python3

from Pluto import pluto
from ctypes import *
from ctypes import POINTER, Structure
from ctypes import c_void_p, c_int, c_char

import threading  
import h264decoder  
import numpy as np  
import cv2    
import time
 

from pyzbar.pyzbar import decode

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

take_pic = False
global frame_data, recording, scan
frame_data = None
recording = False
scan = False
global count 
count = 0
global writer

def qr_decoder(image):
    gray_img = cv2.cvtColor(image, 0)
    barcode = decode(gray_img)
    for obj in barcode:
        points = obj.polygon
        (x, y, w, h) = obj.rect
        pts = np.array(points, np.int32)
        pts = pts.reshape((-1, 1, 2))
        cv2.polylines(image, [pts], True, (0, 255, 0), 3)

        barcodeData = obj.data.decode('utf-8')
        barcodeType = obj.type
        string = "Data: " + str(barcodeData) + " | Type: " + str(barcodeType)
        cv2.putText(frame_data, string, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        with open('qr_code_data.txt', 'r') as f:
            data = f.read()
        if string not in data:
            if data == "":
                data = string
            else:
                data += "\n" + string
            with open('qr_code_data.txt', 'w') as f:
                f.write(data)

        print(string)

        

def read_buffer(lpParam, pFrame):
        frame_event  = threading.Event()
        global frame_data, recording, scan
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
                    frame = frame[:,:w*2,:]
                    frame = cv2.resize(frame, (w*2, h*2))
                    
                    #change color format from bgr to rgb
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    # frame = cv2.resize(frame, (640, 480))  # Resize frame to match video resolution
                    
                    frame_data = frame
                    if recording and frame_data is not None:
                        # print("Capturing video")
                        writer.write(frame_data)

                    if scan:
                        # print("developer mode")
                        qr_decoder(frame_data)

                    cv2.imshow('frame', frame_data)
                    cv2.waitKey(1)

        return

import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if (key == '\x1b'):
            key = sys.stdin.read(2)
        sys.stdin.flush()
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



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
        # writer.release()

        # self.my_pluto.disarm()
    def take_pic(self):
        if frame_data is not None:
            cv2.imwrite("CapturedImage " + str(time.time()) + " .jpg", frame_data)
            print("Image Captured and saved as CapturedImage " + str(time.time()) + " .jpg")
        else:
            time.sleep(0.1)

    def start_recording(self):
        global recording, writer
        recording = True
        writer = cv2.VideoWriter(f'cam_stream{str(time.strftime("%M-%S"))}.avi', cv2.VideoWriter_fourcc(*'XVID'), 20, (640, 480))
        print("Recording started")

    def scan_qrcode(self):
        global scan
        scan = True

# my_pluto.arm()
# time.sleep(5)
# my_pluto.disarm()
# print(frame_data)
# my_pluto_cam = pluto_cam()
# my_pluto_cam.start_cam()
# time.sleep(1)
# # my_pluto_cam.stop_cam()
# # my_pluto_cam.take_pic()

# while my_pluto_cam.cam_running:
#     key = getKey()
#     if key == 'q':
#         my_pluto_cam.stop_cam()
#         # video_thread.join()
#         break
#     if key == 'v':
#             my_pluto_cam.start_recording() if not recording else my_pluto_cam.stop_recording()

    
     
