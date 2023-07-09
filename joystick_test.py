import time
from test_cam import *
import joystick_control

joy = joystick_control.XboxController()

def mapping(x, inMin, inMax, outMin, outMax): 
    x = (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin
    if (x < outMin):
      return int(outMin)
    elif (x > outMax):
      return int(outMax)
    else:
      return int(x)

my_pluto_cam = pluto_cam()
my_pluto_cam.start_cam()

captureTime = 0
count = 0
while my_pluto_cam.cam_running:

    [x, y, a, b, A, B, X, Y, rb, lb] = joy.read()

    my_pluto.rcThrottle = mapping(y, -1, 1, 1000, 2000)    
    my_pluto.rcYaw = mapping(x, -1, 1, 1000, 2000)
    my_pluto.rcPitch = mapping(b, -1, 1, 1000, 2000)
    my_pluto.rcRoll = mapping(a, -1, 1, 1000, 2000)
    print(my_pluto.rcRoll)
    if A: 
        my_pluto.arm()
        # time.sleep(0.5)
        # print("arming", A)
    elif B:
        my_pluto.disarm() 
        # print("disarming", B)
    elif Y:
        my_pluto.take_off()
        # time.sleep(0.5)
        # print("taken off", X)
    elif X:
        my_pluto.land()
        my_pluto.disarm()
        # print("landing", Y)
    elif rb and time.time() - captureTime > 5:
       captureTime = time.time()
       my_pluto_cam.take_pic()