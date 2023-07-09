
# Pluto Camera Python Wrapper

This is a special python wrapper for pluto drone equipped with the Lewei camera.

Go ahead and pull up the terminal and type:
```bash
git clone https://github.com/MalayPhadke/Pluto_Camera_Python_Wrapper.git
```

Please note, the scripts in these repo will only work on a **Linux OS**.




## Usage

The __test_cam.py__ has the class with which we need to create drone camera Object. Run the __example.py__ script and make sure you get the drone stream. The script just creates an instance of the pluto_cam class and starts the camera stream.

In the script, my_pluto_cam is the object created. Assuming that the name of the object created is my_pluto_cam, following are the other function that can be called with this object.

```python
To start the camera stream: my_pluto_cam.start_cam()

To stop the camera stream: my_pluto_cam.stop_cam()

To take a picture: my_pluto_cam.take_pic()

To start recording: my_pluto_cam.start_recording()
```

In the script, my_pluto is the object created of class pluto which will be used to control the drone. Assuming that the name of the object created is my_pluto, follow the lines below for using the wrapper.

```python
To arm : my_pluto.arm()

To disarm : my_pluto.disarm()

To Takeoff : my_pluto.take_off()

To land : my_pluto.land()

To move the drone forward() : my_pluto.forward()

To move the drone backward() : my_pluto.backward()

To move the drone left : my_pluto.left()

To move the drone right : my_pluto.right()

To rotate the drone right : my_pluto.right_yaw()

To rotate the drone left : my_pluto.left_yaw()
```


To read more about drone control using keyboard and joysticks look into
[Pluto Python Wrapper](https://github.com/MalayPhadke/Pluto_Python_Wrapper)

After running the keyboard control script you can click a picture by pressing **'p'** or start recording by pressing **'v'**. The recording will end when you end the drone stream.
## Installation

Go ahead and pull the the required modules by opening up your terminal and type:
```bash
 pip install -r requirements.txt
 ```
## QR Code scanner

Run the __qrcode_scanner.py__ to scan QR codes with your drone!

This script will create a bounding rectangle around the detected QR code and display its data on the top. The data will also be saved in the __qr_code_data.txt__ file. 

You can run this file by connecting your drone to your laptop and at the same time control the drone with your phone.
## Gesture Detection based Drone Control

Run the __gesture_detection.py__ to control your drone with hand gestures!

When the cam module detects a palm, pluto is armed and it takes off. 
It continues to hover in air until a fist is detected. Both the palm and fist detected are saved as jpg images.

Make sure you run this script in a safe environment. The detections is done using **Haar Cascade** algorithm. The xml files for detection were downloaded from [HandGestureDetection](https://github.com/Sandeep-Sthapit/HandGestureDetection)
## Face Detection

Run the __face_detection.py__ for detecting face through the cam module!

This scripts just detects face and creates a bounding rectangle around it with the **Haar Cascade** algorithm (not to be confused with face recognition).

