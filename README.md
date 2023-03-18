Go1-Gesture-Command

This repository contains two packages for using hand gestures to send motion commands to the Unitree Go1:
- ros2_hgr
    * A ROS2 Python package with code forked from Kinivi's repository linked here (insert link here) that uses computer vision, Mediapipe, and a machine learning model to detect 8 different hand gestures.
- go1_cmd
    * A ROS2 C++ package that utilizes 

## Dependencies ##

-    [OpenCV-Python for Ubuntu](https://docs.opencv.org/4.5.4/d2/de6/tutorial_py_setup_in_ubuntu.html)
-    [NumPy](https://numpy.org/install/)
-    [MediaPipe](https://google.github.io/mediapipe/getting_started/python.html)
-    [TensorFlow](https://www.tensorflow.org/install)

## Launch ##

`ros2 launch ros2_hgr hgr.launch.xml`

- The ros2_hgr launch file has defaults set to run the hgr_node with your computer's built-in webcam.

* Author of [Fork](https://github.com/avazahedi/go1-gesture-command): Ava Zahedi

## Grasp Gestures ##

0. Open Palms
1. Close fist
2. One pointer
3. Pinch Index Finger
4. Pinch Middle Finger
5. 3 Finger Grasp
6. 4 Finger Grasp

[gesture](https://user-images.githubusercontent.com/60728026/226082118-d9508ec7-0bd2-4ee7-9335-b19a69a19a05.mp4)