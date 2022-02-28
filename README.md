# Program a Robot in less than 150 Lines of Code

![150_line_arena](https://raw.githubusercontent.com/behavior-circuits/website/master/images/150_lines_arena.png)

The following example is meant to illustrate how simple it is to develop mobile robot AI using behavior circuits.

In this example scenario, two robots called mouse and cat respectively compete in an arena. The aim of the mouse is to reach a cheese placed in the arena before it is cought by the cat. Conversely the cat tries to first catch the mouse. The mouse can try to avoid the cat by hiding behind one of two colored curtains.

Both robots are equipeed with cameras and ultrasonic sensors and feature a ROS interface. Using behavior circuits it is possible to create a intelligent of the mouse in less than 150 lines of code. These include all necessairy code to detect the cat, cheese and hiding places using the camera and read out the ultrasonic sensors.


# Quickstart Guide
To run the code first install the requirements.txt
The Code is designed to run on a [Pioneer P3dx](https://www.generationrobots.com/media/Pioneer3DX-P3DX-RevA.pdf) but in principle can be applied to any robot woth sonar sensors and a camera.
See the `vision.py` and `sonar.py` scripts for reference.

To run the code [ROS](https://www.ros.org/) needs to be installed with version Melodic or above.
If this repository is placed and build in a catkin workspace the code can be launched with
```
 roslaunch maus maus.launch
```
