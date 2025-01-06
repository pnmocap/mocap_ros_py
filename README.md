## Quick start guide

### Axis Studio configuration

1. Launch *Axis Studio*, Open a sample motion file

   [![img](https://github.com/pnmocap/neuron_mocap_live-c4d/raw/main/resource/launch_axis_studio.gif)](https://github.com/pnmocap/neuron_mocap_live-c4d/blob/main/resource/launch_axis_studio.gif)

2. Enable *BVH Broadcasting - Edit*, configure the broadcast settings as follows:

   - Skeleton: **Axis Studio**
   - BVH Format - Rotation: **XYZ**
   - BVH Format - Displacement: **Checked**
   - Frame Format - Type: **Binary**
   - Frame Format - Use old header format: **Unchecked**
   - Protocol: **UDP**
   - Local Address: **192.168.2.40:7001**
   - Destination Address: **192.168.2.150:7012**

   ![as_setting](file:////192.168.2.129/rd_bj/Projects/RDL_Projects/RDL_URDF/release/doc/img/as_setting.png?lastModify=1736145785)



### URDF configuration

1. ROS1

~~~
cd ~/catkin/
catkin_make
source  devel/setup.bash
roslaunch  galbot_one_charlie_description view_robot.launch
~~~





2. ROS2

~~~
colcon build
source  install/setup.bash
ros2  bytedance_description view_urdf.launch.py
~~~





### Mocap Api  configuration

This directory contains motion driver programs for humanoid robot manufacturers. The program uses mocapapi to interface with Axis Studio to obtain BVH motion data and convert it into URDF format for driving the movements of humanoid robots. It can achieve real-time motion driving.

1. Content List

- lib/: This folder includes dynamic libraries of robotapi for various CPU architectures. The library is responsible for interfacing with motion capture devices to obtain motion data and converting it into URDF format data.
- retarget.json: A JSON file used for conversion specific to each robot manufacturer and model.
- robot_api_ros\<x\>.py: A Python script that starts up, calls the robotapi library, reads the retarget.json file, and uses ROS interfaces to drive the robot.

>Note that the lib folder, Python script, and retarget.json must be in the same directory. When the Python script is launched, it will load the JSON file and the .so files in the lib directory from the current directory.

2. Run Python script

~~~
python3 robot_api_ros<x>.py
~~~

> At this point, you will see that the 3D model in the Axis Studio software and the model in the robot window are moving in sync.

