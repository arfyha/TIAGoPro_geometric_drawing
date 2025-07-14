# Geometric Drawing with the TIAGo Pro

This repository contains the necessary files to make the TIAGo Pro detect the whiteboard, drive towards it and draw a geometric figure onto the board.

## Setup
To execute the drawing process, this repository has to be cloned inside of the docker container provided by PAL robotics. After that please go inside the cloned folder and build it:

```bash
colcon build
```
The next step is to source the environment:
```bash
source /opt/ros/humble/setup.bash
source /opt/pal/alum/setup.bash
source install/setup.bash
```
## Simulation
1. Run the following:
```bash
ros2 launch draw world.launch.py
```
This starts Gazebo and RViz at the same time, while also loading the whiteboard model into Gazebo and tilting the head down.

2. Start the point cloud perception pipeline:
```bash
ros2 launch point_cloud_perception perception.launch.py
```
3. Start the frame broadcasting of the geometric properties:
```bash
ros2 run draw fixed_frame_tf2_broadcaster
```
4. In RViz add the PointCloud2 and the TF Display.
5. Select the desired Topic in the PointCloud2 Display (for the end result of the perception pipeline select "whiteboard_cloud").
6. In the Frames section of the TF Display select the desired tf frame (the geometric function frames have "function" as their prefix, the whiteboard frames have "whiteboard" as their prefix).
7. Start the drawing process:
```bash
ros2 run draw draw_function_whiteboard
```

To change the geometric figure, the line 149 in "fixed_frame_tf2_broadcaster.cpp" can be changed to one of the following:
1. std::vector<double> point = getRectangleCoordinates().at(i);
2. ... calculatePointsFunction(i);
3. ... calculateCirclePoints(i);

Steps 3. and 7. can also be combined by running:
```bash
ros2 launch draw draw.launch.py
``` 
