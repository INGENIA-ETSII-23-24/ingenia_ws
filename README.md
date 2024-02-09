## Install
Create workspace and clone this repo:

```
source /opt/ros/foxy/setup.bash
git clone git@github.com:INGENIA-ETSII-23-24/ingenia_ws.git -b foxy workspace
cd workspace/ros_ur_driver
rosdep install --ignore-src --from-paths src -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Launch the simulation
```
ros2 launch ur_bringup simulacion_robot.launch.py 
```


## Launch the real robot
```
ros2 launch ur_bringup robot_real.launch.py 
```
## Launch the point reader
```
ros2 run py_srvcli prueba 
```
