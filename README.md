# ingenia_ws

Primera prueba de workspace para el repositorio de ingenia 2023/24

# Lista de instrucciones para el funcionamiento del paquete de control del robot a través de RViz
# by Adela

cd workspace/ros_ur_driver/
# si pongo tab se autocompleta
ctrl +R : source install/setup.bash

# puedo ejecutar cualquier launch del driver.

# visualizar moveit
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true

# para mover el robot
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=192.168.0.9 use_fake_hardware:=false launch_rviz:=false initial_joint_controller:=joint_trajectory_controller

# and in another shell
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10 launch_rviz:=true use_fake_hardware:=false robot_ip=192.168.0.9
