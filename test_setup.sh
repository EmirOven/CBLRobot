source /opt/ros/foxy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
ros2 launch unity_slam_example unity_slam_example.py
