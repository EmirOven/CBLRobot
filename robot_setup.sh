source /opt/ros/foxy/setup.bash
source install/setup.bash

$robot_password=turtlebot
$robot_ip=192.168.1.34 # I forgot

sshpass -p $robot_password ssh ubuntu@$robot_ip << EOF
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
exit
EOF

# I hope this will work
ros2 launch unity_slam_example unity_slam_example.py