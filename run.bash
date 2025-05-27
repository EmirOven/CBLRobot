#! /bin/bash

echo "Please input the IP for the robot:"
read robot_ip
echo "Please input the password for the robot:"
read robot_password


if dpkg -s sshpass &> /dev/null; then
    echo "sshpass is already installed."
else
    echo "sshpass is not installed. Installing..."
    sudo apt-get update
    sudo apt-get install -y sshpass
fi


sshpass -p $robot_password ssh ubuntu@$robot_ip << EOF
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
exit
EOF

echo "Please input the IP for the PC:"
read pc_ip

ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=$pc_ip

echo "Done!"