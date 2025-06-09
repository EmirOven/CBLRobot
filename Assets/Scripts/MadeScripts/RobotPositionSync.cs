using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav; // For OdometryMsg

public class RobotPositionSync : MonoBehaviour
{
    public Transform virtualRobot; // Reference to the virtual robot's transform
    public string odomTopic = "/odom"; // ROS topic for odometry data

    private ROSConnection ros;

    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<OdometryMsg>(odomTopic, UpdateRobotPosition);
    }

    void UpdateRobotPosition(OdometryMsg odomMsg)
    {
        // Update the virtual robot's position
        Vector3 newPosition = new Vector3(
            (float)odomMsg.pose.pose.position.x,
            0, // Assuming the robot moves on a flat plane
            (float)odomMsg.pose.pose.position.y
        );
        virtualRobot.position = newPosition;

        // Update the virtual robot's rotation
        Quaternion newRotation = new Quaternion(
            (float)odomMsg.pose.pose.orientation.x,
            -(float)odomMsg.pose.pose.orientation.z, // Swap Y and Z for Unity's coordinate system
            -(float)odomMsg.pose.pose.orientation.y,
            (float)odomMsg.pose.pose.orientation.w
        );

        Quaternion correction = Quaternion.Euler(0, 90, 0);
        virtualRobot.rotation = correction * newRotation;
    }
}