using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Tf2; // For OdometryMsg

public class RobotPositionSync : MonoBehaviour
{
    public Transform virtualRobot; // Reference to the virtual robot's transform
    public string odomTopic = "/tf"; // ROS topic for odometry data

    private ROSConnection ros;

    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TFMessageMsg>(odomTopic, UpdateRobotPosition);
    }

    void UpdateRobotPosition(TFMessageMsg odomMsg)
    {
        // Update the virtual robot's position
        Vector3 newPosition = new Vector3(
            (float)odomMsg.transforms.transform.translation.x,
            0, // Assuming the robot moves on a flat plane
            (float)odomMsg.transforms.transform.translation.y
        );
        virtualRobot.position = newPosition;

        // Update the virtual robot's rotation
        Quaternion newRotation = new Quaternion(
            (float)odomMsg.transforms.transform.rotation.x,
            -(float)odomMsg.transforms.transform.rotation.z, // Swap Y and Z for Unity's coordinate system
            -(float)odomMsg.transforms.transform.rotation.y,
            (float)odomMsg.transforms.transform.rotation.w
        );

        Quaternion correction = Quaternion.Euler(0, 90, 0);
        virtualRobot.rotation = correction * newRotation;
    }
}
