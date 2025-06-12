using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Tf2; // TFMessageMsg

/// <summary>
/// Subscribes to a TFMessage on /tf, takes the first TransformStamped, 
/// and applies its translation & rotation to a Unity GameObject.
/// </summary>
public class RobotPositionSync : MonoBehaviour
{
    [Tooltip("Drag your virtual robot (or its parent) here")]
    public Transform virtualRobot;

    [Tooltip("ROS topic where tf frames are broadcast (usually /tf)")]
    public string tfTopic = "/tf";

    ROSConnection ros;

    void Start()
    {
        // 1) Get or create the singleton ROSConnection
        ros = ROSConnection.GetOrCreateInstance();

        // 2) Subscribe to TFMessageMsg → callback UpdateRobotPosition
        //    API ref: https://github.com/Unity-Technologies/Unity-Robotics-ROSTCP-Connector#subscribe
        ros.Subscribe<TFMessageMsg>(tfTopic, UpdateRobotPosition);
    }

    void UpdateRobotPosition(TFMessageMsg msg)
    {
        // TFMessageMsg.transforms is an array of TransformStampedMsg:
        // https://github.com/Unity-Technologies/Unity-Robotics-ROSTCP-Connector/blob/main/com.unity.robotics/Runtime/MessageDefinitions/Tf2/TFMessageMsg.cs
        if (msg.transforms == null || msg.transforms.Length == 0)
        {
            // Nothing to do if no transforms in this message
            return;
        }

        // Take the first stamped transform
        var stamped = msg.transforms[0];
        var t = stamped.transform;

        // --- Position ---
        // ROS uses X-forward, Y-left, Z-up.
        // Unity uses X-right, Y-up, Z-forward.
        // If your robot moves on a flat floor, you may ignore the ROS Z.
        Vector3 rosPos = new Vector3(
            (float)t.translation.x,
            (float)t.translation.y,
            (float)t.translation.z
        );

        // Map ROS→Unity:
        Vector3 unityPos = new Vector3(
            rosPos.x,     // ROS X → Unity X
            rosPos.z,     // ROS Z → Unity Y (up)
            rosPos.y      // ROS Y → Unity Z (forward)
        );
        virtualRobot.localPosition = unityPos;

        // --- Rotation ---
        // ROS quaternion: (x, y, z, w)
        // we need to remap axes similarly.
        Quaternion rosQuat = new Quaternion(
            (float)t.rotation.x,
            (float)t.rotation.y,
            (float)t.rotation.z,
            (float)t.rotation.w
        );

        // Convert ROS→Unity quaternion by swapping/negating axes:
        // (see TF orientation conventions + Unity left-handed system)
        Quaternion unityQuat = new Quaternion(
            rosQuat.x,
            rosQuat.z,
            -rosQuat.y,
            rosQuat.w
        );

        // Optional correction if your robot's base frame is rotated
        // relative to Unity’s forward:
        Quaternion correction = Quaternion.Euler(0f, 90f, 0f);

        virtualRobot.localRotation = correction * unityQuat;
    }
}
