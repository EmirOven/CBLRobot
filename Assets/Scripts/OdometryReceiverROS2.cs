using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Nav;   // For OdometryMsg

/// <summary>
/// Subscribes to ROS 2 /odom (nav_msgs/Odometry) and updates this GameObject’s Transform
/// so that it matches the TurtleBot3’s pose in real time.
/// 
/// Attach this to the “Robot” GameObject in Unity. Ensure that:
///  1. You have a ROSConnection GameObject in the scene (pointing at your ROS 2 ROSBridge).
///  2. The Robot’s initial Unity position corresponds to the TurtleBot’s origin in ROS.
///  3. The topic name ("/odom") matches what the robot is publishing.
/// </summary>
[RequireComponent(typeof(ROSConnection))]
public class OdometryReceiverROS2 : MonoBehaviour
{
    [Header("ROS 2 Settings")]

    [Tooltip("ROS 2 topic name for odometry. Must match the TurtleBot3's published /odom.")]
    public string odomTopic = "/odom";

    [Tooltip("Quality of Service to use for odometry. SENSOR_DATA is a good best-effort for high-rate streams.")]
    // public QoSSettings qos = QoSSettings.SENSOR_DATA;

    // Internal handle to ROSConnection
    private ROSConnection ros2Connector;

    void Start()
    {
        // 1. Acquire (or create) the ROSConnection singleton
        ros2Connector = ROSConnection.GetOrCreateInstance();
        if (ros2Connector == null)
        {
            Debug.LogError("OdometryReceiverROS2: No ROSConnection found or created. " +
                           "Make sure you have a ROSConnection component in the scene.");
            enabled = false;
            return;
        }

        // 2. Subscribe to /odom with the chosen QoS
        ros2Connector.Subscribe<OdometryMsg>(
            odomTopic,
            OnOdometryReceived,
            // qos
        );
    }

    /// <summary>
    /// Callback invoked whenever an Odometry message arrives.
    /// We extract x, y, and yaw, then immediately set transform.position & rotation.
    /// </summary>
    private void OnOdometryReceived(OdometryMsg msg)
    {
        // 1) Read ROS position (meters)
        double rosX = msg.pose.pose.position.x;   // forward
        double rosY = msg.pose.pose.position.y;   // left
        // double rosZ = msg.pose.pose.position.z; // usually 0 for a ground robot

        // 2) Convert to Unity world: (rosX, rosY) → (unityX, unityZ)
        Vector3 unityPos = new Vector3((float)rosX, 0f, (float)rosY);
        transform.position = unityPos;

        // 3) Read ROS quaternion and extract yaw
        var q = msg.pose.pose.orientation;
        // yaw (radians) = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        float siny_cosp = 2f * (float)(q.w * q.z + q.x * q.y);
        float cosy_cosp = 1f - 2f * (float)(q.y * q.y + q.z * q.z);
        float yawRad = Mathf.Atan2(siny_cosp, cosy_cosp);

        // 4) Convert yaw → degrees and make a Unity Quaternion around Y
        float yawDeg = yawRad * Mathf.Rad2Deg;
        transform.rotation = Quaternion.Euler(0f, yawDeg, 0f);
    }
}
