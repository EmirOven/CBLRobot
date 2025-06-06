using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Sensor;

/// <summary>
/// Subscribes to a ROS 2 LaserScanMsg (topic '/scan' by default),
/// converts each valid range+angle into a 2D point in the robot's local frame,
/// transforms it to Unity-world (via this GameObject's Transform), and
/// calls GridManager.PlaceSphereIfCellEmpty(worldPt).
/// 
/// Attach this script to your Robot GameObject (so that local world uses robot's pose).
/// </summary>
public class LaserScanProcessorROS2 : MonoBehaviour
{
    [Header("ROS 2 Settings")]
    [Tooltip("Topic name for LaserScan (default: \"/scan\").")]
    public string laserScanTopic = "/scan";

    // [Tooltip("Quality of Service profile for LaserScan subscription.")]
    // public QoSSettings qos = QoSSettings.SENSOR_DATA; // best-effort, keep last 5

    [Header("References")]
    [Tooltip("Reference to the GridManager in the scene.")]
    public GridManager gridManager;

    // Internal: handle to the ROSConnection
    private ROSConnection ros2Connector;

    private void Start()
    {
        // 1) Find (or create) the ROSConnection in the scene
        ros2Connector = ROSConnection.GetOrCreateInstance();
        if (ros2Connector == null)
        {
            Debug.LogError("LaserScanProcessorROS2: Cannot find or create ROSConnection. Make sure you have ROS-TCP-Connector installed and a ROSConnection GameObject in the scene.");
            return;
        }

        // 2) Check that GridManager has been assigned
        if (gridManager == null)
        {
            Debug.LogError("LaserScanProcessorROS2: GridManager reference is null. Assign it in the Inspector.");
            return;
        }

        // 3) Subscribe to LaserScan using sensor data QoS
        ros2Connector.Subscribe<LaserScanMsg>(
            laserScanTopic,
            OnLaserScanReceived);
            // qos);
    }

    /// <summary>
    /// Callback invoked on every incoming LaserScanMsg from ROS 2.
    /// We iterate through scan.ranges[], convert each valid ray to (x,z), then
    /// transform that to Unity-world and place a sphere if the grid cell was empty.
    /// </summary>
    /// <param name="scan">The incoming LaserScanMsg (units: meters, angles in radians).</param>
    private void OnLaserScanReceived(LaserScanMsg scan)
    {
        // angle starts at angle_min
        float angle = scan.angle_min;

        for (int i = 0; i < scan.ranges.Length; i++)
        {
            float r = scan.ranges[i];
            // Skip invalid measurements
            if (float.IsInfinity(r) || float.IsNaN(r) || r <= scan.range_min || r >= scan.range_max)
            {
                angle += scan.angle_increment;
                continue;
            }

            // 1) Convert polar (r, angle) local ROS frame (x_ros, z_ros):
            //    x_ros = r * cos(angle)   (forward)
            //    z_ros = r * sin(angle)   (sideways)
            float x_ros = r * Mathf.Cos(angle);
            float z_ros = r * Mathf.Sin(angle);

            // 2) Map ROS frame -> Unity local frame:
            // Assume your ROS2-to-Unity mapping is "ROS X->Unity X" and "ROS Z->Unity Z".
            // If you used a different TF convention, adjust this accordingly.
            Vector3 localPt = new Vector3(x_ros, 0f, z_ros);

            // 3) Transform from Robot-local -> Unity-world:
            //    If you attach this script to your Robot GameObject (whose Transform matches /odom->base_link),
            //    then TransformPoint will place it correctly in world space.
            Vector3 worldPt = transform.TransformPoint(localPt);

            // 4) Let the GridManager decide if this cell is new. If so, it will spawn a sphere.
            gridManager.PlaceSphereIfCellEmpty(worldPt);

            angle += scan.angle_increment;
        }
    }
}
