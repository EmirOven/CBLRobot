using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using UnityEngine;

public class LiDARToWalls : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/scan";
    public GameObject pointPrefab; // Prefab to represent each LIDAR point
    public Transform robotOrigin;  // Reference point for the robot

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<LaserScanMsg>(topicName, VisualizeLidar);
    }

    void VisualizeLidar(LaserScanMsg msg)
    {
        float angle = msg.angle_min;
        float angleIncrement = msg.angle_increment;

        // Optional: Clear old points
        foreach (Transform child in transform)
            Destroy(child.gameObject);

        for (int i = 0; i < msg.ranges.Length; i++)
        {
            float distance = msg.ranges[i];

            // Skip invalid range
            if (distance < msg.range_min || distance > msg.range_max)
            {
                angle += angleIncrement;
                continue;
            }

            // Convert polar to Cartesian
            float x = distance * Mathf.Cos(angle);
            float y = distance * Mathf.Sin(angle);
            Vector3 localPosition = new Vector3(x, 0, y); // Assuming 2D scan (XZ plane in Unity)

            // Instantiate point relative to robot
            Vector3 worldPosition = robotOrigin.TransformPoint(localPosition);
            Instantiate(pointPrefab, worldPosition, Quaternion.identity, transform);

            angle += angleIncrement;
        }
    }
}
