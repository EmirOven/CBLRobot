using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;
using System.Collections;
using RosMessageTypes.Std;

public class MoveBaseGoalPublisherE : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/goal_pose";

    public Transform mapFrame;
    public Transform goalFrame;

    void Start()
    {	
    	Debug.Log("Starting ROS publisher setup...");
    	ros = ROSConnection.GetOrCreateInstance();
    	ros.RegisterPublisher<PoseStampedMsg>(topicName);

    	Invoke(nameof(PublishNavigationGoal), 3.0f); // Delay 3s to wait for ROS
    }

    public void PublishNavigationGoal()
    {
        if (mapFrame == null || goalFrame == null)
        {
            Debug.LogWarning("Map and goal frames not set");
            return;
        }
        Debug.Log("Publishing goal to ROS...");
        Debug.Log($"Publishing goal to {topicName} at: {goalFrame.position}");

        PoseStampedMsg goalMsg = new PoseStampedMsg();

        goalMsg.header.frame_id = "map";
        goalMsg.header.stamp = new TimeMsg
        {
            sec = (int)System.DateTimeOffset.UtcNow.ToUnixTimeSeconds(),
            nanosec = (uint)((System.DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() % 1000) * 1000000)
        };

        goalMsg.pose = new PoseMsg
        {
            position = new PointMsg(goalFrame.position.x, goalFrame.position.y, goalFrame.position.z),
            orientation = new QuaternionMsg(goalFrame.rotation.x, goalFrame.rotation.y, goalFrame.rotation.z, goalFrame.rotation.w)
        };

        ros.Publish(topicName, goalMsg);
    }
}
