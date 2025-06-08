using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;

public class MoveBaseGoalPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/move_base_simple/goal";

    public Transform mapFrame;
    public Transform goalFrame;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topicName);
    }

    public void PublishNavigationGoal()
    {
        if (mapFrame == null || goalFrame == null)
        {
            Debug.LogWarning("Map and goal frames not set");
            return;
        }

        PoseStampedMsg goalMsg = new PoseStampedMsg();

        goalMsg.header.frame_id = mapFrame.name;
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
