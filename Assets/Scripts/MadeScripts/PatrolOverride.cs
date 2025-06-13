using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class PatrolOverride : MonoBehaviour
{
    private ROSConnection ros;
    public string topicName = "/unity_override";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<BoolMsg>(topicName);
    }

    public void PausePatrol()
    {
        // Publish True to stop patrol (Unity is taking over)
        ros.Publish(topicName, new BoolMsg(true));
        Debug.Log("Patrol paused (override = true)");
    }

    public void ResumePatrol()
    {
        // Publish False to resume patrol
        ros.Publish(topicName, new BoolMsg(false));
        Debug.Log("Patrol resumed (override = false)");
    }
}

