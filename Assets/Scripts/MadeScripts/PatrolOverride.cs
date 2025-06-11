using UnityEngine;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class PatrolOverride : MonoBehaviour
{
    ROSConnection ros;
    public string overrideTopic = "/unity_override";
    public bool overrideActive = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<BoolMsg>(overrideTopic);
    }

    void Update()
    {
        // For testing: press 'O' to toggle override
        if (Input.GetKeyDown(KeyCode.O))
        {
            overrideActive = !overrideActive;
            PublishOverride(overrideActive);
        }
    }

    public void TriggerOverride()
    {
        overrideActive = true;
        PublishOverride(true);
    }

    public void ClearOverride()
    {
        overrideActive = false;
        PublishOverride(false);
    }

    void PublishOverride(bool state)
    {
        BoolMsg msg = new BoolMsg(state);
        ros.Publish(overrideTopic, msg);
        Debug.Log("Published override: " + state);
    }
}
