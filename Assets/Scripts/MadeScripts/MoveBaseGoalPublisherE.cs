using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;

public class MoveBaseGoalPublisherE : MonoBehaviour
{
    ROSConnection ros;
    public string goalTopic = "/goal_pose";
    public string initPoseTopic = "/initialpose";

    public Transform goalFrame;
    public Transform initialPoseFrame;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(goalTopic);
        ros.RegisterPublisher<PoseWithCovarianceStampedMsg>(initPoseTopic);

        // Send initial pose estimate
        PublishInitialPose();
    }

    public void PublishInitialPose()
    {
        if (initialPoseFrame == null)
        {
            Debug.LogWarning("Initial pose frame not set");
            return;
        }

        var initPose = new PoseWithCovarianceStampedMsg
        {
            header = new HeaderMsg
            {
                frame_id = "map",
                stamp = new TimeMsg { sec = 0, nanosec = 0 }
            },
            pose = new PoseWithCovarianceMsg
            {
                pose = new PoseMsg
                {
                    position = new PointMsg(
                        initialPoseFrame.position.x,
                        initialPoseFrame.position.y,
                        initialPoseFrame.position.z
                    ),
                    orientation = new QuaternionMsg(
                        initialPoseFrame.rotation.x,
                        initialPoseFrame.rotation.y,
                        initialPoseFrame.rotation.z,
                        initialPoseFrame.rotation.w
                    )
                },
                // Minimal covariance to make AMCL happy
                covariance = new double[36] { 
                    0.25, 0, 0, 0, 0, 0,
                    0, 0.25, 0, 0, 0, 0,
                    0, 0, 0.0, 0, 0, 0,
                    0, 0, 0, 0.0, 0, 0,
                    0, 0, 0, 0, 0.0, 0,
                    0, 0, 0, 0, 0, 0.06853891945200942
                }
            }
        };

        ros.Publish(initPoseTopic, initPose);
        Debug.Log($"Published initial pose to {initPoseTopic}");
    }

    public void PublishNavigationGoal()
    {
        if (goalFrame == null)
        {
            Debug.LogWarning("Goal frame not set");
            return;
        }

        var goalMsg = new PoseStampedMsg
        {
            header = new HeaderMsg
            {
                frame_id = "map",
                stamp = new TimeMsg { sec = 0, nanosec = 0 }
            },
            pose = new PoseMsg
            {
                position = new PointMsg(
                    goalFrame.position.x,
                    goalFrame.position.y,
                    goalFrame.position.z
                ),
                orientation = new QuaternionMsg(
                    goalFrame.rotation.x,
                    goalFrame.rotation.y,
                    goalFrame.rotation.z,
                    goalFrame.rotation.w
                )
            }
        };

        ros.Publish(goalTopic, goalMsg);
        Debug.Log($"Published navigation goal to {goalTopic}");
    }
}
