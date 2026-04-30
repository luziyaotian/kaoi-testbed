using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

using JointStateMsg = RosMessageTypes.Sensor.JointStateMsg;

/// <summary>
/// RealRobotStateSubscriber
///
/// Subscribes to /lite6_real/joint_states (published by the twin_to_lite6_bridge
/// when the real robot is running, or by the mock_lite6 node in mock mode).
/// Drives a *second* ArticulationBody chain in the Unity scene — a "ghost"
/// duplicate of the Lite 6 with transparent/coloured material — so the
/// operator can visually see how far behind the real robot is as network
/// conditions degrade.
///
/// This is purely diagnostic, not part of the measurement pipeline (the AoI
/// logger on the ROS side is the authoritative instrument). But it's an
/// intuitive demo for presentations and lets the operator 'feel' the AoI.
///
/// Setup:
///   1. Duplicate the imported Lite 6 in the scene. Call it "Lite6_Ghost".
///   2. Assign its 6 ArticulationBodies to ghostJointBodies below.
///   3. Recolour the ghost's meshes (e.g., red, 50% alpha).
///   4. Attach this script anywhere, set topicName = /lite6_real/joint_states.
/// </summary>
public class RealRobotStateSubscriber : MonoBehaviour
{
    [Header("ROS topic")]
    public string topicName = "/lite6_real/joint_states";

    [Header("Ghost joint references")]
    [Tooltip("Assign the ghost Lite 6's 6 ArticulationBodies in joint1..joint6 order.")]
    public ArticulationBody[] ghostJointBodies = new ArticulationBody[6];

    [Tooltip("Expected joint names in the incoming message (for reordering).")]
    public string[] expectedJointNames = new string[] {
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    };

    private ROSConnection _ros;
    private int _messageCount;

    void Start()
    {
        if (ghostJointBodies == null || ghostJointBodies.Length != 6)
        {
            Debug.LogError("[RealRobotStateSubscriber] ghostJointBodies must have 6 entries.");
            enabled = false;
            return;
        }
        for (int i = 0; i < 6; i++)
        {
            if (ghostJointBodies[i] == null)
            {
                Debug.LogError($"[RealRobotStateSubscriber] ghostJointBodies[{i}] is null.");
                enabled = false;
                return;
            }
        }

        _ros = ROSConnection.GetOrCreateInstance();
        _ros.Subscribe<JointStateMsg>(topicName, OnMessage);

        Debug.Log($"[RealRobotStateSubscriber] subscribed to {topicName}");
    }

    void OnMessage(JointStateMsg msg)
    {
        if (msg.position == null || msg.position.Length == 0) return;

        // If names are provided and match our expected ordering, reorder.
        // Otherwise assume positional.
        double[] reordered = new double[6];
        bool haveAll = true;

        if (msg.name != null && msg.name.Length == msg.position.Length
                              && msg.name.Length >= 6)
        {
            for (int target = 0; target < 6; target++)
            {
                int found = System.Array.IndexOf(msg.name, expectedJointNames[target]);
                if (found < 0)
                {
                    haveAll = false;
                    break;
                }
                reordered[target] = msg.position[found];
            }
        }
        else
        {
            haveAll = false;
        }

        if (!haveAll)
        {
            if (msg.position.Length != 6) return;
            for (int i = 0; i < 6; i++) reordered[i] = msg.position[i];
        }

        // Write directly into the ghost's xDrive target (same convention as
        // JointSliderController). Note: for the ghost to *track* the target
        // you need its drives configured with non-zero stiffness. You can
        // either run JointSliderController.ConfigureDrives on the ghost via
        // a small setup script, or set Stiffness/Damping manually in the
        // Inspector on each joint (Stiffness ~100000, Damping ~10000).
        for (int i = 0; i < 6; i++)
        {
            var ab = ghostJointBodies[i];
            var drive = ab.xDrive;
            drive.target = (float)reordered[i] * Mathf.Rad2Deg;
            ab.xDrive = drive;
        }

        _messageCount++;
        if (_messageCount == 1 || _messageCount % 500 == 0)
        {
            Debug.Log(
                $"[RealRobotStateSubscriber] msg #{_messageCount}: " +
                $"j1={reordered[0]:F3} j2={reordered[1]:F3} j3={reordered[2]:F3}");
        }
    }
}
