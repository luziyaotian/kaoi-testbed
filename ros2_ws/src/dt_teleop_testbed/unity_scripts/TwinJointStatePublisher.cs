using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

using JointStateMsg = RosMessageTypes.Sensor.JointStateMsg;
using HeaderMsg = RosMessageTypes.Std.HeaderMsg;
using TimeMsg = RosMessageTypes.BuiltinInterfaces.TimeMsg;

/// <summary>
/// TwinJointStatePublisher
///
/// Reads joint angles from the 6-DOF Lite 6 ArticulationBody chain and
/// publishes them as sensor_msgs/JointState on /twin/joint_states.
///
/// Drop-in replacement for the headless_twin ROS2 node. Downstream
/// components (bridge, AoI logger, network conditioner) do not need to
/// know the twin is Unity — they just subscribe to /twin/joint_states.
///
/// Setup:
///   1. Attach to the imported Lite 6 root GameObject (UF_ROBOT).
///   2. In the Inspector, drag the 6 ArticulationBody joints (link1..link6)
///      into the jointBodies array, in order.
///   3. Hit Play. Verify with:   ros2 topic hz /twin/joint_states
/// </summary>
public class TwinJointStatePublisher : MonoBehaviour
{
    [Header("ROS topic")]
    public string topicName = "/twin/joint_states";

    [Tooltip("Publish rate in Hz. 50 Hz matches the headless twin default.")]
    public float publishRateHz = 50f;

    [Header("Joint references (order matters!)")]
    [Tooltip("Assign ArticulationBody components for joint1..joint6 in order.")]
    public ArticulationBody[] jointBodies = new ArticulationBody[6];

    [Tooltip("Names published in JointState.name, in same order as jointBodies.")]
    public string[] jointNames = new string[] {
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    };

    // --- private ---
    private ROSConnection _ros;
    private float _publishPeriod;
    private float _timeAccumulator;
    private double[] _lastPositions;
    private float _lastPublishTime;
    private int _publishCount;

    void Start()
    {
        if (jointBodies == null || jointBodies.Length != 6)
        {
            Debug.LogError(
                "[TwinJointStatePublisher] jointBodies must have exactly 6 entries. " +
                "Assign the 6 Lite 6 joints in the Inspector.");
            enabled = false;
            return;
        }
        if (jointNames == null || jointNames.Length != 6)
        {
            Debug.LogError("[TwinJointStatePublisher] jointNames must have 6 entries.");
            enabled = false;
            return;
        }
        for (int i = 0; i < 6; i++)
        {
            if (jointBodies[i] == null)
            {
                Debug.LogError(
                    $"[TwinJointStatePublisher] jointBodies[{i}] is null. " +
                    "Assign all 6 joints in the Inspector.");
                enabled = false;
                return;
            }
        }

        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<JointStateMsg>(topicName);

        _publishPeriod = 1f / Mathf.Max(1f, publishRateHz);
        _timeAccumulator = 0f;
        _lastPositions = new double[6];
        _lastPublishTime = Time.realtimeSinceStartup;
        _publishCount = 0;

        Debug.Log(
            $"[TwinJointStatePublisher v2-clockfix] publishing {topicName} at {publishRateHz} Hz; " +
            $"using DateTimeOffset.UtcNow directly for stamps");
    }

    void Update()
    {
        _timeAccumulator += Time.deltaTime;
        if (_timeAccumulator < _publishPeriod) return;
        _timeAccumulator = 0f;

        var positions = new double[6];
        var velocities = new double[6];
        float now = Time.realtimeSinceStartup;
        float dt = now - _lastPublishTime;
        for (int i = 0; i < 6; i++)
        {
            var ab = jointBodies[i];
            positions[i] = ab.jointPosition.dofCount > 0 ? ab.jointPosition[0] : 0.0;
            velocities[i] = (dt > 1e-5f && _publishCount > 0)
                ? (positions[i] - _lastPositions[i]) / dt
                : 0.0;
        }
        _lastPublishTime = now;

        // Use system Unix wall-clock time directly — no cached offset trick.
        // This matches time.time() on the ROS side, so AoI comes out positive.
        double tNow = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;
        int secPart = (int)Math.Floor(tNow);
        uint nsecPart = (uint)((tNow - secPart) * 1e9);

        var msg = new JointStateMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg { sec = secPart, nanosec = nsecPart },
                frame_id = "world",
            },
            name = jointNames,
            position = positions,
            velocity = velocities,
            effort = new double[0],
        };

        _ros.Publish(topicName, msg);

        Array.Copy(positions, _lastPositions, 6);
        _publishCount++;

        if (_publishCount == 1 || _publishCount % 500 == 0)
        {
            Debug.Log(
                $"[TwinJointStatePublisher] publish #{_publishCount}: " +
                $"j1={positions[0]:F3} j2={positions[1]:F3} j3={positions[2]:F3} " +
                $"j4={positions[3]:F3} j5={positions[4]:F3} j6={positions[5]:F3}");
        }
    }
}
