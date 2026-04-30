/*
 * BrickPoseSubscriber.cs
 * dt_teleop_testbed — digital-twin teleoperation testbed
 *
 * Subscribes to /aruco/marker_0_pose (geometry_msgs/PoseStamped) from
 * the ArUco detector ROS node and moves a GameObject (the "brick") in
 * the Unity scene to match the real-world brick's pose.
 *
 * Attach this script to a Unity GameObject that represents the brick
 * in the digital twin. Assign a "brick" reference (e.g., a cube) in
 * the Inspector. The script handles the Unity/ROS coordinate-frame
 * transformation.
 *
 * ROS camera frame (OpenCV convention):
 *     +X right, +Y down,  +Z forward (into scene)
 * Unity world frame:
 *     +X right, +Y up,    +Z forward
 * Transformation from ROS-camera to Unity-world (assuming the camera
 * is oriented identity at the scene origin):
 *     Unity.X =  ROS.X
 *     Unity.Y = -ROS.Y        (flip Y)
 *     Unity.Z =  ROS.Z
 *
 * For the paper, we place the RealSense camera looking down at the
 * workspace and use a calibration offset via the cameraOffset field.
 */

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class BrickPoseSubscriber : MonoBehaviour
{
    [Tooltip("Topic to subscribe to (PoseStamped).")]
    public string poseTopic = "/aruco/marker_0_pose";

    [Tooltip("GameObject whose transform will be updated to match the brick.")]
    public Transform brickTransform;

    [Tooltip("Offset from the RealSense camera origin to the Unity " +
             "scene origin. Measure this once by hand.")]
    public Vector3 cameraOffset = Vector3.zero;

    [Tooltip("If >0, lerp toward the target pose with this smoothing " +
             "time-constant (s). 0 = teleport. For sync-error measurement " +
             "this MUST be 0 — any smoothing hides real network lag.")]
    public float positionSmoothingTau = 0f;

    [Tooltip("Scale factor applied to position (e.g., 1.0 = metres).")]
    public float scale = 1.0f;

    [Tooltip("Set true to print receipt stats every 2 seconds.")]
    public bool verbose = true;

    private ROSConnection _ros;
    private Vector3 _targetPosition;
    private bool _haveTarget = false;
    private int _rxCount = 0;
    private float _lastLogTime = 0f;

    void Start()
    {
        if (brickTransform == null)
        {
            Debug.LogError("[BrickPoseSubscriber] brickTransform not " +
                           "assigned in Inspector — script disabled.");
            enabled = false;
            return;
        }

        _ros = ROSConnection.GetOrCreateInstance();
        _ros.Subscribe<PoseStampedMsg>(poseTopic, OnPoseReceived);

        Debug.Log($"[BrickPoseSubscriber] subscribed to {poseTopic}; " +
                  $"target = {brickTransform.gameObject.name}");
    }

    void OnPoseReceived(PoseStampedMsg msg)
    {
        // ROS camera-frame coords (metres), offset by camera pose in world
        float rosX = (float)msg.pose.position.x;
        float rosY = (float)msg.pose.position.y;
        float rosZ = (float)msg.pose.position.z;

        // Convert to Unity world frame.
        Vector3 unityPos = new Vector3(rosX, -rosY, rosZ) * scale
                           + cameraOffset;

        _targetPosition = unityPos;
        _haveTarget = true;
        _rxCount++;
    }

    void Update()
    {
        if (!_haveTarget) return;

        if (positionSmoothingTau > 0f)
        {
            float alpha = 1f - Mathf.Exp(-Time.deltaTime /
                                         positionSmoothingTau);
            brickTransform.position = Vector3.Lerp(
                brickTransform.position, _targetPosition, alpha);
        }
        else
        {
            brickTransform.position = _targetPosition;
        }

        if (verbose && (Time.unscaledTime - _lastLogTime) > 2f)
        {
            Debug.Log($"[BrickPoseSubscriber] rx={_rxCount} " +
                      $"pos={_targetPosition:F3}");
            _lastLogTime = Time.unscaledTime;
        }
    }

    void OnDisable()
    {
        if (_ros != null)
        {
            _ros.Unsubscribe(poseTopic);
        }
    }
}
