using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

using BoolMsg = RosMessageTypes.Std.BoolMsg;

/// <summary>
/// GripperController
///
/// Adds a single Open/Close toggle button to the Game view that publishes
/// std_msgs/Bool on /gripper/command. The gripper_bridge ROS node fans this
/// out to the real Lite 6 gripper via xarm_api services.
///
/// Behavior:
///   - First click: closes the gripper (true)
///   - Next click: opens (false)
///   - Keyboard shortcut: G (toggle)
///
/// Setup: attach to UF_ROBOT (same object as JointSliderController).
/// Nothing to wire up in the Inspector — defaults work for the standard
/// testbed topic layout.
/// </summary>
public class GripperController : MonoBehaviour
{
    [Header("ROS topic")]
    public string topicName = "/gripper/command";

    [Header("UI")]
    public Rect buttonRect = new Rect(20, 310, 200, 40);
    public KeyCode toggleKey = KeyCode.G;

    private ROSConnection _ros;
    private bool _closed = false;

    void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<BoolMsg>(topicName);
        Debug.Log($"[GripperController] publisher ready on {topicName} " +
                  $"(toggle with key '{toggleKey}' or on-screen button)");
    }

    void Update()
    {
        if (Input.GetKeyDown(toggleKey))
            Toggle();
    }

    void Toggle()
    {
        _closed = !_closed;
        _ros.Publish(topicName, new BoolMsg { data = _closed });
        Debug.Log($"[GripperController] published {topicName}: " +
                  $"{(_closed ? "CLOSE" : "OPEN")}");
    }

    void OnGUI()
    {
        string label = _closed ? "Gripper: CLOSED  (G to open)"
                               : "Gripper: OPEN    (G to close)";
        var prev = GUI.backgroundColor;
        GUI.backgroundColor = _closed ? new Color(0.8f, 0.4f, 0.4f)
                                      : new Color(0.4f, 0.8f, 0.4f);
        if (GUI.Button(buttonRect, label))
            Toggle();
        GUI.backgroundColor = prev;
    }
}
