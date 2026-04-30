using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

using JointStateMsg = RosMessageTypes.Sensor.JointStateMsg;

/// <summary>
/// JointSliderController
///
/// On-screen panel with 6 sliders for jogging the Lite 6 twin. Writes each
/// slider value into the corresponding ArticulationBody's drive target.
///
/// Auto-seeding: on Play, subscribes to /ufactory/joint_states and waits
/// up to maxSeedWaitSec for the first message. When received, initial
/// slider values are set to match the real robot's current pose. This
/// prevents the twin from jerking to zero on Play.
///
/// If no seed message arrives in time, sliders default to the twin's
/// current joint angles at Play start (also safe — no jerk).
///
/// Setup:
///   1. Attach to UF_ROBOT.
///   2. Joint Bodies: drag link1..link6 ArticulationBodies in order.
///   3. Seed Topic: /ufactory/joint_states (default)
///   4. Hit Play.
///
/// Runtime controls:
///   - Drag sliders: joint moves
///   - H: hide/show panel
///   - R: reset sliders to seeded initial values
/// </summary>
public class JointSliderController : MonoBehaviour
{
    [Header("Joint targets (assign 6 ArticulationBodies)")]
    public ArticulationBody[] jointBodies = new ArticulationBody[6];

    [Header("Auto-seed from real robot (or another publisher)")]
    [Tooltip("Topic to read initial joint values from. " +
             "For real robot: /ufactory/joint_states. " +
             "For mock: /lite6_real/joint_states. " +
             "Leave blank to disable auto-seeding.")]
    public string seedTopic = "/ufactory/joint_states";

    [Tooltip("Max time to wait for first seed message before falling back " +
             "to current twin pose.")]
    public float maxSeedWaitSec = 2.0f;

    [Header("Per-joint slider limits (rad) — Lite 6 specs with 5° safety margin")]
    // Official UFactory Lite 6 Hardware Manual V2.6.0:
    //   J1: ±360°          = ±6.283 rad   (reduced to ±6.1 for safety)
    //   J2: ±150°          = ±2.618 rad   (reduced to ±2.5 for safety)
    //   J3: -3.5° to 300°  = -0.061 to 5.236 rad  (reduced to [0.1, 5.0] for safety)
    //   J4: ±360°          = ±6.283 rad   (reduced to ±6.1 for safety)
    //   J5: ±124°          = ±2.164 rad   (reduced to ±2.05 for safety)
    //   J6: ±360°          = ±6.283 rad   (reduced to ±6.1 for safety)
    //
    // The safety margin exists because ArticulationBody drives can momentarily
    // overshoot their target during fast slider motion or physics transitions.
    // Keeping sliders well inside the hardware limits prevents the real robot
    // from seeing a command at or past its hard limit (which triggers error).
    //
    // Joint 3's lower bound is particularly tight: hardware allows -3.5° but
    // we cap at +5.7° (0.1 rad) to give plenty of overshoot margin.
    public float[] lowerLimits = new float[] {
        -6.1f, -2.5f,  0.1f, -6.1f, -2.05f, -6.1f
    };
    public float[] upperLimits = new float[] {
         6.1f,  2.5f,  5.0f,  6.1f,  2.05f,  6.1f
    };

    [Header("UI")]
    public Rect panelRect = new Rect(20, 20, 360, 280);

    [Header("Drive tuning")]
    public float driveStiffness = 10000f;
    public float driveDamping = 1000f;
    public float driveForceLimit = 10000f;

    [Tooltip("Re-apply drive stiffness/damping every FixedUpdate to defeat " +
             "URDF-Importer's runtime override.")]
    public bool enforceDrivesEveryTick = true;

    // --- runtime state ---
    private float[] _sliderValues = new float[6];
    private float[] _resetValues = new float[6];   // value to reset to (R key)
    private bool _visible = true;
    private bool _seeded = false;
    private float _playStartTime;
    private ROSConnection _ros;

    void Start()
    {
        if (jointBodies == null || jointBodies.Length != 6)
        {
            Debug.LogError("[JointSliderController] jointBodies must have 6 entries.");
            enabled = false;
            return;
        }
        for (int i = 0; i < 6; i++)
        {
            if (jointBodies[i] == null)
            {
                Debug.LogError($"[JointSliderController] jointBodies[{i}] is null.");
                enabled = false;
                return;
            }
        }

        // Default slider values = twin's current joint angles. Safe fallback
        // if seeding fails.
        for (int i = 0; i < 6; i++)
        {
            float current = jointBodies[i].jointPosition.dofCount > 0
                ? jointBodies[i].jointPosition[0]
                : 0f;
            _sliderValues[i] = Mathf.Clamp(current, lowerLimits[i], upperLimits[i]);
            _resetValues[i] = _sliderValues[i];
        }

        ApplyDriveSettings();
        ApplySliderTargets();

        _playStartTime = Time.realtimeSinceStartup;

        if (!string.IsNullOrEmpty(seedTopic))
        {
            _ros = ROSConnection.GetOrCreateInstance();
            _ros.Subscribe<JointStateMsg>(seedTopic, OnSeedMessage);
            Debug.Log($"[JointSliderController] waiting for seed from {seedTopic} " +
                      $"(timeout {maxSeedWaitSec}s)...");
        }
        else
        {
            _seeded = true;
            Debug.Log("[JointSliderController] seeding disabled; using twin pose.");
        }
    }

    private void OnSeedMessage(JointStateMsg msg)
    {
        if (_seeded) return;
        if (msg.position == null || msg.position.Length < 6)
        {
            Debug.LogWarning("[JointSliderController] seed message has < 6 joints; " +
                             "ignoring.");
            return;
        }
        for (int i = 0; i < 6; i++)
        {
            float v = Mathf.Clamp((float)msg.position[i],
                                  lowerLimits[i], upperLimits[i]);
            _sliderValues[i] = v;
            _resetValues[i] = v;
        }
        _seeded = true;
        Debug.Log($"[JointSliderController] seeded from {seedTopic}: [" +
                  $"{_sliderValues[0]:F3}, {_sliderValues[1]:F3}, " +
                  $"{_sliderValues[2]:F3}, {_sliderValues[3]:F3}, " +
                  $"{_sliderValues[4]:F3}, {_sliderValues[5]:F3}]");

        // Drop the subscription — we only need one message.
        if (_ros != null)
            _ros.Unsubscribe(seedTopic);
    }

    void ApplyDriveSettings()
    {
        for (int i = 0; i < 6; i++)
        {
            var ab = jointBodies[i];
            var drive = ab.xDrive;
            drive.stiffness = driveStiffness;
            drive.damping = driveDamping;
            drive.forceLimit = driveForceLimit;
            drive.driveType = ArticulationDriveType.Acceleration;
            ab.xDrive = drive;
        }
    }

    void ApplySliderTargets()
    {
        for (int i = 0; i < 6; i++)
        {
            var ab = jointBodies[i];
            var drive = ab.xDrive;
            drive.target = _sliderValues[i] * Mathf.Rad2Deg;
            ab.xDrive = drive;
        }
    }

    void Update()
    {
        // Timeout check — if seed never arrived, stop waiting after the deadline
        if (!_seeded && (Time.realtimeSinceStartup - _playStartTime) > maxSeedWaitSec)
        {
            _seeded = true;
            Debug.LogWarning(
                $"[JointSliderController] seed timeout after {maxSeedWaitSec}s; " +
                $"using twin pose fallback: [" +
                $"{_sliderValues[0]:F3}, {_sliderValues[1]:F3}, " +
                $"{_sliderValues[2]:F3}, {_sliderValues[3]:F3}, " +
                $"{_sliderValues[4]:F3}, {_sliderValues[5]:F3}]");
            if (_ros != null)
                _ros.Unsubscribe(seedTopic);
        }

        if (Input.GetKeyDown(KeyCode.H))
            _visible = !_visible;

        if (Input.GetKeyDown(KeyCode.R))
            for (int i = 0; i < 6; i++) _sliderValues[i] = _resetValues[i];

        ApplySliderTargets();
    }

    void FixedUpdate()
    {
        if (enforceDrivesEveryTick)
            ApplyDriveSettings();
    }

    void OnGUI()
    {
        if (!_visible) return;
        GUI.Box(panelRect, "Lite 6 Twin  (H: hide, R: reset)");

        float y = panelRect.y + 25;
        for (int i = 0; i < 6; i++)
        {
            GUI.Label(new Rect(panelRect.x + 10, y, 60, 20),
                      $"joint{i + 1}");
            _sliderValues[i] = GUI.HorizontalSlider(
                new Rect(panelRect.x + 70, y + 5, 200, 20),
                _sliderValues[i],
                lowerLimits[i], upperLimits[i]);
            GUI.Label(new Rect(panelRect.x + 280, y, 70, 20),
                      $"{_sliderValues[i]:F2} rad");
            y += 30;
        }

        if (!_seeded)
        {
            GUI.Label(new Rect(panelRect.x + 10, y + 10, panelRect.width - 20, 20),
                      $"Waiting for seed from {seedTopic}...");
        }
    }
}
