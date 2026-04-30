# Unity — Brick subscriber setup

## Prerequisites
- ArUco detection in Python/ROS is confirmed publishing on `/aruco/marker_0_pose`
- Unity project already has ROS-TCP-Connector installed (it does — you use it for joint states)
- The TwinJointStatePublisher and JointSliderController already work

## Step 1 — Regenerate ROS messages (if needed)

PoseStamped comes from `geometry_msgs`. You've almost certainly already generated this because it's a standard message. Verify:

1. Unity top menu: **Robotics > Generate ROS Messages...**
2. Click the "Browse" button next to "ROS message path", navigate to your ROS 2 workspace's
   `install/geometry_msgs/share/geometry_msgs/msg/` directory.
3. Look for `PoseStamped.msg`. If it has a green checkmark next to it, you're done.
   If red, click "Build msg" next to that line.
4. Close the window.

## Step 2 — Create a brick GameObject

In Unity's scene hierarchy:

1. Right-click in Hierarchy > **3D Object > Cube**
2. Rename to "Brick" in the Inspector
3. Set Transform > Scale to something visible, e.g., (0.075, 0.014, 0.024) metres
   — matches real Jenga brick dimensions in metres
4. Pick a visible colour: drag a new Material onto the brick, set colour to a bright red
   so it's obvious in the scene

## Step 3 — Attach BrickPoseSubscriber

1. Select the Brick GameObject in Hierarchy
2. In the Inspector click "Add Component"
3. Search for "Brick Pose Subscriber" and select it
4. Configure in Inspector:
   - **Pose Topic**: `/aruco/marker_0_pose`
   - **Brick Transform**: drag the Brick GameObject itself into this field
   - **Camera Offset**: (0, 0, 0) initially — we'll calibrate later
   - **Position Smoothing Tau**: 0 (important — any smoothing hides network lag)
   - **Scale**: 1
   - **Verbose**: true (for debugging)

## Step 4 — Confirm subscription at runtime

1. Make sure ROS backend is running:
   - RealSense launched
   - ArUco detector running
   - ROS-TCP-Endpoint running (either via launch file or directly)

2. Press Play in Unity

3. Watch the Console — you should see:
   ```
   [BrickPoseSubscriber] subscribed to /aruco/marker_0_pose; target = Brick
   ```

4. Hold the marker in front of the RealSense. The brick in the Unity scene should
   start moving. Every 2 seconds the Console logs:
   ```
   [BrickPoseSubscriber] rx=60 pos=(0.023, -0.012, 0.310)
   ```

5. Move the marker up and down in the real world; the brick should move up and down
   in the Unity scene. (Axis directions may feel inverted — that's a
   calibration-not-correctness issue, fix later.)

## Step 5 — Troubleshooting

**Brick never moves:**
- Check Unity Console for subscription line
- In a separate terminal, confirm ROS side is alive:
  `ros2 topic hz /aruco/marker_0_pose`

**Brick moves chaotically or jumps far away:**
- `marker_size_m` on the Python side doesn't match displayed/printed size
- The pose has extreme values in metres; your brick scale might send it off-screen
- Reduce `scale` field in Inspector to 0.5 or similar during testing

**Brick moves backwards:**
- Unity/ROS coordinate conventions are different. Open the script and toggle the
  `Unity.Y = -ROS.Y` line to `Unity.Y = ROS.Y`, or similar experiments.

## Step 6 — Calibrate `cameraOffset` (later)

Once detection is visible:

1. Place the marker at a known location in the real world, e.g., the centre of your workspace
2. In Unity, note the brick's current position
3. Enter the negative of that position as `cameraOffset` in the Inspector
4. Press Play again — the brick should now appear at the scene origin when the marker is
   at your workspace centre

This is a one-time setup. Save the values in your notes for reproducibility.

## What's measured when this works

The chain is now:
   real brick pose (via ArUco)
   -> PoseStamped msg (timestamped at detection time)
   -> ROS-TCP-Connector over localhost
   -> Unity BrickPoseSubscriber
   -> Unity brick Transform updated

Under tc netem applied on the robot's ethernet interface, ArUco detection is NOT
directly impaired (camera is USB, not ethernet). So K-AoI_obj measures a DIFFERENT
channel than K-AoI_eef. This is actually useful — it means you get:

- K-AoI_eef: impaired by tc netem (robot ethernet)
- K-AoI_obj: NOT impaired by tc netem (camera USB)

For your paper you'll note that K-AoI_obj under the current setup is a control
condition — the object tracking path is free of the impairment, so observed K-AoI_obj
represents baseline perceptual lag.

To actually IMPAIR the object tracking path, you would need to either:
1. Apply netem on a different interface (loopback? WiFi to a remote camera?)
2. Add artificial delay inside the aruco_detector node (simpler — add a
   --ros-args -p delay_ms:=NNN parameter)

For the paper we can discuss this in Limitations / Future Work.
