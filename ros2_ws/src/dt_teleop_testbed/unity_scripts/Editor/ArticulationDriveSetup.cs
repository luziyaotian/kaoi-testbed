#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

/// <summary>
/// ArticulationDriveSetup — Editor utility.
///
/// Purpose: Persistently configure ArticulationBody drives on a robot so
/// joints actually track their targets and don't sag under gravity.
///
/// Why this exists: URDF-Importer leaves drives at stiffness=0, damping=0.
/// Configuring drives at runtime via a MonoBehaviour's Start() works but
/// values revert whenever you exit Play mode. This utility writes the
/// values straight into the scene file, so they survive Play/Stop cycles,
/// project reopens, etc.
///
/// Usage:
///   1. Select the root robot GameObject (e.g. UF_ROBOT) in Hierarchy.
///   2. Right-click → "Robot Utilities/Configure Articulation Drives".
///   3. The script finds all ArticulationBody components under the root,
///      sets stiffness/damping/forceLimit/driveType on each revolute joint,
///      and marks the scene dirty so Unity saves the change.
///   4. Save the scene (Ctrl+S).
///
/// Repeat whenever you reimport the URDF.
/// </summary>
public static class ArticulationDriveSetup
{
    private const float DEFAULT_STIFFNESS = 10000f;
    private const float DEFAULT_DAMPING = 1000f;
    private const float DEFAULT_FORCE_LIMIT = 10000f;

    [MenuItem("GameObject/Robot Utilities/Configure Articulation Drives", false, 10)]
    public static void ConfigureDrives()
    {
        var go = Selection.activeGameObject;
        if (go == null)
        {
            EditorUtility.DisplayDialog(
                "Configure Articulation Drives",
                "No GameObject selected. Click the robot root in the Hierarchy first.",
                "OK");
            return;
        }

        var bodies = go.GetComponentsInChildren<ArticulationBody>(true);
        if (bodies.Length == 0)
        {
            EditorUtility.DisplayDialog(
                "Configure Articulation Drives",
                "No ArticulationBody components found under " + go.name,
                "OK");
            return;
        }

        int configured = 0;
        int skipped = 0;

        foreach (var ab in bodies)
        {
            // Only configure revolute joints — fixed joints don't need drives.
            if (ab.jointType != ArticulationJointType.RevoluteJoint
                && ab.jointType != ArticulationJointType.PrismaticJoint)
            {
                skipped++;
                continue;
            }

            // Record for undo so Ctrl+Z works.
            Undo.RecordObject(ab, "Configure Articulation Drive");

            var drive = ab.xDrive;
            drive.stiffness = DEFAULT_STIFFNESS;
            drive.damping = DEFAULT_DAMPING;
            drive.forceLimit = DEFAULT_FORCE_LIMIT;
            drive.driveType = ArticulationDriveType.Acceleration;
            ab.xDrive = drive;

            // Mark the component dirty so the change is saved with the scene.
            EditorUtility.SetDirty(ab);
            configured++;
        }

        // Mark the scene dirty so Ctrl+S actually has something to save.
        if (configured > 0)
        {
            UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(
                go.scene);
        }

        Debug.Log(
            $"[ArticulationDriveSetup] Configured {configured} drives " +
            $"(stiffness={DEFAULT_STIFFNESS}, damping={DEFAULT_DAMPING}, " +
            $"forceLimit={DEFAULT_FORCE_LIMIT}, driveType=Acceleration). " +
            $"Skipped {skipped} fixed joints. Save the scene to persist.");
    }

    [MenuItem("GameObject/Robot Utilities/Configure Articulation Drives", true)]
    private static bool ValidateConfigureDrives()
    {
        return Selection.activeGameObject != null;
    }
}
#endif
