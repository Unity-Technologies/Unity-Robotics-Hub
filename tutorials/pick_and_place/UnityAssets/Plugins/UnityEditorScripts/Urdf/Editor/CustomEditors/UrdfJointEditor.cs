   

using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    [CustomEditor(typeof(UrdfJoint), true)]
    public class UrdfJointEditor : UnityEditor.Editor
    {
        private UrdfJoint urdfJoint;
        private bool showDetails;

        public override void OnInspectorGUI()
        {
            urdfJoint = (UrdfJoint) target;

            GUILayout.Space(5);

            UrdfJoint.JointTypes newJointType = urdfJoint.JointType;

            EditorGUILayout.BeginVertical("HelpBox");
            newJointType = (UrdfJoint.JointTypes)EditorGUILayout.EnumPopup(
                "Type of joint", newJointType);
            if (newJointType != urdfJoint.JointType)
            {
                if (EditorUtility.DisplayDialog("Confirm joint type change",
                    "Are you sure you want to change the joint type? This will erase all information currently stored in the joint.",
                    "Continue", "Cancel"))
                {
                    UrdfJoint.ChangeJointType(urdfJoint.gameObject, newJointType);
                }
            }
            EditorGUILayout.EndVertical();

            showDetails = EditorGUILayout.Foldout(showDetails, "Joint URDF Configuration", true);
            if (showDetails)
            {
                urdfJoint.jointName = EditorGUILayout.TextField("Name", urdfJoint.jointName);

                if (urdfJoint.JointType != UrdfJoint.JointTypes.Fixed)
                    GUILayout.BeginVertical("HelpBox");
                switch (urdfJoint.JointType)
                {
                    case UrdfJoint.JointTypes.Fixed:
                        break;
                    case UrdfJoint.JointTypes.Continuous:
                        DisplayDynamicsMessage("HingeJoint > Spring > Damper (for damping) and Spring (for friction)");
                        DisplayAxisMessage("HingeJoint > Axis");
                        break;
                    case UrdfJoint.JointTypes.Revolute:
                        DisplayDynamicsMessage("HingeJoint > Spring > Damper (for damping) and Spring (for friction)");
                        DisplayAxisMessage("HingeJoint > Axis");
                        DisplayRequiredLimitMessage("Hinge Joint Limits Manager > Large Angle Limit  / Max");
                        break;
                    case UrdfJoint.JointTypes.Floating:
                        DisplayDynamicsMessage("ConfigurableJoint > xDrive > Position Damper (for Damping) and Position Spring (for friction)");
                        break;
                    case UrdfJoint.JointTypes.Prismatic:
                        DisplayDynamicsMessage("ConfigurableJoint > xDrive > Position Damper (for Damping) and Position Spring (for friction)");
                        DisplayAxisMessage("ConfigurableJoint > Axis");
                        DisplayRequiredLimitMessage("Prismatic Joint Limits Manager > Position Limit Min / Max");
                        break;
                    case UrdfJoint.JointTypes.Planar:
                        DisplayDynamicsMessage("ConfigurableJoint > xDrive > Position Damper (for Damping) and Position Spring (for friction)");
                        DisplayAxisMessage("ConfigurableJoint > Axis and Secondary Axis");
                        DisplayRequiredLimitMessage("ConfigurableJoint > Linear Limit > Limit");
                        break;
                }

                if (urdfJoint.JointType != UrdfJoint.JointTypes.Fixed)
                    GUILayout.EndVertical();
            }
        }

        private void DisplayDynamicsMessage(string dynamicsLocation)
        {
            GUILayout.Space(5);
            EditorGUILayout.LabelField("Joint Dynamics (optional)");

            EditorGUILayout.HelpBox("To define damping and friction values, edit the fields " + dynamicsLocation + ".", MessageType.Info);

        }

        private void DisplayAxisMessage(string axisLocation)
        {
            GUILayout.Space(5);
            
            EditorGUILayout.LabelField("Joint Axis");

            EditorGUILayout.HelpBox("An axis is required for this joint type. Remember to define an axis in " + axisLocation + ".", MessageType.Info);
        }

        public void DisplayRequiredLimitMessage(string limitLocation)
        {
            GUILayout.Space(5);
            EditorGUILayout.LabelField("Joint Limits");

            urdfJoint.EffortLimit = EditorGUILayout.DoubleField("Effort Limit", urdfJoint.EffortLimit);
            urdfJoint.VelocityLimit = EditorGUILayout.DoubleField("Velocity Limit", urdfJoint.VelocityLimit);

            if (!urdfJoint.AreLimitsCorrect())
                EditorGUILayout.HelpBox("Limits are required for this joint type. Please enter valid limit values in " + limitLocation + ".", MessageType.Warning);

            GUILayout.Space(5);
        }
    }
}
