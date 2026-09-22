using System;
using UnityEditor;
using UnityEngine;

namespace Unity.Simulation.VehicleControllers
{
    [CustomEditor(typeof(DifferentialDriveOdometryPublisher))]
    public class OdometryPublisherEditor : Editor
    {
        private DifferentialDriveOdometryPublisher odom;
        private int size;
        private bool b_ShowFoldout;
        private String[] m_PoseLabels = new String[] {"X", "Y", "Z", "Roll", "Pitch", "Yaw"};
        private String[] m_TwistLabels = new String[] {"Vx", "Vy", "Vz", "ωx", "ωy", "ωz"};
        private const float k_CellWidth = 80;
        public void OnEnable()
        {
            odom = target as DifferentialDriveOdometryPublisher;
            size = (int) Mathf.Sqrt(odom.PoseCovariance.Length);
        }

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            b_ShowFoldout = EditorGUILayout.Foldout(b_ShowFoldout, "Covariance Matrices", true);
            if (b_ShowFoldout)
            {
                EditorGUI.indentLevel++;
                EditorGUILayout.LabelField("Pose Covariance");
                // Horizontal Cell Headers
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField("", GUILayout.Width(k_CellWidth / 2));
                for (int i = 0; i < size; i++)
                {
                    EditorGUILayout.LabelField(m_PoseLabels[i], GUILayout.Width(k_CellWidth));
                }

                EditorGUILayout.EndHorizontal();

                for (int i = 0; i < size; i++)
                {
                    EditorGUILayout.BeginHorizontal();
                    EditorGUILayout.LabelField(m_PoseLabels[i], GUILayout.Width(k_CellWidth / 2));
                    for (int j = 0; j < size; j++)
                    {
                        EditorGUILayout.DoubleField(odom.PoseCovariance[(i * 6) + j], GUILayout.Width(k_CellWidth));
                    }
                    EditorGUILayout.EndHorizontal();
                }

                EditorGUILayout.LabelField("Twist Covariance");
                // Horizontal Cell Headers
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField("", GUILayout.Width(k_CellWidth / 2));
                for (int i = 0; i < size; i++)
                {
                    EditorGUILayout.LabelField(m_TwistLabels[i], GUILayout.Width(k_CellWidth));
                }

                EditorGUILayout.EndHorizontal();


                for (int i = 0; i < size; i++)
                {
                    EditorGUILayout.BeginHorizontal();
                    EditorGUILayout.LabelField(m_TwistLabels[i], GUILayout.Width(k_CellWidth / 2));
                    for (int j = 0; j < size; j++)
                    {
                        EditorGUILayout.DoubleField(odom.TwistCovariance[(i * 6) + j], GUILayout.Width(k_CellWidth));
                    }
                    EditorGUILayout.EndHorizontal();
                }
                EditorGUI.indentLevel--;
            }
        }
    }
}
