using System.IO;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using RosSharp.Control;

namespace RosSharp.Urdf.Editor
{
    public class AddDhParameterWindow : EditorWindow
    {
        public FKRobot script;
        public float alpha;
        public float a;
        public float d;
        public float theta;
        public Vector2 scrollpos;
        public string dhDisplay = "";
        public int count = 0;


        public void OnGUI()
        {
            
            GUILayout.Space(10);

            GUILayout.Space(10);
            alpha = EditorGUILayout.FloatField("Alpha", alpha);
            a = EditorGUILayout.FloatField("a", a);
            theta = EditorGUILayout.FloatField("theta", theta);
            d = EditorGUILayout.FloatField("d", d);

            scrollpos = EditorGUILayout.BeginScrollView(scrollpos,false,true ,GUILayout.Width(400), GUILayout.Height(300));
            EditorGUILayout.TextArea(dhDisplay);
            EditorGUILayout.EndScrollView();

            if (GUILayout.Button("Add DH Parameter"))
            {
                if (script.dh.Count >= script.jointChain.Count)
                {
                    EditorUtility.DisplayDialog("Articulation Error",
                        "Number of DH paramaters cannot be more than number of joints", "Ok");
                }
                else
                {


                    if (float.IsNaN(a) || float.IsNaN(d) || float.IsNaN(theta) || float.IsNaN(alpha))
                    {
                        EditorUtility.DisplayDialog("Float Error",
                            "Float Values are Null", "Ok");
                    }
                    else
                    {
                        script.dh.Add(new float[] { alpha, a, theta, d });
                        count++;
                        dhDisplay += string.Format("Joint {0}: Alpha: {1} d:{2} Theta:{3} a:{4}\n", count, alpha, d, theta, a);
                        alpha = a = theta = d = float.NaN;
                    }
                }
            }

            if(GUILayout.Button("Clear DH Parameters"))
            {
                alpha = a = theta = d = float.NaN;

                script.dh.Clear();
                dhDisplay = "";
                count = 0;
            }
        }

        private void OnDestroy()
        {
            SetEditorPrefs();
        }

        private void SetEditorPrefs()
        {
        }

        public void GetEditorPrefs()
        {
        }
    }
}
