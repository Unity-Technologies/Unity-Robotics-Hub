  

using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    [CustomEditor(typeof(Rigidbody))]
    public class RigidbodyEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            Rigidbody _rigidbody = (Rigidbody)target;
            _rigidbody.centerOfMass = EditorGUILayout.Vector3Field("Center Of Mass", _rigidbody.centerOfMass);
            _rigidbody.inertiaTensor = EditorGUILayout.Vector3Field("Inertia Tensor", _rigidbody.inertiaTensor);

            Quaternion inertiaTensorRotation = new Quaternion();
            inertiaTensorRotation.eulerAngles = EditorGUILayout.Vector3Field("Inertia Tensor Rotation", _rigidbody.inertiaTensorRotation.eulerAngles);
            _rigidbody.inertiaTensorRotation = inertiaTensorRotation;
        }
    }
}