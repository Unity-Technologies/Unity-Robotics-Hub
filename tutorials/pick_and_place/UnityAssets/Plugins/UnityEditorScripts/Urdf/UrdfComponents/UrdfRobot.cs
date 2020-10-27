  

using UnityEngine;

namespace RosSharp.Urdf
{
    public enum GeometryTypes { Box, Cylinder, Sphere, Mesh }
    public enum axisType
    {
        zAxis,
        yAxis,
    }

    public class UrdfRobot : MonoBehaviour
    {
        public string FilePath;
        public axisType choosenAxis;
        #region Configure Robot

        public void SetCollidersConvex(bool convex)
        {
            foreach (MeshCollider meshCollider in GetComponentsInChildren<MeshCollider>())
                meshCollider.convex = convex;
        }


        public void SetUseUrdfInertiaData(bool useUrdfData)
        {
            foreach (UrdfInertial urdfInertial in GetComponentsInChildren<UrdfInertial>())
                urdfInertial.useUrdfData = useUrdfData;
        }

        public void SetRigidbodiesUseGravity(bool useGravity)
        {
#if UNITY_2020_1_OR_NEWER
            foreach (ArticulationBody ar in GetComponentsInChildren<ArticulationBody>())
                ar.useGravity = useGravity;
#else
            foreach (Rigidbody rb in GetComponentsInChildren<Rigidbody>())
                rb.useGravity = useGravity;
#endif
        }

        public void GenerateUniqueJointNames()
        {
            foreach (UrdfJoint urdfJoint in GetComponentsInChildren<UrdfJoint>())
                urdfJoint.GenerateUniqueJointName();
        }

        // Add a rotation in the model which gives the correct correspondence between UnitySpace and RosSpace
        public void ChangeToCorrectedSpace(bool rosSpace)
        {
            this.transform.Rotate(0, 180, 0);
        }

        public void AddController(bool controller)
        {
            if (controller && this.gameObject.GetComponent< RosSharp.Control.Controller>() == null)
            {
                this.gameObject.AddComponent<RosSharp.Control.Controller>();
            }
            else
            {
                DestroyImmediate(this.gameObject.GetComponent<RosSharp.Control.Controller>());
                DestroyImmediate(this.gameObject.GetComponent<RosSharp.Control.FKRobot>());
                JointControl[] scriptList = GetComponentsInChildren<JointControl>();
                foreach (JointControl script in scriptList)
                    DestroyImmediate(script);
            }
        }

        public void AddFkRobot(bool fkRobot)
        {
            if (fkRobot && this.gameObject.GetComponent<RosSharp.Control.FKRobot>() == null)
            {
                this.gameObject.AddComponent<RosSharp.Control.FKRobot>();
            }
            else
            {
                DestroyImmediate(this.gameObject.GetComponent<RosSharp.Control.FKRobot>());
            }
        }

        public void SetAxis(Urdf.axisType setAxis)
        {
            this.choosenAxis = setAxis;
        }

        #endregion
    }
}
