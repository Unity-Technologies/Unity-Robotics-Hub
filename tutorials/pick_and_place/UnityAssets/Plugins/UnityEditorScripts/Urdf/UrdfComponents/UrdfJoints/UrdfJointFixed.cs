   
using UnityEngine;

namespace RosSharp.Urdf
{
    public class UrdfJointFixed : UrdfJoint
    {
        public override JointTypes JointType => JointTypes.Fixed;

        public static UrdfJoint Create(GameObject linkObject)
        {
            UrdfJointFixed urdfJoint = linkObject.AddComponent<UrdfJointFixed>();
            #if UNITY_2020_1_OR_NEWER
                urdfJoint.unityJoint = linkObject.AddComponent<ArticulationBody>();
            #else
                        urdfJoint.UnityJoint = linkObject.AddComponent<FixedJoint>();
                        urdfJoint.UnityJoint.autoConfigureConnectedAnchor = true;
            #endif

            return urdfJoint;
        }

        protected override bool IsJointAxisDefined()
        {
            return true; //Axis isn't used
        }
    }
}

