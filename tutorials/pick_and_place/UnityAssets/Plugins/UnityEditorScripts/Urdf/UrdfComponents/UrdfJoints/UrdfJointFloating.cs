   
using UnityEngine;

namespace RosSharp.Urdf
{
    public class UrdfJointFloating : UrdfJoint
    {
        public override JointTypes JointType => JointTypes.Floating;

        public static UrdfJoint Create(GameObject linkObject)
        {
            UrdfJointFloating urdfJoint = linkObject.AddComponent<UrdfJointFloating>();
            #if UNITY_2020_1_OR_NEWER
                urdfJoint.unityJoint = linkObject.AddComponent<ArticulationBody>();
                //Doesnt have any equivalent Articulatiob Joint
            #else
            urdfJoint.UnityJoint = linkObject.AddComponent<ConfigurableJoint>();
            #endif
            return urdfJoint;
        }

#region Runtime

        public override float GetPosition()
        {
            #if UNITY_2020_1_OR_NEWER
                Debug.Log("'ArticulationBody' does not contain a definition for 'connectedAnchor' and no accessible extension method 'connectedAnchor'");
                Vector3 distanceFromAnchor = ((ArticulationBody)unityJoint).transform.localPosition ;/*-
                                         ((ArticulationBody)UnityJoint).connectedAnchor;*/
            #else
            Vector3 distanceFromAnchor = ((ConfigurableJoint)UnityJoint).transform.localPosition - 
                                         ((ConfigurableJoint)UnityJoint).connectedAnchor;
            #endif
            return distanceFromAnchor.magnitude;
        }

#endregion

        protected override bool IsJointAxisDefined()
        {
            return true; //Axis isn't used
        }
    }
}

