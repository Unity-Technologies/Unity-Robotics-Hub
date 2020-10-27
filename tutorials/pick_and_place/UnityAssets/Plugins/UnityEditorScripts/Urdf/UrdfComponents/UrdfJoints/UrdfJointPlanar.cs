   
using System;
using UnityEngine;

namespace RosSharp.Urdf
{
    public class UrdfJointPlanar : UrdfJoint
    {
        public override JointTypes JointType => JointTypes.Planar;

        public static UrdfJoint Create(GameObject linkObject)
        {
            UrdfJointPlanar urdfJoint = linkObject.AddComponent<UrdfJointPlanar>();
#if UNITY_2020_1_OR_NEWER
            urdfJoint.unityJoint = linkObject.AddComponent<ArticulationBody>();
            urdfJoint.unityJoint.jointType = ArticulationJointType.PrismaticJoint;
#else
            urdfJoint.unityJoint = linkObject.AddComponent<ConfigurableJoint>();
            urdfJoint.unityJoint.autoConfigureConnectedAnchor = true;
#endif


#if UNITY_2020_1_OR_NEWER
#else
            ConfigurableJoint configurableJoint = (ConfigurableJoint) urdfJoint.unityJoint;

            // degrees of freedom:
            configurableJoint.xMotion = ConfigurableJointMotion.Free;
            configurableJoint.yMotion = ConfigurableJointMotion.Free;
            configurableJoint.zMotion = ConfigurableJointMotion.Locked;
            configurableJoint.angularXMotion = ConfigurableJointMotion.Locked;
            configurableJoint.angularYMotion = ConfigurableJointMotion.Locked;
            configurableJoint.angularZMotion = ConfigurableJointMotion.Locked;
#endif
            return urdfJoint;
        }

        public override float GetPosition()
        {
            Vector3 distanceFromAnchor = unityJoint.transform.localPosition ;
            Debug.Log("'ArticulationBody' does not contain a definition for 'connectedAnchor' and no accessible extension method 'connectedAnchor'");
            return distanceFromAnchor.magnitude;
        }

        protected override void ImportJointData(Joint joint)
        {
#if UNITY_2020_1_OR_NEWER
            AdjustMovement(joint);



            if (joint.dynamics != null)
            {
                unityJoint.linearDamping = (float)joint.dynamics.damping;
                unityJoint.jointFriction = (float)joint.dynamics.friction;
            }
            else
            {
                unityJoint.linearDamping = 0;
                unityJoint.jointFriction = 0;
            }
#else
            ConfigurableJoint configurableJoint = (ConfigurableJoint)unityJoint;
            Vector3 normal = (joint.axis != null) ? GetAxis(joint.axis) : GetDefaultAxis();
            Vector3 axisX = Vector3.forward;
            Vector3 axisY = Vector3.left;
            Vector3.OrthoNormalize(ref normal, ref axisX, ref axisY);
            configurableJoint.axis = axisX;
            configurableJoint.secondaryAxis = axisY;

            // spring, damper & max. force:
            if (joint.dynamics != null)
            {
                configurableJoint.xDrive = GetJointDrive(joint.dynamics);
                configurableJoint.yDrive = GetJointDrive(joint.dynamics);
            }

            if (joint.limit != null)
                configurableJoint.linearLimit = GetLinearLimit(joint.limit);
#endif
        }

        #region Export

        protected override Joint ExportSpecificJointData(Joint joint)
        {
#if UNITY_2020_1_OR_NEWER
            joint.axis = GetAxisData(axisofMotion);
            joint.dynamics = new Joint.Dynamics(unityJoint.linearDamping, unityJoint.jointFriction);
            joint.limit = ExportLimitData();
#else
            ConfigurableJoint configurableJoint = (ConfigurableJoint)unityJoint;
            joint.axis = GetAxisData(Vector3.Cross(configurableJoint.axis, configurableJoint.secondaryAxis));
            joint.dynamics = new Joint.Dynamics(configurableJoint.xDrive.positionDamper, configurableJoint.xDrive.positionSpring);
            joint.limit = ExportLimitData();
#endif
            return joint;
        }

        protected override Joint.Limit ExportLimitData()
        {
#if UNITY_2020_1_OR_NEWER
            ArticulationDrive drive = GetComponent<ArticulationBody>().yDrive;
            return new Joint.Limit(drive.lowerLimit, drive.upperLimit, EffortLimit, VelocityLimit);
#else
            ConfigurableJoint configurableJoint = (ConfigurableJoint)unityJoint;
            return new Joint.Limit(
                Math.Round(-configurableJoint.linearLimit.limit, RoundDigits),
                Math.Round(configurableJoint.linearLimit.limit, RoundDigits),
                EffortLimit, VelocityLimit);
#endif
        }

        public override bool AreLimitsCorrect()
        {
#if UNITY_2020_1_OR_NEWER
            ArticulationBody joint = GetComponent<ArticulationBody>();
            return joint.linearLockY == ArticulationDofLock.LimitedMotion &&
                joint.linearLockZ == ArticulationDofLock.LimitedMotion &&
                joint.yDrive.lowerLimit < joint.yDrive.upperLimit &&
                joint.zDrive.lowerLimit < joint.zDrive.upperLimit;
#else
            ConfigurableJoint joint = (ConfigurableJoint)unityJoint;
            return joint != null && joint.linearLimit.limit != 0;
#endif
        }

        protected override bool IsJointAxisDefined() 
        {
#if UNITY_2020_1_OR_NEWER
            Debug.Log("Cannot convert type 'UnityEngine.ArticulationBody' to 'UnityEngine.ConfigurableJoint'");
            return false;
#else
            ConfigurableJoint joint = (ConfigurableJoint)unityJoint;
            return !(Math.Abs(joint.axis.x) < Tolerance &&
                     Math.Abs(joint.axis.y) < Tolerance &&
                     Math.Abs(joint.axis.z) < Tolerance)
                   && !(Math.Abs(joint.secondaryAxis.x) < Tolerance &&
                        Math.Abs(joint.secondaryAxis.y) < Tolerance &&
                        Math.Abs(joint.secondaryAxis.z) < Tolerance);
#endif
        }

        protected override void AdjustMovement(Joint joint) 
        {
            axisofMotion = (joint.axis == null || joint.axis.xyz == null) ? new Vector3(1, 0, 0) : new Vector3((float)joint.axis.xyz[0], (float)joint.axis.xyz[1], (float)joint.axis.xyz[2]);
            int motionAxis = joint.axis.AxisofMotion();
            Quaternion motion = unityJoint.anchorRotation;

            unityJoint.linearLockX = ArticulationDofLock.LockedMotion;
            if (joint.limit != null)
            {
                unityJoint.linearLockY = ArticulationDofLock.LimitedMotion;
                unityJoint.linearLockZ = ArticulationDofLock.LimitedMotion;
                ArticulationDrive drive = unityJoint.xDrive;
                drive.upperLimit = (float)joint.limit.upper;
                drive.lowerLimit = (float)joint.limit.lower;
                unityJoint.zDrive = drive;
                unityJoint.yDrive = drive;
            }
            else
            {
                unityJoint.linearLockZ = ArticulationDofLock.FreeMotion;
                unityJoint.linearLockY = ArticulationDofLock.FreeMotion;
            }

            switch (motionAxis)
            {
                case 0:
                    motion.eulerAngles = new Vector3(0, -90, 0);
                    break;
                case 1:
                    motion.eulerAngles = new Vector3(0, 0, 0);
                    break;
                case 2:
                    motion.eulerAngles = new Vector3(0, 0, 90);
                    break;
            }
            unityJoint.anchorRotation = motion;
        }

        #endregion
    }


}

