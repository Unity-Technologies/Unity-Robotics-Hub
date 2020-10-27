   

using System;
using UnityEngine;


namespace RosSharp.Urdf
{
    public class UrdfJointPrismatic : UrdfJoint
    {

        private ArticulationDrive drive;

        public override JointTypes JointType => JointTypes.Prismatic;

        public static UrdfJoint Create(GameObject linkObject)
        {
            UrdfJointPrismatic urdfJoint = linkObject.AddComponent<UrdfJointPrismatic>();
#if UNITY_2020_1_OR_NEWER
            linkObject.AddComponent<ArticulationBody>();
            urdfJoint.unityJoint = linkObject.GetComponent<ArticulationBody>(); 
            urdfJoint.unityJoint.jointType = ArticulationJointType.PrismaticJoint;
#else
            urdfJoint.unityJoint = linkObject.AddComponent<ConfigurableJoint>();
            urdfJoint.unityJoint.autoConfigureConnectedAnchor = true;

            ConfigurableJoint configurableJoint = (ConfigurableJoint) urdfJoint.unityJoint;

            // degrees of freedom:
            configurableJoint.xMotion = ConfigurableJointMotion.Limited;
            configurableJoint.yMotion = ConfigurableJointMotion.Locked;
            configurableJoint.zMotion = ConfigurableJointMotion.Locked;
            configurableJoint.angularXMotion = ConfigurableJointMotion.Locked;
            configurableJoint.angularYMotion = ConfigurableJointMotion.Locked;
            configurableJoint.angularZMotion = ConfigurableJointMotion.Locked;

            linkObject.AddComponent<PrismaticJointLimitsManager>();
#endif
            return urdfJoint;
        }

        #region Runtime

        /// <summary>
        /// Returns the current position of the joint in meters
        /// </summary>
        /// <returns>floating point number for joint position in meters</returns>
        public override float GetPosition()
        {
            #if UNITY_2020_1_OR_NEWER
                return unityJoint.jointPosition[3];
            #else
            return Vector3.Dot(unityJoint.transform.localPosition - unityJoint.connectedAnchor, unityJoint.axis);
            #endif
        }

        /// <summary>
        /// Returns the current velocity of joint in meters per second
        /// </summary>
        /// <returns>floating point for joint velocity in meters per second</returns>
        public override float GetVelocity()
        {
#if UNITY_2020_1_OR_NEWER
            return unityJoint.velocity[3];
#else
            return float.NaN;
#endif
        }

        /// <summary>
        /// Returns current joint torque in N
        /// </summary>
        /// <returns>floating point in N</returns>
        public override float GetEffort()
        {
#if UNITY_2020_1_OR_NEWER
            return unityJoint.jointForce[3];
#else
                return float.NaN;
#endif

        }

        /// <summary>
        /// Rotates the joint by deltaState m 
        /// </summary>
        /// <param name="deltaState">amount in m by which joint needs to be rotated</param>
        protected override void OnUpdateJointState(float deltaState)
        {
#if UNITY_2020_1_OR_NEWER
            ArticulationDrive drive = unityJoint.xDrive;
            drive.target += deltaState;
            unityJoint.xDrive = drive;
#else
            transform.Translate(unityJoint.axis * deltaState);
#endif
        }

#endregion

#region Import

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
                unityJoint.angularDamping = 0;
                unityJoint.jointFriction = 0;
            }
#else
            ArticulationBody prismaticJoint = (ArticulationBody) unityJoint;
            prismaticJoint.axis = (joint.axis != null) ? GetAxis(joint.axis) : GetDefaultAxis();

            if (joint.dynamics != null)
                prismaticJoint.xDrive = GetJointDrive(joint.dynamics);

            if (joint.limit != null)
            {
                PrismaticJointLimitsManager prismaticLimits = GetComponent<PrismaticJointLimitsManager>();
                prismaticLimits.InitializeLimits(joint.limit);
            }
#endif
        }

        /// <summary>
        /// Reads axis joint information and rotation to the articulation body to produce the required motion
        /// </summary>
        /// <param name="joint">Structure containing joint information</param>
        protected override void AdjustMovement(Joint joint) // Test this function
        {
            axisofMotion = (joint.axis != null && joint.axis.xyz != null) ? joint.axis.xyz.ToVector3() : new Vector3(1,0,0);
            unityJoint.linearLockX = (joint.limit != null) ? ArticulationDofLock.LimitedMotion : ArticulationDofLock.FreeMotion;
            unityJoint.linearLockY = ArticulationDofLock.LockedMotion;
            unityJoint.linearLockZ = ArticulationDofLock.LockedMotion;

            Vector3 axisofMotionUnity = axisofMotion.Ros2Unity();
            Quaternion Motion = new Quaternion();
            Motion.SetFromToRotation(new Vector3(1, 0, 0),axisofMotionUnity);
            unityJoint.anchorRotation = Motion;

            if (joint.limit != null)
            {
                ArticulationDrive drive = unityJoint.xDrive;
                drive.upperLimit = (float)joint.limit.upper;
                drive.lowerLimit = (float)joint.limit.lower;
                drive.forceLimit = (float)joint.limit.effort;
                unityJoint.maxLinearVelocity = (float)joint.limit.velocity;
                unityJoint.xDrive = drive;
            }
        }

#endregion


        #region Export

        protected override Joint ExportSpecificJointData(Joint joint)
        {
#if UNITY_2020_1_OR_NEWER
            joint.axis = GetAxisData(axisofMotion);
            joint.dynamics = new Joint.Dynamics(unityJoint.linearDamping, unityJoint.jointFriction);
            joint.limit = ExportLimitData();
#else
            ConfigurableJoint configurableJoint = (ConfigurableJoint)unityJoint;

            joint.axis = GetAxisData(configurableJoint.axis);
            joint.dynamics = new Joint.Dynamics(configurableJoint.xDrive.positionDamper, configurableJoint.xDrive.positionSpring);
            joint.limit = ExportLimitData();
#endif
            return joint;
        }

        public override bool AreLimitsCorrect()
        {
#if UNITY_2020_1_OR_NEWER 
            ArticulationBody joint = GetComponent<ArticulationBody>();
            return joint.linearLockX == ArticulationDofLock.LimitedMotion && joint.xDrive.lowerLimit < joint.xDrive.upperLimit;
#else
            PrismaticJointLimitsManager limits = GetComponent<PrismaticJointLimitsManager>();
            return limits != null && limits.PositionLimitMin < limits.PositionLimitMax;
#endif
        }

        protected override Joint.Limit ExportLimitData()
        {
#if UNITY_2020_1_OR_NEWER
            ArticulationDrive drive = GetComponent<ArticulationBody>().xDrive;
            return new Joint.Limit(drive.lowerLimit, drive.upperLimit, drive.forceLimit, unityJoint.maxLinearVelocity);
#else
            PrismaticJointLimitsManager prismaticLimits = GetComponent<PrismaticJointLimitsManager>();
            return new Joint.Limit(
                Math.Round(prismaticLimits.PositionLimitMin, RoundDigits),
                Math.Round(prismaticLimits.PositionLimitMax, RoundDigits),
                EffortLimit,
                VelocityLimit);
#endif
        }

#endregion
    }
}

