   

using System;
using UnityEngine;

namespace RosSharp.Urdf
{
    public class UrdfJointRevolute : UrdfJoint
    {
        public override JointTypes JointType => JointTypes.Revolute;
        
        public static UrdfJoint Create(GameObject linkObject)
        {
            UrdfJointRevolute urdfJoint = linkObject.AddComponent<UrdfJointRevolute>();
            #if UNITY_2020_1_OR_NEWER
            urdfJoint.unityJoint = linkObject.GetComponent<ArticulationBody>(); 
            urdfJoint.unityJoint.jointType = ArticulationJointType.RevoluteJoint;

#else
                        urdfJoint.unityJoint = linkObject.AddComponent<HingeJoint>();
                        urdfJoint.unityJoint.autoConfigureConnectedAnchor = true;
                        ((HingeJoint)urdfJoint.unityJoint).useLimits = true;
                        linkObject.AddComponent<HingeJointLimitsManager>();
#endif

            return urdfJoint;
        }

        #region Runtime

        /// <summary>
        /// Returns the current position of the joint in radians
        /// </summary>
        /// <returns>floating point number for joint position in radians</returns>
        public override float GetPosition()
        {
            #if UNITY_2020_1_OR_NEWER
            return ((ArticulationBody)unityJoint).jointPosition[0] * Mathf.Deg2Rad;
#else
                        return -((HingeJoint)unityJoint).angle * Mathf.Deg2Rad;
#endif
        }

        /// <summary>
        /// Returns the current velocity of joint in radians per second
        /// </summary>
        /// <returns>floating point for joint velocity in radians per second</returns>
        public override float GetVelocity()
        {
            #if UNITY_2020_1_OR_NEWER
                return ((ArticulationBody)unityJoint).jointVelocity[0] * Mathf.Deg2Rad;
            #else
                        return -((HingeJoint)unityJoint).velocity * Mathf.Deg2Rad;
            #endif
        }

        /// <summary>
        /// Returns current joint torque in Nm
        /// </summary>
        /// <returns>floating point in Nm</returns>
        public override float GetEffort()
        {
            #if UNITY_2020_1_OR_NEWER
                return unityJoint.jointForce[0];
            #else
                        return -((HingeJoint)unityJoint).motor.force;
            #endif
        }


        /// <summary>
        /// Rotates the joint by deltaState radians 
        /// </summary>
        /// <param name="deltaState">amount in radians by which joint needs to be rotated</param>
        protected override void OnUpdateJointState(float deltaState)
        {
#if UNITY_2020_1_OR_NEWER
            ArticulationDrive drive = unityJoint.xDrive;
            drive.target += deltaState * Mathf.Rad2Deg;
            unityJoint.xDrive = drive;
#else
                        Quaternion rot = Quaternion.AngleAxis(-deltaState * Mathf.Rad2Deg, unityJoint.axis);
                        transform.rotation = transform.rotation * rot;
#endif
        }

#endregion

        protected override void ImportJointData(Joint joint)
        {
#if UNITY_2020_1_OR_NEWER
            AdjustMovement(joint);
            if (joint.dynamics != null)
            {
                unityJoint.angularDamping = (float)joint.dynamics.damping; 
                unityJoint.jointFriction = (float)joint.dynamics.friction;
            }
            else
            {
                unityJoint.angularDamping = 0;
                unityJoint.jointFriction = 0;
            }
#else
                        unityJoint.axis = (joint.axis != null) ? GetAxis(joint.axis) : GetDefaultAxis();

                        if (joint.dynamics != null)
                            ((HingeJoint)unityJoint).spring = GetJointSpring(joint.dynamics);

                        if (joint.limit != null)
                            GetComponent<HingeJointLimitsManager>().InitializeLimits(joint.limit, (HingeJoint)unityJoint);
#endif
        }

        protected override Joint ExportSpecificJointData(Joint joint)
        {
#if UNITY_2020_1_OR_NEWER
            joint.axis = GetAxisData(axisofMotion);
            joint.dynamics = new Joint.Dynamics(unityJoint.angularDamping, unityJoint.jointFriction);
            joint.limit = ExportLimitData();
#else
            joint.axis = GetAxisData(unityJoint.axis);
            joint.dynamics = new Joint.Dynamics(((HingeJoint)unityJoint).spring.damper, ((HingeJoint)unityJoint).spring.spring);

            joint.limit = ExportLimitData();
#endif

            return joint;
        }

        public override bool AreLimitsCorrect()
        {
#if UNITY_2020_1_OR_NEWER
            ArticulationBody drive = GetComponent<ArticulationBody>();
            return drive.linearLockX == ArticulationDofLock.LimitedMotion && drive.xDrive.lowerLimit < drive.xDrive.upperLimit;
#else
            HingeJointLimitsManager limits = GetComponent<HingeJointLimitsManager>();
            return limits != null && limits.LargeAngleLimitMin < limits.LargeAngleLimitMax;
#endif
        }

        protected override Joint.Limit ExportLimitData()
        {
#if UNITY_2020_1_OR_NEWER
            ArticulationDrive drive = unityJoint.xDrive;
            return new Joint.Limit(drive.lowerLimit * Mathf.Deg2Rad, drive.upperLimit * Mathf.Deg2Rad, drive.forceLimit, unityJoint.maxAngularVelocity);
#else
            HingeJointLimitsManager hingeJointLimits = GetComponent<HingeJointLimitsManager>();
            return new Joint.Limit(
                Math.Round(hingeJointLimits.LargeAngleLimitMin * Mathf.Deg2Rad, RoundDigits),
                Math.Round(hingeJointLimits.LargeAngleLimitMax * Mathf.Deg2Rad, RoundDigits),
                EffortLimit,
                VelocityLimit);
#endif
        }

        /// <summary>
        /// Reads axis joint information and rotation to the articulation body to produce the required motion
        /// </summary>
        /// <param name="joint">Structure containing joint information</param>
        protected override void AdjustMovement(Joint joint) 
        {
            axisofMotion = (joint.axis != null && joint.axis.xyz != null) ? joint.axis.xyz.ToVector3() : new Vector3(1, 0, 0);
            unityJoint.linearLockX = ArticulationDofLock.LimitedMotion;
            unityJoint.linearLockY = ArticulationDofLock.LockedMotion;
            unityJoint.linearLockZ = ArticulationDofLock.LockedMotion;
            unityJoint.twistLock = ArticulationDofLock.LimitedMotion;

            Vector3 axisofMotionUnity = axisofMotion.Ros2Unity();
            Quaternion Motion = new Quaternion();
            Motion.SetFromToRotation(new Vector3(1, 0, 0), -1 * axisofMotionUnity);
            unityJoint.anchorRotation = Motion;

            if (joint.limit != null)
            {
                ArticulationDrive drive = unityJoint.xDrive;
                drive.upperLimit = (float)(joint.limit.upper * Mathf.Rad2Deg);
                drive.lowerLimit = (float)(joint.limit.lower * Mathf.Rad2Deg);
                drive.forceLimit = (float)(joint.limit.effort);
                unityJoint.maxAngularVelocity = (float)(joint.limit.velocity); 
                drive.damping = unityJoint.xDrive.damping;
                drive.stiffness = unityJoint.xDrive.stiffness;
                unityJoint.xDrive = drive;
            }
        }
    }
}

