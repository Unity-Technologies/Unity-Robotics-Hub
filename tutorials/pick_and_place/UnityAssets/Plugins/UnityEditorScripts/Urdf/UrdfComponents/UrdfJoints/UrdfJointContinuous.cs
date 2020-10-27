   


using UnityEngine;

namespace RosSharp.Urdf
{
    public class UrdfJointContinuous : UrdfJoint
    {
        public override JointTypes JointType => JointTypes.Continuous;

        public static UrdfJoint Create(GameObject linkObject)
        {
            UrdfJointContinuous urdfJoint = linkObject.AddComponent<UrdfJointContinuous>();
#if UNITY_2020_1_OR_NEWER
            urdfJoint.unityJoint = linkObject.GetComponent<ArticulationBody>(); 
            urdfJoint.unityJoint.jointType = ArticulationJointType.RevoluteJoint;
#else
            urdfJoint.unityJoint = linkObject.AddComponent<HingeJoint>();
            urdfJoint.unityJoint.autoConfigureConnectedAnchor = true;
#endif
            return urdfJoint;
        }

#region Runtime
        /// <summary>
        /// Returns the current position of the joint in radians
        /// </summary>
        /// <returns>floating point number for joint position in radians</returns>
        public override float GetPosition() // Check Units
        {

#if UNITY_2020_1_OR_NEWER
            return unityJoint.jointPosition[0];
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
                return unityJoint.velocity[0];
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
            drive.target += deltaState;
            unityJoint.xDrive = drive;
#else
            Quaternion rot = Quaternion.AngleAxis(-deltaState * Mathf.Rad2Deg, unityJoint.axis);
            transform.rotation = transform.rotation * rot;
#endif
        }

        #endregion

        protected override void ImportJointData(Joint joint)
        {

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
        }

        protected override Joint ExportSpecificJointData(Joint joint)
        {
#if UNITY_2020_1_OR_NEWER
            joint.axis = GetAxisData(axisofMotion);
            joint.dynamics = new Joint.Dynamics(unityJoint.angularDamping, unityJoint.jointFriction);
            joint.limit = ExportLimitData();
#else

            joint.axis = GetAxisData(unityJoint.axis);
            joint.dynamics = new Joint.Dynamics(
                ((HingeJoint)unityJoint).spring.damper, 
                ((HingeJoint)unityJoint).spring.spring);
#endif
            return joint;
        }


        /// <summary>
        /// Reads axis joint information and rotation to the articulation body to produce the required motion
        /// </summary>
        /// <param name="joint">Structure containing joint information</param>
        protected override void AdjustMovement(Joint joint) 
        {
            axisofMotion = joint.axis.xyz.ToVector3();
            unityJoint.linearLockX = ArticulationDofLock.LockedMotion;
            unityJoint.linearLockY = ArticulationDofLock.LockedMotion;
            unityJoint.linearLockZ = ArticulationDofLock.LockedMotion;
            unityJoint.twistLock = ArticulationDofLock.FreeMotion;

            Vector3 axisofMotionUnity = axisofMotion.Ros2Unity();
            Quaternion motion = new Quaternion();
            motion.SetFromToRotation(new Vector3(1, 0, 0), -1 * axisofMotionUnity);
            unityJoint.anchorRotation = motion;

            if (joint.limit != null)
            {
                ArticulationDrive drive = unityJoint.xDrive;
                drive.forceLimit = (float)(joint.limit.effort);
                unityJoint.maxAngularVelocity = (float)(joint.limit.velocity);
                drive.damping = unityJoint.xDrive.damping;
                drive.stiffness = unityJoint.xDrive.stiffness;
                unityJoint.xDrive = drive;
            }
        }

    }
}

