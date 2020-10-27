   

using System;
using UnityEngine;

namespace RosSharp.Urdf
{
    #if UNITY_2020_1_OR_NEWER
        [RequireComponent(typeof(ArticulationBody))]
    #else
        [RequireComponent(typeof(Joint))]
    #endif
    public abstract class UrdfJoint : MonoBehaviour
    {
        public enum JointTypes
        {
            Fixed,
            Continuous,
            Revolute,
            Floating,
            Prismatic,
            Planar
        }

#if UNITY_2020_1_OR_NEWER
        protected UnityEngine.ArticulationBody unityJoint;
        protected Vector3 axisofMotion;
#else
        protected UnityEngine.Joint unityJoint;
#endif
        public string jointName;

        public abstract JointTypes JointType { get; } // Clear out syntax
        public bool IsRevoluteOrContinuous => JointType == JointTypes.Revolute || JointType == JointTypes.Revolute;
        public double EffortLimit = 1e3;
        public double VelocityLimit = 1e3;
        
        protected const int RoundDigits = 6;
        protected const float Tolerance = 0.0000001f;

        public static void Create(GameObject linkObject, JointTypes jointType, Joint joint = null)
        { 
            #if UNITY_2020_1_OR_NEWER 
             //ArticulationBody parentRigidbody = linkObject.transform.parent.gameObject.GetComponent<ArticulationBody>();
#else
            Rigidbody parentRigidbody = linkObject.transform.parent.gameObject.GetComponent<Rigidbody>();
            if (parentRigidbody == null) return;
#endif

            UrdfJoint urdfJoint = AddCorrectJointType(linkObject, jointType);

            if (joint != null)
            {
                urdfJoint.jointName = joint.name;
                urdfJoint.ImportJointData(joint);
            }
        }

        private static UrdfJoint AddCorrectJointType(GameObject linkObject, JointTypes jointType)
        {
            UrdfJoint urdfJoint = null;

            switch (jointType)
            {
                case JointTypes.Fixed:
                    urdfJoint = UrdfJointFixed.Create(linkObject);
                    break;
                case JointTypes.Continuous:
                    urdfJoint = UrdfJointContinuous.Create(linkObject);
                    break;
                case JointTypes.Revolute:
                    urdfJoint = UrdfJointRevolute.Create(linkObject);
                    break;
                case JointTypes.Floating:
                    urdfJoint = UrdfJointFloating.Create(linkObject);
                    break;
                case JointTypes.Prismatic:
                    urdfJoint = UrdfJointPrismatic.Create(linkObject);
                    break;
                case JointTypes.Planar:
                    urdfJoint = UrdfJointPlanar.Create(linkObject);
                    break;
            }


#if UNITY_2020_1_OR_NEWER
#else
            UnityEngine.Joint unityJoint = linkObject.GetComponent<UnityEngine.Joint>();
            unityJoint.connectedBody = linkObject.transform.parent.gameObject.GetComponent<Rigidbody>();
            unityJoint.autoConfigureConnectedAnchor = true;
#endif

            return urdfJoint;
        }

        /// <summary>
        /// Changes the type of the joint
        /// </summary>
        /// <param name="linkObject">Joint whose type is to be changed</param>
        /// <param name="newJointType">Type of the new joint</param>
        public static void ChangeJointType(GameObject linkObject, JointTypes newJointType)
        {
            linkObject.transform.DestroyImmediateIfExists<UrdfJoint>();
            linkObject.transform.DestroyImmediateIfExists<HingeJointLimitsManager>();
            linkObject.transform.DestroyImmediateIfExists<PrismaticJointLimitsManager>();
            #if UNITY_2020_1_OR_NEWER
                        linkObject.transform.DestroyImmediateIfExists<UnityEngine.ArticulationBody>();
            #else            
                        linkObject.transform.DestroyImmediateIfExists<UnityEngine.Joint>();
            #endif
                        AddCorrectJointType(linkObject, newJointType);
        }

        #region Runtime

        public void Start()
        {
            #if UNITY_2020_1_OR_NEWER
                        unityJoint = GetComponent<UnityEngine.ArticulationBody>();
            #else
                        unityJoint = GetComponent<UnityEngine.Joint>();
            #endif
        }

        public virtual float GetPosition()
        {
            return 0;
        }

        public virtual float GetVelocity()
        {
            return 0;
        }

        public virtual float GetEffort()
        {
            return 0;
        }

        public void UpdateJointState(float deltaState)
        {
            OnUpdateJointState(deltaState);
        } 
        protected virtual void OnUpdateJointState(float deltaState) { }

        #endregion

        #region Import Helpers
        
        public static JointTypes GetJointType(string jointType)
        {
            switch (jointType)
            {
                case "fixed":
                    return JointTypes.Fixed;
                case "continuous":
                    return JointTypes.Continuous;
                case "revolute":
                    return JointTypes.Revolute;
                case "floating":
                    return JointTypes.Floating;
                case "prismatic":
                    return JointTypes.Prismatic;
                case "planar":
                    return JointTypes.Planar;
                default:
                    return JointTypes.Fixed;
            }
        }

        protected virtual void ImportJointData(Joint joint) { }
        
        protected static Vector3 GetAxis(Joint.Axis axis) 
        {
            return axis.xyz.ToVector3().Ros2Unity();
        }

        protected static Vector3 GetDefaultAxis()
        {
            return new Vector3(-1, 0, 0);
        }

        protected static JointDrive GetJointDrive(Joint.Dynamics dynamics)
        {
            return new JointDrive
            {
                maximumForce = float.MaxValue,
                positionDamper = (float)dynamics.damping,
                positionSpring = (float)dynamics.friction
            };
        }

        protected static JointSpring GetJointSpring(Joint.Dynamics dynamics)
        {
            return new JointSpring
            {
                damper = (float)dynamics.damping,
                spring = (float)dynamics.friction,
                targetPosition = 0
            };
        }

        protected static SoftJointLimit GetLinearLimit(Joint.Limit limit)
        {
            return new SoftJointLimit { limit = (float)limit.upper };
        }

        protected virtual void AdjustMovement(Joint joint) {}

        #endregion

        #region Export

        public Joint ExportJointData()
        {
            #if UNITY_2020_1_OR_NEWER
                        unityJoint = GetComponent<UnityEngine.ArticulationBody>();
            #else
                        unityJoint = GetComponent<UnityEngine.Joint>();
            #endif
            CheckForUrdfCompatibility();

            //Data common to all joints
            Joint joint = new Joint(
                jointName,
                JointType.ToString().ToLower(),
                gameObject.transform.parent.name,
                gameObject.name,
                UrdfOrigin.ExportOriginData(transform));

            joint.limit = ExportLimitData();
            return ExportSpecificJointData(joint);
        }

        public static Joint ExportDefaultJoint(Transform transform)
        {
            return new Joint(
                transform.parent.name + "_" + transform.name + "_joint",
                JointTypes.Fixed.ToString().ToLower(),
                transform.parent.name,
                transform.name,
                UrdfOrigin.ExportOriginData(transform));
        }

        #region ExportHelpers

        protected virtual Joint ExportSpecificJointData(Joint joint)
        {
            return joint;
        }

        protected virtual Joint.Limit ExportLimitData()
        {
            return null; // limits aren't used
        }

        public virtual bool AreLimitsCorrect()
        {
            return true; // limits aren't needed
        }

        protected virtual bool IsJointAxisDefined()
        {
#if UNITY_2020_1_OR_NEWER
            if (axisofMotion == null)
                return false;
            else
                return true;
#else
                        UnityEngine.Joint joint = GetComponent<UnityEngine.Joint>();
                        return !(Math.Abs(joint.axis.x) < Tolerance &&
                                 Math.Abs(joint.axis.y) < Tolerance &&
                                 Math.Abs(joint.axis.z) < Tolerance);
#endif
        }

        public void GenerateUniqueJointName()
        {
            jointName = transform.parent.name + "_" + transform.name + "_joint";
        }

        protected Joint.Axis GetAxisData(Vector3 axis)
        {
            double[] rosAxis = axis.ToRoundedDoubleArray();
            return new Joint.Axis(rosAxis);
        }
        
        private bool IsAnchorTransformed() // TODO : Check for tolerances before implementation
        {

            UnityEngine.Joint joint = GetComponent<UnityEngine.Joint>();

            return Math.Abs(joint.anchor.x) > Tolerance || 
                Math.Abs(joint.anchor.x) > Tolerance ||
                Math.Abs(joint.anchor.x) > Tolerance;
        }

        private void CheckForUrdfCompatibility()
        {
            if (!AreLimitsCorrect())
                Debug.LogWarning("Limits are not defined correctly for Joint " + jointName + " in Link " + name +
                                 ". This may cause problems when visualizing the robot in RVIZ or Gazebo.", 
                                 gameObject);
            if (!IsJointAxisDefined())
                Debug.LogWarning("Axis for joint " + jointName + " is undefined. Axis will not be written to URDF, " +
                                 "and the default axis will be used instead.", 
                                 gameObject);
#if UNITY_2020_1_OR_NEWER

#else
            if (IsAnchorTransformed())
                Debug.LogWarning("The anchor position defined in the joint connected to " + name + " will be" +
                                 " ignored in URDF. Instead of modifying anchor, change the position of the link.", 
                                 gameObject);
#endif

        }

        #endregion

        #endregion
    }
}

