 

using UnityEngine;

namespace RosSharp
{
    public class PrismaticJointLimitsManager : MonoBehaviour
    {
        public float PositionLimitMin;
        public float PositionLimitMax;

        public float Tolerance = 0.01f;

        private ConfigurableJoint configurableJoint;
        private float referencePosition;

        private void Awake()
        {
            configurableJoint = GetComponent<ConfigurableJoint>();
            referencePosition = Vector3.Dot(transform.localPosition, configurableJoint.axis);
        }

        private void FixedUpdate()
        {
            ApplyLimits();
        }

        private void OnValidate()
        {
            if (PositionLimitMax < PositionLimitMin)
                PositionLimitMax = PositionLimitMin;
        }

        private void ApplyLimits()
        {
            float position = Vector3.Dot(transform.localPosition, configurableJoint.axis) - referencePosition;
            
            if (position - PositionLimitMin < Tolerance)
            {
                configurableJoint.xMotion = ConfigurableJointMotion.Limited;
                configurableJoint.linearLimit = UpdateLimit(configurableJoint.linearLimit, -PositionLimitMin);
            }
            else if (PositionLimitMax - position < Tolerance)
            {
                configurableJoint.xMotion = ConfigurableJointMotion.Limited;
                configurableJoint.linearLimit = UpdateLimit(configurableJoint.linearLimit, PositionLimitMax);
            }
            else
            {
                configurableJoint.xMotion = ConfigurableJointMotion.Free;
            }
        }

        private static SoftJointLimit UpdateLimit(SoftJointLimit softJointLimit, float limit)
        {
            softJointLimit.limit = limit;
            return softJointLimit;
        }

        public void InitializeLimits(Urdf.Joint.Limit limit)
        {
            PositionLimitMax = (float)limit.upper;
            PositionLimitMin = (float)limit.lower;
        }

    }
}
