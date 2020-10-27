  

using UnityEngine;

namespace RosSharp.Urdf
{
    public static class UrdfOrigin
    {
        #region Import

        public static void ImportOriginData(Transform transform, Origin origin)
        {
            if (origin != null)
            {
                transform.Translate(GetPositionFromUrdf(origin));
                transform.Rotate(GetRotationFromUrdf(origin));
            }
        }
        public static Vector3 GetPositionFromUrdf(Origin origin)
        {
            if (origin.Xyz != null)
                return origin.Xyz.ToVector3().Ros2Unity();
            
            return Vector3.zero;
        }
        public static Vector3 GetRotationFromUrdf(Origin origin)
        {
            if (origin.Rpy != null)
                return new Vector3(
                    (float)+origin.Rpy[1] * Mathf.Rad2Deg,
                    (float)-origin.Rpy[2] * Mathf.Rad2Deg,
                    (float)-origin.Rpy[0] * Mathf.Rad2Deg);

            return Vector3.zero;
        }

        #endregion

        #region Export

        public static Origin ExportOriginData(Transform transform)
        {
            double[] xyz = ExportXyzData(transform);
            double[] rpy = ExportRpyData(transform);

            if (xyz != null || rpy != null)
                return new Origin(xyz, rpy);

            return null;
        }

        private static double[] ExportXyzData(Transform transform)
        {
            Vector3 xyzVector = transform.localPosition.Unity2Ros();
            return xyzVector == Vector3.zero ? null : xyzVector.ToRoundedDoubleArray();
        }

        private static double[] ExportRpyData(Transform transform)
        {
            Vector3 rpyVector = new Vector3(
                -transform.localEulerAngles.z * Mathf.Deg2Rad,
                transform.localEulerAngles.x * Mathf.Deg2Rad,
                -transform.localEulerAngles.y * Mathf.Deg2Rad);
            return rpyVector == Vector3.zero ? null : rpyVector.ToRoundedDoubleArray();
        }

        #endregion
    }
}
