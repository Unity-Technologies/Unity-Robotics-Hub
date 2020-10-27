  

using System;
using UnityEngine;

namespace RosSharp.Urdf
{
#if UNITY_2020_1_OR_NEWER
    [RequireComponent(typeof(ArticulationBody))]
#else
    [RequireComponent(typeof(Rigidbody))]
#endif
    public class UrdfInertial : MonoBehaviour
    {
        public bool displayInertiaGizmo;

        public bool useUrdfData;
        public Vector3 centerOfMass;
        public Vector3 inertiaTensor;
        public Quaternion inertiaTensorRotation;
        public Quaternion inertialAxisRotation;

        private const int RoundDigits = 10;
        private const float MinInertia = 1e-6f;

        public static void Create(GameObject linkObject, Link.Inertial inertial = null)
        {
            UrdfInertial urdfInertial = linkObject.AddComponent<UrdfInertial>();

#if UNITY_2020_1_OR_NEWER
            ArticulationBody robotLink = urdfInertial.GetComponent<ArticulationBody>();
#else
            Rigidbody robotLink = urdfInertial.GetComponent<Rigidbody>();
#endif
            if (inertial != null)
            {
                robotLink.mass = (float)inertial.mass;
                if (inertial.origin != null) {
                    
                    robotLink.centerOfMass = UrdfOrigin.GetPositionFromUrdf(inertial.origin);
                }
                else
                {
                    robotLink.centerOfMass = Vector3.zero;
                }
                urdfInertial.ImportInertiaData(inertial);
                 
                urdfInertial.useUrdfData = true;
            }

            urdfInertial.displayInertiaGizmo = false;
        }

#region Runtime

        private void Start()
        {

            UpdateLinkData();

        }

        public void UpdateLinkData()
        {

#if UNITY_2020_1_OR_NEWER
            ArticulationBody robotLink = GetComponent<ArticulationBody>();

#else
              Rigidbody robotLink = GetComponent<Rigidbody>();  
#endif

            if (useUrdfData)
            {
                robotLink.centerOfMass = centerOfMass;
                robotLink.inertiaTensor = inertiaTensor;
                robotLink.inertiaTensorRotation = inertiaTensorRotation * inertialAxisRotation;
            }
            else
            {
                robotLink.ResetCenterOfMass();
                robotLink.ResetInertiaTensor();
            }
        }

        private void OnDrawGizmosSelected()
        {
            if (displayInertiaGizmo)
            {
                #if UNITY_2020_1_OR_NEWER
                    Debug.Log("'ArticulationBody' does not contain a definition for 'inertiaTensorRotation' and no accessible extension method 'inertiaTensorRotation'");
               /* Gizmos.color = Color.blue;
                Gizmos.DrawRay(transform.position, GetComponent<ArticulationBody>().inertiaTensorRotation * Vector3.forward * GetComponent<ArticulationBody>().inertiaTensor.z);
                Gizmos.color = Color.green;
                Gizmos.DrawRay(transform.position, GetComponent<ArticulationBody>().inertiaTensorRotation * Vector3.up * GetComponent<ArticulationBody>().inertiaTensor.y);
                Gizmos.color = Color.red;
                Gizmos.DrawRay(transform.position, GetComponent<ArticulationBody>().inertiaTensorRotation * Vector3.right * GetComponent<ArticulationBody>().inertiaTensor.x);*/
                #else
                Gizmos.color = Color.blue;
                Gizmos.DrawRay(transform.position, GetComponent<Rigidbody>().inertiaTensorRotation * Vector3.forward * GetComponent<Rigidbody>().inertiaTensor.z);
                Gizmos.color = Color.green;
                Gizmos.DrawRay(transform.position, GetComponent<Rigidbody>().inertiaTensorRotation * Vector3.up * GetComponent<Rigidbody>().inertiaTensor.y);
                Gizmos.color = Color.red;
                Gizmos.DrawRay(transform.position, GetComponent<Rigidbody>().inertiaTensorRotation * Vector3.right * GetComponent<Rigidbody>().inertiaTensor.x);
                #endif
            }
        }

#endregion

#region Import

        private void ImportInertiaData(Link.Inertial inertial)
        {
            Vector3 eigenvalues;
            Vector3[] eigenvectors;
            Matrix3x3 rotationMatrix = ToMatrix3x3(inertial.inertia);
            rotationMatrix.DiagonalizeRealSymmetric(out eigenvalues, out eigenvectors);
#if UNITY_2020_1_OR_NEWER
            ArticulationBody robotLink = GetComponent<ArticulationBody>();

#else
            Rigidbody robotLink = GetComponent<Rigidbody>();
#endif

            Vector3 inertiaEulerAngles;

            if(inertial.origin != null)
            {
                inertiaEulerAngles = UrdfOrigin.GetRotationFromUrdf(inertial.origin);
            }
            else
            {
                inertiaEulerAngles = new Vector3(0, 0, 0);
            }

            this.inertialAxisRotation.eulerAngles = inertiaEulerAngles;
            

            robotLink.inertiaTensor = ToUnityInertiaTensor(FixMinInertia(eigenvalues));
            robotLink.inertiaTensorRotation = ToQuaternion(eigenvectors[0], eigenvectors[1], eigenvectors[2]).Ros2Unity() * this.inertialAxisRotation;

            this.centerOfMass = robotLink.centerOfMass;
            this.inertiaTensor = robotLink.inertiaTensor;
            this.inertiaTensorRotation = robotLink.inertiaTensorRotation;

        }

        private static Vector3 ToUnityInertiaTensor(Vector3 vector3)
        {
            return new Vector3(vector3.y, vector3.z, vector3.x);
        }

        private static Matrix3x3 ToMatrix3x3(Link.Inertial.Inertia inertia)
        {
            return new Matrix3x3(
                new[] { (float)inertia.ixx, (float)inertia.ixy, (float)inertia.ixz,
                                             (float)inertia.iyy, (float)inertia.iyz,
                                                                 (float)inertia.izz });
        }

        private static Vector3 FixMinInertia(Vector3 vector3)
        {
            for (int i = 0; i < 3; i++)
            {
                if (vector3[i] < MinInertia)
                    vector3[i] = MinInertia;
            }
            return vector3;
        }

        private static Quaternion ToQuaternion(Vector3 eigenvector0, Vector3 eigenvector1, Vector3 eigenvector2)
        {
            //From http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
            float tr = eigenvector0[0] + eigenvector1[1] + eigenvector2[2];
            float qw, qx, qy, qz;
            if (tr > 0)
            {
                float s = Mathf.Sqrt(tr + 1.0f) * 2f; // S=4*qw 
                qw = 0.25f * s;
                qx = (eigenvector1[2] - eigenvector2[1]) / s;
                qy = (eigenvector2[0] - eigenvector0[2]) / s;
                qz = (eigenvector0[1] - eigenvector1[0]) / s;
            }
            else if ((eigenvector0[0] > eigenvector1[1]) & (eigenvector0[0] > eigenvector2[2]))
            {
                float s = Mathf.Sqrt(1.0f + eigenvector0[0] - eigenvector1[1] - eigenvector2[2]) * 2; // S=4*qx 
                qw = (eigenvector1[2] - eigenvector2[1]) / s;
                qx = 0.25f * s;
                qy = (eigenvector1[0] + eigenvector0[1]) / s;
                qz = (eigenvector2[0] + eigenvector0[2]) / s;
            }
            else if (eigenvector1[1] > eigenvector2[2])
            {
                float s = Mathf.Sqrt(1.0f + eigenvector1[1] - eigenvector0[0] - eigenvector2[2]) * 2; // S=4*qy
                qw = (eigenvector2[0] - eigenvector0[2]) / s;
                qx = (eigenvector1[0] + eigenvector0[1]) / s;
                qy = 0.25f * s;
                qz = (eigenvector2[1] + eigenvector1[2]) / s;
            }
            else
            {
                float s = Mathf.Sqrt(1.0f + eigenvector2[2] - eigenvector0[0] - eigenvector1[1]) * 2; // S=4*qz
                qw = (eigenvector0[1] - eigenvector1[0]) / s;
                qx = (eigenvector2[0] + eigenvector0[2]) / s;
                qy = (eigenvector2[1] + eigenvector1[2]) / s;
                qz = 0.25f * s;
            }
            return new Quaternion(qx, qy, qz, qw);
        }

#endregion

#region Export
        public Link.Inertial ExportInertialData() 
        {
#if UNITY_2020_1_OR_NEWER
            ArticulationBody robotLink = GetComponent<ArticulationBody>();

#else
            Rigidbody robotLink = GetComponent<Rigidbody>();
#endif

            if (robotLink == null)
                return null;

            UpdateLinkData();
            Vector3 originAngles = inertialAxisRotation.eulerAngles;
            Origin inertialOrigin = new Origin(robotLink.centerOfMass.Unity2Ros().ToRoundedDoubleArray(), new double[] { (double)originAngles.x, (double)originAngles.y, (double)originAngles.z });
            Link.Inertial.Inertia inertia = ExportInertiaData();

            return new Link.Inertial(Math.Round(robotLink.mass, RoundDigits), inertialOrigin, inertia);
        }

        private Link.Inertial.Inertia ExportInertiaData()
        {
#if UNITY_2020_1_OR_NEWER
            ArticulationBody robotLink = GetComponent<ArticulationBody>();

#else
            Rigidbody robotLink = GetComponent<Rigidbody>();
#endif
            Matrix3x3 lamdaMatrix = new Matrix3x3(new[] {
                robotLink.inertiaTensor[0],
                robotLink.inertiaTensor[1],
                robotLink.inertiaTensor[2] });

            Matrix3x3 qMatrix = Quaternion2Matrix(robotLink.inertiaTensorRotation * Quaternion.Inverse(inertialAxisRotation));

            Matrix3x3 qMatrixTransposed = qMatrix.Transpose();

            Matrix3x3 inertiaMatrix = qMatrix * lamdaMatrix * qMatrixTransposed;


            return ToRosCoordinates(ToInertia(inertiaMatrix));
        }



        private static Matrix3x3 Quaternion2Matrix(Quaternion quaternion)
        {
            Quaternion rosQuaternion = Quaternion.Normalize(quaternion);
            float qx = rosQuaternion.x;
            float qy = rosQuaternion.y;
            float qz = rosQuaternion.z;
            float qw = rosQuaternion.w;

            //From http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
            return new Matrix3x3(new float[] {
                1 - (2 * qy * qy) - (2 * qz * qz),
                (2 * qx * qy) - (2 * qz * qw),
                (2 * qx * qz) + (2 * qy * qw),

                (2 * qx * qy) + (2 * qz * qw),
                1 - (2 * qx * qx) - (2 * qz * qz),
                (2 * qy * qz) - (2 * qx * qw),

                (2 * qx * qz) - (2 * qy * qw),
                (2 * qy * qz) + (2 * qx * qw),
                1 - (2 * qx * qx) - (2 * qy * qy)});
        }

        private static Link.Inertial.Inertia ToInertia(Matrix3x3 matrix)
        {
            return new Link.Inertial.Inertia(matrix[0][0], matrix[0][1], matrix[0][2],
                matrix[1][1], matrix[1][2],
                matrix[2][2]);
        }

        private static Link.Inertial.Inertia ToRosCoordinates(Link.Inertial.Inertia unityInertia)
        {
            return new Link.Inertial.Inertia(0, 0, 0, 0, 0, 0)
            {
                ixx = unityInertia.izz,
                iyy = unityInertia.ixx,
                izz = unityInertia.iyy,

                ixy = -unityInertia.ixz,
                ixz = unityInertia.iyz,
                iyz = -unityInertia.ixy
            };
        }
#endregion
    }
}

