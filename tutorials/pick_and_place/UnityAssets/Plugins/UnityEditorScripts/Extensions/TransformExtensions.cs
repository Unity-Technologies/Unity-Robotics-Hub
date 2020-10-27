 

using System;
using System.IO;
using UnityEngine;
using Object = UnityEngine.Object;

namespace RosSharp
{
    public static class TransformExtensions
    {
        private const int RoundDigits = 6;

        public static void DestroyImmediateIfExists<T>(this Transform transform) where T : Component
        {
            T component = transform.GetComponent<T>();
            if (component != null)
                Object.DestroyImmediate(component);
        }

        public static T AddComponentIfNotExists<T>(this Transform transform) where T : Component
        {
            T component = transform.GetComponent<T>();
            if (component == null)
                component = transform.gameObject.AddComponent<T>();
            return component;
        }

        public static void SetParentAndAlign(this Transform transform, Transform parent, bool keepLocalTransform = true)
        {
            Vector3 localPosition = transform.localPosition;
            Quaternion localRotation = transform.localRotation;
            transform.parent = parent;
            if (keepLocalTransform)
            {
                transform.position = transform.parent.position + localPosition;
                transform.rotation = transform.parent.rotation * localRotation;
            }
            else
            {
                transform.localPosition = Vector3.zero;
                transform.localRotation = Quaternion.identity;
            }
        }

        public static bool HasExactlyOneChild(this Transform transform)
        {
            return transform.childCount == 1;
        }

        public static void MoveChildTransformToParent(this Transform parent, bool transferRotation = true)
        {
            //Detach child in order to get a transform indenpendent from parent
            Transform childTransform = parent.GetChild(0);
            parent.DetachChildren();

            //Copy transform from child to parent
            parent.position = childTransform.position;
            parent.localScale = childTransform.localScale;

            if (transferRotation)
            {
                parent.rotation = childTransform.rotation;
                childTransform.localRotation = Quaternion.identity;
            }

            childTransform.parent = parent;

            childTransform.localPosition = Vector3.zero;
            childTransform.localScale = Vector3.one;
        }

        public static double[] ToRosRPY(this Vector3 transform)
        {
            Vector3 rpyVector = new Vector3(
                -transform.z * Mathf.Deg2Rad,
                transform.x * Mathf.Deg2Rad,
                -transform.y * Mathf.Deg2Rad);
            return rpyVector == Vector3.zero ? null : rpyVector.ToRoundedDoubleArray();
        }

        public static Vector3 Ros2Unity(this Vector3 vector3)
        {
            return new Vector3(-vector3.y, vector3.z, vector3.x);
        }

        public static Vector3 Unity2Ros(this Vector3 vector3)
        {
            return new Vector3(vector3.z, -vector3.x, vector3.y);
        }

        public static Vector3 Ros2UnityScale(this Vector3 vector3)
        {
            return new Vector3(vector3.y, vector3.z, vector3.x);
        }

        public static Vector3 Unity2RosScale(this Vector3 vector3)
        {
            return new Vector3(vector3.z, vector3.x, vector3.y);
        }

        public static Quaternion Ros2Unity(this Quaternion quaternion)
        {
            return new Quaternion(quaternion.y, -quaternion.z, -quaternion.x, quaternion.w);
        }

        public static Quaternion Unity2Ros(this Quaternion quaternion)
        {
            return new Quaternion(-quaternion.z, quaternion.x, -quaternion.y, quaternion.w);
        }

        public static double[] ToRoundedDoubleArray(this Vector3 vector3)
        {
            double[] arr = new double[3];
            for (int i = 0; i < 3; i++)
                arr[i] = Math.Round(vector3[i], RoundDigits);

            return arr;
        }

        public static Vector3 ToVector3(this double[] array)
        {
            return new Vector3((float)array[0], (float)array[1], (float)array[2]);
        }

        public static string SetSeparatorChar(this string path)
        {
            return path.Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
        }

        /// <summary>
        /// This function is written to compare two double arrays.
        /// It returns true if each number respective number in the array is equal to within a delta amount to each other.
        /// Vector3 == Vector3 only allows for a difference of 1e-5
        /// </summary>
        /// <param name="array">First double array</param>
        /// <param name="array2">Second Double array</param>
        /// <param name="delta">Allowed delta between numbers allowed</param>
        /// <returns></returns>
        public static bool DoubleDeltaCompare(this double[] array, double[] array2, double delta)
        {
            if (array.Length != array2.Length)
                return false;
            for (int i = 0; i < array.Length ; i++)
            {
                if ((array[i] >= array2[i] - delta) && (array[i] <= array2[i] + delta))
                    continue;
                else
                    return false;
            }
            return true;
        }

        /// <summary>
        /// This function is written to compare two Vector3.
        /// It returns true if each number respective number in the array is equal to within a delta amount to each other.
        /// </summary>
        /// <param name="array">First Vector3</param>
        /// <param name="array2">Second Vector3</param>
        /// <param name="delta">Allowed delta between numbers allowed</param>
        /// <returns></returns>
        public static bool VectorEqualDelta(this Vector3 source, Vector3 exported, double delta)
        {
            
            return Vector3.SqrMagnitude(source - exported) < delta;
        }

        public static bool EqualsDelta(this double first, double second, double delta)
        {
            if (Math.Abs(first - second) <= Math.Abs(delta * first))
                return true;
            else
                return false;
        }

        /// <summary>
        /// Implments element-wise subtraction between two matrices
        /// </summary>
        /// <param name="first"></param>
        /// <param name="second"></param>
        /// <returns></returns>
        public static Matrix4x4 Subtract(this Matrix4x4 first, Matrix4x4 second)
        {
            Matrix4x4 result = new Matrix4x4();
            for(int i = 0; i < 4; i++)
            {
                for(int j = 0; j < 4; j++)
                {
                    result[i, j] = first[i, j] - second[i, j];
                }
            }
            return result;
        }

        /// <summary>
        /// Divides each float number of a Matrix4x4 type with a float scalar.
        /// The function is added as it does not support element-wise division by float
        /// https://docs.unity3d.com/ScriptReference/Matrix4x4.html
        /// </summary>
        /// <param name="first">Matrix divisor</param>
        /// <param name="second">Float dividend</param>
        /// <returns></returns>
        public static Matrix4x4 FloatDivide(this Matrix4x4 first, float second)
        {
            Matrix4x4 result = new Matrix4x4();
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    result[i, j] = first[i, j] / second;
                }
            }
            return result;
        }

        /// <summary>
        /// Function overload of ToString method of Matrix3x3 to achieve unrounded print of the matrix
        /// </summary>
        /// <param name="first">Matrix to be printed</param>
        /// <returns></returns>
        public static string ToString(this Matrix4x4 first)
        {
            return ("Matrix : " + first[0, 0] + " " + first[0, 1] + " " + first[0, 2] + " " + first[0, 3] + " " +
                first[1, 0] + " " + first[1, 1] + " " + first[1, 2] + " " + first[1, 3] + " " +
                first[2, 0] + " " + first[2, 1] + " " + first[2, 2] + " " + first[2, 3] + " " +
                first[3, 0] + " " + first[3, 1] + " " + first[3, 2] + " " + first[3, 3]);
        }
    }
}
