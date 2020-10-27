/*
© Unity 3D, 2020
Author: Berkay Alp Cakal (berkay_alp.cakal.ct@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System.Collections.Generic;
using UnityEngine;

namespace RosSharp
{
    public class Matrix3x3
    {
        public float[][] elements;

        public Matrix3x3(float x = 0)
        {
            elements = new float[][] { new float[3], new float[3], new float[3] };
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    elements[i][j] = x;
        }

        public Matrix3x3(float[] elements)
        {
            if (elements.Length == 3) // elements = [xx, yy, zz] 
                this.elements = new float[][] { new float[] { elements[0], 0, 0 },
                                                new float[] { 0, elements[1], 0 },
                                                new float[] { 0, 0, elements[2] } };

            if (elements.Length == 6) // elements = [xx, xy, xz, yy, yz, zz] 
                this.elements = new float[][] { new float[] { elements[0], elements[1], elements[2] },
                                                new float[] { elements[1], elements[3], elements[4] },
                                                new float[] { elements[2], elements[4], elements[5] } };

            if (elements.Length == 9) // elements = [xx, xy, xz, yx, yy, yz, zx, zy, zz]
                this.elements = new float[][] { new float[] { elements[0], elements[1], elements[2] },
                                                new float[] { elements[3], elements[4], elements[5] },
                                                new float[] { elements[6], elements[7], elements[8] } };
        }

        public Matrix3x3(float[][] elements)
        {
            this.elements = new float[][] { new float[3], new float[3], new float[3] };
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    this.elements[i][j] = elements[i][j];
        }

        public Matrix3x3(Vector3[] vectors)
        {
            elements = new float[][] { new float[] { vectors[0].x, vectors[0].y, vectors[0].z },
                                        new float[] { vectors[1].x, vectors[1].y, vectors[1].z },
                                        new float[] { vectors[2].x, vectors[2].y, vectors[2].z } };
        }

        public float[] this[int i]
        {
            get { return elements[i]; }
            set { elements[i] = value; }
        }

        public static Matrix3x3 operator +(Matrix3x3 A, Matrix3x3 B)
        {
            Matrix3x3 result = new Matrix3x3();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    result[i][j] = A[i][j] + B[i][j];
            return result;
        }

        public static Matrix3x3 operator +(Matrix3x3 A, float x)
        {
            Matrix3x3 result = new Matrix3x3();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    result[i][j] = A[i][j] + x;
            return result;
        }

        public static Matrix3x3 operator -(Matrix3x3 A, Matrix3x3 B)
        {
            Matrix3x3 result = new Matrix3x3();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    result[i][j] = A[i][j] - B[i][j];
            return result;
        }

        public static Matrix3x3 operator -(Matrix3x3 A, float x)
        {
            Matrix3x3 result = new Matrix3x3();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    result[i][j] = A[i][j] - x;
            return result;
        }

        public static Matrix3x3 operator *(Matrix3x3 A, float x)
        {
            Matrix3x3 result = new Matrix3x3();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    result[i][j] = A[i][j] * x;
            return result;
        }

        public static Matrix3x3 operator *(Matrix3x3 A, Matrix3x3 B)
        {
            Matrix3x3 result = new Matrix3x3();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    for (int k = 0; k < 3; k++)
                        result[i][j] += A[i][k] * B[k][j];
            return result;
        }

        public static Vector3 operator *(Matrix3x3 A, Vector3 B)
        {
            Vector3 row0 = new Vector3(A[0][0], A[0][1], A[0][2]);
            Vector3 row1 = new Vector3(A[1][0], A[1][1], A[1][2]);
            Vector3 row2 = new Vector3(A[2][0], A[2][1], A[2][2]);
            return new Vector3(Vector3.Dot(row0, B), Vector3.Dot(row1, B), Vector3.Dot(row2, B));
        }

        public float Determinant()
        {
            float result = 0.0f;
            for (int i = 0; i < 3; i++)
                result += (elements[0][i] * (elements[1][(i + 1) % 3] * elements[2][(i + 2) % 3] - elements[1][(i + 2) % 3] * elements[2][(i + 1) % 3]));
            return result;
        }

        public float Trace()
        {
            return (elements[0][0] + elements[1][1] + elements[2][2]);
        }

        public bool IsDiagonal()
        {
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    if (i != j && elements[i][j] != 0)
                        return false;
            return true;
        }

        public Matrix3x3 Transpose()
        {
            return new Matrix3x3(new float[]
                                { elements[0][0], elements[1][0], elements[2][0],
                                  elements[0][1], elements[1][1], elements[2][1],
                                  elements[0][2], elements[1][2], elements[2][2] });
        }

        public void DiagonalizeRealSymmetric(out Vector3 EigenvaluesOut, out Vector3[] EigenvectorsOut)
        {
            if (IsDiagonal())
            {
                
                EigenvaluesOut = new Vector3(elements[0][0], elements[1][1], elements[2][2]);
                EigenvectorsOut = new Vector3[] { new Vector3(1, 0, 0 ),
                                                  new Vector3(0, 1, 0 ),
                                                  new Vector3(0, 0, 1 ) };
                return;
            }
            EigenvaluesOut = Eigenvalues();
            EigenvectorsOut = Eigenvectors(new float[] { EigenvaluesOut[0], EigenvaluesOut[1], EigenvaluesOut[2] });

        }

        public Vector3 Eigenvalues()
        {
            /* Smith, Oliver K. (April 1961), "Eigenvalues of a symmetric 3 × 3 matrix."
             * Communications of the ACM, 4 (4): 168, doi:10.1145/355578.366316             */

            Matrix3x3 matrix3x3 = this;

            float traceA = matrix3x3.Trace();
            float q = traceA / 3;
            float p1 = matrix3x3[0][1] * matrix3x3[0][1] + matrix3x3[0][2] * matrix3x3[0][2] + matrix3x3[1][2] * matrix3x3[1][2];
            float p2 = (matrix3x3[0][0] - q) * (matrix3x3[0][0] - q) + (matrix3x3[1][1] - q) * (matrix3x3[1][1] - q) + (matrix3x3[2][2] - q) * (matrix3x3[2][2] - q) + 2 * p1;
            float p = Mathf.Sqrt(p2 / 6);

            Matrix3x3 B = (matrix3x3 * (1f / p)) + (new Matrix3x3(new float[] { -q / p, -q / p, -q / p }));
            float angle = Mathf.Clamp(B.Determinant() / 2f, -1, 1);
            float theta = Mathf.Acos(angle) / 3;
            
            Vector3 beta = new Vector3();
            Vector3 alpha = new Vector3();
            for (int k = 0; k < 3; k++)
            {
                beta[k] = 2f * Mathf.Cos(theta + (2 * Mathf.PI * k) / 3);
                alpha[k] = p * beta[k] + q;
            }

            return alpha;
        }

        private Vector3[] Eigenvectors(float[] eigenvalues_unsorted)
        {
            float[] eigenvalues = (float[])eigenvalues_unsorted.Clone();
            System.Array.Sort(eigenvalues);

            Vector3 eigenvector0 = GetEigenvector0(eigenvalues);
            Vector3 eigenvector1 = GetEigenvector1(eigenvalues, eigenvector0);
            Vector3 eigenvector2 = GetEigenvector2(eigenvalues, eigenvector0, eigenvector1);

            List<float> values = new List<float> { eigenvalues[0], eigenvalues[1], eigenvalues[2] };
            List<float[]> vectors = new List<float[]>(){
                    new float[] { eigenvector0[0], eigenvector0[1], eigenvector0[2] },
                    new float[] { eigenvector1[0], eigenvector1[1], eigenvector1[2] },
                    new float[] { eigenvector2[0], eigenvector2[1], eigenvector2[2] }
                    };
            List<float> values_unsorted = new List<float>();
            List<float[]> vectors_unsorted = new List<float[]>();

            for (int i = 0; i < 3; i++)
            {
                int idx = values.IndexOf(eigenvalues_unsorted[i]);
                values_unsorted.Add(values[idx]);
                vectors_unsorted.Add(vectors[idx]);
                values.RemoveAt(idx);
                vectors.RemoveAt(idx);
            }

            return new Vector3[] { new Vector3(vectors_unsorted[0][0], vectors_unsorted[0][1], vectors_unsorted[0][2]),
                                   new Vector3(vectors_unsorted[1][0], vectors_unsorted[1][1], vectors_unsorted[1][2]),
                                   new Vector3(vectors_unsorted[2][0], vectors_unsorted[2][1], vectors_unsorted[2][2]) };
        }

        private Vector3 GetEigenvector0(float[] eigenvalues)
        {
            if (IsTwoEigenvaluesEqual(eigenvalues))
                return new Vector3(1, 0, 0);

            Vector3 eigenvector0;

            Vector3 row0 = new Vector3(elements[0][0] - eigenvalues[0], elements[0][1], elements[0][2]);
            Vector3 row1 = new Vector3(elements[1][0], elements[1][1] - eigenvalues[0], elements[1][2]);
            Vector3 row2 = new Vector3(elements[2][0], elements[2][1], elements[2][2] - eigenvalues[0]);

            Vector3 cross_r0r1 = Vector3.Cross(row0, row1);
            Vector3 cross_r0r2 = Vector3.Cross(row0, row2);
            Vector3 cross_r1r2 = Vector3.Cross(row1, row2);

            float dot0 = Vector3.Dot(cross_r0r1, cross_r0r1);
            float dot1 = Vector3.Dot(cross_r0r2, cross_r0r2);
            float dot2 = Vector3.Dot(cross_r1r2, cross_r1r2);
            float dmax = dot0;
            int imax = 0;

            if (dot1 > dmax) { dmax = dot1; imax = 1; }
            if (dot2 > dmax) { imax = 2; }

            if (imax == 0)
                eigenvector0 = new Vector3(cross_r0r1[0] / Mathf.Sqrt(dot0), cross_r0r1[1] / Mathf.Sqrt(dot0), cross_r0r1[2] / Mathf.Sqrt(dot0));
            else if (imax == 1)
                eigenvector0 = new Vector3(cross_r0r2[0] / Mathf.Sqrt(dot1), cross_r0r2[1] / Mathf.Sqrt(dot1), cross_r0r2[2] / Mathf.Sqrt(dot1));
            else
                eigenvector0 = new Vector3(cross_r1r2[0] / Mathf.Sqrt(dot2), cross_r1r2[1] / Mathf.Sqrt(dot2), cross_r1r2[2] / Mathf.Sqrt(dot2));

            return eigenvector0;
        }

        private Vector3 GetEigenvector1(float[] eigenvalues, Vector3 eigenvector0)
        {
            Matrix3x3 inertiaTensor = this;

            if (IsTwoEigenvaluesEqual(eigenvalues))
                return new Vector3(0, 1, 0);

            Vector3 eigenvector1 = Vector3.zero;
            Vector3[] UV = CalculateOrthogonalComplement(eigenvector0);
            Vector3 AU = inertiaTensor * UV[0];
            Vector3 AV = inertiaTensor * UV[1];

            float m00 = Vector3.Dot(UV[0], AU) - eigenvalues[1];
            float m01 = Vector3.Dot(UV[0], AV);
            float m11 = Vector3.Dot(UV[1], AV) - eigenvalues[1];

            if (Mathf.Abs(m00) > Mathf.Abs(m11))
            {
                float maxAbscomp = Mathf.Max(Mathf.Abs(m00), Mathf.Abs(m01));
                if (maxAbscomp > 0)
                {
                    if (Mathf.Abs(m00) >= Mathf.Abs(m01)) { m01 /= m00; m00 = 1 / Mathf.Sqrt(1 + m01 * m01); m01 *= m00; }
                    else { m00 /= m01; m01 = 1 / Mathf.Sqrt(1 + m00 * m00); m00 *= m01; }
                    eigenvector1 = (UV[0] * m01) + (UV[1] * -m00);
                }
                else
                    eigenvector1 = UV[0];
            }

            else
            {
                float maxAbscomp = Mathf.Max(Mathf.Abs(m11), Mathf.Abs(m01));
                if (maxAbscomp > 0)
                {
                    if (Mathf.Abs(m11) >= Mathf.Abs(m01)) { m01 /= m11; m11 = 1 / Mathf.Sqrt(1 + m01 * m01); m01 *= m11; }
                    else { m11 /= m01; m01 = 1 / Mathf.Sqrt(1 + m11 * m11); m11 *= m01; }
                    eigenvector1 = (UV[0] * m11) + (UV[1] * -m00);
                }
                else
                    eigenvector1 = UV[0];
            }

            return eigenvector1;
        }

        private Vector3 GetEigenvector2(float[] eigenvalues, Vector3 eigenvector0, Vector3 eigenvector1)
        {
            if (IsTwoEigenvaluesEqual(eigenvalues))
                return new Vector3(0, 0, 1);

            return Vector3.Cross(eigenvector0, eigenvector1);
        }

        private Vector3[] CalculateOrthogonalComplement(Vector3 W)
        {
            Vector3 U; Vector3 V;
            float invLength = 0;

            if (Mathf.Abs(W[0]) > Mathf.Abs(W[1]))
            {
                invLength = 1 / Mathf.Sqrt(W[0] * W[0] + W[2] * W[2]);
                U = new Vector3(-W[2] * invLength, 0, W[0] * invLength);
            }
            else
            {
                invLength = 1 / Mathf.Sqrt(W[1] * W[1] + W[2] * W[2]);
                U = new Vector3(0, W[2] * invLength, -W[1] * invLength);
            }

            V = Vector3.Cross(W, U);

            return new Vector3[] { U, V };
        }

        private bool IsTwoEigenvaluesEqual(float[] eigenvalues)
        {
            return (eigenvalues[0] == eigenvalues[1] || eigenvalues[1] == eigenvalues[2] || eigenvalues[0] == eigenvalues[2]);
        }

    }
}