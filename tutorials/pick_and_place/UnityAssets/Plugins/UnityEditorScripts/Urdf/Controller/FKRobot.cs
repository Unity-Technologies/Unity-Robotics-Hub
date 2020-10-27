using UnityEngine;
using System.Collections.Generic;
using System;

namespace RosSharp.Control{
    public class FKRobot : MonoBehaviour
    {
        private GameObject robot;
        private static int prismaticJointVariable = 3;
        private static int revoluteJointVariable = 2;
        public List<float[]> dh;
        public List<ArticulationBody> jointChain;
        public List<float> currentAngles;

        public Quaternion endEffectorRotation;
        public Matrix4x4 endEffectorPosition;

        // TODO : Automatically adding DH parameters 2. Detecting number of active joints
        void Start()
        {
            robot = GameObject.FindWithTag("robot");
            if(dh == null)
                dh = new List<float[]>();
            
            jointChain = new List<ArticulationBody>();
            foreach (ArticulationBody joint in robot.GetComponentsInChildren<ArticulationBody>())
            {
                if (joint.jointType != ArticulationJointType.FixedJoint)
                    jointChain.Add(joint);
            }
        }

        void FixedUpdate()
        {
            if(dh.Count == jointChain.Count)
                FK();
        }

        /// <summary>
        /// Returns a homogenous transform which moves the end effector co-ordinate system to the world co-ordinate system
        /// </summary>
        /// <param name="angles">List of float numbers representing joint poistions of a robot in meters or radians.</param>
        /// <returns></returns>
        public Matrix4x4 FK(List<float> angles = null)
        {
            currentAngles = currentJointParameters();
            if (angles == null)
            {
                PopulateDHparameters(currentAngles);
            }
            else
            {
                PopulateDHparameters(angles);
            }
            endEffectorPosition = Matrix4x4.identity;
            for (int i = 0; i < dh.Count; i++)
            {
                Matrix4x4 temp = FWDMatrix2(dh[i]).transpose;
                endEffectorPosition = endEffectorPosition * (FWDMatrix2(dh[i]).transpose);
            }

            return endEffectorPosition;
        
        }

        /// <summary>
        /// Modifies the DH parameters with the current joint positions from the robot in Unity Simulation
        /// JointPostition: 0-2: rotation along XYZ axis 3-5: Translation along XYZ co-ordinates
        /// https://docs.unity3d.com/2020.1/Documentation/ScriptReference/ArticulationBody-jointPosition.html
        /// </summary>
        /// <param name="angles">List of float numbers representing joint poistions of a robot in meters or radians</param>
        public void PopulateDHparameters(List<float> angles)
        {
            for (int i = 0; i < jointChain.Count; i++)
            {
                if (jointChain[i].jointType == ArticulationJointType.RevoluteJoint)
                {
                    dh[i][revoluteJointVariable] = jointChain[i].jointPosition[0];
                }
                else if (jointChain[i].jointType == ArticulationJointType.PrismaticJoint)
                {
                    dh[i][prismaticJointVariable] = jointChain[i].jointPosition[3];
                }
                else
                {
                    Debug.LogError("Other joint types not supported");
                }
            }


        }

        public List<float> currentJointParameters()
        {
            List<float> angles = new List<float>();
            for (int i = 0; i < jointChain.Count; i++)
            {
                angles.Add((float)Math.Round(jointChain[i].jointPosition[0],2));
            }
            return angles;
        }

        /// <summary>
        /// Returns a homogenous transformatino matrix formed using a set of DH paramters of a joint in new DH convention.
        /// https://en.wikipedia.org/wiki/Denavit–Hartenberg_parameters
        /// https://docs.unity3d.com/2020.1/Documentation/ScriptReference/ArticulationBody-jointPosition.html
        /// </summary>
        /// <param name="DHparameters">Array of four float parameters</param>
        /// <returns></returns>
        private Matrix4x4 FWDMatrix(float[] DHparameters) 
        {
            return new Matrix4x4(new Vector4(Mathf.Cos(DHparameters[2]),-Mathf.Sin(DHparameters[2]),0,DHparameters[1]),
                                new Vector4(Mathf.Sin(DHparameters[2]) * Mathf.Cos(DHparameters[0]),Mathf.Cos(DHparameters[2]) * Mathf.Cos(DHparameters[0]),-Mathf.Sin(DHparameters[0]),-Mathf.Sin(DHparameters[0]) * DHparameters[3]),
                                new Vector4(Mathf.Sin(DHparameters[2]) * Mathf.Sin(DHparameters[0]),Mathf.Cos(DHparameters[2]) * Mathf.Sin(DHparameters[0]), Mathf.Cos(DHparameters[0]),Mathf.Cos(DHparameters[0]) * DHparameters[3]),
                                new Vector4(0,0,0,1));
        }

        /// <summary>
        /// Returns a homogenous transformatino matrix formed using a set of DH paramters of a joint in new DH convention.
        /// https://en.wikipedia.org/wiki/Denavit–Hartenberg_parameters
        /// </summary>
        /// <param name="DHparameters">Array of four float parameters</param>
        /// <returns></returns>
        private Matrix4x4 FWDMatrix2(float[] DHparameters) 
        {
            return new Matrix4x4(new Vector4(Mathf.Cos(DHparameters[2]), -Mathf.Sin(DHparameters[2]) * Mathf.Cos(DHparameters[0]), Mathf.Sin(DHparameters[2]) * Mathf.Sin(DHparameters[0]), DHparameters[1] * Mathf.Cos(DHparameters[2])),
                                new Vector4(Mathf.Sin(DHparameters[2]) , Mathf.Cos(DHparameters[2]) * Mathf.Cos(DHparameters[0]), -Mathf.Sin(DHparameters[0]) * Mathf.Cos(DHparameters[2]), Mathf.Sin(DHparameters[2]) * DHparameters[1]),
                                new Vector4(0, Mathf.Sin(DHparameters[0]), Mathf.Cos(DHparameters[0]), DHparameters[3]),
                                new Vector4(0, 0, 0, 1));
        }

    }
}