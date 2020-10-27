using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// TODO: Check matrix access type across the board;
namespace RosSharp.Control
{
    public class IKRobot : MonoBehaviour
    {
        private GameObject robot;
        private FKRobot Fk;
        private float[,] jacobian;

        public float degreeDelta = .5f;
        public float mDelta = .001f;
        void Start()
        {
            robot = GameObject.FindWithTag("robot");
            Fk = robot.GetComponent<FKRobot>();
            jacobian = new float[6, Fk.jointChain.Count];
        }

        void Update()
        {
            List<float> jointParameters = Fk.currentJointParameters();
            CalculateJacobian(jointParameters);
        }

        private void CalculateJacobian(List<float> jointParameters)
        {
            Matrix4x4 currentP = Fk.FK(jointParameters);
            for(int i = 0; i < jointParameters.Count; i++)
            {
                Matrix4x4 dT = CalculateDifferential(jointParameters, differentialOrder.first,i);
                jacobian[i, 0] = dT[0, 3];
                jacobian[i, 1] = dT[1, 3];
                jacobian[i, 2] = dT[2, 3];
            }
        }


        //Central Difference
        private Matrix4x4 CalculateDifferential(List<float> jointParameters, differentialOrder order, int jointNumber)
        {
            float delta;
            Matrix4x4 dT = new Matrix4x4();
            if (Fk.jointChain[jointNumber].jointType == ArticulationJointType.RevoluteJoint)
            {
                delta = degreeDelta;
            }
            else
            {
                delta = mDelta;
            }

            if (order == differentialOrder.first)
            {
                List<float> front = jointParameters;
                List<float> back = jointParameters;
                front[jointNumber] += delta;
                back[jointNumber] -= delta;
                Matrix4x4 Tfront = Fk.FK(front);
                Matrix4x4 Tback = Fk.FK(back);
                Matrix4x4 Tdiff = Tfront.Subtract(Tback);
                dT = Tdiff.FloatDivide(degreeDelta * Mathf.Deg2Rad);
            }
            return dT;
        }

        enum differentialOrder
        {
            first = 1,
            second = 2,
            fourth = 4,
            sixth = 6,
            eigth = 8
        }
    }
}
