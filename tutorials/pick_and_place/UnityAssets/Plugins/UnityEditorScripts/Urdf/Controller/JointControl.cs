using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp;

public class JointControl : MonoBehaviour
{
    RosSharp.Control.Controller controller;

    public RosSharp.Control.RotationDirection direction;
    public RosSharp.Control.ControlType controltype;
    public float speed ;
    public float torque ;
    public float acceleration;
    public ArticulationBody joint;

    public float jointforce;

    void Start()
    {
        direction = 0;
        controller = (RosSharp.Control.Controller)this.GetComponentInParent(typeof(RosSharp.Control.Controller));
        joint = this.GetComponent<ArticulationBody>();
        controller.UpdateControlType(this);
        speed = controller.speed;
        torque = controller.torque;
        acceleration = controller.acceleration;
    }

    void FixedUpdate(){

        speed = controller.speed;
        torque = controller.torque;
        acceleration = controller.acceleration;

        if(joint.jointType == ArticulationJointType.RevoluteJoint)
            jointforce = joint.jointForce[0];

        if(joint.jointType != ArticulationJointType.FixedJoint)
        {
            if (controltype == RosSharp.Control.ControlType.PositionControl)
            {
                ArticulationDrive currentDrive = joint.xDrive;
                float newTargetDelta = (int)direction * Time.fixedDeltaTime * speed;
                currentDrive.target += newTargetDelta;
                joint.xDrive = currentDrive;
            }
            
        }
    }
}
