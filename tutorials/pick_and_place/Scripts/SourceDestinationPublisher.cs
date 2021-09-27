using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class SourceDestinationPublisher : MonoBehaviour
{
    const int k_NumRobotJoints = 6;

    // Variables required for ROS communication
    [SerializeField]
    string m_TopicName = "/niryo_joints";

    [SerializeField]
    GameObject m_NiryoOne;
    [SerializeField]
    GameObject m_Target;
    [SerializeField]
    GameObject m_TargetPlacement;
    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);

    // Robot Joints
    UrdfJointRevolute[] m_JointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<NiryoMoveitJointsMsg>(m_TopicName);

        m_JointArticulationBodies = new UrdfJointRevolute[k_NumRobotJoints];
        var linkName = "world/base_link/shoulder_link";
        m_JointArticulationBodies[0] = m_NiryoOne.transform.Find(linkName).GetComponent<UrdfJointRevolute>();

        linkName += "/arm_link";
        m_JointArticulationBodies[1] = m_NiryoOne.transform.Find(linkName).GetComponent<UrdfJointRevolute>();

        linkName += "/elbow_link";
        m_JointArticulationBodies[2] = m_NiryoOne.transform.Find(linkName).GetComponent<UrdfJointRevolute>();

        linkName += "/forearm_link";
        m_JointArticulationBodies[3] = m_NiryoOne.transform.Find(linkName).GetComponent<UrdfJointRevolute>();

        linkName += "/wrist_link";
        m_JointArticulationBodies[4] = m_NiryoOne.transform.Find(linkName).GetComponent<UrdfJointRevolute>();

        linkName += "/hand_link";
        m_JointArticulationBodies[5] = m_NiryoOne.transform.Find(linkName).GetComponent<UrdfJointRevolute>();
    }

    public void Publish()
    {
        var sourceDestinationMessage = new NiryoMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            sourceDestinationMessage.joints[i] = m_JointArticulationBodies[i].GetPosition();
        }

        // Pick Pose
        sourceDestinationMessage.pick_pose = new PoseMsg
        {
            position = m_Target.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, m_Target.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        sourceDestinationMessage.place_pose = new PoseMsg
        {
            position = m_TargetPlacement.transform.position.To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        // Finally send the message to server_endpoint.py running in ROS
        m_Ros.Send(m_TopicName, sourceDestinationMessage);
    }
}
