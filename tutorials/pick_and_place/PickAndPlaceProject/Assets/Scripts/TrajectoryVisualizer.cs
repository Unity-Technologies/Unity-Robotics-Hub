using RosMessageTypes.NiryoMoveit;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

[AutoRegisteredVisualizer]
public class TrajectoryVisualizer : IMessageVisualizer<MoverServiceRequest>
{
    MoverServiceRequest msg;
    string msg_string;
    DebugDraw.Drawing drawing;

    public void Begin(MoverServiceRequest msg, MessageMetadata meta)
    {
        this.msg = msg;
        this.msg_string = msg.ToString();

        drawing = DebugDraw.CreateDrawing();

        MessageVisualizations.Draw<FLU>(drawing, msg.pick_pose, Color.red, "pick_pose");
        MessageVisualizations.Draw<FLU>(drawing, msg.place_pose, Color.green, "place_pose");
    }

    public void End()
    {
    }

    public void OnGUI()
    {
        GUILayout.Label(msg_string);
    }
}
