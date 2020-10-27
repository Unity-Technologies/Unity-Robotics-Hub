using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Visualization
{
    public class InteractiveMarkerFeedback : Message
    {
        public const string RosMessageName = "visualization_msgs/InteractiveMarkerFeedback";

        //  Time/frame info.
        public Header header { get; set; }
        //  Identifying string. Must be unique in the topic namespace.
        public string client_id { get; set; }
        //  Feedback message sent back from the GUI, e.g.
        //  when the status of an interactive marker was modified by the user.
        //  Specifies which interactive marker and control this message refers to
        public string marker_name { get; set; }
        public string control_name { get; set; }
        //  Type of the event
        //  KEEP_ALIVE: sent while dragging to keep up control of the marker
        //  MENU_SELECT: a menu entry has been selected
        //  BUTTON_CLICK: a button control has been clicked
        //  POSE_UPDATE: the pose has been changed using one of the controls
        public const byte KEEP_ALIVE = 0;
        public const byte POSE_UPDATE = 1;
        public const byte MENU_SELECT = 2;
        public const byte BUTTON_CLICK = 3;
        public const byte MOUSE_DOWN = 4;
        public const byte MOUSE_UP = 5;
        public byte event_type { get; set; }
        //  Current pose of the marker
        //  Note: Has to be valid for all feedback types.
        public Pose pose { get; set; }
        //  Contains the ID of the selected menu entry
        //  Only valid for MENU_SELECT events.
        public uint menu_entry_id { get; set; }
        //  If event_type is BUTTON_CLICK, MOUSE_DOWN, or MOUSE_UP, mouse_point
        //  may contain the 3 dimensional position of the event on the
        //  control.  If it does, mouse_point_valid will be true.  mouse_point
        //  will be relative to the frame listed in the header.
        public Point mouse_point { get; set; }
        public bool mouse_point_valid { get; set; }

        public InteractiveMarkerFeedback()
        {
            this.header = new Header();
            this.client_id = "";
            this.marker_name = "";
            this.control_name = "";
            this.event_type = 0;
            this.pose = new Pose();
            this.menu_entry_id = 0;
            this.mouse_point = new Point();
            this.mouse_point_valid = false;
        }

        public InteractiveMarkerFeedback(Header header, string client_id, string marker_name, string control_name, byte event_type, Pose pose, uint menu_entry_id, Point mouse_point, bool mouse_point_valid)
        {
            this.header = header;
            this.client_id = client_id;
            this.marker_name = marker_name;
            this.control_name = control_name;
            this.event_type = event_type;
            this.pose = pose;
            this.menu_entry_id = menu_entry_id;
            this.mouse_point = mouse_point;
            this.mouse_point_valid = mouse_point_valid;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.client_id));
            listOfSerializations.Add(SerializeString(this.marker_name));
            listOfSerializations.Add(SerializeString(this.control_name));
            listOfSerializations.Add(BitConverter.GetBytes(this.event_type));
            listOfSerializations.AddRange(pose.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.menu_entry_id));
            listOfSerializations.AddRange(mouse_point.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.mouse_point_valid));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            var client_idStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.client_id = DeserializeString(data, offset, client_idStringBytesLength);
            offset += client_idStringBytesLength;
            var marker_nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.marker_name = DeserializeString(data, offset, marker_nameStringBytesLength);
            offset += marker_nameStringBytesLength;
            var control_nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.control_name = DeserializeString(data, offset, control_nameStringBytesLength);
            offset += control_nameStringBytesLength;
            this.event_type = data[offset];;
            offset += 1;
            offset = this.pose.Deserialize(data, offset);
            this.menu_entry_id = BitConverter.ToUInt32(data, offset);
            offset += 4;
            offset = this.mouse_point.Deserialize(data, offset);
            this.mouse_point_valid = BitConverter.ToBoolean(data, offset);
            offset += 1;

            return offset;
        }

    }
}
