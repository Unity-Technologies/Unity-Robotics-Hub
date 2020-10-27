using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Visualization
{
    public class InteractiveMarker : Message
    {
        public const string RosMessageName = "visualization_msgs/InteractiveMarker";

        //  Time/frame info.
        //  If header.time is set to 0, the marker will be retransformed into
        //  its frame on each timestep. You will receive the pose feedback
        //  in the same frame.
        //  Otherwise, you might receive feedback in a different frame.
        //  For rviz, this will be the current 'fixed frame' set by the user.
        public Header header { get; set; }
        //  Initial pose. Also, defines the pivot point for rotations.
        public Pose pose { get; set; }
        //  Identifying string. Must be globally unique in
        //  the topic that this message is sent through.
        public string name { get; set; }
        //  Short description (< 40 characters).
        public string description { get; set; }
        //  Scale to be used for default controls (default=1).
        public float scale { get; set; }
        //  All menu and submenu entries associated with this marker.
        public MenuEntry[] menu_entries { get; set; }
        //  List of controls displayed for this marker.
        public InteractiveMarkerControl[] controls { get; set; }

        public InteractiveMarker()
        {
            this.header = new Header();
            this.pose = new Pose();
            this.name = "";
            this.description = "";
            this.scale = 0.0f;
            this.menu_entries = new MenuEntry[0];
            this.controls = new InteractiveMarkerControl[0];
        }

        public InteractiveMarker(Header header, Pose pose, string name, string description, float scale, MenuEntry[] menu_entries, InteractiveMarkerControl[] controls)
        {
            this.header = header;
            this.pose = pose;
            this.name = name;
            this.description = description;
            this.scale = scale;
            this.menu_entries = menu_entries;
            this.controls = controls;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(pose.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.name));
            listOfSerializations.Add(SerializeString(this.description));
            listOfSerializations.Add(BitConverter.GetBytes(this.scale));
            
            listOfSerializations.Add(BitConverter.GetBytes(menu_entries.Length));
            foreach(var entry in menu_entries)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(controls.Length));
            foreach(var entry in controls)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.pose.Deserialize(data, offset);
            var nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.name = DeserializeString(data, offset, nameStringBytesLength);
            offset += nameStringBytesLength;
            var descriptionStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.description = DeserializeString(data, offset, descriptionStringBytesLength);
            offset += descriptionStringBytesLength;
            this.scale = BitConverter.ToSingle(data, offset);
            offset += 4;
            
            var menu_entriesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.menu_entries= new MenuEntry[menu_entriesArrayLength];
            for(var i =0; i <menu_entriesArrayLength; i++)
            {
                this.menu_entries[i] = new MenuEntry();
                offset = this.menu_entries[i].Deserialize(data, offset);
            }
            
            var controlsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.controls= new InteractiveMarkerControl[controlsArrayLength];
            for(var i =0; i <controlsArrayLength; i++)
            {
                this.controls[i] = new InteractiveMarkerControl();
                offset = this.controls[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
