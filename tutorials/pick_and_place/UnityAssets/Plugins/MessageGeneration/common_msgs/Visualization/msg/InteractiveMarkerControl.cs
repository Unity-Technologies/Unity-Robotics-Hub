using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Visualization
{
    public class InteractiveMarkerControl : Message
    {
        public const string RosMessageName = "visualization_msgs/InteractiveMarkerControl";

        //  Represents a control that is to be displayed together with an interactive marker
        //  Identifying string for this control.
        //  You need to assign a unique value to this to receive feedback from the GUI
        //  on what actions the user performs on this control (e.g. a button click).
        public string name { get; set; }
        //  Defines the local coordinate frame (relative to the pose of the parent
        //  interactive marker) in which is being rotated and translated.
        //  Default: Identity
        public Quaternion orientation { get; set; }
        //  Orientation mode: controls how orientation changes.
        //  INHERIT: Follow orientation of interactive marker
        //  FIXED: Keep orientation fixed at initial state
        //  VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
        public const byte INHERIT = 0;
        public const byte FIXED = 1;
        public const byte VIEW_FACING = 2;
        public byte orientation_mode { get; set; }
        //  Interaction mode for this control
        //  
        //  NONE: This control is only meant for visualization; no context menu.
        //  MENU: Like NONE, but right-click menu is active.
        //  BUTTON: Element can be left-clicked.
        //  MOVE_AXIS: Translate along local x-axis.
        //  MOVE_PLANE: Translate in local y-z plane.
        //  ROTATE_AXIS: Rotate around local x-axis.
        //  MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
        public const byte NONE = 0;
        public const byte MENU = 1;
        public const byte BUTTON = 2;
        public const byte MOVE_AXIS = 3;
        public const byte MOVE_PLANE = 4;
        public const byte ROTATE_AXIS = 5;
        public const byte MOVE_ROTATE = 6;
        //  "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
        //  MOVE_3D: Translate freely in 3D space.
        //  ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
        //  MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
        public const byte MOVE_3D = 7;
        public const byte ROTATE_3D = 8;
        public const byte MOVE_ROTATE_3D = 9;
        public byte interaction_mode { get; set; }
        //  If true, the contained markers will also be visible
        //  when the gui is not in interactive mode.
        public bool always_visible { get; set; }
        //  Markers to be displayed as custom visual representation.
        //  Leave this empty to use the default control handles.
        // 
        //  Note: 
        //  - The markers can be defined in an arbitrary coordinate frame,
        //    but will be transformed into the local frame of the interactive marker.
        //  - If the header of a marker is empty, its pose will be interpreted as 
        //    relative to the pose of the parent interactive marker.
        public Marker[] markers { get; set; }
        //  In VIEW_FACING mode, set this to true if you don't want the markers
        //  to be aligned with the camera view point. The markers will show up
        //  as in INHERIT mode.
        public bool independent_marker_orientation { get; set; }
        //  Short description (< 40 characters) of what this control does,
        //  e.g. "Move the robot". 
        //  Default: A generic description based on the interaction mode
        public string description { get; set; }

        public InteractiveMarkerControl()
        {
            this.name = "";
            this.orientation = new Quaternion();
            this.orientation_mode = 0;
            this.interaction_mode = 0;
            this.always_visible = false;
            this.markers = new Marker[0];
            this.independent_marker_orientation = false;
            this.description = "";
        }

        public InteractiveMarkerControl(string name, Quaternion orientation, byte orientation_mode, byte interaction_mode, bool always_visible, Marker[] markers, bool independent_marker_orientation, string description)
        {
            this.name = name;
            this.orientation = orientation;
            this.orientation_mode = orientation_mode;
            this.interaction_mode = interaction_mode;
            this.always_visible = always_visible;
            this.markers = markers;
            this.independent_marker_orientation = independent_marker_orientation;
            this.description = description;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.name));
            listOfSerializations.AddRange(orientation.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.orientation_mode));
            listOfSerializations.Add(BitConverter.GetBytes(this.interaction_mode));
            listOfSerializations.Add(BitConverter.GetBytes(this.always_visible));
            
            listOfSerializations.Add(BitConverter.GetBytes(markers.Length));
            foreach(var entry in markers)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.Add(BitConverter.GetBytes(this.independent_marker_orientation));
            listOfSerializations.Add(SerializeString(this.description));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.name = DeserializeString(data, offset, nameStringBytesLength);
            offset += nameStringBytesLength;
            offset = this.orientation.Deserialize(data, offset);
            this.orientation_mode = data[offset];;
            offset += 1;
            this.interaction_mode = data[offset];;
            offset += 1;
            this.always_visible = BitConverter.ToBoolean(data, offset);
            offset += 1;
            
            var markersArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.markers= new Marker[markersArrayLength];
            for(var i =0; i <markersArrayLength; i++)
            {
                this.markers[i] = new Marker();
                offset = this.markers[i].Deserialize(data, offset);
            }
            this.independent_marker_orientation = BitConverter.ToBoolean(data, offset);
            offset += 1;
            var descriptionStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.description = DeserializeString(data, offset, descriptionStringBytesLength);
            offset += descriptionStringBytesLength;

            return offset;
        }

    }
}
