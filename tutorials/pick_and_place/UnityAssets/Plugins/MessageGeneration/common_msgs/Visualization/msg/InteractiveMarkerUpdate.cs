using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Visualization
{
    public class InteractiveMarkerUpdate : Message
    {
        public const string RosMessageName = "visualization_msgs/InteractiveMarkerUpdate";

        //  Identifying string. Must be unique in the topic namespace
        //  that this server works on.
        public string server_id { get; set; }
        //  Sequence number.
        //  The client will use this to detect if it has missed an update.
        public ulong seq_num { get; set; }
        //  Type holds the purpose of this message.  It must be one of UPDATE or KEEP_ALIVE.
        //  UPDATE: Incremental update to previous state. 
        //          The sequence number must be 1 higher than for
        //          the previous update.
        //  KEEP_ALIVE: Indicates the that the server is still living.
        //              The sequence number does not increase.
        //              No payload data should be filled out (markers, poses, or erases).
        public const byte KEEP_ALIVE = 0;
        public const byte UPDATE = 1;
        public byte type { get; set; }
        // Note: No guarantees on the order of processing.
        //       Contents must be kept consistent by sender.
        // Markers to be added or updated
        public InteractiveMarker[] markers { get; set; }
        // Poses of markers that should be moved
        public InteractiveMarkerPose[] poses { get; set; }
        // Names of markers to be erased
        public string[] erases { get; set; }

        public InteractiveMarkerUpdate()
        {
            this.server_id = "";
            this.seq_num = 0;
            this.type = 0;
            this.markers = new InteractiveMarker[0];
            this.poses = new InteractiveMarkerPose[0];
            this.erases = new string[0];
        }

        public InteractiveMarkerUpdate(string server_id, ulong seq_num, byte type, InteractiveMarker[] markers, InteractiveMarkerPose[] poses, string[] erases)
        {
            this.server_id = server_id;
            this.seq_num = seq_num;
            this.type = type;
            this.markers = markers;
            this.poses = poses;
            this.erases = erases;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.server_id));
            listOfSerializations.Add(BitConverter.GetBytes(this.seq_num));
            listOfSerializations.Add(BitConverter.GetBytes(this.type));
            
            listOfSerializations.Add(BitConverter.GetBytes(markers.Length));
            foreach(var entry in markers)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(poses.Length));
            foreach(var entry in poses)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(erases.Length));
            foreach(var entry in erases)
                listOfSerializations.Add(SerializeString(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var server_idStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.server_id = DeserializeString(data, offset, server_idStringBytesLength);
            offset += server_idStringBytesLength;
            this.seq_num = BitConverter.ToUInt64(data, offset);
            offset += 8;
            this.type = data[offset];;
            offset += 1;
            
            var markersArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.markers= new InteractiveMarker[markersArrayLength];
            for(var i =0; i <markersArrayLength; i++)
            {
                this.markers[i] = new InteractiveMarker();
                offset = this.markers[i].Deserialize(data, offset);
            }
            
            var posesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.poses= new InteractiveMarkerPose[posesArrayLength];
            for(var i =0; i <posesArrayLength; i++)
            {
                this.poses[i] = new InteractiveMarkerPose();
                offset = this.poses[i].Deserialize(data, offset);
            }
            
            var erasesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.erases= new string[erasesArrayLength];
            for(var i =0; i <erasesArrayLength; i++)
            {
                var erasesStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.erases[i] = DeserializeString(data, offset, erasesStringBytesLength);
                offset += erasesStringBytesLength;
            }

            return offset;
        }

    }
}
