using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Visualization
{
    public class InteractiveMarkerInit : Message
    {
        public const string RosMessageName = "visualization_msgs/InteractiveMarkerInit";

        //  Identifying string. Must be unique in the topic namespace
        //  that this server works on.
        public string server_id { get; set; }
        //  Sequence number.
        //  The client will use this to detect if it has missed a subsequent
        //  update.  Every update message will have the same sequence number as
        //  an init message.  Clients will likely want to unsubscribe from the
        //  init topic after a successful initialization to avoid receiving
        //  duplicate data.
        public ulong seq_num { get; set; }
        //  All markers.
        public InteractiveMarker[] markers { get; set; }

        public InteractiveMarkerInit()
        {
            this.server_id = "";
            this.seq_num = 0;
            this.markers = new InteractiveMarker[0];
        }

        public InteractiveMarkerInit(string server_id, ulong seq_num, InteractiveMarker[] markers)
        {
            this.server_id = server_id;
            this.seq_num = seq_num;
            this.markers = markers;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.server_id));
            listOfSerializations.Add(BitConverter.GetBytes(this.seq_num));
            
            listOfSerializations.Add(BitConverter.GetBytes(markers.Length));
            foreach(var entry in markers)
                listOfSerializations.Add(entry.Serialize());

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
            
            var markersArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.markers= new InteractiveMarker[markersArrayLength];
            for(var i =0; i <markersArrayLength; i++)
            {
                this.markers[i] = new InteractiveMarker();
                offset = this.markers[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
