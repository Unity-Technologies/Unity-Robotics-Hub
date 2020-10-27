using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Visualization
{
    public class MarkerArray : Message
    {
        public const string RosMessageName = "visualization_msgs/MarkerArray";

        public Marker[] markers { get; set; }

        public MarkerArray()
        {
            this.markers = new Marker[0];
        }

        public MarkerArray(Marker[] markers)
        {
            this.markers = markers;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(markers.Length));
            foreach(var entry in markers)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var markersArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.markers= new Marker[markersArrayLength];
            for(var i =0; i <markersArrayLength; i++)
            {
                this.markers[i] = new Marker();
                offset = this.markers[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
