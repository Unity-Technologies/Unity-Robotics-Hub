using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.ObjectRecognition;

namespace RosMessageTypes.ObjectRecognition
{
    public class RecognizedObjectArray : Message
    {
        public const string RosMessageName = "object_recognition_msgs-master/RecognizedObjectArray";

        // #################################################### HEADER ###########################################################
        public Header header { get; set; }
        //  This message type describes a potential scene configuration: a set of objects that can explain the scene
        public RecognizedObject[] objects { get; set; }
        // #################################################### SEARCH ###########################################################
        //  The co-occurrence matrix between the recognized objects
        public float[] cooccurrence { get; set; }

        public RecognizedObjectArray()
        {
            this.header = new Header();
            this.objects = new RecognizedObject[0];
            this.cooccurrence = new float[0];
        }

        public RecognizedObjectArray(Header header, RecognizedObject[] objects, float[] cooccurrence)
        {
            this.header = header;
            this.objects = objects;
            this.cooccurrence = cooccurrence;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(objects.Length));
            foreach(var entry in objects)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(cooccurrence.Length));
            foreach(var entry in cooccurrence)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            
            var objectsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.objects= new RecognizedObject[objectsArrayLength];
            for(var i =0; i <objectsArrayLength; i++)
            {
                this.objects[i] = new RecognizedObject();
                offset = this.objects[i].Deserialize(data, offset);
            }
            
            var cooccurrenceArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.cooccurrence= new float[cooccurrenceArrayLength];
            for(var i =0; i <cooccurrenceArrayLength; i++)
            {
                this.cooccurrence[i] = BitConverter.ToSingle(data, offset);
                offset += 4;
            }

            return offset;
        }

    }
}
