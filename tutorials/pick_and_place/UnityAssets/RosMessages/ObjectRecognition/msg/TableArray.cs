using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.ObjectRecognition;

namespace RosMessageTypes.ObjectRecognition
{
    public class TableArray : Message
    {
        public const string RosMessageName = "object_recognition_msgs-master/TableArray";

        public Header header { get; set; }
        //  Just an array of tables
        public Table[] tables { get; set; }

        public TableArray()
        {
            this.header = new Header();
            this.tables = new Table[0];
        }

        public TableArray(Header header, Table[] tables)
        {
            this.header = header;
            this.tables = tables;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(tables.Length));
            foreach(var entry in tables)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            
            var tablesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.tables= new Table[tablesArrayLength];
            for(var i =0; i <tablesArrayLength; i++)
            {
                this.tables[i] = new Table();
                offset = this.tables[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
