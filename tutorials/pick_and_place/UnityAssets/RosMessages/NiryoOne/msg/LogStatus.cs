using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.NiryoOne
{
    public class LogStatus : Message
    {
        public const string RosMessageName = "niryo_one_msgs/LogStatus";

        public Header header { get; set; }
        //  in MB
        public int log_size { get; set; }
        public int available_disk_size { get; set; }
        public bool purge_log_on_startup { get; set; }

        public LogStatus()
        {
            this.header = new Header();
            this.log_size = 0;
            this.available_disk_size = 0;
            this.purge_log_on_startup = false;
        }

        public LogStatus(Header header, int log_size, int available_disk_size, bool purge_log_on_startup)
        {
            this.header = header;
            this.log_size = log_size;
            this.available_disk_size = available_disk_size;
            this.purge_log_on_startup = purge_log_on_startup;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.log_size));
            listOfSerializations.Add(BitConverter.GetBytes(this.available_disk_size));
            listOfSerializations.Add(BitConverter.GetBytes(this.purge_log_on_startup));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            this.log_size = BitConverter.ToInt32(data, offset);
            offset += 4;
            this.available_disk_size = BitConverter.ToInt32(data, offset);
            offset += 4;
            this.purge_log_on_startup = BitConverter.ToBoolean(data, offset);
            offset += 1;

            return offset;
        }

    }
}
