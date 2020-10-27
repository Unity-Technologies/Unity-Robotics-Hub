using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    public class SoftwareVersion : Message
    {
        public const string RosMessageName = "niryo_one_msgs/SoftwareVersion";

        public string rpi_image_version { get; set; }
        public string ros_niryo_one_version { get; set; }
        public string[] motor_names { get; set; }
        public string[] stepper_firmware_versions { get; set; }

        public SoftwareVersion()
        {
            this.rpi_image_version = "";
            this.ros_niryo_one_version = "";
            this.motor_names = new string[0];
            this.stepper_firmware_versions = new string[0];
        }

        public SoftwareVersion(string rpi_image_version, string ros_niryo_one_version, string[] motor_names, string[] stepper_firmware_versions)
        {
            this.rpi_image_version = rpi_image_version;
            this.ros_niryo_one_version = ros_niryo_one_version;
            this.motor_names = motor_names;
            this.stepper_firmware_versions = stepper_firmware_versions;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.rpi_image_version));
            listOfSerializations.Add(SerializeString(this.ros_niryo_one_version));
            
            listOfSerializations.Add(BitConverter.GetBytes(motor_names.Length));
            foreach(var entry in motor_names)
                listOfSerializations.Add(SerializeString(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(stepper_firmware_versions.Length));
            foreach(var entry in stepper_firmware_versions)
                listOfSerializations.Add(SerializeString(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var rpi_image_versionStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.rpi_image_version = DeserializeString(data, offset, rpi_image_versionStringBytesLength);
            offset += rpi_image_versionStringBytesLength;
            var ros_niryo_one_versionStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.ros_niryo_one_version = DeserializeString(data, offset, ros_niryo_one_versionStringBytesLength);
            offset += ros_niryo_one_versionStringBytesLength;
            
            var motor_namesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.motor_names= new string[motor_namesArrayLength];
            for(var i =0; i <motor_namesArrayLength; i++)
            {
                var motor_namesStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.motor_names[i] = DeserializeString(data, offset, motor_namesStringBytesLength);
                offset += motor_namesStringBytesLength;
            }
            
            var stepper_firmware_versionsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.stepper_firmware_versions= new string[stepper_firmware_versionsArrayLength];
            for(var i =0; i <stepper_firmware_versionsArrayLength; i++)
            {
                var stepper_firmware_versionsStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.stepper_firmware_versions[i] = DeserializeString(data, offset, stepper_firmware_versionsStringBytesLength);
                offset += stepper_firmware_versionsStringBytesLength;
            }

            return offset;
        }

    }
}
