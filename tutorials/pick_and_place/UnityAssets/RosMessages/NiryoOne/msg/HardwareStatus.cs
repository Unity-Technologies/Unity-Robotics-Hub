using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.NiryoOne
{
    public class HardwareStatus : Message
    {
        public const string RosMessageName = "niryo_one_msgs/HardwareStatus";

        public Header header { get; set; }
        //  Raspberry Pi board
        public int rpi_temperature { get; set; }
        //  Robot version : 1 (previous one) or 2 (current one)
        public int hardware_version { get; set; }
        //  Motors
        public bool connection_up { get; set; }
        public string error_message { get; set; }
        public int calibration_needed { get; set; }
        public bool calibration_in_progress { get; set; }
        public string[] motor_names { get; set; }
        public string[] motor_types { get; set; }
        public int[] temperatures { get; set; }
        public double[] voltages { get; set; }
        public int[] hardware_errors { get; set; }

        public HardwareStatus()
        {
            this.header = new Header();
            this.rpi_temperature = 0;
            this.hardware_version = 0;
            this.connection_up = false;
            this.error_message = "";
            this.calibration_needed = 0;
            this.calibration_in_progress = false;
            this.motor_names = new string[0];
            this.motor_types = new string[0];
            this.temperatures = new int[0];
            this.voltages = new double[0];
            this.hardware_errors = new int[0];
        }

        public HardwareStatus(Header header, int rpi_temperature, int hardware_version, bool connection_up, string error_message, int calibration_needed, bool calibration_in_progress, string[] motor_names, string[] motor_types, int[] temperatures, double[] voltages, int[] hardware_errors)
        {
            this.header = header;
            this.rpi_temperature = rpi_temperature;
            this.hardware_version = hardware_version;
            this.connection_up = connection_up;
            this.error_message = error_message;
            this.calibration_needed = calibration_needed;
            this.calibration_in_progress = calibration_in_progress;
            this.motor_names = motor_names;
            this.motor_types = motor_types;
            this.temperatures = temperatures;
            this.voltages = voltages;
            this.hardware_errors = hardware_errors;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.rpi_temperature));
            listOfSerializations.Add(BitConverter.GetBytes(this.hardware_version));
            listOfSerializations.Add(BitConverter.GetBytes(this.connection_up));
            listOfSerializations.Add(SerializeString(this.error_message));
            listOfSerializations.Add(BitConverter.GetBytes(this.calibration_needed));
            listOfSerializations.Add(BitConverter.GetBytes(this.calibration_in_progress));
            
            listOfSerializations.Add(BitConverter.GetBytes(motor_names.Length));
            foreach(var entry in motor_names)
                listOfSerializations.Add(SerializeString(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(motor_types.Length));
            foreach(var entry in motor_types)
                listOfSerializations.Add(SerializeString(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(temperatures.Length));
            foreach(var entry in temperatures)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(voltages.Length));
            foreach(var entry in voltages)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(hardware_errors.Length));
            foreach(var entry in hardware_errors)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            this.rpi_temperature = BitConverter.ToInt32(data, offset);
            offset += 4;
            this.hardware_version = BitConverter.ToInt32(data, offset);
            offset += 4;
            this.connection_up = BitConverter.ToBoolean(data, offset);
            offset += 1;
            var error_messageStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.error_message = DeserializeString(data, offset, error_messageStringBytesLength);
            offset += error_messageStringBytesLength;
            this.calibration_needed = BitConverter.ToInt32(data, offset);
            offset += 4;
            this.calibration_in_progress = BitConverter.ToBoolean(data, offset);
            offset += 1;
            
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
            
            var motor_typesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.motor_types= new string[motor_typesArrayLength];
            for(var i =0; i <motor_typesArrayLength; i++)
            {
                var motor_typesStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.motor_types[i] = DeserializeString(data, offset, motor_typesStringBytesLength);
                offset += motor_typesStringBytesLength;
            }
            
            var temperaturesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.temperatures= new int[temperaturesArrayLength];
            for(var i =0; i <temperaturesArrayLength; i++)
            {
                this.temperatures[i] = BitConverter.ToInt32(data, offset);
                offset += 4;
            }
            
            var voltagesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.voltages= new double[voltagesArrayLength];
            for(var i =0; i <voltagesArrayLength; i++)
            {
                this.voltages[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }
            
            var hardware_errorsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.hardware_errors= new int[hardware_errorsArrayLength];
            for(var i =0; i <hardware_errorsArrayLength; i++)
            {
                this.hardware_errors[i] = BitConverter.ToInt32(data, offset);
                offset += 4;
            }

            return offset;
        }

    }
}
