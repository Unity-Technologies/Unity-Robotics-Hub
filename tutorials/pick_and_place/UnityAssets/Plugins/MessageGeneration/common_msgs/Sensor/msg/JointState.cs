using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Sensor
{
    public class JointState : Message
    {
        public const string RosMessageName = "sensor_msgs/JointState";

        //  This is a message that holds data to describe the state of a set of torque controlled joints. 
        // 
        //  The state of each joint (revolute or prismatic) is defined by:
        //   * the position of the joint (rad or m),
        //   * the velocity of the joint (rad/s or m/s) and 
        //   * the effort that is applied in the joint (Nm or N).
        // 
        //  Each joint is uniquely identified by its name
        //  The header specifies the time at which the joint states were recorded. All the joint states
        //  in one message have to be recorded at the same time.
        // 
        //  This message consists of a multiple arrays, one for each part of the joint state. 
        //  The goal is to make each of the fields optional. When e.g. your joints have no
        //  effort associated with them, you can leave the effort array empty. 
        // 
        //  All arrays in this message should have the same size, or be empty.
        //  This is the only way to uniquely associate the joint name with the correct
        //  states.
        public Header header { get; set; }
        public string[] name { get; set; }
        public double[] position { get; set; }
        public double[] velocity { get; set; }
        public double[] effort { get; set; }

        public JointState()
        {
            this.header = new Header();
            this.name = new string[0];
            this.position = new double[0];
            this.velocity = new double[0];
            this.effort = new double[0];
        }

        public JointState(Header header, string[] name, double[] position, double[] velocity, double[] effort)
        {
            this.header = header;
            this.name = name;
            this.position = position;
            this.velocity = velocity;
            this.effort = effort;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(name.Length));
            foreach(var entry in name)
                listOfSerializations.Add(SerializeString(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(position.Length));
            foreach(var entry in position)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(velocity.Length));
            foreach(var entry in velocity)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(effort.Length));
            foreach(var entry in effort)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            
            var nameArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.name= new string[nameArrayLength];
            for(var i =0; i <nameArrayLength; i++)
            {
                var nameStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.name[i] = DeserializeString(data, offset, nameStringBytesLength);
                offset += nameStringBytesLength;
            }
            
            var positionArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.position= new double[positionArrayLength];
            for(var i =0; i <positionArrayLength; i++)
            {
                this.position[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }
            
            var velocityArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.velocity= new double[velocityArrayLength];
            for(var i =0; i <velocityArrayLength; i++)
            {
                this.velocity[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }
            
            var effortArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.effort= new double[effortArrayLength];
            for(var i =0; i <effortArrayLength; i++)
            {
                this.effort[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }

            return offset;
        }

    }
}
