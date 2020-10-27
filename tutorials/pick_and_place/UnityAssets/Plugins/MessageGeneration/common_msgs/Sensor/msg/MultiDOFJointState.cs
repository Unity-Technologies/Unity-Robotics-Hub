using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Sensor
{
    public class MultiDOFJointState : Message
    {
        public const string RosMessageName = "sensor_msgs/MultiDOFJointState";

        //  Representation of state for joints with multiple degrees of freedom, 
        //  following the structure of JointState.
        // 
        //  It is assumed that a joint in a system corresponds to a transform that gets applied 
        //  along the kinematic chain. For example, a planar joint (as in URDF) is 3DOF (x, y, yaw)
        //  and those 3DOF can be expressed as a transformation matrix, and that transformation
        //  matrix can be converted back to (x, y, yaw)
        // 
        //  Each joint is uniquely identified by its name
        //  The header specifies the time at which the joint states were recorded. All the joint states
        //  in one message have to be recorded at the same time.
        // 
        //  This message consists of a multiple arrays, one for each part of the joint state. 
        //  The goal is to make each of the fields optional. When e.g. your joints have no
        //  wrench associated with them, you can leave the wrench array empty. 
        // 
        //  All arrays in this message should have the same size, or be empty.
        //  This is the only way to uniquely associate the joint name with the correct
        //  states.
        public Header header { get; set; }
        public string[] joint_names { get; set; }
        public Transform[] transforms { get; set; }
        public Twist[] twist { get; set; }
        public Wrench[] wrench { get; set; }

        public MultiDOFJointState()
        {
            this.header = new Header();
            this.joint_names = new string[0];
            this.transforms = new Transform[0];
            this.twist = new Twist[0];
            this.wrench = new Wrench[0];
        }

        public MultiDOFJointState(Header header, string[] joint_names, Transform[] transforms, Twist[] twist, Wrench[] wrench)
        {
            this.header = header;
            this.joint_names = joint_names;
            this.transforms = transforms;
            this.twist = twist;
            this.wrench = wrench;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(joint_names.Length));
            foreach(var entry in joint_names)
                listOfSerializations.Add(SerializeString(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(transforms.Length));
            foreach(var entry in transforms)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(twist.Length));
            foreach(var entry in twist)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(wrench.Length));
            foreach(var entry in wrench)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            
            var joint_namesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.joint_names= new string[joint_namesArrayLength];
            for(var i =0; i <joint_namesArrayLength; i++)
            {
                var joint_namesStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.joint_names[i] = DeserializeString(data, offset, joint_namesStringBytesLength);
                offset += joint_namesStringBytesLength;
            }
            
            var transformsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.transforms= new Transform[transformsArrayLength];
            for(var i =0; i <transformsArrayLength; i++)
            {
                this.transforms[i] = new Transform();
                offset = this.transforms[i].Deserialize(data, offset);
            }
            
            var twistArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.twist= new Twist[twistArrayLength];
            for(var i =0; i <twistArrayLength; i++)
            {
                this.twist[i] = new Twist();
                offset = this.twist[i].Deserialize(data, offset);
            }
            
            var wrenchArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.wrench= new Wrench[wrenchArrayLength];
            for(var i =0; i <wrenchArrayLength; i++)
            {
                this.wrench[i] = new Wrench();
                offset = this.wrench[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
