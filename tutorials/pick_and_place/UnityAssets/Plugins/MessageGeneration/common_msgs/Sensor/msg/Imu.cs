using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Sensor
{
    public class Imu : Message
    {
        public const string RosMessageName = "sensor_msgs/Imu";

        //  This is a message to hold data from an IMU (Inertial Measurement Unit)
        // 
        //  Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
        // 
        //  If the covariance of the measurement is known, it should be filled in (if all you know is the 
        //  variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
        //  A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
        //  data a covariance will have to be assumed or gotten from some other source
        // 
        //  If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
        //  estimate), please set element 0 of the associated covariance matrix to -1
        //  If you are interpreting this message, please check for a value of -1 in the first element of each 
        //  covariance matrix, and disregard the associated estimate.
        public Header header { get; set; }
        public Quaternion orientation { get; set; }
        public double[] orientation_covariance { get; set; }
        //  Row major about x, y, z axes
        public Vector3 angular_velocity { get; set; }
        public double[] angular_velocity_covariance { get; set; }
        //  Row major about x, y, z axes
        public Vector3 linear_acceleration { get; set; }
        public double[] linear_acceleration_covariance { get; set; }
        //  Row major x, y z 

        public Imu()
        {
            this.header = new Header();
            this.orientation = new Quaternion();
            this.orientation_covariance = new double[9];
            this.angular_velocity = new Vector3();
            this.angular_velocity_covariance = new double[9];
            this.linear_acceleration = new Vector3();
            this.linear_acceleration_covariance = new double[9];
        }

        public Imu(Header header, Quaternion orientation, double[] orientation_covariance, Vector3 angular_velocity, double[] angular_velocity_covariance, Vector3 linear_acceleration, double[] linear_acceleration_covariance)
        {
            this.header = header;
            this.orientation = orientation;
            this.orientation_covariance = orientation_covariance;
            this.angular_velocity = angular_velocity;
            this.angular_velocity_covariance = angular_velocity_covariance;
            this.linear_acceleration = linear_acceleration;
            this.linear_acceleration_covariance = linear_acceleration_covariance;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(orientation.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(orientation_covariance.Length));
            foreach(var entry in orientation_covariance)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            listOfSerializations.AddRange(angular_velocity.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(angular_velocity_covariance.Length));
            foreach(var entry in angular_velocity_covariance)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            listOfSerializations.AddRange(linear_acceleration.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(linear_acceleration_covariance.Length));
            foreach(var entry in linear_acceleration_covariance)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.orientation.Deserialize(data, offset);
            
            var orientation_covarianceArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.orientation_covariance= new double[orientation_covarianceArrayLength];
            for(var i =0; i <orientation_covarianceArrayLength; i++)
            {
                this.orientation_covariance[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }
            offset = this.angular_velocity.Deserialize(data, offset);
            
            var angular_velocity_covarianceArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.angular_velocity_covariance= new double[angular_velocity_covarianceArrayLength];
            for(var i =0; i <angular_velocity_covarianceArrayLength; i++)
            {
                this.angular_velocity_covariance[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }
            offset = this.linear_acceleration.Deserialize(data, offset);
            
            var linear_acceleration_covarianceArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.linear_acceleration_covariance= new double[linear_acceleration_covarianceArrayLength];
            for(var i =0; i <linear_acceleration_covarianceArrayLength; i++)
            {
                this.linear_acceleration_covariance[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }

            return offset;
        }

    }
}
