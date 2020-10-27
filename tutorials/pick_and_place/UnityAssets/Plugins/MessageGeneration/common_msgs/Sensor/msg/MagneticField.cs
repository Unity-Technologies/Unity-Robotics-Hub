using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Sensor
{
    public class MagneticField : Message
    {
        public const string RosMessageName = "sensor_msgs/MagneticField";

        //  Measurement of the Magnetic Field vector at a specific location.
        //  If the covariance of the measurement is known, it should be filled in
        //  (if all you know is the variance of each measurement, e.g. from the datasheet,
        // just put those along the diagonal)
        //  A covariance matrix of all zeros will be interpreted as "covariance unknown",
        //  and to use the data a covariance will have to be assumed or gotten from some
        //  other source
        public Header header { get; set; }
        //  timestamp is the time the
        //  field was measured
        //  frame_id is the location and orientation
        //  of the field measurement
        public Vector3 magnetic_field { get; set; }
        //  x, y, and z components of the
        //  field vector in Tesla
        //  If your sensor does not output 3 axes,
        //  put NaNs in the components not reported.
        public double[] magnetic_field_covariance { get; set; }
        //  Row major about x, y, z axes
        //  0 is interpreted as variance unknown

        public MagneticField()
        {
            this.header = new Header();
            this.magnetic_field = new Vector3();
            this.magnetic_field_covariance = new double[9];
        }

        public MagneticField(Header header, Vector3 magnetic_field, double[] magnetic_field_covariance)
        {
            this.header = header;
            this.magnetic_field = magnetic_field;
            this.magnetic_field_covariance = magnetic_field_covariance;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(magnetic_field.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(magnetic_field_covariance.Length));
            foreach(var entry in magnetic_field_covariance)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.magnetic_field.Deserialize(data, offset);
            
            var magnetic_field_covarianceArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.magnetic_field_covariance= new double[magnetic_field_covarianceArrayLength];
            for(var i =0; i <magnetic_field_covarianceArrayLength; i++)
            {
                this.magnetic_field_covariance[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }

            return offset;
        }

    }
}
