using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;

namespace RosMessageTypes.Stereo
{
    public class DisparityImage : Message
    {
        public const string RosMessageName = "stereo_msgs/DisparityImage";

        //  Separate header for compatibility with current TimeSynchronizer.
        //  Likely to be removed in a later release, use image.header instead.
        public Header header { get; set; }
        //  Floating point disparity image. The disparities are pre-adjusted for any
        //  x-offset between the principal points of the two cameras (in the case
        //  that they are verged). That is: d = x_l - x_r - (cx_l - cx_r)
        public Image image { get; set; }
        //  Stereo geometry. For disparity d, the depth from the camera is Z = fT/d.
        public float f { get; set; }
        //  Focal length, pixels
        public float T { get; set; }
        //  Baseline, world units
        //  Subwindow of (potentially) valid disparity values.
        public RegionOfInterest valid_window { get; set; }
        //  The range of disparities searched.
        //  In the disparity image, any disparity less than min_disparity is invalid.
        //  The disparity search range defines the horopter, or 3D volume that the
        //  stereo algorithm can "see". Points with Z outside of:
        //      Z_min = fT / max_disparity
        //      Z_max = fT / min_disparity
        //  could not be found.
        public float min_disparity { get; set; }
        public float max_disparity { get; set; }
        //  Smallest allowed disparity increment. The smallest achievable depth range
        //  resolution is delta_Z = (Z^2/fT)*delta_d.
        public float delta_d { get; set; }

        public DisparityImage()
        {
            this.header = new Header();
            this.image = new Image();
            this.f = 0.0f;
            this.T = 0.0f;
            this.valid_window = new RegionOfInterest();
            this.min_disparity = 0.0f;
            this.max_disparity = 0.0f;
            this.delta_d = 0.0f;
        }

        public DisparityImage(Header header, Image image, float f, float T, RegionOfInterest valid_window, float min_disparity, float max_disparity, float delta_d)
        {
            this.header = header;
            this.image = image;
            this.f = f;
            this.T = T;
            this.valid_window = valid_window;
            this.min_disparity = min_disparity;
            this.max_disparity = max_disparity;
            this.delta_d = delta_d;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(image.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.f));
            listOfSerializations.Add(BitConverter.GetBytes(this.T));
            listOfSerializations.AddRange(valid_window.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.min_disparity));
            listOfSerializations.Add(BitConverter.GetBytes(this.max_disparity));
            listOfSerializations.Add(BitConverter.GetBytes(this.delta_d));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.image.Deserialize(data, offset);
            this.f = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.T = BitConverter.ToSingle(data, offset);
            offset += 4;
            offset = this.valid_window.Deserialize(data, offset);
            this.min_disparity = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.max_disparity = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.delta_d = BitConverter.ToSingle(data, offset);
            offset += 4;

            return offset;
        }

    }
}
