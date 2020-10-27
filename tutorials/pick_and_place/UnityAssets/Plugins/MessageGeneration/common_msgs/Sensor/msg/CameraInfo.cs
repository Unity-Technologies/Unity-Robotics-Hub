using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Sensor
{
    public class CameraInfo : Message
    {
        public const string RosMessageName = "sensor_msgs/CameraInfo";

        //  This message defines meta information for a camera. It should be in a
        //  camera namespace on topic "camera_info" and accompanied by up to five
        //  image topics named:
        // 
        //    image_raw - raw data from the camera driver, possibly Bayer encoded
        //    image            - monochrome, distorted
        //    image_color      - color, distorted
        //    image_rect       - monochrome, rectified
        //    image_rect_color - color, rectified
        // 
        //  The image_pipeline contains packages (image_proc, stereo_image_proc)
        //  for producing the four processed image topics from image_raw and
        //  camera_info. The meaning of the camera parameters are described in
        //  detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.
        // 
        //  The image_geometry package provides a user-friendly interface to
        //  common operations using this meta information. If you want to, e.g.,
        //  project a 3d point into image coordinates, we strongly recommend
        //  using image_geometry.
        // 
        //  If the camera is uncalibrated, the matrices D, K, R, P should be left
        //  zeroed out. In particular, clients may assume that K[0] == 0.0
        //  indicates an uncalibrated camera.
        // ######################################################################
        //                      Image acquisition info                          #
        // ######################################################################
        //  Time of image acquisition, camera coordinate frame ID
        public Header header { get; set; }
        //  Header timestamp should be acquisition time of image
        //  Header frame_id should be optical frame of camera
        //  origin of frame should be optical center of camera
        //  +x should point to the right in the image
        //  +y should point down in the image
        //  +z should point into the plane of the image
        // ######################################################################
        //                       Calibration Parameters                         #
        // ######################################################################
        //  These are fixed during camera calibration. Their values will be the #
        //  same in all messages until the camera is recalibrated. Note that    #
        //  self-calibrating systems may "recalibrate" frequently.              #
        //                                                                      #
        //  The internal parameters can be used to warp a raw (distorted) image #
        //  to:                                                                 #
        //    1. An undistorted image (requires D and K)                        #
        //    2. A rectified image (requires D, K, R)                           #
        //  The projection matrix P projects 3D points into the rectified image.#
        // ######################################################################
        //  The image dimensions with which the camera was calibrated. Normally
        //  this will be the full camera resolution in pixels.
        public uint height { get; set; }
        public uint width { get; set; }
        //  The distortion model used. Supported models are listed in
        //  sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
        //  simple model of radial and tangential distortion - is sufficient.
        public string distortion_model { get; set; }
        //  The distortion parameters, size depending on the distortion model.
        //  For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
        public double[] D { get; set; }
        //  Intrinsic camera matrix for the raw (distorted) images.
        //      [fx  0 cx]
        //  K = [ 0 fy cy]
        //      [ 0  0  1]
        //  Projects 3D points in the camera coordinate frame to 2D pixel
        //  coordinates using the focal lengths (fx, fy) and principal point
        //  (cx, cy).
        public double[] K { get; set; }
        //  3x3 row-major matrix
        //  Rectification matrix (stereo cameras only)
        //  A rotation matrix aligning the camera coordinate system to the ideal
        //  stereo image plane so that epipolar lines in both stereo images are
        //  parallel.
        public double[] R { get; set; }
        //  3x3 row-major matrix
        //  Projection/camera matrix
        //      [fx'  0  cx' Tx]
        //  P = [ 0  fy' cy' Ty]
        //      [ 0   0   1   0]
        //  By convention, this matrix specifies the intrinsic (camera) matrix
        //   of the processed (rectified) image. That is, the left 3x3 portion
        //   is the normal camera intrinsic matrix for the rectified image.
        //  It projects 3D points in the camera coordinate frame to 2D pixel
        //   coordinates using the focal lengths (fx', fy') and principal point
        //   (cx', cy') - these may differ from the values in K.
        //  For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
        //   also have R = the identity and P[1:3,1:3] = K.
        //  For a stereo pair, the fourth column [Tx Ty 0]' is related to the
        //   position of the optical center of the second camera in the first
        //   camera's frame. We assume Tz = 0 so both cameras are in the same
        //   stereo image plane. The first camera always has Tx = Ty = 0. For
        //   the right (second) camera of a horizontal stereo pair, Ty = 0 and
        //   Tx = -fx' * B, where B is the baseline between the cameras.
        //  Given a 3D point [X Y Z]', the projection (x, y) of the point onto
        //   the rectified image is given by:
        //   [u v w]' = P * [X Y Z 1]'
        //          x = u / w
        //          y = v / w
        //   This holds for both images of a stereo pair.
        public double[] P { get; set; }
        //  3x4 row-major matrix
        // ######################################################################
        //                       Operational Parameters                         #
        // ######################################################################
        //  These define the image region actually captured by the camera       #
        //  driver. Although they affect the geometry of the output image, they #
        //  may be changed freely without recalibrating the camera.             #
        // ######################################################################
        //  Binning refers here to any camera setting which combines rectangular
        //   neighborhoods of pixels into larger "super-pixels." It reduces the
        //   resolution of the output image to
        //   (width / binning_x) x (height / binning_y).
        //  The default values binning_x = binning_y = 0 is considered the same
        //   as binning_x = binning_y = 1 (no subsampling).
        public uint binning_x { get; set; }
        public uint binning_y { get; set; }
        //  Region of interest (subwindow of full camera resolution), given in
        //   full resolution (unbinned) image coordinates. A particular ROI
        //   always denotes the same window of pixels on the camera sensor,
        //   regardless of binning settings.
        //  The default setting of roi (all values 0) is considered the same as
        //   full resolution (roi.width = width, roi.height = height).
        public RegionOfInterest roi { get; set; }

        public CameraInfo()
        {
            this.header = new Header();
            this.height = 0;
            this.width = 0;
            this.distortion_model = "";
            this.D = new double[0];
            this.K = new double[9];
            this.R = new double[9];
            this.P = new double[12];
            this.binning_x = 0;
            this.binning_y = 0;
            this.roi = new RegionOfInterest();
        }

        public CameraInfo(Header header, uint height, uint width, string distortion_model, double[] D, double[] K, double[] R, double[] P, uint binning_x, uint binning_y, RegionOfInterest roi)
        {
            this.header = header;
            this.height = height;
            this.width = width;
            this.distortion_model = distortion_model;
            this.D = D;
            this.K = K;
            this.R = R;
            this.P = P;
            this.binning_x = binning_x;
            this.binning_y = binning_y;
            this.roi = roi;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.height));
            listOfSerializations.Add(BitConverter.GetBytes(this.width));
            listOfSerializations.Add(SerializeString(this.distortion_model));
            
            listOfSerializations.Add(BitConverter.GetBytes(D.Length));
            foreach(var entry in D)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(K.Length));
            foreach(var entry in K)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(R.Length));
            foreach(var entry in R)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(P.Length));
            foreach(var entry in P)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            listOfSerializations.Add(BitConverter.GetBytes(this.binning_x));
            listOfSerializations.Add(BitConverter.GetBytes(this.binning_y));
            listOfSerializations.AddRange(roi.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            this.height = BitConverter.ToUInt32(data, offset);
            offset += 4;
            this.width = BitConverter.ToUInt32(data, offset);
            offset += 4;
            var distortion_modelStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.distortion_model = DeserializeString(data, offset, distortion_modelStringBytesLength);
            offset += distortion_modelStringBytesLength;
            
            var DArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.D= new double[DArrayLength];
            for(var i =0; i <DArrayLength; i++)
            {
                this.D[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }
            
            var KArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.K= new double[KArrayLength];
            for(var i =0; i <KArrayLength; i++)
            {
                this.K[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }
            
            var RArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.R= new double[RArrayLength];
            for(var i =0; i <RArrayLength; i++)
            {
                this.R[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }
            
            var PArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.P= new double[PArrayLength];
            for(var i =0; i <PArrayLength; i++)
            {
                this.P[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }
            this.binning_x = BitConverter.ToUInt32(data, offset);
            offset += 4;
            this.binning_y = BitConverter.ToUInt32(data, offset);
            offset += 4;
            offset = this.roi.Deserialize(data, offset);

            return offset;
        }

    }
}
