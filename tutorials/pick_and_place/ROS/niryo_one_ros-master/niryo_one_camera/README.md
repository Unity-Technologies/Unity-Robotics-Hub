# Niryo One Vision
This package handles the image processing part of the Niryo One.
It's made from several publishers and services to ease the camera's use

## Setup
The setup happens in _niryo_one_camera/config/_. In this folder, you must have a
- _video_server_setup.yaml_ file which will contains several parameters has the frame rate use or the camera port.
- _XXXXXX.yaml_ file which represents camera intrinsic parameters. The name of the later should be mentionned
in the field **obj_calib_name** of the file setup file _video_server_setup.yaml_


## Launch
To launch the vision node, you can use the command :
`roslaunch niryo_one_bringup vision_node.launch`

Note that if you want to run it locally, you can use the following launch file : 
```xml
<launch>
  <node name="niryo_one_vision" pkg="niryo_one_camera" type="camera_publisher_and_services.py" output="screen">
      <rosparam file="$(find niryo_one_camera)/config/video_server_setup_local.yaml"/>
  </node>
</launch>
```

## Topic list

Publishers list
- _/niryo_one_vision/compressed_video_stream_ : Publish CompressedImage messages
each time a image is read and decoded. The actualization rate will vary according
to the frame rate and the subsampling rate

Services list
- _/niryo_one_vision/obj_detection_rel_ : Gives a relative position of a 
specify object. Use ObjDetection srv
- _/niryo_one_vision/set_calibration_camera_ : Set new camera intrinsics object.
 Use SetCalibrationCam srv
- _/niryo_one_vision/get_calibration_camera_ : Get infos about camera intrinsics parameters.
Use GetCalibrationCam srv
- _/niryo_one_vision/take_picture_ : Save the last image seen in a specified path. Use TakePicture srv
- _/niryo_one_vision/start_stop_video_streaming_ : Start and stop video streaming. Use SetBool from std_srvs