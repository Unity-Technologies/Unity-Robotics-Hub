import time
import os
import subprocess
import rospy


class JevoisCamera:

    def __init__(self, video_devices_path, user_home_dir):
        self.guvcview = None
        self.video_devices_path = video_devices_path
        self.user_home_dir = user_home_dir
        self.video_config_path = user_home_dir + '/.config/guvcview2/'
        self.camera_device = ''

    def setup(self):
        self.camera_device = self.find_jevois_device()
        if self.camera_device == '':
            return False, 'Could not find JeVois camera device'
        return self.create_guvcview_config_file_if_missing()

    def find_jevois_device(self):
        device_list = os.listdir(self.video_devices_path)
        for device in device_list:
            name_path = self.video_devices_path + device + '/name'
            if os.path.exists(name_path):
                with open(name_path, 'r') as f:
                    camera_name = f.readline().rstrip()
                    if camera_name.find('JeVois') != -1:
                        rospy.loginfo("Found JeVois camera device!")
                        return device
        return ''

    def create_guvcview_config_file_if_missing(self):
        filename = self.video_config_path + self.camera_device
        # Check if config file exists
        if os.path.isfile(filename):
            return True, ''

        # If config file doesn't exist, create a default one

        if not os.path.exists(self.video_config_path):
            rospy.loginfo("Create guvcview config directory: " + str(self.video_config_path))
            os.makedirs(self.video_config_path)

        rospy.loginfo("Create default guvcview config file for device " + str(self.camera_device))
        try:
            with open(filename, 'w') as f:
                f.write(self.get_default_guvcview_config())
                return True, ''
        except Exception as e:
            return False, str(e)

    def set_guvcview_fps(self, fps):
        if self.camera_device == '':
            return False
        filename = self.video_config_path + self.camera_device

        with open(filename, "r+") as f:
            lines = f.readlines()
            for i, line in enumerate(lines):
                if line.startswith("fps_num="):
                    lines[i] = "fps_num=1\n"
                if line.startswith("fps_denom="):
                    lines[i] = "fps_denom=" + str(fps) + "\n"

            f.seek(0)
            text_to_write = ''.join(lines)
            f.write(text_to_write)
            f.truncate()

        return True

    def start_guvcview(self, res_x=None, res_y=None, video_format=None):
        cmd = ['guvcview', '-d', '/dev/' + self.camera_device,
        '-g', 'none', '-a', 'none']

        if (res_x is not None) and (res_y is not None):
            cmd.append('-x')
            cmd.append(str(res_x) + 'x' + str(res_y))
    
        if video_format is not None:
            cmd.append('-f')
            cmd.append(str(video_format))

        rospy.loginfo("Starting guvcview with cmd: " + ' '.join(cmd))
        self.guvcview = subprocess.Popen(cmd)

    def stop_guvcview(self):
        if self.guvcview:
            rospy.loginfo("Shutdown camera - stopping guvcview")
            self.guvcview.terminate()

    # This default config avoids getting an error when changing fps setting
    # Most of the important values will be overriden when starting guvcview
    def get_default_guvcview_config(self):
        return """#video input width
width=640
#video input height
height=480
#video input format
format=YUYV
#video input capture method
capture=mmap
#audio api
audio=none
#gui api
gui=none
#render api
render=none
#video codec [raw mjpg mpeg flv1 wmv1 mpg2 mp43 dx50 h264 vp80 theo]
video_codec=dx50
#audio codec [pcm mp2 mp3 aac ac3 vorb]
audio_codec=mp2
#profile name
profile_name=default.gpfl
#profile path
profile_path="""+ str(self.user_home_dir) +"""
#video name
video_name=my_video.mkv
#video path
video_path="""+ str(self.user_home_dir) +"""
#video sufix flag
video_sufix=1
#photo name
photo_name=my_photo.jpg
#photo path
photo_path="""+ str(self.user_home_dir) +"""
#photo sufix flag
photo_sufix=1
#fps numerator (def. 1)
fps_num=1
#fps denominator (def. 25)
fps_denom=25
#audio device index (-1 - api default)
audio_device=1
#video fx mask
video_fx=0x0
#audio fx mask
audio_fx=0x0
#OSD mask
osd_mask=0x0
"""
