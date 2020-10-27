
class JevoisModule(object):

    def __init__(self, name, fps, res_x, res_y,
                video_format, should_start_guvcview):
        self.name = name

        # Video params
        self.fps = fps
        self.res_x = res_x
        self.res_y = res_y
        self.video_format = video_format
        self.should_start_guvcview = should_start_guvcview

    # To override
    # Params to load on Jevois camera via serial
    # Return an array of commands to send
    def get_serial_params(self):
        raise NotImplementedError()
