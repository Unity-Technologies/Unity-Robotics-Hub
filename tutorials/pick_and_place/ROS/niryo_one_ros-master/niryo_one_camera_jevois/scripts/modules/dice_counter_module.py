from modules.jevois_module import JevoisModule


class DiceCounterModule(JevoisModule):

    def __init__(self, name):
        super(DiceCounterModule, self).__init__(
            name=name,
            fps=7.5,
            res_x=640,
            res_y=480,
            video_format='YUYV',
            should_start_guvcview=True)

    def get_serial_params(self):
        return ['setpar serout USB']
