from flask import Flask

flask_app = Flask(__name__)

from niryo_one_rpi.wifi import flask_views
