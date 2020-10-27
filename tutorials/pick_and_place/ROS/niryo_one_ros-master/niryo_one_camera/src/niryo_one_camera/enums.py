from enum import Enum, unique
from cv2 import FONT_HERSHEY_SIMPLEX

# -- Text Display
RED = (0, 0, 255)
PURPLE = (255, 0, 255)
ORANGE = (0, 127, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
font = FONT_HERSHEY_SIMPLEX
font_scale_big = 2.0
font_scale_normal = 0.7
thickness_big = 3
thickness_small = 2


# Command list
@unique
class Command(Enum):
    GET_OBJECT_POS = 1
    GET_OBJECT_RELATIVE_POS = 2


# Values for HSV threshold
@unique
class ColorHSV(Enum):
    """
    MIN HSV, MAX HSV, Invert Hue (bool)
    """
    BLUE = [90, 50, 85], [125, 255, 255], False
    RED = [15, 80, 75], [170, 255, 255], True
    GREEN = [40, 60, 75], [85, 255, 255], False
    ANY = [0, 50, 100], [179, 255, 255], False


# Values use for HS'V thresholding
@unique
class ColorHSVPrime(Enum):
    """
    MIN HSV, MAX HSV, Invert Hue (bool)
    """
    BLUE = [90, 70, 100], [115, 255, 255], False
    RED = [15, 70, 100], [170, 255, 255], True
    GREEN = [40, 70, 100], [85, 255, 255], False
    ANY = [0, 70, 140], [179, 255, 255], False


# Different object types handle by the API. The value corresponds to the object's number of sides
@unique
class ObjectType(Enum):
    SQUARE = 4
    CIRCLE = -1
    ANY = 0
