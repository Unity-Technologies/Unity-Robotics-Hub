import cv2
import math
import numpy as np

from .enums import *
from .markers_detection import extract_img_markers, draw_markers


# Image Processing
def threshold_hsv(img, list_min_v, list_max_v, reverse_hue=False, use_s_prime=False):
    """
    Take BGR image (OpenCV imread result) and return thresholded image
    according to values on HSV (Hue, Saturation, Value)
    Pixel will worth 1 if a pixel has a value between min_v and max_v for all channels
    :param img: image BGR if rgb_space = False
    :param list_min_v: list corresponding to [min_value_H,min_value_S,min_value_V]
    :param list_max_v: list corresponding to [max_value_H,max_value_S,max_value_V]
    :param use_s_prime: Bool -> True if you want to use S channel as S' = S x V else classic
    :param reverse_hue: Useful for Red color cause it is at both extremum
    :return: threshold image
    """
    frame_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    if use_s_prime:
        frame_hsv[:, :, 1] = (1. / 255) * frame_hsv[:, :, 1] * frame_hsv[:, :, 2].astype(np.uint8)

    if not reverse_hue:
        return cv2.inRange(frame_hsv, tuple(list_min_v), tuple(list_max_v))
    else:
        list_min_v_c = list(list_min_v)
        list_max_v_c = list(list_max_v)
        lower_bound_red, higher_bound_red = sorted([list_min_v_c[0], list_max_v_c[0]])
        list_min_v_c[0], list_max_v_c[0] = 0, lower_bound_red
        low_red_im = cv2.inRange(frame_hsv, tuple(list_min_v_c), tuple(list_max_v_c))
        list_min_v_c[0], list_max_v_c[0] = higher_bound_red, 179
        high_red_im = cv2.inRange(frame_hsv, tuple(list_min_v_c), tuple(list_max_v_c))
        return cv2.addWeighted(low_red_im, 1.0, high_red_im, 1.0, 0)


def morphological_transformations(im_thresh, morpho_type="CLOSE", kernel_shape=(5, 5), kernel_type="ELLIPSE"):
    """
    Take black & white image and apply morphological transformation
    :param im_thresh: Black & White Image
    :param morpho_type: CLOSE/OPEN/ERODE/DILATE => See on OpenCV/Google if you do not know these words
    :param kernel_shape: tuple corresponding to the size of the kernel
    :param kernel_type: RECT/ELLIPSE/CROSS => see on OpenCV
    :return: image after processing
    """
    if kernel_type == "CROSS":
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, kernel_shape)
    elif kernel_type == "RECT":
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, kernel_shape)
    else:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, kernel_shape)

    if morpho_type == "OPEN":
        morph_type = cv2.MORPH_OPEN
    elif morpho_type == "DILATE":
        morph_type = cv2.MORPH_DILATE
    elif morpho_type == "ERODE":
        morph_type = cv2.MORPH_ERODE
    else:
        morph_type = cv2.MORPH_CLOSE

    return cv2.morphologyEx(im_thresh, morph_type, kernel)


# Contours
def biggest_contour_finder(img):
    res = biggest_contours_finder(img, nb_contours_max=1)
    if not res:
        return res
    else:
        return res[0]


def biggest_contours_finder(img, nb_contours_max=3):
    """
    Fonction to find the biggest contour in an binary image
    :param img: Binary Image
    :param nb_contours_max: maximal number of contours which will be returned
    :return: biggest contours found
    """
    contours = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)[-2]
    if not contours:
        return None
    contours_area = list()
    for cnt in contours:
        contours_area.append(cv2.contourArea(cnt))
    biggest_contours = []
    le = len(contours_area)
    if nb_contours_max > le:
        nb_contours = le
        id_contours_sorted_init = list(range(nb_contours))
    else:
        nb_contours = nb_contours_max
        id_contours_sorted_init = np.argpartition(contours_area, -nb_contours)[-nb_contours:]
    id_contours_sorted = [x for x in sorted(id_contours_sorted_init, key=lambda idi: -contours_area[idi])]

    for i in range(nb_contours):
        id_used = id_contours_sorted[i]

        if contours_area[id_used] < 400:
            break

        biggest_contours.append(contours[id_used])
    return biggest_contours


def get_contour_barycenter(contour):
    """
    Return barycenter of an OpenCV Contour
    :param contour: OpenCV Contour
    :return: barycenter
    """
    moments = cv2.moments(contour)
    if moments['m00'] == 0:
        rect = cv2.boundingRect(contour)
        cx, cy, _, _ = rect
    else:
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

    return cx, cy


def get_contour_angle(cnt):
    """
    Return orientation of a contour
    :param cnt: contour
    :return: Angle in radians
    """
    rotrect = cv2.minAreaRect(cnt)
    angle = rotrect[-1]
    size1, size2 = rotrect[1][0], rotrect[1][1]
    ratio_size = float(size1) / float(size2)
    if 1.25 > ratio_size > 0.75:
        if angle < -45:
            angle = 90 + angle
    else:
        if size1 < size2:
            angle = angle + 180
        else:
            angle = angle + 90

        if angle > 90:
            angle = angle - 180

    return math.radians(angle)


def draw_contours(img, contours):
    """
    Draw a list of contour on an image and return the drawing image
    :param img: Image
    :param contours: contours list
    :return: Image with drawing
    """
    if len(img.shape) == 2:
        img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        img_bgr = img.copy()
    cv2.drawContours(img_bgr, contours, -1, (255, 0, 0), 3)
    return img_bgr


def extract_img_workspace(img, workspace_ratio=1.0):
    """
    Extract working area from an image thanks to 4 Niryo's markers
    :param img: OpenCV image which contain 4 Niryo's markers
    :param workspace_ratio: Ratio between the width and the height of the area represented by the markers
    :return: extracted and warped working area image
    """
    return extract_img_markers(img, workspace_ratio)


def relative_pos_from_pixels(image_work, x_pixels, y_pixels):
    """
    Transform a pixels position to a relative position
    :param image_work: Image where the object is detected
    :param x_pixels: coordinate X
    :param y_pixels: coordinate Y
    :return:
    """
    return float(x_pixels) / image_work.shape[1], float(y_pixels) / image_work.shape[0]


# - SHOW FUNCTIONS
keys_leave = [27, ord('q')]


def show_and_check_close(window_name, image):
    """
    Display an image and check whether the user want to close
    :param window_name: window's name
    :param image: Image
    :return: boolean indicating if the user wanted to leave
    """
    cv2.imshow(window_name, image)
    return cv2.waitKey(1) in keys_leave


def show_img(window_name, image, wait_ms=1):
    """
    Display an image during a certain time
    :param window_name: window's name
    :param image: Image
    :param wait_ms: Wait time in milliseconds
    :return: value of the key pressed during the display
    """
    if type(wait_ms) == float:
        wait_ms = int(wait_ms)
    if wait_ms < 1:
        wait_ms = 1
    cv2.imshow(window_name, image)
    return cv2.waitKey(wait_ms)


def show_and_wait_close(window_name, image):
    """
    Display an image and wait that the user close it
    :param window_name: window's name
    :param image: Image
    :return: None
    """
    cv2.imshow(window_name, image)
    cv2.waitKey(0)


# -- IMAGE EDITING
def compress_image(img, quality=90):
    """
    Compress OpenCV image
    :param img: OpenCV Image
    :param quality: integer between 1 - 100. The higher it is, the less information will be lost,
    but the heavier the compressed image will be
    :return: string representing compressed image
    """
    result, encimg = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not result:
        return False, None

    return True, np.array(encimg).tostring()


def uncompress_image(compressed_image):
    """
    Take a compressed img and return an OpenCV image
    :param compressed_image: compressed image
    :return: OpenCV image
    """
    np_arr = np.fromstring(compressed_image, np.uint8)
    return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


def add_annotation_to_image(img, text, write_on_top=True):
    """
    Add Annotation to an image
    :param img: Image
    :param text: text string
    :param write_on_top: if you write the text on top
    :return: img with text written on it
    """
    font_scale_used = font_scale_normal
    thickness_used = thickness_small
    h_im, w_im = img.shape[:2]
    (text_width, text_height), baseline = cv2.getTextSize(text, font, fontScale=font_scale_used,
                                                          thickness=thickness_used)
    text_true_height = text_height + baseline
    if write_on_top:
        cv2.rectangle(img, (0, 0),
                      (int(text_width * 1.1), int(text_true_height * 1.35)),
                      WHITE, cv2.FILLED)
        cv2.putText(img, text,
                    (int(text_width * 0.05), int(text_true_height * 1.2 - baseline)),
                    font, font_scale_used, ORANGE, thickness_used)
    else:
        cv2.rectangle(img,
                      (int(text_width * 1.1), h_im - int(text_true_height * 1.35)),
                      (0, h_im),
                      WHITE, cv2.FILLED)
        cv2.putText(img, text,
                    (int(text_width * 0.05), h_im - baseline),
                    font, font_scale_used, ORANGE, thickness_used)
    return img


def undistort_image(img, mtx, dist, newcameramtx=None):
    """
    Use camera intrinsics to undistort raw image
    :param img: Raw Image
    :param mtx: Camera Intrinsics matrix
    :param dist: Distortion Coefficient
    :param newcameramtx: Camera Intrinsics matrix after correction
    :return: Undistorted image
    """
    return cv2.undistort(src=img, cameraMatrix=mtx,
                         distCoeffs=dist, newCameraMatrix=newcameramtx)


def resize_img(image, width=None, height=None, inter=cv2.INTER_AREA):
    """
    Resize an image. The user should precise only width or height if he wants to keep image's ratio
    :param image: OpenCV Image
    :param width: Target Width
    :param height: Target Height
    :param inter: OpenCV interpolation flag
    :return: resized image
    """
    height_init, width_init = image.shape[:2]
    if width is None and height is None:
        return image

    if width is None:
        ratio = height / float(height_init)
        dim = (int(width_init * ratio), height)

    elif height is None:
        ratio = width / float(width_init)
        dim = (width, int(height_init * ratio))
    else:
        dim = (width, height)

    resized = cv2.resize(image, dim, interpolation=inter)

    return resized


def concat_imgs(tuple_imgs, axis=1):
    """
    # Concat multiple images along 1 axis
    :param tuple_imgs: tuple of images
    :param axis: 0 means vertically and 1 means horizontally
    :return: Concat image
    """
    new_list_imgs = []
    for image in tuple_imgs:
        if len(image.shape) == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        new_list_imgs.append(image)

    concat_im = np.concatenate(tuple(new_list_imgs), axis=axis)
    return concat_im


# ROS

def extract_img_from_ros_msg(ros_msg):
    """
    Take a ROS CompressedImage message and return the image uncompressed
    :param ros_msg: a ROS CompressedImag
    :return: image uncompressed
    """
    data_image = ros_msg.data
    if not data_image:
        return None
    image = uncompress_image(data_image)
    return image


# DEBUG

def debug_threshold_color(img, color_str):
    """
    Return masked image to see the effect of color threshold
    :param img: OpenCV image
    :param color_str: String corresponding to color
    :return: Masked image
    """
    if color_str not in ColorHSV.__members__:
        color_str = "ANY"
    param_thresh = ColorHSV[color_str].value
    thresh_img = threshold_hsv(img, *param_thresh)
    im_morph = morphological_transformations(thresh_img, "OPEN", kernel_shape=(7, 7))
    return cv2.bitwise_and(img, img, mask=im_morph)


def debug_markers(img, workspace_ratio=1.0):
    """
    Display detected markers on an image
    :param img: OpenCV image which contain Niryo's markers
    :param workspace_ratio: Ratio between the width and the height of the area represented by the markers
    :return: (status, annotated image)
    """
    return draw_markers(img, workspace_ratio)
