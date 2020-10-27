from niryo_one_msgs.srv import ObjDetectionResponse
from niryo_one_msgs.msg import ObjectPose

from ..image_functions import *
from ..enums import *


class ObjectDetector:
    def __init__(self, obj_type, obj_color, workspace_ratio=1.0, ret_image_bool=False):
        self._img = None
        self._im_thresh = None

        self._obj_type = obj_type
        self._nb_sides = obj_type.value
        self._obj_color = obj_color
        self._ret_image_bool = ret_image_bool

        self._workspace_ratio = workspace_ratio

        self._p_init = None
        self._draw_method = 2

        self._draw_marker_size = 0
        self._draw_marker_thickness = 0
        self._draw_text_thickness = 0

    def actualize_img(self, img):
        # Setting the new image
        self._img = img
        # Adjusting drawing values to fit the image size. These values are totally empirical
        h, w = img.shape[:2]
        self._draw_marker_size = 9 if w > 499 else 4
        self._draw_marker_thickness = 5 if w > 499 else 3

        self._draw_text_thickness = 2 if w > 499 else 1

    def get_img(self):
        return self._img

    def actualize_im_thresh(self, im_thresh):
        self._im_thresh = im_thresh

    def get_im_thresh(self):
        return self._im_thresh

    def get_hsv_parameters(self):
        list_min_hsv, list_max_hsv, reverse_hue = self._obj_color.value
        return list_min_hsv, list_max_hsv, reverse_hue

    def should_ret_image(self):
        return self._ret_image_bool

    def set_nb_sides(self, nb_sides):
        self._nb_sides = nb_sides

    def extract_object_with_hsv(self, img):
        """
        Execute object detection pipeline using parameters written in object_detector
        :param img: OpenCV image from Webcam Stream
        :param self: ObjectDetector object
        :return: status, ROS message of type ObjectPose, annotated image if give_image is True else None
        """
        # Init values
        msg_res_pos = ObjectPose()

        # Extract working area image from markers
        im_work = extract_img_workspace(img, workspace_ratio=self._workspace_ratio)
        if im_work is None:  # Case where working area is not found
            status = ObjDetectionResponse.MARKERS_NOT_FOUND
            return status, msg_res_pos, "", "", img

        # Generating threshold image with HSV
        im_thresh = self.image_preprocess_with_hsv(im_work, use_s_prime=False)

        # Extracting biggest blob from image. This will correspond to the object
        cx, cy, angle, obj_type, obj_color, im_draw = self.extract_biggest_blob(im_thresh)

        if cx is None:
            status = ObjDetectionResponse.OBJECT_NOT_FOUND
        else:
            x_rel, y_rel = self.relative_pos_from_pixels(cx, cy)
            # Filling message
            msg_res_pos.x = x_rel
            msg_res_pos.y = y_rel
            msg_res_pos.yaw = angle

            status = ObjDetectionResponse.SUCCESSFUL

        if self.should_ret_image() and status == ObjDetectionResponse.SUCCESSFUL:
            # im_draw = cv2.resize(im_draw, dsize=(450, 450))
            list_pos = [msg_res_pos.x, msg_res_pos.y, msg_res_pos.z]
            im_draw = get_annotated_image_rel_pos(im_draw, list_pos, angle)

        return status, msg_res_pos, obj_type, obj_color, im_draw

    def image_preprocess_with_hsv(self, im_work, use_s_prime=False):
        """

        :param im_work:
        :param use_s_prime: Bool -> True if you want to use S channel as S' = S x V else classic
        :return:
        """
        self.actualize_img(im_work)
        list_min_hsv, list_max_hsv, reverse_hue = self.get_hsv_parameters()
        im_thresh = threshold_hsv(im_work, list_min_hsv, list_max_hsv, reverse_hue=reverse_hue, use_s_prime=use_s_prime)
        im_morph = morphological_transformations(im_thresh, "OPEN", kernel_shape=(7, 7))
        self.actualize_im_thresh(im_morph)
        return im_morph

    def relative_pos_from_pixels(self, x_pixels, y_pixels):
        return float(x_pixels) / self._img.shape[1], float(y_pixels) / self._img.shape[0]

    def extract_biggest_blob(self, im_thresh):
        """
        Function to extract the biggest shape from a threshed image.
        :return: X,Y coordinates of the center / Rotation angle / IM

        Multiple drawing methods:
            - 1 : Draw on Threshold image
            - 2 : Draw on BGR
            - 3 : Draw on BGR with mask
        """
        cx, cy = None, None
        angle = 0
        found_something = False

        if not self._ret_image_bool or self._draw_method == 2:
            im_ret = self._img
            thickness_contours = 3
        elif self._draw_method == 3:
            im_ret = cv2.bitwise_and(self._img, self._img, mask=self._im_thresh)
            thickness_contours = 2
        else:
            im_ret = cv2.cvtColor(self._im_thresh, cv2.COLOR_GRAY2BGR)
            thickness_contours = 2

        best_cnts = biggest_contours_finder(im_thresh)
        if best_cnts is None:
            return None, None, None, "", "", im_ret
        best_cnt = None
        obj_type = ObjectType.ANY

        for best_cnt in best_cnts:
            cx, cy = get_contour_barycenter(best_cnt)
            if self._nb_sides is not None:
                peri = cv2.arcLength(best_cnt, True)
                approx = cv2.approxPolyDP(best_cnt, 0.035 * peri, True)

                # ANY CASE
                if self._obj_type == ObjectType.ANY:
                    if len(approx) > 5:
                        obj_type = ObjectType.CIRCLE
                    else:
                        obj_type = ObjectType.SQUARE
                        angle = get_contour_angle(best_cnt)

                    found_something = True
                    break
                else:
                    obj_type = self._obj_type
                    # Circle Case
                    if self._obj_type == ObjectType.CIRCLE and len(approx) > 5:
                        found_something = True
                        break

                    elif len(approx) == self._nb_sides:
                        angle = get_contour_angle(best_cnt)
                        found_something = True
                        break
        if not found_something:
            return None, None, None, "", "", im_ret

        # Getting color
        colors_representation = np.mean(self._img[cy - 3:cy + 3, cx - 3:cx + 3], axis=(0, 1))
        most_present_channel = np.argmax(colors_representation)

        obj_color = ["BLUE", "GREEN", "RED"][most_present_channel]

        if self._ret_image_bool and cx is not None:
            p_init = self._p_init
            if p_init is None:
                h_im, w_im = im_ret.shape[:2]
                p_init = (w_im // 2, h_im // 2)
            cv2.drawContours(im_ret, [best_cnt], 0, PURPLE, thickness_contours)
            cv2.arrowedLine(im_ret, p_init, (cx, cy), BLUE, thickness=self._draw_text_thickness)

            cv2.drawMarker(im_ret, p_init, markerType=cv2.MARKER_DIAMOND,
                           markerSize=self._draw_marker_size, thickness=self._draw_marker_thickness, color=RED)
            cv2.drawMarker(im_ret, (cx, cy), markerType=cv2.MARKER_TILTED_CROSS,
                           markerSize=self._draw_marker_size, thickness=self._draw_marker_thickness, color=ORANGE)

        return cx, cy, angle, obj_type.name, obj_color, im_ret


def get_annotated_image_rel_pos(img, list_pos, angle, write_on_top=True):
    """
    Annotate image with position information
    :param img: Image
    :param list_pos: List of position [x,y,z]
    :param angle: the angle
    :param write_on_top: if text should be write on top
    :return: annotated image
    """
    x, y, _ = list_pos
    # print(x,y,z,angle)
    if x is None or x == "None":
        return img
    text = "x:{:.1f}% y:{:.1f}% Roll {:.1f}".format(100 * x, 100 * y,
                                                    angle)
    return add_annotation_to_image(img, text, write_on_top=write_on_top)
