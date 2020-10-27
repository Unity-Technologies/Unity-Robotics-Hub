import numpy as np
import cv2
from .math_functions import euclidean_dist_2_pts


def extract_img_markers(img, workspace_ratio=1.0):
    """
    Extract working area from an image thanks to 4 Niryo's markers
    :param img: OpenCV image which contain 4 Niryo's markers
    :param workspace_ratio: Ratio between the width and the height of the area represented by the markers
    :return: extracted and warped working area image
    """

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    img_thresh = cv2.adaptiveThreshold(gray, maxValue=255, adaptiveMethod=cv2.ADAPTIVE_THRESH_MEAN_C,
                                       thresholdType=cv2.THRESH_BINARY, blockSize=15, C=25)

    list_good_candidates = find_markers_from_img_thresh(img_thresh)
    if not list_good_candidates or len(list_good_candidates) > 6:
        return None

    if len(list_good_candidates) == 4:
        list_good_candidates = sort_markers_detection(list_good_candidates)
    else:
        list_good_candidates = complicated_sort_markers(list_good_candidates, workspace_ratio=workspace_ratio)
        if list_good_candidates is None:
            return None

    im_cut = extract_sub_img(img, list_good_candidates, ratio_w_h=workspace_ratio)
    return im_cut


def extract_sub_img(img, list_corners, ratio_w_h=1.0):
    """
    Extract an small image from a big one using a Perspective Warp
    :param img: Big image from which the small one will be extracted
    :param list_corners: corners list of the small image
    :param ratio_w_h: Width over Height ratio of the area. It helps to not stretch the working area image
    :return: extracted and warped image
    """
    if list_corners is None or len(list_corners) != 4:
        return None

    if ratio_w_h >= 1.0:
        target_w_area = int(round(ratio_w_h * 200))
        target_h_area = 200
    else:
        ratio_w_h = 1.0 / ratio_w_h
        target_h_area = int(round(ratio_w_h * 200))
        target_w_area = 200

    points_grid = []

    for marker in list_corners:
        points_grid.append(marker.get_center())
    points_grid = np.array(points_grid, dtype=np.float32)
    final_pts = np.array(
        [[0, 0], [target_w_area - 1, 0],
         [target_w_area - 1, target_h_area - 1], [0, target_h_area - 1]],
        dtype=np.float32)
    transfo_matrix = cv2.getPerspectiveTransform(points_grid, final_pts)
    # print transfo_matrix
    # print np.linalg.det(transfo_matrix)
    area_im = cv2.warpPerspective(img, transfo_matrix, (target_w_area, target_h_area))
    return area_im


def draw_markers(img, workspace_ratio=1.0):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    img_thresh = cv2.adaptiveThreshold(gray, maxValue=255, adaptiveMethod=cv2.ADAPTIVE_THRESH_MEAN_C,
                                       thresholdType=cv2.THRESH_BINARY, blockSize=15, C=32)

    list_good_candidates = find_markers_from_img_thresh(img_thresh)
    if not list_good_candidates:
        return False, img
    im_draw = img.copy()
    for marker in list_good_candidates:
        cx, cy = marker.get_center()
        radius = marker.get_radius()
        cv2.circle(im_draw, (cx, cy), radius, (0, 0, 255), 2)
    if len(list_good_candidates) > 6:
        return False, im_draw

    if len(list_good_candidates) == 4:
        list_good_candidates = sort_markers_detection(list_good_candidates)
    else:
        list_good_candidates = complicated_sort_markers(list_good_candidates, workspace_ratio=workspace_ratio)
        if list_good_candidates is None:
            return False, im_draw

    for i, marker in enumerate(list_good_candidates[:4]):
        cx, cy = marker.get_center()
        radius = marker.get_radius()
        cv2.circle(im_draw, (cx, cy), radius, (0, 200, 0), 2)
        cv2.putText(im_draw, "{}".format(i + 1),
                    (cx + 5, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 3)
        cv2.putText(im_draw, "{}".format(i + 1),
                    (cx + 5, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 0), 2)
    return True, im_draw


class PotentialMarker:
    def __init__(self, center, radius, cnt):
        self.center = center
        self.x = center[0]
        self.y = center[1]
        self.radius = radius
        self.contour = cnt
        self.is_merged = False

    def get_center(self):
        return self.center

    def __str__(self):
        return "{} - {} - {}".format(self.x, self.y, self.radius)

    def __repr__(self):
        return self.__str__()


class Marker:
    def __init__(self, potential_marker):
        self.list_centers = [potential_marker.get_center()]
        self.list_radius = [potential_marker.radius]
        self.list_contours = [potential_marker.contour]
        self.cx = self.list_centers[0][0]
        self.cy = self.list_centers[0][1]
        self.radius = potential_marker.radius
        self.identifiant = None
        self.value_for_id = None

    def get_radius(self):
        return self.radius

    def get_center(self):
        return self.cx, self.cy

    def add_circle(self, obj_potential_marker):
        self.list_centers.append(obj_potential_marker.get_center())
        self.list_radius.append(obj_potential_marker.radius)
        obj_potential_marker.is_merged = True

        (x, y) = np.mean(self.list_centers, axis=0)
        self.cx, self.cy = int(round(x)), int(round(y))

        self.radius = int(round(max(self.list_radius)))

    def nb_circles(self):
        return len(self.list_centers)

    def get_id_from_slice(self, img_thresh):
        x, y, w, h = self.cx - 1, self.cy - 1, 3, 3
        self.value_for_id = np.mean(img_thresh[y:y + h, x:x + w])
        # return value_for_id
        if self.value_for_id > 200:
            self.identifiant = "A"
        else:
            self.identifiant = "B"

        return self.identifiant
        # return value_for_id

    def __str__(self):
        return "{} - {}".format(self.nb_circles(), self.list_centers)

    def __repr__(self):
        return self.__str__()


def sort_markers_detection(list_markers):
    def rotate(l, n):
        return l[n:] + l[:n]

    list_sort_y = sorted(list_markers, key=lambda m: m.cy)

    top1, top2, bottom1, bottom2 = list_sort_y

    if top1.cx < top2.cx:
        top_left = top1
        top_right = top2
    else:
        top_left = top2
        top_right = top1

    if bottom1.cx < bottom2.cx:
        bottom_left = bottom1
        bottom_right = bottom2
    else:
        bottom_left = bottom2
        bottom_right = bottom1

    list_markers_unsorted = [top_left, top_right, bottom_right, bottom_left]
    list_id = [marker.identifiant for marker in list_markers_unsorted]

    if list_id.count("A") == 1:
        list_corners_sorted = rotate(list_markers_unsorted, n=list_id.index("A"))
    elif list_id.count("B") == 1:
        list_corners_sorted = rotate(list_markers_unsorted, n=list_id.index("B"))
    else:
        return list_markers_unsorted

    return list_corners_sorted


def complicated_sort_markers(list_markers, workspace_ratio):
    import itertools

    if workspace_ratio >= 1.0:
        target_w_area = int(round(workspace_ratio * 200))
        target_h_area = 200
    else:
        ratio_w_h = 1.0 / workspace_ratio
        target_h_area = int(round(ratio_w_h * 200))
        target_w_area = 200
    list_id = [marker.identifiant for marker in list_markers]
    count_A = list_id.count("A")
    count_B = list_id.count("B")
    if count_A < 3 > count_B:
        return None
    if count_A < count_B:
        id_first_marker = "A"
        id_second_marker = "B"
    else:
        id_first_marker = "B"
        id_second_marker = "A"
    list_combinaisons = []
    list_marker_1 = [marker for marker in list_markers if marker.identifiant == id_first_marker]
    list_marker_2 = [marker for marker in list_markers if marker.identifiant == id_second_marker]
    if list_marker_1:
        list_combinaisons_marker_2 = itertools.combinations(list_marker_2, 3)
        for marker1 in list_marker_1:
            for combi_markers2 in list_combinaisons_marker_2:
                combin = [marker1] + list(combi_markers2)

                list_combinaisons.append(sort_markers_detection(combin))
    else:
        for combinaison in itertools.combinations(list_marker_2, 4):
            list_combinaisons.append(combinaison)
    if not list_combinaisons:
        return None

    final_pts = np.array(
        [[0, 0], [target_w_area - 1, 0],
         [target_w_area - 1, target_h_area - 1], [0, target_h_area - 1]],
        dtype=np.float32)
    list_det_transfo_matrix = []
    for combin in list_combinaisons:
        points_grid = np.array([[mark.cx, mark.cy] for mark in combin], dtype=np.float32)

        transfo_matrix = cv2.getPerspectiveTransform(points_grid, final_pts)
        list_det_transfo_matrix.append(np.linalg.det(transfo_matrix))

    best_combin_ind = np.argmin(abs(np.array(list_det_transfo_matrix) - 1))
    best_markers = list_combinaisons[best_combin_ind]
    return best_markers


def find_markers_from_img_thresh(img_thresh, max_dist_between_centers=3, min_radius_circle=4,
                                 max_radius_circle=35, min_radius_marker=7):
    contours = cv2.findContours(img_thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
    list_potential_markers = []
    for cnt in contours:
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        if not min_radius_circle < radius < max_radius_circle:
            continue
        center = (int(round(x)), int(round(y)))
        radius = int(radius)
        list_potential_markers.append(PotentialMarker(center, radius, cnt))

    list_potential_markers = sorted(list_potential_markers, key=lambda m: m.x)
    list_good_candidates = []

    for i, potential_marker in enumerate(list_potential_markers):
        if potential_marker.is_merged:
            continue
        marker1 = Marker(potential_marker)
        center_marker = marker1.get_center()

        for potential_marker2 in list_potential_markers[i + 1:]:
            if potential_marker.is_merged:
                continue
            center_potential = potential_marker2.get_center()
            if center_potential[0] - center_marker[0] > max_dist_between_centers:
                break
            dist = euclidean_dist_2_pts(center_marker, center_potential)
            if dist <= max_dist_between_centers:
                marker1.add_circle(potential_marker2)
                center_marker = marker1.get_center()

        if marker1.nb_circles() > 2 and marker1.radius >= min_radius_marker:
            list_good_candidates.append(marker1)
            marker1.get_id_from_slice(img_thresh)

    return list_good_candidates
