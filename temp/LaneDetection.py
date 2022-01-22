import time

import cv2
import numpy as np
import json
from LaneDetection.Threshold import combine_thresholds
from LaneDetection.Visualisation import draw_lane_lines, plot_image
from dataclasses import dataclass
from typing import Optional


@dataclass
class DeterministicInference:
    """
        Responsible for Receiving and Processing Lane Image, and process that image.
    """
    left_curve: Optional[any] = None
    right_curve: Optional[any] = None
    b1: Optional[any] = None
    b2: Optional[any] = None
    deviation: Optional[any] = 0
    is_lane_detected: bool = False
    num_left_lanes: Optional[any] = 0
    num_right_lanes: Optional[any] = 0
    steering_angle: Optional[any] = 0

    def __str__(self):
        return json.dumps(self.__dict__())

    def __dict__(self):
        d = {
            'B1': [],
            'B2': [],
            'Offset': 0,
            'IsDetLaneDetected': False,
            'NumLeftLane': -1,
            'NumRightLane': -1,
            'SteeringAngle': 0
        }
        if self.is_lane_detected:
            d['B1'] = self.b1.tolist()
            d['B2'] = self.b2.tolist()
            d['Offset'] = self.deviation
            d['IsDetLaneDetected'] = self.is_lane_detected
            d['NumLeftLane'] = self.num_left_lanes
            d['NumRightLane'] = self.num_right_lanes
            d['SteeringAngle'] = self.steering_angle
        return d


homography_matrix = np.array([[-4.04058539e-01, -1.99277316e+00, 3.58626343e+02],
                              [3.14895253e-05, -2.40580412e+00, 3.62439127e+02],
                              [2.17081999e-07, -7.80116697e-03, 1.00000000e+00]])


# Detecting edges using combine_thresholds
def edge_detect(img, draw=False):
    combined = combine_thresholds(img)
    if draw:
        plot_image(img, combined)
    return combined


# Perspective Transform
def perspective_transform(img, draw=False):
    img_size = (img.shape[1], img.shape[0])
    perspective_transformed = cv2.warpPerspective(img, homography_matrix, img_size)
    if draw:
        plot_image(img, perspective_transformed)
    return perspective_transformed


# Find lane pixels in image and store in array
def find_lane_pixels(binary_warped):
    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0)
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = int(histogram.shape[0] // 2)
    left_x_hist_base = np.argmax(histogram[:midpoint])
    right_x_hist_base = np.argmax(histogram[midpoint:]) + midpoint

    # Choosing Hyper-parameters
    # Choose the number of sliding windows
    num_windows = 12
    # Set the width of the windows +/- margin
    margin = 50
    # Set minimum number of pixels found to recenter window
    min_pix = 50

    # Set height of windows - based on num_windows above and image shape
    window_height = int(binary_warped.shape[0] // num_windows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    non_zero_y = np.array(nonzero[0])
    non_zero_x = np.array(nonzero[1])
    # Current positions to be updated later for each window in num_windows
    left_x_current = left_x_hist_base
    right_x_current = right_x_hist_base

    # Create empty lists to receive left and right lane pixel indices
    left_lane_ind = []
    right_lane_ind = []

    left_number_empty_windows = 0
    right_number_empty_windows = 0

    # Step through the windows one by one
    for window in range(num_windows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_x_left_low = left_x_current - margin
        win_x_left_high = left_x_current + margin
        win_x_right_low = right_x_current - margin
        win_x_right_high = right_x_current + margin

        # Identify the nonzero pixels in x and y within the window #
        good_left_ind = ((non_zero_y >= win_y_low) & (non_zero_y < win_y_high) &
                         (non_zero_x >= win_x_left_low) & (non_zero_x < win_x_left_high)).nonzero()[0]
        good_right_ind = ((non_zero_y >= win_y_low) & (non_zero_y < win_y_high) &
                          (non_zero_x >= win_x_right_low) & (non_zero_x < win_x_right_high)).nonzero()[0]

        # Append these indices to the lists
        left_lane_ind.append(good_left_ind)
        right_lane_ind.append(good_right_ind)

        # If you found > min_pix pixels, recenter next window on their mean position
        if len(good_left_ind) > min_pix:
            left_x_current = int(np.mean(non_zero_x[good_left_ind]))
        else:
            left_number_empty_windows += 1
        if len(good_right_ind) > min_pix:
            right_x_current = int(np.mean(non_zero_x[good_right_ind]))
        else:
            right_number_empty_windows += 1

    # Concatenate the arrays of indices (previously was a list of lists of pixels)
    left_lane_ind = np.concatenate(left_lane_ind)
    right_lane_ind = np.concatenate(right_lane_ind)

    # Extract left and right line pixel positions
    left_x = non_zero_x[left_lane_ind]
    left_y = non_zero_y[left_lane_ind]
    right_x = non_zero_x[right_lane_ind]
    right_y = non_zero_y[right_lane_ind]

    # Find dashed lane
    dashed_lane = 0
    if left_number_empty_windows > right_number_empty_windows:
        dashed_lane = -1
    if right_number_empty_windows > left_number_empty_windows:
        dashed_lane = 1

    return left_x, left_y, right_x, right_y, dashed_lane


def fit_polynomial(binary_warped, left_x, left_y, right_x, right_y, dash_lane):
    # Fit a second order polynomial to each using `np.poly_fit`
    left_fit = np.polyfit(left_y, left_x, 2)
    right_fit = np.polyfit(right_y, right_x, 2)
    plot_y = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    total_pixel = np.sum(binary_warped)

    image_size = binary_warped.shape
    y_eval = np.max(plot_y)
    # Define conversions in x and y from pixels space to meters
    y_m_per_pix = 30 / 720
    x_m_per_pix = 3.7 / 700

    # Fit new polynomials to x,y in world space
    left_fit_cr = np.polyfit(left_y * y_m_per_pix, left_x * x_m_per_pix, 2)
    right_fit_cr = np.polyfit(right_y * y_m_per_pix, right_x * x_m_per_pix, 2)

    # Calculate radius of curve
    left_curve = ((1 + (
            2 * left_fit_cr[0] * y_eval * y_m_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
        2 * left_fit_cr[0])
    right_curve = ((1 + (
            2 * right_fit_cr[0] * y_eval * y_m_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
        2 * right_fit_cr[0])

    # Calculate the intercept points at the bottom of our image
    left_intercept = left_fit[0] * image_size[0] ** 2 + left_fit[1] * image_size[0] + left_fit[2]
    right_intercept = right_fit[0] * image_size[0] ** 2 + right_fit[1] * image_size[0] + right_fit[2]

    # Calculate lane deviation from center of lane
    center = (left_intercept + right_intercept) / 2.0
    # Use intercept points to calculate the lane deviation of the vehicle
    lane_deviation = (image_size[1] / 2.0 - center)

    # Calculate steering angle
    steering_angle = 180 * (np.arctan(lane_deviation / (y_eval / 2))) / np.pi
    steering_angle = float(steering_angle)

    if lane_deviation > 15:
        steering_angle = 10
    elif lane_deviation < -15:
        steering_angle = -10

    # Calculate number of left lanes and right lanes
    num_left_lanes = -1
    num_right_lanes = -1

    if dash_lane == -1:
        num_left_lanes = 3
    elif dash_lane == 1:
        num_left_lanes = 2

    if dash_lane == -1:
        num_right_lanes = 0
    elif dash_lane == 1:
        num_right_lanes = 1

    if 2000 <= total_pixel <= 15000:
        return DeterministicInference(left_curve, right_curve, left_fit, right_fit, lane_deviation,
                                      True, num_left_lanes, num_right_lanes, steering_angle)
    else:
        return DeterministicInference(None, None, None, None, 0, False, -1, -1, 0)


def detect_lane(img, draw_only=False):
    transformed = perspective_transform(edge_detect(img))
    left_x, right_x, left_y, right_y, dashed_lane = find_lane_pixels(transformed)

    try:
        output = fit_polynomial(transformed, left_x, right_x, left_y, right_y, dashed_lane)
        if draw_only:
            return draw_lane_lines(transformed, img, output)
        else:
            return output
    except Exception:
        if draw_only:
            return img
        else:
            return DeterministicInference(None, None, None, None, 0, False, -1, -1, 0)