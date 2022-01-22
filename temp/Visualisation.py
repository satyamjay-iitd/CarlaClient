import numpy as np
import cv2
from matplotlib import pyplot as plt

homography_matrix_inverse = np.array([[4.35352296e-01, -8.25814651e-01, 1.43951177e+02],
                                      [-3.22056352e-04, -4.14220012e-01, 1.50649060e+02],
                                      [-1.37279185e-06, -3.23614908e-03, 1.00000000e+00]])


def draw_lane_lines(warped_binary_image, undistorted_image, output):

    left_fit = output.b1
    right_fit = output.b2
    left_radius = output.left_curve
    right_radius = output.right_curve
    lane_deviation = output.deviation

    # Create a blank image to draw the lines on
    warp_zero = np.zeros_like(warped_binary_image).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    plot_y = np.linspace(0, warped_binary_image.shape[0]-1, warped_binary_image.shape[0])

    left_fit_x = left_fit[0] * plot_y ** 2 + left_fit[1] * plot_y + left_fit[2]
    right_fit_x = right_fit[0] * plot_y ** 2 + right_fit[1] * plot_y + right_fit[2]

    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = np.array([np.transpose(np.vstack([left_fit_x, plot_y]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fit_x, plot_y])))])
    pts = np.hstack((pts_left, pts_right))

    # Draw the lane onto the warped blank image
    cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    unwarp = cv2.warpPerspective(color_warp, homography_matrix_inverse, (undistorted_image.shape[1], undistorted_image.shape[0]))
    # Combine the result with the original image
    result = cv2.addWeighted(undistorted_image, 1, unwarp, 0.3, 0)

    # Write text on image
    curvature_text = "Curvature: Left = " + str(np.round(left_radius, 2)) + ", Right = " + str(
        np.round(right_radius, 2))
    font = cv2.FONT_HERSHEY_TRIPLEX
    cv2.putText(result, curvature_text, (30, 60), font, 0.5, (0, 255, 0), 2)
    deviation_text = "Lane deviation from center = {:.2f} m".format(lane_deviation)
    font = cv2.FONT_HERSHEY_TRIPLEX
    cv2.putText(result, deviation_text, (30, 90), font, 0.5, (0, 255, 0), 2)

    return result


def plot_image(original_image, processed_image):
    f, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 9))
    f.tight_layout()
    ax1.imshow(original_image)
    ax1.set_title('Original Image', fontsize=50)
    ax2.imshow(processed_image, cmap='gray')
    ax2.set_title('Processed Image', fontsize=50)
    plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)
    plt.show()
