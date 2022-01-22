import cv2
import numpy as np

# Basic info: Canny edge detection is not used bcoz for lane detection it gave a lot of lines, edges which are not
# required

"""
Description: Returns binary image that has the pixels within given gradient threshold (using Sobel Operator) value 
for gradients in given direction
Params 
    img       : image 
    orient    : direction of gradient
    thresh_min: Max threshold for gradient value to filter pixels
    thresh_max: Max threshold for gradient value to filter pixels
Return: returns binary array same as input image size
"""

sobel_kernel = 3


def abs_sobel_thresh(img, orient='x', thresh=(0, 255)):
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # Apply x or y gradient with the OpenCV Sobel() function
    # and take the absolute value
    if orient == 'x':
        abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel))
    if orient == 'y':
        abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel))
    # Rescale back to 8 bit integer
    scaled_sobel = np.uint8(255 * abs_sobel / np.max(abs_sobel))
    # Create a binary threshold to select pixels based on gradient strength
    binary_output = np.zeros_like(scaled_sobel)
    # Here I'm using inclusive (>=, <=) thresholds, but exclusive is ok too
    binary_output[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1

    # Return the result
    return binary_output


# This function is a variant of the above defined abs_sobel_thresh
# Uses the overall gradient along x and y direction, user defined kernel size to change the size of the image
# The rest working is same as the above defined function
# NOTE: We are not using this function for edge detection

"""
Description: Returns binary image that has the pixels within given gradient threshold(using Sobel Operator) value 
for magnitude of overall gradient
Params 
    img          : image 
    sobel_kernel : size of the region in the image to calc gradient of (should be an odd number)
    mag_thresh   : tuple having min and max threshold
Return: returns binary array representing the pixels within gradient threshold
"""


def mag_thresh(img, mag=(0, 255)):
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # Take both Sobel x and y gradients
    sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    # Calculate the gradient magnitude
    gradmag = np.sqrt(sobel_x ** 2 + sobel_y ** 2)
    # Rescale to 8 bit
    scale_factor = np.max(gradmag) / 255
    gradmag = (gradmag / scale_factor).astype(np.uint8)
    # Create a binary image of ones where threshold is met, zeros otherwise
    binary_output = np.zeros_like(gradmag)
    binary_output[(gradmag >= mag[0]) & (gradmag <= mag[1])] = 1

    # Return the binary image
    return binary_output


# Define a function to threshold an image for a given range and Sobel kernel
def dir_threshold(img, thresh=(0, np.pi / 2)):
    # Grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # Calculate the x and y gradients
    sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    # Take the absolute value of the gradient direction,
    # apply a threshold, and create a binary image result
    abs_grad_dir = np.arctan2(np.absolute(sobel_y), np.absolute(sobel_x))
    binary_output = np.zeros_like(abs_grad_dir)
    binary_output[(abs_grad_dir >= thresh[0]) & (abs_grad_dir <= thresh[1])] = 1

    # Return the binary image
    return binary_output


def combine_thresholds(img):
    grad_x = abs_sobel_thresh(img, orient='x', thresh=(20, 100))
    grad_y = abs_sobel_thresh(img, orient='y', thresh=(20, 100))
    mag_binary = mag_thresh(img, mag=(30, 150))
    dir_binary = dir_threshold(img, thresh=(0.8, 1.2))
    combined = np.zeros_like(dir_binary)
    combined[((grad_x == 1) & (grad_y == 1)) | ((mag_binary == 1) & (dir_binary == 1))] = 1

    return combined


# Above functions, converted the image to grayscale so we lost valuable color info
# We need to find out a way to get that color spaces back
# HSV and HLS are some of the important color spaces(than RGB) used in image analysis
# Smarter color threshold can be designed

# This function is an experiment with different values of color and gradient to get best results

def pipeline(img):
    # Making HLS color space and making a separate channel for S
    s_thresh = (170, 255)
    sx_thresh = (20, 100)
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    s_channel = hls[:, :, 2]

    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    # Sobel x
    sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
    # Take the derivative in x
    abs_sobel_x = np.absolute(sobel_x)
    # Absolute x derivative to accentuate lines away from horizontal
    scaled_sobel = np.uint8(255 * abs_sobel_x / np.max(abs_sobel_x))

    # Threshold x gradient
    thresh_min = 20
    thresh_max = 100
    sx_binary = np.zeros_like(scaled_sobel)
    sx_binary[(scaled_sobel >= thresh_min) & (scaled_sobel <= thresh_max)] = 1

    # Threshold color channel
    s_thresh_min = 170
    s_thresh_max = 255
    s_binary = np.zeros_like(s_channel)
    s_binary[(s_channel >= s_thresh_min) & (s_channel <= s_thresh_max)] = 1

    # Stack each channel to view their individual contributions in green and blue respectively
    # This returns a stack of the two binary images, whose components you can see as different colors
    color_binary = np.dstack((np.zeros_like(sx_binary), sx_binary, s_binary)) * 255

    # Combine the two binary thresholds
    combined_binary = np.zeros_like(sx_binary)
    combined_binary[(s_binary == 1) | (sx_binary == 1)] = 1

    return combined_binary