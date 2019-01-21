import cv2
import time
from networktables import NetworkTables
from grip import GripPipeline

def find_length(picture):


    top_left_corner = []

    for contour in GripPipeline().convex_hulls_output:
        # Find the centers of mass of the contours
        # https://docs.opencv.org/3.4.2/dd/d49/tutorial_py_contour_features.html
        x, y, w, h = cv2.boundingRect(contour)
        top_left_corner.append(x)

    if len(top_left_corner) == 2:
        distance = top_left_corner[0] - top_left_corner[1]
        return distance

    return 0

KNOWN_DISTANCE = 32
KNOWN_WIDTH = 11.3104
image = cv2.imread("/reference_pic.jpg")
reference = find_length(image)
focal_length = (reference * KNOWN_DISTANCE) / KNOWN_WIDTH

def distance_to_camera(knownWidth, focalLength, perWidth):

    return (knownWidth * focalLength) / perWidth

def publish_contours(pipeline):

    """
    Performs extra processing on the pipeline's outputs and publishes data to NetworkTables.
    :param pipeline: The pipeline that just processed an image
    :return: None
    """

    top_left_corner = []

    # Find the bounding boxes of the contours to get x, y, width, and height
    for contour in pipeline.convex_hulls_output:
        # Find the centers of mass of the contours
        # https://docs.opencv.org/3.4.2/dd/d49/tutorial_py_contour_features.html
        x, y, w, h = cv2.boundingRect(contour)
        top_left_corner.append(x)

    # Calculate the center between two contours (i.e. half the distance between the two contours)
    distance = 0

    if len(top_left_corner) == 2:
        width = top_left_corner[0] - top_left_corner[1]
        distance = distance_to_camera(KNOWN_WIDTH, focal_length, width) / 12


    print(str(distance) + ' ft')

