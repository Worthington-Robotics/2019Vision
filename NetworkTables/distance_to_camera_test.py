"""
Description: Script that uses a generated GRIP pipeline to calculate contours and publish results to NetworkTables.
Authors: FRC Team 4145
Comments: Make sure to update all IP addresses and paths to correspond with the correct robot.
"""

import cv2
import time
from networktables import NetworkTables
from grip import GripPipeline
from math import sin

#TODO: Update IP addresses and paths
VIDEO_STREAM_URL = "http://192.168.1.2/mjpg/video.mjpg"
ROBORIO_IP = '192.168.1.5'

VISION_TABLE = "vision"
CENTER_X = "centerX"
CENTER_Y = "centerY"

def distance_to_camera(knownWidth, focalLength, imageWidth):
    return knownWidth * focalLength / imageWidth

def publish_contours(pipeline):
    """
    Performs extra processing on the pipeline's outputs and publishes data to NetworkTables.
    :param pipeline: The pipeline that just processed an image
    :return: None
    """

    contour_x_positions = []
    contour_y_positions = []

    # Find the bounding boxes of the contours to get x, y, width, and height
    for contour in pipeline.convex_hulls_output:
        # Find the centers of mass of the contours
        # https://docs.opencv.org/3.4.2/dd/d49/tutorial_py_contour_features.html

        moments = cv2.moments(contour)
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
        contour_x_positions.append(cx)
        contour_y_positions.append(cy)

    # Calculate the distance from the camera to the tape
    distance = -1
    table = NetworkTables.getTable(VISION_TABLE)

    if (len(contour_x_positions) == 2 and len(contour_y_positions) == 2):

        angle = NetworkTables.getEntry('angle')
        KNOWN_DISTANCE = 43.5
        KNOWN_WIDTH = 11.3104 * sin(angle)
        focalLength = KNOWN_DISTANCE * 82 / KNOWN_WIDTH
        length = contour_x_positions[1] - contour_x_positions[0]
        distance = distance_to_camera(KNOWN_WIDTH, focalLength, length)
        
        return distance


def connect():
    """
    Connects to video stream and Network Tables.  Returns the video stream caputure.
    :return: capture
    """

    # Wait for connection to RoboRIO and video stream
    connected = False
    while not connected:
        try:
            # Initialize Network Tables - Address of RoboRIO
            print('\nConnecting to Network Tables...')
            NetworkTables.initialize(server=ROBORIO_IP)

            # Initialize Video Capture
            print('\nConnecting to Video Stream...')

            # USB Camera - Port of camera starting at 0
            capture = cv2.VideoCapture(0)

            # IP Camera - Address of video stream
            # capture = cv2.VideoCapture(VIDEO_STREAM_URL)

            connected = capture.isOpened()
        except:
            connected = False

    print('\nConneted to Video Stream')

    return capture


def processStream(pipeline, capture):
    """
    Processes each frame of the video capture while the video stream is open.
    :param pipeline: The GRIP generated pipeline
    :param capture: The video stream capture
    :return: None
    """

    # Loop while the video stream is open
    while capture.isOpened():
        start = time.time()
        have_frame, frame = capture.read()

        # Process the image and publish to network table
        if have_frame:
            pipeline.process(frame)
            publish_contours(pipeline)

            end = time.time()
            print('elapsed: ' + str(end - start) + '\n')


def main():

    # Initialize pipeline from genereated GRIP code
    pipeline = GripPipeline()
    loop = True
    while loop:
        capture = connect()
        try:
            processStream(pipeline, capture)
        except KeyboardException:
            loop = False
        except Exception as e:
            print('Error processing video stream.\n')
            print(e)

        capture.release()
        print('Disconnected from video stream.\n')


if __name__ == '__main__':
    main()
