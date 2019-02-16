#!/usr/bin/env python3
"""
----------------------------------------------------------------------------
Authors:     FRC Team 4145

Description: This script uses a generated GRIP pipeline to process a camera
             stream and publish results to NetworkTables.  This script is
             designed to work on the 2019 FRCVision Raspberry Pi image.

Comments:    This script should be uploaded to the Raspberry Pi using the
             FRCVision web console.  Navigate to the "Application" tab and
             select the "Uploaded Python file" option.  The grip.py script
             should be manually uploaded to the /home/pi/ directory of the
             Raspberry Pi.
----------------------------------------------------------------------------
"""

import json
import time
import sys
from math import sqrt, degrees, atan

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from networktables import NetworkTablesInstance, NetworkTables

import cv2
import numpy as np
from grip import GripPipeline


class CameraConfig:
    # Camera name
    name = None
    # Path to camera
    path = None
    # Camera config JSON
    config = None
    # Stream config JSON
    stream_config = None


class ContourData:
    def __init__(self, cx, cy, box, left):
        # X coordinate of the contour center
        self.cx = cx
        # Y coordinate of the contour center
        self.cy = cy
        # Minimum containing box of contour
        self.box = box
        # If the contour is a left or right contour
        self.left = left


# Enable/Disable Raw Camera Output
ENABLE_RAW_STREAM = False

# Enable/Disable Custom Camera Output (i.e. crosshairs on target)
ENABLE_CUSTOM_STREAM = True

# Enable Debug
ENABLE_DEBUG = False

# Camera Field of View (Measured Empirically)
CAMERA_FOV = 51.06

# Network Table constants
VISION_TABLE = "SmartDashboard/vision"
CENTER_X = "centerX"
CENTER_Y = "centerY"
ANGLE_OFFSET = "angleOffset"

# Camera config file
config_file = "/boot/frc.json"

# Camera settings
team = None
server = False
camera_configs = []
parsed_width = None
parsed_height = None


def parseError(line):
    """
    Report parse error.
    """

    print("config error in '" + config_file + "': " + line, file=sys.stderr)


def readCameraConfig(config):
    """
    Read a single camera configuration.
    """

    cam = CameraConfig()

    # Parse camera name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # Parse camera path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # Parse stream properties
    cam.stream_config = config.get("stream")

    cam.config = config

    camera_configs.append(cam)
    return True


def readConfig():
    """
    Read the configuration file.
    """

    global team
    global server

    # Parse config file
    try:
        with open(config_file, "rt") as f:
            j = json.load(f)
    except OSError as err:
        print(
            "could not open '{}': {}".format(config_file, err),
            file=sys.stderr)
        return False

    # Top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # Parse team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # Parse network table mode (server/client)
    if "ntmode" in j:
        mode = j["ntmode"]
        if mode.lower() == "client":
            server = False
        elif mode.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(mode))

    # Parse cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False

    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    return True


def startNetworkTables():
    """
    Connect to the Network Tables as a client or start the server locally.
    """

    ntinst = NetworkTablesInstance.getDefault()
    if server:
        print("Setting up NetworkTables server...")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)
        connected = False
        while (not connected):
            time.sleep(1)
            connected = ntinst.isConnected()
            print("NetworkTables Connected: " + str(connected))


def startCamera(config):
    """
    Start running the camera.
    """

    print("Starting camera '{}' on {}".format(config.name, config.path))
    inst = CameraServer.getInstance()
    camera = UsbCamera(config.name, config.path)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    # Start automatic capture stream
    if (ENABLE_RAW_STREAM):
        print("Starting Custom Output Stream...")

        camera_server = inst.startAutomaticCapture(
            camera=camera, return_server=True)

        if config.stream_config is not None:
            camera_server.setConfigJson(json.dumps(config.stream_config))

    return camera


def parseDimensions(camera_config):
    """
    Parse the width and height of the camera.
    """

    width = None
    height = None
    try:
        width = camera_config.config["width"]
        height = camera_config.config["height"]
    except KeyError:
        parseError("Could not read width/height")

    return (width, height)


def startOutputSource(width, height):
    """
    Create an output source and server to ouput custom frames.
    """

    print("Starting Custom Output Stream...")

    inst = CameraServer.getInstance()
    cv_source = inst.putVideo("grip", width, height)

    return cv_source


def readFrame(camera):
    """
    Reads the latest frame from the camera server instance to pass to opencv.
    :param camera: The camera to read from
    :return: The latest frame
    """

    inst = CameraServer.getInstance()
    cv_sink = inst.getVideo(camera=camera)

    (frame_time, frame) = cv_sink.grabFrame(None)

    return frame


def processFrame(frame, pipeline: GripPipeline):
    """
    Performs extra processing on the pipeline's outputs.
    :param pipeline: The pipeline that just processed an image
    :return: The center coordinates of the target
    """

    # Process the Grip Pipeline
    pipeline.process(frame)

    # Populate data from contours
    contour_data = calculateContourData(pipeline)

    # Find the closest target
    (center_x, center_y) = findClosestTarget(contour_data)

    return (center_x, center_y, contour_data)


def calculateContourData(pipeline):
    """
    Populate the various contour data used in future caluculations.
    """
    contour_data = []

    # Find the bounding boxes of the contours to get x, y, width, and height
    for contour in pipeline.filter_contours_output:
        # Find the centers of mass of the contours
        # https://docs.opencv.org/3.4.2/dd/d49/tutorial_py_contour_features.html

        moments = cv2.moments(contour)
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

        # Calculate if the slope of the contour edge is positive
        (box, left) = calculateBoxAndSide(contour)

        contour_data.append(ContourData(cx, cy, box, left))

    if (ENABLE_DEBUG):
        print('Found ' + str(len(contour_data)) + ' Contours')

    return contour_data


def calculateBoxAndSide(contour):
    """
    Calculate the minimum containing box of the contour and determine the side of the pair.
    (i.e. whether the contour is the left or right side of the target)
    """

    # Calculate the 4 corners of the contour
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    # Sort points by x coordinate ascending
    sorted_pts = sorted(box, key=lambda pt: pt[1])

    """
    Find first and third point from the top down (y-axis is the top of the frame)

       (x)               (x)
            x         x
                or
   (x)                       (x)
        x                 x
    """
    bottom = sorted_pts[0]
    top = sorted_pts[2]

    # Find the slope of the first contour edge
    denom = (top[0] - bottom[0])
    slope = 0
    if(denom != 0):
        slope = (top[1] - bottom[1]) / denom

    angle = degrees(atan(slope))

    # This is because the y-axis is the top of the frame
    left = angle < 0

    return ([box], left)


def findClosestTarget(contour_data):
    """
    Find the nearest pair of contours that look like: / \ 
    """

    contour_data.sort(key=lambda c: c.cx)

    # Find all pairs
    pairs = findPairs(contour_data)

    if (ENABLE_DEBUG):
        print('Found ' + str(len(pairs)) + ' Pairs')

    # Find closest pair
    (center_x, center_y) = findClosestPair(pairs)

    return (center_x, center_y)


def findPairs(contour_data):
    """
    Find all pairs of contours that look like: / \ 
    """

    pairs = []

    index = 0
    size = len(contour_data)
    done = False
    while(not done and index < size):
        if (index + 1 < size):

            # If the current contour is left and the next is right, then pair found
            if (contour_data[index].left and (not contour_data[index + 1].left)):
                pairs.append((contour_data[index], contour_data[index+1]))
                index += 1

            index += 1
        else:
            done = True

    return pairs


def findClosestPair(pairs):
    """
    Find the pair of contours closest to the center of the camera.
    """
    global parsed_width
    camera_center = parsed_width / 2

    closest = None
    (closest_x, closest_y) = (-1, -1)

    for (left, right) in pairs:
        (center_x, center_y) = calculateCenter(left, right)
        distance = abs(center_x - camera_center)

        if (closest is None or distance < closest):
            closest = distance
            (closest_x, closest_y) = (center_x, center_y)

    return (closest_x, closest_y)


def calculateCenter(contour_data1: ContourData, contour_data2: ContourData):
    """
    Calculate the midpoint between two contours.
    """

    center_x = (contour_data1.cx + contour_data2.cx) / 2.0
    center_y = (contour_data2.cy + contour_data2.cy) / 2.0

    return (center_x, center_y)


def calculateAngleOffset(center_x):
    """
    Calculates the angle offset.
    (i.e. how much the robot should turn in order to face the target)
    """

    global parsed_width
    angle_offset = -1000

    if (center_x >= 0):
        pixel_offset = (parsed_width / 2) - center_x
        angle_offset = (CAMERA_FOV / parsed_width) * pixel_offset

    return angle_offset


def publishValues(center_x, center_y, angle_offset):
    """
    Publish coordinates/values to the 'vision' network table.
    """

    table = NetworkTables.getTable(VISION_TABLE)
    table.putValue(CENTER_X, center_x)
    table.putValue(CENTER_Y, center_y)
    table.putValue(ANGLE_OFFSET, angle_offset)

    if (ENABLE_DEBUG):
        print('Center: (' + str(center_x) + ', ' + str(center_y) + ')')
        print('Angle Offset: ' + str(angle_offset))


def writeFrame(cv_source, frame, x, y, contour_data):
    """
    Draw crosshairs on the target and put the frame in the output stream.
    """

    # Draw blue border surrounding contours
    for contour in contour_data:
        cv2.drawContours(frame, contour.box, -1, (255, 0, 0), 2)

    # Draw red crosshairs on target
    if (x >= 0 and y >= 0):
        cv2.drawMarker(
            img=frame,
            position=(int(x), int(y)),
            color=(0, 0, 255),
            markerSize=40,
            thickness=3)

    cv_source.putFrame(frame)


def processVision(camera, pipeline: GripPipeline, cv_source):
    """
    Read the latest frame and process using the Grip Pipeline.
    """

    if (camera is not None):
        start = time.time()

        frame = readFrame(camera)
        if (frame is not None):
            (x, y, contour_data) = processFrame(frame, pipeline)

            angle_offset = calculateAngleOffset(x)

            publishValues(x, y, angle_offset)

            if (ENABLE_CUSTOM_STREAM):
                writeFrame(cv_source, frame, x, y, contour_data)

        end = time.time()

        if (ENABLE_DEBUG):
            print('Frame process time: ' + str(end - start) + ' s\n')


def main():
    global config_file
    global parsed_width
    global parsed_height

    if len(sys.argv) >= 2:
        config_file = sys.argv[1]

    # Read configuration
    read = readConfig()
    if not read:
        sys.exit(1)

    # Start NetworkTables
    startNetworkTables()

    # Start camera
    camera = None
    camera_config = None
    if (len(camera_configs) >= 1):
        camera_config = camera_configs[0]
        (parsed_width, parsed_height) = parseDimensions(camera_config)

        camera = startCamera(camera_config)
        time.sleep(3)

    # Start custom output stream
    cv_source = None
    if (ENABLE_CUSTOM_STREAM):
        cv_source = startOutputSource(parsed_width, parsed_height)

    print("Running Grip Pipeline...")

    # Initialize Grip Pipeline
    pipeline = GripPipeline()

    # Loop forever
    while True:
        processVision(camera, pipeline, cv_source)


if __name__ == "__main__":
    main()
