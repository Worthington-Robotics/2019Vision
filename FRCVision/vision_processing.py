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
from math import sqrt

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from networktables import NetworkTablesInstance, NetworkTables

import cv2
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


# Enable/Disable Custom Camera Output (i.e. crosshairs on target)
ENABLE_CUSTOM_OUTPUT = True

# Enable Debug
ENABLE_DEBUG = False

# Network Table constants
VISION_TABLE = "vision"
CENTER_X = "centerX"
CENTER_Y = "centerY"

# Camera config file
config_file = "/boot/frc.json"

# Camera settings
team = None
server = False
camera_configs = []


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
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)


def startCamera(config):
    """
    Start running the camera.
    """

    print("Starting camera '{}' on {}".format(config.name, config.path))
    inst = CameraServer.getInstance()
    camera = UsbCamera(config.name, config.path)
    camera_server = inst.startAutomaticCapture(
        camera=camera, return_server=True)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

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


def startOutputSource(camera_config):
    """
    Create an output source and server to ouput custom frames.
    """

    (width, height) = parseDimensions(camera_config)

    inst = CameraServer.getInstance()
    sink = inst.addServer(name="grip", port=1182)
    cv_source = inst.putVideo("grip", width, height)
    sink.setSource(cv_source)

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


def processFrame(frame, pipeline):
    """
    Performs extra processing on the pipeline's outputs.
    :param pipeline: The pipeline that just processed an image
    :return: The center coordinates of the target
    """

    # Process the Grip Pipeline
    pipeline.process(frame)

    contour_x_positions = []
    contour_y_positions = []

    # Find the bounding boxes of the contours to get x, y, width, and height
    for contour in pipeline.filter_contours_output:
        # Find the centers of mass of the contours
        # https://docs.opencv.org/3.4.2/dd/d49/tutorial_py_contour_features.html

        moments = cv2.moments(contour)
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
        contour_x_positions.append(cx)
        contour_y_positions.append(cy)

    # Calculate the midpoint between two contours
    center_x = -1
    center_y = -1
    distance = -1

    if (len(contour_x_positions) == 2 and len(contour_y_positions) == 2):
        center_x = (contour_x_positions[0] + contour_x_positions[1]) / 2.0
        center_y = (contour_y_positions[0] + contour_y_positions[1]) / 2.0

        distance = sqrt((contour_x_positions[0] - contour_x_positions[1])**2 + (contour_y_positions[0] - contour_y_positions[1])**2)

    print('DISTANCE = ' + str(distance))

    return (center_x, center_y)


def publishValues(center_x, center_y):
    """
    Publish coordinates/values to the 'vision' network table.
    """

    table = NetworkTables.getTable(VISION_TABLE)
    table.putValue(CENTER_X, center_x)
    table.putValue(CENTER_Y, center_y)

    if (ENABLE_DEBUG):
        print('center = (' + str(center_x) + ', ' + str(center_y) + ')')


def writeFrame(cv_source, frame, x, y):
    """
    Draw crosshairs on the target and put the frame in the output stream.
    """

    if (x >= 0 and y >= 0):
        cv2.drawMarker(
            img=frame,
            position=(int(x), int(y)),
            color=(0, 0, 255),
            markerSize=40,
            thickness=3)

    cv_source.putFrame(frame)


def processVision(camera, pipeline, cv_source):
    """
    Read the latest frame and process using the Grip Pipeline.
    """

    if (camera is not None):
        start = time.time()

        frame = readFrame(camera)
        if (frame is not None):
            (x, y) = processFrame(frame, pipeline)

            publishValues(x, y)

            if (ENABLE_CUSTOM_OUTPUT):
                writeFrame(cv_source, frame, x, y)

        end = time.time()

        if (ENABLE_DEBUG):
            print('Frame process time: ' + str(end - start) + ' s\n')


def main():
    global config_file

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
        camera = startCamera(camera_config)
        time.sleep(3)

    # Start custom output stream
    cv_source = None
    if (ENABLE_CUSTOM_OUTPUT):
        cv_source = startOutputSource(camera_config)

    print("Running Grip Pipeline...")

    # Initialize Grip Pipeline
    pipeline = GripPipeline()

    # Loop forever
    while True:
        processVision(camera, pipeline, cv_source)


if __name__ == "__main__":
    main()
