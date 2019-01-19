#!/usr/bin/env python3
#----------------------------------------------------------------------------
# Copyright (c) 2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.
#----------------------------------------------------------------------------

import json
import time
import sys

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from networktables import NetworkTablesInstance

import cv2
from grip import GripPipeline

# Camera config file
CONFIG_FILE = "/boot/frc.json"


class CameraConfig:
    pass


team = None
server = False
camera_configs = []


def parseError(str):
    """
    Report parse error.
    """

    print("config error in '" + CONFIG_FILE + "': " + str, file=sys.stderr)


def readCameraConfig(config):
    """
    Read single camera configuration.
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
    cam.streamConfig = config.get("stream")

    cam.config = config

    camera_configs.append(cam)
    return True


def readConfig():
    """
    Read configuration file.
    """

    global team
    global server

    # Parse config file
    try:
        with open(CONFIG_FILE, "rt") as f:
            j = json.load(f)
    except OSError as err:
        print(
            "could not open '{}': {}".format(CONFIG_FILE, err),
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
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

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


def startCamera(config):
    """
    Start running the camera.
    """

    print("Starting camera '{}' on {}".format(config.name, config.path))
    inst = CameraServer.getInstance()
    camera = UsbCamera(config.name, config.path)
    server = inst.startAutomaticCapture(camera=camera, return_server=True)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))

    return camera


def readFrame(camera):
    """
    Reads the latest frame from the camera server instance to pass to opencv
    :param camera: The camera to read from
    :return: The latest frame
    """

    inst = CameraServer.getInstance()
    cv_sink = inst.getVideo(camera=camera)

    frameTime, frame = cv_sink.grabFrame(None)

    return frame


def processFrame(frame, pipeline):
    """
    Performs extra processing on the pipeline's outputs and publishes data to NetworkTables.
    :param pipeline: The pipeline that just processed an image
    :return: None
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

    # Calculate the center between two contours (i.e. half the distance between the two contours)
    center_x = -1
    center_y = -1

    if (len(contour_x_positions) == 2 and len(contour_y_positions) == 2):
        center_x = (contour_x_positions[0] + contour_x_positions[1]) / 2.0
        center_y = (contour_y_positions[0] + contour_y_positions[1]) / 2.0

    # Publish to the 'vision' network table
    # table = NetworkTables.getTable(VISION_TABLE)
    # table.putValue(CENTER_X, center_x)
    # table.putValue(CENTER_Y, center_y)
    print('center = (' + str(center_x) + ', ' + str(center_y) + ')')


if __name__ == "__main__":
    if len(sys.argv) >= 2:
        CONFIG_FILE = sys.argv[1]

    # Read configuration
    if not readConfig():
        sys.exit(1)

    # Start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)

    # Start camera
    camera = None
    if (len(camera_configs) >= 1):
        cameraConfig = camera_configs[0]
        camera = startCamera(cameraConfig)
        time.sleep(5)

    # Initialize Grip Pipeline
    pipeline = GripPipeline()

    print("Running Custom Code")

    # Loop forever
    while True:
        if (camera is not None):
            start = time.time()

            # Read frame and process using the Grip Pipeline
            frame = readFrame(camera)
            if (frame is not None):
                processFrame(frame, pipeline)

            end = time.time()
            print('elapsed: ' + str(end - start) + '\n')
