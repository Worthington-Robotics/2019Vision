#!/usr/bin/env python3

"""
----------------------------------------------------------------------------
Authors:     FRC Team 4145

Description: This script uses a generated GRIP pipeline to process a camera
             stream and publish results to NetworkTables.  This script is
             designed to work on the 2019 FRCVision Raspberry Pi image.
----------------------------------------------------------------------------
"""

import time
import json
from cscore import CameraServer, VideoSource, UsbCamera
from .constants import Constants


class CameraHost:

    # Width/Height of the vision camera
    parsed_width = None
    parsed_height = None

    # Vision camera
    vision_camera = None
    # Source for outputting OpenCV frames
    cv_source = None

    # Drive camera server
    drive_server = None
    # Drive cameras
    front_camera = None
    back_camera = None
    last_selection = None

    def __init__(self, logger, camera_configs, connection):
        self.logger = logger
        self.camera_configs = camera_configs
        self.connection = connection

        self.startCameras()

    def startCameras(self):
        """
        Start all connected cameras.
        """

        if (len(self.camera_configs) > 0):

            # Start vision camera
            camera_config = self.camera_configs[0]
            (self.parsed_width, self.parsed_height) = self.parseDimensions(camera_config)
            self.vision_camera = self.startVisionCamera(camera_config)
            time.sleep(1)

            # Start custom output stream
            if (Constants.ENABLE_CUSTOM_STREAM):
                self.cv_source = self.startOutputSource(self.parsed_width, self.parsed_height)

            # Start the switchable camera stream
            inst = CameraServer.getInstance()
            self.drive_server = inst.addServer(name="Drive")

            # Start streaming cameras
            if (len(self.camera_configs) >= 2):
                self.front_camera = self.startDriveCamera(self.camera_configs[1])
                self.drive_server.setSource(self.front_camera)

                if (len(self.camera_configs) == 3):
                    self.back_camera = self.startDriveCamera(self.camera_configs[2])

    def startVisionCamera(self, config):
        """
        Start running the vision camera.
        """

        self.logger.logMessage("Starting camera '{}' on {}".format(config.name, config.path))
        inst = CameraServer.getInstance()
        camera = UsbCamera(config.name, config.path)

        camera.setConfigJson(json.dumps(config.config))
        camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

        # Start automatic capture stream
        if (Constants.ENABLE_RAW_STREAM):
            self.logger.logMessage("Starting Raw Output Stream...")

            camera_server = inst.startAutomaticCapture(
                camera=camera, return_server=True)

            if config.stream_config is not None:
                camera_server.setConfigJson(json.dumps(config.stream_config))

        return camera

    def startDriveCamera(self, config):
        """
        Start running a drive camera.
        """

        self.logger.logMessage("Starting camera '{}' on {}".format(config.name, config.path))
        camera = UsbCamera("Drive", config.path)

        camera.setConfigJson(json.dumps(config.config))
        camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

        return camera

    def switchDriveCamera(self):
        """
        Switch the source of the drive camera.
        """

        value = self.connection.getCameraSelection()

        if (value != self.last_selection):
            self.logger.logMessage("Switching Camera Source: " + str(value))

        if (self.drive_server is not None and value != self.last_selection):
            if (self.front_camera is not None and self.front_camera.isConnected() and value == "Front"):
                self.drive_server.setSource(self.front_camera)
            elif (self.back_camera is not None and self.back_camera.isConnected() and value == "Back"):
                self.drive_server.setSource(self.back_camera)

        self.last_selection = value

    def parseDimensions(self, camera_config):
        """
        Parse the width and height of the camera.
        """

        width = None
        height = None
        try:
            width = camera_config.config["width"]
            height = camera_config.config["height"]
        except KeyError:
            self.logger.logMessage("Could not read camera width/height")

        return (width, height)

    def startOutputSource(self, width, height):
        """
        Create an output source and server to ouput custom frames.
        """

        self.logger.logMessage("Starting Custom Output Stream...")

        inst = CameraServer.getInstance()
        cv_source = inst.putVideo("grip", width, height)

        return cv_source

    def readVisionFrame(self):
        """
        Reads the latest frame from the camera server instance to pass to opencv.
        :param camera: The camera to read from
        :return: The latest frame
        """

        frame = None
        if (self.vision_camera is not None):
            inst = CameraServer.getInstance()
            cv_sink = inst.getVideo(camera=self.vision_camera)

            (frame_time, frame) = cv_sink.grabFrame(None)

        return frame

    def outputVisionFrame(self, frame):
        """
        Output an OpenCV frame to the custom vision camera server.
        """

        if (self.cv_source is not None):
            self.cv_source.putFrame(frame)
