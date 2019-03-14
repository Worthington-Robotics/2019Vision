#!/usr/bin/env python3

"""
----------------------------------------------------------------------------
Authors:     FRC Team 4145

Description: This script uses a generated GRIP pipeline to process a camera
             stream and publish results to NetworkTables.  This script is
             designed to work on the 2019 FRCVision Raspberry Pi image.
----------------------------------------------------------------------------
"""

import sys
import json


class CameraConfig:
    # Camera name
    name = None
    # Path to camera
    path = None
    # Camera config JSON
    config = None
    # Stream config JSON
    stream_config = None


class ConfigParser:

    # Camera config file
    config_file = "/boot/frc.json"

    # Camera settings
    team = None
    server = False
    camera_configs = []

    def __init__(self, logger):
        self.logger = logger
        self.readConfig()

    def parseError(self, line):
        """
        Report parse error.
        """

        self.logger.logMessage("config error in '" + self.config_file + "': " + line, file=sys.stderr)

    def readCameraConfig(self, config):
        """
        Read a single camera configuration.
        """

        cam = CameraConfig()

        # Parse camera name
        try:
            cam.name = config["name"]
        except KeyError:
            self.parseError("could not read camera name")
            return False

        # Parse camera path
        try:
            cam.path = config["path"]
        except KeyError:
            self.parseError("camera '{}': could not read path".format(cam.name))
            return False

        # Parse stream properties
        cam.stream_config = config.get("stream")

        cam.config = config

        self.camera_configs.append(cam)
        return True

    def readConfig(self):
        """
        Read the configuration file.
        """

        # Parse config file
        try:
            with open(self.config_file, "rt") as f:
                j = json.load(f)
        except OSError as err:
            self.logger.logMessage("could not open '{}': {}".format(self.config_file, err))
            return False

        # Top level must be an object
        if not isinstance(j, dict):
            self.parseError("must be JSON object")
            return False

        # Parse team number
        try:
            self.team = j["team"]
        except KeyError:
            self.parseError("could not read team number")
            return False

        # Parse network table mode (server/client)
        if "ntmode" in j:
            mode = j["ntmode"]
            if mode.lower() == "client":
                self.server = False
            elif mode.lower() == "server":
                self.server = True
            else:
                self.parseError("could not understand ntmode value '{}'".format(mode))

        # Parse cameras
        try:
            cameras = j["cameras"]
        except KeyError:
            self.parseError("could not read cameras")
            return False

        for camera in cameras:
            read = self.readCameraConfig(camera)
            if not read:
                return False

        return True
