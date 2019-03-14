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
import sys
from networktables import NetworkTablesInstance


# Network Table constants
SMART_DASHBOARD = "SmartDashboard"
CONNECTION_STATUS = "connect"
VISION_TABLE = "vision"
CAMERA_SELECTION = "CameraSelection"


class Connection:

    def __init__(self, logger, server, team):
        self.logger = logger
        self.server = server
        self.team = team
        self.startNetworkTables()

    def startNetworkTables(self):
        """
        Connect to the Network Tables as a client or start the server locally.
        """

        retry_attempts = 5

        ntinst = NetworkTablesInstance.getDefault()

        if self.server:
            self.logger.logMessage("Setting up NetworkTables server...")
            ntinst.startServer()
        else:
            self.logger.logMessage("Setting up NetworkTables client for team {}".format(self.team))
            ntinst.startClientTeam(self.team)

            # Wait for Network Tables to be connected
            connected = False
            attempt = 0
            while (not connected and attempt < retry_attempts):
                time.sleep(1)
                connected = self.isConnectedToRobot(ntinst)
                self.logger.logMessage("NetworkTables Connected: " +
                                       str(connected) + ", Attempt: " + str(attempt))
                attempt += 1

            if (not connected):
                sys.exit(
                    "Connection to robot not established.  Restarting vision processing...")

    def isConnectedToRobot(self, ntinst):
        """
        This ensures that the Pi is properly connected to the Network Tables.
        """

        connected = False

        # Read status from Network Tables
        ntinst = NetworkTablesInstance.getDefault()
        table = ntinst.getTable(SMART_DASHBOARD).getSubTable(VISION_TABLE)
        value = table.getString(CONNECTION_STATUS, "NOT_INIT")
        self.logger.logMessage("Connection Status: " + value)

        # Restart the script when the RoboRio is first connected to the Network Tables
        if (value == "PING"):
            table.putString(CONNECTION_STATUS, "PONG")
            ntinst.flush()
            time.sleep(1)
            sys.exit("Reconnecting to Network Tables...")

        # Wait for the connection to be re-established
        connected = self.waitForConnection(table)
        if (connected):
            table.putString(CONNECTION_STATUS, "RESET")

        return connected

    def waitForConnection(self, table):
        """
        Wait for the connection status of CONNECTED from the Network Tables.
        """

        retry_attempts = 5

        connected = False
        attempt = 0
        while (not connected and attempt < retry_attempts):
            time.sleep(1)
            value = table.getString(CONNECTION_STATUS, "NOT_INIT")
            connected = (value == "CONNECTED")

            self.logger.logMessage("Connection Status: " + str(value) + ", Attempt: " + str(attempt))
            attempt += 1

        return connected

    def publishValues(self, center_x, center_y, angle_offset):
        """
        Publish coordinates/values to the 'vision' network table.
        """

        ntinst = NetworkTablesInstance.getDefault()
        table = ntinst.getTable(SMART_DASHBOARD).getSubTable(VISION_TABLE)

        table.putValue("centerX", center_x)
        table.putValue("centerY", center_y)
        table.putValue("angleOffset", angle_offset)

        self.logger.logMessage('Center: (' + str(center_x) + ', ' + str(center_y) + ')', True)
        self.logger.logMessage('Angle Offset: ' + str(angle_offset), True)

    def getCameraSelection(self):
        """
        Get the current camera selection from the Network Tables
        """

        ntinst = NetworkTablesInstance.getDefault()
        table = ntinst.getTable(SMART_DASHBOARD)
        value = table.getString(CAMERA_SELECTION, "Front")

        return value
