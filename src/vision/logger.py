#!/usr/bin/env python3

"""
----------------------------------------------------------------------------
Authors:     FRC Team 4145

Description: This script uses a generated GRIP pipeline to process a camera
             stream and publish results to NetworkTables.  This script is
             designed to work on the 2019 FRCVision Raspberry Pi image.
----------------------------------------------------------------------------
"""

from .constants import Constants


class Logger:

    usbDrive = None

    def __init__(self, usbDrive):
        self.usbDrive = usbDrive

    def logMessage(self, message, debug=False):
        """
        Output to log file on USB and print to console.
        """

        if ((debug and Constants.ENABLE_DEBUG) or not debug):
            if (self.usbDrive != None):
                self.usbDrive.logMessage(message)

            print(message)

    def saveFrame(self, frame):
        """
        Save the frame to the USB drive.
        """

        (name, write) = self.usbDrive.saveFrame(frame)

        if (name is not None):
            self.logMessage("Saving Frame: " + str(name), True)
            self.logMessage("Write: " + str(write), True)
