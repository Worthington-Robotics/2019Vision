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

import sys
from vision import CameraHost, ConfigParser, Connection, Constants, Logger, UsbDrive, VisionProcessor


def main():

    # Start the USB drive
    usb_drive = UsbDrive()

    # Create logger
    logger = Logger(usb_drive)

    # Parse config from file
    config = ConfigParser(logger)

    # Start NetworkTables
    connection = Connection(logger, config.server, config.team)

    # Start camera(s)
    camera_host = CameraHost(logger, config.camera_configs, connection)

    # Create Vision Processor
    visionProcessor = VisionProcessor(logger, connection, camera_host)

    # Continuously process vision pipeline
    while True:
        visionProcessor.processVision()
        camera_host.switchDriveCamera()


if __name__ == "__main__":
    main()
