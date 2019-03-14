#!/usr/bin/env python3

"""
----------------------------------------------------------------------------
Authors:     FRC Team 4145

Description: This script uses a generated GRIP pipeline to process a camera
             stream and publish results to NetworkTables.  This script is
             designed to work on the 2019 FRCVision Raspberry Pi image.
----------------------------------------------------------------------------
"""


class Constants:

    # Enable debug logging
    ENABLE_DEBUG = False

    # Enable/Disable Raw Camera Output
    ENABLE_RAW_STREAM = False

    # Enable/Disable Custom Camera Output (i.e. crosshairs on target)
    ENABLE_CUSTOM_STREAM = True

    # Enable/Disable saving of images
    ENABLE_IMAGE_SAVE = True

    # Number of frames between saving images
    FRAME_INTERVAL = 25

    # Camera Field of View (Measured Empirically)
    CAMERA_FOV = 51.06
