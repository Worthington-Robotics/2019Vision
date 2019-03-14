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
import numpy as np
from math import sqrt, degrees, atan
import cv2
from cscore import CameraServer
from .grip import GripPipeline
from .constants import Constants


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


class VisionProcessor:

    def __init__(self, logger, connection, vision_camera, cv_source, parsed_width):
        self.logger = logger
        self.connection = connection
        self.vision_camera = vision_camera
        self.cv_source = cv_source
        self.parsed_width = parsed_width
        self.pipeline = GripPipeline()

    def readFrame(self, camera):
        """
        Reads the latest frame from the camera server instance to pass to opencv.
        :param camera: The camera to read from
        :return: The latest frame
        """

        inst = CameraServer.getInstance()
        cv_sink = inst.getVideo(camera=camera)

        (frame_time, frame) = cv_sink.grabFrame(None)

        return frame

    def processFrame(self, frame, pipeline: GripPipeline):
        """
        Performs extra processing on the pipeline's outputs.
        :param pipeline: The pipeline that just processed an image
        :return: The center coordinates of the target
        """

        contour_data = []

        try:
            # Process the Grip Pipeline
            pipeline.process(frame)

            # Populate data from contours
            contour_data = self.calculateContourData(pipeline)

        except (ZeroDivisionError):
            self.logger.logMessage("Divide by 0 exception in GRIP Pipeline")

        # Find the closest target
        (center_x, center_y) = self.findClosestTarget(contour_data)

        return (center_x, center_y, contour_data)

    def calculateContourData(self, pipeline: GripPipeline):
        """
        Populate the various contour data used in future caluculations.
        """
        contour_data = []

        # Find the bounding boxes of the contours to get x, y, width, and height
        for contour in pipeline.filter_contours_output:
            # Find the centers of mass of the contours
            # https://docs.opencv.org/3.4.2/dd/d49/tutorial_py_contour_features.html

            moments = cv2.moments(contour)
            m00 = moments['m00']
            if (m00 != 0):
                cx = int(moments['m10'] / m00)
                cy = int(moments['m01'] / m00)

                # Calculate if the slope of the contour edge is positive
                (box, left) = self.calculateBoxAndSide(contour)

                contour_data.append(ContourData(cx, cy, box, left))

        self.logger.logMessage('Found ' + str(len(contour_data)) + ' Contours', True)

        return contour_data

    def calculateBoxAndSide(self, contour):
        """
        Calculate the minimum containing box of the contour and determine the side of the pair.
        (i.e. whether the contour is the left or right side of the target)
        """

        # Calculate the 4 corners of the contour
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        # Sort points by y coordinate ascending
        sorted_pts = sorted(box, key=lambda pt: pt[1])

        """
        Find first and third point from the top down (y-axis is the top of the frame)

            (x)                (x)
                 x          x
                      or
        (x)                        (x)
             x                  x
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

    def findClosestTarget(self, contour_data):
        """
        Find the nearest pair of contours that look like: / \
        """

        # Find all pairs
        pairs = self.findPairs(contour_data)

        self.logger.logMessage('Found ' + str(len(pairs)) + ' Pairs', True)

        # Find closest pair
        (center_x, center_y) = self.findClosestPair(pairs)

        return (center_x, center_y)

    def findPairs(self, contour_data):
        """
        Find all pairs of contours that look like: / \
        """

        pairs = []

        # Sort contours by center x coordinate ascending
        contour_data.sort(key=lambda c: c.cx)

        index = 0
        size = len(contour_data)
        done = False
        while (not done and index < size):
            if (index + 1 < size):

                # If the current contour is left and the next is right, then pair found
                if (contour_data[index].left and (not contour_data[index + 1].left)):
                    pairs.append((contour_data[index], contour_data[index+1]))
                    index += 1

                index += 1
            else:
                done = True

        return pairs

    def findClosestPair(self, pairs):
        """
        Find the pair of contours closest to the center of the camera.
        """

        camera_center = self.parsed_width / 2

        closest = None
        (closest_x, closest_y) = (-1, -1)

        for (left, right) in pairs:
            (center_x, center_y) = self.calculateCenter(left, right)
            distance = abs(center_x - camera_center)

            if (closest is None or distance < closest):
                closest = distance
                (closest_x, closest_y) = (center_x, center_y)

        return (closest_x, closest_y)

    def calculateCenter(self, contour_data1: ContourData, contour_data2: ContourData):
        """
        Calculate the midpoint between two contours.
        """

        center_x = (contour_data1.cx + contour_data2.cx) / 2.0
        center_y = (contour_data2.cy + contour_data2.cy) / 2.0

        return (center_x, center_y)

    def calculateAngleOffset(self, center_x):
        """
        Calculates the angle offset.
        (i.e. how much the robot should turn in order to face the target)
        """

        angle_offset = -1000

        if (center_x >= 0):
            pixel_offset = (self.parsed_width / 2) - center_x
            angle_offset = (Constants.CAMERA_FOV / self.parsed_width) * pixel_offset

        return angle_offset

    def writeFrame(self, cv_source, frame, x, y, contour_data):
        """
        Draw crosshairs on the target and put the frame in the output stream.
        """

        if (Constants.ENABLE_IMAGE_SAVE):
            self.logger.saveFrame(frame)

        if (Constants.ENABLE_CUSTOM_STREAM):
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

    def processVision(self):
        """
        Read the latest frame and process using the Grip Pipeline.
        """

        if (self.vision_camera is not None):
            start = time.time()

            frame = self.readFrame(self.vision_camera)
            if (frame is not None):
                (x, y, contour_data) = self.processFrame(frame, self.pipeline)

                angle_offset = self.calculateAngleOffset(x)

                self.connection.publishValues(x, y, angle_offset)

                self.writeFrame(self.cv_source, frame, x, y, contour_data)

            end = time.time()

            self.logger.logMessage('Frame process time: ' + str(end - start) + ' s\n', True)
