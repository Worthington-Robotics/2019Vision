#!/usr/bin/env python3

"""
----------------------------------------------------------------------------
Authors:     FRC Team 4145

Description: This script uses a generated GRIP pipeline to process a camera
             stream and publish results to NetworkTables.  This script is
             designed to work on the 2019 FRCVision Raspberry Pi image.
----------------------------------------------------------------------------
"""

import subprocess
import os
import datetime
from cv2 import imwrite
from .constants import Constants


# USB Related directories/files
USB_ROOT_PATH = "/dev/sda"
USB_MOUNT_DIR = "/media/usb0"
IMAGE_DIR = USB_MOUNT_DIR + "/images"
LOG_FILE = "log.txt"


class UsbDrive:

    today_dir = None
    frame_index = 0

    def __init__(self):
        self.startUsbDrive()

    def startUsbDrive(self):
        """
        Mount USB drive and create the images folder.
        """

        # Create the mount point
        is_dir = os.path.isdir(USB_MOUNT_DIR)
        if (not is_dir):
            self.makeDirectory(USB_MOUNT_DIR)

        usb_path = self.getUsbPath()
        if (usb_path is not None):
            print('Mounting USB Device: ' + usb_path + ' to dir: ' + USB_MOUNT_DIR)

            # Mount the USB drive to /media/usb0/ -o uid=pi,gid=pi
            output = self.execute(["sudo", "mount", "-o", "uid=pi,gid=pi",
                                   usb_path, USB_MOUNT_DIR])
            for out in output:
                print(out, end="")

            is_mount = os.path.ismount(USB_MOUNT_DIR)
            if (is_mount):
                # Create the images folder
                is_dir = os.path.isdir(IMAGE_DIR)
                if (not is_dir):
                    self.makeDirectory(IMAGE_DIR)

                # Create the folder for the current day
                now = datetime.datetime.now()
                self.today_dir = IMAGE_DIR + "/" + now.strftime("%Y-%m-%d")
                is_dir = os.path.isdir(self.today_dir)
                if (not is_dir):
                    self.makeDirectory(self.today_dir)
                else:
                    print("Using Directory: " + self.today_dir)
        else:
            print('No USB device found')

    def getUsbPath(self):
        """
        Get the path of the last USB drive inserted.
        """
        usb_path = None

        for i in range(0, 4):
            path = USB_ROOT_PATH
            if (i > 0):
                path = path + str(i)

            exists = os.path.exists(path)
            if (exists):
                usb_path = path

        return usb_path

    def execute(self, cmd):
        """
        Execute a shell command on the pi and continuously yield the output.
        """

        popen = subprocess.Popen(cmd, stdout=subprocess.PIPE, universal_newlines=True)

        for stdout_line in iter(popen.stdout.readline, ""):
            yield stdout_line

        popen.stdout.close()
        popen.wait()

    def makeDirectory(self, dir_path):
        """
        Create a directory/folder on the file system.
        """

        # Create the images folder
        print("Creating Directory: " + dir_path)
        output = self.execute(["sudo", "mkdir", dir_path])
        for out in output:
            print(out, end="")

        # Update permissions
        output = self.execute(["sudo", "chmod", "777", dir_path])
        for out in output:
            print(out, end="")

    def logMessage(self, message):
        """
        Output to log file on USB and print to console.
        """

        if (self.today_dir != None):
            log = self.today_dir + "/" + LOG_FILE
            now = datetime.datetime.now()
            time = now.strftime("%H-%M-%S")
            line = time + ": " + message + "\n"

            file = open(log, "a+")
            file.write(line)
            file.flush()
            file.close()

    def saveFrame(self, frame):
        """
        Save the frame to the USB drive.
        """

        name = None
        write = False
        if (self.today_dir != None and self.frame_index % Constants.FRAME_INTERVAL == 0):
            now = datetime.datetime.now()
            name = self.today_dir + "/" + now.strftime("%H-%M-%S") + ".jpeg"

            write = imwrite(name, frame)

            self.frame_index = 0

        self.frame_index += 1

        return (name, write)
