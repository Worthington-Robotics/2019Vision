# FRCVision
This directory contains python scripts that can be used to run a grip pipeline via the 2019 Raspberry Pi FRC Web Console.

http://wpilib.screenstepslive.com/s/currentCS/m/85074/l/1027798-the-raspberry-pi-frc-console

To run, first setup a new camera in the "Vision Settings" tab.  Camera settings can be uploaded from the camera-config.json file.

Next, copy the exported Grip pipeline to the /home/pi/ directory of the Pi. To make any changes to this Grip Pipeline, simply overwrite the script.

Finally, upload the vision_processing.py script via the "Application" tab, selecting the "Uploaded Python file" option.

Output can be viewed on the "Vision Status" tab by enabling "Console Output".