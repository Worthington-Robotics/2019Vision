# FRCVision
This directory contains python scripts that can be used to run a grip pipeline via the 2019 Raspberry Pi FRC Web Console.

http://wpilib.screenstepslive.com/s/currentCS/m/85074/l/1027798-the-raspberry-pi-frc-console

To run, first setup each camera in the "Vision Settings" tab.  Camera settings can be uploaded from the camera-config.json files.  The first camera added will be used as the vision processing camera.  All other cameras will be used for basic streaming.

Next, copy the exported Grip pipeline "grip.py" to the /home/pi/ directory of the Pi. To make any changes to this Grip Pipeline, simply overwrite the script.

Finally, upload the "vision-processing.py" script via the "Application" tab, selecting the "Uploaded Python file" option.

Output can be viewed on the "Vision Status" tab by enabling "Console Output".  Streams can be accessed from the url matching the pattern (Where "PORT" is the port of the camera):

http://frcvision.local:PORT/stream.mjpg

Ports begin at 1181 and are in the order of:

1. Vision raw stream (if enabled)
2. Vision custom output stream (if enabled)
3. Remaining raw camera streams (if enabled)
