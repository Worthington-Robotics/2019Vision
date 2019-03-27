# 2019Vision
This directory contains Python scripts that can be used to run a GRIP Pipeline via the 2019 Raspberry Pi FRC Web Console.

http://wpilib.screenstepslive.com/s/currentCS/m/85074/l/1027798-the-raspberry-pi-frc-console

![VisionDiagram](documentation/Vision%20Diagram.png)

## Documentation
For setup instructions, see the documentation folder of this repository.

## Resources
Camera configuration files and GRIP Pipelines are located in the resources folder of this repository.

## Viewing Output Streams
Output streams can be viewed by opening the "stream.html" file in a web browser (must be connected to robot wifi/ethernet).

Streams can also be accessed from the url matching the following pattern (Where "PORT" is the port of the specific camera stream):

http://frcvision.local:PORT/stream.mjpg

Ports begin at 1181 and are in the order of:

1. Vision raw stream (if enabled)
2. Vision custom output stream (if enabled)
3. Drive camera stream (if present)
