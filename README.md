Rat-following code
==================
AUTHORS: Alexia Lee, Joshua Arnold, and Scott Heath.

EMAIL: alexia.lee@uqconnect.edu.au

DATE: February 2015

Description
------------
The iRat is a rat-sized robot used for studies in areas such as embodied cognition and 
rat social interaction. The iRat was developed by the Complex and Intelligent Systems group 
at the University of Queensland (http://www.itee.uq.edu.au/cis/home). 
The iRat runs Ubuntu and ROS. 

This code was written during a month-long internship at the Chiba
Lab in UCSD as part of an ongoing effort to help the iRat appear as a social agent.
This repository contains prototype code to allow the iRat to autonomously follow a 
rat using motion tracking. The code can run on both a pre-recorded feed (mostly for 
testing how well the motion-tracking is working) and a live camera feed. In the latter
case, commands are issued to the iRat to enable it to follow the rat. 

This repository also contains other support code such as code to take sample images 
from an overhead camera.

Dependencies
------------
OpenCV 2.4.10, Opencvblobslib

Compilation
-----------
`make` using the provided makefile.

Usage
------
The code runs in real-time (that is, it computes frame-by-frame 
as it displays it, rather than pre-computing everything).
To run the code, go to the directory that the executable is in and type
./bg_subtract <fileName or deviceNumber> <number of iRats> <number of Rats> [<output file>]

For example, to run it on a file called ratsPlay.avi which shows 2 
rats and 1 iRat you would run

./bg_subtract ratsPlay.avi 1 2

To run it on a live video feed, give the program the id of the device.
If it's the default device (built-in webcam) it will be 0 and if it's
the overhead it will probably be 1. So you would run

./bg_subtract 1 1 2 

The program will then display the original frame from the video feed
with bounding boxes around moving objects, a red circle around green 
objects, and text labels for rats and iRats. 

Left click on the original frame to identify the location of rat 1 and right click to identify
the location of the second rat. 

There will also be a window showing the blobs and other processing steps
which you can safely ignore. 

There will be a window containing sliders for adjusting various 
parameters. As a user you should not need to adjust any of these.
If you are a programmer, refer to source code comments for details.

You can optionally output the bounding boxes and object tracking frame
to a video file by supplying the output file name as the last argument
to the program. For instance

./bg_subtract 1 1 2 ratTracking.avi 

will run the program from the feed of device 1 with 1 iRat and 2 rats,
and output the resulting video file to ratTracking.avi. 

While the program is running:
   - Press escape to exit the program
   - Press p to pause the program, and p again to unpause it.
