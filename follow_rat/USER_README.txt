PACKAGE: follow_rat

AUTHORS: Alexia Lee and Joshua Arnold

EMAIL: alexia.lee@uqconnect.edu.au, joshua.arnold1@uqconnect.edu.au

DATE: February 2015

DESCRIPTION: Code to detect moving objects and then identify them as being 
             a rat or an iRat. Code then computes and issues the command 
             velocities required to make the iRat move towards the rat. 

KNOWN ISSUES: The iRat often does not turn quickly enough, and gets confused if the 
              rat is behind it. 

DEPENDENCIES: OpenCV 2.4.10, 
              Opencvblobslib (http://opencvblobslib.github.io/opencvblobslib/)

COMPILATION: run 'rosmake follow_rat'

USAGAGE: The code runs in real-time (that is, it computes frame-by-frame 
         as it displays it, rather than pre-computing everything).

         Make sure you are on the IRAT network.
         Make sure you are running a roscore.
         Add the package to your ROS_PACKAGE_PATH by running
            export ROS_PACKAGE_PATH:=$ROS_PACKAGE_PATH:<path to this package> 

         For instance, if you put this package in the directory /home/irat/other, 
         you would run 
            export ROS_PACKAGE_PATH:=$ROS_PACKAGE_PATH:/home/irat/other

         The command line arguments for the code are as follows: 
            1: the name of the video file or the device number to open. 
               0 is the default device (usually your built-in webcam) and 
               if you plug in an external camera it will usually be 1. 
            2: the number of iRats in the feed; currently it only supports 1
            3: the number of rats in the feed; currently it only supports 1
            4: [optional] name of the output file; this will write the frame 
               with the trajectories and labels etc. to the specified file. 
            5: [optional] the number of miliseconds you want it to wait in 
               each iteration of the main loop. When running the code on a video
               file, it will generally run a lot faster than the framerate and 
               so will be difficult to see. 10 is a sensible number in this case.
               When running the code from a live feed, this option is not necessary
               as it is limited by the framerate of the feed. 

         You should also specify the iRat you are using by specifying it on the commandline.
         Use _topic:=<name of rat>. For example, for the red irat you would pass in 
            _topic:=/irat_red
         Note that the / in front of the irat name is required. 
         If you do not specify the topic on the commandline, it will default to the green iRat. 

         For example, to run the code on device 1 using the red iRat, you would run:
            rosrun follow_rat follow _topic:=/irat_red 1 1 1 

         If in addition you wanted to output the processed video to a file called test_video.avi
         (NOTE the extension is important; if the filename has no extension, file will not output.)
         you would run: 
            rosrun follow_rat follow _topic:=/irat_red 1 1 1 test_video.avi

         If you wanted to test the code on a pre-recorded video file called arena_play.avi
         (just to check that the labelling works correctly, for instance), 
         and wanted to slow it down to about 10 frames per second (so 100ms between frames), 
         and output the processed video to a file called labelling_test.avi, you would run: 
            rosrun follow_rat follow arena_play.avi 1 1 labelling_test.avi 100
         Make sure the iRat is on its side or somewhere safe before you do this, because 
         while it makes no sense to issue velocity commands to the iRat from a video file, 
         the program still will (again this is left in there for testing). 

         When you run the program, you should see a view of the camera/video file and a 
         window containing sliders where you can adjust various parameters. 

         The iRat will have a green label and a green trajectory. By default, the program
         is set to track the GREEN IRAT. If you need to change this, you can do so by 
         adjusting the sliders (green min, red max etc.). There will also be a red circle
         around any moving green objects. 
         
         You need to MANUALLY IDENTIFY THE RAT in the frame by left-clicking on it. The rat will
         then have a red label and a red trajectory. If during the run the label drifts
         away significantly from the rat, left click (repeatedly) on the rat again. 

         The labelling can be is quite robust, but the Kalman filter must have good training 
         at the beginning: avoid having extraneous moving objects in the frame at the 
         beginning (and of course having no other moving objects helps in general).

         While the program is running:
            -- Press escape to exit the program
            -- Press p to pause the program, and p again to unpause it.

END
