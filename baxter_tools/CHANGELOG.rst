1.2.0 (2015-12-21)
---------------------------------
- Updated Smoke Test and Tuck Arms scripts for head pan speed range shift (before [0, 100], and now [0, 1.0])
- Updated ITB names for Navigators' name change

1.1.1 (2015-5-15)
---------------------------------
- Migrates users from baxter_tools/update_robot.py to on-robot rethink-updater over
  *ruser* SSH session. See (http://sdk.rethinkrobotics.com/wiki/SSH_Update) for details

1.1.0 (2014-12-17)
---------------------------------
- Updates baxter_tools for ROS Indigo (queue_size args for Publishers, using cv2 as OpenCV dependancy)
- Updates smoke_tests to use 64-bit Indigo Camera fix

1.0.0 (2014-5-1)
---------------------------------
- Updates baxter_tools scripts to verify software version compatibility
- Updates update_robot adding instructions for attaching grippers before proceeding with update
- Fixes update_robot timeout for identifying available updates by increasing timeout to five seconds

0.7.0 (2013-11-21)
---------------------------------
- Creation of baxter_tools repository from sdk-examples/tools.
- Package restructure in support of Catkin expected standards.
- Adds camera_control from previous location in baxter_examples.
- Adds check for grippers pre-tare/calibrate.
- Adds reset to camera_control tool. Useful when not all cameras enumerated at boot.
- Fixes tucks arms allowing for clean handling of interrupts and partial tucks.

