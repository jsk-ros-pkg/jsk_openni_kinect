# OpenNI2 Tracker

#!! This Package Needs Unprovided External Library "NiTE2" !!

Now(2016.5), the NiTE2 library was not provided in official.   
Please get NiTE2 library file named like "NiTE-Linux-x64-2.0.0.tar.bz2" in some other way...   

If you can get the NiTE2 library, put it into the top of this package   
`(CATKIN_WS)/src/jsk-ros-pkg/jsk_openni_kinect/openni2_tracker/NiTE-Linux-x64-2.0.0.tar.bz2`   
and then, run `catkin build`.

This node cannot be run from `rosrun` because of a troublesome runtime path dependency of NiTE2,   
So please launch like   
`roslaunch openni2_tracker tracker.launch`  
and you'll see
```
Warning: USB events thread - failed to set priority. This might cause loss of data...

Start moving around to get detected...
(PSI pose may be required for skeleton calibration, depending on the configuration)
[02068893] User #1:	New
[ INFO] [1464110006.050095646]: Found a new user.
[02102262] User #1:	Calibrating...
[03703986] User #1:	Tracking!
[ INFO] [1464110007.677402500]: Now tracking user 1
[ INFO] [1464110007.711739009]: Now tracking user 1
[ INFO] [1464110007.743820543]: Now tracking user 1
[ INFO] [1464110007.774881167]: Now tracking user 1
[ INFO] [1464110007.810146388]: Now tracking user 1
[ INFO] [1464110007.842145840]: Now tracking user 1
```
