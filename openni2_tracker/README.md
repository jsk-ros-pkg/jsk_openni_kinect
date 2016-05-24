# OpenNI2 Tracker

#!! This Package Needs Unprovided External Library "NiTE2" !!

Now(2016.5), the NiTE2 library was not provided in official.   
Please get NiTE2 library file named like "NiTE-Linux-x64-2.0.0.tar.bz2" in some other way...   

If you can get the NiTE2 library, put it into the top of this package   
`(CATKIN_WS)/src/jsk-ros-pkg/jsk_openni_kinect/openni2_tracker/NiTE-Linux-x64-2.0.0.tar.bz2`   

This node cannot be run from `rosrun` because of a troublesome runtime path dependency of NiTE2,   
So please launch like   
`roslaunch openni2_tracker tracker.launch`  

