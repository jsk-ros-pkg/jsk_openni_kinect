jsk_openni_kinect
=================

jsk openni tools


## Setup (trusty/inidgo)


### Create catkin workspace
```
$ mkdir-p ~/catkin_ws/ws_jsk_openni_kinect/src
$ cd ~/catkin_ws/ws_jsk_openni_kinect
$ wstool init src
$ wstool set jsk_openni_kinect https://github.com/jsk-ros-pkg/jsk_openni_kinect --git
$ wstool set openni_tracker https://github.com/ros-drivers/openni_tracker --git
$ wstool update
```

### Install missing apt package required to run this package
```
$ rosdep update
$ cd ~/catkin_ws/ws_jsk_openni_kinect
$ rosdep install -r -y --from-paths src --ignore-src
```

### Create primesense-nite-nonfree and install

Install patched version of libopenni-sensor-primesense.sh for devices with `ID 1d27:0609 ASUS`
```
$ ./apt-get-intall-libopenni-sensor-primesense.sh
```

Get NITE-Bin-Linux-x64-v1.5.2.23.tar.bz2 from somewhere and put it in this directory
```
$ ls -al NITE-Bin-Linux-x64-v1.5.2.23.tar.bz2                                            ## make sure you have NITE-BIN
-rw-rw-r-- 1 k-okada k-okada 108732440  5月 25 20:39 NITE-Bin-Linux-x64-v1.5.2.23.tar.bz2 
$ ./apt-get-intall-primesense-nite-nonfree.sh
```

To confirm you have successfully primesense-nite-nonfree,
```
$ ls -al /usr/include/nite          # check if include directory exists
$ ls -al /usr/lib/libXnVNite*       # check if library installed
```

### Compile openni/nite packages
```
$ cd catkin_ws/ws_jsk_openni_kinect
$ catkin b
```

### Run roslaunch and check if your device run correctly
```
$ roslaunch openni_launch openni.launch
$ rosrun openni_tracker openni_tracker _camera_frame_id:=camera_depth_frame
```


## Troubleshooting
### Run lsusb and check if your device correctly recognized by your OS
```
$ lsusb
$ Bus ○○○ Device ○○○: ID 1d27:0601 ASUS
```
If there is no device like "ASUS", the connected device seems to be broken.

### Run NiViewer and check if you can run the device without NiTE function
```
$ NiViewer
```
If you have some error with this command, I recommend you to try
1. disconnect the device
2. sudo service udev restart
3. connect the device again

### Run sample program in NITE-Bin-Dev-Linux-x64-v1.5.2.23/Samples/Bin
```
$ ./Sample-Players
```
