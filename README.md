jsk_openni_kinect
=================

jsk openni tools


## Setup (trusty/inidgo)


### Create catkin workspace
```
$ mkdir-p catkin_ws/ws_jsk_openni_kinect/src
$ cd catkin_ws/ws_jsk_openni_kinect
$ wstool init src
$ wstool set jsk_openni_kinect https://github.com/jsk-ros-pkg/jsk_openni_kinect --git
$ wstool set openni_tracker https://github.com/ros-drivers/openni_tracker --git
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





