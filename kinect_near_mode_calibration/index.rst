kinect_near_mode_calibration ROS Launch Files
=============================================

**Description:** kinect_near_mode_calibration

  
  
       kinect_near_mode_calibration
  
    

**License:** BSD

kinect_near_mode_calibration.launch
-----------------------------------

.. code-block:: bash

  roslaunch kinect_near_mode_calibration kinect_near_mode_calibration.launch


This node is to calibrate kinect with Nyko zoom lens(http://www.nyko.com/products/product-detail/?name=Zoom). To get more accurate depth information, we apply hyperboloid fitting for disparity data.

acquire chessboard images
-------------------------

First, acquire images of chessboard using kinect_calibration package. You have to prepare IR light.

.. code-block:: bash

  mkdir MY_DATA_DIR
  rosrun kinect_calibration acquire_data -r ROWS -c COLS MY_DATA_DIR

The program is two modes: IR camera mode and RGB camera mode. You can change these mode with inputting "i".

The procedure is:

#. Set chessboard the position where can be seen with both IR and RGB camera.
#. Set RGB mode, and save RGB and depth image with inputting "s". In this time, you have to uncover projector and light off IR image (to get depth value).
#. Set IR mode, and save image with inputting "s". In this time, you have to cover projector with something and illuminate chessboard with IR light (it is to detect chessboard lines well in IR camera). You must not move chessboard and kinect after saving RGB image.
#. Iterate from (1) to (3) about 20 times. You should correct data with various position.

compute calibration information
-------------------------------
Second, compute parameters:

- IR CameraInfo

- RGB CameraInfo

- hyperboloid fitting parameters

Compute D, U and V of fitting equation below (please refer http://www.ros.org/wiki/kinect_calibration/technical) :

z = b*f / d'

d' = D*(1/8 * (doff - kd)) + U*u^2 + V*v^2

.. code-block:: bash

  rosrun kinect_near_mode_calibration calibrate -r ROWS -c COLS -s SIZE MY_DATA_DIR

You can get files below inside MY_DATA_DIR:

- calibration_depth.yaml

- calibration_rgb.yaml

- kinect_params.yaml

results
-------

.. figure:: launch/img/pointcloud_c_11.png
  :width: 400

  Pointcloud of chessboard before calibrating. The distortion is big.

.. figure:: launch/img/pointcloud_c_rect_11.png
  :width: 400

  Pointcloud of chessboard after calibrating. The distortion is improved.

get undistorted pointcloud
--------------------------

We attached the data we calibrate in our lab. You can try this package instantly using this calibration data.

.. code-block:: bash

  rosmake depth_image_proc_jsk_patch
  roslaunch sample_zoom.launch

Alternately, if you calibrate zoomed kinect with setting output directry to MY_DATA_DIR, then

.. code-block:: bash

  roslaunch sample_zoom.launch depth_camera_info_url:=MY_DATA_DIR/calibration_depth.yaml rgb_camera_info_url:=MY_DATA_DIR/calibration_rgb.yaml kinect_params_url:=MY_DATA_DIR/kinect_params.yaml

.. figure:: launch/img/hrp2018_look_opencv_book.jpg
  :width: 400

  HRP2 looks OpenCV book. The distance between Kinect and book is about 350mm.

.. figure:: launch/img/opencv_book.jpg
  :width: 400

  left : pointcloud of the book acquired with non-calibrated Kinect. right : pointcloud acquired with calibrated Kinect. You can find out that the distortion is improved.
  
  

Contents
########

.. code-block:: xml

  <launch>
    <arg default="8" name="rows" />
    <arg default="6" name="cols" />
    <arg default="0.108" name="size" />
    <arg default="." name="my_data_dir" />
    <node args="-r $(arg rows) -c $(arg cols) -s $(arg size) $(arg my_data_dir)" name="kinect_near_mode_calibration" pkg="kinect_near_mode_calibration" type="calibrate" />
  
    </launch>

