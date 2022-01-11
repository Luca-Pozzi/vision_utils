# ROS vision_utils package
Save images from ROS topics into common file formats.

> At the current stage of the development, the code heavily relies on the TIAGo robot it has been conceived for.

# Requirements
- OpenCV
- Open3d (required only to work with point clouds)

# Utilization
## Synchronized RGB and depth images 
To save synchronized RGB and depth images as `.png` (plus the raw depth data in a `.csv` file), establish a connection to the robot, source your environment, open a terminal and type

> `rosrun vision_utils img_saver.py`

By default, the data are saved in the `saved` folder inside the `vision_utils` package. A different (existing) save path can be specified as a command line argument.

## Point clouds
To save point clouds as `.ply` files, establish a connection to the robot, source your environment, open a terminal and type

> `rosrun vision_utils pcd_saver_loader.py`

By default, the data are saved in the `saved` folder inside the `vision_utils` package. A different (existing) save path can be specified as a command line argument.

## RGB-D images and point clouds at once
The two nodes mentioned above can be launched at the same time with the command

>`roslaunch vision_utils rgbd_pcd_saver.launch`

By default, the data are saved in the `saved` folder inside the `vision_utils` package. Different paths can be specified with the `img_save_path` and `pcd_save_path` command line arguments.

# TODO
- [ ] get rid of `cv_bridge` dependency to work with every type of image both in Python2 and 3
	- [ ] adapt to CompressedImages or Images
	- [ ] create a unique callback, acquiring RGB, depth and also point cloud in synchro
- [ ] 'map' the TIAGo topics, propose different default topics wheter the nodes are run on the real robot or in simulation
- [ ] divide the RGB and depth acquisition (put 'options' in the current node or create different nodes)