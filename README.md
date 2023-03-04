# ROS vision_utils package

Provides utility functions to convert from ROS `sensor_msgs` messages to Python datatypes.
Currently supported conversions:

- Image to NumPy array
- CompressedImage to NumPy array (for `compressed` topics)
- CompressedImage to NumPy array (for `compressedDepth` topics)
- PointCloud2 to NumPy arrays and NumPy arrays to Open3D PointCloud
- Open3D PointCloud to NumPy arrays and NumPy arrays to PointCloud2.

# Requirements

Listed in the `requirements.txt` file. Use `pip install requirements.txt` to (hopefully) install all the dependencies

- OpenCV
- Open3d
- The `ros_numpy` package

# Use

The [documentation](https://luca-pozzi.github.io/vision_utils/) of the `vision_utils` module is provided via GitHub Pages. The source files are also available in the `docs` folder.

## Import

Import in your script using 

```
from vision_utils import image_conversion
from vision_utils import cloud_conversion
```

## Demo
After having sourced the environment and connected to the robot (or to its simulation), run the node implementing a demo usage of each submodule. 

### Images

```
rosrun vision_utils image_conversion.py
```

### Pointcloud

```
rosrun vision_utils cloud_converter.py
```

# Documentation

Full documentation of the package is available [here](https://tiago-we-cobot.github.io/vision_utils/).

To (re-)build the package documentation (e.g. if changing the docstrings, or adding new methods/submodules): 
0. Install [pdoc3](https://pdoc3.github.io/pdoc/) with 
```
pip3 install pdoc3 
```
1. Browse to the root folder of the package (e.g. with `cd ~/tiago_public_ws/src/vision_utils`).
2. Overwrite the existing documentation by running
```
pdoc --html --force --output-dir docs src/vision_utils
```
# TODO

- `src`
  - [ ] Create the `rgbd_processing.py` node, that given one RGB and/or one depth topic(s) checks for the type, receives the images, inpaint them as per the requested method and create a pointcloud (if depth data is available) upon request.
    - [ ] Create `inpainting.py` object for both RGB and depth
  - `image_conversion.py`
    - [x] Output a `numpy.ndarray` in any case
    - [x] Remove the `depth_in_m` arg, just return depth data as `np.float` in meters
- `scripts`
  - [x] Create a `image_converter.py` in `script` to demonstrate a possible usage of the `decoder` module
    - [ ] The currently hardcoded topic work in simulation (apart from `compressedDepth` one), look for topics on the real robot
  - [ ] In `cloud_converter.py`, make the topic to subscribe a command-line argument
- `docs`
  - [x] Change `image_conversion.py` documentation to Google-style docstrings
  - [x] Add package documentation with `pdoc3`
  - [x] Add `ros_numpy` as a dependency