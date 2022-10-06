# ROS vision_utils package

Provides utility functions to convert from ROS messages to Python datatypes.
Currently supported conversions:

- Image to OpenCV image
- CompressedImage to OpenCV image (for `compressed` topics)
- CompressedImage to NumPy array (for `compressedDepth` topics)
- PointCloud2 to NumPy arrays and NumPy arrays to Open3D PointCloud
- Open3D PointCloud to NumPy arrays and NumPy arrays to PointCloud2.

# Requirements

Listed in the `requirements.txt` file. Use `pip install requirements.txt` to (hopefully) install all the dependencies

- OpenCV
- Open3d (required only to work with point clouds)
- The `ros_numpy` package

# Use

## Demo

After having sourced the environment and connected to the robot (or to its simulation), run the node implementing a demo usage of the `cloud_conversion` submodule with

'''
rosrun vision_utils cloud_converter.py
'''

## decoder.py

Import in your script using `from vision_utils import decoder`.

## cloud_conversions.py

Import in your script using `from vision_utils import cloud_conversion`.

# TODO

- `decoder.py`
  - [ ] Consider renaming to `image_conversion.py` (though it would cause compatibility issues)
  - [ ] Consider switching to `ros_numpy` to remove the `cv2` dependency and output a `numpy.ndarray` in any case
- `scripts`
  - [ ] Create a `image_converter.py` in `script` to demonstrate a possible usage of the `decoder` module
  - [ ] In `cloud_converter.py`, make the topic to subscribe a command-line argument
- `docs`
  - [x] Change `decoder.py` documentation to Google-style docstrings
  - [x] Add package documentation with `pdoc3`
