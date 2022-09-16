# ROS vision_utils package

Provides utility functions to convert from ROS messages to Python datatypes.
Currently supported conversions:

- Image to OpenCV image
- CompressedImage to OpenCV image (for `compressed` topics)
- CompressedImage to NumPy array (for `compressedDepth` topics)

# Requirements

Listed in the `requirements.txt` file. Use `pip install requirements.txt` to (hopefully) install all the dependencies

- OpenCV
- Open3d (required only to work with point clouds)

# Utilization

## decoder.py

Import in your script using `from vision_utils import decoder`.

## cloud_conversions.py

TODO

# TODO

- [ ] Make `cloud_conversion.py` a module and move it to the `src` folder
  - [ ] remove the `scripts` folder
- [ ] Consider converting any ROS message into a NumPy ndarray (which can be handled by OpenCV anyway)
