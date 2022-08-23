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

Copy the import part and the desidered function to your script.
The shebang of the script must be edited to point to the environment where you have installed Open3d if you want to run it as main.

# TODO

- [ ] Make `cloud_conversion.py` a module and move it to the `src` folder
  - [ ] remove the `scripts` folder
