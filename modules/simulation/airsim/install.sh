#!/bin/bash
# AirSim 1.8.1 imports msgpackrpc while generating its package metadata, so the
# RPC dependency must be installed in a separate, earlier pip invocation.
set -e
python3 -m pip install --no-cache-dir "msgpack-rpc-python==0.4.1"
# ROS already supplies compatible NumPy/OpenCV. Avoid AirSim's declared
# opencv-contrib wheel entirely: it shadows Ubuntu's cv2 and breaks cv_bridge.
python3 -m pip install --no-cache-dir --no-deps "airsim==1.8.1"
