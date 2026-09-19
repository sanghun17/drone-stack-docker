USB capture source from ros-drivers/usb_cam tag 0.3.7:
https://github.com/ros-drivers/usb_cam/tree/0.3.7

BSD license and copyright are retained in each source file. Build this capture
backend against the same OpenCV as the demand publisher. Jetson's installed
usb_cam binary links OpenCV 4.2 while its development headers are OpenCV 4.5;
linking that binary into the rectifier would mix incompatible versions.
The stock ROS node is replaced. Local patch: grab_image returns a success flag; select polls for 100 ms without exiting on timeout. EINTR/EAGAIN do not produce a new image. Pixel conversion and normal V4L2 buffer handling remain upstream.
