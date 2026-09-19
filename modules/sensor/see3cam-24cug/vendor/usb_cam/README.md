USB capture source from ros-drivers/usb_cam tag 0.3.7:
https://github.com/ros-drivers/usb_cam/tree/0.3.7

BSD license and copyright are retained in each source file. Build this capture
backend against the same OpenCV as the demand publisher. Jetson's installed
usb_cam binary links OpenCV 4.2 while its development headers are OpenCV 4.5;
linking that binary into the rectifier would mix incompatible versions.
The stock ROS node is replaced. Local patch: grab_image returns a success flag; select polls for 100 ms without exiting on timeout. EINTR/EAGAIN do not produce a new image. MMAP discards error-flagged buffers and incomplete UYVY frames before conversion. Pixel conversion remains upstream.
Cleanup patch: initialize the FFmpeg parser pointer, make unmap/close repeatable,
and release owned RGB and decoder allocations. The node delegates destruction
to UsbCam instead of shutting the same object down twice.
