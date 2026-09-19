// By default this touches no hardware and needs no ROS master. An explicit
// device path additionally checks two open/capture/repeated-shutdown cycles.
#include <usb_cam/usb_cam.h>
#include <iostream>

int main(int argc, char** argv) {
  ros::Time::init();
  {
    usb_cam::UsbCam never_opened;
    never_opened.shutdown();
    never_opened.shutdown();
  }
  if (argc == 2) {
    usb_cam::UsbCam camera;
    for (int cycle = 0; cycle < 2; ++cycle) {
      camera.start(argv[1], usb_cam::UsbCam::IO_METHOD_MMAP,
                   usb_cam::UsbCam::PIXEL_FORMAT_UYVY,
                   usb_cam::UsbCam::COLOR_FORMAT_YUV422P, 1280, 720, 60);
      sensor_msgs::Image image;
      bool received = false;
      for (int attempt = 0; attempt < 30 && !received; ++attempt)
        received = camera.grab_image(&image);
      if (!received || image.data.size() != 1280u * 720u * 3u) {
        std::cerr << "Failed to capture a complete RGB image\n";
        return 1;
      }
      camera.shutdown();
      camera.shutdown();
    }
  } else if (argc != 1) {
    return 2;
  }
  std::cout << "Lifecycle cleanup passed\n";
  return 0;
}
