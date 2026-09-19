#include <cstdio>
// Subscriber-driven ROS publisher using usb_cam 0.3.7 capture code.
// Camera acquisition settings remain owned by run.sh.
#include <ros/ros.h>
#include <usb_cam/usb_cam.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <std_msgs/String.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>

// Publish standard image_transport-compatible topics without loading transport
// plugins linked to a different OpenCV ABI (Jetson has both 4.2 and 4.5).
class DemandPublisher {
 public:
  void advertise(ros::NodeHandle& nh, const std::string& name) {
    raw_ = nh.advertise<sensor_msgs::Image>(name, 1);
    jpeg_ = nh.advertise<sensor_msgs::CompressedImage>(name + "/compressed", 1);
  }
  unsigned getNumSubscribers() const { return raw_.getNumSubscribers() + jpeg_.getNumSubscribers(); }
  void publish(const sensor_msgs::ImageConstPtr& image) {
    if (raw_.getNumSubscribers()) raw_.publish(image);
    if (jpeg_.getNumSubscribers()) {
      cv::Mat pixels = view(*image), bgr;
      if (image->encoding == "rgb8") cv::cvtColor(pixels, bgr, cv::COLOR_RGB2BGR);
      else bgr = pixels;
      sensor_msgs::CompressedImage jpeg;
      jpeg.header = image->header;
      jpeg.format = image->encoding + "; jpeg compressed " + (image->encoding == "mono8" ? "mono8" : "bgr8");
      if (cv::imencode(".jpg", bgr, jpeg.data, {cv::IMWRITE_JPEG_QUALITY, 85})) jpeg_.publish(jpeg);
    }
  }
  static cv::Mat view(const sensor_msgs::Image& image) {
    int channels = image.encoding == "mono8" ? 1 : 3;
    if (image.encoding != "rgb8" && image.encoding != "bgr8" && image.encoding != "mono8")
      throw std::runtime_error("Unsupported USB output encoding: " + image.encoding);
    if (image.step < image.width * channels || image.data.size() < image.step * image.height)
      throw std::runtime_error("Invalid USB image buffer size");
    return cv::Mat(image.height, image.width, channels == 1 ? CV_8UC1 : CV_8UC3,
                   const_cast<uint8_t*>(image.data.data()), image.step);
  }
 private:
  ros::Publisher raw_, jpeg_;
};

class See3Cam {
 public:
  See3Cam() : nh_("~") {
    cv::setNumThreads(2);
    nh_.param<std::string>("video_device", device_, "/dev/video0");
    nh_.param<std::string>("pixel_format", pixel_format_, "uyvy");
    nh_.param<std::string>("color_format", color_format_, "yuv422p");
    nh_.param<std::string>("io_method", io_method_, "mmap");
    nh_.param<std::string>("camera_frame_id", frame_id_, "see3cam_optical_frame");
    std::string name, url;
    nh_.param<std::string>("camera_name", name, "see3cam_24cug");
    nh_.param<std::string>("camera_info_url", url, "");
    nh_.param("image_width", width_, 1280);
    nh_.param("image_height", height_, 720);
    nh_.param("framerate", fps_, 60);
    nh_.param("rectify_fps", rect_fps_, 20.0);
    if (width_ <= 0 || height_ <= 0 || fps_ <= 0 || rect_fps_ <= 0)
      throw std::runtime_error("Invalid camera rate or dimensions");
    info_.reset(new camera_info_manager::CameraInfoManager(nh_, name, url));
    raw_.advertise(nh_, "image_raw");
    // Compatibility with the original/rectified RViz preview layout.
    color_.advertise(nh_, "image_color");
    rect_.advertise(nh_, "image_rect_color");
    raw_info_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1, true);
    rect_info_ = nh_.advertise<sensor_msgs::CameraInfo>("rectified/camera_info", 1, true);
    status_ = nh_.advertise<std_msgs::String>("stream_status", 1, true);
    updateInfo();
    ROS_INFO("See3CAM ready: %dx%d@%d %s; capture starts when an image subscriber connects",
             width_, height_, fps_, pixel_format_.c_str());
  }

  ~See3Cam() { if (opened_) camera_.shutdown(); }

  void spin() {
    ros::WallRate rate(fps_);
    ros::WallTime next_rect, last_status;
    while (ros::ok()) {
      ros::spinOnce();
      const auto now = ros::WallTime::now();
      const bool raw_wanted = raw_.getNumSubscribers() > 0;
      const bool color_wanted = color_.getNumSubscribers() > 0;
      const bool rect_wanted = rect_.getNumSubscribers() > 0;
      if (raw_wanted || color_wanted || rect_wanted) {
        startCapture();
        sensor_msgs::ImagePtr image(new sensor_msgs::Image);
        image->header.frame_id = frame_id_;
        if (camera_.grab_image(image.get())) {
          last_frame_ = ros::WallTime::now();
          stalled_ = false;
          ++captured_;
          auto ci = info_->getCameraInfo();
          ci.header = image->header;
          if (raw_wanted) { raw_.publish(image); ++raw_sent_; }
          if (color_wanted) color_.publish(image);
          if (raw_info_.getNumSubscribers()) raw_info_.publish(ci);
          if (rect_wanted && now >= next_rect) {
            const ros::WallDuration period(1.0 / rect_fps_);
            next_rect = next_rect.isZero() || (now - next_rect) > period ? now + period : next_rect + period;
            try {
              rectify(image, ci);
            } catch (const std::exception& e) {
              ROS_ERROR_THROTTLE(5, "Cannot rectify camera image: %s", e.what());
            }
          }
        } else {
          ++empty_polls_;
          const auto failed_at = ros::WallTime::now();
          stalled_ = (failed_at - last_frame_).toSec() >= 1.0;
          if (stalled_) ROS_WARN_THROTTLE(2, "No fresh camera frame; polling without publishing stale images");
          // A bounded STREAMOFF/STREAMON attempt, never a USB reset or a
          // format/control change. The loop remains responsive to subscriber loss.
          if ((failed_at - last_frame_).toSec() >= 5.0 &&
              (last_restart_.isZero() || (failed_at-last_restart_).toSec() >= 5.0) &&
              restarts_ < 3) {
            ROS_WARN("Camera stalled: restarting V4L2 stream (attempt %u/3)", unsigned(restarts_+1));
            camera_.stop_capturing();
            camera_.start_capturing();
            last_restart_ = ros::WallTime::now();
            ++restarts_;
          }
        }
      } else if (opened_ && camera_.is_capturing()) {
        // Stop immediately: no conversion/draining of frames with zero demand.
        // Delay restarting is unnecessary; the V4L2 handle and controls stay open.
        camera_.stop_capturing();
        stalled_ = false;
        ROS_INFO("No image subscribers: camera capture paused");
      }
      if ((now - last_status).toSec() >= 1.0) {
        if (!opened_ || !camera_.is_capturing()) updateInfo();
        std::ostringstream text;
        text << "capture=" << (opened_ && camera_.is_capturing() ? (stalled_ ? "stalled" : "active") : "idle")
             << " raw_subscribers=" << raw_.getNumSubscribers()
             << " color_subscribers=" << color_.getNumSubscribers()
             << " rect_subscribers=" << rect_.getNumSubscribers()
             << " empty_polls=" << empty_polls_
             << " stream_restarts=" << restarts_
             << " captured_frames=" << captured_
             << " raw_published=" << raw_sent_
             << " rectified_frames=" << rect_sent_;
        std_msgs::String msg; msg.data = text.str(); status_.publish(msg);
        last_status = now;
      }
      rate.sleep();
    }
  }

 private:
  void startCapture() {
    if (!opened_) {
      auto io = usb_cam::UsbCam::io_method_from_string(io_method_);
      auto pf = usb_cam::UsbCam::pixel_format_from_string(pixel_format_);
      auto cf = usb_cam::UsbCam::color_format_from_string(color_format_);
      if (io == usb_cam::UsbCam::IO_METHOD_UNKNOWN || pf == usb_cam::UsbCam::PIXEL_FORMAT_UNKNOWN ||
          cf == usb_cam::UsbCam::COLOR_FORMAT_UNKNOWN)
        throw std::runtime_error("Unknown USB camera format or IO method");
      camera_.start(device_, io, pf, cf, width_, height_, fps_);
      // Match usb_cam_node's existing automatic white-balance default.
      camera_.set_v4l_parameter("white_balance_temperature_auto", 1);
      opened_ = true;
      last_frame_ = ros::WallTime::now();
      ROS_INFO("Image subscriber connected: camera opened");
    } else if (!camera_.is_capturing()) {
      camera_.start_capturing();
      last_frame_ = ros::WallTime::now();
      ROS_INFO("Image subscriber connected: camera capture resumed");
    }
  }

  bool calibrated(const sensor_msgs::CameraInfo& ci) const {
    return ci.width == static_cast<unsigned>(width_) && ci.height == static_cast<unsigned>(height_) &&
           ci.distortion_model == "plumb_bob" && ci.D.size() == 5 &&
           ci.K[0] > 0 && ci.K[4] > 0 && ci.P[0] > 0 && ci.P[5] > 0;
  }

  sensor_msgs::CameraInfo rectInfo(const sensor_msgs::CameraInfo& ci) const {
    auto out = ci;
    std::fill(out.D.begin(), out.D.end(), 0.0);
    std::fill(out.R.begin(), out.R.end(), 0.0);
    out.R[0] = out.R[4] = out.R[8] = 1;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) out.K[r * 3 + c] = ci.P[r * 4 + c];
    return out;
  }

  void updateInfo() {
    // Calibration-only subscribers must not wake capture. Zero stamp means
    // this latched metadata does not claim to be paired with a fresh image.
    auto ci = info_->getCameraInfo();
    ci.header.frame_id = frame_id_;
    ci.header.stamp = ros::Time(0);
    if (raw_info_.getNumSubscribers() || !info_published_) raw_info_.publish(ci);
    if (calibrated(ci) && (rect_info_.getNumSubscribers() || !info_published_)) rect_info_.publish(rectInfo(ci));
    info_published_ = true;
  }

  void rectify(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfo& ci) {
    if (!calibrated(ci) || image->width != ci.width || image->height != ci.height)
      throw std::runtime_error("Need matching full-resolution plumb_bob CameraInfo (K/D/R/P)");
    if (!maps_ready_ || ci.K != map_info_.K || ci.D != map_info_.D || ci.R != map_info_.R || ci.P != map_info_.P) {
      cv::Mat k(3, 3, CV_64F), r(3, 3, CV_64F), p(3, 3, CV_64F);
      for (int y = 0; y < 3; ++y) for (int x = 0; x < 3; ++x) {
        k.at<double>(y,x) = ci.K[y*3+x]; r.at<double>(y,x) = ci.R[y*3+x]; p.at<double>(y,x) = ci.P[y*4+x];
      }
      cv::initUndistortRectifyMap(k, cv::Mat(ci.D), r, p, cv::Size(width_, height_), CV_16SC2, map1_, map2_);
      map_info_ = ci; maps_ready_ = true;
    }
    const auto source = DemandPublisher::view(*image);
    cv::Mat result;
    cv::remap(source, result, map1_, map2_, cv::INTER_LINEAR);
    sensor_msgs::ImagePtr rectified(new sensor_msgs::Image);
    rectified->header = image->header;
    rectified->height = image->height;
    rectified->width = image->width;
    rectified->encoding = image->encoding;
    rectified->step = result.cols * result.elemSize();
    rectified->data.assign(result.data, result.data + result.total() * result.elemSize());
    rect_.publish(rectified);
    auto out = rectInfo(ci);
    if (rect_info_.getNumSubscribers()) rect_info_.publish(out);
    ++rect_sent_;
  }

  ros::NodeHandle nh_;
  DemandPublisher raw_, color_, rect_;
  ros::Publisher raw_info_, rect_info_, status_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> info_;
  usb_cam::UsbCam camera_;
  std::string device_, pixel_format_, color_format_, io_method_, frame_id_;
  int width_, height_, fps_;
  double rect_fps_;
  ros::WallTime last_frame_, last_restart_;
  bool stalled_ = false;
  uint64_t empty_polls_ = 0, restarts_ = 0;
  bool opened_ = false, maps_ready_ = false, info_published_ = false;
  uint64_t captured_ = 0, raw_sent_ = 0, rect_sent_ = 0;
  sensor_msgs::CameraInfo map_info_;
  cv::Mat map1_, map2_;
};

int main(int argc, char** argv) {
  setvbuf(stdout, nullptr, _IOLBF, 0);
  setvbuf(stderr, nullptr, _IONBF, 0);
  ros::init(argc, argv, "camera");
  try { See3Cam camera; camera.spin(); }
  catch (const std::exception& e) { ROS_FATAL("See3CAM: %s", e.what()); return 1; }
  return 0;
}
