#include <api/RpcLibClientBase.hpp>
#include <common/ImageCaptureBase.hpp>
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {

using Clock = std::chrono::steady_clock;
using ImageRequest = msr::airlib::ImageCaptureBase::ImageRequest;
using ImageType = msr::airlib::ImageCaptureBase::ImageType;
using RpcClient = msr::airlib::MultirotorRpcLibClient;

constexpr std::uint64_t kLandingMmapMagic = 0x4C414E4443414D31ULL;
constexpr std::uint32_t kLandingMmapVersion = 1;
constexpr std::size_t kLandingMmapHeaderBytes = 256;

struct LandingMmapHeader {
  std::uint64_t magic;
  std::uint32_t version;
  std::uint32_t header_bytes;
  std::uint32_t width;
  std::uint32_t height;
  std::uint32_t stride;
  std::uint32_t data_bytes;
  std::uint32_t slot_count;
  std::uint32_t active_slot;
  std::uint64_t sequence;
  std::uint64_t timestamp_ns;
  float camera_position[3];
  float camera_orientation_xyzw[4];
  std::uint8_t reserved[kLandingMmapHeaderBytes - 84];
};
static_assert(sizeof(LandingMmapHeader) == kLandingMmapHeaderBytes,
              "landing mmap header size");

struct Frame {
  std::uint64_t timestamp = 0;
  std::vector<std::uint8_t> data;
  msr::airlib::Vector3r camera_position = msr::airlib::Vector3r::Zero();
  msr::airlib::Quaternionr camera_orientation = msr::airlib::Quaternionr::Identity();
};

template <typename T>
T required(const YAML::Node& node, const char* key, const char* section) {
  if (!node[key]) {
    throw std::runtime_error(std::string("missing ") + section + "." + key);
  }
  return node[key].as<T>();
}

class CameraBridge {
 public:
  CameraBridge() : private_nh_("~") {
    private_nh_.param<std::string>(
        "config_path", config_path_,
        "/work/stack-assets/aruco-landing-sim-x86/config/landing_camera.yaml");
    config_ = YAML::LoadFile(config_path_);
    const auto airsim = config_["airsim"];
    const auto camera = config_["camera"];
    const auto ros_config = config_["ros"];

    ip_ = airsim["ip"] ? airsim["ip"].as<std::string>() : "127.0.0.1";
    port_ = airsim["port"] ? airsim["port"].as<int>() : 41451;
    vehicle_name_ = required<std::string>(airsim, "vehicle_name", "airsim");
    camera_name_ = required<std::string>(airsim, "camera_name", "airsim");
    width_ = required<int>(camera, "width", "camera");
    height_ = required<int>(camera, "height", "camera");
    publish_rate_hz_ = required<double>(camera, "publish_rate_hz", "camera");
    transport_ = camera["transport"] ? camera["transport"].as<std::string>() : "rpc";
    mmap_path_ = camera["mmap_path_container"]
                     ? camera["mmap_path_container"].as<std::string>()
                     : "/work/.build/aruco-landing-sim-x86/landing_camera.mmap";
    mmap_buffer_frames_ = camera["mmap_publish_buffer_frames"]
                              ? camera["mmap_publish_buffer_frames"].as<int>()
                              : 1;
    worker_count_ = camera["rpc_workers"] ? camera["rpc_workers"].as<int>() : 1;
    continuous_rpc_ =
        camera["continuous_rpc"] ? camera["continuous_rpc"].as<bool>() : true;
    max_buffer_frames_ =
        camera["publish_buffer_frames"] ? camera["publish_buffer_frames"].as<int>() : 8;
    startup_buffer_frames_ =
        camera["startup_buffer_frames"] ? camera["startup_buffer_frames"].as<int>() : 2;
    image_type_ = camera["image_type"] ? camera["image_type"].as<int>() : 0;
    optical_frame_ = required<std::string>(ros_config, "optical_frame", "ros");

    if (worker_count_ < 1 || mmap_buffer_frames_ < 1 || max_buffer_frames_ < 1 || startup_buffer_frames_ < 1 ||
        startup_buffer_frames_ > max_buffer_frames_) {
      throw std::runtime_error("invalid worker or bounded-buffer configuration");
    }

    image_pub_ = nh_.advertise<sensor_msgs::Image>(
        required<std::string>(ros_config, "image_topic", "ros"), 1);
    info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
        required<std::string>(ros_config, "camera_info_topic", "ros"), 1);
    ground_truth_pub_ =
        nh_.advertise<nav_msgs::Odometry>("/landing/ground_truth/airsim_local_ned", 2);
    camera_info_ = makeCameraInfo(camera);
    image_message_.height = height_;
    image_message_.width = width_;
    image_message_.encoding = "bgr8";
    image_message_.is_bigendian = false;
    image_message_.step = width_ * 3;
    image_message_.data.resize(static_cast<std::size_t>(width_ * height_ * 3));
    const auto mount = camera["mount_frd"];
    camera_translation_body_ = msr::airlib::Vector3r(
        required<double>(mount, "x", "camera.mount_frd"),
        required<double>(mount, "y", "camera.mount_frd"),
        required<double>(mount, "z", "camera.mount_frd"));
    const float mount_roll = static_cast<float>(
        required<double>(mount, "roll_deg", "camera.mount_frd") * M_PI / 180.0);
    const float mount_pitch = static_cast<float>(
        required<double>(mount, "pitch_deg", "camera.mount_frd") * M_PI / 180.0);
    const float mount_yaw = static_cast<float>(
        required<double>(mount, "yaw_deg", "camera.mount_frd") * M_PI / 180.0);
    camera_orientation_body_ =
        Eigen::AngleAxisf(mount_yaw, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(mount_pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(mount_roll, Eigen::Vector3f::UnitX());
    publishStaticTransforms(camera, ros_config);
    applyRenderSettings(camera);

    ROS_INFO_STREAM("AirSim landing C++ camera ready: vehicle=" << vehicle_name_
                    << " camera=" << camera_name_ << " " << width_ << "x" << height_
                    << "@" << publish_rate_hz_ << ", transport=" << transport_);
  }

  ~CameraBridge() {
    stopWorkers();
    closeMmapReader();
  }

  void run() {
    if (transport_ == "mmap") {
      runMmap();
      return;
    }
    if (transport_ != "rpc") {
      throw std::runtime_error("camera.transport must be rpc or mmap");
    }
    running_.store(true);
    workers_.reserve(static_cast<std::size_t>(worker_count_));
    for (int index = 0; index < worker_count_; ++index) {
      workers_.emplace_back(&CameraBridge::acquireLoop, this, index);
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    auto next_publish = Clock::now();
    auto diagnostics_at = Clock::now() + std::chrono::seconds(5);
    bool primed = false;
    while (ros::ok()) {
      Frame frame;
      bool have_frame = false;
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!primed && frames_.size() >= static_cast<std::size_t>(startup_buffer_frames_)) {
          primed = true;
          next_publish = Clock::now();
        }
        if (primed && !frames_.empty() && Clock::now() >= next_publish) {
          auto first = frames_.begin();
          frame = std::move(first->second);
          frames_.erase(first);
          have_frame = true;
        }
      }

      const auto now = Clock::now();
      if (primed && now >= next_publish) {
        if (have_frame) {
          publishFrame(frame);
          ++published_;
        } else {
          ++empty_slots_;
          primed = false;
        }
        next_publish += std::chrono::duration_cast<Clock::duration>(period);
        if (next_publish <= now) {
          const auto behind = std::chrono::duration<double>(now - next_publish).count();
          const auto missed = static_cast<std::uint64_t>(behind * publish_rate_hz_) + 1;
          empty_slots_ += missed;
          next_publish += std::chrono::duration_cast<Clock::duration>(period * missed);
        }
      }

      if (now >= diagnostics_at) {
        std::size_t buffered = 0;
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          buffered = frames_.size();
        }
        ROS_INFO_STREAM("camera cadence: acquired=" << acquired_.exchange(0) / 5.0
                        << " Hz published=" << published_.exchange(0) / 5.0
                        << " Hz buffer=" << buffered << "/" << max_buffer_frames_
                        << " dropped=" << dropped_.exchange(0)
                        << " empty_slots=" << empty_slots_.exchange(0));
        diagnostics_at += std::chrono::seconds(5);
      }

      ros::spinOnce();
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    stopWorkers();
  }

 private:
  void applyRenderSettings(const YAML::Node& camera) const {
    if (!camera["render_console_commands"]) return;
    try {
      RpcClient client(ip_, static_cast<std::uint16_t>(port_), 2.0f);
      for (const auto& command : camera["render_console_commands"]) {
        const std::string value = command.as<std::string>();
        if (!client.simRunConsoleCommand(value)) {
          ROS_WARN_STREAM("AirSim rejected render console command: " << value);
        }
      }
    } catch (const std::exception& exception) {
      ROS_WARN_STREAM("failed to apply AirSim render settings: " << exception.what());
    }
  }

  void initializeMmapReader() {
    ros::Rate retry(10.0);
    while (ros::ok()) {
      mmap_fd_ = ::open(mmap_path_.c_str(), O_RDONLY);
      if (mmap_fd_ >= 0) {
        struct stat metadata {};
        if (::fstat(mmap_fd_, &metadata) == 0 &&
            metadata.st_size >= static_cast<off_t>(kLandingMmapHeaderBytes)) {
          mmap_size_ = static_cast<std::size_t>(metadata.st_size);
          mmap_address_ = ::mmap(nullptr, mmap_size_, PROT_READ, MAP_SHARED, mmap_fd_, 0);
          if (mmap_address_ != MAP_FAILED) {
            const auto* header = static_cast<const LandingMmapHeader*>(mmap_address_);
            if (header->magic == kLandingMmapMagic &&
                header->version == kLandingMmapVersion &&
                header->header_bytes == kLandingMmapHeaderBytes &&
                header->width == static_cast<std::uint32_t>(width_) &&
                header->height == static_cast<std::uint32_t>(height_) &&
                header->slot_count == 2 &&
                mmap_size_ >= kLandingMmapHeaderBytes +
                                  header->slot_count * header->data_bytes) {
              ROS_INFO_STREAM("reading landing camera mmap: " << mmap_path_);
              return;
            }
            ::munmap(mmap_address_, mmap_size_);
            mmap_address_ = nullptr;
          } else {
            mmap_address_ = nullptr;
          }
        }
        ::close(mmap_fd_);
        mmap_fd_ = -1;
      }
      ROS_WARN_STREAM_THROTTLE(2.0, "waiting for landing camera mmap: " << mmap_path_);
      retry.sleep();
    }
    throw std::runtime_error("ROS stopped while waiting for landing camera mmap");
  }

  bool readMmapFrame(Frame& frame, std::uint64_t& sequence) const {
    const auto* header = static_cast<const LandingMmapHeader*>(mmap_address_);
    const std::uint64_t before = __atomic_load_n(&header->sequence, __ATOMIC_ACQUIRE);
    if (before == 0 || (before & 1ULL) != 0 || before == sequence) return false;
    const std::uint32_t slot = header->active_slot;
    const std::uint32_t data_bytes = header->data_bytes;
    if (slot >= header->slot_count || data_bytes != static_cast<std::uint32_t>(width_ * height_ * 3)) {
      return false;
    }
    frame.timestamp = header->timestamp_ns;
    frame.camera_position = msr::airlib::Vector3r(
        header->camera_position[0], header->camera_position[1], header->camera_position[2]);
    frame.camera_orientation = msr::airlib::Quaternionr(
        header->camera_orientation_xyzw[3], header->camera_orientation_xyzw[0],
        header->camera_orientation_xyzw[1], header->camera_orientation_xyzw[2]);
    const auto* source = static_cast<const std::uint8_t*>(mmap_address_) +
                         kLandingMmapHeaderBytes + slot * data_bytes;
    if (frame.data.size() != data_bytes) frame.data.resize(data_bytes);
    std::copy_n(source, data_bytes, frame.data.data());
    const std::uint64_t after = __atomic_load_n(&header->sequence, __ATOMIC_ACQUIRE);
    if (before != after || (after & 1ULL) != 0) return false;
    sequence = after;
    return true;
  }

  void runMmap() {
    initializeMmapReader();
    std::uint64_t sequence = 0;
    std::vector<Frame> ring(static_cast<std::size_t>(mmap_buffer_frames_));
    for (auto& frame : ring) {
      frame.data.resize(static_cast<std::size_t>(width_ * height_ * 3));
    }
    std::size_t head = 0;
    std::size_t buffered = 0;
    bool primed = false;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_));
    auto next_publish = Clock::now();
    auto diagnostics_at = Clock::now() + std::chrono::seconds(5);
    auto last_diagnostic_frame = Clock::now();
    std::uint64_t source_frames = 0;
    std::uint64_t delivered_frames = 0;
    std::uint64_t overflow_drops = 0;
    while (ros::ok()) {
      const bool full = buffered == ring.size();
      const std::size_t write_index = full ? head : (head + buffered) % ring.size();
      if (readMmapFrame(ring[write_index], sequence)) {
        if (full) {
          head = (head + 1) % ring.size();
          ++overflow_drops;
        } else {
          ++buffered;
        }
        ++source_frames;
      }

      const auto now = Clock::now();
      if (!primed && buffered == ring.size()) {
        primed = true;
        next_publish = now;
      }
      if (primed && now >= next_publish) {
        if (buffered > 0) {
          publishFrame(ring[head]);
          head = (head + 1) % ring.size();
          --buffered;
          ++delivered_frames;
        } else {
          primed = false;
        }
        do {
          next_publish += period;
        } while (next_publish <= now);
      }

      std::this_thread::sleep_for(std::chrono::microseconds(250));
      if (now >= diagnostics_at) {
        const double elapsed = std::chrono::duration<double>(now - last_diagnostic_frame).count();
        ROS_INFO_STREAM("camera mmap cadence: source=" << source_frames / elapsed
                        << " Hz delivered=" << delivered_frames / elapsed << " Hz"
                        << " buffer=" << buffered << "/" << ring.size()
                        << " overflow_dropped=" << overflow_drops);
        source_frames = 0;
        delivered_frames = 0;
        overflow_drops = 0;
        last_diagnostic_frame = now;
        diagnostics_at = now + std::chrono::seconds(5);
      }
      ros::spinOnce();
    }
  }

  void closeMmapReader() {
    if (mmap_address_) {
      ::munmap(mmap_address_, mmap_size_);
      mmap_address_ = nullptr;
    }
    if (mmap_fd_ >= 0) {
      ::close(mmap_fd_);
      mmap_fd_ = -1;
    }
  }

  sensor_msgs::CameraInfo makeCameraInfo(const YAML::Node& camera) const {
    const double fov = required<double>(camera, "horizontal_fov_deg", "camera") * M_PI / 180.0;
    const double derived_focal = width_ / (2.0 * std::tan(fov / 2.0));
    const auto intrinsics = camera["intrinsics"];
    const double fx = intrinsics && intrinsics["fx"] ? intrinsics["fx"].as<double>() : derived_focal;
    const double fy = intrinsics && intrinsics["fy"] ? intrinsics["fy"].as<double>() : derived_focal;
    const double cx = intrinsics && intrinsics["cx"] ? intrinsics["cx"].as<double>() : width_ / 2.0;
    const double cy = intrinsics && intrinsics["cy"] ? intrinsics["cy"].as<double>() : height_ / 2.0;

    sensor_msgs::CameraInfo info;
    info.width = width_;
    info.height = height_;
    info.distortion_model = "plumb_bob";
    info.D.assign(5, 0.0);
    if (intrinsics && intrinsics["distortion"]) {
      info.D = intrinsics["distortion"].as<std::vector<double>>();
    }
    info.K = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.P = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    return info;
  }

  void publishStaticTransforms(const YAML::Node& camera, const YAML::Node& ros_config) {
    const auto mount = camera["mount_frd"];
    const double roll = required<double>(mount, "roll_deg", "camera.mount_frd") * M_PI / 180.0;
    const double pitch = required<double>(mount, "pitch_deg", "camera.mount_frd") * M_PI / 180.0;
    const double yaw = required<double>(mount, "yaw_deg", "camera.mount_frd") * M_PI / 180.0;
    tf2::Matrix3x3 frd;
    frd.setRPY(roll, pitch, yaw);
    const tf2::Matrix3x3 d(1, 0, 0, 0, -1, 0, 0, 0, -1);
    const tf2::Matrix3x3 flu = d * frd * d;
    tf2::Quaternion link_q;
    flu.getRotation(link_q);

    geometry_msgs::TransformStamped link;
    link.header.stamp = ros::Time::now();
    link.header.frame_id = required<std::string>(ros_config, "base_frame", "ros");
    link.child_frame_id = required<std::string>(ros_config, "camera_link_frame", "ros");
    link.transform.translation.x = required<double>(mount, "x", "camera.mount_frd");
    link.transform.translation.y = -required<double>(mount, "y", "camera.mount_frd");
    link.transform.translation.z = -required<double>(mount, "z", "camera.mount_frd");
    link.transform.rotation.x = link_q.x();
    link.transform.rotation.y = link_q.y();
    link.transform.rotation.z = link_q.z();
    link.transform.rotation.w = link_q.w();

    tf2::Quaternion optical_q;
    optical_q.setRPY(-M_PI / 2.0, 0.0, -M_PI / 2.0);
    geometry_msgs::TransformStamped optical;
    optical.header.stamp = link.header.stamp;
    optical.header.frame_id = link.child_frame_id;
    optical.child_frame_id = optical_frame_;
    optical.transform.rotation.x = optical_q.x();
    optical.transform.rotation.y = optical_q.y();
    optical.transform.rotation.z = optical_q.z();
    optical.transform.rotation.w = optical_q.w();
    static_broadcaster_.sendTransform(std::vector<geometry_msgs::TransformStamped>{link, optical});
  }

  void acquireLoop(int worker_index) {
    try {
      const auto worker_period = std::chrono::duration<double>(
          static_cast<double>(worker_count_) / publish_rate_hz_);
      auto next_request = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                             std::chrono::duration<double>(
                                                 worker_index / publish_rate_hz_));
      std::this_thread::sleep_until(next_request);
      RpcClient client(ip_, static_cast<std::uint16_t>(port_), 2.0f);
      const std::vector<ImageRequest> request{
          ImageRequest(camera_name_, static_cast<ImageType>(image_type_), false, false)};
      while (running_.load() && ros::ok()) {
        try {
          auto responses = client.simGetImages(request, vehicle_name_);
          if (responses.empty() || responses.front().width != width_ ||
              responses.front().height != height_ || responses.front().image_data_uint8.empty()) {
            continue;
          }
          Frame frame;
          frame.timestamp = responses.front().time_stamp;
          frame.camera_position = responses.front().camera_position;
          frame.camera_orientation = responses.front().camera_orientation;
          frame.data = std::move(responses.front().image_data_uint8);
          {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (frame.timestamp > last_published_timestamp_.load()) {
              frames_[frame.timestamp] = std::move(frame);
              ++acquired_;
              while (frames_.size() > static_cast<std::size_t>(max_buffer_frames_)) {
                frames_.erase(frames_.begin());
                ++dropped_;
              }
            }
          }
          if (!continuous_rpc_) {
            next_request += std::chrono::duration_cast<Clock::duration>(worker_period);
            const auto now = Clock::now();
            while (next_request <= now) {
              next_request += std::chrono::duration_cast<Clock::duration>(worker_period);
            }
            std::this_thread::sleep_until(next_request);
          }
        } catch (const std::exception& exception) {
          ROS_WARN_STREAM_THROTTLE(2.0, "AirSim image RPC failed: " << exception.what());
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      }
    } catch (const std::exception& exception) {
      ROS_ERROR_STREAM("AirSim image worker failed: " << exception.what());
    }
  }

  void publishFrame(const Frame& frame) {
    if (frame.timestamp <= last_published_timestamp_.load()) {
      return;
    }
    last_published_timestamp_.store(frame.timestamp);
    image_message_.header.stamp = ros::Time(frame.timestamp / 1000000000ULL,
                                           frame.timestamp % 1000000000ULL);
    image_message_.header.frame_id = optical_frame_;
    std::copy(frame.data.begin(), frame.data.end(), image_message_.data.begin());
    camera_info_.header = image_message_.header;
    image_pub_.publish(image_message_);
    info_pub_.publish(camera_info_);

    // ImageResponse pose is the camera pose in AirSim local NED. Undo the
    // configured body-to-camera extrinsic to obtain timestamp-aligned vehicle GT.
    const msr::airlib::Quaternionr body_orientation =
        frame.camera_orientation * camera_orientation_body_.inverse();
    const msr::airlib::Vector3r body_position =
        frame.camera_position - body_orientation * camera_translation_body_;
    nav_msgs::Odometry ground_truth;
    ground_truth.header = image_message_.header;
    ground_truth.header.frame_id = "airsim_local_ned";
    ground_truth.child_frame_id = "airsim_body_frd";
    ground_truth.pose.pose.position.x = body_position.x();
    ground_truth.pose.pose.position.y = body_position.y();
    ground_truth.pose.pose.position.z = body_position.z();
    ground_truth.pose.pose.orientation.x = body_orientation.x();
    ground_truth.pose.pose.orientation.y = body_orientation.y();
    ground_truth.pose.pose.orientation.z = body_orientation.z();
    ground_truth.pose.pose.orientation.w = body_orientation.w();
    ground_truth_pub_.publish(ground_truth);
  }

  void stopWorkers() {
    if (!running_.exchange(false)) {
      return;
    }
    for (auto& worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher image_pub_;
  ros::Publisher info_pub_;
  ros::Publisher ground_truth_pub_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  sensor_msgs::CameraInfo camera_info_;
  sensor_msgs::Image image_message_;
  YAML::Node config_;
  std::string config_path_;
  std::string ip_;
  std::string vehicle_name_;
  std::string camera_name_;
  std::string optical_frame_;
  std::string transport_ = "rpc";
  std::string mmap_path_;
  int port_ = 41451;
  int width_ = 0;
  int height_ = 0;
  int image_type_ = 0;
  int mmap_buffer_frames_ = 1;
  int worker_count_ = 1;
  int max_buffer_frames_ = 8;
  int startup_buffer_frames_ = 2;
  double publish_rate_hz_ = 60.0;
  msr::airlib::Vector3r camera_translation_body_ = msr::airlib::Vector3r::Zero();
  msr::airlib::Quaternionr camera_orientation_body_ =
      msr::airlib::Quaternionr::Identity();
  bool continuous_rpc_ = true;
  std::atomic<bool> running_{false};
  std::vector<std::thread> workers_;
  std::mutex queue_mutex_;
  std::map<std::uint64_t, Frame> frames_;
  std::atomic<std::uint64_t> last_published_timestamp_{0};
  std::atomic<std::uint64_t> acquired_{0};
  std::atomic<std::uint64_t> published_{0};
  std::atomic<std::uint64_t> dropped_{0};
  std::atomic<std::uint64_t> empty_slots_{0};
  int mmap_fd_ = -1;
  void* mmap_address_ = nullptr;
  std::size_t mmap_size_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "airsim_landing_camera");
  try {
    CameraBridge bridge;
    bridge.run();
  } catch (const std::exception& exception) {
    ROS_FATAL_STREAM(exception.what());
    return 1;
  }
  return 0;
}
