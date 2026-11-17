// Read-only SDK pipeline -> ROS images. No robot control interfaces.
#include <librealsense2/rs.hpp>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

class Camera : public rclcpp::Node {
 public:
  Camera() : Node("camera", "/camera"), pipeline_(context_), align_(RS2_STREAM_COLOR) {
    int width=declare_parameter<int>("width",640),height=declare_parameter<int>("height",480);
    int fps=declare_parameter<int>("fps",6);
    publish_fps_=declare_parameter<int>("publish_fps",6);
    if (!(publish_fps_>0) || publish_fps_>fps) throw std::runtime_error("publish_fps must be positive and no greater than capture fps");
    auto serial=declare_parameter<std::string>("serial_number","");
    rs2::device selected;
    auto devices=context_.query_devices();
    if (serial.empty()) {
      if (devices.size()!=1) throw std::runtime_error("Exactly one RealSense camera is required, or specify serial_number");
      selected=devices[0];serial=selected.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    } else {
      for (auto device:devices) if (serial==device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) selected=device;
      if (!selected) throw std::runtime_error("Requested camera was not found");
    }
    for (auto sensor:selected.query_sensors()) {
      if (sensor.is<rs2::depth_sensor>()) {
        auto logger=get_logger();
        sensor.set_notifications_callback([logger](rs2::notification event) {
          if (event.get_severity()>=RS2_LOG_SEVERITY_ERROR)
            RCLCPP_ERROR(logger,"SDK notification: %s",event.get_description().c_str());
          else RCLCPP_WARN(logger,"SDK notification: %s",event.get_description().c_str());
        });
      }
    }
    const bool reliable=declare_parameter<bool>("reliable_qos",true);
    auto image_qos=rclcpp::QoS(rclcpp::KeepLast(5));
    if (reliable) image_qos.reliable(); else image_qos.best_effort();
    RCLCPP_INFO(get_logger(), "Image QoS: %s, depth=5", reliable ? "reliable" : "best_effort");
    color_pub_=create_publisher<sensor_msgs::msg::Image>("~/color/image_raw",image_qos);
    depth_pub_=create_publisher<sensor_msgs::msg::Image>("~/aligned_depth_to_color/image_raw",image_qos);
    info_pub_=create_publisher<sensor_msgs::msg::CameraInfo>("~/color/camera_info",image_qos);
    depth_info_pub_=create_publisher<sensor_msgs::msg::CameraInfo>("~/aligned_depth_to_color/camera_info",image_qos);
    rs2::config config;config.enable_device(serial);
    config.enable_stream(RS2_STREAM_DEPTH,width,height,RS2_FORMAT_Z16,fps);
    config.enable_stream(RS2_STREAM_COLOR,width,height,RS2_FORMAT_RGB8,fps);
    auto profile=pipeline_.start(config);started_=true;
    depth_scale_=profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
    if (!(depth_scale_>0) || !std::isfinite(depth_scale_)) throw std::runtime_error("Invalid depth unit");
    auto color=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto intr=color.get_intrinsics();
    info_.width=intr.width;info_.height=intr.height;info_.header.frame_id="camera_color_optical_frame";
    info_.distortion_model=intr.model==RS2_DISTORTION_KANNALA_BRANDT4?"equidistant":"plumb_bob";
    info_.d.assign(intr.coeffs,intr.coeffs+(intr.model==RS2_DISTORTION_KANNALA_BRANDT4?4:5));
    info_.k={intr.fx,0.,intr.ppx,0.,intr.fy,intr.ppy,0.,0.,1.};
    info_.r={1.,0.,0.,0.,1.,0.,0.,0.,1.};
    info_.p={intr.fx,0.,intr.ppx,0.,0.,intr.fy,intr.ppy,0.,0.,0.,1.,0.};
    auto depth=profile.get_stream(RS2_STREAM_DEPTH);
    auto extr=depth.get_extrinsics_to(color);
    Eigen::Map<const Eigen::Matrix<float,3,3,Eigen::ColMajor>> color_from_depth(extr.rotation);
    Eigen::Vector3d translation(extr.translation[0],extr.translation[1],extr.translation[2]);
    Eigen::Matrix3d optical_to_body;optical_to_body<<0,0,1,-1,0,0,0,-1,0;
    Eigen::Matrix3d rotation=optical_to_body*color_from_depth.cast<double>().transpose();
    Eigen::Vector3d offset=-rotation*translation;
    broadcaster_=std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    broadcaster_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{
        transform("camera_color_optical_frame",rotation,offset),
        transform("camera_depth_optical_frame",optical_to_body,Eigen::Vector3d::Zero())});
    RCLCPP_INFO(get_logger(),"SDK pipeline %s; serial=%s; firmware=%s; %dx%d@%d; depth_unit_m=%.9f",
        RS2_API_VERSION_STR,serial.c_str(),selected.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION),width,height,fps,depth_scale_);
    RCLCPP_INFO(get_logger(),"Output capped at %.2f FPS; sensor capture %d FPS",publish_fps_,fps);
    worker_=std::thread([this] {capture();});
  }
  ~Camera() override {
    stopping_=true;
    if (worker_.joinable()) worker_.join();
    if (started_) {try {pipeline_.stop();} catch(const std::exception &e) {RCLCPP_ERROR(get_logger(),"Stop: %s",e.what());}}
  }
  bool failed() const {return failed_;}
 private:
  geometry_msgs::msg::TransformStamped transform(const std::string &child,const Eigen::Matrix3d &r,const Eigen::Vector3d &t) {
    geometry_msgs::msg::TransformStamped msg;msg.header.stamp=now();msg.header.frame_id="camera_link";msg.child_frame_id=child;
    Eigen::Quaterniond q(r);q.normalize();
    msg.transform.translation.x=t.x();msg.transform.translation.y=t.y();msg.transform.translation.z=t.z();
    msg.transform.rotation.x=q.x();msg.transform.rotation.y=q.y();msg.transform.rotation.z=q.z();msg.transform.rotation.w=q.w();return msg;
  }
  builtin_interfaces::msg::Time frame_time(const rs2::frame &frame) {
    double seconds=frame.get_timestamp()*.001;
    auto domain=frame.get_frame_timestamp_domain();
    if (domain==RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK) throw std::runtime_error("Camera timestamp is not synchronized to the host clock");
    return rclcpp::Time(static_cast<int64_t>(seconds*1e9));
  }
  void capture() {
    uint64_t received=0,published=0,timeouts=0;int consecutive_timeouts=0,consecutive_unsynced=0;
    try {
      while (rclcpp::ok() && !stopping_) {
        const auto cycle_start=std::chrono::steady_clock::now();
        rs2::frameset frames;
        if (!pipeline_.try_wait_for_frames(&frames,500)) {
          ++timeouts;
          if (++consecutive_timeouts>=6) throw std::runtime_error("No RGB-D frames for 3 seconds");
          continue;
        }
        consecutive_timeouts=0;
        auto color=frames.get_color_frame();auto original_depth=frames.get_depth_frame();
        if (!color || !original_depth) continue;
        ++received;
        const auto acquired=std::chrono::steady_clock::now();
        double skew=std::abs(color.get_timestamp()-original_depth.get_timestamp());
        if (color.get_frame_timestamp_domain()!=original_depth.get_frame_timestamp_domain() || skew>50. ||
            color.get_frame_timestamp_domain()==RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK) {
          if (++consecutive_unsynced>=45) throw std::runtime_error("SDK RGB/depth timestamps are not synchronized");
          continue;
        }
        consecutive_unsynced=0;
        // Limit expensive alignment/publication, preserving sensor timestamps.
        const double sensor_sec=color.get_timestamp()*.001;
        if (next_publish_sec_>0 && sensor_sec<next_publish_sec_-.001) continue;
        const double output_period=1./publish_fps_;
        if (next_publish_sec_<=0 || sensor_sec-next_publish_sec_>output_period)
          next_publish_sec_=sensor_sec;
        next_publish_sec_+=output_period;
        auto aligned=align_.process(frames).as<rs2::frameset>();auto depth=aligned.get_depth_frame();
        const auto aligned_at=std::chrono::steady_clock::now();
        if (!depth) throw std::runtime_error("SDK alignment returned no depth");
        if (depth.get_width()!=color.get_width() || depth.get_height()!=color.get_height()) throw std::runtime_error("Aligned depth dimensions differ from color");
        sensor_msgs::msg::Image rgb,z;
        rgb.header.stamp=frame_time(color);rgb.header.frame_id="camera_color_optical_frame";
        rgb.width=color.get_width();rgb.height=color.get_height();rgb.encoding="rgb8";rgb.step=rgb.width*3;rgb.is_bigendian=false;
        rgb.data.resize(rgb.step*rgb.height);
        for (unsigned row=0;row<rgb.height;++row)
          std::memcpy(rgb.data.data()+row*rgb.step,static_cast<const uint8_t*>(color.get_data())+row*color.get_stride_in_bytes(),rgb.step);
        z.header.stamp=frame_time(original_depth);z.header.frame_id=rgb.header.frame_id;z.width=rgb.width;z.height=rgb.height;z.encoding="16UC1";z.is_bigendian=false;z.step=z.width*2;
        z.data.resize(z.step*z.height);
        for (unsigned row=0;row<z.height;++row) {
          auto source=reinterpret_cast<const uint16_t*>(static_cast<const uint8_t*>(depth.get_data())+row*depth.get_stride_in_bytes());
          if (std::abs(depth_scale_-.001)<1e-8) std::memcpy(z.data.data()+row*z.step,source,z.step);
          else for (unsigned col=0;col<z.width;++col) {
            auto value=static_cast<uint16_t>(std::min(65535L,std::lround(source[col]*depth_scale_*1000.)));
            std::memcpy(z.data.data()+row*z.step+2*col,&value,2);
          }
        }
        info_.header.stamp=rgb.header.stamp;info_pub_->publish(info_);
        auto depth_info=info_;depth_info.header.stamp=z.header.stamp;depth_info_pub_->publish(depth_info);
        color_pub_->publish(std::move(rgb));depth_pub_->publish(std::move(z));++published;
        const auto finished=std::chrono::steady_clock::now();
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),10000,"RGB-D received=%lu published=%lu timeouts=%lu skew_ms=%.4f wait_ms=%.1f align_ms=%.1f publish_ms=%.1f age_ms=%.1f",received,published,timeouts,skew,
            std::chrono::duration<double,std::milli>(acquired-cycle_start).count(),
            std::chrono::duration<double,std::milli>(aligned_at-acquired).count(),
            std::chrono::duration<double,std::milli>(finished-aligned_at).count(),
            (now().seconds()-color.get_timestamp()*.001)*1000.);
      }
    } catch(const std::exception &e) {
      if (!stopping_ && rclcpp::ok()) {failed_=true;RCLCPP_ERROR(get_logger(),"Capture stopped: %s",e.what());rclcpp::shutdown();}
    }
  }
  rs2::context context_;rs2::pipeline pipeline_;rs2::align align_;
  bool started_=false;double depth_scale_=0.,publish_fps_=6.,next_publish_sec_=0.;
  std::atomic<bool> stopping_{false},failed_{false};std::thread worker_;
  sensor_msgs::msg::CameraInfo info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_,depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_,depth_info_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);rs2::log_to_console(RS2_LOG_SEVERITY_WARN);
  try {
    auto node=std::make_shared<Camera>();rclcpp::spin(node);bool failed=node->failed();node.reset();
    if (rclcpp::ok()) rclcpp::shutdown();
    return failed?1:0;
  } catch(const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("camera"),"Initialization failed: %s",e.what());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
