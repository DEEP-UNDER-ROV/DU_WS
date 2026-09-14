#include "rov_postpro/extract_stereo.hpp"

#include <algorithm>
#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <filesystem>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sstream>

namespace fs = std::filesystem;

namespace rov_postpro {

ExtractStereo::ExtractStereo(const rclcpp::NodeOptions& options)
    : rclcpp::Node("extract_stereo", options) {
  loadParameters();

  fs::create_directories(fs::path(output_dir_) / "left");
  fs::create_directories(fs::path(output_dir_) / "right");
  openCsvIfNeeded();

  if (bag_path_.empty()) {
    RCLCPP_INFO(get_logger(), "No bag_path given -> live topic mode (%s, %s)",
                left_topic_.c_str(), right_topic_.c_str());
    setupLiveSync();
  } else {
    RCLCPP_INFO(get_logger(), "bag_path set -> reading '%s' offline",
                bag_path_.c_str());
    processBag();
    RCLCPP_INFO(get_logger(), "Done. Saved %lu / %lu synced pairs to %s",
                static_cast<unsigned long>(saved_counter_),
                static_cast<unsigned long>(pair_counter_), output_dir_.c_str());
  }
}

void ExtractStereo::loadParameters() {
  bag_path_ = declare_parameter<std::string>("bag_path", "");
  left_topic_ =
      declare_parameter<std::string>("left_topic", "/camera/left/image_raw");
  right_topic_ =
      declare_parameter<std::string>("right_topic", "/camera/right/image_raw");
  output_dir_ =
      declare_parameter<std::string>("output_dir", "./stereo_extract");
  image_format_ = declare_parameter<std::string>("image_format", "png");
  jpg_quality_ = declare_parameter<int>("jpg_quality", 95);
  sync_slop_sec_ = declare_parameter<double>("sync_slop_sec", 0.02);
  sample_stride_ = declare_parameter<int>("sample_stride", 1);
  sample_period_sec_ = declare_parameter<double>("sample_period_sec", 0.0);
  save_timestamps_csv_ = declare_parameter<bool>("save_timestamps_csv", true);
  left_qos_reliability_ =
      declare_parameter<std::string>("left_qos_reliability", "reliable");
  right_qos_reliability_ =
      declare_parameter<std::string>("right_qos_reliability", "reliable");

  if (sample_stride_ < 1) {
    RCLCPP_WARN(get_logger(), "sample_stride must be >= 1, clamping to 1");
    sample_stride_ = 1;
  }
}

void ExtractStereo::openCsvIfNeeded() {
  if (!save_timestamps_csv_) return;
  const auto csv_path = fs::path(output_dir_) / "stereo_pairs.csv";
  csv_stream_.open(csv_path, std::ios::out | std::ios::trunc);
  csv_stream_
      << "index,left_file,right_file,left_stamp_sec,right_stamp_sec,dt_sec\n";
}

// ---------------------------------------------------------------------------
// Live mode
// ---------------------------------------------------------------------------

void ExtractStereo::setupLiveSync() {
  auto qos_for = [](const std::string& reliability) {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    if (reliability == "best_effort") {
      qos.best_effort();
    } else {
      qos.reliable();
    }
    return qos;
  };

  left_sub_ = std::make_shared<message_filters::Subscriber<Image>>(
      this, left_topic_, qos_for(left_qos_reliability_).get_rmw_qos_profile());
  right_sub_ = std::make_shared<message_filters::Subscriber<Image>>(
      this, right_topic_,
      qos_for(right_qos_reliability_).get_rmw_qos_profile());

  SyncPolicy policy(10 /* queue size */);
  policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_sec_));

  // Constructing with (policy, sub0, sub1) directly is ambiguous on some
  // message_filters versions (it can resolve to a generic N-filter template
  // that tries to default-construct the policy). Build from the policy alone
  // and wire up inputs explicitly instead.
  sync_ = std::make_shared<Synchronizer>(policy);
  sync_->connectInput(*left_sub_, *right_sub_);
  sync_->registerCallback(std::bind(&ExtractStereo::stereoCallback, this,
                                    std::placeholders::_1,
                                    std::placeholders::_2));
}

// ---------------------------------------------------------------------------
// Bag mode
// ---------------------------------------------------------------------------

void ExtractStereo::processBag() {
  rosbag2_cpp::Reader reader;
  reader.open(bag_path_);

  rclcpp::Serialization<Image> serializer;

  std::vector<Image::ConstSharedPtr> left_msgs;
  std::vector<Image::ConstSharedPtr> right_msgs;

  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    if (bag_msg->topic_name != left_topic_ &&
        bag_msg->topic_name != right_topic_) {
      continue;
    }

    auto image = std::make_shared<Image>();
    rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
    serializer.deserialize_message(&serialized, image.get());

    if (bag_msg->topic_name == left_topic_) {
      left_msgs.push_back(image);
    } else {
      right_msgs.push_back(image);
    }
  }

  auto by_stamp = [](const Image::ConstSharedPtr& a,
                     const Image::ConstSharedPtr& b) {
    return rclcpp::Time(a->header.stamp) < rclcpp::Time(b->header.stamp);
  };
  std::sort(left_msgs.begin(), left_msgs.end(), by_stamp);
  std::sort(right_msgs.begin(), right_msgs.end(), by_stamp);

  RCLCPP_INFO(get_logger(), "Loaded %zu left / %zu right frames from bag",
              left_msgs.size(), right_msgs.size());

  // Greedy two-pointer nearest-timestamp match, same tolerance as
  // ApproximateTime.
  size_t i = 0, j = 0;
  while (i < left_msgs.size() && j < right_msgs.size()) {
    const double dt = (rclcpp::Time(left_msgs[i]->header.stamp) -
                       rclcpp::Time(right_msgs[j]->header.stamp))
                          .seconds();
    if (std::fabs(dt) <= sync_slop_sec_) {
      stereoCallback(left_msgs[i], right_msgs[j]);
      ++i;
      ++j;
    } else if (dt < 0.0) {
      ++i;  // left is older, advance left
    } else {
      ++j;  // right is older, advance right
    }
  }
}

// ---------------------------------------------------------------------------
// Shared: sampling + saving
// ---------------------------------------------------------------------------

void ExtractStereo::stereoCallback(const Image::ConstSharedPtr& left,
                                   const Image::ConstSharedPtr& right) {
  ++pair_counter_;

  const rclcpp::Time stamp(left->header.stamp);
  if (!shouldSample(stamp)) {
    return;
  }

  saveStereoPair(left, right);
  last_saved_stamp_ = stamp;
  has_last_saved_stamp_ = true;
}

bool ExtractStereo::shouldSample(const rclcpp::Time& stamp) {
  if (sample_period_sec_ > 0.0) {
    if (!has_last_saved_stamp_) return true;
    return (stamp - last_saved_stamp_).seconds() >= sample_period_sec_;
  }
  // stride-based: keep pair 0, N, 2N, ...
  return (pair_counter_ - 1) % static_cast<uint64_t>(sample_stride_) == 0;
}

void ExtractStereo::saveStereoPair(const Image::ConstSharedPtr& left,
                                   const Image::ConstSharedPtr& right) {
  cv_bridge::CvImageConstPtr left_cv, right_cv;
  try {
    left_cv = cv_bridge::toCvShare(left, sensor_msgs::image_encodings::BGR8);
    right_cv = cv_bridge::toCvShare(right, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", e.what());
    return;
  }

  std::ostringstream name;
  name << "frame_" << std::setw(6) << std::setfill('0') << saved_counter_ << "."
       << image_format_;
  const std::string left_file =
      (fs::path(output_dir_) / "left" / name.str()).string();
  const std::string right_file =
      (fs::path(output_dir_) / "right" / name.str()).string();

  std::vector<int> write_params;
  if (image_format_ == "jpg" || image_format_ == "jpeg") {
    write_params = {cv::IMWRITE_JPEG_QUALITY, jpg_quality_};
  }

  cv::imwrite(left_file, left_cv->image, write_params);
  cv::imwrite(right_file, right_cv->image, write_params);

  if (save_timestamps_csv_ && csv_stream_.is_open()) {
    const double lt = rclcpp::Time(left->header.stamp).seconds();
    const double rt = rclcpp::Time(right->header.stamp).seconds();
    csv_stream_ << saved_counter_ << "," << name.str() << "," << name.str()
                << "," << std::setprecision(17) << lt << "," << rt << ","
                << (lt - rt) << "\n";
  }

  ++saved_counter_;
}

}  // namespace rov_postpro
