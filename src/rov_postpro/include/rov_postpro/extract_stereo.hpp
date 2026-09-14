#pragma once

#include <cstdint>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/synchronizer.hpp"

namespace rov_postpro {

class ExtractStereo : public rclcpp::Node {
 public:
  explicit ExtractStereo(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  using Image = sensor_msgs::msg::Image;
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<Image, Image>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void loadParameters();
  void setupLiveSync();
  void processBag();

  void stereoCallback(const Image::ConstSharedPtr& left,
                      const Image::ConstSharedPtr& right);
  bool shouldSample(const rclcpp::Time& stamp);
  void saveStereoPair(const Image::ConstSharedPtr& left,
                      const Image::ConstSharedPtr& right);
  void openCsvIfNeeded();

  std::string bag_path_;
  std::string left_topic_;
  std::string right_topic_;
  std::string output_dir_;
  std::string image_format_;
  int jpg_quality_{95};
  double sync_slop_sec_{0.02};  // max allowed left/right timestamp gap
  int sample_stride_{1};        // keep 1 out of every N synced pairs
  double sample_period_sec_{
      0.0};  // if > 0, overrides stride: min seconds between saved pairs
  bool save_timestamps_csv_{true};
  std::string left_qos_reliability_{"reliable"};
  std::string right_qos_reliability_{"reliable"};

  std::shared_ptr<message_filters::Subscriber<Image>> left_sub_;
  std::shared_ptr<message_filters::Subscriber<Image>> right_sub_;
  std::shared_ptr<Synchronizer> sync_;

  uint64_t pair_counter_{0};   // every synced pair seen, sampled or not
  uint64_t saved_counter_{0};  // pairs actually written to disk
  rclcpp::Time last_saved_stamp_;
  bool has_last_saved_stamp_{false};
  std::ofstream csv_stream_;
};

}  // namespace rov_postpro
