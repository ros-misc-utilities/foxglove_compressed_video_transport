// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FOXGLOVE_COMPRESSED_VIDEO_TRANSPORT__SUBSCRIBER_HPP_
#define FOXGLOVE_COMPRESSED_VIDEO_TRANSPORT__SUBSCRIBER_HPP_

#include <ffmpeg_encoder_decoder/decoder.hpp>
#include <foxglove_msgs/msg/compressed_video.hpp>
#include <image_transport/simple_subscriber_plugin.hpp>
#include <string>

namespace foxglove_compressed_video_transport
{
using Image = sensor_msgs::msg::Image;
using ImageConstPtr = Image::ConstSharedPtr;
using foxglove_msgs::msg::CompressedVideo;
using CompressedVideoConstPtr = CompressedVideo::ConstSharedPtr;

class Subscriber : public image_transport::SimpleSubscriberPlugin<CompressedVideo>
{
public:
  Subscriber();
  ~Subscriber();

  std::string getTransportName() const override { return "foxglove"; }

protected:
  void internalCallback(const CompressedVideoConstPtr & msg, const Callback & user_cb) override;

#ifdef IMAGE_TRANSPORT_API_V1
  void subscribeImpl(
    rclcpp::Node * node, const std::string & base_topic, const Callback & callback,
    rmw_qos_profile_t custom_qos) override;
#else
  void subscribeImpl(
    rclcpp::Node * node, const std::string & base_topic, const Callback & callback,
    rmw_qos_profile_t custom_qos, rclcpp::SubscriptionOptions) override;
#endif

private:
  void frameReady(const ImageConstPtr & img, bool /*isKeyFrame*/) const;
  void initialize(rclcpp::Node * node);
  // -------------- variables
  rclcpp::Logger logger_;
  rclcpp::Node * node_;
  ffmpeg_encoder_decoder::Decoder decoder_;
  std::string decoderType_;
  const Callback * userCallback_;
  uint64_t pts_{0};
};
}  // namespace foxglove_compressed_video_transport
#endif  // FOXGLOVE_COMPRESSED_VIDEO_TRANSPORT__SUBSCRIBER_HPP_
