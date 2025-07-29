// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include <ffmpeg_encoder_decoder/safe_param.hpp>
#include <ffmpeg_encoder_decoder/utils.hpp>
#include <foxglove_compressed_video_transport/subscriber.hpp>
#include <functional>
#include <unordered_map>

using namespace std::placeholders;

namespace foxglove_compressed_video_transport
{
Subscriber::Subscriber() : logger_(rclcpp::get_logger("Subscriber")) {}

Subscriber::~Subscriber() {}

void Subscriber::frameReady(const ImageConstPtr & img, bool) const { (*userCallback_)(img); }

#ifdef IMAGE_TRANSPORT_USE_QOS
static rclcpp::QoS convert_profile(const rmw_qos_profile_t & p)
{
  return (rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(p), p));
}
#else
static const rmw_qos_profile_t & convert_profile(const rmw_qos_profile_t & p) { return (p); }
#endif

#ifdef IMAGE_TRANSPORT_API_V1
void Subscriber::subscribeImpl(
  rclcpp::Node * node, const std::string & base_topic, const Callback & callback,
  rmw_qos_profile_t custom_qos)
{
  initialize(node, base_topic);
  image_transport::SimpleSubscriberPlugin<CompressedVideo>::subscribeImpl(
    node, base_topic, callback, custom_qos);
}
#else
void Subscriber::subscribeImpl(
  rclcpp::Node * node, const std::string & base_topic, const Callback & callback,
  rmw_qos_profile_t custom_qos, rclcpp::SubscriptionOptions opt)
{
  initialize(node, base_topic);
#ifdef IMAGE_TRANSPORT_API_V2
  (void)opt;  // to suppress compiler warning
  image_transport::SimpleSubscriberPlugin<CompressedVideo>::subscribeImpl(
    node, base_topic, callback, convert_profile(custom_qos));
#else
  image_transport::SimpleSubscriberPlugin<CompressedVideo>::subscribeImpl(
    node, base_topic, callback, convert_profile(custom_qos), opt);
#endif
}
#endif

void Subscriber::initialize(rclcpp::Node * node, const std::string & base_topic)
{
  node_ = node;
  uint ns_len = node_->get_effective_namespace().length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');
  param_namespace_ = param_base_name + "." + getTransportName() + ".";
}

void Subscriber::internalCallback(const CompressedVideoConstPtr & msg, const Callback & user_cb)
{
  if (!decoder_.isInitialized()) {
    if (msg->format.empty()) {
      RCLCPP_ERROR_STREAM(logger_, "no encoding provided!");
      return;
    }
    decoder_.setMeasurePerformance(ffmpeg_encoder_decoder::get_safe_param<bool>(
      node_, param_namespace_ + "measure_performance", false));

    userCallback_ = &user_cb;
    const auto decoders = ffmpeg_encoder_decoder::utils::split_by_char(
      ffmpeg_encoder_decoder::get_safe_param<std::string>(
        node_, param_namespace_ + "map." + msg->format, "h264"),
      ',');
    if (!decoder_.initialize(
          msg->format, std::bind(&Subscriber::frameReady, this, _1, _2), decoders)) {
      RCLCPP_ERROR_STREAM(logger_, "cannot initialize decoder!");
      return;
    }
  }
  decoder_.decodePacket(
    msg->format, &msg->data[0], msg->data.size(), pts_++, msg->frame_id.c_str(),
    rclcpp::Time(msg->timestamp));
}
}  // namespace foxglove_compressed_video_transport
