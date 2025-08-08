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
#include <foxglove_compressed_video_transport/parameter_definition.hpp>
#include <foxglove_compressed_video_transport/subscriber.hpp>
#include <functional>
#include <unordered_map>

using namespace std::placeholders;

namespace foxglove_compressed_video_transport
{
using PValue = ParameterDefinition::ParameterValue;
using PDescriptor = ParameterDefinition::ParameterDescriptor;

Subscriber::Subscriber() : logger_(rclcpp::get_logger("FoxgloveSubscriber")) {}

Subscriber::~Subscriber() { decoder_.reset(); }

void Subscriber::shutdown()
{
  if (decoder_.isInitialized()) {
    RCLCPP_INFO_STREAM(logger_, "flushing decoder.");
    decoder_.flush();  // may cause additional frameReady() calls!
    decoder_.reset();
  }
  SimpleSubscriberPlugin::shutdown();
}

void Subscriber::frameReady(const ImageConstPtr & img, bool) const { (*userCallback_)(img); }

#ifdef IMAGE_TRANSPORT_API_V1
void Subscriber::subscribeImpl(
  rclcpp::Node * node, const std::string & base_topic, const Callback & callback,
  QoSType custom_qos)
{
  initialize(node, base_topic);
  image_transport::SimpleSubscriberPlugin<CompressedVideo>::subscribeImpl(
    node, base_topic, callback, custom_qos);
}
#else
void Subscriber::subscribeImpl(
  rclcpp::Node * node, const std::string & base_topic, const Callback & callback,
  QoSType custom_qos, rclcpp::SubscriptionOptions opt)
{
  initialize(node, base_topic);
#ifdef IMAGE_TRANSPORT_API_V2
  (void)opt;  // to suppress compiler warning
  image_transport::SimpleSubscriberPlugin<CompressedVideo>::subscribeImpl(
    node, base_topic, callback, custom_qos);
#else
  image_transport::SimpleSubscriberPlugin<CompressedVideo>::subscribeImpl(
    node, base_topic, callback, custom_qos, opt);
#endif
}
#endif

void Subscriber::initialize(rclcpp::Node * node, const std::string & base_topic_o)
{
  node_ = node;
#ifdef IMAGE_TRANSPORT_RESOLVES_BASE_TOPIC
  const std::string base_topic = base_topic_o;
#else
  const std::string base_topic =
    node_->get_node_topics_interface()->resolve_topic_name(base_topic_o);
#endif
  uint ns_len = node_->get_effective_namespace().length();
  // if a namespace is given (ns_len > 1), then strip one more
  // character to avoid a leading "/" that will then become a "."
  uint ns_prefix_len = ns_len > 1 ? ns_len + 1 : ns_len;
  std::string param_base_name = base_topic.substr(ns_prefix_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');
  paramNamespace_ = param_base_name + "." + getTransportName() + ".";
}

std::string Subscriber::getDecodersFromMap(const std::string & encoding)
{
  const auto x = ffmpeg_encoder_decoder::utils::split_encoding(encoding);
  std::string decoders;
  // successively create parameters that are more and more generic,
  // i.e. hevc.yuv
  for (int i = static_cast<int>(x.size()); i > 0; --i) {
    std::string p_name;
    for (int j = 0; j < i; j++) {
      p_name += "." + x[j];
    }
    ParameterDefinition pdef{
      PValue(""), PDescriptor()
                    .set__name("decoders" + p_name)
                    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
                    .set__description("decoders for encoding: " + p_name)
                    .set__read_only(false)};
    decoders = pdef.declare(node_, paramNamespace_).get<std::string>();
    if (!decoders.empty()) {
      break;
    }
  }
  return (decoders);
}

void Subscriber::internalCallback(const CompressedVideoConstPtr & msg, const Callback & user_cb)
{
  if (decoder_.isInitialized()) {
    // the decoder is already initialized
    decoder_.decodePacket(
      msg->format, &msg->data[0], msg->data.size(), pts_++, msg->frame_id,
      rclcpp::Time(msg->timestamp));
    return;
  }
  userCallback_ = &user_cb;
  const auto codec = ffmpeg_encoder_decoder::utils::split_encoding(msg->format)[0];
  std::string decoder_names = getDecodersFromMap(msg->format);
  decoder_names = ffmpeg_encoder_decoder::utils::filter_decoders(codec, decoder_names);
  if (decoder_names.empty()) {
    decoder_names = ffmpeg_encoder_decoder::utils::find_decoders(codec);
    RCLCPP_WARN_STREAM(
      logger_,
      "no decoders configured for encoding " << msg->format << " defaulting to: " << decoder_names);
  }
  if (decoder_names.empty()) {
    RCLCPP_ERROR_STREAM(
      logger_, "cannot find valid decoder for codec: " << codec << " enc: " << msg->format);
    return;
  }
  RCLCPP_INFO_STREAM(logger_, "trying decoders in order: " << decoder_names);

  for (const auto & dec : ffmpeg_encoder_decoder::utils::split_decoders(decoder_names)) {
    try {
      if (!decoder_.initialize(
            msg->format, std::bind(&Subscriber::frameReady, this, _1, _2), dec)) {
        RCLCPP_ERROR_STREAM(logger_, "cannot initialize decoder: " << dec);
        continue;
      }
    } catch (std::runtime_error & e) {
      continue;
    }
    // sometimes the failure is only detected when the decoding is happening.
    // hopefully this is on the first packet.
    if (!decoder_.decodePacket(
          msg->format, &msg->data[0], msg->data.size(), pts_++, msg->frame_id,
          rclcpp::Time(msg->timestamp))) {
      RCLCPP_ERROR_STREAM(logger_, "decoder cannot decode packet: " << dec);
      decoder_.reset();
      continue;
    }
    RCLCPP_INFO_STREAM(logger_, "successfully opened decoder " << dec);
    break;
  }
}
}  // namespace foxglove_compressed_video_transport
