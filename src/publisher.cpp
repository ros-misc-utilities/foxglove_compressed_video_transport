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
#include <foxglove_compressed_video_transport/publisher.hpp>
#include <foxglove_compressed_video_transport/utils.hpp>

using namespace std::placeholders;

namespace foxglove_compressed_video_transport
{
using ParameterValue = ParameterDefinition::ParameterValue;
using ParameterDescriptor = ParameterDefinition::ParameterDescriptor;
using ParameterType = rcl_interfaces::msg::ParameterType;

static const ParameterDefinition params[] = {
  {ParameterValue("libx264"),
   ParameterDescriptor()
     .set__name("encoder")
     .set__type(ParameterType::PARAMETER_STRING)
     .set__description("ffmpeg encoder to use, see ffmpeg h264 supported encoders")
     .set__read_only(false)},
  {ParameterValue(""),
   ParameterDescriptor()
     .set__name("encoder_av_options")
     .set__type(ParameterType::PARAMETER_STRING)
     .set__description("comma-separated list of AV options: profile:main,preset:ll")
     .set__read_only(false)},
  {ParameterValue(""), ParameterDescriptor()
                         .set__name("preset")
                         .set__type(ParameterType::PARAMETER_STRING)
                         .set__description("ffmpeg encoder preset")
                         .set__read_only(false)},
  {ParameterValue(""), ParameterDescriptor()
                         .set__name("tune")
                         .set__type(ParameterType::PARAMETER_STRING)
                         .set__description("ffmpeg encoder tune")
                         .set__read_only(false)},
  {ParameterValue(""), ParameterDescriptor()
                         .set__name("delay")
                         .set__type(ParameterType::PARAMETER_STRING)
                         .set__description("ffmpeg encoder delay")
                         .set__read_only(false)},
  {ParameterValue(""), ParameterDescriptor()
                         .set__name("crf")
                         .set__type(ParameterType::PARAMETER_STRING)
                         .set__description("ffmpeg encoder crf")
                         .set__read_only(false)},
  {ParameterValue(""), ParameterDescriptor()
                         .set__name("pixel_format")
                         .set__type(ParameterType::PARAMETER_STRING)
                         .set__description("pixel format to use for encoding")
                         .set__read_only(false)},
  {ParameterValue(static_cast<int>(10)),
   ParameterDescriptor()
     .set__name("qmax")
     .set__type(ParameterType::PARAMETER_INTEGER)
     .set__description("max video quantizer scale, see ffmpeg docs")
     .set__read_only(false)
     .set__integer_range(
       {rcl_interfaces::msg::IntegerRange().set__from_value(-1).set__to_value(1024).set__step(1)})},
  {ParameterValue(static_cast<int64_t>(8242880)),
   ParameterDescriptor()
     .set__name("bit_rate")
     .set__type(ParameterType::PARAMETER_INTEGER)
     .set__description("target bit rate, see ffmpeg docs")
     .set__read_only(false)
     .set__integer_range({rcl_interfaces::msg::IntegerRange()
                            .set__from_value(1)
                            .set__to_value(std::numeric_limits<int>::max())
                            .set__step(1)})},
  {ParameterValue(static_cast<int>(1)),
   ParameterDescriptor()
     .set__name("gop_size")
     .set__type(ParameterType::PARAMETER_INTEGER)
     .set__description("gop size (distance between keyframes)")
     .set__read_only(false)
     .set__integer_range({rcl_interfaces::msg::IntegerRange()
                            .set__from_value(1)
                            .set__to_value(std::numeric_limits<int>::max())
                            .set__step(1)})},
  {ParameterValue(false), ParameterDescriptor()
                            .set__name("measure_performance")
                            .set__type(ParameterType::PARAMETER_BOOL)
                            .set__description("enable performance timing")
                            .set__read_only(false)},
  {ParameterValue(static_cast<int>(175)),
   ParameterDescriptor()
     .set__name("performance_interval")
     .set__type(ParameterType::PARAMETER_INTEGER)
     .set__description("after how many frames to print perf info")
     .set__read_only(false)
     .set__integer_range({rcl_interfaces::msg::IntegerRange()
                            .set__from_value(1)
                            .set__to_value(std::numeric_limits<int>::max())
                            .set__step(1)})},
};

Publisher::Publisher() : logger_(rclcpp::get_logger("FoxglovePublisher")) {}

Publisher::~Publisher() {}

// This code was lifted from compressed_image_transport

void Publisher::declareParameter(NodeType node, const ParameterDefinition & definition)
{
  // transport scoped parameter (e.g. image_raw.compressed.format)
  const auto v = definition.declare(node, paramNamespace_);
  const auto & n = definition.descriptor.name;
  if (n == "encoding" || n == "encoder") {
    encoder_.setEncoder(v.get<std::string>());
    RCLCPP_INFO_STREAM(logger_, "using libav encoder: " << v.get<std::string>());
  } else if (n == "encoder_av_options") {
    handleAVOptions(v.get<std::string>());
  } else if (n == "preset" || n == "tune" || n == "delay" || n == "crf") {
    if (!v.get<std::string>().empty()) {
      encoder_.addAVOption(n, v.get<std::string>());
    }
  } else if (n == "pixel_format") {
    encoder_.setAVSourcePixelFormat(v.get<std::string>());
  } else if (n == "qmax") {
    encoder_.setQMax(v.get<int>());
  } else if (n == "bit_rate") {
    encoder_.setBitRate(v.get<int>());
  } else if (n == "gop_size") {
    encoder_.setGOPSize(v.get<int>());
  } else if (n == "measure_performance") {
    measurePerformance_ = v.get<bool>();
    encoder_.setMeasurePerformance(v.get<bool>());
  } else if (n == "performance_interval") {
    performanceInterval_ = v.get<int>();
  } else {
    RCLCPP_ERROR_STREAM(logger_, "unknown parameter: " << n);
  }
}

void Publisher::handleAVOptions(const std::string & opt)
{
  const auto split = utils::splitAVOptions(opt);
  for (const auto & sl : split) {
    const auto kv = utils::splitAVOption(sl);
    if (kv.size() != 2) {
      RCLCPP_WARN_STREAM(logger_, "skipping bad AV option: " << sl);
    } else {
      encoder_.addAVOption(kv[0], kv[1]);
      RCLCPP_INFO_STREAM(logger_, "setting AV option " << kv[0] << " to " << kv[1]);
    }
  }
}

void Publisher::packetReady(
  const std::string & frame_id, const rclcpp::Time & stamp, const std::string & codec,
  uint32_t width, uint32_t height, uint64_t pts, uint8_t flags, uint8_t * data, size_t sz)
{
  (void)codec;
  (void)pts;
  (void)flags;
  (void)width;
  (void)height;
  auto msg = std::make_shared<CompressedVideo>();
  msg->frame_id = frame_id;
  msg->timestamp = stamp;
  msg->format = "h264";
  msg->data.assign(data, data + sz);
#ifdef IMAGE_TRANSPORT_USE_PUBLISHER_T
  (*publishFunction_)->publish(*msg);
#else
  (*publishFunction_)(*msg);
#endif
}

#ifdef IMAGE_TRANSPORT_NEEDS_PUBLISHEROPTIONS
void Publisher::advertiseImpl(
  NodeType node, const std::string & base_topic, QoSType custom_qos, rclcpp::PublisherOptions opt)
{
  auto qos = initialize(node, base_topic, custom_qos);
  SimplePublisherPlugin<CompressedVideo>::advertiseImpl(node, base_topic, qos, opt);
}
#else
void Publisher::advertiseImpl(
  NodeType node, const std::string & base_topic, rmw_qos_profile_t custom_qos)
{
  auto qos = initialize(node, base_topic, custom_qos);
  SimplePublisherPlugin<CompressedVideo>::advertiseImpl(node, base_topic, qos);
}
#endif

Publisher::QoSType Publisher::initialize(
  NodeType node, const std::string & base_topic, QoSType custom_qos)
{
  // namespace handling code lifted from compressed_image_transport
#ifdef IMAGE_TRANSPORT_USE_NODEINTERFACE
  uint ns_len = std::string(node.get_node_base_interface()->get_namespace()).length();
#else
  uint ns_len = node->get_effective_namespace().length();
#endif
  // if a namespace is given (ns_len > 1), then strip one more
  // character to avoid a leading "/" that will then become a "."
  uint ns_prefix_len = ns_len > 1 ? ns_len + 1 : ns_len;
  std::string param_base_name = base_topic.substr(ns_prefix_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');
  paramNamespace_ = param_base_name + "." + getTransportName() + ".";

  for (const auto & p : params) {
    declareParameter(node, p);
  }
  // bump queue size to 2 * distance between keyframes
#ifdef IMAGE_TRANSPORT_USE_QOS
  custom_qos.keep_last(
    std::max(static_cast<int>(custom_qos.get_rmw_qos_profile().depth), 2 * encoder_.getGOPSize()));
#else
  custom_qos.depth = std::max(static_cast<int>(custom_qos.depth), 2 * encoder_.getGOPSize());
#endif
  return (custom_qos);
}

void Publisher::publish(const Image & msg, const PublisherTFn & publish_fn) const
{
  Publisher * me = const_cast<Publisher *>(this);
  me->publishFunction_ = &publish_fn;
  if (!me->encoder_.isInitialized()) {
    if (!me->encoder_.initialize(
          msg.width, msg.height,
          std::bind(&Publisher::packetReady, me, _1, _2, _3, _4, _5, _6, _7, _8, _9),
          msg.encoding)) {
      RCLCPP_ERROR_STREAM(logger_, "cannot initialize encoder!");
      return;
    }
  }
  // may trigger packetReady() callback(s) from encoder!
  me->encoder_.encodeImage(msg);

  if (measurePerformance_) {
    if (static_cast<int>(++me->frameCounter_) > performanceInterval_) {
      me->encoder_.printTimers(logger_.get_name());
      me->encoder_.resetTimers();
      me->frameCounter_ = 0;
    }
  }
}

}  // namespace foxglove_compressed_video_transport
