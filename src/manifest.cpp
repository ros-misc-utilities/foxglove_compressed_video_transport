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

#include <foxglove_compressed_video_transport/publisher.hpp>
#include <foxglove_compressed_video_transport/subscriber.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  foxglove_compressed_video_transport::Publisher, image_transport::PublisherPlugin)
PLUGINLIB_EXPORT_CLASS(
  foxglove_compressed_video_transport::Subscriber, image_transport::SubscriberPlugin)
