/*
 * Copyright (C) 2024 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef GZ_SIM_SYSTEMS_CAMERASTREAM_HH_
#define GZ_SIM_SYSTEMS_CAMERASTREAM_HH_

#include <memory>
#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  class CameraStreamPrivate;

  /// \brief Stream video from camera sensors to a MediaMTX server
  /// via WHIP or RTSP.
  ///
  /// This is a world-level plugin that idles until a stream request
  /// arrives on the control topic. When a request specifies a camera
  /// name and MediaMTX endpoint URL, the plugin starts capturing
  /// frames from that camera and pushes encoded H.264 video to the
  /// endpoint. Multiple simultaneous streams are supported, each
  /// with its own encoder thread.
  ///
  /// ## System Parameters
  ///
  /// - `<topic>`: Control topic name for stream requests.
  ///   Default: /stream/control
  ///
  /// - `<default_bitrate>`: Default encoding bitrate in bps.
  ///   Default: 4000000
  ///
  /// - `<default_fps>`: Default encoding framerate.
  ///   Default: 30
  ///
  /// ## Control Messages
  ///
  /// The plugin listens for gz::msgs::StringMsg_V on the control
  /// topic:
  ///
  /// Start: data=["start", "<camera_name>", "<whip_or_rtsp_url>",
  ///         "<bitrate>", "<fps>"]  (bitrate and fps are optional)
  ///
  /// Stop:  data=["stop", "<camera_name>"]
  ///
  class CameraStream final:
    public System,
    public ISystemConfigure,
    public ISystemPostUpdate
  {
    public: CameraStream();
    public: ~CameraStream() final;

    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    public: void PostUpdate(const UpdateInfo &_info,
                const EntityComponentManager &_ecm) final;

    private: std::unique_ptr<CameraStreamPrivate> dataPtr;
  };
}
}
}
}
#endif
