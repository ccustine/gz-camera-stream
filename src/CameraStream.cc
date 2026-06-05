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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>

#include "gz/sim/rendering/RenderUtil.hh"
#include "gz/sim/rendering/Events.hh"

#include "gz/sim/components/Camera.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Events.hh"
#include "gz/sim/Util.hh"

#include "CameraStream.hh"
#include "StreamContext.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::CameraStreamPrivate
{
  /// \brief Callback for control messages (topic subscriber)
  public: void OnControlMessage(const msgs::StringMsg_V &_msg);

  /// \brief Service callback for control (called via WebSocket req)
  public: bool OnControlService(const msgs::StringMsg_V &_req,
      msgs::Boolean &_res);

  /// \brief Callback invoked in the rendering thread after a render update
  public: void OnPostRender();

  /// \brief No-op image callback to keep camera sensors active
  public: void OnImage(const msgs::Image &);

  /// \brief Start streaming a camera
  /// \param[in] _cameraName Camera sensor name
  /// \param[in] _url WHIP or RTSP endpoint URL
  /// \param[in] _bitrate Encoding bitrate (0 = use default)
  /// \param[in] _fps Encoding fps (0 = use default)
  public: void StartStream(const std::string &_cameraName,
                            const std::string &_url,
                            unsigned int _bitrate,
                            unsigned int _fps);

  /// \brief Stop streaming a camera
  /// \param[in] _cameraName Camera sensor name
  public: void StopStream(const std::string &_cameraName);

  /// \brief Transport node
  public: transport::Node node;

  /// \brief Mutex for stream map access
  public: std::mutex streamMutex;

  /// \brief Active streams keyed by camera name
  public: std::unordered_map<std::string,
      std::unique_ptr<StreamContext>> activeStreams;

  /// \brief Connection to the post-render event
  public: common::ConnectionPtr postRenderConn;

  /// \brief Pointer to the event manager
  public: EventManager *eventMgr = nullptr;

  /// \brief Pointer to the rendering scene
  public: rendering::ScenePtr scene;

  /// \brief Control topic name
  public: std::string controlTopic{"/stream/control"};

  /// \brief Default encoding bitrate
  public: unsigned int defaultBitrate = 4000000;

  /// \brief Default encoding fps
  public: unsigned int defaultFps = 30;

  /// \brief Current simulation time
  public: std::chrono::steady_clock::duration simTime{0};

  /// \brief Sensor topics we've subscribed to (to keep sensors active)
  public: std::unordered_map<std::string, std::string> sensorTopics;
};

void CameraStreamPrivate::OnImage(const msgs::Image &)
{
  // No-op - subscribing keeps the sensor active
}

void CameraStreamPrivate::OnControlMessage(const msgs::StringMsg_V &_msg)
{
  if (_msg.data_size() < 2)
  {
    gzerr << "Stream control message requires at least 2 fields "
           << "(action, camera_name)" << std::endl;
    return;
  }

  const std::string &action = _msg.data(0);
  const std::string &cameraName = _msg.data(1);

  if (action == "start")
  {
    if (_msg.data_size() < 3)
    {
      gzerr << "Stream start requires URL in data[2]" << std::endl;
      return;
    }
    const std::string &url = _msg.data(2);

    unsigned int bitrate = this->defaultBitrate;
    unsigned int fps = this->defaultFps;

    if (_msg.data_size() >= 4 && !_msg.data(3).empty())
    {
      try { bitrate = std::stoul(_msg.data(3)); }
      catch (...) {}
    }
    if (_msg.data_size() >= 5 && !_msg.data(4).empty())
    {
      try { fps = std::stoul(_msg.data(4)); }
      catch (...) {}
    }

    this->StartStream(cameraName, url, bitrate, fps);
  }
  else if (action == "stop")
  {
    this->StopStream(cameraName);
  }
  else
  {
    gzerr << "Unknown stream action: [" << action << "]. "
           << "Use 'start' or 'stop'." << std::endl;
  }
}

bool CameraStreamPrivate::OnControlService(const msgs::StringMsg_V &_req,
    msgs::Boolean &_res)
{
  this->OnControlMessage(_req);
  _res.set_data(true);
  return true;
}

void CameraStreamPrivate::StartStream(const std::string &_cameraName,
                                       const std::string &_url,
                                       unsigned int _bitrate,
                                       unsigned int _fps)
{
  std::lock_guard<std::mutex> lock(this->streamMutex);

  if (this->activeStreams.count(_cameraName))
  {
    gzwarn << "Stream already active for camera [" << _cameraName
            << "], stopping first" << std::endl;
    auto &existing = this->activeStreams[_cameraName];
    existing->Stop();

    // Unsubscribe from sensor topic
    auto topicIt = this->sensorTopics.find(_cameraName);
    if (topicIt != this->sensorTopics.end())
    {
      this->node.Unsubscribe(topicIt->second);
      this->sensorTopics.erase(topicIt);
    }

    this->activeStreams.erase(_cameraName);
  }

  // Width/height will be set when we find the camera in OnPostRender.
  // Use reasonable defaults for now - they'll be updated on first frame.
  auto ctx = std::make_unique<StreamContext>(
      _cameraName, _url, 960, 540, _fps, _bitrate);

  this->activeStreams[_cameraName] = std::move(ctx);

  // Connect to PostRender if this is the first stream
  if (!this->postRenderConn && this->eventMgr)
  {
    this->postRenderConn =
        this->eventMgr->Connect<events::PostRender>(
        std::bind(&CameraStreamPrivate::OnPostRender, this));
    gzmsg << "Connected to PostRender event for camera streaming"
           << std::endl;
  }

  gzmsg << "Stream request queued for camera [" << _cameraName
         << "] -> [" << _url << "]" << std::endl;
}

void CameraStreamPrivate::StopStream(const std::string &_cameraName)
{
  std::lock_guard<std::mutex> lock(this->streamMutex);

  auto it = this->activeStreams.find(_cameraName);
  if (it == this->activeStreams.end())
  {
    gzwarn << "No active stream for camera [" << _cameraName
            << "]" << std::endl;
    return;
  }

  it->second->Stop();

  // Unsubscribe from sensor topic to let sensor go idle
  auto topicIt = this->sensorTopics.find(_cameraName);
  if (topicIt != this->sensorTopics.end())
  {
    this->node.Unsubscribe(topicIt->second);
    this->sensorTopics.erase(topicIt);
  }

  this->activeStreams.erase(it);

  // Disconnect from PostRender if no more streams
  if (this->activeStreams.empty())
  {
    this->postRenderConn.reset();
    this->scene.reset();
    gzmsg << "All streams stopped, disconnected from PostRender"
           << std::endl;
  }
}

void CameraStreamPrivate::OnPostRender()
{
  // Get scene
  if (!this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
  }
  if (!this->scene || !this->scene->IsInitialized() ||
      this->scene->SensorCount() == 0)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(this->streamMutex);

  auto now = std::chrono::steady_clock::now();

  for (auto &[name, ctx] : this->activeStreams)
  {
    // Lazy-initialize camera pointer
    if (!ctx->camera)
    {
      // Try exact name first, then search all sensors for a
      // suffix match. The requested name might be a topic like
      // "X3/front_camera" while the rendering sensor is named
      // "sensor_pod::pod_link::front_camera". We try matching
      // the full name, then the last segment after any "/".
      rendering::SensorPtr sensor;
      sensor = this->scene->SensorByName(name);

      // Extract the short name (last segment after /)
      std::string shortName = name;
      auto slashPos = name.rfind('/');
      if (slashPos != std::string::npos)
        shortName = name.substr(slashPos + 1);

      if (!sensor)
      {
        for (unsigned int i = 0; i < this->scene->SensorCount(); ++i)
        {
          auto s = this->scene->SensorByIndex(i);
          if (!s) continue;

          std::string sName = s->Name();
          // Match full name or short name as suffix after "::"
          if (sName == name || sName == shortName)
          {
            sensor = s;
          }
          else if (sName.size() > shortName.size() &&
                   sName.substr(sName.size() - shortName.size()) == shortName &&
                   sName[sName.size() - shortName.size() - 1] == ':')
          {
            sensor = s;
          }

          if (sensor)
          {
            gzmsg << "Matched camera [" << name << "] to sensor ["
                   << sName << "]" << std::endl;
            break;
          }
        }
      }
      if (!sensor)
      {
        continue;
      }
      ctx->camera = std::dynamic_pointer_cast<rendering::Camera>(sensor);
      if (!ctx->camera)
      {
        gzerr << "Sensor [" << name << "] is not a camera" << std::endl;
        continue;
      }

      unsigned int width = ctx->camera->ImageWidth();
      unsigned int height = ctx->camera->ImageHeight();

      // Set actual camera dimensions before starting encoder
      ctx->width = width;
      ctx->height = height;

      gzmsg << "Found camera [" << name << "] with resolution "
             << width << "x" << height << std::endl;

      if (!ctx->IsRunning())
      {
        ctx->Start();
      }

      // Subscribe to camera topic to keep sensor active
      std::string sensorTopic = "/" + name;
      this->node.Subscribe(sensorTopic,
          &CameraStreamPrivate::OnImage, this);
      this->sensorTopics[name] = sensorTopic;
    }

    if (!ctx->camera || !ctx->IsRunning())
      continue;

    // Copy frame from camera
    unsigned int width = ctx->camera->ImageWidth();
    unsigned int height = ctx->camera->ImageHeight();

    if (ctx->cameraImage.Width() != width ||
        ctx->cameraImage.Height() != height)
    {
      ctx->cameraImage = ctx->camera->CreateImage();
    }

    ctx->camera->Copy(ctx->cameraImage);
    const unsigned char *imgData = ctx->cameraImage.Data<unsigned char>();
    unsigned int channels = ctx->cameraImage.Depth();
    if (imgData && channels > 0)
    {
      ctx->PushFrame(imgData, width, height, channels, now);
    }
  }

  // Reap failed streams (encoder detected write failures)
  std::vector<std::string> toRemove;
  for (auto &[name, ctx] : this->activeStreams)
  {
    if (ctx->HasFailed())
    {
      gzmsg << "Reaping failed stream for camera [" << name << "]"
             << std::endl;
      toRemove.push_back(name);
    }
  }
  for (const auto &name : toRemove)
  {
    auto &ctx = this->activeStreams[name];
    ctx->Stop();
    auto topicIt = this->sensorTopics.find(name);
    if (topicIt != this->sensorTopics.end())
    {
      this->node.Unsubscribe(topicIt->second);
      this->sensorTopics.erase(topicIt);
    }
    this->activeStreams.erase(name);
  }
  if (this->activeStreams.empty() && !toRemove.empty())
  {
    this->postRenderConn.reset();
    this->scene.reset();
    gzmsg << "All streams stopped, disconnected from PostRender"
           << std::endl;
  }
}

CameraStream::CameraStream()
  : System(), dataPtr(std::make_unique<CameraStreamPrivate>())
{
}

CameraStream::~CameraStream()
{
  // Stop all streams
  for (auto &[name, ctx] : this->dataPtr->activeStreams)
  {
    ctx->Stop();
  }
  this->dataPtr->activeStreams.clear();
}

void CameraStream::Configure(
    const Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/, EventManager &_eventMgr)
{
  this->dataPtr->eventMgr = &_eventMgr;

  if (_sdf->HasElement("topic"))
  {
    this->dataPtr->controlTopic = transport::TopicUtils::AsValidTopic(
        _sdf->Get<std::string>("topic"));
  }

  this->dataPtr->defaultBitrate = _sdf->Get<unsigned int>(
      "default_bitrate", this->dataPtr->defaultBitrate).first;

  this->dataPtr->defaultFps = _sdf->Get<unsigned int>(
      "default_fps", this->dataPtr->defaultFps).first;

  // Subscribe to control topic (for gz topic CLI and MediaMTX hooks)
  this->dataPtr->node.Subscribe(this->dataPtr->controlTopic,
      &CameraStreamPrivate::OnControlMessage, this->dataPtr.get());

  // Advertise control service (for WebSocket req from viewer UI)
  this->dataPtr->node.Advertise(this->dataPtr->controlTopic,
      &CameraStreamPrivate::OnControlService, this->dataPtr.get());

  gzmsg << "CameraStream plugin listening on ["
         << this->dataPtr->controlTopic << "] (topic + service)"
         << std::endl;
  gzmsg << "Defaults: bitrate=" << this->dataPtr->defaultBitrate
         << " fps=" << this->dataPtr->defaultFps << std::endl;
}

void CameraStream::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &/*_ecm*/)
{
  this->dataPtr->simTime = _info.simTime;
}

GZ_ADD_PLUGIN(CameraStream,
                    System,
                    CameraStream::ISystemConfigure,
                    CameraStream::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(CameraStream,
                          "gz::sim::systems::CameraStream")
