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
#ifndef GZ_SIM_SYSTEMS_CAMERA_STREAM_STREAMCONTEXT_HH_
#define GZ_SIM_SYSTEMS_CAMERA_STREAM_STREAMCONTEXT_HH_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <gz/sim/config.hh>
#include <gz/rendering/Camera.hh>

#include "FrameQueue.hh"

struct AVCodecContext;
struct AVFormatContext;
struct AVFrame;
struct AVPacket;
struct AVStream;
struct SwsContext;

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  /// \brief Manages a single video stream from a camera sensor to a
  /// MediaMTX endpoint. Each instance owns a dedicated encoder thread
  /// that reads frames from the queue, encodes H.264, and pushes via
  /// WHIP (or RTSP fallback).
  class StreamContext
  {
    /// \brief Constructor
    /// \param[in] _cameraName Name of the camera sensor
    /// \param[in] _url WHIP or RTSP endpoint URL
    /// \param[in] _width Frame width
    /// \param[in] _height Frame height
    /// \param[in] _fps Frames per second
    /// \param[in] _bitrate Encoding bitrate in bps
    public: StreamContext(const std::string &_cameraName,
                          const std::string &_url,
                          unsigned int _width,
                          unsigned int _height,
                          unsigned int _fps,
                          unsigned int _bitrate);

    /// \brief Destructor - stops encoder thread and cleans up FFmpeg
    public: ~StreamContext();

    // Non-copyable
    public: StreamContext(const StreamContext &) = delete;
    public: StreamContext &operator=(const StreamContext &) = delete;

    /// \brief Start the encoder thread
    /// \return true on success
    public: bool Start();

    /// \brief Stop the encoder thread and tear down FFmpeg
    public: void Stop();

    /// \brief Push a frame into the queue (called from render thread)
    /// \param[in] _data Raw RGB pixel data
    /// \param[in] _width Frame width
    /// \param[in] _height Frame height
    /// \param[in] _channels Bytes per pixel (3=RGB, 4=RGBA)
    /// \param[in] _timestamp Frame timestamp
    public: void PushFrame(const uint8_t *_data, unsigned int _width,
                           unsigned int _height, unsigned int _channels,
                           std::chrono::steady_clock::time_point _timestamp);

    /// \brief Whether the stream is actively encoding
    public: bool IsRunning() const;

    /// \brief Whether the stream has failed (write errors to MediaMTX)
    public: bool HasFailed() const;

    /// \brief The camera pointer (set externally by CameraStream plugin)
    public: rendering::CameraPtr camera;

    /// \brief Reusable image buffer for camera->Copy()
    public: rendering::Image cameraImage;

    /// \brief Name of the camera sensor
    public: std::string cameraName;

    private: void EncoderLoop();
    private: bool InitFFmpeg();
    private: void CleanupFFmpeg();

    public: std::string url;
    public: unsigned int width;
    public: unsigned int height;
    private: unsigned int fps;
    private: unsigned int bitrate;

    private: FrameQueue frameQueue{3};
    private: std::thread encoderThread;
    private: std::atomic<bool> running{false};
    private: std::atomic<bool> failed{false};
    private: int consecutiveWriteFailures = 0;

    private: AVCodecContext *codecCtx = nullptr;
    private: AVFormatContext *fmtCtx = nullptr;
    private: AVStream *stream = nullptr;
    private: AVFrame *yuvFrame = nullptr;
    private: AVPacket *pkt = nullptr;
    private: SwsContext *swsCtx = nullptr;

    private: int64_t frameCount = 0;
    private: std::chrono::steady_clock::time_point firstFrameTime;
    private: bool firstFrame = true;
  };
}
}
}
}
#endif
