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
#ifndef GZ_SIM_SYSTEMS_CAMERA_STREAM_FRAMEQUEUE_HH_
#define GZ_SIM_SYSTEMS_CAMERA_STREAM_FRAMEQUEUE_HH_

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <optional>
#include <vector>

#include <gz/sim/config.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  struct Frame
  {
    std::vector<uint8_t> data;
    unsigned int width = 0;
    unsigned int height = 0;
    unsigned int channels = 4;
    std::chrono::steady_clock::time_point timestamp;
  };

  /// \brief Single-producer single-consumer ring buffer for video frames.
  /// Producer (render thread) uses tryPush which overwrites the oldest
  /// frame if full rather than blocking. Consumer (encoder thread) uses
  /// waitAndPop which blocks on a condition variable until a frame is
  /// available or timeout expires.
  class FrameQueue
  {
    public: explicit FrameQueue(size_t _capacity = 3)
      : capacity(_capacity), buffer(_capacity)
    {
    }

    /// \brief Push a frame, overwriting the oldest if full. Never blocks.
    /// \param[in] _data Raw pixel data
    /// \param[in] _width Frame width in pixels
    /// \param[in] _height Frame height in pixels
    /// \param[in] _channels Bytes per pixel (3 for RGB, 4 for RGBA)
    /// \param[in] _timestamp Frame timestamp
    /// \return true if a frame was overwritten (dropped)
    public: bool TryPush(const uint8_t *_data, unsigned int _width,
                         unsigned int _height, unsigned int _channels,
                         std::chrono::steady_clock::time_point _timestamp)
    {
      {
        std::lock_guard<std::mutex> lock(this->mtx);
        auto &slot = this->buffer[this->writeIdx % this->capacity];
        size_t sz = static_cast<size_t>(_width) * _height * _channels;
        slot.data.resize(sz);
        std::memcpy(slot.data.data(), _data, sz);
        slot.width = _width;
        slot.height = _height;
        slot.channels = _channels;
        slot.timestamp = _timestamp;

        bool dropped = (this->count == this->capacity);
        if (dropped)
        {
          this->readIdx++;
        }
        else
        {
          this->count++;
        }
        this->writeIdx++;
      }
      this->cv.notify_one();
      return false;
    }

    /// \brief Wait for a frame, blocking until one is available or timeout.
    /// \param[in] _timeout Maximum time to wait
    /// \return The frame, or std::nullopt on timeout
    public: std::optional<Frame> WaitAndPop(
        std::chrono::milliseconds _timeout = std::chrono::milliseconds(100))
    {
      std::unique_lock<std::mutex> lock(this->mtx);
      if (!this->cv.wait_for(lock, _timeout,
          [this] { return this->count > 0; }))
      {
        return std::nullopt;
      }

      auto &slot = this->buffer[this->readIdx % this->capacity];
      Frame frame;
      frame.data = std::move(slot.data);
      frame.width = slot.width;
      frame.height = slot.height;
      frame.channels = slot.channels;
      frame.timestamp = slot.timestamp;

      this->readIdx++;
      this->count--;
      return frame;
    }

    /// \brief Number of frames currently in the queue
    public: size_t Size() const
    {
      std::lock_guard<std::mutex> lock(this->mtx);
      return this->count;
    }

    /// \brief Discard all queued frames
    public: void Flush()
    {
      std::lock_guard<std::mutex> lock(this->mtx);
      this->readIdx = 0;
      this->writeIdx = 0;
      this->count = 0;
    }

    private: size_t capacity;
    private: std::vector<Frame> buffer;
    private: size_t readIdx = 0;
    private: size_t writeIdx = 0;
    private: size_t count = 0;
    private: mutable std::mutex mtx;
    private: std::condition_variable cv;
  };
}
}
}
}
#endif
