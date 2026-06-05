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

#include "StreamContext.hh"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <gz/common/Console.hh>

using namespace gz;
using namespace sim;
using namespace systems;

StreamContext::StreamContext(const std::string &_cameraName,
                             const std::string &_url,
                             unsigned int _width,
                             unsigned int _height,
                             unsigned int _fps,
                             unsigned int _bitrate)
  : cameraName(_cameraName),
    url(_url),
    width(_width),
    height(_height),
    fps(_fps),
    bitrate(_bitrate)
{
}

StreamContext::~StreamContext()
{
  this->Stop();
}

bool StreamContext::Start()
{
  if (this->running)
    return true;

  // Retry WHIP connection - the DTLS handshake can fail if a prior
  // session's UDP ports haven't been fully released yet
  for (int attempt = 0; attempt < 3; ++attempt)
  {
    if (this->InitFFmpeg())
    {
      this->running = true;
      this->failed = false;
      this->encoderThread = std::thread(&StreamContext::EncoderLoop, this);

      gzmsg << "Started video stream for camera [" << this->cameraName
             << "] -> [" << this->url << "]" << std::endl;
      return true;
    }

    this->CleanupFFmpeg();
    if (attempt < 2)
    {
      gzwarn << "WHIP connection failed for [" << this->cameraName
              << "], retrying in 2s (attempt " << (attempt + 1)
              << "/3)..." << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
  }

  gzerr << "Failed to initialize stream for camera ["
         << this->cameraName << "] after 3 attempts" << std::endl;
  return false;
}

void StreamContext::Stop()
{
  bool wasRunning = this->running.exchange(false);

  if (this->encoderThread.joinable())
    this->encoderThread.join();

  // Only write trailer if the stream was cleanly running (not failed)
  if (wasRunning && !this->failed && this->fmtCtx)
  {
    av_write_trailer(this->fmtCtx);
  }

  this->CleanupFFmpeg();

  if (wasRunning)
  {
    gzmsg << "Stopped video stream for camera [" << this->cameraName
           << "]" << std::endl;
  }
}

void StreamContext::PushFrame(const uint8_t *_data, unsigned int _width,
                               unsigned int _height, unsigned int _channels,
                               std::chrono::steady_clock::time_point _timestamp)
{
  static bool pushLogged = false;
  if (!pushLogged)
  {
    gzmsg << "PushFrame: " << _width << "x" << _height
           << " channels=" << _channels << std::endl;
    pushLogged = true;
  }
  this->frameQueue.TryPush(_data, _width, _height, _channels, _timestamp);
}

bool StreamContext::IsRunning() const
{
  return this->running;
}

bool StreamContext::HasFailed() const
{
  return this->failed;
}

bool StreamContext::InitFFmpeg()
{
  // Detect output format from URL scheme
  const char *formatName = nullptr;
  if (this->url.find("http://") == 0 || this->url.find("https://") == 0)
  {
    // WHIP endpoint
    const AVOutputFormat *whipFmt = av_guess_format("whip", nullptr, nullptr);
    if (whipFmt)
    {
      formatName = "whip";
      gzmsg << "Using WHIP output for [" << this->url << "]" << std::endl;
    }
    else
    {
      gzerr << "WHIP muxer not available in this FFmpeg build. "
             << "Use an rtsp:// URL instead." << std::endl;
      return false;
    }
  }
  else if (this->url.find("rtsp://") == 0)
  {
    formatName = "rtsp";
    gzmsg << "Using RTSP output for [" << this->url << "]" << std::endl;
  }
  else
  {
    gzerr << "Unsupported URL scheme: [" << this->url << "]. "
           << "Use http(s):// for WHIP or rtsp:// for RTSP." << std::endl;
    return false;
  }

  // Allocate output format context
  int ret = avformat_alloc_output_context2(&this->fmtCtx, nullptr,
      formatName, this->url.c_str());
  if (ret < 0 || !this->fmtCtx)
  {
    char errbuf[256];
    av_strerror(ret, errbuf, sizeof(errbuf));
    gzerr << "avformat_alloc_output_context2 failed: " << errbuf << std::endl;
    return false;
  }

  // Find H.264 encoder
  const AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
  if (!codec)
  {
    gzerr << "H.264 encoder not found" << std::endl;
    return false;
  }

  this->stream = avformat_new_stream(this->fmtCtx, nullptr);
  if (!this->stream)
  {
    gzerr << "Failed to create output stream" << std::endl;
    return false;
  }

  this->codecCtx = avcodec_alloc_context3(codec);
  if (!this->codecCtx)
  {
    gzerr << "Failed to allocate codec context" << std::endl;
    return false;
  }

  // Low-latency H.264 settings
  this->codecCtx->bit_rate = this->bitrate;
  this->codecCtx->width = static_cast<int>(this->width);
  this->codecCtx->height = static_cast<int>(this->height);
  this->codecCtx->time_base = {1, static_cast<int>(this->fps)};
  this->codecCtx->framerate = {static_cast<int>(this->fps), 1};
  this->codecCtx->gop_size = static_cast<int>(this->fps);
  this->codecCtx->max_b_frames = 0;
  this->codecCtx->pix_fmt = AV_PIX_FMT_YUV420P;
  this->codecCtx->thread_count = 1;

  // WHIP requires global header
  if (this->fmtCtx->oformat->flags & AVFMT_GLOBALHEADER)
    this->codecCtx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

  av_opt_set(this->codecCtx->priv_data, "preset", "ultrafast", 0);
  av_opt_set(this->codecCtx->priv_data, "tune", "zerolatency", 0);

  ret = avcodec_open2(this->codecCtx, codec, nullptr);
  if (ret < 0)
  {
    char errbuf[256];
    av_strerror(ret, errbuf, sizeof(errbuf));
    gzerr << "avcodec_open2 failed: " << errbuf << std::endl;
    return false;
  }

  ret = avcodec_parameters_from_context(this->stream->codecpar, this->codecCtx);
  if (ret < 0)
  {
    gzerr << "Failed to copy codec params to stream" << std::endl;
    return false;
  }
  this->stream->time_base = this->codecCtx->time_base;

  // Allocate YUV frame
  this->yuvFrame = av_frame_alloc();
  if (!this->yuvFrame)
  {
    gzerr << "Failed to allocate YUV frame" << std::endl;
    return false;
  }
  this->yuvFrame->format = AV_PIX_FMT_YUV420P;
  this->yuvFrame->width = static_cast<int>(this->width);
  this->yuvFrame->height = static_cast<int>(this->height);
  ret = av_frame_get_buffer(this->yuvFrame, 32);
  if (ret < 0)
  {
    gzerr << "Failed to allocate YUV frame buffer" << std::endl;
    return false;
  }

  // Allocate packet
  this->pkt = av_packet_alloc();
  if (!this->pkt)
  {
    gzerr << "Failed to allocate packet" << std::endl;
    return false;
  }

  // sws context created lazily on first frame (need actual channel count)

  // Open output (network connection)
  if (!(this->fmtCtx->oformat->flags & AVFMT_NOFILE))
  {
    ret = avio_open(&this->fmtCtx->pb, this->url.c_str(), AVIO_FLAG_WRITE);
    if (ret < 0)
    {
      char errbuf[256];
      av_strerror(ret, errbuf, sizeof(errbuf));
      gzerr << "avio_open failed for [" << this->url << "]: "
             << errbuf << std::endl;
      return false;
    }
  }

  // Write stream header
  ret = avformat_write_header(this->fmtCtx, nullptr);
  if (ret < 0)
  {
    char errbuf[256];
    av_strerror(ret, errbuf, sizeof(errbuf));
    gzerr << "avformat_write_header failed: " << errbuf << std::endl;
    return false;
  }

  this->frameCount = 0;
  this->firstFrame = true;

  return true;
}

void StreamContext::CleanupFFmpeg()
{
  if (this->swsCtx)
  {
    sws_freeContext(this->swsCtx);
    this->swsCtx = nullptr;
  }

  if (this->pkt)
  {
    av_packet_free(&this->pkt);
    this->pkt = nullptr;
  }

  if (this->yuvFrame)
  {
    av_frame_free(&this->yuvFrame);
    this->yuvFrame = nullptr;
  }

  if (this->codecCtx)
  {
    avcodec_free_context(&this->codecCtx);
    this->codecCtx = nullptr;
  }

  if (this->fmtCtx)
  {
    if (this->fmtCtx->pb &&
        !(this->fmtCtx->oformat->flags & AVFMT_NOFILE))
    {
      avio_closep(&this->fmtCtx->pb);
    }
    avformat_free_context(this->fmtCtx);
    this->fmtCtx = nullptr;
  }

  this->stream = nullptr;
}

void StreamContext::EncoderLoop()
{
  while (this->running)
  {
    auto frame = this->frameQueue.WaitAndPop(std::chrono::milliseconds(100));
    if (!frame)
      continue;

    // Create sws context on first frame (need actual channel count)
    if (!this->swsCtx)
    {
      AVPixelFormat srcFmt = (frame->channels == 4)
          ? AV_PIX_FMT_RGBA : AV_PIX_FMT_RGB24;
      this->swsCtx = sws_getContext(
          static_cast<int>(frame->width), static_cast<int>(frame->height),
          srcFmt,
          static_cast<int>(frame->width), static_cast<int>(frame->height),
          AV_PIX_FMT_YUV420P,
          SWS_POINT, nullptr, nullptr, nullptr);
      if (!this->swsCtx)
      {
        gzerr << "Failed to create sws context" << std::endl;
        this->running = false;
        this->failed = true;
        break;
      }
      gzmsg << "Encoder pixel format: " << (frame->channels == 4 ? "RGBA" : "RGB")
             << " (" << frame->channels << " channels)" << std::endl;
    }

    // Verify dimensions match before scaling
    if (static_cast<int>(frame->width) != this->yuvFrame->width ||
        static_cast<int>(frame->height) != this->yuvFrame->height)
    {
      gzerr << "Frame size mismatch: frame=" << frame->width << "x"
             << frame->height << " encoder=" << this->yuvFrame->width
             << "x" << this->yuvFrame->height << std::endl;
      continue;
    }

    const uint8_t *srcSlice[1] = {frame->data.data()};
    int srcStride[1] = {static_cast<int>(frame->width * frame->channels)};

    int ret = av_frame_make_writable(this->yuvFrame);
    if (ret < 0)
      continue;

    sws_scale(this->swsCtx,
        srcSlice, srcStride, 0, static_cast<int>(frame->height),
        this->yuvFrame->data, this->yuvFrame->linesize);

    // Timestamp based on frame count for consistent timing
    this->yuvFrame->pts = this->frameCount++;

    // Encode
    ret = avcodec_send_frame(this->codecCtx, this->yuvFrame);
    if (ret < 0)
    {
      if (ret != AVERROR(EAGAIN))
      {
        char errbuf[256];
        av_strerror(ret, errbuf, sizeof(errbuf));
        gzerr << "avcodec_send_frame failed: " << errbuf << std::endl;
      }
      continue;
    }

    while (ret >= 0)
    {
      ret = avcodec_receive_packet(this->codecCtx, this->pkt);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
        break;
      if (ret < 0)
      {
        char errbuf[256];
        av_strerror(ret, errbuf, sizeof(errbuf));
        gzerr << "avcodec_receive_packet failed: " << errbuf << std::endl;
        break;
      }

      av_packet_rescale_ts(this->pkt, this->codecCtx->time_base,
                           this->stream->time_base);
      this->pkt->stream_index = this->stream->index;

      ret = av_interleaved_write_frame(this->fmtCtx, this->pkt);
      if (ret < 0)
      {
        this->consecutiveWriteFailures++;
        if (this->consecutiveWriteFailures <= 3)
        {
          char errbuf[256];
          av_strerror(ret, errbuf, sizeof(errbuf));
          gzerr << "av_interleaved_write_frame failed ("
                 << this->consecutiveWriteFailures << "/30): "
                 << errbuf << std::endl;
        }
        // Auto-stop after 30 consecutive failures (~1s at 30fps).
        // This handles MediaMTX going away or network issues.
        if (this->consecutiveWriteFailures >= 30)
        {
          gzerr << "Too many write failures, stopping stream for ["
                 << this->cameraName << "]" << std::endl;
          this->failed = true;
          this->running = false;
          break;
        }
      }
      else
      {
        this->consecutiveWriteFailures = 0;
      }
    }
  }

  // Flush encoder
  avcodec_send_frame(this->codecCtx, nullptr);
  while (true)
  {
    int ret = avcodec_receive_packet(this->codecCtx, this->pkt);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
      break;
    if (ret < 0)
      break;
    av_packet_rescale_ts(this->pkt, this->codecCtx->time_base,
                         this->stream->time_base);
    this->pkt->stream_index = this->stream->index;
    av_interleaved_write_frame(this->fmtCtx, this->pkt);
  }
}
