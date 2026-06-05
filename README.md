# gz-camera-stream

A world-level system plugin for Gazebo Sim that streams H.264 video from camera sensors to a media server via WHIP or RTSP. Frames are captured in the `PostRender` callback, converted from RGB/RGBA to YUV420P, encoded with libx264 (ultrafast/zerolatency), and pushed to the configured endpoint. The plugin is idle by default and activates on demand through a gz-transport control topic.

## Data flow

```
Gazebo (OGRE2 render) -> PostRender -> RGB->YUV420P (swscale) -> H.264 (libx264) -> WHIP or RTSP -> Media Server
```

## Output protocols

The plugin auto-detects the output protocol from the URL scheme provided in the start command:

| URL scheme | Protocol | FFmpeg muxer | Notes |
|-----------|----------|-------------|-------|
| `http://` / `https://` | WHIP | `whip` | WebRTC HTTP Ingest Protocol. Requires FFmpeg 7+ with the WHIP muxer compiled in. Lowest latency path to WebRTC viewers. |
| `rtsp://` | RTSP | `rtsp` | Real Time Streaming Protocol. Available in all FFmpeg builds. Uses TCP transport in container environments for reliability. The receiving server (e.g., MediaMTX) can convert to WebRTC/HLS automatically. |

If the WHIP muxer isn't available in the local FFmpeg build (common with distro packages prior to FFmpeg 7), the plugin logs a clear error and suggests using an `rtsp://` URL instead.

## Requirements

- [Gazebo Sim](https://gazebosim.org/) (Ionic or newer)
- FFmpeg development libraries (`libavcodec`, `libavformat`, `libswscale`, `libavutil`) with libx264
- cmake, pkg-config, C++17 compiler

## Build

```bash
# macOS (Homebrew)
cmake -B build -DCMAKE_PREFIX_PATH=/opt/homebrew
cmake --build build

# Linux
cmake -B build
cmake --build build
```

The build produces `libgz-sim-camera-stream-system.dylib` (macOS) or `.so` (Linux). Point `GZ_SIM_SYSTEM_PLUGIN_PATH` at the directory containing the library.

## Plugin architecture

```
src/
  CameraStream.hh/.cc     World plugin (ISystemConfigure + ISystemPostUpdate)
  StreamContext.hh/.cc     Per-stream FFmpeg encoder + network output
  FrameQueue.hh            Lock-free SPSC ring buffer (render thread -> encoder thread)
```

### CameraStream

Implements `ISystemConfigure` and `ISystemPostUpdate`. On `Configure()`, it reads SDF parameters, subscribes to the control topic (both as a topic subscriber and a service for WebSocket access), and optionally reads `STREAM_PREFIX` and `MEDIAMTX_WHIP_BASE` from the environment. On `PostRender()`, it iterates active streams, lazy-initializes camera pointers by matching sensor names against the rendering scene, copies frames, and pushes them into each stream's ring buffer. Failed streams are reaped automatically after 30 consecutive write failures.

### StreamContext

Each `StreamContext` owns a dedicated encoder thread and the full FFmpeg pipeline: `AVCodecContext` (libx264, ultrafast/zerolatency), `SwsContext` (RGB/RGBA to YUV420P), and `AVFormatContext` (WHIP or RTSP output). The sws context is created lazily on the first frame to detect the actual pixel format (3-channel RGB vs 4-channel RGBA) from the rendering backend. WHIP connections retry up to 3 times with 2-second delays to handle DTLS port reuse after a prior session. RTSP connections use TCP transport by default for reliability in container networks.

### FrameQueue

A 3-slot SPSC ring buffer. The render thread calls `TryPush()` which never blocks - if the encoder is behind, the oldest frame is overwritten (frame dropping over blocking). The encoder thread calls `WaitAndPop()` which blocks on a condition variable until a frame arrives or the timeout expires. Encoder threads sit at near-zero CPU when no frames are being produced.

## SDF configuration

Add the plugin to any world:

```xml
<plugin filename="gz-sim-camera-stream-system"
        name="gz::sim::systems::CameraStream">
  <topic>/stream/control</topic>
  <default_bitrate>4000000</default_bitrate>
  <default_fps>30</default_fps>
  <stream_prefix>my_robot</stream_prefix>
  <mediamtx_base>rtsp://mediamtx:8554</mediamtx_base>
</plugin>
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `topic` | `/stream/control` | gz-transport topic and service for start/stop commands |
| `default_bitrate` | `4000000` | H.264 encoding bitrate in bps |
| `default_fps` | `30` | Encoding framerate |
| `stream_prefix` | *(none)* | Path prefix prepended to all output URLs (e.g., `my_robot` produces `rtsp://host/my_robot/cam1`). Also reads from `STREAM_PREFIX` env var. |
| `mediamtx_base` | *(none)* | Base URL for the media server. When set, the plugin constructs output URLs internally from `<base>/<prefix>/<camera>` instead of requiring the URL in the start command. Also reads from `MEDIAMTX_WHIP_BASE` env var. |

## Stream control

Start and stop streams by publishing `gz.msgs.StringMsg_V` to the control topic:

```bash
# Start via WHIP (requires FFmpeg 7+ with WHIP muxer)
gz topic -t /stream/control -m gz.msgs.StringMsg_V \
  -p 'data: "start" data: "cam1" data: "http://localhost:8889/cam1/whip"'

# Start via RTSP
gz topic -t /stream/control -m gz.msgs.StringMsg_V \
  -p 'data: "start" data: "cam1" data: "rtsp://localhost:8554/cam1"'

# Start with custom bitrate and fps
gz topic -t /stream/control -m gz.msgs.StringMsg_V \
  -p 'data: "start" data: "cam1" data: "rtsp://localhost:8554/cam1" data: "2000000" data: "15"'

# Stop
gz topic -t /stream/control -m gz.msgs.StringMsg_V \
  -p 'data: "stop" data: "cam1"'
```

### Message format

**Start:** `data=["start", "<camera_name>", "<url>", "<bitrate>", "<fps>"]`

- `camera_name` - sensor name to match. Can be a short name (`cam1`), a topic-style path (`X3/front_camera`), or a fully scoped rendering name (`sensor_pod::pod_link::front_camera`).
- `url` - WHIP (`http://...`) or RTSP (`rtsp://...`) endpoint. Optional when `mediamtx_base` is configured.
- `bitrate` / `fps` - optional per-stream overrides.

**Stop:** `data=["stop", "<camera_name>"]`

## Camera name matching

The plugin resolves camera names against the rendering scene in order:

1. Exact match via `Scene::SensorByName()`
2. Short name extraction (last segment after `/`) and exact match
3. Suffix match - scan all sensors for one ending with `::<short_name>`

This lets you refer to cameras by whatever name is convenient. A request for `front_camera` will match `sensor_pod::pod_link::front_camera` in the scene. Matches are logged at `msg` level.

## Design decisions

- **One encoder thread per stream** - no stream waits on another's encode cycle. Threads sleep on a condition variable between frames.
- **Frame dropping over blocking** - the 3-frame ring buffer overwrites the oldest frame if the encoder falls behind, rather than stalling the render thread.
- **WHIP with RTSP fallback** - WHIP gives the lowest-latency path to WebRTC but requires a newer FFmpeg. RTSP works everywhere and most media servers (MediaMTX, Wowza, etc.) convert it to WebRTC on the fly.
- **Write-failure auto-stop** - 30 consecutive write failures trigger automatic stream teardown. Handles the media server going away without leaking encoder threads.
- **Lazy sws context** - pixel format detection is deferred to the first frame so the plugin adapts to whatever the rendering backend produces (RGB or RGBA) without configuration.

## License

Apache License 2.0
