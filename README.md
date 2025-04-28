# PDrAW - Parrot Drones Audio and Video Vector

_PDrAW_ (pronounced like the name "Pedro") is an audio and video pipeline
building library supporting Parrot _ANAFI_ drones. It supports MP4 recordings,
RTP/RTSP streaming with H.264 and H.265 video and AAC audio.

_PDrAW_ was formerly _Parrot Drones Awesome Video Viewer_ when it was only a
video player. It was originally written by Aurélien Barre as a personal project
and is now officially maintained by Parrot Drones SAS.

_PDrAW_ is the main video pipeline implementation of _Ground SDK_ used in the
_FreeFlight_ applications, both on iOS and Android. It is also used on Parrot's
drones and remotes for video pipelines implementations.

_PDrAW_ is also available on desktop platforms (Linux and macOS) as part of the
Parrot Drones SDK.

## Supported platforms

* Linux PC
* macOS
* Windows
* Android (5.0 minimum) (note: for sample code, see _GroundSDK Android_)
* iOS (8.0 minimum) (note: for sample code, see _GroundSDK iOS_)

## Features

* Demuxing
  * Record demuxer
    * Playback of local replays
    * MP4 file format (ISO/IEC 14496-12, ISO Base Media File Format /
    ISO/IEC 14496-14, MP4 File Format)
    * Multi-track support on the video and audio with user selection
    * Chapter support (ISO/IEC 14496-12, 'chap' track)
  * Stream demuxer
    * Live video and streamed replays playback
    * RTP/AVP/UDP streams (RFC 3550, RFC 3551)
    * RTP/AVP/MUX streams with SkyController remotes (using Parrot _libmux_)
    * Unicast only
    * RTSP 1.0 protocol (RFC 2326)
    * Multi-track support on the video in RTSP with user selection
  * Playback control
    * Play/pause
    * Seeking (except for live streams)
    * Playback speed control (except for live streams)
    * Negative speeds for playing backward (except for live streams)
    * Frame-by-frame forward and backward on local replays (MP4 records)
  * Support of Parrot session and frame metadata (see _libvideo_metadata_)
* Decoding
  * Video
    * H.264 video decoding (ITU-T H.264 / ISO/IEC 14496-10), baseline, main
    and high profiles
    * H.265 video decoding (ITU-T H.265 / ISO/IEC 23008-2), main and main 10
    profiles
    * Frame output API for application-side processing on the video (either
     H.264 frames before decoding or YUV frames after decoding)
  * Audio
    * AAC audio decoding (ISO/IEC 13818-7), Low Complexity (LC) profile
* Rendering
  * Video
    * OpenGL video rendering
    * Imaging features:
      * Overexposure zebras
      * RGB and luminance histograms computation
    * User rendering callback functions
      * Custom texture loading with provided video metadata
      * Video overlay with provided video metadata
  * Audio
    * ALSA audio rendering
* Encoding
  * Video
    * H.264 video encoding (x264, ITU-T H.264 / ISO/IEC 14496-10), main profile
    * H.265 video encoding (x265, ITU-T H.265 / ISO/IEC 23008-2), main and
    main 10 profiles
  * Audio
    * AAC audio encoding (Fraunhofer FDK AAC, ISO/IEC 14496-3),
    Low Complexity (LC) profile
* Muxing
  * Record muxer
    * Local recording of replays
    * MP4 file format (ISO/IEC 14496-12, ISO Base Media File Format /
    ISO/IEC 14496-14, MP4 File Format)
    * Multi-track support for video and audio
    * Chapter support (ISO/IEC 14496-12, 'chap' track)
  * Stream muxer
    * Live video and audio streaming (fake audio)
    * RTMP streams (RTMP specification, AMF0/AMF3)
* Sources
  * Video
    * Parrot Video IPC (VIPC) source (I420, N12, NV21)
    * External coded source (H.264 byte stream, H.264 AVCC, H265 byte stream,
    H.265 AVCC)
    * External raw source
  * Audio
    * External audio (raw or coded) source (PCM 16B, AAC LC 16B)
* Sinks
  * Video
    * External coded sink (H.264 byte stream, H.264 AVCC, H265 byte stream,
    H.265 AVCC)
    * External raw sink (I420, NV12, RAW8, RAW16)
  * Audio
    * ALSA source
    * External audio sink (raw or coded) source (PCM 16B, AAC LC 16B)

## Software architecture and APIs

### Main library: _libpdraw_

PDrAW is mainly _libpdraw_.

#### Available APIs

Available APIs for *libpdraw* are:

* C
* C++

#### Dependencies

The library depends on the following Alchemy modules:

* eigen
* libaac
* libaudio-decode
* libaudio-defs
* libaudio-encode
* libaudio-encode-core
* libfutils
* libh264
* libh265
* libmedia-buffers
* libmedia-buffers-memory
* libmedia-buffers-memory-generic
* libmp4
* libpomp
* librtp
* librtsp
* libsdp
* libtransport-packet
* libtransport-socket
* libulog
* libvideo-decode
* libvideo-defs
* libvideo-encode
* libvideo-encode-core
* libvideo-metadata
* libvideo-scale
* libvideo-scale-core
* libvideo-streaming
* (optional) alsa-lib (for AlsaSource and AlsaAudioRenderer support)
* (optional) json
* (optional) libmedia-buffers-memory-hisi
* (optional) libmedia-buffers-memory-ion
* (optional) libmux (for StreamDemuxerMux support)
* (optional) librtmp (for RtmpStreamMuxer support)
* (optional) libvideo-ipc (for VipcSource support)
* (optional) libvideo-ipc-client-config (for VipcSource support)
* (optional) libvideo-ipc-dmabuf-be (for VipcSource support)
* (optional) libvideo-ipc-hisibe (for VipcSource support)
* (optional) libvideo-ipc-network-cbuf-be (for VipcSource support)
* (optional) libvideo-ipc-network-hisi-be (for VipcSource support)
* (optional) libvideo-ipc-shmbe (for VipcSource support)
* (optional) opengl (for GlVideoRenderer support)

#### Threading model

The library is designed to run on a _libpomp_ event loop (_pomp_loop_, see
_libpomp_ documentation), except for rendering functions. All API functions
must be called from the _pomp_loop_ thread and all callback functions are
called from the _pomp_loop_ thread, except for rendering function which must
be called on the rendering (OpenGL) thread and rendering callback functions
which are called also on the rendering thread. _The RecordMuxer_ element runs
on a dedicated internal thread to optimize MP4 writing efficiency.

### Helper libraries

#### libpdraw-backend

_libpdraw-backend_ is a wrapper for _libpdraw_'s API that allows calling
functions from any thread, with the exception of rendering functions which
must still be called from the rendering thread.

Available APIs for _libpdraw-backend_ are:

* C
* C++

The library depends on the following Alchemy modules:

* libfutils
* libpdraw
* libpomp
* libulog

#### libpdraw-vsink

_libpdraw-vsink_ is a helper library to easily create a _libpdraw_ instance
on a stream or record without rendering and get YUV frames, for example to
process them with OpenCV in an application.

The API is available in C. The functions can be called from any thread but
are not thread-safe: they should always be called from the same thread,
or it is the caller's responsibility to synchronize calls if multiple threads
are used.

The _libpdraw-vsink_ module provides two methods for retrieving frames:
* Synchronous retrieval using the _pdraw_vsink_get_frame()_ function.
  * In blocking mode (timeout_ms = -1 or timeout_ms > 0)
  * In non-blocking mode (polling, with timeout_ms = 0),
* Asynchronous retrieval by registering a _get_frame_cb_t_ callback.
  Note: This callback is executed from the _pdraw_vsink_ thread. The caller must
  ensure thread safety and be aware that the _pdraw_vsink_ thread is blocked
  during the callback execution.

The library depends on the following Alchemy modules:

* libfutils
* libmedia-buffers
* libmedia-buffers-memory
* libmedia-buffers-memory-generic
* libpdraw
* libpomp
* libulog

#### libpdraw-gles2hud

_libpdraw-gles2hud_ is a helper library to draw an OpenGL ES 2.0 example HUD
(Head-Up Display) on top of the rendered video using _libpdraw_'s rendering
overlay callback function.

The API is available in C. All functions must be called from the rendering
thread.

The library depends on the following Alchemy modules:

* eigen
* libpdraw
* libulog
* libvideo-metadata
* opengl

#### qpdraw

_qpdraw_ is a helper library to easily create a _Qt_ application using
_libpdraw-backend_. The following classes are provided:

* QPdraw (QObject), a _libpdraw-backend_ wrapper.
* QPdrawDemuxer (QObject), a demuxer wrapper
* QpdrawWidget (QOpenGLWidget), a renderer wrapper

The API is available in C++.

The library depends on the following Alchemy modules:

* libfutils
* libpdraw-backend
* libulog

### Applications

Applications are built on top of _libpdraw_, _libpdraw-backend_,
_libpdraw-vsink_ or *qpdraw*.

#### pdraw-desktop

_pdraw-desktop_ is Linux PC, macOS and Windows application written in C
using SDL2 for windowed display and the UI.

The application depends on the following Alchemy modules:

* eigen
* libfutils
* libmedia-buffers
* libpdraw
* libpdraw-backend
* libpdraw-gles2hud
* libpomp
* libulog
* libvideo-defs
* libvideo-metadata
* sdl2
