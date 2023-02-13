# PDrAW - Parrot Drones Awesome Video Viewer

_PDrAW_ (pronounced like the name "Pedro") is a video player for medias created
by Parrot drones such as _Anafi_ or _Anafi Ai_. The player supports both
streamed (RTP/RTSP) and recorded (MP4) videos.

PDrAW was originally written by Aurélien Barre as a personal project and is
now officially maintained by Parrot Drones SAS. _PDrAW_ is the main video
pipeline implementation of _GroundSDK_. We are using it in our _FreeFlight6_
and _FreeFlight7_ applications, both on iOS and Android. It is also usable on
desktop platforms.

## Supported platforms

* Linux PC
* macOS
* Android (5.0 minimum) (note: for sample code, see _GroundSDK Android_)
* iOS (8.0 minimum) (note: for sample code, see _GroundSDK iOS_)

## Features

* Demuxing
  * Record demuxer
    * Playback of local replays
    * MP4 file format (ISO/IEC 14496-12, ISO Base Media File Format /
    ISO/IEC 14496-14, MP4 File Format)
    * Multi-track support on the video with user selection
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
  * H.264 video decoding (ITU-T H.264 / ISO/IEC 14496-10), baseline, main
  and high profiles
  * H.265 video decoding (ITU-T H.265 / ISO/IEC 23008-2), main and main 10
  profiles
  * Frame output API for application-side processing on the video (either
  H.264 frames before decoding or YUV frames after decoding)
* Rendering
  * OpenGL ES 2.0 video rendering
  * Imaging features:
    * Overexposure zebras
    * RGB and luminance histograms computation
  * User rendering callback functions
    * Custom texture loading with provided video metadata
    * Video overlay with provided video metadata
  * HMD distortion correction
    * Support for Parrot Cockpitglasses 1 & 2
    * Device settings for screen pixel density and mechanical margins
    * User settings for video scale, placement and IPD

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
* libfutils
* libh264
* libh265
* libmp4
* libpomp
* librtsp
* libsdp
* libulog
* libvideo-buffers
* libvideo-buffers-generic
* libvideo-decode
* libvideo-metadata
* libvideo-streaming
* json (optional)
* libmux (optional)
* glfw3 (only on Linux and macOS)
* opengl (only on Linux)

#### Threading model

The library is designed to run on a _libpomp_ event loop (_pomp_loop_, see
_libpomp_ documentation), except for rendering functions. All API functions
must be called from the _pomp_loop_ thread and all callback functions are
called from the _pomp_loop_ thread, except for rendering function which must
be called on the rendering (OpenGL) thread and rendering callback functions
which are called also on the rendering thread.

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

The library depends on the following Alchemy modules:

* libfutils
* libpdraw
* libpomp
* libulog
* libvideo-buffers
* libvideo-buffers-generic
* libvideo-metadata

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
* glfw3 (only on Linux and macOS)
* opengl (only on Linux)

### Applications

Applications are built on top of _libpdraw_ or _libpdraw-backend_.

#### pdraw-desktop

_pdraw-desktop_ is Linux PC and macOS application written in C using SDL2 for
windowed display and the UI.

The application depends on the following Alchemy modules:

* eigen
* libpdraw
* libpdraw-backend
* libpdraw-gles2hud
* libpomp
* libulog
* libvideo-metadata
* sdl2
* glfw3
* opengl (only on Linux)
