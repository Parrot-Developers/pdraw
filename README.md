# PDrAW - Parrot Drones Awesome Video Viewer

PDrAW is a viewer for videos produced by Parrot Drones
(Bebop, Bebop2, Disco, etc.).  
It supports both streamed (RTP) and recorded (MP4) videos.

## Supported platforms

* Linux PC
* MacOS X
* Android (4.2 minimum)
* iOS (8.0 minimum)
* RaspberryPi

## Features

* Demuxing
    * Record demuxer
        * MP4 file format (ISO/IEC 14496-12, ISO Base Media File Format /
        ISO/IEC 14496-14, MP4 File Format)
    * Stream demuxer
        * RTP/AVP/UDP streams (RFC 3550, RFC 3551)
        * Unicast or multicast
        * RTSP protocol (RFC 2326)
        * Opening directly from a SDP (RFC 4566) document
    * Playback control
        * Play/pause
        * Seeking (except for live streams)
        * Playback speed control (except for live streams)
        * Negative speeds for playing backward (except for live streams)
        * Frame-by-frame forward and backward (except for live streams)
    * Support of Parrot session and frame metadata
* Decoding
    * H.264 video decoding (ITU-T H.264 / ISO/IEC 14496-10), baseline, main
    and high profiles
    * Frame output API for application-side processing on the video
* Rendering
    * OpenGL ES 2.0 video rendering
    * HMD distorsion correction
        * Support for Parrot Cockpitglasses 1 & 2
        * Device settings for screen density and mechanical margin
        * User settings for video scale, placement and IPD
    * HUD: real-time flight instruments
    * Headtracking
        * Uses Google VR SDK (Android only)
        * Fake headtracking on Linux/MacOS app using the mouse

## Software architecture and APIs

PDrAW is mainly *libpdraw*, with platform-specific applications built on top.  

### Available applications

Available applications built on top of *libpdraw*:

* *pdraw_linux*: PC-Linux application using SDL2 for windowed display
* *pdraw_macos*: MacOS X application using SDL2 for windowed display
* *pdraw_raspi*: RaspberryPi application using EGL/dispmanx for display
* *pdraw_android*: Android application
* *pdraw_ios*: iOS application

### Available APIs

Available APIs for *libpdraw* are:

* C
* C++
* Java for Android (through JNI)
* Python (using SWIG)

### Software dependencies

TBD

## Building PDrAW

### Building on Ubuntu 16.04+

Required packages:

    $ sudo apt-get install build-essential yasm cmake libtool libc6 libc6-dev
    unzip wget freeglut3-dev libglfw3-dev libsdl2-dev libjson-c-dev
    libcurl4-gnutls-dev libavahi-client-dev

Get the sources:

    $ repo init -u https://github.com/Akaaba/pdraw_manifest.git
    $ repo sync

Compiling for a Linux PC:

    $ ./build.sh -p pdraw-linux -t build -j

To use hardware H.264 decoding (NVDec) with NVidia GPUs, the following packages
are also required:

    $ sudo apt-get install libnuma1 libnuma-dev nvidia-cuda-toolkit

An updated version of CUDA Toolkit may be necessary:
https://developer.nvidia.com/cuda-downloads

Enable *cuvid* in the build configuration

    $ ./build.sh -p pdraw-linux -t xconfig
        -> search for "cuvid" and enable
    $ ./build.sh -p pdraw-linux -t build -j

Note: FFmpeg with *cuvid* uses non-free software; PDrAW cannot be distributed
with *cuvid* enabled.