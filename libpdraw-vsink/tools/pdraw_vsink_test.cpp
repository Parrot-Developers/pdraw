/**
 * Parrot Drones Audio and Video Vector
 * Video sink wrapper library test program (C++)
 *
 * Copyright (c) 2026 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Same feature parity as tools/pdraw_vsink_test.c (sync polling and async
 * frame_ready modes), but driving the new public C++ interface
 * (pdraw_vsink.hpp) directly instead of the C API -- notably, there's no
 * explicit "stop" call anywhere below: the instance is held in a
 * std::unique_ptr<IPdrawVsink>, so it stops and cleans up automatically
 * when that goes out of scope.
 */

#include <errno.h>
#include <getopt.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#define ULOG_TAG pdraw_vsink_test
#include <ulog.h>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <media-buffers/mbuf_raw_video_frame.h>
#include <pdraw-vsink/pdraw_vsink.hpp>
#include <pdraw/pdraw.h>
#include <pdraw/pdraw_defs.h>
#include <video-defs/vdefs.h>
#include <video-metadata/vmeta.h>

ULOG_DECLARE_TAG(ULOG_TAG);


/* Can be called from any thread */
static void printCodedFrameInfo(const struct pdraw_video_frame *frameInfo,
				struct mbuf_coded_video_frame *frame,
				int frameIndex)
{
	struct vmeta_frame *frameMeta = nullptr;
	int err = 0;

	ULOGI("frame #%d (width=%u height=%u)",
	      frameIndex,
	      frameInfo->coded.info.resolution.width,
	      frameInfo->coded.info.resolution.height);

	err = mbuf_coded_video_frame_get_metadata(frame, &frameMeta);
	if (err < 0 && err != -ENOENT)
		ULOG_ERRNO("mbuf_coded_video_frame_get_metadata", -err);
	if (frameMeta != nullptr) {
		uint8_t batteryPercentage;
		vmeta_frame_get_battery_percentage(frameMeta,
						   &batteryPercentage);
		ULOGI("metadata: battery_percentage=%d%%", batteryPercentage);
	}

	vmeta_frame_unref(frameMeta);
}


/* Can be called from any thread */
static void printRawFrameInfo(const struct pdraw_video_frame *frameInfo,
			      struct mbuf_raw_video_frame *frame,
			      int frameIndex)
{
	unsigned int planeCount;
	struct vmeta_frame *frameMeta = nullptr;
	int err = 0;

	ULOGI("frame #%d (width=%d height=%d)",
	      frameIndex,
	      frameInfo->raw.info.resolution.width,
	      frameInfo->raw.info.resolution.height);

	err = mbuf_raw_video_frame_get_metadata(frame, &frameMeta);
	if (err < 0 && err != -ENOENT)
		ULOG_ERRNO("mbuf_raw_video_frame_get_metadata", -err);
	if (frameMeta != nullptr) {
		uint8_t batteryPercentage;
		vmeta_frame_get_battery_percentage(frameMeta,
						   &batteryPercentage);
		ULOGI("metadata: battery_percentage=%d%%", batteryPercentage);
	}

	planeCount = vdef_get_raw_frame_plane_count(&frameInfo->raw.format);
	for (unsigned int k = 0; k < planeCount; k++) {
		const void *plane = nullptr;
		size_t planeLen;
		err = mbuf_raw_video_frame_get_plane(
			frame, k, &plane, &planeLen);
		if (err < 0) {
			ULOG_ERRNO(
				"mbuf_raw_video_frame_get_plane(%u)", -err, k);
			continue;
		}
		ULOGI("plane[%d]: addr=%p stride=%zu",
		      k,
		      plane,
		      frameInfo->raw.plane_stride[k]);
		err = mbuf_raw_video_frame_release_plane(frame, k, plane);
		if (err < 0)
			ULOG_ERRNO("mbuf_raw_video_frame_release_plane(%u)",
				   -err,
				   k);
	}

	vmeta_frame_unref(frameMeta);
}


/* IPdrawVsink::Listener is the C++-native equivalent of the C API's
 * struct pdraw_vsink_cbs; onFrameReady() is called from the pdraw_vsink
 * internal thread, same as the C API's frame_ready callback. */
class TestListener : public PdrawVsink::IPdrawVsink::Listener {
public:
	void onFrameReady([[maybe_unused]] PdrawVsink::IPdrawVsink *vsink,
			  struct pdraw_vsink_frame *frame,
			  const struct pdraw_video_frame *frameInfo) override
	{
		if (frame->type == PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED) {
			printCodedFrameInfo(
				frameInfo, frame->coded, mFrameCount.load());
		} else {
			printRawFrameInfo(
				frameInfo, frame->raw, mFrameCount.load());
		}

		mFrameCount.fetch_add(1);
	}

	int getFrameCount() const
	{
		return mFrameCount.load();
	}

private:
	std::atomic_int mFrameCount = 0;
};


static void printMediaInfo(struct pdraw_media_info *mediaInfo,
			   enum pdraw_vsink_video_media_type videoMediaType)
{
	const struct vdef_format_info *info;

	ULOGI("media_info: name=%s, path=%s", mediaInfo->name, mediaInfo->path);

	if (videoMediaType == PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED)
		info = &mediaInfo->video.coded.info;
	else
		info = &mediaInfo->video.raw.info;

	ULOGI("media_info: duration=%.3fs, res=%ux%u, framerate=%u/%u",
	      mediaInfo->duration / 1000000.0,
	      info->resolution.width,
	      info->resolution.height,
	      info->framerate.num,
	      info->framerate.den);

	pdraw_media_info_free(mediaInfo);
}


/* Called from the main thread */
static int asyncTest(const char *url,
		     enum pdraw_playback_mode playbackMode,
		     enum vmeta_camera_type cameraType,
		     int count,
		     enum pdraw_vsink_video_media_type videoMediaType)
{
	struct pdraw_media_info *mediaInfo = nullptr;
	struct pdraw_vsink_params params = {};
	params.url = url;
	params.playback_mode = playbackMode;
	params.camera_type = cameraType;
	params.video_media_type = videoMediaType;

	TestListener listener;
	PdrawVsink::IPdrawVsink *vsinkRaw = nullptr;
	int res = PdrawVsink::createPdrawVsink(
		&params, &listener, &mediaInfo, &vsinkRaw);
	if (res < 0 || mediaInfo == nullptr) {
		ULOG_ERRNO("createPdrawVsink", -res);
		exit(EXIT_FAILURE);
	}
	/* No explicit "stop" call anywhere in this function: vsink stops and
	 * cleans up automatically once this unique_ptr goes out of scope. */
	std::unique_ptr<PdrawVsink::IPdrawVsink> vsink(vsinkRaw);
	ULOGI("started");

	printMediaInfo(mediaInfo, videoMediaType);

	while (listener.getFrameCount() < count)
		std::this_thread::sleep_for(std::chrono::milliseconds(1));

	return 0;
}


/* Called from the main thread */
static int syncTest(const char *url,
		    enum pdraw_playback_mode playbackMode,
		    enum vmeta_camera_type cameraType,
		    int timeoutMs,
		    int count,
		    enum pdraw_vsink_video_media_type videoMediaType)
{
	int i = 0;
	struct pdraw_media_info *mediaInfo = nullptr;
	struct pdraw_vsink_params params = {};
	params.url = url;
	params.playback_mode = playbackMode;
	params.camera_type = cameraType;
	params.video_media_type = videoMediaType;

	/* No listener: getFrame() polling mode below. */
	PdrawVsink::IPdrawVsink *vsinkRaw = nullptr;
	int res = PdrawVsink::createPdrawVsink(
		&params, nullptr, &mediaInfo, &vsinkRaw);
	if (res < 0 || mediaInfo == nullptr) {
		ULOG_ERRNO("createPdrawVsink", -res);
		exit(EXIT_FAILURE);
	}
	std::unique_ptr<PdrawVsink::IPdrawVsink> vsink(vsinkRaw);
	ULOGI("started");

	printMediaInfo(mediaInfo, videoMediaType);

	while (i < count) {
		struct pdraw_video_frame frameInfo = {};
		struct pdraw_vsink_frame vframe;

		res = vsink->getFrame(timeoutMs, nullptr, &frameInfo, &vframe);
		if (res < 0) {
			ULOG_ERRNO("getFrame", -res);
			std::this_thread::sleep_for(
				std::chrono::milliseconds(1));
			continue;
		}
		if (vframe.type == PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED) {
			printCodedFrameInfo(&frameInfo, vframe.coded, i);
			mbuf_coded_video_frame_unref(vframe.coded);
		} else {
			printRawFrameInfo(&frameInfo, vframe.raw, i);
			mbuf_raw_video_frame_unref(vframe.raw);
		}
		i++;
	}

	return 0;
}


enum class ArgsId {
	NO_DECODE = 256,
};


static const char shortOptions[] = "ht:oc:n:";


static const std::array<struct option, 7> longOptions = {{
	{"help", no_argument, nullptr, 'h'},
	{"offline", no_argument, nullptr, 'o'},
	{"camera-type", required_argument, nullptr, 'c'},
	{"count", required_argument, nullptr, 'n'},
	{"timeout", required_argument, nullptr, 't'},
	{"no-decode",
	 no_argument,
	 nullptr,
	 static_cast<int>(ArgsId::NO_DECODE)},
	{nullptr, 0, nullptr, 0},
}};


static void usage(char *progName)
{
	/* clang-format off */
	printf("Usage: %s [OPTIONS] filepath\n"
	       "Options:\n"
	       "  -h | --help                          "
		       "Print this message\n"
	       "  -o | --offline                       "
		       "Offline demuxing mode (ignore framerate and output "
		       "frames as fast as possible)\n"
	       "  -c | --camera-type <cam>             "
		       "Camera type (e.g. 'front', 'horizontal-stereo-left', "
		       "'down-stereo-right'; default camera if not specified)\n"
	       "  -n | --count <n>                     "
		       "Process at most n frames\n"
	       "  -t | --timeout <num>                 "
		       "Set the maximum time to wait (in ms) to get a frame, "
		       "0 to return immediately (non-blocking mode) or -1 for "
		       "infinite wait\n"
	       "     | --no-decode                     "
		       "Don't decode the frames\n"
	       "\n",
	       progName);
	/* clang-format on */
}


int main(int argc, char **argv)
{
	int status = EXIT_SUCCESS;
	int res;
	int idx;
	int c;
	const char *url = nullptr;
	enum vmeta_camera_type cameraType = VMETA_CAMERA_TYPE_UNKNOWN;
	int timeoutMs = -1;
	int count = 20;
	enum pdraw_playback_mode playbackMode = PDRAW_PLAYBACK_MODE_REALTIME;
	enum pdraw_vsink_video_media_type videoMediaType =
		PDRAW_VSINK_VIDEO_MEDIA_TYPE_RAW;
	bool shouldQuit = false;

	while ((c = getopt_long(
			argc, argv, shortOptions, longOptions.data(), &idx)) !=
	       -1) {
		switch (c) {

		case 'h':
			usage(argv[0]);
			shouldQuit = true;
			break;

		case 'o':
			playbackMode = PDRAW_PLAYBACK_MODE_OFFLINE;
			break;

		case 'c':
			cameraType = vmeta_camera_type_from_str(optarg);
			break;

		case 'n':
			sscanf(optarg, "%d", &count);
			if (count <= 0) {
				usage(argv[0]);
				status = EXIT_FAILURE;
				shouldQuit = true;
				break;
			}
			break;

		case 't':
			sscanf(optarg, "%d", &timeoutMs);
			break;

		case static_cast<int>(ArgsId::NO_DECODE):
			videoMediaType = PDRAW_VSINK_VIDEO_MEDIA_TYPE_CODED;
			break;

		default:
			usage(argv[0]);
			status = EXIT_FAILURE;
			shouldQuit = true;
			break;
		}
	}

	if (shouldQuit)
		goto out;

	if (argc - optind < 1) {
		usage(argv[0]);
		goto out;
	}

	url = argv[optind];
	if (url == nullptr) {
		usage(argv[0]);
		goto out;
	}

	/* Sync mode: polling using IPdrawVsink::getFrame() */
	res = syncTest(url,
		       playbackMode,
		       cameraType,
		       timeoutMs,
		       count,
		       videoMediaType);
	if (res < 0)
		ULOG_ERRNO("syncTest", -res);

	/* Async mode: notify using the frame_ready callback */
	res = asyncTest(url, playbackMode, cameraType, count, videoMediaType);
	if (res < 0)
		ULOG_ERRNO("asyncTest", -res);

	ULOGI("%s", (status == EXIT_SUCCESS) ? "success!" : "failed!");

out:
	exit(status);
}
