/**
 * Parrot Drones Audio and Video Vector
 * Video sink wrapper library face detection example (OpenCV)
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
 * Example built on top of the pdraw_vsink.hpp C++ interface (same as
 * tools/pdraw_vsink_test.cpp) demonstrating how to plug OpenCV processing
 * onto the raw decoded frames: runs Haar-cascade face detection on the luma
 * (Y) plane of each frame. Only the async/callback mode is used here (no
 * sync polling), and only the raw (decoded) media type is requested since
 * detection needs actual pixels.
 */

#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>

#define ULOG_TAG pdraw_vsink_test_opencv
#include <ulog.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <media-buffers/mbuf_raw_video_frame.h>
#include <pdraw-vsink/pdraw_vsink.hpp>
#include <pdraw/pdraw.h>
#include <pdraw/pdraw_defs.h>
#include <video-defs/vdefs.h>
#include <video-metadata/vmeta.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

ULOG_DECLARE_TAG(ULOG_TAG);

static std::atomic_int gFrameCount{0};
static cv::CascadeClassifier gFaceCascade;
static std::string gOutputDir;
static std::atomic_bool gWarnedFormat{false};
static int gMinNeighbors = 5;
static int gMinSize = 0;
static int gMinTrackHits = 1;

/* onFrameReady() (and therefore detectFaces()) is always called serially
 * from the vsink's own internal thread (it blocks that thread until it
 * returns), so this cross-frame state needs no locking. */
struct FaceTrack {
	cv::Rect box;
	int hits;
};
static std::vector<FaceTrack> gTracks;


/* Alchemy binaries normally run against a staging tree, not the real root
 * filesystem -- native-wrapper.sh exports PRODUCT_ROOT_CFG pointing there
 * (same convention as libmetadata-provider's get_config_path()), so prefix
 * the default cascade path with it when set. Only applies to the in-tree
 * opencv2 build (atom.mk's PDRAW_VSINK_CASCADE_PATH_STAGED): the system
 * OpenCV's data is never staged, only ever present at its real host path.
 * An explicit --cascade value is used as-is, without this prefix either
 * way. */
static std::string defaultCascadePath()
{
#if PDRAW_VSINK_CASCADE_PATH_STAGED
	const char *root = getenv("PRODUCT_ROOT_CFG");
	if (root != nullptr && root[0] != '\0')
		return std::string(root) + PDRAW_VSINK_DEFAULT_CASCADE_PATH;
#endif
	return PDRAW_VSINK_DEFAULT_CASCADE_PATH;
}


static float iou(const cv::Rect &a, const cv::Rect &b)
{
	cv::Rect inter = a & b;
	if (inter.area() == 0)
		return 0.f;
	return static_cast<float>(inter.area()) /
	       static_cast<float>(a.area() + b.area() - inter.area());
}


/* Matches this frame's raw detections against tracks kept from the previous
 * frame (nearest by bounding-box overlap), bumping each matched track's hit
 * streak and dropping tracks not re-detected this frame. Only tracks with a
 * streak >= gMinTrackHits are returned -- with the default of 1, every
 * detection is returned immediately (no filtering), so this is a no-op
 * unless --min-track-hits is explicitly raised. A deliberately simple
 * heuristic (no gap tolerance, no Kalman/optical-flow prediction) -- enough
 * to suppress single-frame flicker false positives for this example. */
static std::vector<cv::Rect>
updateTracks(const std::vector<cv::Rect> &detections)
{
	std::vector<FaceTrack> updated;
	std::vector<bool> matched(detections.size(), false);

	for (const auto &track : gTracks) {
		int best = -1;
		float bestIou = 0.3f;
		for (size_t i = 0; i < detections.size(); i++) {
			if (matched[i])
				continue;
			float v = iou(track.box, detections[i]);
			if (v > bestIou) {
				bestIou = v;
				best = static_cast<int>(i);
			}
		}
		if (best >= 0) {
			matched[best] = true;
			updated.push_back({detections[best], track.hits + 1});
		}
	}

	for (size_t i = 0; i < detections.size(); i++) {
		if (!matched[i])
			updated.push_back({detections[i], 1});
	}

	gTracks = std::move(updated);

	std::vector<cv::Rect> confirmed;
	for (const auto &track : gTracks) {
		if (track.hits >= gMinTrackHits)
			confirmed.push_back(track.box);
	}
	return confirmed;
}


/* Called from the pdraw_vsink internal thread */
static void detectFaces(const struct pdraw_video_frame *frameInfo,
			struct mbuf_raw_video_frame *frame,
			int frameIndex)
{
	const void *plane = nullptr;
	size_t planeLen;
	int err;

	/* Only the luma (Y) plane is needed for detection; its layout (full
	 * resolution, 8-bit) is the same for I420 and NV12, so there is no
	 * need to special-case either format below plane 0. Any other raw
	 * format is not handled by this example. */
	if (!vdef_raw_format_cmp(&frameInfo->raw.format, &vdef_i420) &&
	    !vdef_raw_format_cmp(&frameInfo->raw.format, &vdef_nv12)) {
		if (!gWarnedFormat.exchange(true))
			ULOGW("unsupported raw format, skipping frames");
		return;
	}

	err = mbuf_raw_video_frame_get_plane(frame, 0, &plane, &planeLen);
	if (err < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_plane", -err);
		return;
	}

	cv::Mat yPlane(frameInfo->raw.info.resolution.height,
		       frameInfo->raw.info.resolution.width,
		       CV_8UC1,
		       const_cast<void *>(plane),
		       frameInfo->raw.plane_stride[0]);

	/* equalizeHist() always writes to a fresh output Mat -- it never
	 * modifies yPlane in place, so the read-only decoder buffer is left
	 * untouched. Improves contrast, which reduces both missed faces and
	 * false positives from low-contrast noise. */
	cv::Mat equalized;
	cv::equalizeHist(yPlane, equalized);

	std::vector<cv::Rect> detections;
	gFaceCascade.detectMultiScale(equalized,
				      detections,
				      1.1,
				      gMinNeighbors,
				      0,
				      cv::Size(gMinSize, gMinSize));
	std::vector<cv::Rect> faces = updateTracks(detections);

	cv::Mat annotated;
	if (!gOutputDir.empty())
		annotated = yPlane.clone();

	err = mbuf_raw_video_frame_release_plane(frame, 0, plane);
	if (err < 0)
		ULOG_ERRNO("mbuf_raw_video_frame_release_plane", -err);

	ULOGI("frame #%d: %zu face(s)", frameIndex, faces.size());
	for (const auto &face : faces) {
		ULOGI("  face: x=%d y=%d w=%d h=%d",
		      face.x,
		      face.y,
		      face.width,
		      face.height);
	}

	if (!annotated.empty() && !faces.empty()) {
		cv::Mat colorOut;
		cv::cvtColor(annotated, colorOut, cv::COLOR_GRAY2BGR);
		for (const auto &face : faces) {
			cv::rectangle(colorOut, face, cv::Scalar(0, 255, 0), 2);
		}
		std::string path = gOutputDir + "/frame_" +
				   std::to_string(frameIndex) + ".jpg";
		try {
			if (!cv::imwrite(path, colorOut))
				ULOGE("failed to write '%s'", path.c_str());
			else
				ULOGI("wrote '%s'", path.c_str());
		} catch (const cv::Exception &e) {
			ULOGE("failed to write '%s': %s",
			      path.c_str(),
			      e.what());
		}
	}
}


/* IPdrawVsink::Listener is the C++-native equivalent of the C API's
 * struct pdraw_vsink_cbs; onFrameReady() is called from the pdraw_vsink
 * internal thread. */
class TestListener : public PdrawVsink::IPdrawVsink::Listener {
public:
	void onFrameReady(PdrawVsink::IPdrawVsink *vsink,
			  struct pdraw_vsink_frame *frame,
			  const struct pdraw_video_frame *frameInfo) override
	{
		(void)vsink;

		if (frame->type == PDRAW_VSINK_VIDEO_MEDIA_TYPE_RAW)
			detectFaces(frameInfo, frame->raw, gFrameCount.load());

		gFrameCount.fetch_add(1);
	}
};


static void printMediaInfo(struct pdraw_media_info *mediaInfo)
{
	ULOGI("media_info: name=%s, path=%s", mediaInfo->name, mediaInfo->path);

	ULOGI("media_info: duration=%.3fs, res=%ux%u, framerate=%u/%u",
	      mediaInfo->duration / 1000000.0,
	      mediaInfo->video.raw.info.resolution.width,
	      mediaInfo->video.raw.info.resolution.height,
	      mediaInfo->video.raw.info.framerate.num,
	      mediaInfo->video.raw.info.framerate.den);

	pdraw_media_info_free(mediaInfo);
}


/* Called from the main thread */
static int run(const char *url, enum vmeta_camera_type cameraType, int count)
{
	struct pdraw_media_info *mediaInfo = nullptr;
	struct pdraw_vsink_params params = {};
	params.url = url;
	params.playback_mode = PDRAW_PLAYBACK_MODE_REALTIME;
	params.camera_type = cameraType;
	params.video_media_type = PDRAW_VSINK_VIDEO_MEDIA_TYPE_RAW;

	gFrameCount = 0;

	TestListener listener;
	PdrawVsink::IPdrawVsink *vsinkRaw = nullptr;
	int res = PdrawVsink::createPdrawVsink(
		&params, &listener, &mediaInfo, &vsinkRaw);
	if (res < 0 || mediaInfo == nullptr) {
		ULOG_ERRNO("createPdrawVsink", -res);
		return res;
	}
	/* No explicit "stop" call: the vsink stops and cleans up
	 * automatically once this unique_ptr goes out of scope. */
	std::unique_ptr<PdrawVsink::IPdrawVsink> vsink(vsinkRaw);
	ULOGI("started");

	printMediaInfo(mediaInfo);

	while (gFrameCount.load() < count)
		std::this_thread::sleep_for(std::chrono::milliseconds(1));

	return 0;
}


enum ArgsId {
	ARGS_ID_CASCADE = 256,
	ARGS_ID_MIN_NEIGHBORS,
	ARGS_ID_MIN_SIZE,
	ARGS_ID_MIN_TRACK_HITS,
};


static const char shortOptions[] = "hc:n:o:";


static const struct option longOptions[] = {
	{"help", no_argument, nullptr, 'h'},
	{"camera-type", required_argument, nullptr, 'c'},
	{"count", required_argument, nullptr, 'n'},
	{"cascade", required_argument, nullptr, ARGS_ID_CASCADE},
	{"output-dir", required_argument, nullptr, 'o'},
	{"min-neighbors", required_argument, nullptr, ARGS_ID_MIN_NEIGHBORS},
	{"min-size", required_argument, nullptr, ARGS_ID_MIN_SIZE},
	{"min-track-hits", required_argument, nullptr, ARGS_ID_MIN_TRACK_HITS},
	{nullptr, 0, nullptr, 0},
};


static void usage(char *progName)
{
	/* clang-format off */
	printf("Usage: %s [OPTIONS] filepath\n"
	       "Runs Haar-cascade face detection on the decoded frames of a "
	       "PDrAW-supported video source.\n"
	       "Options:\n"
	       "  -h | --help                          "
		       "Print this message\n"
	       "  -c | --camera-type <cam>             "
		       "Camera type (e.g. 'front', 'horizontal-stereo-left', "
		       "'down-stereo-right'; default camera if not specified)\n"
	       "  -n | --count <n>                     "
		       "Process at most n frames\n"
	       "       --cascade <path>                "
		       "Path to the Haar cascade XML file (default: "
		       PDRAW_VSINK_DEFAULT_CASCADE_PATH ")\n"
	       "  -o | --output-dir <dir>              "
		       "Directory (must already exist) to write annotated "
		       "JPEG frames with detected faces (disabled if not "
		       "specified)\n"
	       "       --min-neighbors <n>             "
		       "Higher values reduce false-positive detections at the "
		       "cost of missing some real faces (default: 5, OpenCV's "
		       "own default is 3)\n"
	       "       --min-size <px>                 "
		       "Ignore detections smaller than <px> by <px> pixels "
		       "(default: 0, no minimum)\n"
	       "       --min-track-hits <n>            "
		       "Only report a detection once a similar region has "
		       "been (re-)detected on <n> consecutive frames -- "
		       "filters out single-frame flicker false positives, at "
		       "the cost of a short delay before a real face is first "
		       "reported (default: 1, disabled)\n"
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
	char *url = nullptr;
	enum vmeta_camera_type cameraType = VMETA_CAMERA_TYPE_UNKNOWN;
	int count = 20;
	std::string cascadePath = defaultCascadePath();

	while ((c = getopt_long(argc, argv, shortOptions, longOptions, &idx)) !=
	       -1) {
		switch (c) {

		case 'h':
			usage(argv[0]);
			goto out;

		case 'c':
			cameraType = vmeta_camera_type_from_str(optarg);
			break;

		case 'n':
			sscanf(optarg, "%d", &count);
			if (count <= 0) {
				usage(argv[0]);
				status = EXIT_FAILURE;
				goto out;
			}
			break;

		case ARGS_ID_CASCADE:
			cascadePath = optarg;
			break;

		case 'o':
			gOutputDir = optarg;
			break;

		case ARGS_ID_MIN_NEIGHBORS:
			sscanf(optarg, "%d", &gMinNeighbors);
			break;

		case ARGS_ID_MIN_SIZE:
			sscanf(optarg, "%d", &gMinSize);
			break;

		case ARGS_ID_MIN_TRACK_HITS:
			sscanf(optarg, "%d", &gMinTrackHits);
			break;

		default:
			usage(argv[0]);
			status = EXIT_FAILURE;
			goto out;
		}
	}

	if (argc - optind < 1) {
		usage(argv[0]);
		goto out;
	}

	url = argv[optind];
	if (url == nullptr) {
		usage(argv[0]);
		goto out;
	}

	if (!gFaceCascade.load(cascadePath)) {
		ULOGE("failed to load cascade file '%s'", cascadePath.c_str());
		status = EXIT_FAILURE;
		goto out;
	}

	res = run(url, cameraType, count);
	if (res < 0) {
		ULOG_ERRNO("run", -res);
		status = EXIT_FAILURE;
	}

	ULOGI("%s", (status == EXIT_SUCCESS) ? "success!" : "failed!");

out:
	exit(status);
}
