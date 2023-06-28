/**
 * Parrot Drones Awesome Video Viewer Library
 * RTMP stream muxer
 *
 * Copyright (c) 2018 Parrot Drones SAS
 * Copyright (c) 2016 Aurelien Barre
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

#define ULOG_TAG pdraw_rtmpmuxer
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_muxer_stream_rtmp.hpp"
#include "pdraw_session.hpp"

#ifdef BUILD_LIBRTMP

#	include <time.h>

#	include <libmp4.h>
#	include <media-buffers/mbuf_mem_generic.h>

namespace Pdraw {


#	define NB_SUPPORTED_FORMATS 1
static struct vdef_coded_format supportedFormats[NB_SUPPORTED_FORMATS];
static pthread_once_t supportedFormatsIsInit = PTHREAD_ONCE_INIT;
static void initializeSupportedFormats(void)
{
	supportedFormats[0] = vdef_h264_avcc;
}


const struct rtmp_callbacks RtmpStreamMuxer::mRtmpCbs = {
	.socket_cb = &RtmpStreamMuxer::onSocketCreated,
	.connection_state = &RtmpStreamMuxer::connectionStateCb,
	.peer_bw_changed = &RtmpStreamMuxer::peerBwChangedCb,
	.data_unref = &RtmpStreamMuxer::dataUnrefCb,
};

const uint8_t RtmpStreamMuxer::mDummyAudioSpecificConfig[5] = {
	0x12,
	0x10,
	0x56,
	0xe5,
	0x00,
};

const uint8_t RtmpStreamMuxer::mDummyAudioSample[6] = {
	0x21,
	0x10,
	0x04,
	0x60,
	0x8c,
	0x1c,
};

const int RtmpStreamMuxer::mDummyAudioSampleRate = 44100;

const int RtmpStreamMuxer::mDummyAudioSampleSize = 16;


RtmpStreamMuxer::RtmpStreamMuxer(Session *session,
				 Element::Listener *elementListener,
				 IPdraw::IMuxer::Listener *listener,
				 IPdraw::IMuxer *muxer,
				 const std::string &url,
				 const struct pdraw_muxer_params *params) :
		Muxer(session, elementListener, listener, muxer, params),
		mUrl(url), mDummyAudioTimer(nullptr), mDummyAudioStarted(false),
		mRtmpClient(nullptr), mRtmpConnectionState(RTMP_DISCONNECTED),
		mConfigured(false), mSynchronized(false), mVideoMedia(nullptr),
		mDuration(0.), mWidth(0), mHeight(0), mFramerate(0.),
		mAudioSampleRate(mDummyAudioSampleRate),
		mAudioSampleSize(mDummyAudioSampleSize), mDummyAudioTimestamp(0)
{
	(void)pthread_once(&supportedFormatsIsInit, initializeSupportedFormats);

	Element::setClassName(__func__);
	setCodedVideoMediaFormatCaps(supportedFormats, NB_SUPPORTED_FORMATS);
}


RtmpStreamMuxer::~RtmpStreamMuxer(void)
{
	int err;

	err = internalStop();
}


/* Must be called on the loop thread */
int RtmpStreamMuxer::addInputMedia(
	Media *media,
	const struct pdraw_muxer_video_media_params *params)
{
	int res;
	const uint8_t *sps = nullptr, *pps = nullptr;
	size_t spsSize = 0, ppsSize = 0;
	unsigned int len = 0;

	/* Only accept coded video media */
	CodedVideoMedia *m = dynamic_cast<CodedVideoMedia *>(media);
	if (m == nullptr) {
		PDRAW_LOGE("%s: unsupported input media", __func__);
		return -ENOSYS;
	}

	/* Only accept the first video media */
	if (mVideoMedia != nullptr) {
		PDRAW_LOGE("%s: only 1 video media supported, ignoring",
			   __func__);
		return -EALREADY;
	}

	res = Muxer::addInputMedia(m, params);
	if (res < 0)
		return res;

	mVideoMedia = m;
	mDuration = 0.; /* TODO */
	mWidth = m->info.resolution.width;
	mHeight = m->info.resolution.height;
	mFramerate =
		((m->info.framerate.num != 0) && (m->info.framerate.den != 0))
			? (double)m->info.framerate.num /
				  (double)m->info.framerate.den
			: 30.; /* TODO */

	res = m->getPs(nullptr, nullptr, &sps, &spsSize, &pps, &ppsSize);
	if (res < 0) {
		PDRAW_LOG_ERRNO("CodedVideoMedia::getPs", -res);
		return res;
	}
	len = spsSize + ppsSize +
	      11; /* TODO: the size shoud be retreived from libmp4 */
	mVideoAvcc.resize(len);
	res = mp4_generate_avc_decoder_config(
		sps, spsSize, pps, ppsSize, mVideoAvcc.data(), &len);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_generate_avc_decoder_config", -res);
		return res;
	}

	if (mRtmpConnectionState == RTMP_CONNECTED) {
		res = configure();
		if (res < 0)
			return res;
	}

	return 0;
}


int RtmpStreamMuxer::internalStart(void)
{
	int res;

	mDummyAudioTimer =
		pomp_timer_new(mSession->getLoop(), fakeAudioTimerCb, this);
	if (mDummyAudioTimer == nullptr) {
		res = -ENOMEM;
		PDRAW_LOG_ERRNO("pomp_timer_new", -res);
		return res;
	}

	/* Create the RTMP client and connect */
	mRtmpClient = rtmp_client_new(mSession->getLoop(), &mRtmpCbs, this);
	if (mRtmpClient == nullptr) {
		res = -ENOMEM;
		PDRAW_LOG_ERRNO("rtmp_client_new", -res);
		return res;
	}
	res = rtmp_client_connect(mRtmpClient, mUrl.c_str());
	if (res < 0) {
		PDRAW_LOG_ERRNO("rtmp_client_connect", -res);
		return res;
	}

	return 0;
}


int RtmpStreamMuxer::internalStop(void)
{
	int err;

	/* Free the dummy audio timer */
	if (mDummyAudioTimer != nullptr) {
		err = pomp_timer_clear(mDummyAudioTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
		err = pomp_timer_destroy(mDummyAudioTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -err);
		mDummyAudioTimer = nullptr;
	}

	/* Free the RTMP client */
	if (mRtmpClient != nullptr) {
		if (mRtmpConnectionState != RTMP_DISCONNECTED) {
			err = rtmp_client_disconnect(mRtmpClient);
			if (err < 0)
				PDRAW_LOG_ERRNO("rtmp_client_disconnect", -err);
		}

		rtmp_client_destroy(mRtmpClient);
		mRtmpClient = nullptr;
	}

	mConfigured = false;

	return 0;
}


int RtmpStreamMuxer::configure(void)
{
	int res;

	if (mConfigured)
		return -EPROTO;

	if (mRtmpClient == nullptr)
		return -EPROTO;

	res = rtmp_client_send_metadata(mRtmpClient,
					mDuration,
					mWidth,
					mHeight,
					mFramerate,
					mAudioSampleRate,
					mAudioSampleSize);
	if (res < 0) {
		PDRAW_LOG_ERRNO("rtmp_client_send_metadata", -res);
		return res;
	}

	res = rtmp_client_send_video_avcc(
		mRtmpClient, mVideoAvcc.data(), mVideoAvcc.size(), nullptr);
	if (res < 0) {
		PDRAW_LOG_ERRNO("rtmp_client_send_video_avcc", -res);
		return res;
	}

	res = rtmp_client_send_audio_specific_config(
		mRtmpClient,
		mDummyAudioSpecificConfig,
		sizeof(mDummyAudioSpecificConfig),
		nullptr);
	if (res < 0) {
		PDRAW_LOG_ERRNO("rtmp_client_send_audio_specific_config", -res);
		return res;
	}

	mConfigured = true;
	PDRAW_LOGI("RTMP client configured");
	return 0;
}


int RtmpStreamMuxer::process(void)
{
	int res, inputMediaCount, i;

	if (mState != STARTED)
		return 0;

	Sink::lock();

	inputMediaCount = getInputMediaCount();

	for (i = 0; i < inputMediaCount; i++) {
		CodedVideoMedia *media =
			dynamic_cast<CodedVideoMedia *>(getInputMedia(i));
		if (media == nullptr) {
			res = -ENOENT;
			PDRAW_LOG_ERRNO("getInputMedia", -res);
			continue;
		}

		/* Process each media */
		processMedia(media);
	}

	Sink::unlock();

	return 0;
}


int RtmpStreamMuxer::processMedia(CodedVideoMedia *media)
{
	int res, err;
	struct mbuf_coded_video_frame *frame;

	CodedVideoChannel *channel =
		dynamic_cast<CodedVideoChannel *>(getInputChannel(media));
	if (channel == nullptr) {
		res = -ENODEV;
		PDRAW_LOG_ERRNO("Sink::getInputChannel", -res);
		return res;
	}
	struct mbuf_coded_video_frame_queue *queue = channel->getQueue(this);
	if (queue == nullptr) {
		res = -ENODEV;
		PDRAW_LOG_ERRNO("Channel::getQueue", -res);
		return res;
	}

	/* TODO: This loops drops frames if the processFrame function fails.
	 * This is a problem for most coded streams, so the current behavior
	 * will result in a buggy output stream.
	 * We should instead check what the error was, and decide to either:
	 * - Retry later, which can be achieved by using queue_peek() instead of
	 *   queue_pop(), or
	 * - Discard the frame, flush the queue, and ask the upstream elements
	 *   for a complete resync
	 * The second choice is important to have, because if an error persists,
	 * then this whole element will be stuck on the buggy frame. */
	do {
		res = mbuf_coded_video_frame_queue_pop(queue, &frame);
		if (res < 0) {
			if (res != -EAGAIN)
				PDRAW_LOG_ERRNO(
					"mbuf_coded_video_frame_queue_pop",
					-res);
			continue;
		}

		/* Process each buffer */
		if ((mRtmpConnectionState == RTMP_CONNECTED) && (mConfigured))
			res = processFrame(media, frame);

		err = mbuf_coded_video_frame_unref(frame);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref", -err);
	} while (res == 0);

	return 0;
}


int RtmpStreamMuxer::processFrame(CodedVideoMedia *media,
				  struct mbuf_coded_video_frame *in_frame)
{
	int res;
	struct mbuf_coded_video_frame *frame = nullptr;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	CodedVideoMedia::Frame *meta;
	const void *data = nullptr, *aData;
	size_t len;
	uint32_t ts;

	res = mbuf_coded_video_frame_get_ancillary_data(
		in_frame,
		PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME,
		&ancillaryData);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_ancillary_data",
				-res);
		goto out;
	}
	aData = mbuf_ancillary_data_get_buffer(ancillaryData, NULL);
	meta = (CodedVideoMedia::Frame *)aData;

	if ((!mSynchronized) && (!meta->isSync))
		goto out;
	mSynchronized = true;

	res = mbuf_coded_video_frame_get_packed_buffer(in_frame, &data, &len);
	if (res == 0) {
		/* Frame is already packed */
		frame = in_frame;
		mbuf_coded_video_frame_ref(frame);
	} else if (res == -EPROTO) {
		/* Frame is not packed, copy & pack it */
		struct mbuf_mem *mem;
		res = mbuf_mem_generic_new(len, &mem);
		if (res < 0) {
			PDRAW_LOG_ERRNO("mbuf_mem_generic_new", -res);
			goto out;
		}
		res = mbuf_coded_video_frame_copy(in_frame, mem, &frame);
		if (res < 0) {
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_copy", -res);
			goto out;
		}
		res = mbuf_coded_video_frame_get_packed_buffer(
			frame, &data, &len);
		if (res < 0) {
			PDRAW_LOG_ERRNO(
				"mbuf_coded_video_frame_get_packed_buffer",
				-res);
			goto out;
		}
	} else {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_packed_buffer",
				-res);
		goto out;
	}

	ts = (meta->ntpRawTimestamp + 500) / 1000;

	res = rtmp_client_send_video_frame(
		mRtmpClient, (const uint8_t *)data, len, ts, frame);
	if (res < 0) {
		PDRAW_LOG_ERRNO("rtmp_client_send_video_frame", -res);
		goto out;
	}
	/* At this point, the frame will be released by the dataReleaseCb
	 * function, so we can forget about it */
	frame = nullptr;


	if (!mDummyAudioStarted) {
		mDummyAudioTimestamp = ts;
		res = pomp_timer_set(mDummyAudioTimer, 1);
		if (res < 0)
			PDRAW_LOG_ERRNO("pomp_timer_set", -res);
		else
			mDummyAudioStarted = true;
	}

out:
	if (ancillaryData)
		mbuf_ancillary_data_unref(ancillaryData);
	if (frame) {
		if (data)
			mbuf_coded_video_frame_release_packed_buffer(frame,
								     data);
		mbuf_coded_video_frame_unref(frame);
	}
	return res;
}


void RtmpStreamMuxer::fakeAudioTimerCb(struct pomp_timer *timer, void *userdata)
{
	int res;
	RtmpStreamMuxer *self = (RtmpStreamMuxer *)userdata;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	res = rtmp_client_send_audio_data(self->mRtmpClient,
					  mDummyAudioSample,
					  sizeof(mDummyAudioSample),
					  self->mDummyAudioTimestamp,
					  nullptr);
	if (res < 0)
		PDRAW_LOG_ERRNO("rtmp_client_send_audio_data", -res);

	self->mDummyAudioTimestamp += 23;

	res = pomp_timer_set(timer, 23);
	if (res < 0)
		PDRAW_LOG_ERRNO("pomp_timer_set", -res);
}


void RtmpStreamMuxer::onSocketCreated(int fd, void *userdata)
{
	RtmpStreamMuxer *self = reinterpret_cast<RtmpStreamMuxer *>(userdata);

	self->mSession->socketCreated(fd);
}


void RtmpStreamMuxer::connectionStateCb(enum rtmp_connection_state state,
					void *userdata)
{
	RtmpStreamMuxer *self = (RtmpStreamMuxer *)userdata;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	PDRAW_LOGI("%s: state=%s",
		   __func__,
		   rtmp_connection_state_to_string(state));
	self->mRtmpConnectionState = state;

	if ((self->mRtmpConnectionState == RTMP_CONNECTED) &&
	    (!self->mConfigured)) {
		self->configure();
	}
}


void RtmpStreamMuxer::peerBwChangedCb(uint32_t bandwidth, void *userdata)
{
	RtmpStreamMuxer *self = (RtmpStreamMuxer *)userdata;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	PDRAW_LOGI("%s: peer bandwidth changed to %" PRIu32 " bytes per second",
		   __func__,
		   bandwidth);
}


void RtmpStreamMuxer::dataUnrefCb(uint8_t *data,
				  void *buffer_userdata,
				  void *userdata)
{
	struct mbuf_coded_video_frame *frame =
		(struct mbuf_coded_video_frame *)buffer_userdata;

	if (frame != nullptr) {
		mbuf_coded_video_frame_release_packed_buffer(
			frame, (const void *)data);
		mbuf_coded_video_frame_unref(frame);
	}
}

} /* namespace Pdraw */

#endif /* BUILD_LIBRTMP */
