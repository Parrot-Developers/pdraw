/**
 * Parrot Drones Audio and Video Vector library
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


#	define MUXER_STREAM_RTMP_CONNECTION_TIMEOUT_MS 10000
#	define MUXER_STREAM_RTMP_RECONNECTION_TIMER_MS 2000
#	define MUXER_STREAM_RTMP_RECONNECTION_MAX_COUNT 10


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


/* codecheck_ignore[COMPLEX_MACRO] */
#	define MAP_ENUM_CASE(_prefix1, _prefix2, _name)                       \
	case _prefix1##_name:                                                  \
		return _prefix2##_name


static enum pdraw_muxer_disconnection_reason
rtmpClientDisconnectionReasonToPdraw(
	enum rtmp_client_disconnection_reason reason)
{
	/* clang-format off */
	switch (reason) {
	MAP_ENUM_CASE(RTMP_CLIENT_DISCONNECTION_REASON_,
		      PDRAW_MUXER_DISCONNECTION_REASON_,
		      UNKNOWN);
	MAP_ENUM_CASE(RTMP_CLIENT_DISCONNECTION_REASON_,
		      PDRAW_MUXER_DISCONNECTION_REASON_,
		      CLIENT_REQUEST);
	MAP_ENUM_CASE(RTMP_CLIENT_DISCONNECTION_REASON_,
		      PDRAW_MUXER_DISCONNECTION_REASON_,
		      SERVER_REQUEST);
	MAP_ENUM_CASE(RTMP_CLIENT_DISCONNECTION_REASON_,
		      PDRAW_MUXER_DISCONNECTION_REASON_,
		      NETWORK_ERROR);
	MAP_ENUM_CASE(RTMP_CLIENT_DISCONNECTION_REASON_,
		      PDRAW_MUXER_DISCONNECTION_REASON_,
		      REFUSED);
	MAP_ENUM_CASE(RTMP_CLIENT_DISCONNECTION_REASON_,
		      PDRAW_MUXER_DISCONNECTION_REASON_,
		      ALREADY_IN_USE);
	MAP_ENUM_CASE(RTMP_CLIENT_DISCONNECTION_REASON_,
		      PDRAW_MUXER_DISCONNECTION_REASON_,
		      TIMEOUT);
	MAP_ENUM_CASE(RTMP_CLIENT_DISCONNECTION_REASON_,
		      PDRAW_MUXER_DISCONNECTION_REASON_,
		      INTERNAL_ERROR);

	default: return PDRAW_MUXER_DISCONNECTION_REASON_UNKNOWN;
	}
	/* clang-format on */
}


RtmpStreamMuxer::RtmpStreamMuxer(Session *session,
				 Element::Listener *elementListener,
				 IPdraw::IMuxer::Listener *listener,
				 MuxerWrapper *wrapper,
				 const std::string &url,
				 const struct pdraw_muxer_params *params) :
		Muxer(session, elementListener, listener, wrapper, params),
		mUrl(url), mDummyAudioTimer(nullptr), mDummyAudioStarted(false),
		mRtmpClient(nullptr), mRtmpState(DISCONNECTED),
		mRtmpConnectionState(RTMP_CLIENT_CONN_STATE_DISCONNECTED),
		mRtmpDisconnectionReason(
			RTMP_CLIENT_DISCONNECTION_REASON_UNKNOWN),
		mConfigured(false), mSynchronized(false), mVideoMedia(nullptr),
		mDuration(0.), mWidth(0), mHeight(0), mFramerate(0.),
		mAudioSampleRate(mDummyAudioSampleRate),
		mAudioSampleSize(mDummyAudioSampleSize),
		mDummyAudioTimestamp(0), mStats({}),
		mConnectionWatchdog(nullptr), mHasBeenConnected(false),
		mReconnectionCount(0),
		mReconnectionMaxCount(MUXER_STREAM_RTMP_RECONNECTION_MAX_COUNT),
		mReconnectionTimer(nullptr)
{
	(void)pthread_once(&supportedFormatsIsInit, initializeSupportedFormats);

	Element::setClassName(__func__);
	setCodedVideoMediaFormatCaps(supportedFormats, NB_SUPPORTED_FORMATS);

	mStats.type = PDRAW_MUXER_TYPE_RTMP;
}


RtmpStreamMuxer::~RtmpStreamMuxer(void)
{
	int err;

	err = internalStop();
}


/* Must be called on the loop thread */
int RtmpStreamMuxer::addInputMedia(
	Media *media,
	const struct pdraw_muxer_media_params *params)
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

	if (mRtmpConnectionState == RTMP_CLIENT_CONN_STATE_CONNECTED) {
		res = configure();
		if (res < 0)
			return res;
	}

	return 0;
}


int RtmpStreamMuxer::getStats(struct pdraw_muxer_stats *stats)
{
	ULOG_ERRNO_RETURN_ERR_IF(stats == nullptr, EINVAL);

	*stats = mStats;

	return 0;
}


int RtmpStreamMuxer::internalStart(void)
{
	int res;

	/* Create the dummy audio timer */
	mDummyAudioTimer =
		pomp_timer_new(mSession->getLoop(), fakeAudioTimerCb, this);
	if (mDummyAudioTimer == nullptr) {
		res = -ENOMEM;
		PDRAW_LOG_ERRNO("pomp_timer_new", -res);
		return res;
	}

	/* Create the connection watchdog */
	mConnectionWatchdog = pomp_timer_new(
		mSession->getLoop(), &connectionWatchdogCb, this);
	if (mConnectionWatchdog == nullptr) {
		res = -ENOMEM;
		PDRAW_LOG_ERRNO("pomp_timer_new", -res);
		return res;
	}

	/* Create the reconnection timer */
	mReconnectionTimer =
		pomp_timer_new(mSession->getLoop(), &reconnectionTimerCb, this);
	if (mReconnectionTimer == nullptr) {
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

	/* Free the connection watchdog */
	if (mConnectionWatchdog != nullptr) {
		err = pomp_timer_clear(mConnectionWatchdog);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
		err = pomp_timer_destroy(mConnectionWatchdog);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -err);
		mConnectionWatchdog = nullptr;
	}

	/* Free the connection timer */
	if (mReconnectionTimer != nullptr) {
		err = pomp_timer_clear(mReconnectionTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
		err = pomp_timer_destroy(mReconnectionTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -err);
		mReconnectionTimer = nullptr;
	}

	/* Free the RTMP client */
	if (mRtmpClient != nullptr) {
		if (mRtmpConnectionState !=
		    RTMP_CLIENT_CONN_STATE_DISCONNECTED) {
			err = rtmp_client_disconnect(
				mRtmpClient,
				RTMP_CLIENT_DISCONNECTION_REASON_CLIENT_REQUEST);
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
		if ((mRtmpClient != nullptr) &&
		    (mRtmpConnectionState ==
		     RTMP_CLIENT_CONN_STATE_CONNECTED) &&
		    (mConfigured))
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
	struct mbuf_mem *mem = nullptr;
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
	aData = mbuf_ancillary_data_get_buffer(ancillaryData, nullptr);
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

		res = mbuf_coded_video_frame_finalize(frame);
		if (res < 0) {
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_finalize",
					-res);
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
		if (res != -EAGAIN)
			PDRAW_LOG_ERRNO("rtmp_client_send_video_frame", -res);
		else
			mStats.rtmp.dropped_video_frames++;
		goto out;
	}
	mStats.rtmp.pending_video_frames = res;

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
	if (mem)
		(void)mbuf_mem_unref(mem);
	return res;
}


void RtmpStreamMuxer::onChannelFlush(Channel *channel)
{
	int err;

	Muxer::onChannelFlush(channel);

	/* Flush the RTMP client (dataUnrefCb will be called for each pending
	 * frame) */
	if (mRtmpClient != nullptr) {
		err = rtmp_client_flush(mRtmpClient);
		if (err < 0)
			PDRAW_LOG_ERRNO("rtmp_client_flush", -err);
	}
}


void RtmpStreamMuxer::setRtmpState(RtmpStreamMuxer::RtmpState state)
{
	if (state == mRtmpState)
		return;

	mRtmpState = state;
	PDRAW_LOGI("RTMP state change to %s", getRtmpStateStr(mRtmpState));

	mStats.rtmp.is_connected = (mRtmpState == CONNECTED);
	mHasBeenConnected |= (mRtmpState == CONNECTED);

	/* Notify connection state changed */
	onConnectionStateChanged(
		rtmpStateToMuxerConnectionState(mRtmpState),
		rtmpClientDisconnectionReasonToPdraw(mRtmpDisconnectionReason));
}


const char *RtmpStreamMuxer::getRtmpStateStr(RtmpStreamMuxer::RtmpState val)
{
	switch (val) {
	case RtmpStreamMuxer::RtmpState::DISCONNECTED:
		return "DISCONNECTED";
	case RtmpStreamMuxer::RtmpState::CONNECTING:
		return "CONNECTING";
	case RtmpStreamMuxer::RtmpState::CONNECTED:
		return "CONNECTED";
	default:
		return nullptr;
	}
}


enum pdraw_muxer_connection_state
RtmpStreamMuxer::rtmpStateToMuxerConnectionState(RtmpStreamMuxer::RtmpState val)
{
	/* clang-format off */
	switch (val) {
	MAP_ENUM_CASE(, PDRAW_MUXER_CONNECTION_STATE_, DISCONNECTED);
	MAP_ENUM_CASE(, PDRAW_MUXER_CONNECTION_STATE_, CONNECTING);
	MAP_ENUM_CASE(, PDRAW_MUXER_CONNECTION_STATE_, CONNECTED);

	default: return PDRAW_MUXER_CONNECTION_STATE_UNKNOWN;
	}
	/* clang-format on */
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
	if (res < 0) {
		if (res != -EAGAIN)
			PDRAW_LOG_ERRNO("rtmp_client_send_audio_data", -res);
		else
			self->mStats.rtmp.dropped_audio_frames++;
	} else {
		self->mStats.rtmp.pending_audio_frames = res;
	}

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


int RtmpStreamMuxer::reconnect(void)
{
	int ret;

	ULOG_ERRNO_RETURN_ERR_IF(mRtmpClient == nullptr, EPROTO);

	if (mState != STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	if (mRtmpConnectionState != RTMP_CLIENT_CONN_STATE_DISCONNECTED) {
		PDRAW_LOGE("cannot reconnect in %s state",
			   rtmp_client_conn_state_str(mRtmpConnectionState));
		return -EPROTO;
	}

	rtmp_client_destroy(mRtmpClient);

	mRtmpClient = rtmp_client_new(mSession->getLoop(), &mRtmpCbs, this);
	if (mRtmpClient == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("rtmp_client_new", -ret);
		return ret;
	}

	ret = rtmp_client_connect(mRtmpClient, mUrl.c_str());
	if (ret < 0) {
		PDRAW_LOG_ERRNO("rtmp_client_connect", -ret);
		return ret;
	}

	return 0;
}


int RtmpStreamMuxer::scheduleReconnection(void)
{
	int ret;
	if (mReconnectionCount >= mReconnectionMaxCount)
		return -ETIMEDOUT;

	/* Schedule next attempt */
	ret = pomp_timer_set(mReconnectionTimer,
			     MUXER_STREAM_RTMP_RECONNECTION_TIMER_MS);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_timer_set", -ret);

	return ret;
}


void RtmpStreamMuxer::connectionStateCb(
	enum rtmp_client_conn_state state,
	enum rtmp_client_disconnection_reason disconnection_reason,
	void *userdata)
{
	int err;
	RtmpStreamMuxer *self = (RtmpStreamMuxer *)userdata;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (state == self->mRtmpConnectionState)
		return;

	if (state == RTMP_CLIENT_CONN_STATE_DISCONNECTED) {
		PDRAW_LOGI("%s: state=%s, reason=%s",
			   __func__,
			   rtmp_client_conn_state_str(state),
			   rtmp_client_disconnection_reason_str(
				   disconnection_reason));
	} else {
		PDRAW_LOGI("%s: state=%s",
			   __func__,
			   rtmp_client_conn_state_str(state));
	}

	self->mRtmpConnectionState = state;
	self->mRtmpDisconnectionReason = disconnection_reason;

	switch (state) {
	case RTMP_CLIENT_CONN_STATE_DISCONNECTED:
		if ((self->mState == STARTING) || (self->mState == STARTED)) {
			self->mConfigured = false;
			if (self->mDummyAudioTimer != nullptr) {
				err = pomp_timer_clear(self->mDummyAudioTimer);
				if (err < 0)
					PDRAW_LOG_ERRNO("pomp_timer_clear",
							-err);
				self->mDummyAudioStarted = false;
			}
			/* Only reconnect if ever connected */
			if (self->mHasBeenConnected) {
				err = self->scheduleReconnection();
				if (err < 0) {
					PDRAW_LOG_ERRNO("secheduleReconnection",
							-err);
					goto unrecoverable;
				} else {
					self->setRtmpState(CONNECTING);
				}
			} else {
				goto unrecoverable;
			}
		} else {
			self->setRtmpState(DISCONNECTED);
		}
		break;
	case RTMP_CLIENT_CONN_STATE_CONNECTING:
		if ((self->mState == STARTING) || (self->mState == STARTED)) {
			/* Arm the connection watchdog */
			err = pomp_timer_set(
				self->mConnectionWatchdog,
				MUXER_STREAM_RTMP_CONNECTION_TIMEOUT_MS);
			if (err < 0)
				PDRAW_LOG_ERRNO("pomp_timer_set", -err);
			self->setRtmpState(CONNECTING);
		}
		break;
	case RTMP_CLIENT_CONN_STATE_CONNECTED:
		self->mReconnectionCount = 0;
		if ((self->mState == STARTING) || (self->mState == STARTED)) {
			/* Disarm the connection watchdog */
			err = pomp_timer_clear(self->mConnectionWatchdog);
			if (err < 0)
				PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
			self->setRtmpState(CONNECTED);
		}
		if (!self->mConfigured)
			(void)self->configure();
		break;
	default:
		PDRAW_LOGW("unhandled RTMP connection state: (%d: %s)",
			   state,
			   rtmp_client_conn_state_str(state));
		break;
	}

	return;

unrecoverable:
	self->setRtmpState(DISCONNECTED);
	self->onUnrecoverableError(0);
}


void RtmpStreamMuxer::peerBwChangedCb(uint32_t bandwidth, void *userdata)
{
	RtmpStreamMuxer *self = (RtmpStreamMuxer *)userdata;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	PDRAW_LOGI("%s: peer bandwidth changed to %" PRIu32 " bytes per second",
		   __func__,
		   bandwidth);

	self->mStats.rtmp.max_peer_bw = bandwidth;
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


void RtmpStreamMuxer::connectionWatchdogCb(struct pomp_timer *timer,
					   void *userdata)
{
	RtmpStreamMuxer *self = (RtmpStreamMuxer *)userdata;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	PDRAW_LOGW("timeout while connecting");

	if (self->mRtmpClient != nullptr) {
		int err = rtmp_client_disconnect(
			self->mRtmpClient,
			RTMP_CLIENT_DISCONNECTION_REASON_TIMEOUT);
		if (err < 0)
			PDRAW_LOG_ERRNO("rtmp_client_disconnect", -err);
	}
}


void RtmpStreamMuxer::getReconnectionStrategy(
	enum rtmp_client_disconnection_reason disconnectionReason,
	bool *doReconnect,
	int *reconnectionCount)
{
	switch (disconnectionReason) {
	case RTMP_CLIENT_DISCONNECTION_REASON_CLIENT_REQUEST:
	case RTMP_CLIENT_DISCONNECTION_REASON_SERVER_REQUEST:
	case RTMP_CLIENT_DISCONNECTION_REASON_INTERNAL_ERROR:
		/* No retry */
		*doReconnect = false;
		*reconnectionCount = 0;
		return;
	case RTMP_CLIENT_DISCONNECTION_REASON_REFUSED:
		/* One retry */
		*doReconnect = true;
		*reconnectionCount = 1;
		return;
	case RTMP_CLIENT_DISCONNECTION_REASON_NETWORK_ERROR:
	case RTMP_CLIENT_DISCONNECTION_REASON_TIMEOUT:
	case RTMP_CLIENT_DISCONNECTION_REASON_ALREADY_IN_USE:
		/* Retries */
		*doReconnect = true;
		*reconnectionCount = MUXER_STREAM_RTMP_RECONNECTION_MAX_COUNT;
		return;
	default:
		/* No retry */
		*doReconnect = false;
		*reconnectionCount = 0;
	}
}


void RtmpStreamMuxer::reconnectionTimerCb(struct pomp_timer *timer,
					  void *userdata)
{
	int err;
	RtmpStreamMuxer *self = (RtmpStreamMuxer *)userdata;
	bool doReconnect = false;
	int reconnectionCount = 0;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	getReconnectionStrategy(self->mRtmpDisconnectionReason,
				&doReconnect,
				&reconnectionCount);

	if (!doReconnect)
		goto abort;

	self->mReconnectionCount++;
	self->mReconnectionMaxCount = reconnectionCount;

	if (self->mReconnectionCount >= self->mReconnectionMaxCount)
		goto abort;

	PDRAW_LOGI("reconnecting (%d/%d)",
		   self->mReconnectionCount,
		   self->mReconnectionMaxCount);

	if (self->mRtmpConnectionState != RTMP_CLIENT_CONN_STATE_DISCONNECTED) {
		PDRAW_LOGW("reconnection already pending");
		return;
	}

	err = self->reconnect();
	if (err < 0)
		PDRAW_LOG_ERRNO("reconnect", -err);
	else
		return;

	err = self->scheduleReconnection();
	if (err < 0)
		PDRAW_LOG_ERRNO("scheduleReconnection", -err);
	else
		return;

abort:
	self->setRtmpState(DISCONNECTED);
	self->onUnrecoverableError(0);
}

} /* namespace Pdraw */

#endif /* BUILD_LIBRTMP */
