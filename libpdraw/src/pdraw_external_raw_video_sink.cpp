/**
 * Parrot Drones Audio and Video Vector library
 * Application external raw video sink
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

#define ULOG_TAG pdraw_external_raw_video_sink
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_external_raw_video_sink.hpp"
#include "pdraw_session.hpp"

#include <time.h>

namespace Pdraw {


#define NB_SUPPORTED_FORMATS 6
static struct vdef_raw_format supportedFormats[NB_SUPPORTED_FORMATS];
static pthread_once_t supportedFormatsIsInit = PTHREAD_ONCE_INIT;
static void initializeSupportedFormats(void)
{
	supportedFormats[0] = vdef_i420;
	supportedFormats[1] = vdef_nv12;
	supportedFormats[2] = vdef_i420_10_16le;
	supportedFormats[3] = vdef_nv12_10_16le_high;
	supportedFormats[4] = vdef_raw8;
	supportedFormats[5] = vdef_raw16;
}


ExternalRawVideoSink::ExternalRawVideoSink(
	Session *session,
	Element::Listener *elementListener,
	IPdraw::IRawVideoSink::Listener *listener,
	RawVideoSinkWrapper *wrapper,
	unsigned int mediaId,
	const struct pdraw_video_sink_params *params) :
		SinkElement(session,
			    elementListener,
			    wrapper,
			    1,
			    nullptr,
			    0,
			    nullptr,
			    0,
			    nullptr,
			    0)
{
	Element::setClassName(__func__);
	mVideoSinkListener = listener;
	mVideoSink = wrapper;
	mParams = *params;
	mInputMedia = nullptr;
	mMediaId = 0;
	mTargetMediaId = mediaId;
	mInputFrameQueue = nullptr;
	mInputChannelFlushPending = false;
	mTearingDown = false;
	mPendingRestart = false;

	(void)pthread_once(&supportedFormatsIsInit, initializeSupportedFormats);
	setRawVideoMediaFormatCaps(supportedFormats, NB_SUPPORTED_FORMATS);

	setState(State::CREATED);
}


ExternalRawVideoSink::~ExternalRawVideoSink(void)
{
	int ret;

	if (mState == State::STARTED)
		PDRAW_LOGW("video sink is still running");

	/* Make sure listener functions will no longer be called */
	mVideoSinkListener = nullptr;

	/* Remove any leftover idle callbacks */
	ret = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -ret);

	unsigned int count = getInputMediaCount();
	if (count > 0) {
		PDRAW_LOGW("input media has not been removed");
		if (mInputMedia != nullptr) {
			ret = removeInputMedia(mInputMedia);
			if (ret < 0)
				PDRAW_LOG_ERRNO("removeInputMedia", -ret);
		}
	}

	/* Flush and destroy the queue */
	if (mInputFrameQueue != nullptr) {
		ret = mbuf_raw_video_frame_queue_flush(mInputFrameQueue);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_flush",
					-ret);
		ret = mbuf_raw_video_frame_queue_destroy(mInputFrameQueue);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_destroy",
					-ret);
		mInputFrameQueue = nullptr;
	}

	Media::cleanupMediaInfo(&mMediaInfo);
}


int ExternalRawVideoSink::start(void)
{
	if ((mState == State::STARTED) || (mState == State::STARTING)) {
		return 0;
	}
	if (mState != State::CREATED) {
		PDRAW_LOGE("%s: video sink is not created", __func__);
		return -EPROTO;
	}
	setState(State::STARTING);

	/* Create the queue */
	struct mbuf_raw_video_frame_queue_args queueArgs = {};
	queueArgs.max_frames = mParams.queue_max_count;
	int res = mbuf_raw_video_frame_queue_new_with_args(&queueArgs,
							   &mInputFrameQueue);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_new_with_args",
				-res);
		return res;
	}

	setState(State::STARTED);

	return 0;
}


int ExternalRawVideoSink::stop(void)
{
	int ret;
	RawVideoChannel *channel = nullptr;

	if ((mState == State::STOPPED) || (mState == State::STOPPING))
		return 0;
	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: video sink is not started", __func__);
		return -EPROTO;
	}
	setState(State::STOPPING);

	/* Make sure listener functions will no longer be called.
	 * Note: the IRawVideoSink::Listener::onRawVideoSinkFlush function
	 * will not be called, but the ExternalRawVideoSink::stop function
	 * is only called by the API object destructor, and this is
	 * precisely when we do not want to call listener functions any more;
	 * when destroying the API object, outstanding video frames should have
	 * been previously released by the caller anyway. */
	mVideoSinkListener = nullptr;

	Sink::lock();

	if (mInputMedia == nullptr) {
		Sink::unlock();
		setState(State::STOPPED);
		return 0;
	}

	channel = dynamic_cast<RawVideoChannel *>(getInputChannel(mInputMedia));
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("failed to get channel");
		return -EPROTO;
	}

	Sink::unlock();

	ret = channelTeardown(channel);
	if (ret < 0)
		PDRAW_LOG_ERRNO("channelTeardown", -ret);

	return 0;
}


int ExternalRawVideoSink::setMediaId(unsigned int mediaId)
{

	if (mediaId == mTargetMediaId)
		return 0;

	mTargetMediaId = mediaId;
	int ret = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), idleRenewMedia, this, this);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -ret);
	return 0;
}


unsigned int ExternalRawVideoSink::getMediaId(void) const
{
	return mMediaId;
}


int ExternalRawVideoSink::flush(bool discard)
{
	int ret, err;

	switch (getFlushingState()) {
	case FlushingState::UNFLUSHED:
		/* OK */
		break;
	case FlushingState::FLUSHING:
		return -EALREADY;
	case FlushingState::FLUSHED:
		PDRAW_LOGD("video sink is already %s, nothing to do",
			   discard ? "flushed" : "drained");
		ret = pomp_loop_idle_add_with_cookie(
			mSession->getLoop(), &idleFlushDone, this, this);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -ret);
		else
			setFlushingState(FlushingState::FLUSHING, discard);
		return ret;
	default:
		break;
	}

	/* Signal the application for flushing */
	err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), callVideoSinkFlush, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
	else
		setFlushingState(FlushingState::FLUSHING, discard);

	return 0;
}


int ExternalRawVideoSink::flushDone(bool discard)
{
	int ret;

	if (mFlushDiscard != discard) {
		PDRAW_LOGW("calling %s with discard=%d, expecting discard=%d",
			   __func__,
			   discard,
			   mFlushDiscard);
	}

	Sink::lock();

	if (mInputMedia == nullptr)
		goto exit;

	if (mInputChannelFlushPending) {
		RawVideoChannel *channel = dynamic_cast<RawVideoChannel *>(
			getInputChannel(mInputMedia));
		if (channel == nullptr) {
			PDRAW_LOGE("failed to get channel");
		} else {
			mInputChannelFlushPending = false;
			if (mFlushDiscard)
				ret = channel->flushDone();
			else
				ret = channel->drainDone();
			if (ret < 0)
				PDRAW_LOG_ERRNO("channel->%s",
						-ret,
						mFlushDiscard ? "flushDone"
							      : "drainDone");
		}
	}

exit:
	Sink::unlock();

	setFlushingState(FlushingState::FLUSHED);

	if (mState == State::STOPPING)
		setState(State::STOPPED);

	return 0;
}


void ExternalRawVideoSink::idleFlushDone(void *userdata)
{
	ExternalRawVideoSink *self = (ExternalRawVideoSink *)userdata;
	if (self->mFlushDiscard)
		(void)self->flushDone();
	else
		(void)self->drainDone();
}


void ExternalRawVideoSink::idleRenewMedia(void *userdata)
{

	ExternalRawVideoSink *self =
		reinterpret_cast<ExternalRawVideoSink *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (self->mInputMedia != nullptr)
		self->removeInputMedia(self->mInputMedia);

	self->mSession->addMediaToRawVideoSink(self->mTargetMediaId, self);
}


int ExternalRawVideoSink::addInputMedia(Media *media)
{
	int ret;
	struct pdraw_media_info mediaInfoCopy = {};
	struct vmeta_session sessionMetaCopy = {};

	/* Only accept raw video media */
	RawVideoMedia *m = dynamic_cast<RawVideoMedia *>(media);
	if (m == nullptr) {
		PDRAW_LOGE("unsupported input media");
		return -ENOSYS;
	}

	if ((mTargetMediaId != 0) && (mTargetMediaId != m->id))
		return -EPERM;
	if (mInputMedia != nullptr)
		return -EBUSY;
	if (mState != State::STARTED)
		return -EAGAIN;

	Sink::lock();

	ret = Sink::addInputMedia(m);
	if (ret == -EEXIST) {
		Sink::unlock();
		return ret;
	} else if (ret < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("Sink::addInputMedia", -ret);
		return ret;
	}

	RawVideoChannel *channel =
		dynamic_cast<RawVideoChannel *>(getInputChannel(m));
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("failed to get channel");
		return -EPROTO;
	}
	channel->setQueue(this, mInputFrameQueue);

	mInputMedia = m;
	mMediaId = mTargetMediaId = m->id;

	m->fillMediaInfo(&mMediaInfo);
	/* Deep copy: copy the session metadata */
	mMediaInfoSessionMeta = *mMediaInfo.video.session_meta;
	mMediaInfo.video.session_meta = &mMediaInfoSessionMeta;

	/* Another deep copy only used by the listener (must be unlocked). */
	m->fillMediaInfo(&mediaInfoCopy);
	sessionMetaCopy = *mediaInfoCopy.video.session_meta;
	mediaInfoCopy.video.session_meta = &sessionMetaCopy;
	/* TODO: discuss about whether the onVideoRendererMediaAdded listener
	 * might be called with Sink mutex locked to avoid local copy. */

	Sink::unlock();

	if (mVideoSinkListener != nullptr) {
		mVideoSinkListener->onRawVideoSinkMediaAdded(
			mSession, getVideoSink(), &mediaInfoCopy);
	}

	Media::cleanupMediaInfo(&mediaInfoCopy);

	return ret;
}


int ExternalRawVideoSink::removeInputMedia(Media *media)
{
	int ret;

	Sink::lock();

	if (mInputMedia == media) {
		mInputMedia = nullptr;
		mMediaId = 0;
		if (mVideoSinkListener != nullptr) {
			mVideoSinkListener->onRawVideoSinkMediaRemoved(
				mSession,
				getVideoSink(),
				&mMediaInfo,
				mPendingRestart);
		}

		Media::cleanupMediaInfo(&mMediaInfo);
		mPendingRestart = false;
	}

	RawVideoChannel *channel =
		dynamic_cast<RawVideoChannel *>(getInputChannel(media));
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("failed to get channel");
		return -EPROTO;
	}

	struct mbuf_raw_video_frame_queue *queue = channel->getQueue(this);
	if (queue != nullptr) {
		ret = mbuf_raw_video_frame_queue_flush(queue);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_flush",
					-ret);
	}

	ret = Sink::removeInputMedia(media);
	if (ret < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("Sink::removeInputMedia", -ret);
		return ret;
	}

	Sink::unlock();

	return 0;
}


int ExternalRawVideoSink::prepareRawVideoFrame(
	RawVideoChannel *channel,
	struct mbuf_raw_video_frame *frame)
{
	int ret;
	RawVideoMedia::Frame *in_meta;
	struct pdraw_video_frame out_meta = {};
	struct mbuf_ancillary_data *ancillaryData = nullptr;

	if (mInputMedia == nullptr) {
		PDRAW_LOGE("invalid input media");
		return -ENOENT;
	}
	struct mbuf_raw_video_frame_queue *queue = channel->getQueue(this);
	if (queue == nullptr) {
		PDRAW_LOGE("invalid queue");
		return -ENOENT;
	}
	if (queue != mInputFrameQueue) {
		PDRAW_LOGE("invalid input buffer queue");
		return -EPROTO;
	}

	ret = mbuf_raw_video_frame_get_frame_info(frame, &out_meta.raw);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -ret);
		return ret;
	}

	/* Get the RawVideoMedia::Frame input metadata */
	ret = mbuf_raw_video_frame_get_ancillary_data(
		frame, PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME, &ancillaryData);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_ancillary_data",
				-ret);
		return ret;
	}

	in_meta = (RawVideoMedia::Frame *)mbuf_ancillary_data_get_buffer(
		ancillaryData, nullptr);

	if (!vdef_raw_format_intersect(&out_meta.raw.format,
				       mRawVideoMediaFormatCaps,
				       mRawVideoMediaFormatCapsCount)) {
		PDRAW_LOGE("unsupported raw video input format");
		return -EPROTO;
	}
	out_meta.format = VDEF_FRAME_TYPE_RAW;
	out_meta.ntp_timestamp = in_meta->ntpTimestamp;
	out_meta.ntp_unskewed_timestamp = in_meta->ntpUnskewedTimestamp;
	out_meta.ntp_raw_timestamp = in_meta->ntpRawTimestamp;
	out_meta.ntp_raw_unskewed_timestamp = in_meta->ntpRawUnskewedTimestamp;
	out_meta.play_timestamp = in_meta->playTimestamp;
	out_meta.capture_timestamp = in_meta->captureTimestamp;
	out_meta.local_timestamp = in_meta->localTimestamp;

	/* If the frame is handled by multuple external video sinks, this key
	 * might already have been filled by another sink, so we don't consider
	 * -EEXIST as an error */
	ret = mbuf_raw_video_frame_add_ancillary_buffer(
		frame,
		PDRAW_ANCILLARY_DATA_KEY_VIDEOFRAME,
		&out_meta,
		sizeof(out_meta));
	if (ret < 0 && ret != -EEXIST) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer",
				-ret);
		goto out;
	}
	ret = 0;

out:
	if (ancillaryData != nullptr)
		mbuf_ancillary_data_unref(ancillaryData);
	return ret;
}


void ExternalRawVideoSink::onRawVideoChannelQueue(
	RawVideoChannel *channel,
	struct mbuf_raw_video_frame *frame)
{
	int ret;

	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}
	if (frame == nullptr) {
		PDRAW_LOG_ERRNO("frame", EINVAL);
		return;
	}
	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: video sink is not started", __func__);
		return;
	}
	if (mInputChannelFlushPending) {
		PDRAW_LOGI("frame input: flush pending, discard frame");
		return;
	}
	Sink::lock();

	ret = prepareRawVideoFrame(channel, frame);
	if (ret < 0) {
		Sink::unlock();
		return;
	}

	Sink::onRawVideoChannelQueue(channel, frame);
	setFlushingState(FlushingState::UNFLUSHED);
	Sink::unlock();
}


void ExternalRawVideoSink::onChannelFlush(Channel *channel)
{
	int ret;

	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	PDRAW_LOGD("flushing input channel");
	mInputChannelFlushPending = true;

	ret = flush();
	if (ret < 0 && ret != -EALREADY)
		PDRAW_LOG_ERRNO("flush", -ret);
}


void ExternalRawVideoSink::onChannelDrain(Channel *channel)
{
	int ret;

	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	PDRAW_LOGD("draining input channel");
	mInputChannelFlushPending = true;

	ret = drain();
	if (ret < 0 && ret != -EALREADY)
		PDRAW_LOG_ERRNO("drain", -ret);
}


void ExternalRawVideoSink::onChannelTeardown(Channel *channel)
{
	RawVideoChannel *c = dynamic_cast<RawVideoChannel *>(channel);
	if (c == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	PDRAW_LOGD("tearing down input channel");

	int ret = channelTeardown(c);
	if (ret < 0)
		PDRAW_LOG_ERRNO("channelTeardown", -ret);
}


void ExternalRawVideoSink::onChannelSessionMetaUpdate(Channel *channel)
{
	struct vmeta_session tmpSessionMeta;
	Sink::onChannelSessionMetaUpdate(channel);

	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::lock();
	if (mInputMedia == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("%s: input media not found", __func__);
		return;
	}
	tmpSessionMeta = mInputMedia->sessionMeta;
	Sink::unlock();

	if (mVideoSinkListener != nullptr) {
		mVideoSinkListener->onRawVideoSinkSessionMetaUpdate(
			mSession, getVideoSink(), &tmpSessionMeta);
	}
}


void ExternalRawVideoSink::onChannelReconfigure(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	mPendingRestart = true;

	Sink::lock();
	Sink::onChannelReconfigure(channel);
	Sink::unlock();
}


void ExternalRawVideoSink::onChannelResolutionChange(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	mPendingRestart = true;

	Sink::lock();
	Sink::onChannelResolutionChange(channel);
	Sink::unlock();
}


void ExternalRawVideoSink::onChannelFramerateChange(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	mPendingRestart = true;

	Sink::lock();
	Sink::onChannelFramerateChange(channel);
	Sink::unlock();
}


int ExternalRawVideoSink::channelTeardown(RawVideoChannel *channel)
{
	int ret;

	if (channel == nullptr)
		return -EINVAL;

	Sink::lock();

	if (mInputMedia == nullptr) {
		/* The channel is already torn down, nothing more to do */
		Sink::unlock();
		return 0;
	}

	if (mTearingDown) {
		/* The teardown may already be in progress but mInputMedia
		 * is not yet set to nullptr.
		 * Eg. removeInputMedia() utimately calls the app's
		 * mediaRemoved() callback, which can call the VideoSink
		 * stop() function, which calls channelTeardown() again. */
		Sink::unlock();
		return 0;
	}
	mTearingDown = true;

	/* Remove the input port */
	channel->setQueue(this, nullptr);

	ret = removeInputMedia(mInputMedia);
	if (ret < 0)
		PDRAW_LOG_ERRNO("removeInputMedia", -ret);
	else
		mInputMedia = nullptr;

	mTearingDown = false;
	Sink::unlock();

	ret = flush();
	if (ret < 0 && ret != -EALREADY)
		PDRAW_LOG_ERRNO("drain", -ret);
	else
		ret = 0;

	return ret;
}


/* Listener call from an idle function */
void ExternalRawVideoSink::callVideoSinkFlush(void *userdata)
{
	ExternalRawVideoSink *self =
		reinterpret_cast<ExternalRawVideoSink *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (self->mVideoSinkListener == nullptr) {
		if (self->mFlushDiscard)
			self->flushDone();
		else
			self->drainDone();
	} else {
		if (self->mFlushDiscard) {
			self->mVideoSinkListener->onRawVideoSinkFlush(
				self->mSession, self->getVideoSink());
		} else {
			self->mVideoSinkListener->onRawVideoSinkDrain(
				self->mSession, self->getVideoSink());
		}
	}
}


RawVideoSinkWrapper::RawVideoSinkWrapper(
	Session *session,
	unsigned int mediaId,
	const struct pdraw_video_sink_params *params,
	IPdraw::IRawVideoSink::Listener *listener)
{
	mElement = mSink = new Pdraw::ExternalRawVideoSink(
		session, session, listener, this, mediaId, params);
}


RawVideoSinkWrapper::~RawVideoSinkWrapper(void)
{
	if (isElementStopped())
		return;
	int ret = mSink->stop();
	if (ret < 0)
		ULOG_ERRNO("sink->stop", -ret);
}


int RawVideoSinkWrapper::setMediaId(unsigned int mediaId)
{
	if (isElementStopped())
		return -EPROTO;
	return mSink->setMediaId(mediaId);
}


unsigned int RawVideoSinkWrapper::getMediaId(void)
{
	if (isElementStopped())
		return -EPROTO;
	return mSink->getMediaId();
}


struct mbuf_raw_video_frame_queue *RawVideoSinkWrapper::getQueue(void)
{
	if (isElementStopped())
		return nullptr;
	return mSink->getQueue();
}


int RawVideoSinkWrapper::queueFlushed(void)
{
	if (isElementStopped())
		return -EPROTO;
	return mSink->flushDone();
}


int RawVideoSinkWrapper::queueDrained(void)
{
	if (isElementStopped())
		return -EPROTO;
	return mSink->drainDone();
}

} /* namespace Pdraw */
