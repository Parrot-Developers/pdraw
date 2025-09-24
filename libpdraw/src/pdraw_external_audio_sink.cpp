/**
 * Parrot Drones Audio and Video Vector library
 * Application external audio sink
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

#define ULOG_TAG pdraw_external_audio_sink
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_external_audio_sink.hpp"
#include "pdraw_session.hpp"

#include <time.h>

namespace Pdraw {


#define NB_SUPPORTED_FORMATS 8
static struct adef_format supportedFormats[NB_SUPPORTED_FORMATS];
static pthread_once_t supportedFormatsIsInit = PTHREAD_ONCE_INIT;
static void initializeSupportedFormats(void)
{
	supportedFormats[0] = adef_pcm_16b_44100hz_mono;
	supportedFormats[1] = adef_pcm_16b_44100hz_stereo;
	supportedFormats[2] = adef_pcm_16b_48000hz_mono;
	supportedFormats[3] = adef_pcm_16b_48000hz_stereo;
	supportedFormats[4] = adef_aac_lc_16b_44100hz_mono_adts;
	supportedFormats[5] = adef_aac_lc_16b_44100hz_stereo_adts;
	supportedFormats[6] = adef_aac_lc_16b_48000hz_mono_adts;
	supportedFormats[7] = adef_aac_lc_16b_48000hz_stereo_adts;
}


ExternalAudioSink::ExternalAudioSink(Session *session,
				     Element::Listener *elementListener,
				     IPdraw::IAudioSink::Listener *listener,
				     AudioSinkWrapper *wrapper,
				     unsigned int mediaId) :
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
	mAudioSinkListener = listener;
	mAudioSink = wrapper;
	mInputMedia = nullptr;
	mMediaId = 0;
	mTargetMediaId = mediaId;
	mInputFrameQueue = nullptr;
	mInputChannelFlushPending = false;
	mTearingDown = false;
	mPendingRestart = false;

	(void)pthread_once(&supportedFormatsIsInit, initializeSupportedFormats);
	setAudioMediaFormatCaps(supportedFormats, NB_SUPPORTED_FORMATS);

	setState(State::CREATED);
}


ExternalAudioSink::~ExternalAudioSink(void)
{
	int ret;

	if (mState == State::STARTED)
		PDRAW_LOGW("audio sink is still running");

	/* Make sure listener functions will no longer be called */
	mAudioSinkListener = nullptr;

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
		ret = mbuf_audio_frame_queue_flush(mInputFrameQueue);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_flush", -ret);
		ret = mbuf_audio_frame_queue_destroy(mInputFrameQueue);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_destroy", -ret);
		mInputFrameQueue = nullptr;
	}

	Media::cleanupMediaInfo(&mMediaInfo);
}


int ExternalAudioSink::start(void)
{
	if ((mState == State::STARTED) || (mState == State::STARTING)) {
		return 0;
	}
	if (mState != State::CREATED) {
		PDRAW_LOGE("%s: audio sink is not created", __func__);
		return -EPROTO;
	}
	setState(State::STARTING);

	/* Create the queue */
	int res = mbuf_audio_frame_queue_new(&mInputFrameQueue);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_new", -res);
		return res;
	}

	setState(State::STARTED);

	return 0;
}


int ExternalAudioSink::stop(void)
{
	int ret;
	AudioChannel *channel = nullptr;

	if ((mState == State::STOPPED) || (mState == State::STOPPING))
		return 0;
	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: audio sink is not started", __func__);
		return -EPROTO;
	}
	setState(State::STOPPING);

	/* Make sure listener functions will no longer be called.
	 * Note: the IAudioSink::Listener::onAudioSinkFlush function
	 * will not be called, but the ExternalAudioSink::stop function
	 * is only called by the API object destructor, and this is
	 * precisely when we do not want to call listener functions any more;
	 * when destroying the API object, outstanding audio frames should have
	 * been previously released by the caller anyway. */
	mAudioSinkListener = nullptr;

	Sink::lock();

	if (mInputMedia == nullptr) {
		Sink::unlock();
		setState(State::STOPPED);
		return 0;
	}

	channel = dynamic_cast<AudioChannel *>(getInputChannel(mInputMedia));
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


int ExternalAudioSink::setMediaId(unsigned int mediaId)
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


unsigned int ExternalAudioSink::getMediaId(void) const
{
	return mMediaId;
}


int ExternalAudioSink::flush(bool discard)
{
	int ret, err;

	switch (getFlushingState()) {
	case FlushingState::UNFLUSHED:
		/* OK */
		break;
	case FlushingState::FLUSHING:
		return -EALREADY;
	case FlushingState::FLUSHED:
		PDRAW_LOGD("audio sink is already %s, nothing to do",
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
		mSession->getLoop(), callAudioSinkFlush, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
	else
		setFlushingState(FlushingState::FLUSHING, discard);

	return 0;
}


int ExternalAudioSink::flushDone(bool discard)
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
		AudioChannel *channel = dynamic_cast<AudioChannel *>(
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


void ExternalAudioSink::idleFlushDone(void *userdata)
{
	ExternalAudioSink *self = (ExternalAudioSink *)userdata;
	if (self->mFlushDiscard)
		(void)self->flushDone();
	else
		(void)self->drainDone();
}


void ExternalAudioSink::idleRenewMedia(void *userdata)
{

	ExternalAudioSink *self =
		reinterpret_cast<ExternalAudioSink *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (self->mInputMedia != nullptr)
		self->removeInputMedia(self->mInputMedia);

	self->mSession->addMediaToAudioSink(self->mTargetMediaId, self);
}


int ExternalAudioSink::addInputMedia(Media *media)
{
	int ret;
	struct pdraw_media_info mediaInfoCopy = {};

	/* Only accept raw video media */
	AudioMedia *m = dynamic_cast<AudioMedia *>(media);
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

	AudioChannel *channel =
		dynamic_cast<AudioChannel *>(getInputChannel(m));
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("failed to get channel");
		return -EPROTO;
	}
	channel->setQueue(this, mInputFrameQueue);

	mInputMedia = m;
	mMediaId = mTargetMediaId = m->id;

	m->fillMediaInfo(&mMediaInfo);

	/* Another deep copy only used by the listener (must be unlocked). */
	m->fillMediaInfo(&mediaInfoCopy);

	Sink::unlock();

	if (mAudioSinkListener != nullptr) {
		mAudioSinkListener->onAudioSinkMediaAdded(
			mSession, getAudioSink(), &mediaInfoCopy);
	}

	Media::cleanupMediaInfo(&mediaInfoCopy);

	return ret;
}


int ExternalAudioSink::removeInputMedia(Media *media)
{
	int ret;

	Sink::lock();

	if (mInputMedia == media) {
		mInputMedia = nullptr;
		mMediaId = 0;
		if (mAudioSinkListener != nullptr) {
			mAudioSinkListener->onAudioSinkMediaRemoved(
				mSession,
				getAudioSink(),
				&mMediaInfo,
				mPendingRestart);
		}

		Media::cleanupMediaInfo(&mMediaInfo);
		mPendingRestart = false;
	}

	AudioChannel *channel =
		dynamic_cast<AudioChannel *>(getInputChannel(media));
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("failed to get channel");
		return -EPROTO;
	}

	struct mbuf_audio_frame_queue *queue = channel->getQueue(this);
	if (queue != nullptr) {
		ret = mbuf_audio_frame_queue_flush(queue);
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


int ExternalAudioSink::prepareAudioFrame(AudioChannel *channel,
					 struct mbuf_audio_frame *frame)
{
	int ret;
	AudioMedia::Frame *in_meta;
	struct pdraw_audio_frame out_meta = {};
	struct mbuf_ancillary_data *ancillaryData = nullptr;

	if (mInputMedia == nullptr) {
		PDRAW_LOGE("invalid input media");
		return -ENOENT;
	}
	struct mbuf_audio_frame_queue *queue = channel->getQueue(this);
	if (queue == nullptr) {
		PDRAW_LOGE("invalid queue");
		return -ENOENT;
	}
	if (queue != mInputFrameQueue) {
		PDRAW_LOGE("invalid input buffer queue");
		return -EPROTO;
	}

	ret = mbuf_audio_frame_get_frame_info(frame, &out_meta.audio);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_get_frame_info", -ret);
		return ret;
	}

	/* Get the AudioMedia::Frame input metadata */
	ret = mbuf_audio_frame_get_ancillary_data(
		frame,
		PDRAW_ANCILLARY_DATA_KEY_AUDIOMEDIAFRAME,
		&ancillaryData);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_get_ancillary_data", -ret);
		return ret;
	}

	in_meta = (AudioMedia::Frame *)mbuf_ancillary_data_get_buffer(
		ancillaryData, nullptr);

	if (!adef_format_intersect(&out_meta.audio.format,
				   mAudioMediaFormatCaps,
				   mAudioMediaFormatCapsCount)) {
		PDRAW_LOGE("unsupported audio input format");
		return -EPROTO;
	}
	out_meta.ntp_timestamp = in_meta->ntpTimestamp;
	out_meta.ntp_unskewed_timestamp = in_meta->ntpUnskewedTimestamp;
	out_meta.ntp_raw_timestamp = in_meta->ntpRawTimestamp;
	out_meta.ntp_raw_unskewed_timestamp = in_meta->ntpRawUnskewedTimestamp;
	out_meta.play_timestamp = in_meta->playTimestamp;
	out_meta.capture_timestamp = in_meta->captureTimestamp;
	out_meta.local_timestamp = in_meta->localTimestamp;

	/* If the frame is handled by multuple external audio sinks, this key
	 * might already have been filled by another sink, so we don't consider
	 * -EEXIST as an error */
	ret = mbuf_audio_frame_add_ancillary_buffer(
		frame,
		PDRAW_ANCILLARY_DATA_KEY_AUDIOFRAME,
		&out_meta,
		sizeof(out_meta));
	if (ret < 0 && ret != -EEXIST) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_add_ancillary_buffer", -ret);
		goto out;
	}
	ret = 0;

out:
	if (ancillaryData != nullptr)
		mbuf_ancillary_data_unref(ancillaryData);
	return ret;
}


void ExternalAudioSink::onAudioChannelQueue(AudioChannel *channel,
					    struct mbuf_audio_frame *frame)
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
		PDRAW_LOGE("%s: audio sink is not started", __func__);
		return;
	}
	if (mInputChannelFlushPending) {
		PDRAW_LOGI("frame input: flush pending, discard frame");
		return;
	}
	Sink::lock();

	ret = prepareAudioFrame(channel, frame);
	if (ret < 0) {
		Sink::unlock();
		return;
	}

	Sink::onAudioChannelQueue(channel, frame);
	setFlushingState(FlushingState::UNFLUSHED);
	Sink::unlock();
}


void ExternalAudioSink::onChannelReconfigure(Channel *channel)
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


void ExternalAudioSink::onChannelFlush(Channel *channel)
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


void ExternalAudioSink::onChannelDrain(Channel *channel)
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


void ExternalAudioSink::onChannelTeardown(Channel *channel)
{
	AudioChannel *c = dynamic_cast<AudioChannel *>(channel);
	if (c == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	PDRAW_LOGD("tearing down input channel");

	int ret = channelTeardown(c);
	if (ret < 0)
		PDRAW_LOG_ERRNO("channelTeardown", -ret);
}


int ExternalAudioSink::channelTeardown(AudioChannel *channel)
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
		 * mediaRemoved() callback, which can call the AudioSink
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
		PDRAW_LOG_ERRNO("flush", -ret);
	else
		ret = 0;

	return ret;
}


/* Listener call from an idle function */
void ExternalAudioSink::callAudioSinkFlush(void *userdata)
{
	ExternalAudioSink *self =
		reinterpret_cast<ExternalAudioSink *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (self->mAudioSinkListener == nullptr) {
		if (self->mFlushDiscard)
			self->flushDone();
		else
			self->drainDone();
	} else {
		if (self->mFlushDiscard) {
			self->mAudioSinkListener->onAudioSinkFlush(
				self->mSession, self->getAudioSink());
		} else {
			self->mAudioSinkListener->onAudioSinkDrain(
				self->mSession, self->getAudioSink());
		}
	}
}


AudioSinkWrapper::AudioSinkWrapper(Session *session,
				   unsigned int mediaId,
				   IPdraw::IAudioSink::Listener *listener)
{
	mElement = mSink = new Pdraw::ExternalAudioSink(
		session, session, listener, this, mediaId);
}


AudioSinkWrapper::~AudioSinkWrapper(void)
{
	if (isElementStopped())
		return;
	int ret = mSink->stop();
	if (ret < 0)
		ULOG_ERRNO("ExternalAudioSink::stop", -ret);
}


int AudioSinkWrapper::setMediaId(unsigned int mediaId)
{
	if (isElementStopped())
		return -EPROTO;
	return mSink->setMediaId(mediaId);
}


unsigned int AudioSinkWrapper::getMediaId(void)
{
	if (isElementStopped())
		return -EPROTO;
	return mSink->getMediaId();
}


struct mbuf_audio_frame_queue *AudioSinkWrapper::getQueue(void)
{
	if (isElementStopped())
		return nullptr;
	return mSink->getQueue();
}


int AudioSinkWrapper::queueFlushed(void)
{
	if (isElementStopped())
		return -EPROTO;
	return mSink->flushDone();
}


int AudioSinkWrapper::queueDrained(void)
{
	if (isElementStopped())
		return -EPROTO;
	return mSink->drainDone();
}

} /* namespace Pdraw */
