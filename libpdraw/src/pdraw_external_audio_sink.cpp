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
				     AudioSinkWrapper *wrapper) :
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
	mInputFrameQueue = nullptr;
	mIsFlushed = true;
	mInputChannelFlushPending = false;
	mTearingDown = false;

	(void)pthread_once(&supportedFormatsIsInit, initializeSupportedFormats);
	setAudioMediaFormatCaps(supportedFormats, NB_SUPPORTED_FORMATS);

	setState(CREATED);
}


ExternalAudioSink::~ExternalAudioSink(void)
{
	int ret;

	if (mState == STARTED)
		PDRAW_LOGW("audio sink is still running");

	/* Make sure listener functions will no longer be called */
	mAudioSinkListener = nullptr;

	/* Remove any leftover idle callbacks */
	ret = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -ret);

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
}


int ExternalAudioSink::start(void)
{
	if ((mState == STARTED) || (mState == STARTING)) {
		return 0;
	}
	if (mState != CREATED) {
		PDRAW_LOGE("%s: audio sink is not created", __func__);
		return -EPROTO;
	}
	setState(STARTING);

	/* Get the input media and port */
	Sink::lock();
	unsigned int inputMediaCount = getInputMediaCount();
	if (inputMediaCount != 1) {
		Sink::unlock();
		PDRAW_LOGE("invalid input media count");
		return -EPROTO;
	}
	mInputMedia = dynamic_cast<AudioMedia *>(getInputMedia(0));
	if (mInputMedia == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input media");
		return -EPROTO;
	}
	InputPort *port;
	port = getInputPort(mInputMedia);
	if (port == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input port");
		return -EPROTO;
	}

	/* Create the queue */
	int res = mbuf_audio_frame_queue_new(&mInputFrameQueue);
	if (res < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_new", -res);
		return res;
	}

	/* Setup the input port */
	Channel *c = port->channel;
	AudioChannel *channel = dynamic_cast<AudioChannel *>(c);
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input channel");
		return -EPROTO;
	}
	channel->setQueue(this, mInputFrameQueue);

	Sink::unlock();

	setState(STARTED);

	return 0;
}


int ExternalAudioSink::stop(void)
{
	int ret;
	AudioChannel *channel = nullptr;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if (mState != STARTED) {
		PDRAW_LOGE("%s: audio sink is not started", __func__);
		return -EPROTO;
	}
	setState(STOPPING);

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
		setState(STOPPED);
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


int ExternalAudioSink::flush(void)
{
	int err;

	if (mIsFlushed) {
		PDRAW_LOGD("audio sink is already flushed, nothing to do");
		err = pomp_loop_idle_add_with_cookie(
			mSession->getLoop(), &idleFlushDone, this, this);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
		return 0;
	}

	/* Signal the application for flushing */
	err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), callAudioSinkFlush, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);

	return 0;
}


int ExternalAudioSink::flushDone(void)
{
	int ret;

	Sink::lock();

	if (mInputMedia == nullptr)
		goto exit;

	if (mInputChannelFlushPending) {
		AudioChannel *channel = dynamic_cast<AudioChannel *>(
			getInputChannel(mInputMedia));
		if (channel == nullptr) {
			PDRAW_LOGE("failed to get channel");
		} else {
			mIsFlushed = true;
			mInputChannelFlushPending = false;
			ret = channel->flushDone();
			if (ret < 0)
				PDRAW_LOG_ERRNO("channel->flushDone", -ret);
		}
	}

exit:
	Sink::unlock();

	if (mState == STOPPING)
		setState(STOPPED);

	return 0;
}


void ExternalAudioSink::idleFlushDone(void *userdata)
{
	ExternalAudioSink *self = (ExternalAudioSink *)userdata;
	(void)self->flushDone();
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
	if (mState != STARTED) {
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
	mIsFlushed = false;
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
	if (ret < 0)
		PDRAW_LOG_ERRNO("flush", -ret);
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
	if (ret < 0)
		PDRAW_LOG_ERRNO("flush", -ret);

	return ret;
}


/* Listener call from an idle function */
void ExternalAudioSink::callAudioSinkFlush(void *userdata)
{
	ExternalAudioSink *self =
		reinterpret_cast<ExternalAudioSink *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (self->mAudioSinkListener == nullptr) {
		self->flushDone();
	} else {
		self->mAudioSinkListener->onAudioSinkFlush(
			self->mSession, self->getAudioSink());
	}
}


AudioSinkWrapper::AudioSinkWrapper(Session *session,
				   IPdraw::IAudioSink::Listener *listener)
{
	mElement = mSink =
		new Pdraw::ExternalAudioSink(session, session, listener, this);
}


AudioSinkWrapper::~AudioSinkWrapper(void)
{
	if (mSink == nullptr)
		return;
	int ret = mSink->stop();
	if (ret < 0)
		ULOG_ERRNO("ExternalAudioSink::stop", -ret);
}


struct mbuf_audio_frame_queue *AudioSinkWrapper::getQueue(void)
{
	if (mSink == nullptr)
		return nullptr;
	return mSink->getQueue();
}


int AudioSinkWrapper::queueFlushed(void)
{
	if (mSink == nullptr)
		return -EPROTO;
	return mSink->flushDone();
}

} /* namespace Pdraw */
