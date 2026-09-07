/**
 * Parrot Drones Audio and Video Vector library
 * Video scaler element
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

#define ULOG_TAG pdraw_vscale
#include <ulog.h>

#include "pdraw_scaler_video.hpp"
#include "pdraw_session.hpp"
#include "pdraw_utils.hpp"

#include <time.h>
#include <unistd.h>

#include <vector>

ULOG_DECLARE_TAG(ULOG_TAG);

namespace Pdraw {


const struct vscale_cbs VideoScaler::mScalerCbs = {
	.frame_output = &VideoScaler::frameOutputCb,
	.flush = &VideoScaler::flushCb,
	.stop = &VideoScaler::stopCb,
};


VideoScaler::VideoScaler(Session *session,
			 Element::Listener *elementListener,
			 Source::Listener *sourceListener,
			 IPdraw::IVideoScaler::Listener *listener,
			 VideoScalerWrapper *wrapper,
			 const struct vscale_config *params) :
		FilterElement(session,
			      elementListener,
			      wrapper,
			      1,
			      nullptr,
			      0,
			      nullptr,
			      0,
			      nullptr,
			      0,
			      1,
			      sourceListener),
		mScaler(wrapper), mScalerListener(listener)
{
	const struct vdef_raw_format *supportedInputFormats;
	int supportedInputFormatsCount;

	Element::setClassName(__func__);

	mCompleteFlushHandler.set([this] { idleCompleteFlush(); });

	/* Supported input formats */
	supportedInputFormatsCount = vscale_get_supported_input_formats(
		VSCALE_SCALER_IMPLEM_AUTO, &supportedInputFormats);
	if (supportedInputFormatsCount < 0)
		PDRAW_LOG_ERRNO("vscale_get_supported_input_formats",
				-supportedInputFormatsCount);
	else
		setRawVideoMediaFormatCaps(supportedInputFormats,
					   supportedInputFormatsCount);

	if (params != nullptr) {
		/* Scaler params deep copy */
		mScalerConfig = make_c_struct<unique_c_ptr<vscale_config>>();
		if (!mScalerConfig) {
			PDRAW_LOG_ERRNO("calloc", ENOMEM);
		} else {
			*mScalerConfig = *params;
			if (params->name != nullptr) {
				mScalerName = params->name;
				mScalerConfig->name = mScalerName.c_str();
			} else {
				mScalerConfig->name = nullptr;
			}
			/* TODO: implem_cfg */
		}
	}

	setState(State::CREATED);
}


VideoScaler::~VideoScaler()
{
	int ret;

	if (mState != State::STOPPED)
		PDRAW_LOGW("scaler is still running");

	/* Make sure listener functions will no longer be called */
	mScalerListener = nullptr;

	/* Remove any leftover idle callbacks */
	mSession->getPompLoop()->idleRemove(this);

	if (mVscale != nullptr) {
		ret = vscale_destroy(mVscale);
		if (ret < 0)
			PDRAW_LOG_ERRNO("vscale_destroy", -ret);
	}

	if (mOutputMedia != nullptr)
		PDRAW_LOGW("output media was not properly removed");
}


int VideoScaler::start()
{
	int ret = 0;
	int err;
	Media *media = nullptr;
	const InputPort *port = nullptr;
	Channel *c = nullptr;
	RawVideoChannel *channel = nullptr;

	if ((mState == State::STARTED) || (mState == State::STARTING)) {
		return 0;
	}
	if (mState != State::CREATED) {
		PDRAW_LOGE("%s: scaler is not created", __func__);
		return -EPROTO;
	}
	setState(State::STARTING);

	/* Get the input media and port */
	Sink::lock();
	unsigned int inputMediaCount = getInputMediaCount();
	if (inputMediaCount != 1) {
		Sink::unlock();
		PDRAW_LOGE("invalid input media count");
		ret = -EPROTO;
		goto error;
	}
	media = getInputMedia(0);
	if (media == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input media");
		ret = -EPROTO;
		goto error;
	}
	mInputMedia = dynamic_cast<RawVideoMedia *>(media);
	if (mInputMedia == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input media");
		ret = -EPROTO;
		goto error;
	}
	port = getInputPort(mInputMedia);
	if (port == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input port");
		ret = -EPROTO;
		goto error;
	}

	/* Initialize the scaler */
	if (mScalerConfig) {
		/* The configuration was provided through the constructor;
		 * simply override the input config */
		mScalerConfig->input.format = mInputMedia->format;
		mScalerConfig->input.info = mInputMedia->info;
	} else {
		mScalerConfig = make_c_struct<unique_c_ptr<vscale_config>>();
		if (!mScalerConfig) {
			Sink::unlock();
			ret = -ENOMEM;
			PDRAW_LOG_ERRNO("calloc", -ret);
			goto error;
		}
		mScalerConfig->implem = VSCALE_SCALER_IMPLEM_AUTO;
		mScalerConfig->input.format = mInputMedia->format;
		mScalerConfig->input.info = mInputMedia->info;
		mScalerConfig->output.info = mScalerConfig->input.info;
		mScalerConfig->output.info.resolution.width = 1280; /* TODO */
		mScalerConfig->output.info.resolution.height = 720; /* TODO */
	}
	ret = vscale_new(mSession->getLoop(),
			 mScalerConfig.get(),
			 &mScalerCbs,
			 this,
			 &mVscale);
	if (ret < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("vscale_new", -ret);
		goto error;
	}

	/* Setup the input port */
	c = port->channel.get();
	channel = dynamic_cast<RawVideoChannel *>(c);
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input channel");
		goto error;
	}
	mInputBufferQueue = mbuf::Queue::wrapExisting(
		vscale_get_input_buffer_queue(mVscale), false);
	channel->setQueue(this, mInputBufferQueue.get());
	mInputBufferPool = vscale_get_input_buffer_pool(mVscale);
	channel->setPool(this, mInputBufferPool);

	Sink::unlock();

	setState(State::STARTED);

	return 0;

error:
	if (mInputMedia != nullptr) {
		/* mInputMedia must be removed synchronously to avoid holding a
		 * reference to a media that can be destroyed at any moment by
		 * the Source element */
		err = removeInputMedia(mInputMedia);
		if (err < 0)
			PDRAW_LOG_ERRNO("removeInputMedia", -err);
		else
			mInputMedia = nullptr;
	}
	err = stop();
	if (err < 0)
		PDRAW_LOG_ERRNO("stop", -err);
	return ret;
}


int VideoScaler::stop()
{
	int ret;

	if ((mState == State::STOPPED) || (mState == State::STOPPING))
		return 0;
	if (mState == State::CREATED) {
		/* Skip flush/drain for unstarted elements to prevent them from
		 * getting stuck in CREATED and blocking session stop */
		setState(State::STOPPED);
		return 0;
	}
	if ((mState != State::STARTING) && (mState != State::STARTED)) {
		PDRAW_LOGE("%s: scaler is not started", __func__);
		return -EPROTO;
	}
	setState(State::STOPPING);
	mVscaleStopPending = true;

	/* Make sure listener functions will no longer be called */
	mScalerListener = nullptr;

	Source::lock();
	if (mOutputMedia != nullptr)
		mOutputMedia->setTearingDown();
	Source::unlock();

	/* Flush everything */
	ret = flush();
	if (ret < 0 && ret != -EALREADY)
		PDRAW_LOG_ERRNO("flush", -ret);
	else
		ret = 0;

	/* When the flush is complete, stopping will be triggered */
	return ret;
}


int VideoScaler::flush(bool discard)
{
	int ret = 0;
	int err;
	unsigned int outputChannelCount;
	Channel *outputChannel;

	switch (getFlushingState()) {
	case FlushingState::UNFLUSHED:
		/* OK */
		break;
	case FlushingState::FLUSHING:
		return -EALREADY;
	case FlushingState::FLUSHED:
		if (mInputBufferQueue != nullptr) {
			ret = mInputBufferQueue->getCount();
			if (ret < 0) {
				PDRAW_LOG_ERRNO("queue::getCount", -ret);
				return ret;
			} else if (ret > 0) {
				setFlushingState(FlushingState::UNFLUSHED);
				break;
			}
		}
		PDRAW_LOGD("scaler is already %s, nothing to do",
			   discard ? "flushed" : "drained");
		ret = mSession->getPompLoop()->idleAdd(&mCompleteFlushHandler,
						       this);
		if (ret < 0)
			PDRAW_LOG_ERRNO("Loop::idleAdd", -ret);
		else
			setFlushingState(FlushingState::FLUSHING, discard);
		return ret;
	default:
		break;
	}

	setFlushingState(FlushingState::FLUSHING, discard);

	Source::lock();
	if (mOutputMedia != nullptr) {
		if (mFlushDiscard) {
			/* Flush the output channels (async) */
			outputChannelCount =
				getOutputChannelCount(mOutputMedia.get());
			for (unsigned int i = 0; i < outputChannelCount; i++) {
				outputChannel =
					getOutputChannel(mOutputMedia.get(), i);
				if (outputChannel == nullptr) {
					PDRAW_LOGW(
						"failed to get output channel "
						"at index %d",
						i);
					continue;
				}
				err = outputChannel->flush();
				if (err < 0 && err != -EALREADY) {
					PDRAW_LOG_ERRNO(
						"channel->flush "
						"(channel index=%u)",
						-err,
						i);
				}
			}
		} else {
			/* Drain event is called once flush is complete */
			mOutputChannelDrainRequired = true;
		}
	}
	Source::unlock();

	/* Flush the scaler (async)
	 * (the input channel queue is flushed by vscale) */
	if (mVscale != nullptr) {
		if (!mVscaleFlushPending) {
			ret = vscale_flush(mVscale, mFlushDiscard);
			if (ret < 0)
				PDRAW_LOG_ERRNO("vscale_flush", -ret);
			else
				mVscaleFlushPending = true;
		}
	} else {
		completeFlush();
	}

	return ret;
}


void VideoScaler::completeFlush()
{
	int ret;
	int err;
	unsigned int outputChannelCount;
	Channel *outputChannel;
	bool pending = false;

	if (mVscaleFlushPending)
		return;

	/* Drain the output channels (async) */
	Source::lock();
	if (!mFlushDiscard && mOutputChannelDrainRequired &&
	    mOutputMedia != nullptr) {
		mOutputChannelDrainRequired = false;
		outputChannelCount = getOutputChannelCount(mOutputMedia.get());
		for (unsigned int i = 0; i < outputChannelCount; i++) {
			outputChannel = getOutputChannel(mOutputMedia.get(), i);
			if (outputChannel == nullptr) {
				PDRAW_LOGW(
					"failed to get output channel "
					"at index %d",
					i);
				continue;
			}
			err = outputChannel->drain();
			if (err < 0 && err != -EALREADY) {
				PDRAW_LOG_ERRNO(
					"channel->drain (channel index=%u)",
					-err,
					i);
			}
		}
	}
	if (mOutputMedia != nullptr) {
		outputChannelCount = getOutputChannelCount(mOutputMedia.get());
		for (unsigned int i = 0; i < outputChannelCount; i++) {
			outputChannel = getOutputChannel(mOutputMedia.get(), i);
			if (outputChannel == nullptr) {
				PDRAW_LOGW(
					"failed to get output channel "
					"at index %d",
					i);
				continue;
			}
			if (outputChannel->isFlushPending() ||
			    outputChannel->isDrainPending()) {
				pending = true;
				break;
			}
		}
	}
	Source::unlock();

	if (pending)
		return;

	setFlushingState(FlushingState::FLUSHED);

	Sink::lock();
	if ((mInputMedia != nullptr) && mInputChannelFlushPending) {
		mInputChannelFlushPending = false;
		Channel *inputChannel = getInputChannel(mInputMedia);
		if (inputChannel == nullptr) {
			PDRAW_LOGE("failed to get input channel");
		} else {
			if (mFlushDiscard)
				ret = inputChannel->flushDone();
			else
				ret = inputChannel->drainDone();
			if (ret < 0) {
				PDRAW_LOG_ERRNO("channel->%s",
						-ret,
						mFlushDiscard ? "flushDone"
							      : "drainDone");
			}
		}
	}
	Sink::unlock();

	tryStop();
}


void VideoScaler::idleCompleteFlush()
{
	completeFlush();
}


int VideoScaler::tryStop()
{
	int ret;
	int outputChannelCount = 0;

	if (mState != State::STOPPING)
		return 0;

	/* Remove the input port */
	Sink::lock();
	if (mInputMedia != nullptr) {
		Channel *c = getInputChannel(mInputMedia);
		auto *channel = dynamic_cast<RawVideoChannel *>(c);
		if (channel == nullptr) {
			PDRAW_LOGE("failed to get channel");
		} else {
			channel->setQueue(this, nullptr);
			channel->setPool(this, nullptr);
		}

		ret = removeInputMedia(mInputMedia);
		if (ret < 0)
			PDRAW_LOG_ERRNO("removeInputMedia", -ret);
		else
			mInputMedia = nullptr;
	}
	Sink::unlock();

	/* Teardown the output channels
	 * Note: loop downwards because calling teardown on a channel may or
	 * may not synchronously remove the channel from the output port */
	Source::lock();
	if (mOutputMedia != nullptr) {
		outputChannelCount = getOutputChannelCount(mOutputMedia.get());

		for (int i = outputChannelCount - 1; i >= 0; i--) {
			auto *channel = getOutputChannel(mOutputMedia.get(), i);
			if (channel == nullptr) {
				PDRAW_LOGW("failed to get channel at index %d",
					   i);
				continue;
			}
			ret = channel->teardown();
			if (ret < 0)
				PDRAW_LOG_ERRNO("channel->teardown", -ret);
		}
	}
	Source::unlock();

	/* Stop the scaler
	 * tryStop() can be re-entered while STOPPING, but vscale_stop() must
	 * only be issued once: vscale_libyuv's worker thread exits after its
	 * first stop request and never processes a second one. */
	if (mVscale != nullptr) {
		if (!mVscaleStopIssued) {
			mVscaleStopIssued = true;
			ret = vscale_stop(mVscale);
			if (ret < 0) {
				PDRAW_LOG_ERRNO("vscale_stop", -ret);
				return ret;
			}
		}
	} else {
		mVscaleStopPending = false;
		completeStop();
	}

	return 0;
}


void VideoScaler::completeStop()
{
	int ret;
	unsigned int outputChannelCount;

	Source::lock();
	if (mOutputMedia == nullptr) {
		Source::unlock();
		goto exit;
	}
	outputChannelCount = getOutputChannelCount(mOutputMedia.get());
	if (outputChannelCount > 0) {
		Source::unlock();
		return;
	}

	/* Remove the output port */
	if (Source::mListener) {
		Source::mListener->onOutputMediaRemoved(
			this, mOutputMedia.get(), getVideoScaler());
	}
	ret = removeOutputPort(mOutputMedia.get());
	if (ret < 0) {
		PDRAW_LOG_ERRNO("removeOutputPort", -ret);
	} else {
		mOutputMedia.reset();
	}

	Source::unlock();

exit:
	if ((!mVscaleStopPending) && (mOutputMedia == nullptr))
		setState(State::STOPPED);
}


int VideoScaler::createOutputMedia(
	const struct vdef_raw_frame *frameInfo,
	[[maybe_unused]] const RawVideoMedia::Frame &frame)
{

	int ret;

	Source::lock();

	try {
		mOutputMedia = std::make_unique<RawVideoMedia>(mSession);
	} catch (const std::bad_alloc &) {
		Source::unlock();
		PDRAW_LOGE("output media allocation failed");
		return -ENOMEM;
	}
	std::string path = mInputMedia->getPath() + ">" + Element::getName() +
			   "$" + mOutputMedia->getName();
	mOutputMedia->setPath(path);

	ret = addOutputPort(mOutputMedia.get());
	if (ret < 0) {
		Source::unlock();
		PDRAW_LOG_ERRNO("addOutputPort", -ret);
		return ret;
	}

	mOutputMedia->format = frameInfo->format;
	vdef_frame_to_format_info(&frameInfo->info, &mOutputMedia->info);
	mOutputMedia->info.framerate = mInputMedia->info.framerate;
	mOutputMedia->sessionMeta = mInputMedia->sessionMeta;
	mOutputMedia->copyPropertiesFrom(mInputMedia);

	Source::unlock();

	if (Source::mListener)
		Source::mListener->onOutputMediaAdded(
			this, mOutputMedia.get(), getVideoScaler());

	return 0;
}


void VideoScaler::onRawVideoChannelQueue(RawVideoChannel *channel,
					 struct mbuf_raw_video_frame *frame)
{

	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_IF(frame == nullptr, EINVAL);

	if (mState != State::STARTED) {
		PDRAW_LOGE("frame input: scaler is not started");
		return;
	}
	if (mVscaleFlushPending || mInputChannelFlushPending) {
		PDRAW_LOGI("frame input: flush pending, discard frame");
		return;
	}
	Sink::lock();
	if (mInputBufferQueue == nullptr ||
	    !channel->hasQueue(mInputBufferQueue.get())) {
		Sink::unlock();
		PDRAW_LOGE("invalid queue");
		return;
	}

	Sink::onRawVideoChannelQueue(channel, frame);
	setFlushingState(FlushingState::UNFLUSHED);
	Sink::unlock();
}


void VideoScaler::onChannelFlush(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	PDRAW_LOGD("flushing input channel");
	mInputChannelFlushPending = true;

	int ret = flush();
	if (ret < 0 && ret != -EALREADY)
		PDRAW_LOG_ERRNO("flush", -ret);
}


void VideoScaler::onChannelDrain(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	PDRAW_LOGD("draining input channel");
	mInputChannelFlushPending = true;

	int ret = drain();
	if (ret < 0 && ret != -EALREADY)
		PDRAW_LOG_ERRNO("drain", -ret);
}


void VideoScaler::onChannelFlushed(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	const Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("%s: output media not found", __func__);
		return;
	}
	PDRAW_LOGD("'%s': channel flushed media name=%s (channel owner=%p)",
		   Element::getName().c_str(),
		   media->getName().c_str(),
		   channel->getOwner());

	completeFlush();
}


void VideoScaler::onChannelDrained(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	const Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("%s: output media not found", __func__);
		return;
	}
	PDRAW_LOGD("'%s': channel drained media name=%s (channel owner=%p)",
		   Element::getName().c_str(),
		   media->getName().c_str(),
		   channel->getOwner());

	completeFlush();
}


void VideoScaler::onChannelTeardown(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	PDRAW_LOGD("tearing down input channel");

	int ret = stop();
	if (ret < 0)
		PDRAW_LOG_ERRNO("stop", -ret);
}


void VideoScaler::onChannelUnlink(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	Source::onChannelUnlink(channel);

	if (mState == State::STOPPING)
		completeStop();
}


void VideoScaler::onChannelSessionMetaUpdate(Channel *channel)
{
	struct vmeta_session tmpSessionMeta;

	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	Sink::lock();
	if (mInputMedia == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("%s: input media not found", __func__);
		return;
	}
	tmpSessionMeta = mInputMedia->sessionMeta;
	Sink::unlock();

	Source::lock();
	if (mOutputMedia == nullptr) {
		Source::unlock();
		PDRAW_LOGE("%s: output media not found", __func__);
		return;
	}
	mOutputMedia->sessionMeta = tmpSessionMeta;
	Source::unlock();

	PDRAW_LOGD("updating session metadata");

	FilterElement::onChannelSessionMetaUpdate(channel);
}


void VideoScaler::frameOutputCb([[maybe_unused]] struct vscale_scaler *scaler,
				int status,
				struct mbuf_raw_video_frame *out_frame,
				void *userdata)
{

	int ret;
	auto *self = static_cast<VideoScaler *>(userdata);
	struct vdef_raw_frame info;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	const RawVideoMedia::Frame *in_meta;
	RawVideoMedia::Frame out_meta;
	unsigned int outputChannelCount;

	if (status != 0) {
		PDRAW_LOGE("scaler error: %d(%s)", -status, strerror(-status));
		return;
	}

	PDRAW_LOG_ERRNO_RETURN_IF(userdata == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_IF(out_frame == nullptr, EINVAL);

	if (self->mState != State::STARTED) {
		PDRAW_LOGE("frame output: scaler is not started");
		return;
	}
	if (self->mFlushDiscard &&
	    (self->mVscaleFlushPending || self->mInputChannelFlushPending)) {
		PDRAW_LOGI("frame output: flush pending, discard frame");
		return;
	}
	ret = mbuf_raw_video_frame_get_frame_info(out_frame, &info);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -ret);
		return;
	}

	self->Sink::lock();
	if (self->mInputMedia == nullptr) {
		self->Sink::unlock();
		PDRAW_LOG_ERRNO("invalid input media", EPROTO);
		return;
	}
	self->Sink::unlock();

	ret = mbuf_raw_video_frame_get_ancillary_data(
		out_frame,
		PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME,
		&ancillaryData);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_ancillary_data",
				-ret);
		return;
	}
	in_meta = static_cast<const RawVideoMedia::Frame *>(
		mbuf_ancillary_data_get_buffer(ancillaryData, nullptr));
	out_meta = *in_meta;
	out_meta.scalerOutputTimestamp = pdraw_getTimestampFromMbufFrame(
		out_frame, VSCALE_ANCILLARY_KEY_OUTPUT_TIME);
	mbuf_ancillary_data_unref(ancillaryData);
	ancillaryData = nullptr;

	ret = mbuf_raw_video_frame_remove_ancillary_data(
		out_frame, PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_remove_ancillary_data",
				-ret);
		return;
	}

	ret = mbuf_raw_video_frame_add_ancillary_buffer(
		out_frame,
		PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME,
		&out_meta,
		sizeof(out_meta));
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer",
				-ret);
		return;
	}

	if (self->mScalerListener != nullptr) {
		self->mScalerListener->videoScalerFrameOutput(
			self->mSession, self->getVideoScaler(), out_frame);
	}

	self->Source::lock();

	if (self->mOutputMedia == nullptr) {
		ret = self->createOutputMedia(&info, out_meta);
		if (ret < 0) {
			self->Source::unlock();
			PDRAW_LOG_ERRNO("createOutputMedia", -ret);
			return;
		}
	}

	/* Push the frame (unless it is silent) */
	if (!(info.info.flags & VDEF_FRAME_FLAG_SILENT)) {
		outputChannelCount =
			self->getOutputChannelCount(self->mOutputMedia.get());
		for (unsigned int i = 0; i < outputChannelCount; i++) {
			Channel *c = self->getOutputChannel(
				self->mOutputMedia.get(), i);
			auto *channel = dynamic_cast<RawVideoChannel *>(c);
			if (channel == nullptr) {
				PDRAW_LOGE("failed to get channel at index %d",
					   i);
				continue;
			}
			ret = channel->queue(out_frame);
			if (ret < 0)
				PDRAW_LOG_ERRNO("channel->queue", -ret);
		}
	} else {
		PDRAW_LOGD("silent frame (ignored)");
	}

	self->Source::unlock();
}


void VideoScaler::flushCb([[maybe_unused]] struct vscale_scaler *scaler,
			  void *userdata)
{

	auto *self = static_cast<VideoScaler *>(userdata);

	PDRAW_LOG_ERRNO_RETURN_IF(userdata == nullptr, EINVAL);

	PDRAW_LOGD("scaler is flushed");
	self->mVscaleFlushPending = false;

	self->completeFlush();
}


void VideoScaler::stopCb([[maybe_unused]] struct vscale_scaler *scaler,
			 void *userdata)
{

	auto *self = static_cast<VideoScaler *>(userdata);

	PDRAW_LOG_ERRNO_RETURN_IF(userdata == nullptr, EINVAL);

	PDRAW_LOGD("scaler is stopped");
	self->mVscaleStopPending = false;
	self->completeStop();
}


VideoScalerWrapper::VideoScalerWrapper(
	Session *session,
	const struct vscale_config *params,
	IPdraw::IVideoScaler::Listener *listener) :
		ElementWrapper(new Pdraw::VideoScaler(session,
						      session,
						      session,
						      listener,
						      this,
						      params)),
		mScaler(static_cast<Pdraw::VideoScaler *>(mElement))
{
}


VideoScalerWrapper::~VideoScalerWrapper()
{
	if (isElementStopped())
		return;
	int ret = mScaler->stop();
	if (ret < 0)
		ULOG_ERRNO("VideoScaler::stop", -ret);
}

} /* namespace Pdraw */
