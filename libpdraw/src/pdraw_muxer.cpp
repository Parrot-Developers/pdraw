/**
 * Parrot Drones Awesome Video Viewer Library
 * Generic muxer
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

#define ULOG_TAG pdraw_muxer
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_muxer.hpp"
#include "pdraw_session.hpp"

#include <time.h>

#include <h264/h264.h>
#include <h265/h265.h>

namespace Pdraw {


Muxer::Muxer(Session *session,
	     Element::Listener *elementListener,
	     IPdraw::IMuxer::Listener *listener,
	     IPdraw::IMuxer *muxer,
	     const struct pdraw_muxer_params *params) :
		SinkElement(session,
			    elementListener,
			    UINT_MAX,
			    nullptr,
			    0,
			    nullptr,
			    0),
		mMuxer(muxer), mMuxerListener(listener), mFlushing(false)
{
	Element::setClassName(__func__);

	mParams = *params;

	setState(CREATED);
}


Muxer::~Muxer(void)
{
	int res;

	if ((mState == STARTED) || (mState == STARTING))
		PDRAW_LOGW("%s: still running (%s)",
			   __func__,
			   Element::getElementStateStr(mState));

	unsigned int count = getInputMediaCount();
	if (count > 0) {
		PDRAW_LOGW("%s: not all input media have been removed",
			   __func__);
		res = removeInputMedias();
	}
}


int Muxer::start(void)
{
	int res, err;

	if ((mState == STARTED) || (mState == STARTING)) {
		return 0;
	}
	if (mState != CREATED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}
	setState(STARTING);

	res = internalStart();
	if (res < 0)
		goto error;

	setState(STARTED);

	return 0;

error:
	err = stop();
	return res;
}


int Muxer::stop(void)
{
	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if ((mState != STARTED) && (mState != STARTING)) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}
	setState(STOPPING);

	if (mFlushing)
		return 0;

	return completeStop();
}


int Muxer::completeStop(void)
{
	int res;

	res = internalStop();
	if (res < 0)
		return res;

	res = removeInputMedias();
	if (res < 0)
		return res;

	setState(STOPPED);

	return 0;
}


void Muxer::completeFlush(void)
{
	int err, inputMediaCount, i;
	mFlushing = false;

	Sink::lock();

	inputMediaCount = getInputMediaCount();
	for (i = 0; i < inputMediaCount; i++) {
		Media *media = getInputMedia(i);
		if (media == nullptr)
			continue;
		CodedVideoChannel *codedChannel =
			dynamic_cast<CodedVideoChannel *>(
				getInputChannel(media));
		RawVideoChannel *rawChannel =
			dynamic_cast<RawVideoChannel *>(getInputChannel(media));
		if (codedChannel != nullptr) {
			err = codedChannel->flushDone();
			if (err < 0)
				PDRAW_LOG_ERRNO("codedChannel->flushDone",
						-err);
		} else if (rawChannel != nullptr) {
			err = rawChannel->flushDone();
			if (err < 0)
				PDRAW_LOG_ERRNO("rawChannel->flushDone", -err);
		}
	}

	Sink::unlock();

	if (mState == STOPPING)
		(void)completeStop();
}


void Muxer::idleCompleteFlush(void *userdata)
{
	Muxer *self = (Muxer *)userdata;
	self->completeFlush();
}


int Muxer::addInputMedia(Media *media,
			 const struct pdraw_muxer_video_media_params *params)
{
	int res;
	CodedVideoChannel *codedChannel = nullptr;
	RawVideoChannel *rawChannel = nullptr;
	struct mbuf_coded_video_frame_queue *codedQueue = nullptr;
	struct mbuf_raw_video_frame_queue *rawQueue = nullptr;

	Sink::lock();

	res = Sink::addInputMedia(media);
	if (res == -EEXIST) {
		Sink::unlock();
		return res;
	} else if (res < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("Sink::addInputMedia", -res);
		return res;
	}

	codedChannel =
		dynamic_cast<CodedVideoChannel *>(getInputChannel(media));
	rawChannel = dynamic_cast<RawVideoChannel *>(getInputChannel(media));
	if (codedChannel != nullptr) {
		res = mbuf_coded_video_frame_queue_new(&codedQueue);
		if (res < 0) {
			Sink::unlock();
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_new",
					-res);
			goto error;
		}
		codedChannel->setQueue(this, codedQueue);

		res = addQueueEvtToLoop(codedQueue, mSession->getLoop());
		if (res < 0)
			goto error;

		Sink::unlock();

		return 0;
	} else if (rawChannel != nullptr) {
		res = mbuf_raw_video_frame_queue_new(&rawQueue);
		if (res < 0) {
			Sink::unlock();
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_new", -res);
			goto error;
		}
		rawChannel->setQueue(this, rawQueue);

		res = addQueueEvtToLoop(rawQueue, mSession->getLoop());
		if (res < 0)
			goto error;

		Sink::unlock();

		return 0;
	} else {
		res = -ENODEV;
		PDRAW_LOG_ERRNO("Sink::getInputChannel", -res);
		goto error;
	}

error:
	removeInputMedia(media);
	Sink::unlock();
	return res;
}


int Muxer::removeInputMedia(Media *media)
{
	int res;

	Sink::lock();

	CodedVideoChannel *codedChannel =
		dynamic_cast<CodedVideoChannel *>(getInputChannel(media));
	RawVideoChannel *rawChannel =
		dynamic_cast<RawVideoChannel *>(getInputChannel(media));

	if (!codedChannel && !rawChannel) {
		Sink::unlock();
		res = -ENODEV;
		PDRAW_LOG_ERRNO("Sink::getInputChannel", -res);
		return res;
	}
	/* Keep a reference on the queue to destroy it after removing
	 * the input media (avoids deadlocks when trying to push new
	 * frames out
	 * of upstream elements whereas the queue is already destroyed) */
	struct mbuf_coded_video_frame_queue *codedQueue =
		codedChannel ? codedChannel->getQueue(this) : nullptr;
	struct mbuf_raw_video_frame_queue *rawQueue =
		rawChannel ? rawChannel->getQueue(this) : nullptr;

	res = Sink::removeInputMedia(media);
	if (res < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("Sink::removeInputMedia", -res);
		return res;
	}

	if (codedQueue != nullptr) {
		res = removeQueueEvtFromLoop(codedQueue, mSession->getLoop());
		if (res < 0)
			PDRAW_LOG_ERRNO("removeQueueEvtFromLoop", -res);
		res = mbuf_coded_video_frame_queue_flush(codedQueue);
		if (res < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_flush",
					-res);
		res = mbuf_coded_video_frame_queue_destroy(codedQueue);
		if (res < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_destroy",
					-res);
	}
	if (rawQueue != nullptr) {
		res = removeQueueEvtFromLoop(rawQueue, mSession->getLoop());
		if (res < 0)
			PDRAW_LOG_ERRNO("removeQueueEvtFromLoop", -res);
		res = mbuf_raw_video_frame_queue_flush(rawQueue);
		if (res < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_flush",
					-res);
		res = mbuf_raw_video_frame_queue_destroy(rawQueue);
		if (res < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_destroy",
					-res);
	}

	Sink::unlock();

	return 0;
}


int Muxer::removeInputMedias(void)
{
	int res, inputMediaCount, i;

	Sink::lock();

	inputMediaCount = getInputMediaCount();

	/* Note: loop downwards because calling removeInputMedia removes
	 * input ports and decreases the media count */
	for (i = inputMediaCount - 1; i >= 0; i--) {
		Media *media = getInputMedia(i);
		if (media == nullptr) {
			PDRAW_LOG_ERRNO("getInputMedia", ENOENT);
			continue;
		}
		res = removeInputMedia(media);
	}

	Sink::unlock();

	return 0;
}


int Muxer::addQueueEvtToLoop(struct mbuf_coded_video_frame_queue *queue,
			     struct pomp_loop *loop)
{
	int res;
	struct pomp_evt *evt = nullptr;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(queue == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(loop == nullptr, EINVAL);

	res = mbuf_coded_video_frame_queue_get_event(queue, &evt);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_get_event", -res);
		return res;
	}

	res = pomp_evt_attach_to_loop(evt, loop, &queueEventCb, this);
	if (res < 0) {
		PDRAW_LOG_ERRNO("pomp_evt_attach_to_loop", -res);
		return res;
	}

	return 0;
}


int Muxer::addQueueEvtToLoop(struct mbuf_raw_video_frame_queue *queue,
			     struct pomp_loop *loop)
{
	int res;
	struct pomp_evt *evt = nullptr;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(queue == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(loop == nullptr, EINVAL);

	res = mbuf_raw_video_frame_queue_get_event(queue, &evt);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_get_event", -res);
		return res;
	}

	res = pomp_evt_attach_to_loop(evt, loop, &queueEventCb, this);
	if (res < 0) {
		PDRAW_LOG_ERRNO("pomp_evt_attach_to_loop", -res);
		return res;
	}

	return 0;
}


int Muxer::removeQueueEvtFromLoop(struct mbuf_coded_video_frame_queue *queue,
				  struct pomp_loop *loop)
{
	int res;
	struct pomp_evt *evt = nullptr;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(queue == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(loop == nullptr, EINVAL);

	res = mbuf_coded_video_frame_queue_get_event(queue, &evt);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_get_event", -res);
		return res;
	}

	if (!pomp_evt_is_attached(evt, loop))
		return 0;

	res = pomp_evt_detach_from_loop(evt, loop);
	if (res < 0) {
		PDRAW_LOG_ERRNO("pomp_evt_detach_from_loop", -res);
		return res;
	}

	return 0;
}


int Muxer::removeQueueEvtFromLoop(struct mbuf_raw_video_frame_queue *queue,
				  struct pomp_loop *loop)
{
	int res;
	struct pomp_evt *evt = nullptr;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(queue == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(loop == nullptr, EINVAL);

	res = mbuf_raw_video_frame_queue_get_event(queue, &evt);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_get_event", -res);
		return res;
	}

	res = pomp_evt_detach_from_loop(evt, loop);
	if (res < 0) {
		PDRAW_LOG_ERRNO("pomp_evt_detach_from_loop", -res);
		return res;
	}

	return 0;
}


/* Called on the loop thread */
void Muxer::queueEventCb(struct pomp_evt *evt, void *userdata)
{
	int res;
	Muxer *self = (Muxer *)userdata;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	res = self->process();
}


void Muxer::onChannelFlush(Channel *channel)
{
	int ret;

	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	CodedVideoChannel *cvchannel =
		dynamic_cast<CodedVideoChannel *>(channel);
	RawVideoChannel *rvchannel = dynamic_cast<RawVideoChannel *>(channel);

	if (cvchannel != nullptr) {
		struct mbuf_coded_video_frame_queue *queue =
			cvchannel->getQueue(this);
		if (queue == nullptr) {
			PDRAW_LOGE("invalid queue");
			return;
		}

		ret = mbuf_coded_video_frame_queue_flush(queue);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_flush",
					-ret);
			return;
		}
	} else if (rvchannel != nullptr) {
		struct mbuf_raw_video_frame_queue *queue =
			rvchannel->getQueue(this);
		if (queue == nullptr) {
			PDRAW_LOGE("invalid queue");
			return;
		}

		ret = mbuf_raw_video_frame_queue_flush(queue);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_flush",
					-ret);
			return;
		}
	}

	mFlushing = true;

	ret = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), &idleCompleteFlush, this, this);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -ret);
}


void Muxer::onChannelTeardown(Channel *channel)
{
	Sink::onChannelTeardown(channel);

	if (!mInputPorts.empty())
		return;

	int ret = stop();
	if (ret < 0)
		PDRAW_LOG_ERRNO("stop", -ret);
}


int Muxer::setThumbnail(enum pdraw_muxer_thumbnail_type type,
			const uint8_t *data,
			size_t size)
{
	return -ENOSYS;
}

} /* namespace Pdraw */
