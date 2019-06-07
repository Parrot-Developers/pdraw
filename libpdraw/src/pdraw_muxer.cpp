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


Muxer::Muxer(Session *session, Element::Listener *elementListener) :
		CodedSinkElement(session, elementListener, UINT_MAX, nullptr, 0)
{
	Element::setClassName(__func__);

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
	int res;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if ((mState != STARTED) && (mState != STARTING)) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}
	setState(STOPPING);

	res = internalStop();
	if (res < 0)
		return res;

	res = removeInputMedias();

	setState(STOPPED);

	return 0;
}


int Muxer::addInputMedia(CodedVideoMedia *media)
{
	int res;
	CodedChannel *channel = nullptr;
	struct mbuf_coded_video_frame_queue *queue = nullptr;

	CodedSink::lock();

	res = CodedSink::addInputMedia(media);
	if (res == -EEXIST) {
		CodedSink::unlock();
		return res;
	} else if (res < 0) {
		CodedSink::unlock();
		PDRAW_LOG_ERRNO("Sink::addInputMedia", -res);
		return res;
	}

	channel = getInputChannel(media);
	if (channel == nullptr) {
		CodedSink::unlock();
		res = -ENODEV;
		PDRAW_LOG_ERRNO("Sink::getInputChannel", -res);
		goto error;
	}
	channel->setKey(this);

	res = mbuf_coded_video_frame_queue_new(&queue);
	if (res < 0) {
		CodedSink::unlock();
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_new", -res);
		goto error;
	}
	channel->setQueue(queue);

	res = addQueueEvtToLoop(queue, mSession->getLoop());
	if (res < 0)
		goto error;

	CodedSink::unlock();

	return 0;

error:
	removeInputMedia(media);
	CodedSink::unlock();
	return res;
}


int Muxer::removeInputMedia(CodedVideoMedia *media)
{
	int res;

	CodedSink::lock();

	CodedChannel *channel = getInputChannel(media);
	if (channel == nullptr) {
		CodedSink::unlock();
		res = -ENODEV;
		PDRAW_LOG_ERRNO("Sink::getInputChannel", -res);
		return res;
	}

	/* Keep a reference on the queue to destroy it after removing the
	 * input media (avoids deadlocks when trying to push new frames out
	 * of upstream elements whereas the queue is already destroyed) */
	struct mbuf_coded_video_frame_queue *queue = channel->getQueue();

	res = CodedSink::removeInputMedia(media);
	if (res < 0) {
		CodedSink::unlock();
		PDRAW_LOG_ERRNO("Sink::removeInputMedia", -res);
		return res;
	}

	if (queue != nullptr) {
		res = removeQueueEvtFromLoop(queue, mSession->getLoop());
		res = mbuf_coded_video_frame_queue_flush(queue);
		if (res < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_flush",
					-res);
		res = mbuf_coded_video_frame_queue_destroy(queue);
		if (res < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_destroy",
					-res);
	}

	CodedSink::unlock();

	return 0;
}


int Muxer::removeInputMedias(void)
{
	int res, inputMediaCount, i;

	CodedSink::lock();

	inputMediaCount = getInputMediaCount();

	/* Note: loop downwards because calling removeInputMedia removes
	 * input ports and decreases the media count */
	for (i = inputMediaCount - 1; i >= 0; i--) {
		CodedVideoMedia *media = getInputMedia(i);
		if (media == nullptr) {
			PDRAW_LOG_ERRNO("getInputMedia", ENOENT);
			continue;
		}
		res = removeInputMedia(media);
	}

	CodedSink::unlock();

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


void Muxer::onChannelTeardown(CodedChannel *channel)
{
	CodedSink::onChannelTeardown(channel);

	if (!mInputPorts.empty())
		return;

	int ret = stop();
	if (ret < 0)
		PDRAW_LOG_ERRNO("stop", -ret);
}

} /* namespace Pdraw */
