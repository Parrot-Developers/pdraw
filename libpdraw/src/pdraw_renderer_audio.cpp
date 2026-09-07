/**
 * Parrot Drones Audio and Video Vector library
 * Audio renderer interface
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

#define ULOG_TAG pdraw_rndaud
#include <ulog.h>

#include "pdraw_renderer_audio.hpp"
#if defined(PDRAW_USE_ALSA)
#	include "pdraw_renderer_audio_alsa.hpp"
#endif
#include "pdraw_session.hpp"

#include <errno.h>

ULOG_DECLARE_TAG(ULOG_TAG);

namespace Pdraw {

std::unique_ptr<AudioRenderer> AudioRenderer::create(
	[[maybe_unused]] Session *session,
	[[maybe_unused]] Element::Listener *listener,
	[[maybe_unused]] AudioRendererWrapper *wrapper,
	[[maybe_unused]] IPdraw::IAudioRenderer::Listener *rndListener,
	[[maybe_unused]] unsigned int mediaId,
	[[maybe_unused]] const struct pdraw_audio_renderer_params *params)
{
#if defined(PDRAW_USE_ALSA)
	try {
		return std::make_unique<AlsaAudioRenderer>(
			session,
			listener,
			wrapper,
			rndListener,
			static_cast<uint32_t>(Media::Type::AUDIO),
			mediaId,
			params);
	} catch (const std::bad_alloc &) {
		ULOGE("%s: failed to allocate renderer", __func__);
		return nullptr;
	}
#else
	ULOGE("no audio renderer implementation found");
	return nullptr;
#endif /* PDRAW_USE_ALSA */
}


AudioRenderer::AudioRenderer(
	Session *session,
	Element::Listener *listener,
	AudioRendererWrapper *wrapper,
	IPdraw::IAudioRenderer::Listener *rndListener,
	[[maybe_unused]] uint32_t mediaTypeCaps,
	const struct adef_format *audioMediaFormatCaps,
	int audioMediaFormatCapsCount,
	[[maybe_unused]] unsigned int mediaId,
	[[maybe_unused]] const struct pdraw_audio_renderer_params *params) :
		SinkElement(session,
			    listener,
			    wrapper,
			    1,
			    nullptr,
			    0,
			    nullptr,
			    0,
			    audioMediaFormatCaps,
			    audioMediaFormatCapsCount),
		mRenderer(wrapper), mRendererListener(rndListener)
{

	mCompleteStopHandler.set([this] { idleCompleteStop(); });
}


AudioRenderer::~AudioRenderer()
{
	/* Remove any leftover idle callbacks */
	mSession->getPompLoop()->idleRemove(this);
}


void AudioRenderer::removeRendererListener()
{
	std::scoped_lock lock(mListenerMutex);
	mRendererListener = nullptr;
}


void AudioRenderer::asyncCompleteStop()
{
	int err = mSession->getPompLoop()->idleAdd(&mCompleteStopHandler, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("Loop::idleAdd", -err);
}


/* Listener call from an idle function */
void AudioRenderer::idleCompleteStop()
{
	completeStop();
}


AudioRendererWrapper::AudioRendererWrapper(
	Session *session,
	unsigned int mediaId,
	const struct pdraw_audio_renderer_params *params,
	IPdraw::IAudioRenderer::Listener *listener)
{
	auto impl = Pdraw::AudioRenderer::create(
		session, session, this, listener, mediaId, params);
	mRenderer = impl.get();
	mElement = impl.release();
}


AudioRendererWrapper::~AudioRendererWrapper()
{
	if (isElementStopped())
		return;
	int ret = mRenderer->stop();
	if (ret < 0)
		ULOG_ERRNO("AudioRenderer::stop", -ret);
}


int AudioRendererWrapper::setMediaId(unsigned int mediaId)
{
	if (isElementStopped())
		return -EPROTO;

	return mRenderer->setMediaId(mediaId);
}


unsigned int AudioRendererWrapper::getMediaId()
{
	if (isElementStopped())
		return -EPROTO;

	return mRenderer->getMediaId();
}


int AudioRendererWrapper::setParams(
	const struct pdraw_audio_renderer_params *params)
{
	if (isElementStopped())
		return -EPROTO;

	return mRenderer->setParams(params);
}


int AudioRendererWrapper::getParams(struct pdraw_audio_renderer_params *params)
{
	if (isElementStopped())
		return -EPROTO;

	return mRenderer->getParams(params);
}

} /* namespace Pdraw */
