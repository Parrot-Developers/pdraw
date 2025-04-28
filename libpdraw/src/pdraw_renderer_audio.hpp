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

#ifndef _PDRAW_RENDERER_AUDIO_HPP_
#define _PDRAW_RENDERER_AUDIO_HPP_

#include "pdraw_element.hpp"

#include <pdraw/pdraw.hpp>

namespace Pdraw {

class AudioRendererWrapper;


class AudioRenderer : public SinkElement {
public:
	virtual ~AudioRenderer(void);

	virtual int setMediaId(unsigned int mediaId) = 0;

	virtual unsigned int getMediaId(void) const = 0;

	virtual int
	setParams(const struct pdraw_audio_renderer_params *params) = 0;

	virtual int getParams(struct pdraw_audio_renderer_params *params) = 0;

	static AudioRenderer *
	create(Session *session,
	       Element::Listener *listener,
	       AudioRendererWrapper *wrapper,
	       IPdraw::IAudioRenderer::Listener *rndListener,
	       unsigned int mediaId,
	       const struct pdraw_audio_renderer_params *params);

protected:
	AudioRenderer(Session *session,
		      Element::Listener *listener,
		      AudioRendererWrapper *wrapper,
		      IPdraw::IAudioRenderer::Listener *rndListener,
		      uint32_t mediaTypeCaps,
		      const struct adef_format *audioMediaFormatCaps,
		      int audioMediaFormatCapsCount,
		      unsigned int mediaId,
		      const struct pdraw_audio_renderer_params *params);

	void removeRendererListener(void);

	void asyncCompleteStop(void);

	virtual void completeStop(void) = 0;

	IPdraw::IAudioRenderer *mRenderer;
	IPdraw::IAudioRenderer::Listener *mRendererListener;
	pthread_mutex_t mListenerMutex;

private:
	static void idleCompleteStop(void *userdata);
};


class AudioRendererWrapper : public IPdraw::IAudioRenderer,
			     public ElementWrapper {
public:
	AudioRendererWrapper(Session *session,
			     unsigned int mediaId,
			     const struct pdraw_audio_renderer_params *params,
			     IPdraw::IAudioRenderer::Listener *listener);

	~AudioRendererWrapper(void);

	int setMediaId(unsigned int mediaId) override;

	unsigned int getMediaId(void) override;

	int
	setParams(const struct pdraw_audio_renderer_params *params) override;

	int getParams(struct pdraw_audio_renderer_params *params) override;

	void clearElement(void) override
	{
		ElementWrapper::clearElement();
		mRenderer = nullptr;
	}

	Sink *getSink()
	{
		return mRenderer;
	}

	AudioRenderer *getAudioRenderer()
	{
		return mRenderer;
	}

private:
	AudioRenderer *mRenderer;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_RENDERER_AUDIO_HPP_ */
