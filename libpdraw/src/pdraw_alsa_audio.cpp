/**
 * Parrot Drones Audio and Video Vector library
 * ALSA audio
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


#define ULOG_TAG pdraw_alsaaudio
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_alsa_audio.hpp"

#ifdef PDRAW_USE_ALSA

#	define ARRAY_SIZE(x) (sizeof(x) / sizeof(*(x)))


namespace Pdraw {


static const struct {
	enum _snd_pcm_format sndFormat;
	const struct adef_format *adefFormat;
} audio_format_map[] = {
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_8000hz_stereo},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_8000hz_mono},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_11025hz_mono},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_11025hz_stereo},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_12000hz_mono},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_12000hz_stereo},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_16000hz_mono},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_16000hz_stereo},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_22050hz_mono},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_22050hz_stereo},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_24000hz_mono},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_24000hz_stereo},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_32000hz_mono},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_32000hz_stereo},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_44100hz_mono},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_44100hz_stereo},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_48000hz_mono},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_48000hz_stereo},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_64000hz_mono},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_64000hz_stereo},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_88200hz_mono},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_88200hz_stereo},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_96000hz_mono},
	{SND_PCM_FORMAT_S16_LE, &adef_pcm_16b_96000hz_stereo},
};


snd_pcm_format_t AlsaAudio::adefFormatToAlsa(const struct adef_format *format)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(audio_format_map); i++) {
		if (adef_format_cmp(audio_format_map[i].adefFormat, format))
			return audio_format_map[i].sndFormat;
	}

	return SND_PCM_FORMAT_UNKNOWN;
}


const struct adef_format *AlsaAudio::alsaFormatToAdef(snd_pcm_format_t format)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(audio_format_map); i++) {
		if (audio_format_map[i].sndFormat == format)
			return audio_format_map[i].adefFormat;
	}

	return nullptr;
}

} /* namespace Pdraw */

#endif /* PDRAW_USE_ALSA */
