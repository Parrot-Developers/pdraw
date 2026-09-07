/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — AlsaAudio format conversion roundtrip (Tier A)
 *
 * Copyright (c) 2026 Parrot Drones SAS
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

/* Tier A — no fixture needed; AlsaAudio::adefFormatToAlsa() and
 * AlsaAudio::alsaFormatToAdef() are pure look-up functions.
 *
 * ROUNDTRIP SEMANTICS
 * audioFormatMap maps 24 distinct adef_format constants to a single ALSA
 * format (SND_PCM_FORMAT_S16_LE), so adefFormatToAlsa() is many-to-one.
 * alsaFormatToAdef() returns the first map entry for a given snd_pcm_format_t,
 * which is always &adef_pcm_16b_8000hz_stereo for SND_PCM_FORMAT_S16_LE.
 *
 *   alsa → adef → alsa  (lossless): SND_PCM_FORMAT_S16_LE
 *                        → &adef_pcm_16b_8000hz_stereo
 *                        → SND_PCM_FORMAT_S16_LE
 *
 *   adef → alsa → adef  (identity only for first entry): any adef format
 *                        → SND_PCM_FORMAT_S16_LE
 *                        → &adef_pcm_16b_8000hz_stereo (first map entry)
 */

#ifdef PDRAW_USE_ALSA
#	define PDRAW_TEST_ALSA_AUDIO_FORMAT_ENABLED 1
#endif

#define ULOG_TAG pdraw_test_alsa_audio_format
#include "test_common.h"

#ifdef PDRAW_TEST_ALSA_AUDIO_FORMAT_ENABLED
#	include <audio-defs/adefs.h>
#	include <ulog.h>

#	include "pdraw_alsa_audio.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;


/* All 24 adef_format constants present in audioFormatMap, in map order. */
static const struct adef_format *const kKnownFormats[] = {
	&adef_pcm_16b_8000hz_stereo, &adef_pcm_16b_8000hz_mono,
	&adef_pcm_16b_11025hz_mono,  &adef_pcm_16b_11025hz_stereo,
	&adef_pcm_16b_12000hz_mono,  &adef_pcm_16b_12000hz_stereo,
	&adef_pcm_16b_16000hz_mono,  &adef_pcm_16b_16000hz_stereo,
	&adef_pcm_16b_22050hz_mono,  &adef_pcm_16b_22050hz_stereo,
	&adef_pcm_16b_24000hz_mono,  &adef_pcm_16b_24000hz_stereo,
	&adef_pcm_16b_32000hz_mono,  &adef_pcm_16b_32000hz_stereo,
	&adef_pcm_16b_44100hz_mono,  &adef_pcm_16b_44100hz_stereo,
	&adef_pcm_16b_48000hz_mono,  &adef_pcm_16b_48000hz_stereo,
	&adef_pcm_16b_64000hz_mono,  &adef_pcm_16b_64000hz_stereo,
	&adef_pcm_16b_88200hz_mono,  &adef_pcm_16b_88200hz_stereo,
	&adef_pcm_16b_96000hz_mono,  &adef_pcm_16b_96000hz_stereo,
};

static constexpr size_t kKnownFormatCount =
	sizeof(kKnownFormats) / sizeof(kKnownFormats[0]);


/* ── adefFormatToAlsa ────────────────────────────────────────────────────── */

/* Every entry in audioFormatMap targets SND_PCM_FORMAT_S16_LE. */
static void testAdefToAlsaAllKnownFormats()
{
	for (size_t i = 0; i < kKnownFormatCount; i++)
		CU_ASSERT_EQUAL(AlsaAudio::adefFormatToAlsa(kKnownFormats[i]),
				SND_PCM_FORMAT_S16_LE);
}


/* A format not in audioFormatMap must return SND_PCM_FORMAT_UNKNOWN. */
static void testAdefToAlsaUnknownFormat()
{
	/* Build a PCM format that differs from every map entry (32-bit depth
	 * is absent from audioFormatMap which only contains 16-bit entries). */
	struct adef_format unknown = adef_pcm_16b_48000hz_stereo;
	unknown.bit_depth = 32;
	CU_ASSERT_EQUAL(AlsaAudio::adefFormatToAlsa(&unknown),
			SND_PCM_FORMAT_UNKNOWN);
}


/* ── alsaFormatToAdef ────────────────────────────────────────────────────── */

/* SND_PCM_FORMAT_S16_LE is the only ALSA format in the map; it must return
 * the first matching entry (&adef_pcm_16b_8000hz_stereo), and that entry must
 * carry the expected PCM attributes. */
static void testAlsaToAdefKnownFormat()
{
	const struct adef_format *fmt =
		AlsaAudio::alsaFormatToAdef(SND_PCM_FORMAT_S16_LE);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fmt);

	/* First map entry is always returned for SND_PCM_FORMAT_S16_LE. */
	CU_ASSERT_TRUE(adef_format_cmp(fmt, &adef_pcm_16b_8000hz_stereo));

	/* Spot-check the PCM attributes encoded in the constant. */
	CU_ASSERT_EQUAL(fmt->encoding, ADEF_ENCODING_PCM);
	CU_ASSERT_EQUAL(fmt->bit_depth, 16u);
	CU_ASSERT_EQUAL(fmt->channel_count, 2u); /* stereo */
	CU_ASSERT_EQUAL(fmt->sample_rate, 8000u);
	CU_ASSERT_TRUE(fmt->pcm.signed_val);
	CU_ASSERT_TRUE(fmt->pcm.little_endian);
}


/* ALSA formats absent from audioFormatMap must return nullptr. */
static void testAlsaToAdefUnknownFormats()
{
	CU_ASSERT_PTR_NULL(AlsaAudio::alsaFormatToAdef(SND_PCM_FORMAT_UNKNOWN));
	CU_ASSERT_PTR_NULL(AlsaAudio::alsaFormatToAdef(SND_PCM_FORMAT_S24_LE));
	CU_ASSERT_PTR_NULL(AlsaAudio::alsaFormatToAdef(SND_PCM_FORMAT_S32_LE));
	CU_ASSERT_PTR_NULL(AlsaAudio::alsaFormatToAdef(SND_PCM_FORMAT_U8));
}


/* ── Roundtrips ──────────────────────────────────────────────────────────── */

/* alsa → adef → alsa: lossless for SND_PCM_FORMAT_S16_LE.
 * alsaFormatToAdef() returns the first map entry; that entry must convert
 * back to the same ALSA format. */
static void testRoundtripAlsaToAdefToAlsa()
{
	const struct adef_format *adef =
		AlsaAudio::alsaFormatToAdef(SND_PCM_FORMAT_S16_LE);
	CU_ASSERT_PTR_NOT_NULL_FATAL(adef);
	CU_ASSERT_EQUAL(AlsaAudio::adefFormatToAlsa(adef),
			SND_PCM_FORMAT_S16_LE);
}


/* adef → alsa → adef: identity only for the first map entry.
 * For every other entry the forward leg maps to SND_PCM_FORMAT_S16_LE and
 * the reverse returns &adef_pcm_16b_8000hz_stereo (first entry), not the
 * original. We verify:
 *   a) the forward leg always produces SND_PCM_FORMAT_S16_LE, and
 *   b) the result of the reverse leg itself converts back to S16_LE
 *      (i.e. it is a valid, known adef format). */
static void testRoundtripAdefToAlsaToAdef()
{
	/* First entry: full identity roundtrip. */
	snd_pcm_format_t alsa =
		AlsaAudio::adefFormatToAlsa(&adef_pcm_16b_8000hz_stereo);
	CU_ASSERT_EQUAL(alsa, SND_PCM_FORMAT_S16_LE);
	const struct adef_format *back = AlsaAudio::alsaFormatToAdef(alsa);
	CU_ASSERT_PTR_NOT_NULL_FATAL(back);
	CU_ASSERT_TRUE(adef_format_cmp(back, &adef_pcm_16b_8000hz_stereo));

	/* All other entries: forward leg is S16_LE; reverse leg lands on
	 * the first map entry, which must itself round-trip cleanly. */
	for (size_t i = 1; i < kKnownFormatCount; i++) {
		alsa = AlsaAudio::adefFormatToAlsa(kKnownFormats[i]);
		CU_ASSERT_EQUAL(alsa, SND_PCM_FORMAT_S16_LE);
		back = AlsaAudio::alsaFormatToAdef(alsa);
		CU_ASSERT_PTR_NOT_NULL(back);
		if (back)
			CU_ASSERT_EQUAL(AlsaAudio::adefFormatToAlsa(back),
					SND_PCM_FORMAT_S16_LE);
	}
}

#endif /* PDRAW_TEST_ALSA_AUDIO_FORMAT_ENABLED */


CU_TestInfo g_pdraw_test_alsa_audio_format[] = {
#ifdef PDRAW_TEST_ALSA_AUDIO_FORMAT_ENABLED
	{FN("testAdefToAlsaAllKnownFormats"), testAdefToAlsaAllKnownFormats},
	{FN("testAdefToAlsaUnknownFormat"), testAdefToAlsaUnknownFormat},
	{FN("testAlsaToAdefKnownFormat"), testAlsaToAdefKnownFormat},
	{FN("testAlsaToAdefUnknownFormats"), testAlsaToAdefUnknownFormats},
	{FN("testRoundtripAlsaToAdefToAlsa"), testRoundtripAlsaToAdefToAlsa},
	{FN("testRoundtripAdefToAlsaToAdef"), testRoundtripAdefToAlsaToAdef},
#endif
	CU_TEST_INFO_NULL,
};
