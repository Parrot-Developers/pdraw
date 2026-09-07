/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — test doubles implementation
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

#define ULOG_TAG pdraw_test_mocks
#include "test_mocks.hpp"
#include "test_common.h"

ULOG_DECLARE_TAG(ULOG_TAG);

namespace PdrawTest {

/* Accept H.264 byte-stream coded video; vdef_h264_byte_stream is a
 * pre-defined global constant from <video-defs/vdefs.h>. */
const struct vdef_coded_format kTestSinkCodedCaps[1] = {vdef_h264_byte_stream};

/* Accept I420 planar raw video. */
const struct vdef_raw_format kTestSinkRawCaps[1] = {vdef_i420};

/* Accept AAC-LC 16-bit 44.1 kHz stereo ADTS audio. */
const struct adef_format kTestSinkAudioCaps[1] = {
	adef_aac_lc_16b_44100hz_stereo_adts};

} /* namespace PdrawTest */
