/**
 * Parrot Drones Audio and Video Vector library
 * Record muxer
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

#define ULOG_TAG pdraw_recmux_png
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_muxer_record_png.hpp"
#include "pdraw_muxer_record_png_media.hpp"

#include <array>

namespace Pdraw {


constexpr size_t NB_SUPPORTED_CODED_FORMATS = 1;
static std::array<vdef_coded_format, NB_SUPPORTED_CODED_FORMATS>
	supportedCodedFormats;
static std::once_flag supportedFormatsOnceFlag;
static void initializeSupportedFormats()
{
	supportedCodedFormats[0] = vdef_png;
}


PngRecordMuxer::PngRecordMuxer(Session *session,
			       Element::Listener *elementListener,
			       IPdraw::IMuxer::Listener *listener,
			       MuxerWrapper *wrapper,
			       const std::string &fileName,
			       const struct pdraw_muxer_params *params) :
		PhotoRecordMuxer(session,
				 elementListener,
				 listener,
				 wrapper,
				 fileName,
				 params)
{
	std::call_once(supportedFormatsOnceFlag, initializeSupportedFormats);

	Element::setClassName(__func__);

	setCodedVideoMediaFormatCaps(supportedCodedFormats.data(),
				     supportedCodedFormats.size());
}


/* Called on the writer thread */
std::unique_ptr<RecordMuxer::MuxerMedia>
PngRecordMuxer::PngRecordMuxer::createMedia(const MuxerMediaConfig &cfg)
{
	try {
		switch (cfg.type) {
		case Media::Type::CODED_VIDEO:
			return make_unique<PngMuxerMedia>(this, cfg);
		default:
			PDRAW_LOGE("unsupported media type: %d",
				   static_cast<int>(cfg.type));
			return nullptr;
		}
	} catch (const std::bad_alloc &) {
		PDRAW_LOGE("output media allocation failed");
		return nullptr;
	}
}

} /* namespace Pdraw */
