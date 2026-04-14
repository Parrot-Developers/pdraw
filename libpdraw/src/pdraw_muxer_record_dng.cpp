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

#define ULOG_TAG pdraw_recmux_dng
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_muxer_record_dng.hpp"
#include "pdraw_muxer_record_dng_media.hpp"

#ifdef BUILD_LIBDNG_PARROT

#	include <array>

namespace Pdraw {


constexpr size_t NB_SUPPORTED_RAW_FORMATS = 12;
static std::array<vdef_raw_format, NB_SUPPORTED_RAW_FORMATS>
	supportedRawFormats;
static std::once_flag supportedFormatsOnceFlag;
static void initializeSupportedFormats()
{
	supportedRawFormats[0] = vdef_bayer_rggb;
	supportedRawFormats[1] = vdef_bayer_bggr;
	supportedRawFormats[2] = vdef_bayer_grbg;
	supportedRawFormats[3] = vdef_bayer_gbrg;
	supportedRawFormats[4] = vdef_bayer_rggb_10_packed;
	supportedRawFormats[5] = vdef_bayer_bggr_10_packed;
	supportedRawFormats[6] = vdef_bayer_grbg_10_packed;
	supportedRawFormats[7] = vdef_bayer_gbrg_10_packed;
	supportedRawFormats[8] = vdef_bayer_rggb_10;
	supportedRawFormats[9] = vdef_bayer_bggr_10;
	supportedRawFormats[10] = vdef_bayer_grbg_10;
	supportedRawFormats[11] = vdef_bayer_gbrg_10;
}


enum dng_bayer_phase mapVdefToDngPhase(vdef_raw_pix_order order)
{
	switch (order) {
	case VDEF_RAW_PIX_ORDER_RGGB:
		return DNG_BAYER_RGGB;
	case VDEF_RAW_PIX_ORDER_BGGR:
		return DNG_BAYER_BGGR;
	case VDEF_RAW_PIX_ORDER_GRBG:
		return DNG_BAYER_GRBG;
	case VDEF_RAW_PIX_ORDER_GBRG:
		return DNG_BAYER_GBRG;
	default:
		return DNG_BAYER_UNKNOWN;
	}
}


DngRecordMuxer::DngRecordMuxer(Session *session,
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

	setRawVideoMediaFormatCaps(supportedRawFormats.data(),
				   supportedRawFormats.size());
}


/* Called on the writer thread */
std::unique_ptr<RecordMuxer::MuxerMedia>
DngRecordMuxer::DngRecordMuxer::createMedia(const MuxerMediaConfig &cfg)
{
	try {
		switch (cfg.type) {
		case Media::Type::RAW_VIDEO:
			return make_unique<DngMuxerMedia>(this, cfg);
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


/* Called on the writer thread */
int DngRecordMuxer::internalSetThumbnail(enum pdraw_muxer_thumbnail_type type,
					 const uint8_t *data,
					 size_t size)
{
	PDRAW_CHECK_WRITER_THREAD(true);

	int ret = dng_mux_set_thumbnail(mDngMux, data, size);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("dng_mux_set_thumbnail", -ret);
		return ret;
	}

	return 0;
}


/* Called on the writer thread */
int DngRecordMuxer::internalSetFileMetadata(enum pdraw_muxer_metadata_type type,
					    const uint8_t *data,
					    size_t size,
					    const void *params,
					    size_t paramsSize)
{
	PDRAW_CHECK_WRITER_THREAD(true);

	if (mDngMux == nullptr)
		return -EPROTO;

	if (type == PDRAW_MUXER_METADATA_TYPE_DNG_LSC) {
		if (params == nullptr ||
		    paramsSize < sizeof(struct pdraw_muxer_dng_lsc_params))
			return -EINVAL;

		const auto *lscParams =
			static_cast<const struct pdraw_muxer_dng_lsc_params *>(
				params);
		int ret = dng_mux_set_lsc(
			mDngMux,
			data,
			size,
			lscParams->width,
			lscParams->height,
			lscParams->count,
			mapVdefToDngPhase(
				(vdef_raw_pix_order)lscParams->format));
		if (ret < 0) {
			PDRAW_LOG_ERRNO("dng_mux_set_lsc", -ret);
			return ret;
		}
		return 0;
	}

	return -ENOSYS;
}


/* Called on the writer thread */
void DngRecordMuxer::onInternalStopThread()
{
	PhotoRecordMuxer::onInternalStopThread();
	if (mDngMux) {
		dng_mux_close(mDngMux);
		mDngMux = nullptr;
	}
}


/* Called on the writer thread */
int DngRecordMuxer::onBeforeAddMuxerMedias()
{
	struct dng_mux_config config = {
		.filemode = 0644,
		.flags = 0,
	};

	int ret = dng_mux_open(&config, &mDngMux);
	if (ret < 0) {
		ULOG_ERRNO("dng_mux_open", -ret);
		return ret;
	}

	return PhotoRecordMuxer::onBeforeAddMuxerMedias();
}

} /* namespace Pdraw */

#endif /* BUILD_LIBDNG_PARROT */
