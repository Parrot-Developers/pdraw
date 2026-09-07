/**
 * Parrot Drones Audio and Video Vector
 * Video sink wrapper library
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

/*
 * C-ABI bridge on top of the public C++ interface (pdraw_vsink.hpp) --
 * mirrors packages/libpdraw-overlayer/src/pdraw_overlayer_wrapper.cpp and
 * packages/pdraw/libpdraw-backend/src/pdraw_backend_wrapper.cpp exactly:
 * only catch std::bad_alloc tightly around the allocating call, no broader
 * exception safety net (matches the only pattern used anywhere in this
 * codebase's C-ABI wrapper layers).
 */

#include <errno.h>
#include <memory>

#include <pdraw-vsink/pdraw_vsink.h>
#include <pdraw-vsink/pdraw_vsink.hpp>

#define ULOG_TAG pdraw_vsink
#include <ulog.h>

namespace VsinkWrapper {

/* Bridges the C++-native IPdrawVsink::Listener mechanism back to the C
 * API's struct pdraw_vsink_cbs -- mirrors pdraw_wrapper.cpp's own
 * PdrawListener class exactly (there, the C++ core's Listener is the native
 * path and the C API is a thin adapter built on top of it). */
class PdrawVsinkListener : public PdrawVsink::IPdrawVsink::Listener {
public:
	PdrawVsinkListener(const struct pdraw_vsink_cbs &cbs, void *userdata) :
			mCbs(cbs), mUserdata(userdata)
	{
	}

	void onFrameReady([[maybe_unused]] PdrawVsink::IPdrawVsink *vsink,
			  struct pdraw_vsink_frame *frame,
			  const struct pdraw_video_frame *frameInfo) override
	{
		if (mCbs.frame_ready != nullptr)
			(*mCbs.frame_ready)(frame, frameInfo, mUserdata);
	}

private:
	struct pdraw_vsink_cbs mCbs;
	void *mUserdata = nullptr;
};

} /* namespace VsinkWrapper */

struct pdraw_vsink {
	/* Declared before impl so it outlives it: impl's destructor (~Vsink())
	 * synchronously stops the internal thread and detaches its queue
	 * event(s) before returning, so no onFrameReady() call can still be
	 * in flight by the time listener is torn down. */
	std::unique_ptr<VsinkWrapper::PdrawVsinkListener> listener;
	std::unique_ptr<PdrawVsink::IPdrawVsink> impl;
};

extern "C" {

int pdraw_vsink_start(const struct pdraw_vsink_params *params,
		      const struct pdraw_vsink_cbs *cbs,
		      void *cbsUserdata,
		      struct pdraw_media_info **mediaInfo,
		      struct pdraw_vsink **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	std::unique_ptr<struct pdraw_vsink> self;
	try {
		self = std::make_unique<struct pdraw_vsink>();
		if (cbs != nullptr && cbs->frame_ready != nullptr) {
			self->listener = std::make_unique<
				VsinkWrapper::PdrawVsinkListener>(*cbs,
								  cbsUserdata);
		}
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create pdraw_vsink instance");
		return -ENOMEM;
	}

	PdrawVsink::IPdrawVsink *vsink = nullptr;
	int res = PdrawVsink::createPdrawVsink(
		params, self->listener.get(), mediaInfo, &vsink);
	if (res < 0)
		return res;
	self->impl.reset(vsink);

	*retObj = self.release();
	return 0;
}


int pdraw_vsink_stop(struct pdraw_vsink *self)
{
	if (self == nullptr)
		return 0;

	std::unique_ptr<struct pdraw_vsink> owner(self);
	return 0;
}


int pdraw_vsink_get_frame(struct pdraw_vsink *self,
			  int timeoutMs,
			  struct mbuf_mem *frameMemory,
			  struct pdraw_video_frame *frameInfo,
			  struct pdraw_vsink_frame *retFrame)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == nullptr, EINVAL);

	return self->impl->getFrame(
		timeoutMs, frameMemory, frameInfo, retFrame);
}

} /* extern "C" */
