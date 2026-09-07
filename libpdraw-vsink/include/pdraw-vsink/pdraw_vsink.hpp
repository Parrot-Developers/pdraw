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

#pragma once

#include <pdraw-vsink/pdraw_vsink_defs.h>

namespace PdrawVsink {

/**
 * PDrAW video sink instance interface.
 * See the createPdrawVsink() function for creating an instance.
 * getFrame() must not be used if a Listener was provided at creation time
 * (see IPdrawVsink::Listener); onFrameReady() is invoked from this
 * instance's own internal thread.
 */
class IPdrawVsink {
public:
	/**
	 * PDrAW video sink instance listener, the C++-native equivalent of
	 * the C API's struct pdraw_vsink_cbs. Registered at creation time
	 * (see createPdrawVsink()); when provided, frames are pushed to
	 * onFrameReady() instead of being retrieved via getFrame().
	 */
	class Listener {
	public:
		virtual ~Listener() = default;

		/**
		 * Called when a frame is ready.
		 * The frame does not need to be unreferenced in this
		 * function, but if the frame needs to be kept to be used
		 * later it should be referenced and later unreferenced when
		 * no longer needed by calling
		 * mbuf_raw_video_frame_ref()/mbuf_coded_video_frame_ref()
		 * and the matching _unref() function.
		 * This function is called from the vsink instance's own
		 * internal thread; the caller must ensure thread safety and
		 * should keep in mind that this thread is blocked by this
		 * function call.
		 * @param vsink: the PDrAW video sink instance
		 * @param frame: the frame
		 * @param frameInfo: information about the frame
		 */
		virtual void
		onFrameReady(IPdrawVsink *vsink,
			     struct pdraw_vsink_frame *frame,
			     const struct pdraw_video_frame *frameInfo) = 0;
	};

	/**
	 * Destroy a PDrAW video sink instance.
	 * This function frees all resources associated with the instance.
	 * Equivalent to the C API's pdraw_vsink_stop().
	 */
	virtual ~IPdrawVsink() = default;

	/**
	 * Get a frame.
	 * The caller can pass a mbuf_mem object to hold the frame. If not
	 * given, this call will allocate a memory internally. The returned
	 * frame is properly referenced, so the caller will need to call
	 * mbuf_raw_video_frame_unref()/mbuf_coded_video_frame_unref() when
	 * the frame is no longer needed.
	 * @param timeoutMs: timeout of wait, 0 to return immediately
	 *                   (non-blocking mode) or -1 for infinite wait
	 * @param frameMemory: memory used by the frame (optional)
	 * @param frameInfo: frame information (output)
	 * @param retFrame: frame (output)
	 * @return 0 in case of success, -ETIMEDOUT if timeout occurred,
	 *         negative errno value in case of error
	 */
	virtual int getFrame(int timeoutMs,
			     struct mbuf_mem *frameMemory,
			     struct pdraw_video_frame *frameInfo,
			     struct pdraw_vsink_frame *retFrame) = 0;
};


/**
 * Create a PDrAW video sink instance and connect to a URL.
 * The parameters structure is mandatory but only the url field needs to be
 * filled (network URL or local file); other fields are optional and can be
 * left at their default value. Pass a listener (see IPdrawVsink::Listener)
 * to receive frames asynchronously, or nullptr to use getFrame() polling
 * instead -- this is the C++ equivalent of the C API's separate cbs/
 * cbs_userdata parameters to pdraw_vsink_start(). The camera_type value can
 * be left at
 * VMETA_CAMERA_TYPE_UNKNOWN to select the default camera. The instance
 * handle is returned through the retObj parameter. When no longer needed,
 * the instance must be freed by deleting it.
 * @param params: instance parameters
 * @param listener: instance listener, to receive frames asynchronously
 *                  (optional, can be null)
 * @param mediaInfo: optional pointer to a media info structure (output)
 *                   Note: ownership of this structure is transferred to
 *                   the caller, who must free it with pdraw_media_info_free()
 * @param retObj: PDrAW video sink instance handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_VSINK_API int createPdrawVsink(const struct pdraw_vsink_params *params,
				     IPdrawVsink::Listener *listener,
				     struct pdraw_media_info **mediaInfo,
				     IPdrawVsink **retObj);

} /* namespace PdrawVsink */
