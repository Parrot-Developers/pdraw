/**
 * Parrot Drones Audio and Video Vector
 * Video sink wrapper library
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

#pragma once

#include <inttypes.h>
#include <pdraw/pdraw_defs.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* To be used for all public API */
#ifdef PDRAW_VSINK_API_EXPORTS
#	ifdef _WIN32
#		define PDRAW_VSINK_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define PDRAW_VSINK_API __attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !PDRAW_VSINK_API_EXPORTS */
#	define PDRAW_VSINK_API
#endif /* !PDRAW_VSINK_API_EXPORTS */


/* Forward declarations */
struct pdraw_vsink;


/* Callback functions */
struct pdraw_vsink_cbs {
	/* Called when a frame is ready (optional, can be null).
	 * The frame does not need to be unreferenced in the callback
	 * implementation, but if the frame needs to be kept to be used later by
	 * the application it should be referenced and later unreferenced when
	 * no longer needed by calling mbuf_raw_video_frame_ref() and
	 * mbuf_raw_video_frame_unref().
	 * This function is called from the pdraw_vsink internal thread.
	 * Note: the caller must ensure thread safety and should keep in mind
	 * that the pdraw_vsink thread is blocked by this function call.
	 * @param frame: the mbuf_raw_video_frame structure
	 * @param frame_info: information about the frame
	 * @param userdata: user data pointer */
	void (*frame_ready)(struct mbuf_raw_video_frame *frame,
			    struct pdraw_video_frame *frame_info,
			    void *userdata);
};


/* Instance parameters */
struct pdraw_vsink_params {
	/* Network URL or local file (mandatory) */
	const char *url;

	/* Playback synchronization mode.
	 * - PDRAW_PLAYBACK_MODE_REALTIME: Frames are delivered following the
	 * original stream timestamps (synchronized with the clock).
	 * - PDRAW_PLAYBACK_MODE_OFFLINE: Frames are delivered as fast as the
	 * demuxing and decoding pipeline allows, ignoring real-time pacing. */
	enum pdraw_playback_mode playback_mode;

	/* Camera type to select (optional; set to VMETA_CAMERA_TYPE_UNKNOWN to
	 * select the default camera) */
	enum vmeta_camera_type camera_type;

	/* Callback functions (optional; if the frame_ready callback is not
	 * provided, use the pdraw_vsink_get_frame() function to retrieve
	 * frames) */
	struct pdraw_vsink_cbs cbs;

	/* Callback functions user data pointer */
	void *cbs_userdata;
};


/**
 * Create a pdraw_vsink instance and connect to a URL.
 * The parameters structure is mandatory but only the url field needs to be
 * filled (network URL or local file); other fields are optional and can be
 * null. The camera_type value can be set to VMETA_CAMERA_TYPE_UNKNOWN to select
 * the default camera. The instance handle is returned through the ret_obj
 * parameter. When no longer needed, the instance must be freed using the
 * pdraw_vsink_stop() function.
 * @param params: instance parameters
 * @param media_info: optional pointer to a media info structure (output)
 *                    Note: the ownership of the memory stays with the library
 *                    instance and will be freed in pdraw_vsink_stop().
 * @param ret_obj: pdraw_vsink instance handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_VSINK_API int pdraw_vsink_start(const struct pdraw_vsink_params *params,
				      struct pdraw_media_info **media_info,
				      struct pdraw_vsink **ret_obj);


/**
 * Stop and destroy a pdraw_vsink instance.
 * @param self: pdraw_vsink instance handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_VSINK_API int pdraw_vsink_stop(struct pdraw_vsink *self);


/**
 * Get a frame.
 * The caller can pass a mbuf_mem object to hold the frame. If not given, this
 * call will allocate a memory internally. The returned frame is properly
 * referenced, so the caller will need to call mbuf_raw_video_frame_unref() when
 * the frame is no longer needed.
 * Note: this function cannot be used if a get_frame_cb_t callback function has
 * been provided in the initial parameters.
 * @param self: pdraw_vsink instance handle
 * @param timeout_ms: timeout of wait, 0 to return immediately (non-blocking
 *                    mode) or -1 for infinite wait
 * @param frame_memory: memory used by the frame (optional)
 * @param frame_info: frame information (output)
 * @param ret_frame: frame (output)
 * @return 0 in case of success, -ETIMEDOUT if timeout occurred,
 *         negative errno value in case of error
 */
PDRAW_VSINK_API int
pdraw_vsink_get_frame(struct pdraw_vsink *self,
		      int timeout_ms,
		      struct mbuf_mem *frame_memory,
		      struct pdraw_video_frame *frame_info,
		      struct mbuf_raw_video_frame **ret_frame);


#ifdef __cplusplus
}
#endif /* __cplusplus */
