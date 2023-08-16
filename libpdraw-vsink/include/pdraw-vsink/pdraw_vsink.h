/**
 * Parrot Drones Awesome Video Viewer
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

#ifndef _PDRAW_VSINK_H_
#define _PDRAW_VSINK_H_

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


/**
 * Create a pdraw_vsink instance and connect to a URL.
 * The instance handle is returned through the ret_obj parameter.
 * When no longer needed, the instance must be freed using the
 * pdraw_vsink_stop() function.
 * @param url: URL to open (network URL or local file)
 * @param media_info: media info pointer to fill if not NULL.
 * Note: mdeia_info will be freed in pdraw_vsink_stop().
 * @param ret_obj: pdraw_vsink instance handle (output)
 * @param : timeout in second to connect (input)
 * Note: 0 doesn't use a timeout
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_VSINK_API int pdraw_vsink_start(const char *url,
				      struct pdraw_media_info **media_info,
				      struct pdraw_vsink **ret_obj,
					  time_t timeout_seconds);


/**
 * Stop and destroy a pdraw_vsink instance.
 * @param self: pdraw_vsink instance handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_VSINK_API int pdraw_vsink_stop(struct pdraw_vsink *self);


/**
 * Get a frame.
 * The caller can pass a mbuf_mem object to hold the frame. If not given, this
 * call will allocate a memory internally.
 * @param self: pdraw_vsink instance handle
 * @param frame_memory: memory used by the frame (optional)
 * @param frame_info: frame information (output)
 * @param ret_frame: frame (output)
 * @param timeout_seconds: timeout in second to get frame (input)
 * Note: 0 doesn't use a timeout
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_VSINK_API int
pdraw_vsink_get_frame(struct pdraw_vsink *self,
		      struct mbuf_mem *frame_memory,
		      struct pdraw_video_frame *frame_info,
		      struct mbuf_raw_video_frame **ret_frame,
			  time_t timeout_seconds);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_LIBPDRAW_VSINK_H_ */
