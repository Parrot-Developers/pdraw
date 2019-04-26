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
 * @param ret_obj: pdraw_vsink instance handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_VSINK_API int pdraw_vsink_start(const char *url,
				      struct pdraw_vsink **ret_obj);


/**
 * Stop and destroy a pdraw_vsink instance.
 * @param self: pdraw_vsink instance handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_VSINK_API int pdraw_vsink_stop(struct pdraw_vsink *self);


/**
 * Get a frame.
 * The buffer to hold the frame data must be created by the caller but can
 * have a null capacity. The buffer will be reallocated by the function if
 * necessary. The ownership of the buffer stays with the caller and the buffer
 * should be unreferenced once no longer needed.
 * @param self: pdraw_vsink instance handle
 * @param timeout_ms: timeout in milliseconds (0 means return immediately,
 *                    negative value means wait forever)
 * @param frame: frame information (output)
 * @param buffer: buffer to be filled with the frame data
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_VSINK_API int pdraw_vsink_get_frame(struct pdraw_vsink *self,
					  int timeout_ms,
					  struct pdraw_video_frame *frame,
					  struct vbuf_buffer *buffer);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_LIBPDRAW_VSINK_H_ */
