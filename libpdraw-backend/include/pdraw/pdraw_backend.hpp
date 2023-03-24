/**
 * Parrot Drones Awesome Video Viewer
 * PDrAW back-end library
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

#ifndef _PDRAW_BACKEND_HPP_
#define _PDRAW_BACKEND_HPP_

#include <inttypes.h>

#include <string>

#include <libpomp.h>
#include <pdraw/pdraw.hpp>

/* To be used for all public API */
#ifdef PDRAW_BACKEND_API_EXPORTS
#	ifdef _WIN32
#		define PDRAW_BACKEND_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define PDRAW_BACKEND_API __attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !PDRAW_BACKEND_API_EXPORTS */
#	define PDRAW_BACKEND_API
#endif /* !PDRAW_BACKEND_API_EXPORTS */

using namespace Pdraw;

namespace PdrawBackend {


/* PDrAW back-end object interface;
 * see the createPdrawBackend() function for creating a PDrAW back-end instance
 */
class IPdrawBackend : public IPdraw {
public:
	/**
	 * Destroy a PDrAW back-end instance.
	 * This function frees all resources associated with a PDrAW back-end
	 * instance. The stop() function must be called and one must wait for
	 * the stopResponse() listener function to be called prior to destroying
	 * the PDrAW back-end instance.
	 */
	virtual ~IPdrawBackend(void) {}

	/**
	 * Start a PDrAW back-end instance.
	 * This function must be called after the PDrAW back-end object creation
	 * prior to calling any other function.
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int start(void) = 0;

	/**
	 * Stop a PDrAW back-end instance.
	 * This function must be called prior to destroying the PDrAW back-end
	 * instance.
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int stop(void) = 0;

	/**
	 * Get the PDrAW back-end internal event loop.
	 * This function returns a pointer to the event loop used internally.
	 * @return a pointer on the internal loop on success,
	 *         nullptr in case of error
	 */
	virtual struct pomp_loop *getLoop(void) = 0;
};


/**
 * Create a PDrAW back-end instance.
 * All API functions can be called from any thread, with the exception of
 * rendering functions which must still be called from the rendering thread.
 * All listener functions with the exception of video renderer listener
 * functions are called from the internal pomp_loop thread; synchronization
 * is up to the application. A valid listener must be provided. The instance
 * handle is returned through the retObj parameter.
 * When no longer needed, the instance must be deleted to free the resources.
 * The stop() function must be called and one must wait for the stopResponse()
 * listener function to be called prior to destroying the instance.
 * @param listener: listener functions implementation
 * @param retObj: PDrAW back-end instance handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int createPdrawBackend(IPdraw::Listener *listener,
					 IPdrawBackend **retObj);


} /* namespace PdrawBackend */

#endif /* !_PDRAW_BACKEND_HPP_ */
