/**
 * Parrot Drones Awesome Video Viewer Library
 * Pipeline element
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

#ifndef _PDRAW_ELEMENT_HPP_
#define _PDRAW_ELEMENT_HPP_

#include "pdraw_media.hpp"

#include <errno.h>
#include <pthread.h>

#include <string>
#include <vector>

namespace Pdraw {

class Session;

class Element {
public:
	friend class Session;

	enum State {
		INVALID,
		CREATED,
		STARTING,
		STARTED,
		STOPPING,
		STOPPED,
	};

	class Listener {
	public:
		virtual ~Listener(void) {}

		virtual void onElementStateChanged(Element *element,
						   Element::State state) = 0;

		virtual void asyncElementStateChange(Element *element,
						     Element::State state) = 0;
	};

	virtual ~Element(void);

	virtual int start(void) = 0;

	virtual int stop(void) = 0;

	void lock(void);

	void unlock(void);

	Element::State getState(void);

	static const char *getElementStateStr(Element::State val);

protected:
	Element(Session *session, Listener *listener);

	void setState(Element::State state);

	void setStateAsyncNotify(Element::State state);

	Session *mSession;
	Listener *mListener;
	Element::State mState;
	pthread_mutex_t mMutex;
	std::string mName;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_ELEMENT_HPP_ */
