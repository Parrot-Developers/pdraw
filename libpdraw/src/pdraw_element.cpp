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

#include "pdraw_element.hpp"

#include <errno.h>

#define ULOG_TAG pdraw_element
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_element);

namespace Pdraw {


Element::Element(Session *session, Listener *listener) :
		mSession(session), mListener(listener), mState(INVALID),
		mName("")
{
	int res;
	pthread_mutexattr_t attr;
	bool attr_created = false;

	res = pthread_mutexattr_init(&attr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutexattr_init", res);
		goto error;
	}
	attr_created = true;

	res = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutexattr_settype", res);
		goto error;
	}

	res = pthread_mutex_init(&mMutex, &attr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutex_init", res);
		goto error;
	}

	pthread_mutexattr_destroy(&attr);
	return;

error:
	if (attr_created)
		pthread_mutexattr_destroy(&attr);
}


Element::~Element(void)
{
	mState = INVALID;
	pthread_mutex_destroy(&mMutex);
}


void Element::lock(void)
{
	pthread_mutex_lock(&mMutex);
}


void Element::unlock(void)
{
	pthread_mutex_unlock(&mMutex);
}


Element::State Element::getState(void)
{
	pthread_mutex_lock(&mMutex);
	Element::State ret = mState;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


void Element::setState(Element::State state)
{
	pthread_mutex_lock(&mMutex);
	if (state == mState) {
		pthread_mutex_unlock(&mMutex);
		return;
	}

	mState = state;
	pthread_mutex_unlock(&mMutex);
	ULOGI("'%s': element state change to %s",
	      mName.c_str(),
	      getElementStateStr(state));

	if (mListener)
		mListener->onElementStateChanged(this, state);
}


void Element::setStateAsyncNotify(Element::State state)
{
	pthread_mutex_lock(&mMutex);
	if (state == mState) {
		pthread_mutex_unlock(&mMutex);
		return;
	}

	mState = state;
	pthread_mutex_unlock(&mMutex);
	ULOGI("'%s': element state change to %s (async notify)",
	      mName.c_str(),
	      getElementStateStr(state));

	if (mListener)
		mListener->asyncElementStateChange(this, state);
}


const char *Element::getElementStateStr(Element::State val)
{
	switch (val) {
	case Element::State::INVALID:
		return "INVALID";
	case Element::State::CREATED:
		return "CREATED";
	case Element::State::STARTING:
		return "STARTING";
	case Element::State::STARTED:
		return "STARTED";
	case Element::State::STOPPING:
		return "STOPPING";
	case Element::State::STOPPED:
		return "STOPPED";
	default:
		return NULL;
	}
}

} /* namespace Pdraw */
