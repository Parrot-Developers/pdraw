/**
 * Parrot Drones Awesome Video Viewer Library
 * Session metadata
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

#ifndef _PDRAW_METADATA_SESSION_HPP_
#define _PDRAW_METADATA_SESSION_HPP_

#include "pdraw_utils.hpp"

#include <inttypes.h>
#include <pthread.h>

#include <string>
#include <vector>

namespace Pdraw {


class SessionSelfMetadata {
public:
	SessionSelfMetadata(void);

	~SessionSelfMetadata(void);

	void lock(void);

	void unlock(void);

	void getFriendlyName(std::string *friendlyName);

	void setFriendlyName(const std::string &friendlyName);

	void getSerialNumber(std::string *serialNumber);

	void setSerialNumber(const std::string &serialNumber);

	void getSoftwareVersion(std::string *softwareVersion);

	void setSoftwareVersion(const std::string &softwareVersion);

	bool isPilot(void);

	void setPilot(bool isPilot);

private:
	pthread_mutex_t mMutex;
	std::string mFriendlyName;
	std::string mSerialNumber;
	std::string mSoftwareVersion;
	bool mIsPilot;
};


class SessionPeerMetadata {
public:
	SessionPeerMetadata(void);

	~SessionPeerMetadata(void);

	void lock(void);

	void unlock(void);

	void get(struct vmeta_session *meta);

	void set(const struct vmeta_session *meta);

	enum pdraw_drone_model getDroneModel(void)
	{
		return mDroneModel;
	}

private:
	pthread_mutex_t mMutex;
	struct vmeta_session mMeta;
	enum pdraw_drone_model mDroneModel;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_METADATA_SESSION_HPP_ */
