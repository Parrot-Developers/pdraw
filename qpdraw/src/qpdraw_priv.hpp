/**
 * Parrot Drones Audio and Video Vector
 * Qt PDrAW object
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

#include <pdraw/pdraw_backend.hpp>
#include <pdraw/qpdraw.hpp>

#include <memory>


using Pdraw::IPdraw;
using PdrawBackend::IPdrawBackend;


namespace QPdraw {
namespace Internal {


class QPdrawPriv : public IPdraw::Listener {

public:
	explicit QPdrawPriv(QPdraw *parent);
	~QPdrawPriv() override = default;

	int start();

	int stop();

	intptr_t getInternal() const;

	struct pomp_loop *getLoop();

private:
	void stopResponse(IPdraw *pdraw, int status) override;

	void onMediaAdded(IPdraw *pdraw,
			  const struct pdraw_media_info *info,
			  void *elementUserData) override;

	void onMediaRemoved(IPdraw *pdraw,
			    const struct pdraw_media_info *info,
			    void *elementUserData) override;

	void onSocketCreated(IPdraw *pdraw, int fd) override;

	QPdraw *mParent = nullptr;
	std::unique_ptr<IPdrawBackend> mPdraw{};
};

} /* namespace Internal */
} /* namespace QPdraw */
