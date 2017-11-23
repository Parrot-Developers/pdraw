/**
 * Parrot Drones Awesome Video Viewer Library
 * Utilities
 *
 * Copyright (c) 2016 Aurelien Barre
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PDRAW_UTILS_HPP_
#define _PDRAW_UTILS_HPP_

#include <inttypes.h>

#include <pdraw/pdraw_defs.h>
#include <Eigen/Eigen>


void pdraw_euler2quat(const struct vmeta_euler *euler, struct vmeta_quaternion *quat);


void pdraw_quat2euler(const struct vmeta_quaternion *quat, struct vmeta_euler *euler);


float pdraw_wrapToPi(float angle);


void pdraw_coordsDistanceAndBearing(double latitude1, double longitude1,
                                    double latitude2, double longitude2,
                                    double *distance, double *bearing);


void pdraw_parseLocationString(char *locationStr, struct vmeta_location *location);


void pdraw_friendlyTimeFromUs(uint64_t time, unsigned int *hrs, unsigned int *min,
                              unsigned int *sec, unsigned int *msec);


int pdraw_videoDimensionsFromH264Sps(uint8_t *pSps, unsigned int spsSize,
    unsigned int *width, unsigned int *height,
    unsigned int *cropLeft, unsigned int *cropRight,
    unsigned int *cropTop, unsigned int *cropBottom,
    unsigned int *sarWidth, unsigned int *sarHeight);

#endif /* !_PDRAW_UTILS_HPP_ */
