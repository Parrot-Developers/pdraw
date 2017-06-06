/**
 * @file pdraw_utils.hpp
 * @brief Parrot Drones Awesome Video Viewer Library - utilities
 * @date 05/11/2016
 * @author aurelien.barre@akaaba.net
 *
 * Copyright (c) 2016 Aurelien Barre <aurelien.barre@akaaba.net>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 * 
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 * 
 *   * Neither the name of the copyright holder nor the names of the
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PDRAW_UTILS_HPP_
#define _PDRAW_UTILS_HPP_

#include <inttypes.h>

#include <pdraw/pdraw_defs.h>


#define location_t pdraw_location_t
#define quaternion_t pdraw_quaternion_t
#define euler_t pdraw_euler_t
#define speed_t pdraw_speed_t


void pdraw_quat_conj(const quaternion_t *qSrc, quaternion_t *qDst);


void pdraw_quat_mult(const quaternion_t *qA, const quaternion_t *qB, quaternion_t *qDst);


void pdraw_euler2quat(const euler_t *euler, quaternion_t *quat);


void pdraw_quat2euler(const quaternion_t *quat, euler_t *euler);


void pdraw_coordsDistanceAndBearing(double latitude1, double longitude1,
                                    double latitude2, double longitude2,
                                    double *distance, double *bearing);


void pdraw_parseLocationString(char *locationStr, location_t *location);


void pdraw_friendlyTimeFromUs(uint64_t time, unsigned int *hrs, unsigned int *min,
                              unsigned int *sec, unsigned int *msec);


int pdraw_videoDimensionsFromH264Sps(uint8_t *pSps, unsigned int spsSize,
    unsigned int *width, unsigned int *height,
    unsigned int *cropLeft, unsigned int *cropRight,
    unsigned int *cropTop, unsigned int *cropBottom,
    unsigned int *sarWidth, unsigned int *sarHeight);

#endif /* !_PDRAW_UTILS_HPP_ */
