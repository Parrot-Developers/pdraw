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


typedef struct
{
    bool isValid;
    double latitude;
    double longitude;
    double altitude;
    uint8_t svCount;

} location_t;


typedef struct
{
    float w;
    float x;
    float y;
    float z;

} quaternion_t;


typedef struct
{
    float phi;      // roll
    float theta;    // pitch
    float psi;      // yaw

} euler_t;


typedef struct
{
    float north;
    float east;
    float down;

} speed_t;


void pdraw_euler2quat(const euler_t *euler, quaternion_t *quat);


void pdraw_quat2euler(const quaternion_t *quat, euler_t *euler);


void pdraw_coordsDistanceAndBearing(double latitude1, double longitude1,
                                    double latitude2, double longitude2,
                                    double *distance, double *bearing);


#endif /* !_PDRAW_UTILS_HPP_ */
