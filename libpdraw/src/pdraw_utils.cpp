/**
 * @file pdraw_utils.cpp
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

#include <math.h>
#include <string.h>

#include "pdraw_utils.hpp"

#define ULOG_TAG libpdraw
#include <ulog.h>


void pdraw_euler2quat(const euler_t *euler, quaternion_t *quat)
{
    if ((!euler) || (!quat))
        return;

    float c1, c2, c3, s1, s2, s3, psi, theta, phi;
    float qw, qx, qy, qz, n;
    phi = euler->phi;
    theta = euler->theta;
    psi = euler->psi;
    c1 = cosf(phi / 2);
    s1 = sinf(phi / 2);
    c2 = cosf(theta / 2);
    s2 = sinf(theta / 2);
    c3 = cosf(psi / 2);
    s3 = sinf(psi / 2);
    qw = c1 * c2 * c3 + s1 * s2 * s3;
    qx = s1 * c2 * c3 - c1 * s2 * s3;
    qy = c1 * s2 * c3 + s1 * c2 * s3;
    qz = c1 * c2 * s3 - s1 * s2 * c3;
    n = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if (n != 0.)
    {
        qw /= n;
        qx /= n;
        qy /= n;
        qz /= n;
    }

    quat->w = qw;
    quat->x = qx;
    quat->y = qy;
    quat->z = qz;
}


void pdraw_quat2euler(const quaternion_t *quat, euler_t *euler)
{
    if ((!quat) || (!euler))
        return;

    float psi, theta, phi;
    float qw, qx, qy, qz;
    qw = quat->w;
    qx = quat->x;
    qy = quat->y;
    qz = quat->z;
    phi = atan2f(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
    theta = asinf(2 * (qw * qy - qz * qx));
    psi = atan2f(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

    euler->phi = phi;
    euler->theta = theta;
    euler->psi = psi;
}


void pdraw_coordsDistanceAndBearing(double latitude1, double longitude1,
                                    double latitude2, double longitude2,
                                    double *distance, double *bearing)
{
    /* http://www.igismap.com/haversine-formula-calculate-geographic-distance-earth/ */
    /* http://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/ */
    double a, c, d, x, y, b;
    double r = 6371000.; // earth radius
    double lat1 = latitude1 * M_PI / 180.;
    double lon1 = longitude1 * M_PI / 180.;
    double lat2 = latitude2 * M_PI / 180.;
    double lon2 = longitude2 * M_PI / 180.;
    a = sin((lat2 - lat1) / 2) * sin((lat2 - lat1) / 2)
        + cos(lat1) * cos(lat2) * sin((lon2 - lon1) / 2) * sin((lon2 - lon1) / 2);
    c = 2 * atan2(sqrt(a), sqrt(1. - a));
    d = r * c;
    x = cos(lat2) * sin(lon2 - lon1);
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1);
    b = atan2(x, y);
    if (distance) *distance = d;
    if (bearing) *bearing = b;
}
