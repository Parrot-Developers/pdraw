/**
 * Parrot Drones Awesome Video Viewer
 * Desktop application
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

#include <Eigen/Eigen>

#include "pdraw_desktop.h"


void pdraw_desktop_view_create_matrices(struct pdraw_desktop *self,
					float *view_mat,
					float *proj_mat,
					float near,
					float far)
{
	float w = 1.f;
	float h = (float)self->window_width / (float)self->window_height;
	float a = (far + near) / (far - near);
	float b = -((2 * far * near) / (far - near));

	Eigen::Matrix4f p_mat;
	p_mat << w, 0, 0, 0, 0, h, 0, 0, 0, 0, a, b, 0, 0, 1, 0;

	Eigen::Matrix4f v_mat = Eigen::Matrix4f::Identity();

	unsigned int i, j;
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++)
			view_mat[j * 4 + i] = v_mat(i, j);
	}
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++)
			proj_mat[j * 4 + i] = p_mat(i, j);
	}
}
