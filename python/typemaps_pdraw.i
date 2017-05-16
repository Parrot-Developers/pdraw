/**
 * @file typemaps_pdraw.i
 * @brief Parrot Drones Awesome Video Viewer Library - python bindings types
 * @date 16/05/2017
 * @author louisjoseph.fournier@gmail.com
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
 

/* import numpy array */
%{
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>
%}

%init %{
import_array();
%}

%inline {

/* conversion to numpy array */

PyObject* plane2numpyArray(uint8_t* plane, int w, int h)
{
    int type = NPY_UINT8;
    npy_intp dim[3] = { h, w, 1 };
    PyObject *ret = PyArray_SimpleNewFromData(3, dim, type, plane);
    return ret;
}

}

%newobject pdraw_video_frame_t;

%typemap(newfree) pdraw_video_frame_t * {
    delete $1;
}

%extend Pdraw::IPdraw {
    /*
     * overloaded functions look python string as char*
     */
    int open
            (char* srcAddr,
             char* ifaceAddr,
             int srcStreamPort,
             int srcControlPort,
             int dstStreamPort,
             int dstControlPort,
             int qosMode)
     {

         std::string srcAddr_ = srcAddr;
         std::string ifaceAddr_;

         if(ifaceAddr)
            ifaceAddr_ = ifaceAddr;

         return self->open(
            srcAddr_, ifaceAddr_, srcStreamPort, srcControlPort,
            dstStreamPort, dstControlPort, qosMode);
    }

    pdraw_video_frame_t* getProducerLastFrame(void *producerCtx, int waitUs = 0)
    {
        pdraw_video_frame_t* frame = new pdraw_video_frame_t();
        int ret = self->getProducerLastFrame(producerCtx, frame, waitUs);

        if (ret) {
            delete frame;
            return nullptr;
        } else {
            return frame;
        }
    }
}

%extend pdraw_video_frame_t {
    /* replace plane list with plane() method */
    PyObject* plane(size_t i) {
        int planeNb = self->colorFormat == PDRAW_COLOR_FORMAT_YUV420PLANAR ? 3 : 2;
        if ((int)i >= planeNb)
            return Py_None;

        int w = self->width, h = self->height;
        if (planeNb == 3 && i > 0) {
            w /= 2;
            h /= 2;
        }

        return plane2numpyArray(self->plane[i], w, h);
    }
}
