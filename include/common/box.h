// Software License Agreement (BSD License)
//
//   PaCMan Vision (PaCV) - https://github.com/Tabjones/pacman_vision
//   Copyright (c) 2015-2016, Federico Spinelli (fspinelli@gmail.com)
//   All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder(s) nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#ifndef _BOX_H_
#define _BOX_H_

#include <common/common_std.h>

namespace pacv
{
///Data structure for box, defined by bounduaries and some basic arithmetics
struct Box
{
    typedef std::shared_ptr<Box> Ptr;
    double x1,y1,z1;
    double x2,y2,z2;
    //ctors
    Box(){}
    Box(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax)
        : x1(xmin), y1(ymin), z1(zmin), x2(xmax), y2(ymax), z2(zmax) {}
    Box(const Box& other) : x1(other.x1),  y1(other.y1), z1(other.z1),
        x2(other.x2), y2(other.y2), z2(other.z2) {}
    Box(Box&& other) : x1(std::move(other.x1)), y1(std::move(other.y1)), z1(std::move(other.z1)),
        x2(std::move(other.x2)),  y2(std::move(other.y2)), z2(std::move(other.z2)) {}
    //dtor
    ~Box(){}
    Box& operator= (const Box& other);
    Box& operator= (Box&& other);
    Box operator* (const float scale) const;
    bool operator == (const Box &other) const;
    bool operator != (const Box &other) const;
};
}
#endif

