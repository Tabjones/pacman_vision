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
#include <common/box.h>
#include <common/common_std.h>

namespace pacv
{
//Box implementations
Box&
Box::operator= (const Box& other)
{
    if (this != &other){
        x1=other.x1;
        x2=other.x2;
        y1=other.y1;
        y2=other.y2;
        z1=other.z1;
        z2=other.z2;
    }
    return *this;
}
Box&
Box::operator= (Box&& other)
{
    x1= std::move(other.x1);
    x2= std::move(other.x2);
    y1= std::move(other.y1);
    y2= std::move(other.y2);
    z1= std::move(other.z1);
    z2= std::move(other.z2);
    return *this;
}
Box
Box::operator* (const float scale) const
{
    double x1s,y1s,z1s;
    double x2s,y2s,z2s;
    x1s = (x2*(1-scale) + x1*(1+scale))*0.5;
    y1s = (y2*(1-scale) + y1*(1+scale))*0.5;
    z1s = (z2*(1-scale) + z1*(1+scale))*0.5;
    x2s = (x2*(1+scale) + x1*(1-scale))*0.5;
    y2s = (y2*(1+scale) + y1*(1-scale))*0.5;
    z2s = (z2*(1+scale) + z1*(1-scale))*0.5;
    return (Box(x1s, y1s, z1s, x2s, y2s, z2s));
}

bool
Box::operator == (const Box &other) const
{
    if (x1 == other.x1 &&
        x2 == other.x2 &&
        y1 == other.y1 &&
        y2 == other.y2 &&
        z1 == other.z1 &&
        z1 == other.z2)
        return true;
    return false;
}

bool
Box::operator != (const Box &other) const
{
    return !(*this == other);
}
}
