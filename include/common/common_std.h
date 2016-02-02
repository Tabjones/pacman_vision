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
#ifndef _COMMON_STD_H_
#define _COMMON_STD_H_
// General Utils
#include <cmath>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <random>
/*! \file common_std.h
    \brief Bunch of global functions and typedefs for std library, used within Pacman Vision
*/

namespace pacv
{
#define D2R M_PI/180  ///< deg to rad conversion
#define R2D 180/M_PI  ///< rad to deg conversion

typedef std::lock_guard<std::mutex> LOCK; ///< Default lock guard type


/**
 * @brief Get an uniformely distributed REAL number in [a, b) if inclusive=false,
 * or in [a, b] if inclusive=true.
 */
double UniformRealIn(const double a, const double b, bool inclusive=false);
/**
 * @brief Get an uniformely distributed INTEGER number in [a, b].
 */
int UniformIntIn(const int a, const int b);
}
#endif

