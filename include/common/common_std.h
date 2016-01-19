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

