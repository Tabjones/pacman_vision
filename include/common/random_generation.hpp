#ifndef _RANDOM_GENERATION_HPP_
#define _RANDOM_GENERATION_HPP_

#include <random>
#include <cmath>

///Simple functions to quickly get uniformely distributed numbers in given
//intervals
namespace pacv
{
    std::random_device device_;
    std::mt19937_64 engine_(device_());
    /**
     * @brief Get an uniformely distributed REAL number in [a, b) if inclusive=false,
     * or in [a, b] if inclusive=true.
     */
    double UniformRealIn(const double a, const double b, bool inclusive=false)
    {
        if(inclusive){
            std::uniform_real_distribution<double> dis(a,
                    std::nextafter(b, std::numeric_limits<double>::max()));
            return (dis(engine_));
        }
        else{
            std::uniform_real_distribution<double> dis(a,b);
            return (dis(engine_));
        }
    }
    /**
     * @brief Get an uniformely distributed INTEGER number in [a, b].
     */
    int UniformIntIn(const int a, const int b)
    {
        std::uniform_int_distribution<int> dis(a,b);
        return (dis(engine_));
    }
}
#endif
