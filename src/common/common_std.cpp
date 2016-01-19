#include <common/common_std.h>
#include <pcl/console/print.h>

namespace pacv
{
double UniformRealIn(const double a, const double b, bool inclusive)
{
    static std::random_device device;
    static std::mt19937_64 engine(device());
    if(inclusive){
        std::uniform_real_distribution<double> dis(a,
                std::nextafter(b, std::numeric_limits<double>::max()));
        return (dis(engine));
    }
    else{
        std::uniform_real_distribution<double> dis(a,b);
        return (dis(engine));
    }
}
int UniformIntIn(const int a, const int b)
{
    static std::random_device device;
    static std::mt19937_64 engine(device());
    std::uniform_int_distribution<int> dis(a,b);
    return (dis(engine));
}
}
