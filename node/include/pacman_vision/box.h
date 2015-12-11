#ifndef _BOX_H_
#define _BOX_H_

#include <memory>
#include <utility>

//Data structure for box, defined by bounduaries and some basic arithmetics
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
};
#endif

