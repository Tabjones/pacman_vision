#include <pacman_vision/box.h>

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

