#include "rigid2d.hpp"
#include <iostream>
using namespace std;

explicit Transform2D::Transform2D(const Vector2D & trans)
{
    // function to create a transformation that is pure translation
}

explicit Transform2D::Transform2D(double radians)
{
    // function to create a transformation that is pure rotation
}

Transform2D::Transform2D(const Vector2D & trans, double radians)
{
    // function to create a transformation with translation and rotation
}

Vector2D Transform2D::operator()(Vector2D v) const
{
    // apply a transformation to a Vector2D
}

Transform2D Transform2D::inv() const
{
    // invert the transformation
}

Transform2D & Transform2D::operator*=(const Transform2D & rhs)
{
    // function to compose this transform with another and store the result in this object
}

friend std::ostream & Transform2D::operator<<(std::ostream & os, const Transform2D & tf)
{
    
}