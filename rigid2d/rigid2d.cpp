#include "rigid2d.hpp"
#include <iostream>
#include <cmath>

using namespace std;


rigid2d::Transform2D::Transform2D(const rigid2d::Vector2D & trans)
{
    double T[3][3] = {1, 0, trans.x, 0, 1, trans.y, 0, 0, 1};
}

rigid2d::Transform2D::Transform2D(double radians)
{
    double T[3][3] = {cos(radians), sin(radians), 0, sin(radians), cos(radians), 0, 0, 0, 1};
}

rigid2d::Transform2D::Transform2D(const Vector2D & trans, double radians)
{
    double T[3][3] = {cos(radians), sin(radians), trans.x, sin(radians), cos(radians), trans.y, 0, 0, 1};
}


int main()
{
    rigid2d::Vector2D v;
    cin >> v;
    cout << v;
    return 0;
}



// Transform2D Transform2D::inv() const
// {
//     // invert the transformation
// }

// Transform2D & Transform2D::operator*=(const Transform2D & rhs)
// {
//     // function to compose this transform with another and store the result in this object
// }

// friend std::ostream & Transform2D::operator<<(std::ostream & os, const Transform2D & tf)
// {
    
// }