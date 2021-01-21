#include "rigid2d.hpp"
#include <iostream>
#include <cmath>

using namespace std;
using namespace rigid2d;


Transform2D::Transform2D()
{
    T[0][0] = 1.0;
    T[0][1] = 0.0;
    T[0][2] = 0.0;
    T[1][0] = 0.0;
    T[1][1] = 1.0;
    T[1][2] = 0.0;
    T[2][0] = 0.0;
    T[2][1] = 0.0;
    T[2][2] = 1.0;
}

Transform2D::Transform2D(const Vector2D & trans)
{
    T[0][0] = 1.0;
    T[0][1] = 0.0;
    T[0][2] = trans.x;
    T[1][0] = 0.0;
    T[1][1] = 1.0;
    T[1][2] = trans.y;
    T[2][0] = 0.0;
    T[2][1] = 0.0;
    T[2][2] = 1.0;
}

Transform2D::Transform2D(double radians)
{
    T[0][0] = cos(radians);
    T[0][1] = sin(radians);
    T[0][2] = 0.0;
    T[1][0] = -sin(radians);
    T[1][1] = cos(radians);
    T[1][2] = 0.0;
    T[2][0] = 0.0;
    T[2][1] = 0.0;
    T[2][2] = 1.0;
}

Transform2D::Transform2D(const Vector2D & trans, double radians)
{
    T[0][0] = cos(radians);
    T[0][1] = sin(radians);
    T[0][2] = trans.x;
    T[1][0] = -sin(radians);
    T[1][1] = cos(radians);
    T[1][2] = trans.y;
    T[2][0] = 0.0;
    T[2][1] = 0.0;
    T[2][2] = 1.0;
}

Vector2D Transform2D::operator()(Vector2D v) const
{
    Transform2D tMat;
    cout << tMat.T;
    Vector2D vNew;
    return vNew;
}

int main()
{
    Vector2D v1, v2;
    double rad = 2.0;

    cin >> v1;
    cout << v1;

    cout << v2;

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