#include "rigid2d.hpp"
#include <iostream>
#include <cmath>

using namespace std;
using namespace rigid2d;


Transform2D::Transform2D()
{
    double T[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
}

Transform2D::Transform2D(const Vector2D & trans)
{
    double T[3][3] = {{1.0, 0.0, trans.x}, {0.0, 1.0, trans.y}, {0.0, 0.0, 1.0}};
}

Transform2D::Transform2D(double radians)
{
    double T[3][3] = {{cos(radians), sin(radians), 0.0}, {-sin(radians), cos(radians), 0.0}, {0.0, 0.0, 1.0}};
}

Transform2D::Transform2D(const Vector2D & trans, double radians)
{
    double T[3][3] = {{cos(radians), sin(radians), trans.x}, {-sin(radians), cos(radians), trans.y}, {0.0, 0.0, 1.0}};
}

Vector2D Transform2D::operator()(Vector2D v) const
{
    Vector2D vNew;

    // Tr = tMat.T;
    vNew.x = (T[0][0] * v.x) + (T[0][1] * v.y) + T[0][2];
    vNew.y = (T[1][0] * v.x) + (T[1][1] * v.y) + T[1][2];
    return vNew;
}

Transform2D Transform2D::inv() const
{
    Transform2D Tinv;

    double TinvMat[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    Tinv.T = TinvMat;
    return Tinv;
}

int main()
{
    Vector2D v1, v2;
    double rad = 2.0;
    cin >> v1;
    cout << v1;

    Transform2D transformMat;
    // v2 = Transform2D;
}


// Transform2D & Transform2D::operator*=(const Transform2D & rhs)
// {
//     // function to compose this transform with another and store the result in this object
// }

// friend std::ostream & Transform2D::operator<<(std::ostream & os, const Transform2D & tf)
// {
    
// }