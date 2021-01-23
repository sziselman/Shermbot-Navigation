#include "rigid2d.hpp"
#include <iostream>
#include <cmath>
#include <string>

namespace rigid2d 
{
    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        os << '[' << v.x << ' ' << v.y << ']' << std::endl;
        return os;
    }

    /// spoke with Nathaniel Nyberg and Arun Kumar for help to understand function
    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
        std::cout << "Input a 2 Dimensional Vector:" << std::endl;
        is >> v.x;
        while (is.fail()) {
            is.clear();
            is.ignore();
            is >> v.x;
        }

        is >> v.y;
        while (is.fail()) {
            is.clear();
            is.ignore();
            is >> v.y;
        }
        return is;
    }

    Transform2D::Transform2D()
    {
        costh = 1;
        sinth = 0;
        x = 0;
        y = 0;
    }

    Transform2D::Transform2D(const Vector2D & trans)
    {
        costh = 1;
        sinth = 0;
        x = trans.x;
        y = trans.y;
    }

    Transform2D::Transform2D(double radians)
    {
        costh = cos(radians);
        sinth = sin(radians);
        x = 0;
        y = 0;
    }

    Transform2D::Transform2D(const Vector2D & trans, double radians)
    {
        costh = cos(radians);
        sinth = sin(radians);
        x = trans.x;
        y = trans.y;
    }

    Vector2D Transform2D::operator()(Vector2D v) const
    {
        Vector2D vNew;
        vNew.x = (v.x * costh) + (v.y * (-sinth));
        vNew.y = (v.x * sinth) + (v.y * costh);
        return vNew;
    }

    Transform2D Transform2D::inv() const
    {
        Transform2D Tinv;

        Tinv.costh = costh;
        Tinv.sinth = -sinth;
        Tinv.x = (-x * costh) + (-y * sinth);
        Tinv.y = (x * sinth) + (-y * costh);
        return Tinv;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {
        double mat_00 = (costh * rhs.costh) + (-sinth * rhs.sinth);
        double mat_10 = (sinth * rhs.costh) + (costh * rhs.sinth);
        double mat_02 = (costh * rhs.x) + ((-sinth) * rhs.y) + x;
        double mat_12 = (sinth * rhs.x) + (costh * rhs.y) + y;
        costh = acos(mat_00);
        sinth = asin(mat_10);
        x = mat_02;
        y = mat_12;
        return *this;
    }

    Transform2D operator *(Transform2D lhs, const Transform2D & rhs)
    {
        return lhs*=rhs;
    }

    Twist2D::Twist2D(double ang_vel, double x_vel, double y_vel)
    {
        dth = ang_vel;
        dx = x_vel;
        dy = y_vel;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
    {
        double theta = acos(tf.costh);
        os << "dtheta (degrees): " << theta << " " << "dx: " << tf.x << " " << "dy: " << tf.y << std::endl;
        return os;
    }

        std::istream & operator>>(std::istream & is, Transform2D & tf)
    {
        double rad;
        Vector2D vec;

        std::cout << "Input a Transformation:" << std::endl;
        is >> rad;
        while (is.fail()) {
            is.clear();
            is.ignore();
            is >> rad;
        }

        is >> vec.x;
        while (is.fail()) {
            is.clear();
            is.ignore();
            is >> vec.x;
        }

        is >> vec.y;
        while (is.fail()) {
            is.clear();
            is.ignore();
            is >> vec.y;
        }

        tf = Transform2D(vec, rad);

        return is;
    }

}