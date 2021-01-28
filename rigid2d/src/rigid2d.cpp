#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include <cmath>
#include <string>

namespace rigid2d 
{
    Vector2D Vector2D::normalize() const
    {
        Vector2D vNew;
        double vecMag = sqrt(pow(x, (double) 2) + pow(x, (double) 2));
        vNew.x = x/vecMag;
        vNew.y = y/vecMag;
        return vNew;
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        os << '[' << v.x << ' ' << v.y << ']' << std::endl;
        return os;
    }

    /// spoke with Nathaniel Nyberg and Arun Kumar for help to understand function
    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
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

    const double& Transform2D::getCosTh() const
    {
        return costh;
    }

    const double& Transform2D::getSinTh() const
    {
        return sinth;
    }

    const double& Transform2D::getX() const
    {
        return x;
    }

    const double& Transform2D::getY() const
    {
        return y;
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
        vNew.x = (v.x * costh) + (v.y * (-sinth)) + x;
        vNew.y = (v.x * sinth) + (v.y * costh) + y;
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
        costh = mat_00;
        sinth = mat_10;
        x = mat_02;
        y = mat_12;
        return *this;
    }

    Transform2D operator *(Transform2D lhs, const Transform2D & rhs)
    {
        return lhs*=rhs;
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

    Twist2D Transform2D::operator()(Twist2D tw) const
    {
        Twist2D twNew;
        twNew.dth = tw.dth;
        twNew.dx = (y * tw.dth) + (costh * tw.dx) - (sinth * tw.dy);
        twNew.dy = -(x * tw.dth) + (sinth * tw.dx) + (costh * tw.dy);
        return twNew;
    }

    std::ostream & operator<<(std::ostream & os, const Twist2D & tw)
    {
        os << "angular velocity: " << tw.dth << " " << "Translational velocity x: " << tw.dx << " " << "Translational velocity y: " << tw.dy << std::endl;
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & tw)
    {
        is >> tw.dth;
        while (is.fail()) {
            is.clear();
            is.ignore();
            is >> tw.dth;
        }

        is >> tw.dx;
        while (is.fail()) {
            is.clear();
            is.ignore();
            is >> tw.dx;
        }

        is >> tw.dy;
        while (is.fail()) {
            is.clear();
            is.ignore();
            is >> tw.dy;
        }
        return is;
    }
}