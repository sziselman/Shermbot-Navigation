#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include <cmath>
#include <string>

namespace rigid2d
{
    DiffDrive::DiffDrive()
    {
        wheelBase = 0;
        wheelRad = 0;
        x = 0;
        y = 0;
        th = 0;
        thL = 0;
        thR = 0;
    }

    DiffDrive::DiffDrive(double base, double rad, double xx, double yy, double theta, double left, double right)
    {
        wheelBase = base;
        wheelRad = rad;
        x = xx;
        y = yy;
        th = theta;
        thL = left;
        thR = right;
    }
    
    const double& DiffDrive::getWheelBase() const
    {
        return wheelBase;
    }

    const double& DiffDrive::getWheelRad() const
    {
        return wheelRad;
    }

    const double& DiffDrive::getX() const
    {
        return x;
    }

    const double& DiffDrive::getY() const
    {
        return y;
    }

    const double& DiffDrive::getTh() const
    {
        return th;
    }

    const double& DiffDrive::getThL() const
    {
        return thL;
    }

    const double& DiffDrive::getThR() const
    {
        return thR;
    }

    wheelVel DiffDrive::convertTwist(const Twist2D & tw)
    {
        wheelVel u;
        double d = wheelBase / 2;
        double r = wheelRad;

        double omg = tw.dth;
        double vbx = (tw.dx * cos(th)) + (tw.dy * sin(th));

        u.uR = (-(d / r) * omg) + (vbx / r);
        u.uL = ((d / r) * omg) + (vbx / r);
        return u;
    }

    Twist2D DiffDrive::getTwist(double thLnew, double thRnew)
    {
        Twist2D twist;
        double delTh = (wheelRad / wheelBase) * ((thRnew - thR) - (thLnew - thL));
        double delX = (wheelRad / 2) * cos(th) * ((thLnew - thL) + (thRnew - thR));
        double delY = (wheelRad / 2) * sin(th) * ((thLnew - thL) + (thRnew - thR));
        twist.dth = delTh;
        twist.dx = delX;
        twist.dy = delY;
        return twist;
    }
    DiffDrive & DiffDrive::operator()(double thLnew, double thRnew)
    {
        double delTh = (wheelRad / wheelBase) * ((thRnew - thR) - (thLnew - thL));
        double delX = (wheelRad / 2) * cos(th) * ((thLnew - thL) + (thRnew - thR));
        double delY = (wheelRad / 2) * sin(th) * ((thLnew - thL) + (thRnew - thR));
        th += delTh;
        th = normalize_angle(th);
        x += delX;
        y += delY;
        thL = thLnew;
        thR = thRnew;
        return *this;
    }

    std::ostream & operator<<(std::ostream & os, const DiffDrive & dd)
    {
        os << '(' << dd.x << ',' << dd.y << ',' << dd.th << ')' << std::endl;
        return os;
    }
}