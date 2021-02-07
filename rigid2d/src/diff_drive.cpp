#include "rigid2d/diff_drive.hpp"
#include <iostream>
#include <cmath>
#include <string>

namespace rigid2d
{
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

    wheelVel DiffDrive::convertTwistB(const Twist2D & tw)
    {
        wheelVel u;
        double d = wheelBase / 2;
        double r = wheelRad;

        u.uR = (tw.dx / r) - ((tw.dy / r) * tan(th)) + (d * tw.dth / r);
        u.uL = u.uR - ((2 * d / r) * tw.dth);
        return u;
    }

    wheelVel DiffDrive::convertTwistW(const Twist2D & tw)
    {
        wheelVel u;
        double d = wheelBase / 2;
        double r = wheelRad;

        u.uR = (tw.dx + (d * tw.dth * cos(th))) / (r * cos(th));
        u.uL = u.uR - ((2 * d / r) * tw.dth);
        return u;
    }

    DiffDrive & DiffDrive::operator()(double thLnew, double thRnew)
    {
        double delTh = (wheelRad / wheelBase) * ((thRnew - thR) - (thLnew - thL));
        double delX = (wheelRad / 2) * cos(th) * ((thLnew - thL) + (thRnew - thR));
        double delY = (wheelRad / 2) * sin(th) * ((thLnew - thL) + (thRnew - thR));
        th += delTh;
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