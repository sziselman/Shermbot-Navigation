#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include <cmath>
#include <string>

namespace rigid2d
{
    DiffDrive::DiffDrive()
    {
        wheelBase = 0.0;
        wheelRad = 0.0;
        x = 0.0;
        y = 0.0;
        th = 0.0;
        thL = 0.0;
        thR = 0.0;
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

        // double omg = tw.dth;
        double omg = tw.dth;
        // double vbx = (tw.dx * cos(th)) + (tw.dy * sin(th));
        double vbx = tw.dx;

        u.uL = (-(d / r) * omg) + (vbx / r);
        u.uR = ((d / r) * omg) + (vbx / r);
        return u;
    }

    Twist2D DiffDrive::getTwist(double thLnew, double thRnew)
    {
        // Find change in wheel angles
        double dUL = thLnew - thL;
        double dUR = thRnew - thR;

        // Calculate the twist Vb
        Twist2D twistb;
        twistb.dth = (wheelRad / wheelBase) * (dUR - dUL);
        twistb.dx = (wheelRad / 2) * (dUL + dUR);
        twistb.dy = 0.0;

        // Integrate twist to get Tbb'
        Transform2D Tbb = integrateTwist(twistb);

        // get displacement in the body frame 
        Twist2D dqb;

        dqb.dth = atan(Tbb.getSinTh() / Tbb.getCosTh());
        // dqb.dth = asin(Tbb.getSinTh());
        dqb.dx = Tbb.getX();
        dqb.dy = Tbb.getY();

        // get adjoint A(theta, 0, 0)
        Transform2D adj = Transform2D(th);

        // Convert twist to desired displacement
        Twist2D dq = adj(dqb);

        return dq;
    }
    DiffDrive & DiffDrive::operator()(double thLnew, double thRnew)
    {
        // Find change in wheel angles
        double dUL = thLnew - thL;
        double dUR = thRnew - thR;

        // Calculate twist Vb
        Twist2D twistb;
        twistb.dth = (wheelRad / wheelBase) * (dUR - dUL);
        twistb.dx = (wheelRad / 2) * (dUL + dUR);
        twistb.dy = 0.0;

        // Integrate twist to get Tbb'
        Transform2D Tbb = integrateTwist(twistb);

        // Get displacement in the body frame
        Twist2D dqb;

        dqb.dth = atan(Tbb.getSinTh() / Tbb.getCosTh());
        // dqb.dth = asin(Tbb.getSinTh());
        dqb.dx = Tbb.getX();
        dqb.dy = Tbb.getY();

        // get adjoint A(theta, 0, 0)
        Transform2D adj = Transform2D(th);

        // Convert twist to desired displacement
        Twist2D dq = adj(dqb);
        
        // Update the configuration of the robot
        th += dq.dth;
        x += dq.dx;
        y += dq.dy;
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