#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for differential drive kinematics.

#include<cmath>
#include<iostream>
#include<rigid2d/rigid2d.hpp>

namespace rigid2d
{
    struct wheelVel
    {
        double uL;
        double uR;
    };

    /// \brief This class models the kinematics of a differential drive robot
    /// with a given wheel base and wheel radius
    class DiffDrive
    {
        private:
            double wheelBase;
            double wheelRad;
            double x;
            double y;
            double th;
            double thL;
            double thR;
        public:
            /// \brief create a Differential Drive object
            /// \param wheelBase;
            /// \param wheelRad;
            /// \param x;
            /// \param y;
            /// \param th;
            /// \param thL;
            /// \param thR;
            DiffDrive(double base, double rad, double xx, double yy, double theta, double left, double right);

            /// \brief access the wheel base value
            /// \return wheel base value
            const double& getWheelBase() const;

            /// \brief access the wheel radius value
            /// \return wheel radius value
            const double& getWheelRad() const;

            /// \brief access the x location of the robot configuration
            /// \return x location of robot configuration
            const double& getX() const;

            /// \brief access the y location of the robot configuration
            /// \return y location of robot configuration
            const double& getY() const;

            /// \brief access the angle of the robot configuration
            /// \return angle of robot configuration
            const double& getTh() const;

            /// \brief access the angle of the left wheel
            /// \return angle of the left wheel
            const double& getThL() const;

            /// \brief access the angle of the right wheel
            /// \return angle of the right wheel
            const double& getThR() const;

            /// \brief operator<<(...) declared outside of this class, tracks configuration
            friend std::ostream & operator<<(std::ostream & os, const DiffDrive & dd);

            /// \brief converts a desired twist to equivalent wheel velocities
            /// \param tw - the desired twist (body frame)
            /// \return wheel velocities
            wheelVel convertTwistB(const Twist2D & tw);

            /// \brief converts a desired twist to equivalent wheel velocities
            /// \param tw - the desired twist (world frame)
            /// \return wheel velocities
            wheelVel convertTwistW(const Twist2D & tw);

            /// \brief gets the twist associated with new wheel angles
            /// \param thLnew - the new left wheel angle
            /// \param thRnew - the new right wheel angle
            /// \return the twist
            Twist2D getTwist(double thLnew, double thRnew);

            /// \brief updates the configuration of the robot given updated wheel angles
            /// \param thLnew - the new left wheel angle
            /// \param thRnew - the new right wheel angle
            /// \return the updated configuration
            DiffDrive & operator()(double thLnew, double thRnew);

    };
    
    /// \brief prints a human readable version of the configuration:
    /// An example output: (x, y, th)
    /// \param os - an output stream
    /// \param dd - the DiffDrive configuration to output
    std::ostream & operator<<(std::ostream & os, const DiffDrive & dd);
}
#endif