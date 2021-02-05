#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for differential drive kinematics.

#include<cmath>

namespace rigid2d
{
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
        public:
            /// \brief converts a desired twist to the equivalent wheel
            /// velocities required to achieve that twist

            /// \brief operator<<(...) declared outside of this class, tracks configuration
            friend std::ostream & operator<<(std::ostream & os, const DiffDrive & dd);
    }
    
    /// \brief prints a human readable version of the configuration:
    /// An example output: (x, y, th)
    /// \param os - an output stream
    /// \param dd - the DiffDrive configuration to output
    std::ostream & operator<<(std::ostream & os, const DiffDrive & dd);

    /// \brief Read the parameters of a differential drive robot
    /// Should be able to read in the following order:
    /// wheelBase, wheelRad, x, y, th
    std::istream & operator>>(std::istream & is, DiffDrive & dd);
}
#endif