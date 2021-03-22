#ifndef CIRCLE_FIT_LIBRARY_INCLUDE_GUARD_HPP
#define CIRCLE_FIT_LIBRARY_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for circle fit algorithm

#include <armadillo>

namespace circle_fit
{
    using namespace arma;

    /// a class that contains functions for implementing circle fitting algorithm
    class CircleFit
    {
        private:
            double xCenter;
            double yCenter;
            double radius;
        public:
            /// \brief a function used to initialize circle fit object
            /// \param data - a nx2 matrix containing (x,y) location of the data points
            CircleFit(mat data);

            /// \brief accesses the x coordinate of the center of the circle
            const double & getX() const;

            /// \brief accesses the y coordinate of the center of the circle
            const double & getY() const;

            /// \brief accesses the radius of the circle
            const double & getR() const;
    };
}

#endif