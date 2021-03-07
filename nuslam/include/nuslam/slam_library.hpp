#ifndef SLAM_LIBRARY_INCLUDE_GUARD_HPP
#define SLAM_LIBRARY_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for Simultaneous Localization and Mapping (SLAM) calculations

#include<armadillo>
#include"rigid2d/rigid2d.hpp"
#include"rigid2d/diff_drive.hpp"

namespace slam_library
{
    using namespace arma;
    using namespace rigid2d;

    /// \brief a function that generates multivariate gaussian noise
    /// \param mean 1xn column vector
    /// \param var nxn matrix
    colvec MultiGauss(const colvec mean, const mat var);


    /// \brief a class that contains functions when utilizing Extended Kalman Filter
    /// At each time step t, the EKF takes odometry (u) and sensor measurements (z)
    /// to generate estimate of full state vector (zeta)
    class ExtendedKalman
    {
        private:
            // the number of landmarks
            int n;
            int size = 3+(2*n);
            double x;
            double y;
            double th;

            colvec stateVec;
            colvec processMean;
            mat processNoise;

        public:
            ExtendedKalman(double xx, double yy, double theta, int num);

            /// \brief generates an estimate of the fulls tate vector z_hat
            /// \return estimated state vector z_t = g(z_{t-1}, u_t, w_t)
            colvec getEstimate(const Twist2D & tw);

            /// \brief gets the matrix A_k using the state k-1
            /// \return a (3+2n)x(3+2n) matrix
            mat getA(const Twist2D & tw);

            /// \brief gets the matrix H_k using the state k-1
            /// \return a 2x(3+2n) matrix
            mat getW();

            /// \brief gets the matrix V_k using the state k-1
            /// \return the multivariate Gaussian noise V_k~N(0,R)
            mat getV();
    };
}

#endif