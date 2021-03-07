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
    /// \param mean nx1 column vector
    /// \param var nxn matrix
    colvec MultiGauss(const colvec mean, const mat var);


    /// \brief a class that contains functions when utilizing Extended Kalman Filter
    /// At each time step t, the EKF takes odometry (u) and sensor measurements (z)
    /// to generate estimate of full state vector (zeta)
    class ExtendedKalman
    {
        private:
            colvec stateVec;        // (3+2n)x1 state vector
            mat processNoise;       // 3x3 process noise Q matrix
            mat sensorNoise;        // 2x2 sensor noise R matrix
            mat cov;                // (3+2n)x(3+2n) initial covariance matrix

            int len;                // size of the state vector
            int n;                  // number of landmarks

            /// \brief initialize the initial covariance matrix
            /// \param num - the number of landmarks
            /// \return (3+2n)x(3_2n) covariance matrix
            void initCov(int num);

        public:
            /// \brief create a class for using Extended Kalman Filter SLAM
            /// \param robotState - a 3x1 column vector representing the state of the robot
            /// \param mapState - a 2nx1 column representing the state of the map, where n is the number of landmarks
            /// \param Q - a 3x3 matrix representing process noise
            /// \param R - a 2x2 matrix representing sensor noise
            ExtendedKalman(colvec robotState, colvec mapState, mat Q, mat R);

            /// \brief generates an estimate of the fulls tate vector z_hat
            /// \param tw - the twist command
            /// \return estimated state vector z_t = g(z_{t-1}, u_t, w_t)
            void predict(const Twist2D & tw);

            /// \brief updates the stateVector everytime a new landmark is encountered
            /// \param j - the landmark j
            /// \param z - 2x1 column vector containing range-bearing measurements
            void update(int j, colvec z);

            /// \brief gets the matrix A_k using the state k-1
            /// \param tw - the twist command
            /// \return a (3+2n)x(3+2n) matrix
            mat getA(const Twist2D & tw);

            /// \brief gets the matrix H_k using the state k-1
            /// \return a 2x(3+2n) matrix
            mat getW();

            /// \brief gets the matrix V_k using the state k-1
            /// \return the multivariate Gaussian noise V_k~N(0,R)
            mat getV();

            /// \brief gets the matrix H_j
            /// \param j - the landmark j
            /// \return 2x(3+2n) matrix, the derivative of h_j wrt the state
            mat getH(int j);

            /// \brief gets the measurement for range and bearing to landmark j
            /// \param j - the landmark j
            /// \return h_j
            colvec h(int j);

            /// \brief calculates the Kalman Gain from the linearized measurement model
            /// \param j - the landmark j
            /// \return (3+2n)x2 matrix, Kalman gain 
            mat KalmanGain(int j);

            colvec getSens();
    };
}

#endif