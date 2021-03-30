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

    /// \brief a function that calculates the range-bearing measurement
    /// \param xRel the x distance of the marker relative to the robot
    /// \param yRel the y distance of the marker relative to the robot
    /// \param th the angle of the robot
    colvec RangeBearing(double xRel, double yRel);

    /// \brief a class that contains functions when utilizing Extended Kalman Filter
    /// At each time step t, the EKF takes odometry (u) and sensor measurements (z)
    /// to generate estimate of full state vector (zeta)
    class ExtendedKalman
    {
        private:
            colvec stateVec;        // (3+2n)x1 state vector
            mat processNoise;       // 3x3 process noise Q matrix (Q needs to be bigger than R)
            mat sensorNoise;        // 2x2 sensor noise R matrix 
            mat cov;                // (3+2n)x(3+2n) initial covariance matrix

            int len;                // size of the state vector
            int n;                  // number of landmarks
            int N = 0;              // the number of landmarks visited

            /// \brief initialize the initial covariance matrix
            /// \param num - the number of landmarks
            /// \return (3+2n)x(3_2n) covariance matrix
            void initCov();

        public:
            /// \brief create a class for using Extended Kalman Filter SLAM
            /// \param robotState - a 3x1 column vector representing the state of the robot
            /// \param mapState - a 2nx1 column representing the state of the map, where n is the number of landmarks
            /// \param Q - a 3x3 matrix representing process noise
            /// \param R - a 2x2 matrix representing sensor noise
            ExtendedKalman(colvec robotState, colvec mapState, mat Q, mat R);

            /// \brief returns the state vector
            /// \return stateVec
            const colvec & getStateVec() const;

            /// \brief updates the state vector of the EKF object
            /// \param newState - the new state vector
            ExtendedKalman & updateStateVec(colvec newState);

            /// \brief returns the covariance matrix
            /// \return cov
            const mat & getCov() const;

            /// \brief updates the covariance matrix of the EKF object
            /// \param covNew - the new covariance matrix
            ExtendedKalman & updateCov(mat covNew);

            /// \brief g function that updates the estimate using the model
            /// \param prevState - a (3+2n)x1 column vector representing the state of the robot
            /// \param tw - the twist / controls
            /// \return - a (3+2n)x1 column vector representing the change of state of the robot
            colvec g(colvec prevState, const Twist2D & tw);

            /// \brief function that populates Q_bar
            /// \param Q - a 3x3 matrix
            /// \return - a (3+2n)x(3+2n) matrix Q_bar
            mat Q_bar();

            /// \brief a function that calculates A matrix
            /// \param prevState - a (3+2n)x1 column vector representing the state of the robot
            /// \param tw - the twist / controls
            /// \return - a (3+2n)x(3+2n) matrix representing g'
            mat getA(colvec prevState, const Twist2D & tw);

            /// \brief gets the measurement for range and bearing to landmark j
            /// \param j - the landmark j
            /// \return h_j
            colvec h(int j);

            /// \brief calculates the Kalman Gain from the linearized measurement model
            /// \param j - the landmark j
            /// \return (3+2n)x2 matrix, Kalman gain 
            mat KalmanGain(int j);

            /// \brief gets the matrix H_j
            /// \param j - the landmark j
            /// \return 2x(3+2n) matrix, the derivative of h_j wrt the state
            mat getH(int j);

            /// \brief gets the matrix H_j (to use the temporary state vector instead of estimated)
            /// spoke with Nathaniel Nyberg to create this function for data association
            mat getH2(int j, vec temp);

            /// \brief function used for data association
            /// \param z_i : the range bearing measurement
            /// \return an integer representing the marker id
            int DataAssociation(vec z_i);
    };
}

#endif