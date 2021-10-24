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

    /// \brief a function that calculates the range-bearing measurement given (x,y) values
    /// \param x the x distance of the marker relative to the robot
    /// \param y the y distance of the marker relative to the robot
    colvec cartesian2polar(double x, double y);

    /// \brief a class that contains functions when utilizing Extended Kalman Filter
    /// At each time step t, the EKF takes odometry (u) and sensor measurements (z)
    /// to generate estimate of full state vector (zeta)
    class ExtendedKalman
    {
        private:
            colvec state_vector;        // (3+2n)x1 state vector
            mat process_noise;       // 3x3 process noise Q matrix (Q needs to be bigger than R)
            mat sensor_noise;        // 2x2 sensor noise R matrix 
            mat covariance;                // (3+2n)x(3+2n) initial covariance matrix

            int len;                // size of the state vector
            int n;                  // number of landmarks
            int seen_landmarks;     // the number of landmarks visited

            /// \brief initialize the initial covariance matrix
            void initCov();

            /// \brief function that populates Q_bar
            /// \return - a (3+2n)x(3+2n) matrix Q_bar
            mat expanded_process_noise(void);

            /// \brief a function that calculates A matrix
            /// \param tw - the twist / controls
            /// \return - a (3+2n)x(3+2n) matrix representing g'
            mat getA(const Twist2D &tw);

            /// \brief g function that updates the estimate using the model
            /// \param tw - the twist / controls
            void predictEstimate(const Twist2D &tw);

            /// \brief function that propagates the uncertainty
            /// \param prev_state
            /// \param tw
            void propagateUncertainty(const Twist2D &tw);

        public:
            /// \brief constructor for object
            ExtendedKalman();

            /// \brief create a class for using Extended Kalman Filter SLAM
            /// \param robotState - a 3x1 column vector representing the state of the robot
            /// \param mapState - a 2nx1 column representing the state of the map, where n is the number of landmarks
            /// \param Q - a 3x3 matrix representing process noise
            /// \param R - a 2x2 matrix representing sensor noise
            ExtendedKalman(colvec robotState, colvec mapState, mat Q, mat R);

            /// \brief implements prediction step for EKF SLAM
            /// \param tw - the twist
            void predict(const Twist2D &tw);

            /// \brief gets the measurement for range and bearing to landmark j
            /// \param j - the landmark j
            /// \return h_j
            colvec computeTheoreticalMeasurement(int j, colvec state_vec);

            /// \brief gets the matrix H_j
            /// \param j - the landmark j
            /// \param state_vec
            /// \return 2x(3+2n) matrix, the derivative of h_j wrt the state
            mat linearizedMeasurementModel(int j, colvec state_vec);

            /// \brief associate a measurement to a landmark
            /// \param z_i : the range bearing measurement
            /// \return an integer representing the marker id
            int associateLandmark(colvec z_i);

            /// \brief initialize a landmark
            /// \param z_i : the range bearing measurement
            /// \param id : the id of the landmark
            void initializeLandmark(colvec z_id, int id);

            /// \brief implement udpdate step for EKF SLAM
            /// \param tw : the twist
            /// \param z_id : the measurement of landmark id
            /// \param id : the id of the landmark
            void update(const Twist2D &tw, colvec z_id, int id);

            // /// \brief returns the state vector
            // /// \return state_vector
            // const colvec & getstate_vector() const;

            // /// \brief updates the state vector of the EKF object
            // /// \param newState - the new state vector
            // ExtendedKalman & updatestate_vector(colvec newState);

            // /// \brief returns the covariance matrix
            // /// \return covariance
            // const mat & getCov() const;

            // /// \brief updates the covariance matrix of the EKF object
            // /// \param covNew - the new covariance matrix
            // ExtendedKalman & updateCov(mat covNew);

            // /// \brief calculates the Kalman Gain from the linearized measurement model
            // /// \param j - the landmark j
            // /// \return (3+2n)x2 matrix, Kalman gain 
            // mat KalmanGain(int j);

            // /// \brief gets the matrix H_j (to use the temporary state vector instead of estimated)
            // /// spoke with Nathaniel Nyberg to create this function for data association
            // mat getH2(int j, vec temp);
    };
}

#endif