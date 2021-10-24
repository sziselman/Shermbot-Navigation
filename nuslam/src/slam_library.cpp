/// \file slam_library.cpp
/// \brief a library that contains functions for implementing Extended Kalman Filter SLAM

#include "nuslam/slam_library.hpp"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <armadillo>
#include <cmath>
#include <random>

namespace slam_library
{
    using namespace arma;
    using namespace rigid2d;

    colvec xy_to_rangeBearing(double xRel, double yRel)
    {
        colvec rangeBearing(2);
        rangeBearing(0) = sqrt(pow(xRel, 2) + (pow(yRel, 2)));
        rangeBearing(1) = normalize_angle(atan2(yRel, xRel));
        return rangeBearing;
    }

    void ExtendedKalman::initCov()
    {
        covariance = mat(len, len, fill::zeros);
        
        for (int i = 3; i < len; ++i)
        {
            covariance(i, i) = INT_MAX;
        }
        return;
    }

    ExtendedKalman::ExtendedKalman() {

    }

    ExtendedKalman::ExtendedKalman(colvec robotState, colvec mapState, mat Q, mat R)
    {
        // size = arma::size(robotState) + arma::size(mapState);
        n = mapState.n_elem / 2;
        len = robotState.n_elem + mapState.n_elem;
        state_vector = colvec(len);

        process_noise = Q;
        sensor_noise = R;

        // propagate the column vector v (state vector)
        state_vector(0) = robotState(0);
        state_vector(1) = robotState(1);
        state_vector(2) = robotState(2);

        for (int i = 3; i < len; ++i)
        {
            state_vector(i) = mapState(i - 3);
        }

        initCov();
        return;
    }

    colvec ExtendedKalman::predictEstimate(const Twist2D & tw)
    {
        double dq_th, dq_x, dq_y;

        double theta = state_vector(0);

        if (tw.dth == 0.0)
        {
            dq_th = 0.0;
            dq_x = tw.dx * cos(theta);
            dq_y = tw.dx * sin(theta);
        } else
        {
            dq_th = tw.dth;
            dq_x = -(tw.dx / tw.dth) * sin(theta) + (tw.dx / tw.dth) * sin(theta + tw.dth);
            dq_y = (tw.dx / tw.dth) * cos(theta) - (tw.dx / tw.dth) * cos(theta + tw.dth);
        }

        colvec estimate_state(3+2*n);
        estimate_state = state_vector;
        estimate_state(0) += dq_th;
        estimate_state(1) += dq_x; 
        estimate_state(2) += dq_y;

        return estimate_state;
    }

    mat ExtendedKalman::propagateUncertainty(const Twist2D &tw) {
        mat Q_bar(len, len);
        Q_bar = expanded_process_noise();

        mat A(len, len);
        A = getA(tw);

        mat uncertainty(len, len);
        uncertainty = A * covariance * A.t() + Q_bar;

        return uncertainty;
    }

    mat ExtendedKalman::expanded_process_noise(void)
    {
        mat Q_bar(len, len, fill::zeros);

        Q_bar(0, 0) = process_noise(0, 0);
        Q_bar(0, 1) = process_noise(0, 1);
        Q_bar(0, 2) = process_noise(0, 2);
        Q_bar(1, 0) = process_noise(1, 0);
        Q_bar(1, 1) = process_noise(1, 1);
        Q_bar(1, 2) = process_noise(1, 2);
        Q_bar(2, 0) = process_noise(2, 0);
        Q_bar(2, 1) = process_noise(2, 1);
        Q_bar(2, 2) = process_noise(2, 2);

        return Q_bar;
    }

    mat ExtendedKalman::getA(const Twist2D & tw)
    {
        double theta = state_vector(0);

        mat I(len, len, fill::eye);

        mat B(len, len, fill::zeros);

        if (tw.dth == 0)
        {
            B(1, 0) = -tw.dx * sin(theta);
            B(2, 0) = tw.dx * cos(theta);
        } else
        {
            B(1, 0) = -(tw.dx / tw.dth) * cos(theta) + (tw.dx / tw.dth) * cos(theta + tw.dth);
            B(2, 0) = -(tw.dx / tw.dth) * sin(theta) + (tw.dx / tw.dth) * sin(theta + tw.dth);
        }

        mat A(len, len);
        A = I + B;
        return A;
    }

    colvec ExtendedKalman::computeTheoreticalMeasurement(int j, colvec state_vec)
    {
        colvec h_j(2);
        double m_xj = state_vec(1+2*j);
        double m_yj = state_vec(2+2*j);
        double th = state_vec(0);
        double x = state_vec(1);
        double y = state_vec(2);

        h_j(0) = sqrt(pow(m_xj - x, 2) + pow(m_yj - y, 2));
        h_j(1) = normalize_angle(atan2(m_yj - y, m_xj - x) - th);

        return h_j;
    }

    mat ExtendedKalman::linearizedMeasurementModel(int j, colvec state_vec)
    {
        mat H(2, len, fill::zeros);

        double dx = state_vec(1+2*j) - state_vec(1);
        double dy = state_vec(2+2*j) - state_vec(2);
        double d = pow(dx, 2) + pow(dy, 2);

        H(1, 0) = -1;
        H(0, 1) = -dx / sqrt(d);
        H(1, 1) = dy / d;
        H(0, 2) = -dy / sqrt(d);
        H(1, 2) = -dx / d;
        H(0, 3+2*(j-1)) = dx / sqrt(d);
        H(1, 3+2*(j-1)) = -dy / d;
        H(0, 4+2*(j-1)) = dy / sqrt(d);
        H(1, 4+2*(j-1)) = dx / d;
        return H;
    }

    int ExtendedKalman::associateLandmark(vec z_i)
    {
        vec temp(3+2*(seen_landmarks+1));

        double max_threshold = 50;
        double min_threshold = .5;
        
        // if no landmarks have been seen, initialize this new landmark
        if (seen_landmarks == 0) {
            state_vector(3) = state_vector(1) + z_i(0) * cos(z_i(1) + state_vector(0));
            state_vector(4) = state_vector(2) + z_i(0) * sin(z_i(1) + state_vector(0));

           return seen_landmarks++;
        }

        // utilize a temporary state vector with landmark N+1
        temp(span(0, 2+2*seen_landmarks)) = state_vector(span(0, 2+2*seen_landmarks));
        temp(3+2*seen_landmarks) = temp(1) + z_i(0) * cos(z_i(1) + temp(0));
        temp(4+2*seen_landmarks) = temp(2) + z_i(0) * sin(z_i(1) + temp(0));

        for (int k = 1; k < seen_landmarks+1; k++)
        {
            // compute the linearized measurement model
            mat H_k = linearizedMeasurementModel(k, temp);
            
            // compute the covariance
            mat psi_k = H_k * covariance * H_k.t() + sensor_noise;

            // compute the expected measurement
            vec z_hat = computeTheoreticalMeasurement(k, temp);

            // compute the mahalanobis distance
            mat d_k = (z_i - z_hat).t() * psi_k.i() * (z_i - z_hat);
            double mahalanobis = d_k(0);
  
            // if mahalanobis distance is less than some threshold, return the current landmark
            if (mahalanobis < min_threshold) {
                return k;
                
            }
            // if mahalanobis distance is too far but too close to any landmarks, then skip it
            else if ((mahalanobis > min_threshold) && (mahalanobis < max_threshold)) {
                return -1; // skip the landmark
            }
        }

        // add the new landmark to the state vector
        state_vector(3+2*seen_landmarks) = state_vector(1) + z_i(0) * cos(z_i(1) + state_vector(0));
        state_vector(4+2*seen_landmarks) = state_vector(2) + z_i(0) * sin(z_i(1) + state_vector(0));
        return seen_landmarks++;
    }

    // const colvec & ExtendedKalman::getstate_vector() const
    // {
    //     return state_vector;
    // }

    // ExtendedKalman & ExtendedKalman::updatestate_vector(colvec newState)
    // {
    //     state_vector = newState;
    //     return *this;
    // }

    // const mat & ExtendedKalman::getCov() const
    // {
    //     return covariance;
    // }

    // ExtendedKalman & ExtendedKalman::updateCov(mat newCov)
    // {
    //     covariance = newCov;
    //     return *this;
    // }

    // mat ExtendedKalman::KalmanGain(int j)
    // {
    //     mat K_i(len,2);
    //     K_i = covariance * getH(j).t() * (getH(j) * covariance * getH(j).t() + sensor_noise).i();
    //     return K_i;
    // }

    // mat ExtendedKalman::getH2(int j, vec temp)
    // {
    //     mat tempH;

    //     double dx, dy, d;

    //     if (j >= n)
    //     {
    //         dx = temp(1+2*j) - temp(1);
    //         dy = temp(2+2*j) - temp(2);
    //         d = pow(dx, 2) + pow(dy, 2);

    //         tempH = mat(2, 3+2*(n+1), fill::zeros);
    //     } else
    //     {
    //         dx = state_vector(1+2*j) - state_vector(1);
    //         dy = state_vector(2+2*j) - state_vector(2);
    //         d = pow(dx, 2) + pow(dy, 2);

    //         tempH = mat(2, 3+2*n, fill::zeros);
    //     }

    //     tempH(1, 0) = -1;
    //     tempH(0, 1) = -dx / sqrt(d);
    //     tempH(1, 1) = dy / d;
    //     tempH(0, 2) = -dy / sqrt(d);
    //     tempH(1, 2) = -dx / d;
    //     tempH(0, 3+2*j) = dx / sqrt(d);
    //     tempH(1, 3+2*j) = -dy / d;
    //     tempH(0, 4+2*j) = dy / sqrt(d);
    //     tempH(1, 4+2*j) = dx / d;
    //     return tempH;
    // }

}