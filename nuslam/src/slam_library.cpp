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

    colvec cartesian2polar(double x, double y)
    {
        colvec rangeBearing(2);
        rangeBearing(0) = sqrt(pow(x, 2) + (pow(y, 2)));
        rangeBearing(1) = normalize_angle(atan2(y, x));
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
        return;
    }

    ExtendedKalman::ExtendedKalman(colvec robotState, colvec mapState, mat Q, mat R)
    {
        // size = arma::size(robotState) + arma::size(mapState);
        n = mapState.n_elem / 2;
        len = robotState.n_elem + mapState.n_elem;
        seen_landmarks = 0;

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

    void ExtendedKalman::predict(const Twist2D &tw) {
        predictEstimate(tw);
        propagateUncertainty(tw);
        return;
    }

    void ExtendedKalman::predictEstimate(const Twist2D & tw)
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

        state_vector(0) += dq_th;
        state_vector(1) += dq_x;
        state_vector(2) += dq_y;

        return;
    }

    void ExtendedKalman::propagateUncertainty(const Twist2D &tw) {
        mat Q_bar(len, len);
        Q_bar = expanded_process_noise();

        mat A(len, len);
        A = getA(tw);

        mat uncertainty(len, len);
        uncertainty = A * covariance * A.t() + Q_bar;

        covariance = uncertainty;
        return;
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
        double m_xj = state_vec(3+2*(j-1));
        double m_yj = state_vec(4+2*(j-1));
        // std::cout << "m_x,j: " << m_xj << ", m_y,j: " << m_yj << std::endl;
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

        double dx = state_vec(3+2*(j-1)) - state_vec(1);
        // std::cout << "dx: " << dx << "\r" << std::endl;

        double dy = state_vec(4+2*(j-1)) - state_vec(2);
        // std::cout << "dy: " << dy << "\r" << std::endl;

        double d = pow(dx, 2) + pow(dy, 2);
        // std::cout << "d: " << d << "\r" << std::endl;

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

    int ExtendedKalman::associateLandmark(colvec z_i)
    {
        // colvec temp(3+2*(seen_landmarks+1));
        colvec temp(len, fill::zeros);

        // std::cout << "seen landmarks: " << seen_landmarks << std::endl;

        // double min_threshold = 1e-12;
        // double max_threshold = 1e-9;
        double min_threshold = 0.01;
        double max_threshold = 200;
        
        // if no landmarks have been seen, initialize this new landmark
        if (seen_landmarks == 0) {
           seen_landmarks++;
           return seen_landmarks;
        }

        // utilize a temporary state vector with landmark N+1
        temp(span(0, 2+2*seen_landmarks)) = state_vector(span(0, 2+2*seen_landmarks));

        temp(3+2*seen_landmarks) = temp(1) + z_i(0) * cos(z_i(1) + temp(0));
        temp(4+2*seen_landmarks) = temp(2) + z_i(0) * sin(z_i(1) + temp(0));

        for (int k = 1; k < seen_landmarks+1; k++)
        {
            // std::cout << "landmark " << k << "\r" << std::endl;

            // compute the linearized measurement model
            mat H_k = linearizedMeasurementModel(k, temp);
            
            // compute the covariance
            mat psi_k = H_k * covariance * H_k.t() + sensor_noise;
            
            // compute the expected measurement
            vec z_hat = computeTheoreticalMeasurement(k, temp);

            // compute the mahalanobis distance
            double mahalanobis;

            // set mahalanobis distance for landmark N+1 to be min threshold
            if (k == seen_landmarks+1) {
                mahalanobis = min_threshold;
            }
            // calculate mahalanobis distance for all other landmarks
            else {
                // std::cout << "z_i: " << z_i(0) << ", " << z_i(1)  << "\r" << std::endl;
                // std::cout << "z_hat: " << z_hat(0) << ", " << z_hat(1)  << "\r" << std::endl;
                colvec dz = z_i - z_hat;
                // std::cout << "dz: " << dz(0) << ", " << dz(1)  << "\r" << std::endl;

                mat d_k = (z_i - z_hat).t() * psi_k.i() * (z_i - z_hat);
                mahalanobis = d_k(0);
            }
  
            std::cout << "mahalanobis distance from landmark " << k << " is " << mahalanobis  << "\r" << std::endl;

            // if mahalanobis distance is less than some threshold, return the current landmark
            if (mahalanobis < min_threshold) {
                std::cout << "mahalanobis distance less than threshold, returning the current landmark\r" << std::endl;
                return k;
            }
            // if mahalanobis distance is within a "gray" area, return -1 to skip it
            else if ((mahalanobis > min_threshold) && (mahalanobis < max_threshold)) {
                std::cout << "mahalanobis distance within gray area, ignore the measurement\r" << std::endl;
                return -1; // skip the landmark
            }
        }

        std::cout << "mahalanobis distance is outside threshold, initializing new landmark\r" << std::endl;
        // add the new landmark to the list of seen landmarks
        seen_landmarks++;
        return seen_landmarks;
    }

    void ExtendedKalman::initializeLandmark(colvec z_i, int id) {
        std::cout << "initializing landmark " << id << "\r" << std::endl;
        state_vector(3+2*(id-1)) = state_vector(1) + z_i(0)*cos(z_i(1) + state_vector(0));
        state_vector(4+2*(id-1)) = state_vector(2) + z_i(0)*sin(z_i(1) + state_vector(0));

        for (int i = 0; i < len; i++) {
            std::cout << state_vector(i) << "\r" << std::endl;
        }
        return;
    }
    
    void ExtendedKalman::update(const Twist2D &tw, colvec z_id, int id) {
        std::cout << "Update step +++++++++++++++++++++++++++++++++++++++++++++++++\r" << std::endl;

        // compute theoretical measurement given state estimate
        colvec z_hat = computeTheoreticalMeasurement(id, state_vector);

        std::cout << "z_hat distance: " << z_hat(0) << ", angle: " << z_hat(1) << "\r" << std::endl; 

        // compute Kalman gain from linearized measurement model
        mat H_id = linearizedMeasurementModel(id, state_vector);

        for (auto h : H_id) {
            std::cout << "H element:" << h << "\r" << std::endl;
        }

        mat K_id = covariance * H_id.t() * (H_id * covariance * H_id.t() + sensor_noise).i();

        for (auto k : K_id) {
            std::cout << "K element: " << k << "\r" << std::endl;
        }
        // compute posterior state update
        state_vector += K_id * (z_id - z_hat);

        for (auto s : state_vector) {
            std::cout << "state vector element: " << s << "\r" << std::endl;
        }

        // compute posterior covariance update
        covariance = (mat(3+2*n, 3+2*n,arma::fill::eye) - K_id * H_id) * covariance;

        for (auto c : covariance) {
            std::cout << "covariance element: " << c << "\r" << std::endl;
        }
        return;
    }

    const colvec & ExtendedKalman::getStateVector() const
    {
        return state_vector;
    }

    const mat & ExtendedKalman::getCovariance() const
    {
        return covariance;
    }

    const int & ExtendedKalman::getSeenLandmarks() const
    {
        return seen_landmarks;
    }

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