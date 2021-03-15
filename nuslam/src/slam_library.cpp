/// \file slam_library.cpp
/// \brief a library that contains functions for implementing Extended Kalman Filter SLAM

#include "nuslam/slam_library.hpp"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <armadillo>
#include <cmath>
#include <random>

std::mt19937 & get_random()
{
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number genrator object. This is always the
     // same object every time get_random is called
     return mt;
 }

namespace slam_library
{
    using namespace arma;
    using namespace rigid2d;

    colvec MultiGauss(const colvec mean, const mat var)
    {
        mat L = chol(var);

        colvec u(size(mean));
        std::normal_distribution<> d(0, 1);
        for (int i = 0; i < mean.n_elem; ++i)
        {
            u(i) = d(get_random());
        }

        colvec v = mean + L*u;
        return v;
    }

    colvec RangeBearing(double xRel, double yRel)
    {
        colvec rangeBearing(2);
        rangeBearing(0) = sqrt(pow(xRel, 2) + (pow(yRel, 2)));
        rangeBearing(1) = normalize_angle(atan2(yRel, xRel));
        return rangeBearing;
    }

    ExtendedKalman::ExtendedKalman(colvec robotState, colvec mapState, mat Q, mat R)
    {
        // size = arma::size(robotState) + arma::size(mapState);
        n = mapState.n_elem / 2;
        len = robotState.n_elem + mapState.n_elem;
        stateVec = colvec(len);

        processNoise = Q;
        sensorNoise = R;

        // propagate the column vector v (state vector)
        stateVec(0) = robotState(0);
        stateVec(1) = robotState(1);
        stateVec(2) = robotState(2);

        for (int i = 3; i < len; ++i)
        {
            stateVec(i) = mapState(i - 3);
        }

        initCov();
    }

    const colvec & ExtendedKalman::getStateVec() const
    {
        return stateVec;
    }

    void ExtendedKalman::initCov()
    {
        cov = mat(len, len, fill::zeros);
        
        for (int i = 3; i < len; ++i)
        {
            cov(i, i) = INT_MAX;
        }
    }

    ExtendedKalman & ExtendedKalman::predict(const Twist2D & tw)
    {
        // update the estimate using the model

        double th = stateVec(0);

        double dq_th, dq_x, dq_y;

        // zero rotational velocity
        if (tw.dth == 0)
        {
            dq_th = 0.0;
            dq_x = tw.dx * cos(th);
            dq_y = tw.dx * sin(th);
        } else  // non-zero rotational velocity
        {
            dq_th = tw.dth; 
            dq_x = -(tw.dx / tw.dth) * sin(th) + (tw.dx / tw.dth) * sin(th + tw.dth);
            dq_y = (tw.dx / tw.dth) * cos(th) - (tw.dx / tw.dth) * cos(th + tw.dth);
        }
        
        stateVec(0) += dq_th; 
        stateVec(1) += dq_x;
        stateVec(2) += dq_y;

        // propagate Q_bar
        mat Q_bar(len, len, fill::zeros);

        Q_bar(span(0, 2), span(0, 2)) = processNoise(span(0, 2), span(0, 2));

        // propagate uncertainty using the linearized state transition model
        mat A(len, len);
        A = getA(th, tw);

        mat covNew = A * cov * A.t() + Q_bar;
        cov = covNew;
        return *this;
    }

    ExtendedKalman & ExtendedKalman::update(int j, colvec z)
    {
        // get matrices H_j, K_j
        mat K_j(len, 2);
        K_j = KalmanGain(j);

        mat H_j(2, len);
        H_j = getH(j);

        // compute theoretical measurement given the current state estimate
        colvec z_hat(2);
        z_hat = h(j);

        // compute the posterior state update
        stateVec += K_j * (z_hat - z);

        // compute the posterior covariance
        mat I(len, len, fill::eye);
        mat covNew(len, len);
        covNew = (I - K_j * H_j) * cov;
        cov = covNew;
        return *this;
    }

    mat ExtendedKalman::getA(double th_prev, const Twist2D & tw)
    {
        mat B(len, len, fill::zeros);

        mat I(len, len, fill::eye);

        if (tw.dth == 0)
        {
            B(1, 0) = -tw.dx * sin(th_prev);
            B(2, 0) = tw.dx * cos(th_prev);
        } else {
            B(1, 0) = -(tw.dx / tw.dth) * cos(th_prev) + (tw.dx / tw.dth) * cos(th_prev + tw.dth);
            B(2, 0) = -(tw.dx / tw.dth) * sin(th_prev) + (tw.dx / tw.dth) * sin(th_prev + tw.dth);
        }
        mat A = I + B;
        return A;
    }

    mat ExtendedKalman::getH(int j)
    {
        mat H(2, len, fill::zeros);

        double dx = stateVec(1+2*j) - stateVec(1);
        double dy = stateVec(2+2*j) - stateVec(2);
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

    colvec ExtendedKalman::h(int j)
    {
        colvec h_j(2);
        double m_xj = stateVec(1+2*j);
        double m_yj = stateVec(2+2*j);
        double th = stateVec(0);
        double x = stateVec(1);
        double y = stateVec(2);

        double dx = m_xj - x;
        double dy = m_yj - y;

        h_j(0) = sqrt(pow(dx, 2) + pow(dy, 2));
        h_j(1) = normalize_angle(atan2(dy, dx) - th);

        return h_j;
    }

    mat ExtendedKalman::KalmanGain(int j)
    {
        mat K_i(len,2);
        K_i = cov * getH(j).t() * (getH(j) * cov * getH(j).t() + sensorNoise).i();
        return K_i;
    }
}