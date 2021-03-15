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

    ExtendedKalman & ExtendedKalman::updateStateVec(colvec newState)
    {
        stateVec = newState;
        return *this;
    }

    const mat & ExtendedKalman::getCov() const
    {
        return cov;
    }

    ExtendedKalman & ExtendedKalman::updateCov(mat newCov)
    {
        cov = newCov;
        return *this;
    }

    void ExtendedKalman::initCov()
    {
        cov = mat(len, len, fill::zeros);
        
        for (int i = 3; i < len; ++i)
        {
            cov(i, i) = INT_MAX;
        }
    }

    colvec ExtendedKalman::g(colvec prevState, const Twist2D & tw)
    {
        double dq_th, dq_x, dq_y;

        double theta = prevState(0);

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

        colvec newState(3+2*n);
        newState = prevState;
        newState(0) += dq_th;
        newState(1) += dq_x; 
        newState(2) += dq_y;

        return newState;
    }

    mat ExtendedKalman::Q_bar()
    {
        mat Q_bar(len, len, fill::zeros);

        Q_bar(0, 0) = processNoise(0, 0);
        Q_bar(0, 1) = processNoise(0, 1);
        Q_bar(0, 2) = processNoise(0, 2);
        Q_bar(1, 0) = processNoise(1, 0);
        Q_bar(1, 1) = processNoise(1, 1);
        Q_bar(1, 2) = processNoise(1, 2);
        Q_bar(2, 0) = processNoise(2, 0);
        Q_bar(2, 1) = processNoise(2, 1);
        Q_bar(2, 2) = processNoise(2, 2);
        return Q_bar;
    }

    mat ExtendedKalman::getA(colvec prevState, const Twist2D & tw)
    {
        double theta = prevState(0);

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
}