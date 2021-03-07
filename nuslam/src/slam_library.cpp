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
    }

    void ExtendedKalman::initCov(int num)
    {
        cov = mat(3+2*num, 3+2*num, fill::zeros);
        
        for (int i = 3; i < 3+2*num; ++i)
        {
            cov(i, i) = INT_MAX;
        }
    }

    void ExtendedKalman::predict(const Twist2D & tw)
    {
        // update the estimate using the model

        double th = stateVec(0);
        
        colvec vec1(len, fill::zeros);

        if (tw.dth == 0)
        {
            vec1(0) = 0;
            vec1(1) = tw.dx * cos(th);
            vec1(2) = tw.dx * sin(th);
        } else
        {
            vec1(0) = tw.dth;
            vec1(1) = -(tw.dx/tw.dth)*sin(th) + (tw.dx/tw.dth)*sin(th+tw.dth);
            vec1(2) = (tw.dx/tw.dth)*cos(th) - (tw.dx/tw.dth)*cos(th+tw.dth);
        }
        stateVec += vec1;

        // propagate Q_bar
        mat Q_bar(len, len, fill::zeros);

        Q_bar(span(0, 2), span(0, 2)) = processNoise(span(0, 2), span(0, 2));

        // propagate uncertainty using the linearized state transition model
        mat covNew = getA(tw)*cov*getA(tw).t() + Q_bar;
        cov = covNew;
    }

    void ExtendedKalman::update(int j, colvec z)
    {
        stateVec += KalmanGain(j) * (h(j) - z);

        mat I(len, len, fill::eye);
        mat covNew(len, len);
        covNew = (I - KalmanGain(j)*getH(j))*cov;
        cov = covNew;
    }

    mat ExtendedKalman::getA(const Twist2D & tw)
    {
        mat B(len, len, fill::zeros);

        mat I(len, len, fill::eye);

        double th = stateVec(0);

        if (tw.dth == 0)
        {
            B(0, 1) = -tw.dx * sin(th);
            B(0, 2) = tw.dx * cos(th);
        } else {
            B(0, 1) = -(tw.dx/tw.dth)*cos(th) + (tw.dx/tw.dth)*cos(th + tw.dth);
            B(0, 2) = -(tw.dx/tw.dth)*sin(th) + (tw.dx/tw.dth)*sin(th + tw.dth);
        }
        mat A = I + B;
        return A;
    }

    mat ExtendedKalman::getH(int j)
    {
        mat H(2, len, fill::zeros);

        double dx = stateVec(3+2*j) - stateVec(1);
        double dy = stateVec(4+2*j) - stateVec(2);
        double d = pow(dx, 2) + pow(dy, 2);

        H(0, 1) = -1;
        H(1, 0) = -dx / sqrt(d);
        H(1, 1) = dy / d;
        H(2, 0) = -dy / sqrt(d);
        H(2, 1) = -dx / d;
        H(0, 3+2*(j-1)) = dx / sqrt(d);
        H(1, 3+2*(j-1)) = -dy / d;
        H(0, 4+2*(j-1)) = dy / sqrt(d);
        H(1, 4+2*(j-1)) = dx / d;
        return H;
    }

    colvec ExtendedKalman::h(int j)
    {
        colvec h_j(2);
        double m_xj = stateVec(3+2*j);
        double m_yj = stateVec(4+2*j);
        double th = stateVec(0);
        double x = stateVec(1);
        double y = stateVec(2);

        h_j(0) = sqrt(pow(m_xj - x, 2) + (pow(m_yj - y, 2)));
        h_j(1) = atan2(m_yj - y, m_xj - x) - th;

        return h_j;
    }

    mat ExtendedKalman::KalmanGain(int j)
    {
        mat K_i = cov * getH(j).t() * (getH(j) * cov * getH(j).t() + sensorNoise).i();
        return K_i;
    }
}