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

    ExtendedKalman::ExtendedKalman(double xx, double yy, double theta, int num)
    {
        n = num;
        x = xx;
        y = yy;
        th = theta;
        stateVec(0) = theta;
        stateVec(1) = xx;
        stateVec(2) = yy;

        processMean(0) = 0;
        processMean(1) = 0;
        processMean(2) = 0;
    }

    colvec ExtendedKalman::getEstimate(const Twist2D & tw)
    {
        colvec estimate;

        colvec vec1(size, fill::zeros);

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

        colvec vec2(size, fill::zeros);
        colvec wt = MultiGauss(processMean, processNoise);

        vec2(0) = wt(0);
        vec2(1) = wt(1);
        vec2(2) = wt(2);

        estimate = stateVec + vec1 + vec2;
        return estimate;
    }

    mat ExtendedKalman::getA(const Twist2D & tw)
    {
        mat B(size, size, fill::zeros);

        mat I(size, size, fill::eye);

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
}