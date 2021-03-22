/// \file circle_fit_library.cpp
/// \brief a library that implements circle fit algorithm

#include "nuslam/circle_fit_library.hpp"
#include <armadillo>

namespace circle_fit
{
    using namespace arma;


    CircleFit::CircleFit(mat data)
    {
        int n = data.n_cols;

        double x_hat = sum(data.col(0)) / n;
        double y_hat = sum(data.col(1)) / n;

        // create the data matrix
        mat Z(4, n, fill::ones);
        
        for (int i; i < n; ++i)
        {
            // shift coordinates so that the centroid is at the origin
            Z(i, 1) = data(i, 0) - x_hat;
            Z(i, 2) = data(i, 1) - y_hat;
            // compute z_i = x_i^2 + y_i^2
            Z(i, 0) = pow(Z(i, 1), 2) + pow(Z(i, 2), 2);
        }

        // compute the mean of z
        double zMean = sum(Z.col(0)) / n;

        // form the moment matrix
        mat M = (Z.t() * Z) / n;

        // form the constraint matrix for the "Hyperaccute algebraic fit"
        mat H(4, 4, fill::eye);
        H(0, 0) = 8 * zMean;
        H(0, 3) = 2;
        H(3, 0) = 2;
        H(3, 3) = 0;

        // compute H^-1
        mat Hinv(4, 4, fill::eye);
        Hinv(0, 0) = 0;
        Hinv(0, 3) = 0.5;
        Hinv(3, 0) = 0.5;
        Hinv(3, 3) = -2 * zMean;

        // compute the Singular Value Decomposition of Z
        mat U;
        vec s;
        mat V;
        svd(U, s, V, Z);

        // determine A based on the smallest singular value sigma4
        vec A;
        if (s(3) < pow(10, -12))
        {
            A = V.col(3);
        } else
        {
            mat Y = V * s * V.t();
            // Find the eigenvalues and vectors of Q
            mat Q = Y * Hinv * Y;

            vec eigval;
            mat eigvec;
            eig_sym(eigval, eigvec, Q);

            // find the eigenvector A* corresponding to the smallest positive eigenvalue of Q
            double eigMax = 1000000;
            int eigInd = 0;
            for (int j = 0; j < int(eigval.n_elem); ++j)
            {
                if ((eigval(j) > 0) && (eigval(j) < eigMax))
                {
                    eigMax = eigval(j);
                    eigInd = j;
                }
            }
            vec A_star = eigvec.col(eigInd);
            A = Y.i() * A_star;
        }

        // find the constants for the equation of the circle
        double a = -A(1) / (2*A(0));
        double b = -A(2) / (2*A(0));
        double R2 = (pow(A(1), 2) + pow(A(2), 2) - 4 * A(0) * A(3)) / (4 * pow(A(0), 2));

        // find center coordinates
        xCenter = a + x_hat;
        yCenter = b + y_hat;
        radius = sqrt(R2);
    }

    const double & CircleFit::getX() const
    {
        return xCenter;
    }

    const double & CircleFit::getY() const
    {
        return yCenter;
    }

    const double & CircleFit::getR() const
    {
        return radius;
    }
}