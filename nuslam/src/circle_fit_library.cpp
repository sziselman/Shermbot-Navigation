/// \file circle_fit_library.cpp
/// \brief a library that implements circle fit algorithm

#include "nuslam/circle_fit_library.hpp"
#include <cmath>

namespace circle_fit
{
    using namespace arma;

    visualization_msgs::Marker CircleFit(std::vector<geometry_msgs::Point> data)
    {
        // compute the (x,y) coordinates of the centroid

        double x_hat = 0, y_hat = 0;
        int n = data.size();

        for (auto point : data)
        {
            x_hat += point.x / n;
            y_hat += point.y / n;
        }
        
        // shift centroid so that the center is at the origin
        for (int i = 0; i < n; ++i)
        {
            data[i].x -= x_hat;
            data[i].y -= y_hat;
        }

        // form the data matrix from the n data points
        mat Z(n, 4, fill::ones);
        for (int j = 0; j < n; ++j)
        {
            Z(j, 0) = pow(data[j].x, 2) + pow(data[j].y, 2);
            Z(j, 1) = data[j].x;
            Z(j, 2) = data[j].y;
        }

        // compute the mean of z
        double z_bar = sum(Z.col(0)) / n;

        // form the constraint matrix for the "Hyperaccute algebraic fit"
        mat H(4, 4, fill::eye);
        H(0, 0) = 2 * z_bar;
        H(0, 3) = 2;
        H(3, 0) = 2;
        H(3, 3) = 0;

        // compute H^-1
        mat Hinv(4, 4, fill::eye);
        Hinv(0, 0) = 0;
        Hinv(0, 3) = 0.5;
        Hinv(3, 0) = 0.5;
        Hinv(3, 3) = -2 * z_bar;

        // compute the Singular Value Decomposition of Z
        mat U;
        vec s;
        mat V;
        svd(U, s, V, Z);

        // solve for A based on sigma_4
        vec A;

        if (s(3) < 1e-12)
        {
            A = V.col(3);
        } else
        {
            mat Y = V * diagmat(s) * V.t();
            mat Q = Y * Hinv * Y;
            vec eigval;
            mat eigvec;
            eig_sym(eigval, eigvec, Q);

            // find the eigenvector corresponding to the smallest positive eigenvalue of Q
            int eig_index = 0;
            double eig_max = INT_MAX;

            for (int i = 0; i < eigval.size(); ++i)
            {
                if (eigval(i) > 0 && eigval(i) < eig_max)
                {
                    eig_index = i;
                    eig_max = eigval(i);
                }
            }
            vec Astar = eigvec.col(eig_index);
            A = solve(Y, Astar);
        }

        // solve for a, b, and R^2 (equation of the circle)
        double a = -A(1) / (2*A(0));
        double b = -A(2) / (2*A(0));
        double R2 = (pow(A(1),2) + pow(A(2), 2) - 4*A(0)*A(3)) / (4*pow(A(0),2));

        // create the marker to return
        visualization_msgs::Marker marker;
        marker.ns = "real";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.pose.position.x = a + x_hat;
        marker.pose.position.y = b + y_hat;
        marker.scale.x = sqrt(R2);
        marker.scale.y = sqrt(R2);
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.frame_locked = true;

        return marker;
    }
    

}