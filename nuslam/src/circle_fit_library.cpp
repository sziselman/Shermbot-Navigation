/// \file circle_fit_library.cpp
/// \brief a library that implements circle fit algorithm

#include "rigid2d/rigid2d.hpp"
#include "nuslam/circle_fit_library.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

namespace circle_fit
{
    using namespace arma;

    visualization_msgs::Marker circleFit(std::vector<geometry_msgs::Point> data)
    {
        // compute the (x,y) coordinates of the centroid

        double x_hat = 0, y_hat = 0;

        for (auto point : data)
        {
            x_hat += point.x / data.size();
            y_hat += point.y / data.size();
        }
        
        // shift centroid so that the center is at the origin
        for (int i = 0; i < data.size(); i++)
        {
            data[i].x = data[i].x - x_hat;
            data[i].y = data[i].y - y_hat;
        }

        // form the data matrix from the n data points
        double z_bar = 0;

        mat Z(data.size(), 4, fill::ones);
        for (int j = 0; j < data.size(); j++)
        {
            geometry_msgs::Point p = data[j];
            double z = pow(p.x, 2) + pow(p.y, 2);
            z_bar += z/data.size();

            Z(j, 0) = z;
            Z(j, 1) = p.x;
            Z(j, 2) = p.y;
        }
        
        // form the constraint matrix for the "Hyperaccute algebraic fit"
        mat H(4, 4, fill::eye);
        H(0, 0) = 2 * z_bar;
        H(0, 3) = 2;
        H(3, 0) = 2;
        H(3, 3) = 0;

        // compute H^-1
        mat Hinv(4, 4, fill::eye);
        Hinv(0, 0) = 0.0;
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

        if (s.size() < 4) {
            visualization_msgs::Marker marker;
            marker.id = -1;
            return marker;
        }

        if (s(3) < 1e-12) {
            A = V.col(3);
        } 
        else {
            mat Y = V * diagmat(s) * V.t();
            mat Q = Y * Hinv * Y;
            
            vec eigval;
            mat eigvec;

            eig_sym(eigval, eigvec, Q);

            // find the eigenvector corresponding to the smallest positive eigenvalue of Q
            int eig_index = 0;
            double eig_max = INT_MAX;

            for (int i = 0; i < eigval.size(); i++)
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
        double tube_radius = sqrt(R2);

        tf2::Quaternion marker_quat;
        marker_quat.setRPY(0.0, 0.0, 0.0);
        geometry_msgs::Quaternion quat = tf2::toMsg(marker_quat);

        // // create the marker to return
        visualization_msgs::Marker marker;
        marker.ns = "real";
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.pose.position.x = a + x_hat;
        marker.pose.position.y = b + y_hat;
        marker.pose.position.z = 0.25;
        marker.pose.orientation = quat;
        marker.scale.x = 2*tube_radius;
        marker.scale.y = 2*tube_radius;
        marker.scale.z = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 199. / 255.;
        marker.color.g = 234. / 255.;
        marker.color.b = 70. / 255.;
        marker.frame_locked = true;

        return marker;
    }
    
    std::vector<std::vector<geometry_msgs::Point>> clusterPoints(std::vector<float> ranges, double minRange, double maxRange)
    {
        using namespace rigid2d;
        std::vector<std::vector<geometry_msgs::Point>> clusters;

        int curr_angle = 0;
        double threshold = 0.04;

        std::vector<geometry_msgs::Point> current_cluster;

        while (curr_angle < 360)
        {
            // if the point is out of range, then ignore it
            if ((ranges[curr_angle] > maxRange) || (ranges[curr_angle] < minRange))
            {
                curr_angle += 1;
                continue;
            }

            // use small angle approximation to approximate the distance between the current point and the next point
            int next_angle = (curr_angle + 1) % 360;

            double curr_dist = ranges[curr_angle];
            double next_dist = ranges[next_angle];

            geometry_msgs::Point point;
            point.x = ranges[curr_angle] * cos(deg2rad(curr_angle));
            point.y = ranges[curr_angle] * sin(deg2rad(curr_angle));

            // if the distance between the two points is less than the threshold
            if (fabs(curr_dist - next_dist) < threshold)
            {
                // if at current is 359 and next is 0
                // takes care of wrap-around
                if (next_angle < curr_angle)
                {
                    // add the point to the first cluster found
                    clusters[0].push_back(point);
                } else
                {
                    current_cluster.push_back(point);
                    curr_angle += 1;
                }
            } else // if the distance between the points is greater than the threshold
            {
                // add current point to the cluster
                current_cluster.push_back(point);
                
                // add cluster to the vector of clusters
                clusters.push_back(current_cluster);

                // create new cluster
                current_cluster.clear();
                curr_angle += 1;
            }

            if (next_angle < curr_angle){
                break;
            }
        }

        // if cluster has less than 3 points, discard it
        for (int i = 0; i < clusters.size(); i++)
        {
            if (clusters[i].size() < 3)
            {
                clusters.erase(clusters.begin() + i);
            }
        }
        return clusters;
    }

    bool classifyCluster(std::vector<geometry_msgs::Point> cluster)
    {
        geometry_msgs::Point p1, p2, p3;
        p2 = cluster[0];
        p3 = cluster[cluster.size()-1];

        std::vector<double> angles;

        for (int i = 1; i < cluster.size() - 1; i++)
        {
            // calculate the angle from p1 to p to p2
            p1 = cluster[i];
            
            double num = p2.y*(p1.x-p3.x) + p1.y*(p3.x-p2.x) + p3.y*(p2.x-p1.x);
            double den = (p2.x-p1.x)*(p1.x-p3.x) + (p2.y-p1.y)*(p1.y-p3.y);

            double angle = rigid2d::rad2deg(atan2(num, den));

            angles.push_back(angle);
        }

        double mean = 0;
        for (double angle : angles)
        {
            mean += angle / angles.size();
        }

        double std_dev = 0;
        for (double angle : angles)
        {
            std_dev += pow(angle-mean, 2);
        }

        std_dev = sqrt(std_dev/angles.size());

        if (std_dev < 10)
        {
            return true;
        } else 
        {
            return false;
        }
    }
}