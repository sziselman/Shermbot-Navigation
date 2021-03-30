#ifndef CIRCLE_FIT_LIBRARY_INCLUDE_GUARD_HPP
#define CIRCLE_FIT_LIBRARY_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for circle fit algorithm

#include <armadillo>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

namespace circle_fit
{
    /// \brief function that takes a vector of clustered points and returns a cylinder marker
    /// \param data - vector of clustered points
    /// \return a cylindrical marker
    visualization_msgs::Marker CircleFit(std::vector<geometry_msgs::Point> data);

    /// \brief function that clusters points in groups corresponding to individual landmarks
    /// \param minRange - the minimum range that the scanner can detect
    /// \param maxRange - the maximum range that the scanner can detect
    /// \param ranges - the vector of ranges that the lidar scanner detects
    /// \return a vector of "clusters" that contain points for each cluster
    std::vector<std::vector<geometry_msgs::Point>> ClusterPoints(std::vector<float> ranges, double minRange, double maxRange);

    /// \brief a function that clasifies clusters as circle or not circle
    /// \param cluster - the vector of clusters (vectors of points)
    /// \return - boolean value true (if circle) and false (if not circle)
    bool ClassifyCluster(std::vector<geometry_msgs::Point> cluster);
}

#endif