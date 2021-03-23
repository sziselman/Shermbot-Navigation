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
    using namespace arma;
    
    /// \brief function that takes a vector of clustered points and returns a cylinder marker
    /// \param data - vector of clustered points
    /// \return a cylindrical marker
    visualization_msgs::Marker CircleFit(std::vector<geometry_msgs::Point> data);
}

#endif