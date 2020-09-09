#ifndef ROTATEGRF_H
#define ROTATEGRF_H

#include <array>
#include <Eigen/Dense>

std::array<double,3> rotate_grf(std::array<double,3> surface_norm, std::array<double,3> grf)
{
    // Receive data and convert to Eigen
    Eigen::Vector3d Zs;
    Zs << 0,0,1;

    Eigen::Vector3d surface_norm_eig;
    surface_norm_eig << surface_norm[0],surface_norm[1],surface_norm[2];

    // Normalize surface normal
    surface_norm_eig.normalize();

    Eigen::Vector3d grf_eig;
    grf_eig << grf[0],grf[1],grf[2];

    // Compute priors
    Eigen::Vector3d v = Zs.cross(surface_norm_eig);
    double s = v.norm();
    double c = Zs.dot(surface_norm_eig);

    Eigen::Matrix3d vskew;
    vskew << 0, -v[2], v[1],
             v[2], 0, -v[0],
             -v[1], v[0], 0;

    Eigen::Matrix3d R; // Rotation matrix to rotate from contact frame to spatial frame
    double eps = 0.000001;
    if (s < eps)
    {
        R = Eigen::Matrix3d::Identity();
    }
    else
    {
        R = Eigen::Matrix3d::Identity() + vskew + vskew*vskew * (1-c)/(s*s);
    }

    Eigen::Vector3d grf_spatial_eig = R*grf_eig;
    std::array<double,3> grf_spatial = {grf_spatial_eig[0],grf_spatial_eig[1],grf_spatial_eig[2]};

    return grf_spatial;
}


#endif