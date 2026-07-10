#include "uav_vision/georeference.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


std::vector<double> NEDtoGPS(double north, double east, double altitude, double lat0, double lon0)
{
    std::vector<double> gpsPoint; 
    gpsPoint.push_back(lat0 + (north / Georeference::kEarthRadius) * (180.0 / M_PI));
    gpsPoint.push_back(lon0 + (east / (Georeference::kEarthRadius * cos(M_PI * lat0 / 180.0))) * (180.0 / M_PI));
    gpsPoint.push_back(altitude);
    return gpsPoint; 
}

Georeference::Georeference(const double fx, const double fy, const double cx, const double cy)
            : f_x(fx), f_y(fy), c_x(cx), c_y(cy) 
{
    this->A_ = cv::Matx33d(f_x, 0, c_x, 0, f_y, c_y, 0, 0, 1);
}
void Georeference::rBodyNED(double yaw, double pitch, double roll)
{
    Eigen::Matrix3d R = (Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ())
                       * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                       * Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX())).matrix();
    cv::Mat tmp;
    cv::eigen2cv(R, tmp);
    this->bodyNED = cv::Matx33d(tmp); 
}

cv::Matx33d Georeference::cameraToBody()
{
    // SDF pose: pitch=+π/2 → optical axis (+X_link) → body +Z (down in NED) ✓
    // Image right (+u) → body +Y (East): col 0 = (0, +1, 0)
    // Image down  (+v) → body -X (South): col 1 = (-1, 0, 0)
    // Depth       (+z) → body +Z (Down):  col 2 = (0, 0, 1)
    return cv::Matx33d(0, -1, 0,
                       1,  0, 0,
                       0,  0, 1);
}

std::vector<double> Georeference::pixelToGPS(double u_pixel, double v_pixel, double altitude, double* gus_out)
{
    cv::Matx31d pixelMatrix = cv::Matx31d(u_pixel, v_pixel, 1.0);
    cv::Matx31d deprojectedMatrix = this->A_.inv() * pixelMatrix;
    cv::Matx31d rayNED = this->bodyNED * this->cameraToBody() * deprojectedMatrix;

    if (std::abs(rayNED(2,0)) < 1e-6)
        throw std::runtime_error("Ray is horizontal, no ground intersection.");

    cv::Matx31d cam_offset_body(0.1, 0.0, 0.05);
    cv::Matx31d cam_offset_world = this->bodyNED * cam_offset_body;

    double camera_alt = altitude - cam_offset_world(2, 0);

    double lambda = camera_alt / rayNED(2, 0);
    if (lambda < 0.0)
        throw std::runtime_error("Ray points away from ground.");

    if (gus_out) {
        double ray_norm = std::sqrt(rayNED(0,0)*rayNED(0,0) + rayNED(1,0)*rayNED(1,0) + rayNED(2,0)*rayNED(2,0));
        double cos_tilt = rayNED(2, 0) / ray_norm;
        *gus_out = camera_alt / (cos_tilt * cos_tilt);
    }

    double north = cam_offset_world(0, 0) + rayNED(0, 0) * lambda;
    double east  = cam_offset_world(1, 0) + rayNED(1, 0) * lambda;

    return NEDtoGPS(north, east, altitude, this->lat0, this->lon0);
}
