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
    // Camera frame: x=right, y=down, z=into scene (optical axis)
    // Body frame (NED): x=forward/N, y=right/E, z=down
    // cam-x (right)  → body-y (East)
    // cam-y (down)   → body-x (North)
    // cam-z (optical)→ body-z (down)
    return cv::Matx33d(0, 1, 0,
                       1, 0, 0,
                       0, 0, 1);
}

std::vector<double> Georeference::pixelToGPS(double u_pixel, double v_pixel, double altitude){ 
//    auto deprojectedMatrix;

    cv::Matx31d pixelMatrix = cv::Matx31d(u_pixel, v_pixel, 1.0); 
   cv::Matx31d deprojectedMatrix = this->A_.inv() * pixelMatrix;
    cv::Matx31d rayNED; 
    rayNED = this->bodyNED * this->cameraToBody() * deprojectedMatrix;

    if (std::abs(rayNED(2,0)) < 1e-6)
        throw std::runtime_error("Ray is horizontal — no ground intersection.");

    double lambda = altitude / rayNED(2,0);
    if (lambda < 0.0)
        throw std::runtime_error("Ray points away from ground.");

    std::vector<double> groundPointNED;
    groundPointNED.push_back(rayNED(0,0) * lambda);
    groundPointNED.push_back(rayNED(1,0) * lambda);
    std::vector<double> GPS = NEDtoGPS(groundPointNED[0], groundPointNED[1], altitude, this->lat0, this->lon0);
    return GPS;
}
