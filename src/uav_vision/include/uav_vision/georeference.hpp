#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class Georeference {
    public:
        Georeference(const double fx, const double fy, const double cx, const double cy);  
        ~Georeference() = default;
        void rBodyNED(double yaw, double pitch, double roll);
        cv::Matx33d cameraToBody();
        std::vector<double> pixelToGPS(double u_pixel, double v_pixel, double altitude);     
        static constexpr double kEarthRadius = 6378137.0; 

        // this should be called for each telemetry update 
        void setReferencePoint(double lat, double lon) {
            lat0 = lat;
            lon0 = lon;
        }
    private: 
        double f_x{0};
        double f_y{0}; 
        double c_x{0}; 
        double c_y{0}; 
        double lat0{0};
        double lon0{0};
        cv::Matx33d bodyNED; 
        cv::Matx33d A_{cv::Matx33d::eye()};
};