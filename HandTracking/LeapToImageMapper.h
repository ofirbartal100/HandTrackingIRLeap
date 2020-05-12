#pragma once
#include <opencv2/opencv.hpp>
#include "LeapMotion.h"


class LeapToImageMapper
{
    LeapMotion* _leap;
    std::vector<cv::Point2f> _showed_pattern;
    std::vector<cv::Point2f> _image_points;
    std::vector<cv::Point3f> _object_points;

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    const cv::Size frameSize = cv::Size(720, 540);
    double reprojection_error;
    double calibration_data[9] = { 6.2601116553800409e+02, 0. ,320. ,0. ,6.2601116553800409e+02 ,240. ,0. ,0.,	1. };
    double distortion_coef_data[5] = { -2.2357192494523123e-01 ,-3.4491331520987045e-01, 0., 0., 1.5648534709863269e+00 };

public:
    std::vector<cv::Point2f> projections;
    bool isCalibrated;

    LeapToImageMapper(LeapMotion* leap)
    {
        _leap = leap;
        cameraMatrix = cv::Mat(3, 3, CV_64F, calibration_data);
        distCoeffs = cv::Mat(5, 1, CV_64F, distortion_coef_data);
    }

    void SetPattern(vector<cv::Point2f> pattern)
    {
        _showed_pattern = pattern;
    }

    vector<cv::Point2f> getPattern()
    {
        return _showed_pattern;
    }

    void Calibrate()
    {
        auto object_points = std::vector<std::vector<cv::Point3f>>();
        object_points.push_back(_object_points);

        auto image_points = std::vector<std::vector<cv::Point2f>>();
        image_points.push_back(_image_points);

        //cameraMatrix = cv::initCameraMatrix2D(object_points, image_points, frameSize, 0);

        std::cout << cameraMatrix << std::endl;
        reprojection_error = cv::calibrateCamera(object_points, image_points, frameSize, cameraMatrix,
            distCoeffs, rvecs, tvecs, cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_ZERO_TANGENT_DIST);

        std::cout << reprojection_error << std::endl;
        //std::cout << distCoeffs << std::endl;

        isCalibrated = true;
    }

    bool Map()
    {
        std::vector<cv::Point3f> res = _leap->GetJoints();
        auto corresponding_points = std::vector<cv::Point3f>();
        int c = 0;

        if (res.size() >= 21)
        {
            //corresponding to the palm and fingertips
            for (auto r : res)
            {
                if (c == 5 || c == 8 || c == 9 || c == 12 || c == 13 || c == 16 || c == 17 || c == 20)
                {
                    corresponding_points.push_back(r);
                }
                c++;
            }
            cv::projectPoints(corresponding_points, rvecs[0], tvecs[0], cameraMatrix, distCoeffs, projections);
            return true;
        }
        return false;
    }

    void MovePattern(cv::Point2f point)
    {
        for (int i=0; i< _showed_pattern.size();i++)
        {
            _showed_pattern[i] += point;
        }
    }

    void RegisterPoints()
    {
        std::vector<cv::Point3f> res = _leap->GetJoints();

        if (res.size() == 21)
        {
            auto corresponding_points = std::vector<cv::Point3f>();
            int c = 0;

            //corresponding to the palm and fingertips
            for (auto r : res)
            {
                if (c == 5 || c == 8 || c == 9 || c == 12 || c == 13 || c == 16 || c == 17 || c == 20)
                {
                    corresponding_points.push_back(r);
                }
                c++;
            }

            for (auto corresponding_point : corresponding_points)
            {
                _object_points.push_back(corresponding_point);
            }
            for (auto showed_pattern_point : _showed_pattern)
            {
                _image_points.push_back(showed_pattern_point);
            }

        }
    }
};