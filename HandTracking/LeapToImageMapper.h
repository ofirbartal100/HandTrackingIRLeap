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

    cv::Mat_<float> transform_scale;
    cv::Mat_<float> transform_rotate;
    cv::Mat_<float> transform_translate;

public:
    std::vector<cv::Point2f> projections;
    bool isCalibrated;
    bool isRecording;

    LeapToImageMapper(LeapMotion* leap)
    {
        _leap = leap;
        cameraMatrix = cv::Mat(3, 3, CV_64F, calibration_data);// *1.125;
        distCoeffs = cv::Mat(5, 1, CV_64F, distortion_coef_data);
        transform_scale = cv::Mat(2, 3, CV_32F);
        transform_rotate = cv::Mat(2, 3, CV_32F);
        transform_translate = cv::Mat(2, 3, CV_32F);
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
               /* if (c == 0 || c == 2 || c == 4 || c == 5 || c == 8 || c == 9 || c == 12 || c == 13 || c == 16 || c == 17 || c == 20)
                {*/
                    corresponding_points.push_back(r);
                //}
                c++;
            }
            //pinhole camera model maps to image when center is (0,0)
            cv::projectPoints(corresponding_points, rvecs[0], tvecs[0], cameraMatrix, distCoeffs, projections);
            for (int i = 0; i < projections.size(); i++)
            {
                projections[i].x += frameSize.width / 2;
                projections[i].y += frameSize.height / 2;
            }
            return true;
        }
        return false;
    }

    std::vector<cv::Point2f> MapLeapTo2DJoints()
    {
        std::vector<cv::Point3f> res = _leap->GetJoints();
        std::vector<cv::Point2f> joints_projections;
        if (res.size() >= 21)
        {
            //pinhole camera model maps to image when center is (0,0)
            cv::projectPoints(res, rvecs[0], tvecs[0], cameraMatrix, distCoeffs, joints_projections);
            for (int i = 0; i < joints_projections.size(); i++)
            {
                joints_projections[i].x += frameSize.width / 2;
                joints_projections[i].y += frameSize.height / 2;
            }
        }
        return joints_projections;
    }

    void MovePattern(cv::Point2f point)
    {
        for (int i = 0; i < _showed_pattern.size(); i++)
        {
            _showed_pattern[i] += point;
        }
    }

    void TransformPattern(float angle, float scaleX, float scaleY, float translationX, float translationY)
    {
        float upperX = _showed_pattern[0].x;
        float lowerX = _showed_pattern[0].x;
        float upperY = _showed_pattern[0].y;
        float lowerY = _showed_pattern[0].y;

        for (int i = 0; i < _showed_pattern.size(); i++)
        {
            int x = _showed_pattern[i].x;
            int y = _showed_pattern[i].y;

            if (upperX < x)
                upperX = x;
            else if (lowerX > x)
                lowerX = x;

            if (upperY < y)
                upperY = y;
            else if (lowerY > y)
                lowerY = y;
        }
        float centerX = (upperX + lowerX) / 2;
        float centerY = (upperY + lowerY) / 2;

        double pi = acos(-1);
        float alpha = cos(angle*pi / 180);
        float beta = sin(angle*pi / 180);

        transform_scale << scaleX, 0, centerX*(1 - scaleX), 0, scaleY, centerY*(1 - scaleY);
        transform_rotate << alpha, -beta, (1 - alpha)*centerX + beta * centerY, beta, alpha, -beta * centerX + (1 - alpha) * centerY;
        transform_translate << 1, 0, translationX, 0, 1, translationY;

        for (int i = 0; i < _showed_pattern.size(); i++)
        {
            float tempx, tempy;
            cv::Mat_<float> padded_point(3, 1);
            padded_point << _showed_pattern[i].x, _showed_pattern[i].y, 1;
            //scale
            cv::Mat_<float> transformed_point = transform_scale * padded_point;
            padded_point.at<float>(0, 0) = transformed_point.at<float>(0, 0);
            padded_point.at<float>(1, 0) = transformed_point.at<float>(1, 0);

            //rotation
            transformed_point = transform_rotate * padded_point;
            padded_point.at<float>(0, 0) = transformed_point.at<float>(0, 0);
            padded_point.at<float>(1, 0) = transformed_point.at<float>(1, 0);

            //translation
            transformed_point = transform_translate * padded_point;
            _showed_pattern[i].x = transformed_point.at<float>(0, 0);
            _showed_pattern[i].y = transformed_point.at<float>(1, 0);
            //_showed_pattern[i] = transform * cv::Mat(_showed_pattern[i]);
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
                cv::Point2f pinhole_point = image_point_to_pinhole_point(showed_pattern_point);
                _image_points.push_back(pinhole_point);
            }

        }
    }

    cv::Point2f image_point_to_pinhole_point(cv::Point2f point)
    {
        cv::Point2f pinhole_point;
        pinhole_point.x = point.x - frameSize.width / 2;
        pinhole_point.y = point.y - frameSize.height / 2;
        return pinhole_point;
    }

};