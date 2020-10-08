#include <filesystem>
#include "opencv2/core/cvstd_wrapper.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <opencv2/objdetect.hpp>
#include <ctime>
#include <ppl.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>

#include "BaslerCamera.h"
using namespace std;

//#define CALIBRATION

int main(int argc, char **argv)
{

    cv::Mat frame;
    // Set the hook
    cv::VideoCapture basler_camera(0);
    /*BaslerCamera basler_camera;
    basler_camera.StartGrabbing();
    frame = basler_camera.RetrieveFrame();*/

    // display settings
    int primDispW = 1920;
    int primDispH = 1200;
    int projectorW = 1280;
    int projectorH = 720;

#ifdef CALIBRATION
    cv::Mat chessPattern = cv::Mat::zeros(cv::Size(projectorW, projectorH), CV_8UC3);
    cv::Mat capture;

    //draw chess pattern
    int chessSize = 100;
    int chessNumX = projectorW / chessSize;
    int chessNumY = projectorH / chessSize;
    //chessNumX-1 must be odd
    if (chessNumX % 2 == 1) chessNumX--;
    //chessNumY-1 must be even
    if (chessNumY % 2 == 0) chessNumY--;
    //white background
    cv::rectangle(chessPattern, cv::Rect(0, 0, projectorW, projectorH), cv::Scalar(255, 255, 255), -1);
    //draw black chess boxes
    int offsetX = (projectorW - chessNumX * chessSize) / 2;
    int offsetY = (projectorH - chessNumY * chessSize) / 2;
    std::vector<cv::Point2f> projectPoints;
    for (int y = 0; y < chessNumY; y++) {
        for (int x = 0; x < chessNumX; x++) {
            int topleftX = offsetX + x * chessSize;

            int topleftY = offsetY + y * chessSize;
            if ((x + y) % 2 == 0) {
                cv::rectangle(chessPattern, cv::Rect(topleftX, topleftY, chessSize, chessSize), cv::Scalar(0, 0, 0), -1);
            }
            if (x&&y) {
                projectPoints.push_back(cv::Point2f(topleftX, topleftY));
            }
        }
    }

    // set fullscreen window
    cv::namedWindow("chess_pattern_window", cv::WINDOW_NORMAL);
    cv::setWindowProperty("chess_pattern_window", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    //project chess pattern and capture it
    while (1) {
        int c = cv::waitKey(1);
        if (c == 27) {
            //capture
            //capture = cv::imread("chess_capt.png", 1);	// Ofir, please change here to your camera capture code
            //capture = basler_camera.RetrieveFrame();	// Ofir, please change here to your camera capture code
            basler_camera.read(capture);	// Ofir, please change here to your camera capture code
            break;
        }
        //project chess pattern
        cv::imshow("chess_pattern_window", chessPattern);
        //cv::moveWindow("chess_pattern_window", primDispW, 0);
    }

    //decode chess board
    std::vector<cv::Point2f> imagePoints;
    //find chess corners
    auto found = cv::findChessboardCorners(capture, cv::Size2i(chessNumX - 1, chessNumY - 1), imagePoints);
    if (found)
    {
        //refine corners
        cv::Mat capt_gray = cv::Mat(capture.size(), CV_8UC1);
        cv::cvtColor(capture, capt_gray, cv::COLOR_BGR2GRAY);
        cv::find4QuadCornerSubpix(capt_gray, imagePoints, cv::Size2i(3, 3));
        cv::drawChessboardCorners(capture, cv::Size2i(chessNumX - 1, chessNumY - 1), imagePoints, found);

        cv::imshow("Calibration", capture);
        cv::waitKey(0);
        cv::destroyWindow("Calibration");

        //find homography
        cv::Mat masks;
        cv::Mat hmat = cv::findHomography(imagePoints, projectPoints, masks, cv::RANSAC);

        //save homography mat
        cv::FileStorage fs("hmat.xml", cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            std::cout << "File can not be opened." << std::endl;
            return -1;
        }
        fs << "homography_mat" << hmat;
        fs.release();
    }
#else
    cv::Mat hmat;

    //read homography matrix
    cv::FileStorage fs("hmat.xml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cout << "File can not be opened." << std::endl;
        return -1;
    }
    fs["homography_mat"] >> hmat;
    fs.release();

    //apply homography transformation to captured image
    //cv::Mat camera = cv::imread("chess_capt.png", 1);	// Ofir, please change here to your camera capture code
    //cv::Mat camera = basler_camera.RetrieveFrame();	// Ofir, please change here to your camera capture code
    cv::Mat camera;
    basler_camera.read(camera);	// Ofir, please change here to your camera capture code
    cv::Mat warped = cv::Mat::zeros(cv::Size(projectorW, projectorH), CV_8UC3);
    cv::TickMeter meter;
    meter.start();
    cv::warpPerspective(camera, warped, hmat, cv::Size(projectorW, projectorH));
    meter.stop();
    std::cout << meter.getTimeMilli() << "ms" << std::endl;

    //disp homography transformation result
     // set fullscreen window
    cv::namedWindow("Homography", cv::WINDOW_NORMAL);
    cv::setWindowProperty("Homography", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::imshow("Homography", warped);
    cv::waitKey(0);
    cv::destroyWindow("Homography");

#endif

    return 0;

    }