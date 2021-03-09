#include "GL\freeglut.h"
#include "opencv2\opencv.hpp"
#include "Opencv2Opengl.h"
#include "opencv2/core/cvstd_wrapper.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <opencv2/objdetect.hpp>
#include "VideoShower.h"
#include "ImageManipulator.h"
#include "VideoShowerAndPattern.h"
#include "BaslerCamera.h"
#include "LeapMotion.h"
#include "VideoSaver.h"
#include "PointsPatternManipulator.h"
#include "UserInputHandler.h"
#include "LeapToImageMapper.h"
#include "LeapToImageMappingManipulator.h"
#include "PointsPatternManipulator.h"
#include "AlgorithmHook.h"
#include "AnnotatedVideoSaver.h"
#include "AppearanceModel.h"
#include "UnityDataPipe.h";
#include <vector>
#include <algorithm>
#include <filesystem>
#include <ppl.h>
#include <ctime>
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{


#pragma region Initializations
    // Initializations
    bool isAlgorithmRunning = false;

    // Leap & Basler initialization
    vector<cv::Point3f> leap_values;
    LeapMotion leap;

    //show only mapping at first
    cv::Mat black_frame(720, 540, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat ir_frame;
    BaslerCamera basler_camera;

    //assuming its calibrated already and the file is good
    LeapToImageMapper mapper(&leap, &black_frame);
    mapper.LoadPoints();
    mapper.Calibrate();

    //show the video
    VideoShowerAndPattern video_shower;
    auto manipulator = new LeapToImageMappingManipulator(&mapper, cv::Scalar(255, 0, 50));
    video_shower.ApplyImageManipulation(manipulator);


    vector<cv::Point2f> regrresed_keypoint, appearance_regrresed_keypoint;
    vector<cv::Point2i> unity_keypoint;

    //model init
    AppearanceModel appearanceModel;

    int stage = 0;

    //start sensors
    leap.Connect();
    leap.StartGrabbing();
    basler_camera.StartGrabbing();



#pragma endregion


#pragma region Calibration Process

    //calibration stage - show mapped leap motion on the hand
    video_shower.Start(&black_frame);

    //wait for start button to be pressed
    bool waitForUserInput = true;
    while (waitForUserInput)
    {
        char key = (char)cv::waitKey(0);
        if (key == ' ')
        {
            waitForUserInput = false;
            //get the current frame
            ir_frame = basler_camera.RetrieveFrame();
            regrresed_keypoint = mapper.projections;//clone projection mapping
            stage = 1;
            leap.StopGrabbing();
            video_shower.Stop();
        }
        else if (key == 'q')
        {
            stage = 1;
            leap.StopGrabbing();
            video_shower.Stop();
            return 0;
        }
    }
#pragma endregion


#pragma region Main Loop


    //after 'start' was pressed

    //init spout receiver
    cv::Mat game_frame;
    Opencv2Spout spoutConverter(argc, argv, 640, 480, false);
    char receiverName[9];
    strcpy_s(receiverName, 9, "SpoutCam");
    bool receiverExists = spoutConverter.initReceiver(receiverName);
    if (receiverExists)
        cout << "Receiver " << receiverName << " detected " << endl;
    else
    {
        cout << "No receiver detected." << endl;
        return -1;
    }

    //pull keypoint updates from unity
    UnityDataPipe unityKeypointsPipe;

    isAlgorithmRunning = true;

    //game stage / show tracked keypoints on black_frame
    PointsPatternManipulator* pointManipulator = new PointsPatternManipulator(regrresed_keypoint, cv::Scalar(255, 0, 50));
    video_shower.ApplyImageManipulation(pointManipulator);
    video_shower.Start(&black_frame);


    // 500 FPS frames loop
    // get ir frames for fast regression and deformation
    thread* ir_frames_loop = new thread([&]()
    {
        while (basler_camera.IsGrabbing() && isAlgorithmRunning)
        {
            //get the current frame
            ir_frame = basler_camera.RetrieveFrame();


            //regress the latest keypoints 
            //use ir_frame and regrresed_keypoint and predict the new regrresed_keypoint
            appearance_regrresed_keypoint = appearanceModel.forward(ir_frame, regrresed_keypoint);
            regrresed_keypoint = appearance_regrresed_keypoint;

            //deform game_frame using regrresed_keypoint
            // deformed_game_frame <= deform(game_frame,regrresed_keypoint)
            pointManipulator->ChangePoints(regrresed_keypoint);

            //project to projector plane and project
            // projector.project(deformed_game_frame)


            /*
            // uncomment if using the leap
            //get latest leap values
            leap_values = leap.GetJoints();

            //project leap on frame
            */
        }

        //Close all objects
        basler_camera.Close();
    });


    // 30 FPS 
    // get frames from game with hand texture and position
    thread* game_frames_loop = new thread([&]()
    {
        while (isAlgorithmRunning)
        {
            game_frame = spoutConverter.receiveTexture();
            unity_keypoint = unityKeypointsPipe.ReadKeypointsFromPipe();
            Sleep(1000 / 35);
        }
    });

    while (isAlgorithmRunning)
    {
        char key = (char)cv::waitKey(0);
        if (key == 'q')
        {
            stage = 1;
            isAlgorithmRunning = false;
            ir_frames_loop->join();
            game_frames_loop->join();
            video_shower.Stop();
            return 0;
        }
    }

#pragma endregion


    return 0;
}