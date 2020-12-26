#include <iostream>
#include "GL\glew.h"
#include "GL\freeglut.h"
#include "opencv2\opencv.hpp"
#include "Opencv2Opengl.h"
#include <filesystem>
#include "opencv2/core/cvstd_wrapper.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <opencv2/objdetect.hpp>
#include <ctime>
#include <ppl.h>

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
#include "AlgorithmHook.h"
#include "AnnotatedVideoSaver.h"
using namespace std;
#define USE_GLEW


int main(int argc, char **argv)
{
    // Initializations
    bool isAlgorithmRunning = true;

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

    // Leap & Basler initialization
    vector<cv::Point3f> leap_values;
    LeapMotion leap;
    leap.Connect();
    cv::Mat ir_frame;
    LeapToImageMapper mapper(&leap, &ir_frame);
    BaslerCamera basler_camera;


    basler_camera.StartGrabbing();
    leap.StartGrabbing();

    vector<cv::Point2f> regrresed_keypoint;


    // 500 FPS frames loop
    // get ir frames for fast regression and deformation
    thread* ir_frames_loop = new thread([&]()
    {
        while (basler_camera.IsGrabbing())
        {
            //get the current frame
            ir_frame = basler_camera.RetrieveFrame();


            //regress the latest keypoints 
            //use ir_frame and regrresed_keypoint and predict the new regrresed_keypoint
            // regrresed_keypoint <= model(ir_frame,regrresed_keypoint,attention?)

            //deform game_frame using regrresed_keypoint
            // deformed_game_frame <= deform(game_frame,regrresed_keypoint)

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

    // get frames from game with hand texture and position
    thread* game_frames_loop = new thread([&]()
    {
        while (isAlgorithmRunning)
        {
            game_frame = spoutConverter.receiveTexture();
            Sleep(1000 / 40);
        }

    });

    ir_frames_loop->join();
    game_frames_loop->join();

    return 0;
}

