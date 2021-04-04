
#include "UdpServerParser.h"
#include <igl/opengl/glfw/Viewer.h>
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
//#include "UnityDataPipe.h";
#include "FrameDeformator.h";
#include <vector>
#include <algorithm>
#include <filesystem>
#include <ppl.h>
#include <ctime>
#include <iostream>
#include <chrono>
using namespace std::chrono;
using namespace std;

igl::opengl::glfw::Viewer viewer;
FrameDeformation deformator;

//need to be here because of imports issues
void SetMeshFromFrame(cv::Mat image, std::vector<cv::Point2f> v)
{
    if (v.size() == 21)
    {
        deformator.SetMeshFromFrame(image, v);
        viewer.data().set_mesh(deformator.U, deformator.F);
        viewer.data().set_uv(deformator.UV);
        viewer.data().set_texture(deformator._R, deformator._G, deformator._B);
        viewer.data().show_texture = true;
    }
}

//need to be here because of imports issues
void UpdateTextureAndHandles(cv::Mat image, std::vector<cv::Point2f> v)
{
    if (v.size() == 21)
    {
        deformator.UpdateTextureAndHandles(image, v);
        viewer.data().set_uv(deformator.UV);
        viewer.data().set_texture(deformator._R, deformator._G, deformator._B);
    }
}

//need to be here because of imports issues
void DeformMesh(std::vector<cv::Point2f> v)
{
    if (v.size() == 21)
    {
        deformator.DeformMesh(v);
        viewer.data().set_mesh(deformator.U, deformator.F);
    }
}



int main(int argc, char **argv)
{

#pragma region Initializations
    // Initializations
    bool isAlgorithmRunning = false, isSetMesh = false;

    // Leap & Basler initialization
    //vector<cv::Point3f> leap_values;
    //LeapMotion leap;

    //show only mapping at first
    cv::Mat black_frame(540, 720, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat ir_frame;
    //BaslerCamera basler_camera(500, 5000);
    //cv::VideoCapture basler_camera(0);
    //cv::VideoCapture basler_camera("D:\\TAU\\Research\\AnnotatedVideos\\2020-12-28_16-12-54-v\\Original.avi");
    //cv::VideoCapture basler_camera("D:\\TAU\\Research\\AnnotatedVideos\\2020-12-28_16-09-31-v\\Original.avi");

    //assuming its calibrated already and the file is good
    //LeapToImageMapper mapper(&leap, &black_frame);
    //mapper.LoadPoints();
    //mapper.Calibrate();

    //show the video
    //VideoShowerAndPattern video_shower;
    //auto manipulator = new LeapToImageMappingManipulator(&mapper, cv::Scalar(255, 0, 50));
    //video_shower.ApplyImageManipulation(manipulator);


    vector<cv::Point2f> regrresed_keypoint, appearance_regrresed_keypoint;
    vector<cv::Point2f> unity_keypoint(21);

    //model init
    //AppearanceModel appearanceModel;

    //start sensors
    //leap.Connect();
    //leap.StartGrabbing();
    //basler_camera.StartGrabbing();


    thread* viewer_loop = new thread([&]()
    {
        viewer.launch();
    });

    //pull keypoint updates from unity
    UdpServerParser unityKeypointsParser;

    int a = 0;
    cin >> a;
    //init spout receiver
    cv::Mat game_frame;
    //1024x764
    Opencv2Spout spoutConverter(argc, argv, 1024, 768, false);
    char receiverName[256];
    strcpy_s(receiverName, 256, "UserPerspective");
    bool receiverExists = spoutConverter.initReceiver(receiverName);
    if (receiverExists)
        cout << "Receiver " << receiverName << " detected " << endl;
    else
    {
        cout << "No receiver detected." << endl;
        return -1;
    }



#pragma endregion

#pragma region Calibration Process (if leap and camera are not calibrated)

    //calibration stage - show mapped leap motion on the hand
    //video_shower.Start(&black_frame);

    //wait for start button to be pressed
    /*bool waitForUserInput = true;
    while (waitForUserInput)
    {
        char key = (char)getchar();
        if (key == ' ')
        {
            waitForUserInput = false;
            //get the current frame
            //ir_frame = basler_camera.RetrieveFrame();
            //basler_camera >> ir_frame;
            //regrresed_keypoint = mapper.projections;//clone projection mapping

            game_frame = spoutConverter.receiveTexture();
            unity_keypoint = unityKeypointsPipe.ReadKeypointsFromPipe();
            deformator.SetMeshFromFrame(game_frame, unity_keypoint); //register mesh and compute L-1
            //leap.StopGrabbing();
            //video_shower.Stop();
        }
        else if (key == 'q')
        {
            //leap.StopGrabbing();
            //video_shower.Stop();
            waitForUserInput = false;
            return 0;
        }
    }*/
#pragma endregion

#pragma region Main Loop


    //after 'start' was pressed

    //cv::Mat deformed_game_frame;
    isAlgorithmRunning = true;

    //game stage / show tracked keypoints on black_frame
    //PointsPatternManipulator* pointManipulator = new PointsPatternManipulator(regrresed_keypoint, cv::Scalar(255, 0, 50));
    //video_shower.ApplyImageManipulation(pointManipulator);
    //video_shower.Start(&black_frame);
    //video_shower.Start(&ir_frame);

    // 500 FPS frames loop
    // get ir frames for fast regression and deformation
    thread* ir_frames_loop = new thread([&]()
    {
        /*
         //while (basler_camera.IsGrabbing() && isAlgorithmRunning)
        while (basler_camera.isOpened() && isAlgorithmRunning)
        {
            //get the current frame
            //ir_frame = basler_camera.RetrieveFrame();
            basler_camera >> ir_frame;


            //regress the latest keypoints
            //use ir_frame and regrresed_keypoint and predict the new regrresed_keypoint
            appearanceModel.forward(ir_frame, regrresed_keypoint);
            //appearance_regrresed_keypoint = appearanceModel.forward(ir_frame, regrresed_keypoint);
            //regrresed_keypoint = appearance_regrresed_keypoint;

            appearanceModel.forward(ir_frame, regrresed_keypoint);


            //deform game_frame using regrresed_keypoint
            // deformed_game_frame <= deform(game_frame,regrresed_keypoint)
            //pointManipulator->ChangePoints(regrresed_keypoint);

            DeformMesh(regrresed_keypoint); // deformation + projection via igl viewer

            //project to projector plane and project
            // projector.project(deformed_game_frame)



            // uncomment if using the leap
            //get latest leap values
            //leap_values = leap.GetJoints();

            //project leap on frame

        }

        //Close all objects
        //basler_camera.Close();
        basler_camera.release();
        */
    });


    // 30 FPS 
    // get frames from game with hand texture and position
    thread* game_frames_loop = new thread([&]()
    {
        while (isAlgorithmRunning)
        {
            game_frame = spoutConverter.receiveTexture();
            //spoutConverter.draw(game_frame, true);
            bool success = unityKeypointsParser.updatedKeypoints(unity_keypoint);
            if (success && !isSetMesh)
            {
                SetMeshFromFrame(game_frame, unity_keypoint);
                isSetMesh = true;
            }
            else if(success)
            {
                UpdateTextureAndHandles(game_frame, unity_keypoint);
            }
            if (cv::waitKey(30) > 0)
                break;
        }
    });

    while (isAlgorithmRunning)
    {
        char key;
        cin >> key;
        if (key == 'q')
        {
            isAlgorithmRunning = false;
            ir_frames_loop->join();
            game_frames_loop->join();
            return 0;
        }
    }

#pragma endregion

    return 0;
}
