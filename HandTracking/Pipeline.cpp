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


int main2(int argc, char **argv)
{
	SetConsoleTitle("Hand Tracking Console");
	/*
	vector<cv::Point2f> pattern1 = { cv::Point2f(510,366),cv::Point2f(487,314) ,cv::Point2f(497,301) ,
		cv::Point2f(507,294) ,cv::Point2f(522,295),cv::Point2f(541,317) };
*/
	//tip of fingers and base of fingers left back hand
	//vector<cv::Point2f> pattern1 = { /*cv::Point2f(500,302),*/
	//	cv::Point2f(461,308),cv::Point2f(453,252),
 //       cv::Point2f(440,305),cv::Point2f(425,241),
 //       cv::Point2f(423,314),cv::Point2f(405,257),
 //       cv::Point2f(405,330),cv::Point2f(378,280),
	//};

    //tip of fingers and base of fingers right front hand
    //vector<cv::Point2f> pattern1 = { /*cv::Point2f(500,302),*/
    //    cv::Point2f(376,334),cv::Point2f(370,297),
    //    cv::Point2f(364,336),cv::Point2f(356,296),
    //    cv::Point2f(356,341),cv::Point2f(343,306),
    //    cv::Point2f(347,351),cv::Point2f(328,325),
    //};
    //tip of fingers and base of fingers right front hand
    /*vector<cv::Point2f> pattern1 = { cv::Point2f(220,390),cv::Point2f(290,320),
        cv::Point2f(256,328),cv::Point2f(280,269),
        cv::Point2f(239,319),cv::Point2f(259,253),
        cv::Point2f(220,318),cv::Point2f(233,259),
        cv::Point2f(199,324),cv::Point2f(205,271),
    };*/

 /*   vector<cv::Point2f> pattern1 = { cv::Point2f(720-220,390),cv::Point2f(720 - 320,320),
        cv::Point2f(720 - 256,328),cv::Point2f(720 - 280,269),
        cv::Point2f(720 - 239,319),cv::Point2f(720 - 259,253),
        cv::Point2f(720 - 220,318),cv::Point2f(720 - 233,259),
        cv::Point2f(720 - 199,324),cv::Point2f(720 - 205,271),
    };
*/
	LeapMotion leap;
	leap.Connect();
    cv::Mat frame;

	LeapToImageMapper mapper(&leap,&frame);
	//mapper.SetPattern(pattern1);
    AnnotatedVideoSaver annotatedVideoSaver;

    //VideoSaver videoSaver;
    VideoShowerAndPattern video_shower;
    auto manipulator = new LeapToImageMappingManipulator(&mapper);
    video_shower.ApplyImageManipulation(manipulator);
	// Set the hook
	UserInputHandler* a = UserInputHandler::getInstance();
    CalibrationAlgorithmHook* c = new CalibrationAlgorithmHook(&mapper);
    RecordAlgorithmHook* r = new RecordAlgorithmHook(&annotatedVideoSaver);
    a->baseAlgoHook = new BaseAlgorithmHook(a,c,r);
    a->dynamicAlgoHook = a->baseAlgoHook;
	a->Go();

	BaslerCamera basler_camera;
	//cv::VideoCapture basler_camera(0);

	

	int counter = 0;
	bool save = false;
	//save = true;
	//Initialize capture,saver,shower
	basler_camera.StartGrabbing();
	leap.StartGrabbing();

	//if (save)
	//	//videoSaver.Start("IR_Video_20.avi");
 //       annotatedVideoSaver.Start("D:\\TAU\\Research\\AnnotatedVideos\\");

	video_shower.Start(&frame);     


	thread* main_loop = new thread([&]()
	{
		while (basler_camera.IsGrabbing() && video_shower.running )
		{
			frame = basler_camera.RetrieveFrame();
			if (annotatedVideoSaver.running)
			{
                annotatedVideoSaver.AddFrameAndAnnotation(frame.clone(), Annotation(mapper.MapLeapTo2DJoints()));
			}
				//videoSaver.AddFrame(frame.clone());
		}

		//Close all objects
		video_shower.Stop();

		/*if (save)
            annotatedVideoSaver.Close();*/
			//videoSaver.Close();

		basler_camera.Close();
		//basler_camera.release();
	});

	cout << "Listening For User Input..\n";
	
	////// Don't mind this, it is a meaningless loop to keep a console application running.
	////// I used this to test the keyboard hook functionality. If you want to test it, keep it in ;)
	MSG msg;
	while (GetMessage(&msg, NULL, 0, 0))
	{
		
	}
	main_loop->join();
	if(main_loop)
	{
		delete main_loop;
		cout << "thread* main_loop deleted\n";
	}

	return 0;
}