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
using namespace std;


int main(int argc, char **argv)
{
	SetConsoleTitle("Hand Tracking Console");
	/*
	vector<cv::Point2f> pattern1 = { cv::Point2f(510,366),cv::Point2f(487,314) ,cv::Point2f(497,301) ,
		cv::Point2f(507,294) ,cv::Point2f(522,295),cv::Point2f(541,317) };
*/
	//tip of fingers and base of fingers
	vector<cv::Point2f> pattern1 = { /*cv::Point2f(500,302),*/
		cv::Point2f(461,308),cv::Point2f(453,252),
        cv::Point2f(440,305),cv::Point2f(425,241),
        cv::Point2f(423,314),cv::Point2f(405,257),
        cv::Point2f(405,330),cv::Point2f(378,280),
	};

	LeapMotion leap;
	leap.Connect();

	LeapToImageMapper mapper(&leap);

	mapper.SetPattern(pattern1);

	// Set the hook
	UserInputHandler* a = UserInputHandler::getInstance();
	a->algoHook = new CalibrationAlgorithmHook(&mapper);
	a->Go();

	BaslerCamera basler_camera;
	//cv::VideoCapture basler_camera(0);
	cv::Mat frame;
	VideoSaver videoSaver;
	VideoShowerAndPattern video_shower;
	auto manipulator = new LeapToImageMappingManipulator(&mapper);
	video_shower.ApplyImageManipulation(manipulator);

	int counter = 0;
	bool save = false;
	//save = true;
	//Initialize capture,saver,shower
	basler_camera.StartGrabbing();
	leap.StartGrabbing();

	if (save)
		videoSaver.Start("IR_Video_20.avi");

	video_shower.Start(&frame);     


	thread* main_loop = new thread([&]()
	{
		while (basler_camera.IsGrabbing() && video_shower.running /*&& (counter++ < 500 * 15 | !save)*/)
		//while (basler_camera.read(frame) && video_shower.running /*&& (counter++ < 500 * 15 | !save)*/)
		{
			frame = basler_camera.RetrieveFrame();
			if (save)
				videoSaver.AddFrame(frame.clone());
		}

		//Close all objects
		video_shower.Stop();

		if (save)
			videoSaver.Close();

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