#pragma once
#include <filesystem>
#include "opencv2/tracking.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <ctime>
#include <ppl.h>
#include <concurrent_queue.h>
#include <thread>
#include "ImageManipulator.h"

using namespace std;

class VideoShower
{
protected:
	const cv::Mat* referencedFrame;
	bool is_image_manipulation;
	ImageManipulator* manipulator;

	thread* showing_thread;

	void showing_loop()
	{
		cout << "start showing" << endl;
		while (running)
		{
			//every loop takes the most up to date value of the referenced frame, and show it when ready
			cv::Mat temp = referencedFrame->clone();
			if (is_image_manipulation)
			{
				manipulator->Manipulate(temp);
			}
			cv::imshow("Preview", temp);
			int k = cv::waitKey(30);
			if (k == 27)
			{
				running = false;
				cout << "Closed Video Shower\n";
				break;
			}
		}

	}
public:
	bool running;

	VideoShower()
	{
		running = false;
		manipulator = new ImageManipulator();
	}
	~VideoShower()
	{
		if (running)
			Stop();
		if(showing_thread)
		{
			delete showing_thread;
			cout << "thread* showing_thread deleted\n";
		}
		if (manipulator)
		{
			delete manipulator;
			cout << "ImageManipulator* manipulator deleted\n";
		}
	}

	//start the showing thread in the background, showing the registered frame
	void Start(const cv::Mat* registerFrame)
	{
		running = true;
		referencedFrame = registerFrame;
		showing_thread = new thread(&VideoShower::showing_loop, this);
	}

	//stop the showing thread, waiting for it to finish
	void Stop()
	{
		if (running)
		{
			running = false;
			showing_thread->join();
			cout << "Closed Video Shower\n";
		}
	}

};
