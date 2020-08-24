#pragma once
#include <filesystem>
#include "opencv2/core/cvstd_wrapper.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <opencv2/objdetect.hpp>
#include <ctime>
#include <ppl.h>
#include <concurrent_queue.h>
#include <thread>

using namespace std;
using namespace concurrency;


class VideoSaver
{
private:
	cv::VideoWriter outputVideo;
	concurrent_queue<cv::Mat> grabbed_frames;
	bool running;

	int frameRate;
	cv::Size frameSize;

	thread* saving_thread;
	// Background thread function to asynchronicly save captured frames to a video file
	void saving_loop()
	{
		cv::Mat temp_frame;

		cout << "start saving video" << endl;
		//As long as we are saving, and there are frames to save
		while (running || !grabbed_frames.empty())
		{
			if (grabbed_frames.try_pop(temp_frame))
			{
				outputVideo << temp_frame;
			}
			Sleep(1);
		}
		cout << "done saving video" << endl;
	}
public:
	VideoSaver()
	{
		running = false;
		frameRate = 500;
		frameSize = cv::Size(720, 540);
	}
	~VideoSaver()
	{
		if (running)
			Close();
	}

	//start the background thread that saves frames.
	void Start(string outputName = "outputVideo.avi")
	{
		outputVideo.open(outputName, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), frameRate, frameSize, false);
		running = true;
		saving_thread = new thread(&VideoSaver::saving_loop, this);
	}

	//add a frame to the saving frames queue if running
	void AddFrame(cv::Mat frame)
	{
		if (running)
			grabbed_frames.push(frame);
	}

	//stop recieving new frames and join the background thread
	void Close()
	{
		running = false;
		saving_thread->join();
		outputVideo.release();
	}
};
