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

// Include files to use the pylon API.
#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif

extern "C" {
#include <LeapC.h>
#include <ExampleConnection.h>
}


using namespace cv;
using namespace std;
using namespace Pylon;
using namespace concurrency;

class LeapMotion
{
private:
	int64_t lastFrameID = 0; //The last frame received

	LEAP_TRACKING_EVENT *frame;

	Vec3d LeapVToVec3d(LEAP_VECTOR lv)
	{
		return Vec3d(lv.x, lv.y, lv.z);
	}
public:
	LeapMotion() {}
	~LeapMotion() {	}

	//Blocking
	void Connect()
	{
		//leap connection
		OpenConnection();
		while (!IsConnected)
			millisleep(100); //wait a bit to let the connection complete

		printf("Connected.");
		LEAP_DEVICE_INFO* deviceProps = GetDeviceProperties();
		if (deviceProps)
			printf("Using device %s.\n", deviceProps->serial);
	}

	bool UpdateFrame()
	{
		frame = GetFrame();
		if (frame && (frame->tracking_frame_id > lastFrameID)) {
			lastFrameID = frame->tracking_frame_id;
			return true;
		}
		return false;
	}


	Vec3d* GetJoints()
	{
		Vec3d joints[21];
		if (frame->nHands > 0)
		{
			LEAP_HAND* hand = &frame->pHands[0];
			int c = 0;
			joints[c] = LeapVToVec3d(hand->palm.position);
			c++;

			for (int i = 0; i < 4; i++)
				joints[c + i] = LeapVToVec3d(hand->index.bones->next_joint);
			c += 4;

			for (int i = 0; i < 4; i++)
				joints[c + i] = LeapVToVec3d(hand->middle.bones->next_joint);
			c += 4;

			for (int i = 0; i < 4; i++)
				joints[c + i] = LeapVToVec3d(hand->ring.bones->next_joint);
			c += 4;

			for (int i = 0; i < 4; i++)
				joints[c + i] = LeapVToVec3d(hand->pinky.bones->next_joint);

			return joints;
		}
		return nullptr;
	}

};
class BaslerCamera
{
private:
	// Camera instance pointer
	CBaslerUsbInstantCamera * camera;

	// This smart pointer will receive the grab result data.
	CGrabResultPtr ptrGrabResult;
public:
	BaslerCamera()
	{
		PylonInitialize();

		// Create an instant camera object with the camera device found first.
		camera = new CBaslerUsbInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());

		// Print the model name of the camera.
		cout << "Using device " << camera->GetDeviceInfo().GetModelName() << endl;

		camera->Open();
		camera->AcquisitionFrameRateEnable = true;
		camera->AcquisitionFrameRate = 500;
		camera->ExposureTime = 1000;
	}
	~BaslerCamera()
	{
		Close();
	}

	void StartGrabbing() const
	{
		camera->StartGrabbing();
	}
	Mat RetrieveFrame()
	{
		// Camera.StopGrabbing() is called automatically by the RetrieveResult() method
		// Wait for an image and then retrieve it. A timeout of 500 ms is used.
		camera->RetrieveResult(500, ptrGrabResult, TimeoutHandling_ThrowException);
		// Image grabbed successfully?
		if (ptrGrabResult->GrabSucceeded())
		{
			//Return an Mat object containing the captured frame
			return Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8U, (uint8_t*)ptrGrabResult->GetBuffer());
		}
		else
		{
			//Error Mat
			return Mat();
		}
	}
	bool IsGrabbing() const
	{
		return camera->IsGrabbing();
	}
	void Close() {
		if (camera->IsOpen())
			camera->Close();
	}
};

class VideoSaver
{
private:
	VideoWriter outputVideo;
	concurrent_queue<Mat> grabbed_frames;
	bool running;

	int frameRate;
	Size frameSize;

	thread* saving_thread;
	// Background thread function to asynchronicly save captured frames to a video file
	void saving_loop()
	{
		Mat temp_frame;

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
		frameSize = Size(720, 540);
	}
	~VideoSaver()
	{
		if (running)
			Close();
	}

	//start the background thread that saves frames.
	void Start(string outputName = "outputVideo.avi")
	{
		outputVideo.open(outputName, VideoWriter::fourcc('M', 'J', 'P', 'G'), frameRate, frameSize, false);
		running = true;
		saving_thread = new thread(&VideoSaver::saving_loop, this);
	}

	//add a frame to the saving frames queue if running
	void AddFrame(Mat frame)
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

typedef void(*image_manipulation_func)(Mat& im);

class VideoShower
{
protected:
	const Mat* referencedFrame;
	bool running;
	bool is_image_manipulation;
	image_manipulation_func manipulate_image;

	thread* showing_thread;
	void showing_loop()
	{
		cout << "start showing" << endl;

		while (running)
		{
			//every loop takes the most up to date value of the referenced frame, and show it when ready
			Mat temp = referencedFrame->clone();
			if (is_image_manipulation)
			{
				manipulate_image(temp);
			}
			cv::imshow("Preview", temp);
			int k = waitKey(30);
			if (k == 27)
			{
				break;
			}
		}
		cout << "done showing" << endl;
	}
public:
	VideoShower() { running = false; }
	~VideoShower()
	{
		if (running)
			Stop();
	}

	//start the showing thread in the background, showing the registered frame
	void Start(const Mat* registerFrame)
	{
		running = true;
		referencedFrame = registerFrame;
		showing_thread = new thread(&VideoShower::showing_loop, this);
	}

	//stop the showing thread, waiting for it to finish
	void Stop()
	{
		running = false;
		showing_thread->join();
	}

};


class VideoShowerAndPattern : public VideoShower
{

public:
	static vector<Vec2i> _pattern;

	static void show_pattern(Mat& m)
	{
		for (auto p : _pattern)
		{
			auto c = Point(p[0], p[1]);
			circle(m, c, 5, Scalar(200, 0, 0), -1);
		}
	}
	void ApplyImageManipulation(vector<Vec2i> pattern)
	{
		_pattern = pattern;
		manipulate_image = [](Mat& m) {
			show_pattern(m);
		};
		is_image_manipulation = true;
	}

	void StopImageManipulation()
	{
		is_image_manipulation = false;
	}
};


class CalibrationModule
{
public:
	const vector<Vec2i> pattern1 = { Vec2i(100,100),Vec2i(150,150) ,Vec2i(210,100) ,Vec2i(310,310) ,Vec2i(190,410) };
};
