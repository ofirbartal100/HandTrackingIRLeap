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

extern "C"{
	#include <LeapC.h>
	#include <ExampleConnection.h>
}


using namespace cv;
using namespace std;
using namespace Pylon;
using namespace concurrency;


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

class VideoShower
{
private:
	const Mat* referencedFrame;
	bool running;

	thread* showing_thread;
	void showing_loop()
	{
		cout << "start showing" << endl;

		while (running)
		{
			//every loop takes the most up to date value of the referenced frame, and show it when ready
			Mat temp = referencedFrame->clone();
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


int main(int argc, char **argv)
{
	//leap connection
	OpenConnection();
	while (!IsConnected)
		millisleep(100); //wait a bit to let the connection complete

	printf("Connected.");
	LEAP_DEVICE_INFO* deviceProps = GetDeviceProperties();
	if (deviceProps)
		printf("Using device %s.\n", deviceProps->serial);

	

	BaslerCamera basler_camera;
	Mat frame;
	VideoSaver videoSaver;
	VideoShower video_shower;

	int counter = 0;
	bool save = false;
	//save = true;
	//Initialize capture,saver,shower
	basler_camera.StartGrabbing();

	if(save)
		videoSaver.Start("IR_Hands_Video_18.avi");

	video_shower.Start(&frame);


	//cout << "start capturing" << endl;
	//clock_t begin = clock();
	while (basler_camera.IsGrabbing() /*&& (counter++ < 500 * 15 | !save)*/ )
	{
		frame = basler_camera.RetrieveFrame();

		if(save)
			videoSaver.AddFrame(frame.clone());
	}
	//clock_t end = clock();
	//cout << "done capturing in: " << double(end - begin) / CLOCKS_PER_SEC << "seconds" << endl;

	//Close all objects
	video_shower.Stop();

	if(save)
		videoSaver.Close();

	basler_camera.Close();
}