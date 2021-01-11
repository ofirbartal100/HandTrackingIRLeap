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
#include <thread>

// Include files to use the pylon API.
#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif

using namespace std;
using namespace Pylon;

class BaslerCamera
{
private:
    // Camera instance pointer
    CBaslerUsbInstantCamera * camera;

    // This smart pointer will receive the grab result data.
    CGrabResultPtr ptrGrabResult;
public:
    BaslerCamera(int fps = 500, int exposure = 2000)
    {
        PylonInitialize();

        //protect from abortion in case camera is not connected
        try
        {
            // Create an instant camera object with the camera device found first.
            camera = new CBaslerUsbInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
        }
        catch(const GenICam_3_1_Basler_pylon_v5_1::RuntimeException& e)
        {
            cout << "No Basler Camera Detected!" << endl;
            exit(-3);
        }


        // Print the model name of the camera.
        cout << "Using device " << camera->GetDeviceInfo().GetModelName() << endl;

        camera->Open();
        camera->AcquisitionFrameRateEnable = true;
        camera->AcquisitionFrameRate = fps;
        camera->ExposureTime = 2000;

    }
    ~BaslerCamera()
    {
        Close();
        PylonTerminate();
    }

    void StartGrabbing() const
    {
        camera->StartGrabbing();
    }

    cv::Mat RetrieveFrame()
    {
        // Camera.StopGrabbing() is called automatically by the RetrieveResult() method
        // Wait for an image and then retrieve it. A timeout of 500 ms is used.
        camera->RetrieveResult(500, ptrGrabResult, TimeoutHandling_ThrowException);
        // Image grabbed successfully?
        if (ptrGrabResult->GrabSucceeded())
        {
            //Return an Mat object containing the captured frame
            return cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8U, (uint8_t*)ptrGrabResult->GetBuffer());
        }
        else
        {
            //Error Mat
            return cv::Mat();
        }
    }
    bool IsGrabbing() const
    {
        return camera->IsGrabbing();
    }
    void Close() {
        if (camera->IsOpen())
            camera->Close();

        cout << "Closed Basler Camera\n";
    }
};