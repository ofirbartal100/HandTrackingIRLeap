#pragma once
#include <opencv2/videoio.hpp>
#include <concurrent_queue.h>
#include <iostream>
#include <synchapi.h>
#include "VideoSaver.h"
#include <iomanip>
#include "AnnotationSaver.h"

std::string GetCurrentTimeForFileName()
{
    auto time = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%F_%T"); // ISO 8601 without timezone information.
    auto s = ss.str();
    std::replace(s.begin(), s.end(), ':', '-');
    return s;
}

class AnnotatedVideoSaver
{
private:
    VideoSaver videoSaver;
    AnnotationSaver annotationSaver;
    bool running;
   
public:
    AnnotatedVideoSaver()
    {
        running = false;
    }
    ~AnnotatedVideoSaver()
    {
        if (running)
            Close();
    }

    //start the background thread that saves frames.
    void Start(std::string path = ".\\")
    {
        string timestamp = GetCurrentTimeForFileName();
        videoSaver.Start(path + timestamp + "\\Original.avi");
        annotationSaver.Start(path + timestamp + "\\Annotations.csv");
        running = true;
    }

    //add a frame to the saving frames queue if running
    void AddFrameAndAnnotation(cv::Mat frame, Annotation annotation)
    {
        if (running)
        {
            videoSaver.AddFrame(frame);
            annotationSaver.AddAnnotation(annotation);
        }
    }

    //stop recieving new frames and join the background thread
    void Close()
    {
        videoSaver.Close();
        annotationSaver.Close();
        running = false;
        cout << "Finish Saving\n";
    }
};
