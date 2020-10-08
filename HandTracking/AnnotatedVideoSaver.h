#pragma once
#include <opencv2/videoio.hpp>
#include <concurrent_queue.h>
#include <iostream>
#include <synchapi.h>
#include <direct.h>
#include "VideoSaver.h"
#include <iomanip>
#include "AnnotationSaver.h"
#define CRT_SECURE_NO_WARNINGS


class AnnotatedVideoSaver
{
private:
    VideoSaver videoSaver;
    AnnotationSaver annotationSaver;
    static std::string GetCurrentTimeForFileName()
    {
        auto time = std::time(nullptr);
        struct tm timeinfo;
        localtime_s(&timeinfo, &time);
        std::stringstream ss;
        ss << std::put_time(&timeinfo, "%F_%T"); // ISO 8601 without timezone information.
        auto s = ss.str();
        std::replace(s.begin(), s.end(), ':', '-');
        return s;
    }
public:
    bool running;

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
        if (!running)
        {
            string timestamp = GetCurrentTimeForFileName();
            _mkdir((path + timestamp + "\\").c_str());
            videoSaver.Start(path + timestamp + "\\Original.avi");
            annotationSaver.Start(path + timestamp + "\\Annotations.csv");
            running = true;
        }
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
        running = false;
        thread* tr1 = new thread([&]()
        {
            videoSaver.Close();
        });
        thread* tr2 = new thread([&]()
        {
            annotationSaver.Close();
        });
        tr1->join();
        tr2->join();
        cout << "Finish Saving\n";
    }
};
