#pragma once
#include <filesystem>
#include <ctime>
#include <ppl.h>
#include <concurrent_queue.h>
#include <thread>
#include <iostream>
#include <synchapi.h>
#include <fstream>
#include <numeric>

using namespace std;
using namespace concurrency;

class Annotation
{
    static const int keypoint_number = 21;
    std::vector<cv::Point2f> keypoints;
    float keypoints_values[keypoint_number * 2];

public:
    Annotation() { }
    Annotation(std::vector<cv::Point2f> values) { keypoints = values; }


    string ToCsvRecord()
    {
        for (int i = 0; i < keypoint_number; i++)
        {
            keypoints_values[2 * i] = keypoints[i].x;
            keypoints_values[2 * i + 1] = keypoints[i].y;
        }
        string record = std::accumulate(std::begin(keypoints_values), std::end(keypoints_values), string(),
            [](string &ss, string &s)
        {
            return ss.empty() ? s : ss + "," + s;
        });

        return record + "\n";
    }
};

class AnnotationSaver
{
private:
    concurrent_queue<Annotation> grabbed_annotations;
    bool running;
    int frameRate;
    std::ofstream output_file;

    thread* saving_thread;
    // Background thread function to asynchronicly save captured frames to a video file
    void saving_loop()
    {
        Annotation temp_annotation;
        cout << "start saving video" << endl;
        //As long as we are saving, and there are frames to save
        while (running || !grabbed_annotations.empty())
        {
            if (grabbed_annotations.try_pop(temp_annotation))
            {
                output_file << temp_annotation.ToCsvRecord();
            }
            Sleep(1);
        }
        cout << "done saving annotations" << endl;
    }
public:
    AnnotationSaver()
    {
        running = false;
    }
    ~AnnotationSaver()
    {
        if (running)
            Close();
    }

    //start the background thread that saves frames.
    void Start(string outputName = "Annotations.csv")
    {
        output_file = std::ofstream(outputName);
        running = true;
        saving_thread = new thread(&AnnotationSaver::saving_loop, this);
    }

    //add a frame to the saving frames queue if running
    void AddAnnotation(Annotation annotation)
    {
        if (running)
            grabbed_annotations.push(annotation);
    }

    //stop recieving new frames and join the background thread
    void Close()
    {
        running = false;
        saving_thread->join();
        cout << "Finish Saving\n";
        output_file.close();
    }
};
