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


    string  ToCsvRecord()
    {
        std::string record;
        for (int i = 0; i < keypoint_number; i++)
        {
            keypoints_values[2 * i] = keypoints[i].x;
            keypoints_values[2 * i + 1] = keypoints[i].y;

            if(i>0)
            {
                record += ",";
            }

            record += to_string(keypoints_values[2 * i]);
            record += "," ;
            record += to_string(keypoints_values[2 * i+1]);
        }
        record += "\n";
        return record;
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
        output_file << "w_x,w_y,t0_x,t0_y,t1_x,t1_y,t2_x,t2_y,t3_x,t3_y,i0_x,i0_y,i1_x,i1_y,i2_x,i2_y,i3_x,i3_y,m0_x,m0_y,m1_x,m1_y,m2_x,m2_y,m3_x,m3_y,r0_x,r0_y,r1_x,r1_y,r2_x,r2_y,r3_x,r3_y,p0_x,p0_y,p1_x,p1_y,p2_x,p2_y,p3_x,p3_y\n";
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
        output_file.close();
    }
};
