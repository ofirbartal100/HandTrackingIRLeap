#pragma once
#include <windows.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <regex>
#include <opencv2/core/mat.hpp>
using namespace std;


class UnityDataPipe
{
    HANDLE fileHandle;
    char* buffer;
    int buffer_size;

    int ReadString(char* output) {
        ULONG read = 0;
        int index = 0;
        do {
            ReadFile(fileHandle, output + index++, 1, &read, NULL);
        } while (read > 0 && *(output + index - 1) != 0);
        return index;
    }

    int* raw_keypoints;
    std::vector<cv::Point2f> keypoints;


public:
    UnityDataPipe(int buf_size = 300)
    {
        buffer_size = buf_size;
        buffer = new char[buffer_size];
        raw_keypoints = new int[42];
        //read skeleton from unity
        fileHandle = CreateFileW(TEXT(L"\\\\.\\pipe\\my-very-cool-pipe-example"), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);
    }

    std::vector<cv::Point2f> ReadKeypointsFromPipe()
    {
        memset(buffer, 0, buffer_size);
        int len = ReadString(buffer);

        raw_keypoints = { 0 };
        int counter = 0;
        for (int i = 0; i < len && counter < 42; i++)
        {
            if (buffer[i] >= '0' &&  buffer[i] <= '9')
            {
                while (buffer[i] >= '0' &&  buffer[i] <= '9')
                {
                    raw_keypoints[counter] = raw_keypoints[counter] * 10 + buffer[i] - '0';
                    i++;
                }
                std::cout << raw_keypoints[counter];
                counter += 1;

            }
        }

        //read all 42 keypoints
        if(counter == 42)
        {
            //update the keypoints
            for (int i = 0; i < 21; i++)
            {
                keypoints.push_back(cv::Point2f(raw_keypoints[2 * i], raw_keypoints[2 * i + 1]));
            }
        }

        return keypoints;
    }


};