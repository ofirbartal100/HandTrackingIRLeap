#pragma once
#include <windows.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iostream>
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
        ReadFile(fileHandle, output, buffer_size, &read, NULL);
        output[read] = 0;
        return read;
        /*do {
            ReadFile(fileHandle, output + index++, 1, &read, NULL);
        } while (read > 0 && *(output + index - 1) != 0);
        return index;*/
    }

    int raw_keypoints[21 * 2 + 1];
    std::vector<cv::Point2f> keypoints;


public:
    UnityDataPipe(int buf_size = 1024 * 4)
    {
        buffer_size = buf_size;
        buffer = new char[buffer_size];
        //read skeleton from unity
        fileHandle = CreateFileW(TEXT(L"\\\\.\\pipe\\my-very-cool-pipe-example-2"),GENERIC_READ | GENERIC_WRITE, FILE_SHARE_WRITE,
            NULL, OPEN_EXISTING, 0, NULL);//Pipe must exist.
    }

    void ReadKeypointsFromPipe(std::vector<cv::Point2f>& keypoints)
    {
        memset(buffer, 0, buffer_size);
        int len = ReadString(buffer);

        int start = -1, end = len;
        int temp_start = 0;
        int ii = 0;
        while (ii < len)
        {
            char cc = buffer[ii];
            if (cc == '#')
            {
                temp_start = ii;
            }
            if (cc == '@')
            {
                end = ii;
                start = temp_start;
            }
            ii++;
        }
        string s(buffer);
        raw_keypoints[0] = 0;
        int counter = 0;
        if (start > 0)
        {
            cout << s.substr(start, end - start) << endl;
            for (int i = start; i < end && counter < 21 * 2; i++)
            {
                if (buffer[i] >= '0' &&  buffer[i] <= '9')
                {
                    while (buffer[i] >= '0' &&  buffer[i] <= '9' && i < end)
                    {
                        raw_keypoints[counter] = raw_keypoints[counter] * 10 + buffer[i] - '0';
                        i++;
                    }
                    //std::cout << raw_keypoints[counter];
                    counter += 1;
                    raw_keypoints[counter] = 0;
                }
            }
            //read all 42 keypoints
            if (counter == 21 * 2)
            {
                keypoints.clear();
                //update the keypoints
                for (int i = 0; i < 21; i++)
                {
                    keypoints.push_back(cv::Point2f(raw_keypoints[2 * i], raw_keypoints[2 * i + 1]));
                }

            }
        }


    }


};