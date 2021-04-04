#pragma once
#include "server/asio/service.h"
#include "server/asio/udp_server.h"
#include <opencv2/core/mat.hpp>
#include <string>
#include <vector>
#include <iostream>
#pragma comment (lib, "crypt32")
class AsioService : public CppServer::Asio::Service
{
public:
    using CppServer::Asio::Service::Service;

protected:
    void onError(int error, const std::string& category, const std::string& message) override
    {
        std::cout << "Asio service caught an error with code " << error << " and category '" << category << "': " << message << std::endl;
    }
};


class UdpParser
{

public:

    cv::Point2f keypoints[21];
    int raw_keypoints[42];

    void Parse(std::string str, size_t size)
    {
        ReadKeypointsFromPipe(str, size);
    }


    void ReadKeypointsFromPipe(std::string str, size_t size)
    {
        const char *buffer = str.c_str();
        int len = (int)str.size();
        int temp = 0;
        int counter = 0;
        char c;
        if (len > 0)
        {
            for (int i = 0; i < len && counter < 21 * 2; i++)
            {
                c = buffer[i];

                if (c >= '0' &&  c <= '9')
                {
                    while (c >= '0' &&  c <= '9' && i < len)
                    {
                        temp = temp * 10 + c - '0';
                        i++;
                        c = buffer[i];
                    }
                   
                    raw_keypoints[counter] = temp;
                    temp = 0;
                    counter += 1;
                }
            }
            //read all 42 keypoints
            if (counter == 21 * 2)
            {
                //update the keypoints
                for (int i = 0; i < 21; i++)
                {
                    keypoints[i].x = raw_keypoints[2 * i];
                    keypoints[i].y = raw_keypoints[2 * i + 1];
                }

            }
        }
    }
};


class UdpListeningServer : public CppServer::Asio::UDPServer
{
public:
    using CppServer::Asio::UDPServer::UDPServer;
    UdpParser* parser;

    void setParser(UdpParser* pointer_parser)
    {
        parser = pointer_parser;
    }

protected:
    void onStarted() override
    {
        // Start receive datagrams
        ReceiveAsync();
    }

    void onReceived(const asio::ip::udp::endpoint& endpoint, const void* buffer, size_t size) override
    {

        std::string message((const char*)buffer, size);
        //std::cout << "Incoming: " << message << std::endl;

        parser->Parse(message, size);
        // Echo the message back to the sender
        //SendAsync(endpoint, message);

        // Continue receive datagrams
        ReceiveAsync();
    }

    void onSent(const asio::ip::udp::endpoint& endpoint, size_t sent) override
    {
        // Continue receive datagrams
        ReceiveAsync();
    }

    void onError(int error, const std::string& category, const std::string& message) override
    {
        std::cout << "Echo UDP server caught an error with code " << error << " and category '" << category << "': " << message << std::endl;
    }
};


class UdpServerParser
{
public:
    // UDP server port
    int port;
    std::shared_ptr<AsioService> service;
    std::shared_ptr<UdpListeningServer> server;
    UdpParser *parser;


    UdpServerParser()
    {
        port = 3333;

        // Create a new Asio service
        service = std::make_shared<AsioService>();

        // Start the Asio service
        std::cout << "Asio service starting...";
        service->Start();
        std::cout << "Done!" << std::endl;

        // Create a new UDP echo server
        server = std::make_shared<UdpListeningServer>(service, port);
        parser = new UdpParser();
        server->setParser(parser);
        // Start the server
        std::cout << "Server starting...";
        server->Start();
        std::cout << "Done!" << std::endl;


    }

    ~UdpServerParser()
    {
        // Stop the server
        std::cout << "Server stopping...";
        server->Stop();
        std::cout << "Done!" << std::endl;

        // Stop the Asio service
        std::cout << "Asio service stopping...";
        service->Stop();
        std::cout << "Done!" << std::endl;

        delete parser;
    }

    bool updatedKeypoints(std::vector<cv::Point2f>& keypoints)
    {
        if (keypoints.size() == 21)
        {
            //update the keypoints
            for (int i = 0; i < 21; i++)
            {
                keypoints[i].x = parser->keypoints[i].x;
                keypoints[i].y = parser->keypoints[i].y;
            }
            return true;
        }
        return false;
    }


};