/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include<stdlib.h>

#ifdef _JB_EDIT_

#include <sys/types.h>
#include <sys/shm.h>
#include <unistd.h>

#define ROW_SIZE 480
#define COL_SIZE 640

struct RGB_DATA{
    unsigned char r[ROW_SIZE][COL_SIZE];
    unsigned char g[ROW_SIZE][COL_SIZE];
    unsigned char b[ROW_SIZE][COL_SIZE];
};

#endif

#define STREAM_MODE


using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
#ifdef STREAM_MODE // Stream Mode with YOLO
    
    if(argc != 3)
    {
        cerr << endl << "argc:" << argc << "!= 3"<< endl;
    }


    cv::VideoCapture cap(0);

    if (!cap.isOpened()) {
        cerr << endl << "Could not open camera feed." << endl;
        return -1;
    }

#ifdef _JB_EDIT_

    int shmid = 0;
    RGB_DATA  *shm_info= NULL;
    void *shared_memory = (void *)0;

    shmid = shmget((key_t)4003, sizeof(RGB_DATA), 0666|IPC_CREAT);

    if (shmid == -1)
    {
        perror("shmget failed : ");
        exit(0);
    }

    shared_memory = shmat(shmid, (void *)0, 0666|IPC_CREAT);

    if (shared_memory == (void *)-1)
    {
        perror("shmat attach is failed : ");
        exit(0);
    }

    shm_info = (RGB_DATA*) shared_memory;
#endif

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);
    
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point initT = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point initT = std::chrono::monotonic_clock::now();
#endif


    // Main loop
    while(true)//cv::waitKey(0) != 27)
    {
       //Create a new Mat
        cv::Mat frame;

        //Send the captured frame to the new Mat
        cap >> frame;

        if(frame.empty())
            break;
#ifdef _JB_EDIT_
        for (int y = 0; y < ROW_SIZE; y++) {
            // y번째 row에 대한 주소를 포인터에 저장한 후
            uchar* pointer_input = frame.ptr<uchar>(y);

            for (int x = 0; x < COL_SIZE; x++) {

                // row 포인터로부터 (x * 3 )번째 떨어져 있는 픽셀을 가져옵니다.
                //0, 1, 2 순서대로 blue, green, red 채널값을 가져올 수있는 이유는 하나의 픽셀이 메모리상에 b g r 순서대로 저장되기 때문입니다.
                shm_info->b[y][x] = pointer_input[x * 3 + 0];
                shm_info->g[y][x] = pointer_input[x * 3 + 1];
                shm_info->r[y][x] = pointer_input[x * 3 + 2];
            }
        }

        SLAM.SharedBoundingBoxData();
#endif

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point nowT = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point nowT = std::chrono::monotonic_clock::now();
#endif
        // Pass the image to the SLAM system
        SLAM.TrackMonocular(frame, std::chrono::duration_cast<std::chrono::duration<double> >(nowT-initT).count());
    }

#ifdef _JB_EDIT_
    SLAM.YOLOShutDown();
#endif

    // Stop all threads
    SLAM.Shutdown();

    //slam->SaveSeperateKeyFrameTrajectoryTUM("KeyFrameTrajectory-1.txt", "KeyFrameTrajectory-2.txt", "KeyFrameTrajectory-3.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    
#else   // Example Mode
    
    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
#endif

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
