/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<unistd.h>
#include<thread>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;


// ------------------------------------------------------------
// Function declarations
// ------------------------------------------------------------

void LoadImages(
    const string &strFile,
    vector<string> &vstrImageFilenames,
    vector<double> &vTimestamps
);

void ProcessImages(
    ORB_SLAM2::System* pSLAM,
    const string& sequencePath,
    const vector<string>& vstrImageFilenames,
    const vector<double>& vTimestamps
);


// ------------------------------------------------------------
// Main
// ------------------------------------------------------------

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl
             << "Usage: ./mono_tum "
                "path_to_vocabulary "
                "path_to_settings "
                "path_to_sequence"
             << endl;

        return 1;
    }


    // --------------------------------------------------------
    // Retrieve paths to images
    // --------------------------------------------------------

    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;

    string strFile = string(argv[3]) + "/rgb.txt";

    LoadImages(
        strFile,
        vstrImageFilenames,
        vTimestamps
    );

    int nImages = vstrImageFilenames.size();


    // --------------------------------------------------------
    // Create SLAM system
    //
    // IMPORTANT:
    //
    // false means DO NOT create the Viewer thread.
    //
    // On macOS, Pangolin/AppKit must run on the MAIN thread.
    // We therefore start the viewer explicitly below.
    // --------------------------------------------------------

    ORB_SLAM2::System SLAM(
        argv[1],
        argv[2],
        ORB_SLAM2::System::MONOCULAR,
        true
    );


    // --------------------------------------------------------
    // Start image processing in a worker thread
    // --------------------------------------------------------

    std::thread trackingThread(
        ProcessImages,
        &SLAM,
        string(argv[3]),
        std::cref(vstrImageFilenames),
        std::cref(vTimestamps)
    );


    // --------------------------------------------------------
    // IMPORTANT:
    //
    // Pangolin MUST run on the macOS MAIN THREAD.
    //
    // This is what prevents:
    //
    // "nextEventMatchingMask should only be called
    //  from the Main Thread!"
    //
    // --------------------------------------------------------

    SLAM.RunViewer();


    // --------------------------------------------------------
    // Wait for image-processing thread to finish
    // --------------------------------------------------------

    if(trackingThread.joinable())
        trackingThread.join();


    // --------------------------------------------------------
    // Stop all SLAM threads
    // --------------------------------------------------------

    SLAM.Shutdown();


    return 0;
}


// ------------------------------------------------------------
// Process TUM images
// ------------------------------------------------------------

void ProcessImages(
    ORB_SLAM2::System* pSLAM,
    const string& sequencePath,
    const vector<string>& vstrImageFilenames,
    const vector<double>& vTimestamps)
{
    int nImages = vstrImageFilenames.size();

    vector<float> vTimesTrack(nImages);


    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: "
         << nImages
         << endl << endl;


    // --------------------------------------------------------
    // Main tracking loop
    // --------------------------------------------------------

    cv::Mat im;

    for(int ni = 0; ni < nImages; ni++)
    {
        // ----------------------------------------------------
        // Read image
        // ----------------------------------------------------

        im = cv::imread(
            sequencePath + "/" + vstrImageFilenames[ni],
            cv::IMREAD_UNCHANGED
        );

        double tframe = vTimestamps[ni];


        // ----------------------------------------------------
        // Check image
        // ----------------------------------------------------

        if(im.empty())
        {
            cerr << endl
                 << "Failed to load image at: "
                 << sequencePath << "/"
                 << vstrImageFilenames[ni]
                 << endl;

            return;
        }


        // ----------------------------------------------------
        // Start timing
        // ----------------------------------------------------

        auto t1 = std::chrono::steady_clock::now();


        // ----------------------------------------------------
        // Pass image to ORB-SLAM2
        // ----------------------------------------------------

        pSLAM->TrackMonocular(
            im,
            tframe
        );


        // ----------------------------------------------------
        // End timing
        // ----------------------------------------------------

        auto t2 = std::chrono::steady_clock::now();


        double ttrack =
            std::chrono::duration_cast<
                std::chrono::duration<double>
            >(t2 - t1).count();


        vTimesTrack[ni] = ttrack;


        // ----------------------------------------------------
        // Match the original TUM playback rate
        // ----------------------------------------------------

        double T = 0;

        if(ni < nImages - 1)
        {
            T = vTimestamps[ni + 1] - tframe;
        }
        else if(ni > 0)
        {
            T = tframe - vTimestamps[ni - 1];
        }


        if(ttrack < T)
        {
            usleep(
                static_cast<useconds_t>(
                    (T - ttrack) * 1e6
                )
            );
        }
    }


    cout << endl;
    cout << "Image processing finished."
         << endl;


    // --------------------------------------------------------
    // Tracking statistics
    // --------------------------------------------------------

    sort(
        vTimesTrack.begin(),
        vTimesTrack.end()
    );


    float totaltime = 0;

    for(int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }


    cout << "-------" << endl << endl;

    cout << "median tracking time: "
         << vTimesTrack[nImages / 2]
         << endl;

    cout << "mean tracking time: "
         << totaltime / nImages
         << endl;


    // --------------------------------------------------------
    // Save trajectory
    // --------------------------------------------------------

    pSLAM->SaveKeyFrameTrajectoryTUM(
        "KeyFrameTrajectory.txt"
    );
}


// ------------------------------------------------------------
// Load TUM image timestamps and filenames
// ------------------------------------------------------------

void LoadImages(
    const string &strFile,
    vector<string> &vstrImageFilenames,
    vector<double> &vTimestamps)
{
    ifstream f;

    f.open(strFile.c_str());


    if(!f.is_open())
    {
        cerr << "Failed to open image list: "
             << strFile
             << endl;

        return;
    }


    // --------------------------------------------------------
    // Skip first three header lines
    // --------------------------------------------------------

    string s0;

    getline(f, s0);
    getline(f, s0);
    getline(f, s0);


    // --------------------------------------------------------
    // Read timestamp + image filename
    // --------------------------------------------------------

    while(!f.eof())
    {
        string s;

        getline(f, s);

        if(!s.empty())
        {
            stringstream ss;

            ss << s;

            double t;
            string sRGB;

            ss >> t;
            ss >> sRGB;

            vTimestamps.push_back(t);
            vstrImageFilenames.push_back(sRGB);
        }
    }
}