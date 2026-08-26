#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>

#include <opencv2/opencv.hpp>
#include <System.h>

using namespace std;

void ProcessWebcam(
    ORB_SLAM2::System* pSLAM,
    std::atomic<bool>* running)
{
    cv::VideoCapture cap(0);

    if(!cap.isOpened())
    {
        cerr << "ERROR: Could not open webcam." << endl;
        *running = false;
        return;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    cout << "Webcam opened successfully." << endl;

    while(*running)
    {
        cv::Mat frame;

        cap >> frame;

        if(frame.empty())
        {
            cerr << "ERROR: Empty frame received." << endl;
            break;
        }

        double timestamp =
            chrono::duration_cast<chrono::duration<double>>(
                chrono::steady_clock::now().time_since_epoch()
            ).count();

        pSLAM->TrackMonocular(frame, timestamp);

        // Small delay so we don't hammer the camera/CPU
        std::this_thread::sleep_for(
            std::chrono::milliseconds(1)
        );
    }

    cap.release();

    *running = false;
}

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl
             << "Usage: ./mono_webcam "
             << "path_to_vocabulary path_to_settings"
             << endl;

        return 1;
    }

    cout << endl;
    cout << "Starting ORB-SLAM2 Webcam..." << endl;

    /*
     * IMPORTANT:
     *
     * true creates the Viewer object.
     * We DO NOT launch it as a separate thread.
     *
     * RunViewer() will be called from the MAIN THREAD.
     */
    ORB_SLAM2::System SLAM(
        argv[1],
        argv[2],
        ORB_SLAM2::System::MONOCULAR,
        true
    );

    atomic<bool> running(true);

    /*
     * Webcam + SLAM processing happens in a worker thread.
     */
    thread webcamThread(
        ProcessWebcam,
        &SLAM,
        &running
    );

    /*
     * IMPORTANT FOR macOS:
     *
     * Pangolin/AppKit must run on the MAIN THREAD.
     */
    SLAM.RunViewer();

    /*
     * Viewer closed.
     * Stop webcam processing.
     */
    running = false;

    if(webcamThread.joinable())
        webcamThread.join();

    SLAM.Shutdown();

    cout << "ORB-SLAM2 stopped." << endl;

    return 0;
}