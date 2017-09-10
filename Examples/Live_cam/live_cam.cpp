//
//  live_test.cpp
//  ORB_SLAM2
//
//  Created by Shun-Cheng Wu on 10/09/2017.
//
//

#include "live_cam.hpp"

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int processing(char **argv, ORB_SLAM2::System *slamPtr);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    auto resultFuture = async(launch::async, processing, argv, &SLAM);
    SLAM.RunViewer();
    return resultFuture.get();
}

int processing(char **argv, ORB_SLAM2::System *slamPtr) {
    ORB_SLAM2::System& SLAM = *slamPtr;
    
    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    
    int nImages;
    if(string(argv[3]) == "live"){
        cout << "live mode" << endl;
        nImages = 0;
        vTimestamps.push_back(0.0);
    } else {
        cout << "load image" << endl;
        LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);
        nImages = vstrImageFilenames.size();
    }
    
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;
    
    // Main loop
    cv::Mat im;
    if(string(argv[3]) == "live"){
        char stopkey = 0;
        cv::VideoCapture cap(0);
        cap.set(CV_CAP_PROP_FRAME_WIDTH,1200);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT,600);
        CV_Assert(cap.isOpened()==1 && "Camera cannot be opened");
        while (stopkey != 'q') {
            auto ni = nImages;
            ++nImages;
            // Capture image from camera
            cap.grab();
            cap.read(im);
            cv::cvtColor(im,im,cv::COLOR_RGB2GRAY); //convert to gray scale
            //cv::imshow("12345current image", im);
            //cvWaitKey(30);
            double tframe = vTimestamps[ni];
            
            if(im.empty())
            {
                cerr << endl << "Failed to load image" << endl;
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
            
            vTimesTrack.resize(nImages);
            vTimesTrack[ni]=ttrack;
            
            // Wait to load the next frame
            double T=0;
            if(ni<nImages-1)
                T = vTimestamps[ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestamps[ni-1];
            
            if(ttrack<T)
                std::this_thread::sleep_for(std::chrono::duration<double, std::micro>((T-ttrack)*1e6));
            vTimestamps.push_back(ttrack);
            
            //stopkey = cv::waitKey(30);
            cout<<nImages<<endl;
        }
    } else {
        for(int ni=0; ni<nImages; ni++)
        {
            // Read image from file
            im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
            double tframe = vTimestamps[ni];
            
            if(im.empty())
            {
                cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
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
                std::this_thread::sleep_for(std::chrono::duration<double, std::micro>((T-ttrack)*1e6));
        }
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
    
    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }
    
    string strPrefixLeft = strPathToSequence + "/image_0/";
    
    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);
    
    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}
