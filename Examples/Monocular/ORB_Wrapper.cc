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
#include <iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

struct ORB_Wrapper
{
	typedef ORB_SLAM2::KeyFrame KeyFrame;

	ORB_SLAM2::System SLAM;

	KeyFrame * mCurrentKeyFrame;

	std::vector< KeyFrame * > mKeyFramePtr;

	ORB_Wrapper(const char* vocabFile, const char * settingsFile):
		SLAM(vocabFile,settingsFile,ORB_SLAM2::System::MONOCULAR,true),
		mCurrentKeyFrame(nullptr){}

	cv::Mat process(
			const cv::Mat& im,
			const double& timeStamp,
			bool& keyFrameUpdated)
	{

		keyFrameUpdated = false;
		cv::Mat Tcw = SLAM.TrackMonocular(im, timeStamp);
		ORB_SLAM2::KeyFrame* currentKeyFrame = SLAM.GetLastKeyFrame();
		if(currentKeyFrame)
		{
			if(!mCurrentKeyFrame)
			{
				mKeyFramePtr.push_back(currentKeyFrame);
				mCurrentKeyFrame = currentKeyFrame;
				keyFrameUpdated = true;
			}
			else
			{
				if(mCurrentKeyFrame!=currentKeyFrame)
				{
					mKeyFramePtr.push_back(currentKeyFrame);
					mCurrentKeyFrame = currentKeyFrame;
					keyFrameUpdated = true;
				}
			}
		}

		float minID, maxID;
		GetMinMaxInverseDepth(mCurrentKeyFrame, minID, maxID);

		return Tcw.clone();
	}

	// Get the min - max inverse depth
	// return value indicates if we were able to successfully get them
	bool GetMinMaxInverseDepth(ORB_SLAM2::KeyFrame* pKF, float& minInverseDepth, float& maxInverseDepth)
	{
		if(pKF and isValid(pKF))
		{
			pKF->ComputeMinMaxInverseDepth(minInverseDepth, maxInverseDepth);
			return true;
		}
		return false;

	}

	// Return a vector of KeyFrame pointers that are covsibile with the current KF
	std::vector< KeyFrame* > GetCovisibleKeyFrames(ORB_SLAM2::KeyFrame* pKF, const int& N)
	{
		std::vector< KeyFrame* > covisibleFrames;
		if(pKF and isValid(pKF))
		{
			auto covisibleKeyFrames = pKF->GetBestCovisibilityKeyFrames(N);

			for( auto& frame: covisibleKeyFrames)
			{
				if( frame->isBad()) continue;
				covisibleFrames.push_back(frame);
			}
		}
		return covisibleFrames;
	}

	bool isValid(KeyFrame* pKF)
	{
		return !pKF->isBad();
	}

	~ORB_Wrapper()
	{
		SLAM.Shutdown();
	}
};


struct Frame
{
	cv::Mat Tcw;
	cv::Mat im;
	ORB_SLAM2::KeyFrame* keyFramePtr;
};

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

    std::vector< Frame > frames;

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ORB_Wrapper SLAM(argv[1],argv[2]);

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
        bool hasNewKeyFrame;
        cv::Mat Tcw  = SLAM.process(im,tframe, hasNewKeyFrame);
        Frame f;
        f.im = im.clone();
        f.Tcw = Tcw.clone();
        if(hasNewKeyFrame)
        {
        	f.keyFramePtr = SLAM.mCurrentKeyFrame;
        }
        else
        	f.keyFramePtr = nullptr;
        frames.push_back(f);

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
    //SLAM.Shutdown();

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
   // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

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
