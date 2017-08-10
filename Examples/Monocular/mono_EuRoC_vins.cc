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
#include <stdlib.h>
#include <signal.h>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <imudata.h>
#include <configparam.h>


using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void LoadImus(const string &strImuPath, vector<ORB_SLAM2::IMUData> &vImus);

// 对类型变量的操作是原子的，不会被线程调度机制打断。
// SIGINT信号标志位
volatile sig_atomic_t sigflag = 0;

// 信号处理函数
void sigint_function(int sig)
{
    cout << "SIGINT catch" << endl;
    sigflag = 1;
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cerr << endl << "Usage: ./mono_euroc_vins  vocabulary  settings " << endl;
        cerr << endl << "For example: " << endl <<
             "./mono_euroc_vins ../../Vocabulary/ORBvoc.bin ../../config/euroc.yaml" << endl;

        return 1;
    }

    // 读取IMU配置文件
    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);

    // IMU Image 数据集位置
    string datafolder = string(fSettings["bagfile"]);
    string IMUfile = datafolder + "/imu0/data.csv";
    string Imagefolder = datafolder + "/cam0/data";
    string Imagefile = datafolder + "/cam0/data.csv";

    // SIGINT表示终端的终端信号
    // 该函数表示，当有来自键盘的SIGINT信号时，执行信号处理函数sigint_function。
    signal(SIGINT, sigint_function);

    // 加载IMU数据
    vector<ORB_SLAM2::IMUData> vImus;
    LoadImus(IMUfile, vImus);
    int nImus = vImus.size();
    cout << "Imus in data: " << nImus << endl;
    if (nImus <= 0)
    {
        cerr << "ERROR: Failed to load imus" << endl;
        return 1;
    }

    // 加载图像数据，图像存储位置和时间戳
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(Imagefolder, Imagefile, vstrImageFilenames, vTimestamps);
    int nImages = vstrImageFilenames.size();
    cout << "Images in data: " << nImages << endl;

    if (nImages <= 0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    const int startIdx = fSettings["Seq.startIdx"];
    // const int endIdx = vstrImageFilenames.size();
    // const int endIdx =fSettings["Seq.endIdx"];

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // 选取指定的数据范围，默认startIdx=1。
    int imagestart = startIdx;
    int startimutime = 0;
    // double time_interval = 1.0/double(fSettings["Camera.fps"]);
    // 找到第一个图像时间戳小于于等于IMU时间戳+time_interval的IMU数据。
    while(1)
    {
        if(vImus[startimutime]._t >= vTimestamps[imagestart-1])
            break;

        startimutime++;

        if(sigflag) // ctrl-c exit
            return 1;
    }

    // 找到第一个图像时间戳大于等于IMU时间戳的图像。
    while (1)
    {
        //printf("imagestarttime: %.6f\n",vTimestamps[imagestart]);
        if (startimutime <= vTimestamps[imagestart])
            break;

        imagestart++;

        if (sigflag) // ctrl-c exit
            return 1;
    }

    printf("startimutime: %.6f\n", vImus[startimutime]._t);
    printf("imagestarttime: %.6f\n", vTimestamps[startimutime]);

    cout << "imagestart:" << imagestart << endl;

    // Main loop
    long imuindex = startimutime;
    cv::Mat im;
    for (int ni = imagestart; ni < nImages; ni++)
    {
        if (sigflag) // ctrl-c exit
            break;

        // Read image from file
        im = cv::imread(vstrImageFilenames[ni], CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        vector<ORB_SLAM2::IMUData> vimu;
        while (1)
        {
            const ORB_SLAM2::IMUData &imudata = vImus[imuindex];
            if (imudata._t >= tframe)
                break;
            vimu.push_back(imudata);
            imuindex++;
        }

        if (im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << vstrImageFilenames[ni] << endl;
            return 1;
        }

        // 相机20fps, imu为200hz, 两帧之间不应该超过10个imu数据
        if (vimu.size() > 10)
            printf("%lu imu messages between images, note", vimu.size());
        if (vimu.size() == 0)
            printf("no imu message between images!");

        // 记录Track时间
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image and imu to the SLAM system
        SLAM.TrackMonoVI(im, vimu, tframe);


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (ni < nImages - 1)
            T = vTimestamps[ni + 1] - tframe;
        else if (ni > 0)
            T = tframe - vTimestamps[ni - 1];

        if (ttrack < T)
            usleep((T - ttrack) * 1e6);

    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    // Save camera trajectory
    string Trajectory_folder = fSettings["test.InitVIOTmpPath"];
    SLAM.SaveKeyFrameTrajectoryTUM(Trajectory_folder + "KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryTUM(Trajectory_folder + "CameraFrameTrajectory.txt");
    return 0;
}


// 读取IMU数据文件内容，读取全部内容保存到vIMUs中
void LoadImus(const string &strImuPath, vector<ORB_SLAM2::IMUData> &vImus)
{
    ifstream fImus;
    fImus.open(strImuPath.c_str());
    vImus.reserve(30000);
    //int testcnt = 10;
    while (!fImus.eof())
    {
        string s;
        getline(fImus, s);
        if (!s.empty())
        {
            char c = s.at(0);
            if (c < '0' || c > '9')
                continue;

            stringstream ss;
            ss << s;
            double tmpd;
            int cnt = 0;
            double data[10];    // timestamp, wx,wy,wz, ax,ay,az
            while (ss >> tmpd)
            {
                data[cnt] = tmpd;
                cnt++;
                if (cnt == 7)
                    break;
                if (ss.peek() == ',' || ss.peek() == ' ')
                    ss.ignore();
            }
            // data[0]是时间戳。
            data[0] *= 1e-9;
            ORB_SLAM2::IMUData imudata(data[1], data[2], data[3],
                                       data[4], data[5], data[6], data[0]);
            vImus.push_back(imudata);

        }
    }
}


// 读取图像数据， 保存图像时间戳vTimeStamps和存放位置vstrImages
void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    //int testcnt=10;
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            char c = s.at(0);
            if (c < '0' || c > '9')
                continue;

            stringstream ss;
            ss << s;
            long tmpi;
            int cnt = 0;
            while (ss >> tmpi)
            {
                cnt++;
                if (cnt == 1)
                {
                    //cout<<tmpi<<endl;
                    break;
                }
                if (ss.peek() == ',' || ss.peek() == ' ' || ss.peek() == '.')
                    ss.ignore();
            }
            //string tmpstr(strImagePath + "/" + ss.str() + ".png");
            string tmpstr(strImagePath + "/" + to_string(tmpi) + ".png");
            //cout<<tmpstr<<endl;
            vstrImages.push_back(tmpstr);
            //double t;
            //ss >> t;
            vTimeStamps.push_back(tmpi * 1e-9);

        }

        //if(testcnt--<=0)
        //    break;
    }
    /*for(size_t i=0; i<vstrImages.size(); i++)
    {
        printf("image: %s\n timestamp: %.9f\n",vstrImages[i].c_str(),vTimeStamps[i]);
    }*/
}