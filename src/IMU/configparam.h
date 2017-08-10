#ifndef CONFIGPARAM_H
#define CONFIGPARAM_H


#include <Eigen/Dense>
#include <opencv2/opencv.hpp>


namespace ORB_SLAM2
{
    class ConfigParam
    {
    public:
        // Eigen矩阵内存对齐
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ConfigParam(std::string configfile);

        double _testDiscardTime;

        static Eigen::Matrix4d GetEigTbc();

        static cv::Mat GetMatTbc();

        static Eigen::Matrix4d GetEigT_cb();

        static cv::Mat GetMatT_cb();

        static int GetLocalWindowSize();

        static double GetImageDelayToIMU();

        static bool GetAccMultiply9p8();

        static double GetG()
        {
            return _g;
        }

        // 测试数据集位置，img,IMU节点名称
        std::string _bagfile;
        std::string _imageTopic;
        std::string _imuTopic;

        static std::string getTmpFilePath();

        static std::string _tmpFilePath;

        static double GetVINSInitTime()
        {
            return _nVINSInitTime;
        }

        static bool GetRealTimeFlag()
        {
            return _bRealTime;
        }

    private:
        static Eigen::Matrix4d _EigTbc;
        static cv::Mat _MatTbc;
        static Eigen::Matrix4d _EigTcb;
        static cv::Mat _MatTcb;
        static int _LocalWindowSize;
        static double _ImageDelayToIMU;
        static bool _bAccMultiply9p8;            // ACC数据是否归一化

        static double _g;
        // 初始化所用时间
        static double _nVINSInitTime;
        static bool _bRealTime;                // 是否实时运行

    };


}


#endif