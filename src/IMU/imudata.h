#ifndef IMUDATA_H
#define IMUDATA_H

#include <Eigen/Dense>


namespace ORB_SLAM2
{
    using namespace Eigen;

    class IMUData
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // IMU测量离散高斯白噪声
        static Matrix3d _gyrMeasCov;
        static Matrix3d _accMeasCov;

        static Matrix3d getGyrMeasCov(void)
        {
            return _gyrMeasCov;
        }

        static Matrix3d getAccMeasCov(void)
        {
            return _accMeasCov;
        }

        // IMU传感器bias
        static Matrix3d _gyrBiasRWCov;
        static Matrix3d _accBiasRWCov;

        static Matrix3d getGyrBiasRWCov(void)
        {
            return _gyrBiasRWCov;
        }

        static Matrix3d getAccBiasRWCov(void)
        {
            return _accBiasRWCov;
        }

        static double _gyrBiasRw2;
        static double _accBiasRw2;

        static double getGyrBiasRW2(void)
        {
            return _gyrBiasRw2;
        }

        static double getAccBiasRW2(void)
        {
            return _accBiasRw2;
        }

        // 构造函数
        IMUData(const double &gx, const double &gy, const double &gz,
                const double &ax, const double &ay, const double &az,
                const double &t);

        // gyr,acc数据
        Vector3d _g;
        Vector3d _a;

        // 时间戳
        double _t;

    };

}


#endif