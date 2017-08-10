#ifndef TVISLAM_IMUPREINTEGRATOR_H
#define TVISLAM_IMUPREINTEGRATOR_H

#include <Eigen/Dense>

#include "IMU/imudata.h"
#include "IMU/so3.h"

namespace ORB_SLAM2
{
    using namespace Eigen;
    using namespace Sophus;

    typedef Eigen::Matrix<double, 9, 9> Matrix9d;

    class IMUPreintegrator
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        IMUPreintegrator();

        IMUPreintegrator(const IMUPreintegrator &pre);

        // 重置初始化状态
        void reset();

        // 增量式更新所有测量状态，jacobian和噪声不确定性
        void update(const Vector3d &omega, const Vector3d &acc, const double &dt);


        // 状态测量增量
        inline Eigen::Vector3d getDeltaP() const
        {
            return _delta_P;
        }

        inline Eigen::Vector3d getDeltaV() const
        {
            return _delta_V;
        }

        inline Eigen::Matrix3d getDeltaR() const
        {
            return _delta_R;
        }


        // 测量状态关于传感器偏移的jacob
        inline Eigen::Matrix3d getJPBiasg() const
        {
            return _J_P_Biasg;
        }

        inline Eigen::Matrix3d getJPBiasa() const
        {
            return _J_P_Biasa;
        }

        inline Eigen::Matrix3d getJVBiasg() const
        {
            return _J_V_Biasg;
        }

        inline Eigen::Matrix3d getJVBiasa() const
        {
            return _J_V_Biasa;
        }

        inline Eigen::Matrix3d getJRBiasg() const
        {
            return _J_R_Biasg;
        }


        // 测量状态增量噪声不确定性
        inline Matrix9d getCovPVPhi() const
        {
            return _cov_P_V_Phi;
        }

        inline double getDeltaTime() const
        {
            return _delta_time;
        }

        static Matrix3d skew(const Vector3d &v)
        {
            return SO3::hat(v);
        }

        static Matrix3d Expmap(const Vector3d &v)
        {
            return SO3::exp(v).matrix();
        }


        // IMU预积分论文公式3，旋转向量的Jacobian
        // Right Jacobian for Log map in SO(3) - equation (10.86) in
        // G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
        static Matrix3d JacobianR(const Vector3d &w)
        {
            Matrix3d Jr = Matrix3d::Identity();
            double theta = w.norm();
            if (theta < 0.00001)
            {
                return Jr;
            }
            else
            {
                Vector3d k = w.normalized();
                Matrix3d K = skew(k);
                Jr = Matrix3d::Identity()
                     - (1 - cos(theta)) / theta * K
                     + (1 - sin(theta) / theta) * K * K;
            }

            return Jr;
        }

        static Matrix3d JacobianRInv(const Vector3d &w)
        {
            Matrix3d Jrinv = Matrix3d::Identity();
            double theta = w.norm();

            if (theta < 0.00001)
            {
                return Jrinv;
            }
            else
            {
                Vector3d k = w.normalized();
                Matrix3d K = SO3::hat(k);
                Jrinv = Matrix3d::Identity()
                        + 0.5 * SO3::hat(w)
                        + (1.0 - (1.0 + cos(theta)) * theta / (2.0 * sin(theta))) * K * K;
            }

            return Jrinv;

        }

        static Matrix3d JacobianL(const Vector3d &w)
        {
            return JacobianR(-w);
        }

        static Matrix3d JacobianLInv(const Vector3d &w)
        {
            return JacobianRInv(-w);
        }


        // 归一化四元数和旋转矩阵
        inline Quaterniond normalizeRotationQ(const Quaterniond &r)
        {
            Quaterniond _r(r);

            // 两个互为相反数的四元数表示同一个旋转
            if (_r.w() < 0)
            {
                _r.coeffs() *= -1;
            }

            return _r.normalized();
        }

        // 四元数归一化后转换为旋转矩阵
        inline Matrix3d normalizeRotationM(const Matrix3d &R)
        {
            Quaterniond qr(R);
            return normalizeRotationQ(qr).toRotationMatrix();
        }


    private:

        // IMU测量预积分值PVR，论文的公式35,36,37
        Eigen::Vector3d _delta_P;    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
        Eigen::Vector3d _delta_V;    // V_k+1 = V_k + R_k*a_k*dt
        Eigen::Matrix3d _delta_R;    // R_k+1 = R_k*exp(w_k*dt). note: Rwc, Rwc'=Rwc*[w_body]x

        // IMU漂移ba,bg更新后的预积分测量值校正量的Jacobian，补充材料A20
        Eigen::Matrix3d _J_P_Biasg;
        Eigen::Matrix3d _J_P_Biasa;
        Eigen::Matrix3d _J_V_Biasg;
        Eigen::Matrix3d _J_V_Biasa;
        Eigen::Matrix3d _J_R_Biasg;

        // IMU预积分测量噪声的不确定性, 补充材料的A9
        Matrix9d _cov_P_V_Phi;

        // 预积分IMU数据之间的时间间隔
        double _delta_time;

    };

}

#endif


