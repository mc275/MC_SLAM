#include "IMU/IMUPreintegrator.h"

namespace ORB_SLAM2
{

    IMUPreintegrator::IMUPreintegrator(const IMUPreintegrator &pre) :
            _delta_P(pre._delta_P), _delta_V(pre._delta_V), _delta_R(pre._delta_R),
            _J_P_Biasg(pre._J_P_Biasg), _J_P_Biasa(pre._J_P_Biasa),
            _J_V_Biasg(pre._J_V_Biasg), _J_V_Biasa(pre._J_V_Biasa), _J_R_Biasg(pre._J_R_Biasg),
            _cov_P_V_Phi(pre._cov_P_V_Phi), _delta_time(pre._delta_time)
    {

    }


    IMUPreintegrator::IMUPreintegrator()
    {

        // 测量状态的增量
        _delta_P.setZero();
        _delta_V.setZero();
        _delta_R.setIdentity();

        // 测量状态的增量关于传感器偏移的jacob
        _J_P_Biasg.setZero();
        _J_P_Biasa.setZero();
        _J_V_Biasg.setZero();
        _J_V_Biasa.setZero();
        _J_R_Biasg.setZero();

        // 测量状态的增量噪声的不确定性
        _cov_P_V_Phi.setZero();

        _delta_time = 0;

    }


    void IMUPreintegrator::reset()
    {

        _delta_P.setZero();
        _delta_V.setZero();
        _delta_R.setIdentity();

        _J_P_Biasg.setZero();
        _J_P_Biasa.setZero();
        _J_V_Biasg.setZero();
        _J_V_Biasa.setZero();
        _J_R_Biasg.setZero();

        _cov_P_V_Phi.setZero();

        _delta_time = 0;

    }


    // 增量式的更新 测量状态的增量，jacob和噪声方差
    // acc = acc_measuremtn - bias_a
    // omega = gyro_measurement - bias_g
    // 以上两个测量值都是一次的测量量，不是从KFi->KFj的全部IMU测量值,计算的预积分是两个IMU数据间隔之间的预积分
    void IMUPreintegrator::update(const Vector3d &omega, const Vector3d &acc, const double &dt)
    {

        double dt2 = dt * dt;

        // 步骤1 计算IMU旋转预积分中本次的测量值
        Matrix3d dR = Expmap(omega * dt);
        Matrix3d Jr = JacobianR(omega * dt);            // paper formula 35 J_r_k

        // 步骤2 计算IMU预积分噪声的不确定性
        // 补充材料中的公式 A.7-A.9,其中误差矩阵顺序为RVP，此处顺序为PVR，
        Matrix3d I3x3 = Matrix3d::Identity();
        Matrix<double, 9, 9> A = Matrix<double, 9, 9>::Identity();

        A.block<3, 3>(6, 6) = dR.transpose();                     // block大小是3x3，起始位置(6,6)

        A.block<3, 3>(3, 6) = -_delta_R * skew(acc) * dt;

        A.block<3, 3>(0, 6) = -0.5 * _delta_R * skew(acc) * dt2;

        A.block<3, 3>(0, 3) = I3x3 * dt;

        Matrix<double, 9, 3> Bg = Matrix<double, 9, 3>::Zero();
        Bg.block<3, 3>(6, 0) = Jr * dt;

        Matrix<double, 9, 3> Ca = Matrix<double, 9, 3>::Zero();
        Ca.block<3, 3>(3, 0) = _delta_R * dt;
        Ca.block<3, 3>(0, 0) = 0.5 * _delta_R * dt2;

        _cov_P_V_Phi = A * _cov_P_V_Phi * A.transpose() +
                       Bg * IMUData::getGyrMeasCov() * Bg.transpose() +
                       Ca * IMUData::getAccMeasCov() * Ca.transpose();

        // 步骤3 IMU漂移ba,bg更新后的预积分测量值校正量中的Jacobian
        // 补充材料中的公式 A.20，做了一些推导变成迭代更新
        _J_P_Biasa += _J_V_Biasa * dt - 0.5 * _delta_R * dt2;
        _J_P_Biasg += _J_V_Biasg * dt - 0.5 * _delta_R * skew(acc) * _J_R_Biasg * dt2;
        _J_V_Biasa += -_delta_R * dt;
        _J_V_Biasg += -_delta_R * skew(acc) * _J_R_Biasg * dt;
        _J_R_Biasg = dR.transpose() * _J_R_Biasg - Jr * dt;

        // 步骤4 计算IMU预积分
        // 等号右侧表示本次两帧之间测量的预积分值,右乘更新
        _delta_P += _delta_V * dt + 0.5 * _delta_R * acc * dt2;
        _delta_V += _delta_R * acc * dt;
        _delta_R = normalizeRotationM(_delta_R * dR);

        _delta_time += dt;

    }


}