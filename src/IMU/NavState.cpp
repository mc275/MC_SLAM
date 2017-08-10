#include <NavState.h>

namespace ORB_SLAM2
{

    NavState::NavState()
    {
        _P.setZero();
        _V.setZero();

        _BiasGyr.setZero();
        _BiasAcc.setZero();

        _dBias_g.setZero();
        _dBias_a.setZero();

    }


    NavState::NavState(const NavState &_ns) :
            _P(_ns._P), _V(_ns._V), _R(_ns._R),
            _BiasGyr(_ns._BiasGyr), _BiasAcc(_ns._BiasAcc),
            _dBias_g(_ns._dBias_g), _dBias_a(_ns._dBias_a)
    {

    }


    // 更新所有状态
    // order in 'update_' = dP, dV, dPhi, dBiasGyr, dBiasAcc
    void NavState::IncSmall(Vector15d update)
    {
        // pi = pi + dpi,    pj = pj + dpj
        // vi = vi + dvi,       vj = vj + dvj
        // Ri = Ri*Exp(dphi_i), Rj = Rj*Exp(dphi_j)     IMU是坐标系变换,右乘？
        // delta_biasg_i = delta_biasg_i + dbgi,    delta_biasg_j = delta_biasg_j + dbgj
        // delta_biasa_i = delta_biasa_i + dbai,    delta_biasa_j = delta_biasa_j + dbaj

        Vector3d upd_P = update.segment<3>(0);          // 从0开始的三个元素
        Vector3d upd_V = update.segment<3>(3);
        Vector3d upd_Phi = update.segment<3>(6);
        Vector3d upd_dBg = update.segment<3>(9);
        Vector3d upd_dBa = update.segment<3>(12);

        // 位置更新
        _P += upd_P;

        // 速度更新
        _V += upd_V;

        // 旋转更新
        Sophus::SO3 dR = Sophus::SO3::exp(upd_Phi);
        _R = Get_R() * dR;

        // 偏移量更新
        _dBias_g += upd_dBg;
        _dBias_a += upd_dBa;

    }


    // 更新位姿状态
    void NavState::IncSmallPR(Vector6d dPR)
    {
        Vector3d upd_P = dPR.segment<3>(0);
        Vector3d upd_Phi = dPR.segment<3>(3);

        _P += upd_P;
        _R = _R * Sophus::SO3::exp(upd_Phi);
    }


    // 更新速度状态
    void NavState::IncSmallV(Vector3d dV)
    {
        _V += dV;
    }


    // 更新位姿和速度状态
    void NavState::IncSmallPVR(Vector9d updatePVR)
    {

        Vector3d upd_P = updatePVR.segment<3>(0);
        Vector3d upd_V = updatePVR.segment<3>(3);
        Vector3d upd_Phi = updatePVR.segment<3>(6);

        _P += upd_P;
        _V += upd_V;

        // Matrix3d R = Get_R().matrix();
        // 右乘更新
        Sophus::SO3 dR = Sophus::SO3::exp(upd_Phi);
        _R = Get_R() * dR;

    }


    // 更新传感器偏移量信息
    void NavState::IncSmallBias(Vector6d updateBias)
    {

        Vector3d upd_dBg = updateBias.segment<3>(0);
        Vector3d upd_dBa = updateBias.segment<3>(3);

        _dBias_g += upd_dBg;
        _dBias_a += upd_dBa;

    }


}
