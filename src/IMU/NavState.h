#ifndef NAVSTATE_H
#define NAVSTATE_H

#include "Eigen/Geometry"
#include "so3.h"

namespace ORB_SLAM2
{
    using namespace Eigen;
    typedef Matrix<double, 15, 1> Vector15d;
    typedef Matrix<double, 9, 1> Vector9d;
    typedef Matrix<double, 6, 1> Vector6d;
    
    class NavState
    {
        
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        NavState();
        NavState(const NavState &_ns);
        
       
        
        // 获取与设置运动参数
        Sophus::SO3 Get_R()             const
        {
            return _R;
        }
        
        Matrix3d Get_RotMatrix()        const
        {
            return _R.matrix();
        }
        
        Vector3d Get_P()                const
        {
            return _P;
        }
        
        Vector3d Get_V()                const
        {
            return _V;
        }
        
        void Set_Pos(const Vector3d &pos)
        {
            _P = pos;
        }
        
        void Set_Vel(const Vector3d &vel)
        {
            _V = vel;
        }
        
        void Set_Rot(const Matrix3d &rot)
        {
            _R = Sophus::SO3(rot);
        }
        
        void Set_Rot(const Sophus::SO3 &rot)
        {
            _R = rot;
        }
        
        
        
        // 获取与设置传感器偏移量
        Vector3d Get_BiasGyr() const
        {
            return _BiasGyr;
        }
        
        Vector3d Get_BiasAcc() const
        {
            return _BiasAcc;
        }
        
        void Set_BiasGyr(const Vector3d &bg)
        {
            _BiasGyr = bg;
        }
        
        void Set_BiasAcc(const Vector3d &ba)
        {
            _BiasAcc = ba;
        }
        
        
        
        // 获取与设置传感器偏移变化量
        Vector3d Get_dBias_Gyr() const
        {
            return _dBias_g;
        }
        
        Vector3d Get_dBias_Acc() const
        {
            return _dBias_a;
        }
        
        void Set_DeltaBiasGyr(const Vector3d &dbg)
        {
            _dBias_g = dbg;
        }
        
        void Set_DeltaBiasAcc(const Vector3d &dba)
        {
            _dBias_a = dba;
        }
        
        
        
        // 增量式更新状态
        // delta = [dP, dV, dPhi, dBa, dBg]
        void IncSmall(Vector15d delta);
        void IncSmallPVR(Vector9d dPVR);
        void IncSmallPR(Vector6d dPR);
        void IncSmallV(Vector3d dV);
        void IncSmallBias(Vector6d dBias);
        
        
    private:
        // 位置，速度，旋转
        Vector3d _P;            
        Vector3d _V;            
        Sophus::SO3 _R;         
        
        // 陀螺仪、加速度计偏移
        // 优化中保持不变
        Vector3d _BiasGyr;       
        Vector3d _BiasAcc;       
        
        // 陀螺仪、加速度计偏移增量
        // 优化中的更新量
        Vector3d _dBias_g;      
        Vector3d _dBias_a;      
        
    };
    
}



#endif


