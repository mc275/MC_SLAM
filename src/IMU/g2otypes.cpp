#include <g2otypes.h>

namespace g2o 
{
    
    using namespace ORB_SLAM2;
    
    
    // EdgePRIDP class
    void EdgePRIDP::computeError()
    {
	const Vector3d Pi = computePc();
	_error = _measurement - cam_project();
    }
    
    // 根据顶点计算Pc
    Vector3d EdgePRIDP::computePc()
    {
	// 逆深度, 参考KF, CKF, IMU-Cam外参
	const VertexIDP *vIDP = static_cast<const VertexIDP *>(_vertices[0]);	
	const VertexNavStatePR *vPR0 = static_cast<const VertexNavStatePR *>(_vertices[1]);
	const VertexNavStatePR *vPRi = static_cast<const VertexNavStatePR *>(_vertices[2]);
	const VertexNavStatePR *vPRcb = static_cast<const VertexNavStatePR *>(_vertices[3]);
	
	const Matrix3d R0 = vPR0->estimate().Get_RotMatrix();
	const Vector3d t0 = vPR0->estimate().Get_P();
	const Matrix3d Ri = vPRi->estimate().Get_RotMatrix();
	const Vector3d ti = vPRi->estimate().Get_P();
	const Matrix3d Rcb = vPRcb->estimate().Get_RotMatrix();
	const Vector3d tcb = vPRcb->estimate().Get_P();
	
	// 参考帧中MP逆深度
	double rho = vIDP->estimate();
	
	if(rho < 1e-6)
	{
	    std::cerr<<"1. rho = "<< rho << ", rho < 1e-6, shouldn't" << std::endl;
	    rho = 1e-6;
	    setLevel(1);
	}
	
	// MP在参考帧相机坐标系下的坐标
	Vector3d P0;
	P0<< refnormxy[0], refnormxy[1], 1;
	double d = 1.0/rho;
	P0 *= d;
	
	// 将参考帧相机坐标系下的地图MP坐标,转换到CKF相机坐标系下
	// 整个过程涉及三次坐标转换 c0=>b0, b0=>bi, bi=>ci
	// c0=>b0: Tb0c0 = [Rb0c0|tb0c0], Rb0c0 = Rcb', tb0c0 = -Rcb'*tcb
	// b0=>bi: Tbib0 = [Rbib0|tbib0], Rbib0 = Ri'*R0, tbib0 = Rbib0*(t0-t1)
	// bi=>ci: Tc1b1 = [Rcibi|tcibi], Rcibi = Rcb,  tcibi = tcb
	// Pi = Tcibi*Tbib0*Tb0c0*P0
	// 从参考帧到CKF的旋转阵 Rcic0 = Rcibi*Rbiw*Rwb0*Rb0c0
	// 从参考帧到CKF的位移 tcic0 = tcb-Rcic0*tcb*Rcb*Ri'*(t0-ti)
	const Matrix3d Rcic0 = Rcb*Ri.transpose()*R0*Rcb.transpose();
	const Vector3d Pi = Rcic0*P0 + tcb-Rcic0*tcb+Rcb*Ri.transpose()*(t0-ti);
	return Pi;
	
    }
    
    void EdgePRIDP::linearizeOplus()
    {
	const VertexIDP *vIDP = static_cast<const VertexIDP *>(_vertices[0]);
	const VertexNavStatePR *vPR0 = static_cast<const VertexNavStatePR *>(_vertices[1]);
	const VertexNavStatePR *vPRi = static_cast<const VertexNavStatePR *>(_vertices[2]);
	const VertexNavStatePR *vPRcb = static_cast<const VertexNavStatePR *>(_vertices[3]);
	
	const Matrix3d R0 = vPR0->estimate().Get_RotMatrix();
	const Vector3d t0 = vPR0->estimate().Get_P();
	const Matrix3d Ri = vPRi->estimate().Get_RotMatrix();
	const Vector3d ti = vPRi->estimate().Get_P();
	const Matrix3d Rcb = vPRcb->estimate().Get_RotMatrix();
	const Vector3d tcb = vPRcb->estimate().Get_P();
	
	double rho = vIDP->estimate();
	if(rho<1e-6)
	{
	    std::cerr<<"1. rho = " << rho << ", rho<1e-6, shouldn't" << std::endl;
	    rho = 1e-6;
	    setLevel(1);
	}
	
	// 参考帧相机坐标系下坐标
	Vector3d P0;
	P0 << refnormxy[0], refnormxy[1], 1;
	double d = 1.0/rho;
	P0 *= d;
	
	// 当前关键帧相机坐标系下坐标
	const Matrix3d Rcic0 = Rcb*Ri.transpose()*R0*Rcb.transpose();
	const Vector3d Pi = Rcic0*P0 + tcb - Rcic0*tcb + Rcb*Ri.transpose()*(t0-ti);
	
	// 
	
	
	
	
	
	
	
	
	
	
    }

    
    


    
    
}


