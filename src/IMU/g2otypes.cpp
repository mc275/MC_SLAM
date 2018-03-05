/****************************************************************
 * 性质：hat(R*t) = R*hat(t)*R^T
 * 关于运动方程(IMU预积分观测方程)的残差对不同优化状态的Jacobain
 * 参考补充材料第2节
 ****************************************************************/

#include <g2otypes.h>

namespace g2o
{

    using namespace ORB_SLAM2;

    // 性质：hat(R*t) = R*hat(t)*R^T
    // class EdgePRIDP 
    // 误差:MP的投影像素位置误差
    void EdgePRIDP::computeError()
    {
        const Vector3d Pi = computePc();
        _error = _measurement - cam_project(Pi);
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

        if (rho < 1e-6)
        {
            std::cerr << "1. rho = " << rho << ", rho<1e-6, shouldn't" << std::endl;
            rho = 1e-6;
            setLevel(1);
        }

        // MP在参考帧相机坐标系下的坐标
        Vector3d P0;
        P0 << refnormxy[0], refnormxy[1], 1;
        double d = 1.0 / rho;
        P0 *= d;

        // 将参考帧相机坐标系下的地图MP坐标,转换到CKF相机坐标系下
        // 整个过程涉及三次坐标转换 c0=>b0, b0=>bi, bi=>ci
        // c0=>b0: Tb0c0 = [Rb0c0|tb0c0], Rb0c0 = Rcb', tb0c0 = -Rcb'*tcb
        // b0=>bi: Tbib0 = [Rbib0|tbib0], Rbib0 = Ri'*R0, tbib0 = Rbib0*(t0-t1)
        // bi=>ci: Tc1b1 = [Rcibi|tcibi], Rcibi = Rcb,  tcibi = tcb
        // Pi = Tcibi*Tbib0*Tb0c0*P0
        // 从参考帧到CKF的旋转阵 Rcic0 = Rcibi*Rbiw*Rwb0*Rb0c0
        // 从参考帧到CKF的位移 tcic0 = tcb-Rcic0*tcb*Rcb*Ri'*(t0-ti)
        const Matrix3d Rcic0 = Rcb * Ri.transpose() * R0 * Rcb.transpose();
        const Vector3d Pi = Rcic0 * P0 + tcb - Rcic0 * tcb + Rcb * Ri.transpose() * (t0 - ti);
        return Pi;

    }

    // 根据IMU运动方程,本代码中IMU预积分估计的pose为右乘更新,jacobian形式受到左右乘的影响
    void EdgePRIDP::linearizeOplus()
    {
        const VertexIDP *vIDP = static_cast<const VertexIDP *>(_vertices[0]);
        const VertexNavStatePR *vPR0 = static_cast<const VertexNavStatePR *>(_vertices[1]);
        const VertexNavStatePR *vPRi = static_cast<const VertexNavStatePR *>(_vertices[2]);
        const VertexNavStatePR *vPRcb = static_cast<const VertexNavStatePR *>(_vertices[3]);

        const Matrix3d R0 = vPR0->estimate().Get_RotMatrix();
        const Vector3d t0 = vPR0->estimate().Get_P();
        const Matrix3d Ri = vPRi->estimate().Get_RotMatrix();
        const Matrix3d RiT = Ri.transpose();
        const Vector3d ti = vPRi->estimate().Get_P();
        const Matrix3d Rcb = vPRcb->estimate().Get_RotMatrix();
        const Vector3d tcb = vPRcb->estimate().Get_P();

        double rho = vIDP->estimate();    // 逆深度
        if (rho < 1e-6)
        {
            std::cerr << "2. rho = " << rho << ", rho<1e-6, shouldn't" << std::endl;
            rho = 1e-6;
            setLevel(1);
        }

        // 参考帧相机坐标系下坐标
        Vector3d P0;
        P0 << refnormxy[0], refnormxy[1], 1;
        double d = 1.0 / rho;
        P0 *= d;

        // 当前关键帧相机坐标系下坐标
        const Matrix3d Rcic0 = Rcb * Ri.transpose() * R0 * Rcb.transpose();
        const Vector3d Pi = Rcic0 * P0 + tcb - Rcic0 * tcb + Rcb * Ri.transpose() * (t0 - ti);


        // error = obs-K*Pi
        // Vertex 0:rho(IDP)
        // J0 = -JPi*dPi/drho
        double x = Pi[0];
        double y = Pi[1];
        double z = Pi[2];


        Matrix<double, 2, 3> Maux;
        Maux.setZero();
        Maux(0, 0) = fx;
        Maux(0, 1) = 0;
        Maux(0, 2) = -x / z * fx;
        Maux(1, 0) = 0;
        Maux(1, 1) = fy;
        Maux(1, 2) = -y / z * fy;
        // Jpi = dKPi/dPi, 是相机投影方程对Pi的Jacob
        Matrix<double, 2, 3> Jpi = Maux / z;

        // Vertex 0: IDP
        Vector3d J_pi_rho = Rcic0 * (-d * P0);            // Jpi__rho= dPi/drho = -1/rho*P0
        _jacobianOplus[0] = -Jpi * J_pi_rho;            // 2x1

        // Vertex 1: PR0
        Matrix3d J_pi_t0 = Rcb * RiT;                // dPi/dt0
        Matrix3d J_pi_r0 = -Rcic0 * SO3::hat(P0 - tcb) * Rcb;
        Matrix<double, 3, 6> J_pi_pr0;
        // se3向量t在前
        J_pi_pr0.topLeftCorner(3, 3) = J_pi_t0;
        J_pi_pr0.topRightCorner(3, 3) = J_pi_r0;
        _jacobianOplus[1] = -Jpi * J_pi_pr0;            // 2x6

        // Vertex 2: PR1
        // Jacob = dPi/dT
        // hat(R*t) = R*hat(t)*R^T
        Matrix3d J_pi_ti = -Rcb * RiT;
        Vector3d taux = RiT * (R0 * Rcb.transpose() * (P0 - tcb) + t0 - ti);
        Matrix3d J_pi_ri = Rcb * SO3::hat(taux);
        Matrix<double, 3, 6> J_pi_pri;
        J_pi_pri.topLeftCorner(3, 3) = J_pi_ti;
        J_pi_pri.topRightCorner(3, 3) = J_pi_ri;
        _jacobianOplus[2] = -Jpi * J_pi_pri;            // 2x6

        // Vertex 3: PRcb
        Matrix3d J_pi_tcb = Matrix3d::Identity() - Rcic0;
        Matrix3d J_pi_rcb = -SO3::hat(Rcic0 * (P0 - tcb)) * Rcb
                            + Rcic0 * SO3::hat(P0 - tcb) * Rcb
                            - Rcb * SO3::hat(RiT * (t0 - ti));
        Matrix<double, 3, 6> J_pi_prcb;
        J_pi_prcb.topLeftCorner(3, 3) = J_pi_tcb;
        J_pi_prcb.topRightCorner(3, 3) = J_pi_rcb;

        _jacobianOplus[3] = -Jpi * J_pi_prcb;            // 2x6

    }


    // class EdgeNavStatePRV
    // VI-SLAM motion model error，也可以理解为IMU的观测方程
    void EdgeNavStatePRV::computeError()
    {
        const VertexNavStatePR *vPRi = static_cast<const VertexNavStatePR *>(_vertices[0]);
        const VertexNavStatePR *vPRj = static_cast<const VertexNavStatePR *>(_vertices[1]);
        const VertexNavStateV *vVi = static_cast<const VertexNavStateV *>(_vertices[2]);
        const VertexNavStateV *vVj = static_cast<const VertexNavStateV *>(_vertices[3]);
        const VertexNavStateBias *vBiasi = static_cast<const VertexNavStateBias *>(_vertices[4]);

        // 待优化变量
        // i时刻的pose, V, Bias
        const NavState &NSPRi = vPRi->estimate();
        const Vector3d Pi = NSPRi.Get_P();
        const Sophus::SO3 Ri = NSPRi.Get_R();

        const NavState &NSVi = vVi->estimate();
        const Vector3d Vi = NSVi.Get_V();

        const NavState &NSBiasi = vBiasi->estimate();
        const Vector3d dBgi = NSBiasi.Get_dBias_Gyr();
        const Vector3d dBai = NSBiasi.Get_dBias_Acc();

        // j时刻的pose, V
        const NavState &NSPRj = vPRj->estimate();
        const Vector3d Pj = NSPRj.Get_P();
        const Sophus::SO3 Rj = NSPRj.Get_R();

        const NavState &NSVj = vVj->estimate();
        const Vector3d Vj = NSVj.Get_V();

        // IMU预积分结果是测量值
        // 时间差, 位移差, 速度增量, 相对pose
        const IMUPreintegrator &M = _measurement;
        // 两关键帧之间的时间间隔
        const double dTij = M.getDeltaTime();
        const double dT2 = dTij * dTij;
        // 预积分测量值
        const Vector3d dPij = M.getDeltaP();
        const Vector3d dVij = M.getDeltaV();
        const Sophus::SO3 dRij = Sophus::SO3(M.getDeltaR());

        const Sophus::SO3 RiT = Ri.inverse();    // RiT = Riw

        // 预积分论文公式45
        // error_P
        const Vector3d rPij = RiT * (Pj - Pi - Vi * dTij - 0.5 * GravityVec * dT2)
                              - (dPij + M.getJPBiasg() * dBgi + M.getJPBiasa() * dBai);
        // error_v
        const Vector3d rVij = RiT * (Vj - Vi - GravityVec * dTij)
                              - (dVij + M.getJVBiasg() * dBgi + M.getJVBiasa() * dBai);
        // error_R
        const Sophus::SO3 dR_dbg = Sophus::SO3::exp(M.getJRBiasg() * dBgi);
        const Sophus::SO3 rRij = (dRij * dR_dbg).inverse() * RiT * Rj;
        const Vector3d rPhiij = rRij.log();

        Vector9d err;
        err.setZero();

        // 9 Dims error, PRV
        err.segment<3>(0) = rPij;
        err.segment<3>(3) = rPhiij;
        err.segment<3>(6) = rVij;

        _error = err;

    }

    void EdgeNavStatePRV::linearizeOplus()
    {

        const VertexNavStatePR *vPRi = static_cast<const VertexNavStatePR *>(_vertices[0]);
        const VertexNavStatePR *vPRj = static_cast<const VertexNavStatePR *>(_vertices[1]);
        const VertexNavStateV *vVi = static_cast<const VertexNavStateV *>(_vertices[2]);
        const VertexNavStateV *vVj = static_cast<const VertexNavStateV *>(_vertices[3]);
        const VertexNavStateBias *vBiasi = static_cast<const VertexNavStateBias *>(_vertices[4]);

        // i时刻视觉估计状态
        const NavState &NSPRi = vPRi->estimate();
        const Vector3d Pi = NSPRi.Get_P();
        const Matrix3d Ri = NSPRi.Get_RotMatrix();

        const NavState &NSVi = vVi->estimate();
        const Vector3d Vi = NSVi.Get_V();

        const NavState &NSBiasi = vBiasi->estimate();
        const Vector3d dBgi = NSBiasi.Get_dBias_Gyr();

        // j时刻的视觉估计状态
        const NavState &NSPRj = vPRj->estimate();
        const Vector3d Pj = NSPRj.Get_P();
        const Matrix3d Rj = NSPRj.Get_RotMatrix();

        const NavState &NSVj = vVj->estimate();
        const Vector3d Vj = NSVj.Get_V();

        // IMU预积分测量值
        const IMUPreintegrator &M = _measurement;
        const double dTij = M.getDeltaTime();
        const double dT2 = dTij * dTij;

        Matrix3d O3x3 = Matrix3d::Zero();
        Matrix3d RiT = Ri.transpose();
        Matrix3d RjT = Rj.transpose();
        Vector3d rPhiij = _error.segment<3>(3);
        // so3旋转矩阵的右jacobian的逆
        Matrix3d JrInv_rPhi = Sophus::SO3::JacobianRInv(rPhiij);
        // 预积分中的旋转角对陀螺仪偏移jacobian
        Matrix3d J_rPhi_dbg = M.getJRBiasg();

        // 1. 状态更新方式
        // pi = pi + dpi,    pj = pj + dpj
        // Ri = Ri*Exp(dphi_i), Rj = Rj*Exp(dphi_j)
        // vi = vi + dvi,       vj = vj + dvj
        // dBgi = dBgi + dbgi_update,    dBgj = dBgj + dbgj_update
        // dBai = dBai + dbai_update,    dBaj = dBaj + dbaj_update

        // 2. 9D误差向量的顺序PVR

        // 3. Jacobians
        // derr_P/dPR0, derr_P/dPR1, derr_P/dV0, derr_P/dV1, derr_P/dBias0
        // derr_R/dPR0, derr_R/dPR1, derr_R/dV0, derr_R/dV1, derr_R/dBias0
        // derr_V/dPR0, derr_V/dPR1, derr_V/dV0, derr_V/dV1, derr_V/dBias0

        // 4. 误差项
// 	const Vector3d rPij = RiT*(Pj-Pi-Vi*dTij - 0.5*GravityVec*dT2)
// 				- (dPij + M.getJPBiasa()*dBgi + M.getJPBiasa()*dBai);
// 	const Vector3d rVij = RiT*(Vj-Vi-GravityVec*dTij)
// 				- (dVij+M.getJVBiasg()*dBgi+M.getJVBiasa()*dBai);
// 	const Sophus::SO3 dR_dbg = Sophus::SO3::exp(M.getJPBiasg()*dBgi);
// 	const Sophus::SO3 rRij = (dRij * dR_dbg).inverse*RiT*Rj;
// 	const Vector3d rPhiij = rRij.log();

        // Jacobian求法见补充材料2.1节，并用到了论文中的公式2
        // 误差关于i时刻的vision PRV的jacob
        Matrix<double, 9, 6> JPRi;
        JPRi.setZero();

        // 误差项rPij关于顶点0的PR的jacobian
        JPRi.block<3, 3>(0, 0) = -RiT;                                // J_rPij_Pi
        JPRi.block<3, 3>(0, 3) = Sophus::SO3::hat(RiT * (Pj - Pi - Vi * dTij - 0.5 * GravityVec * dT2));    // J_rPij_Ri

        // Matrix3d ExprPhiijTrans = Sophus::SO3::exp(rPhiij).inverse().matrix();
        // Matrix3d JrBiasGCorr = Sophus::SO3::JacobianR(J_rPhi_dbg*dBgi);

        // 误差项rPhiij关于顶点0的PR的jacobian
        JPRi.block<3, 3>(3, 0) = O3x3;                // J_rPhiij_Pi
        JPRi.block<3, 3>(3, 3) = -JrInv_rPhi * RjT * Ri;        // J_rphiij_Ri

        // 误差项rVij关于顶点0的PR的jacobian
        JPRi.block<3, 3>(6, 0) = O3x3;                // J_rVij_Pi
        JPRi.block<3, 3>(6, 3) = Sophus::SO3::hat(RiT * (Vj - Vi - GravityVec * dTij));        // J_rVij_Ri

        // 误差项rPij,rPhiij,rVij关于顶点2的V的jacobian
        Matrix<double, 9, 3> JVi;
        JVi.setZero();
        JVi.block<3, 3>(0, 0) = -RiT * dTij;            // J_rPij_Vi
        JVi.block<3, 3>(3, 0) = O3x3;                // J_rphiij_Vi
        JVi.block<3, 3>(6, 0) = -RiT;                // J_rVij_Vi


        // 误差关于j时刻的vision PRV的jacobian
        Matrix<double, 9, 6> JPRj;
        JPRj.setZero();

        // 误差项rPij关于顶点1的PR的Jacobian
        JPRj.block<3, 3>(0, 0) = RiT;                // J_rPij_Pj
        JPRj.block<3, 3>(0, 3) = O3x3;                // J_rPhi_Rj

        // 误差项rPhij关于顶点1的PR的Jacobian
        JPRj.block<3, 3>(3, 0) = O3x3;                // J_rPhij_Pj
        JPRj.block<3, 3>(3, 3) = JrInv_rPhi;            // J_rPhij_Rj

        // 误差项rVij关于顶点1的PR的Jacobian
        JPRj.block<3, 3>(6, 0) = O3x3;                // J_rVij_Pj
        JPRj.block<3, 3>(6, 3) = O3x3;                // J_rVij_Rj

        // 误差项rPij, rPhiij, rVij关于顶点3的V的jacobiian
        Matrix<double, 9, 3> JVj;
        JVj.setZero();
        JVj.block<3, 3>(0, 0) = O3x3;                // J_rPij_Vj
        JVj.block<3, 3>(3, 0) = O3x3;                // J_rPhiij_Vj
        JVj.block<3, 3>(6, 0) = RiT;                // J_rVij_Vj


        // 误差关于IMU漂移变化量的jacobian
        Matrix<double, 9, 6> JBiasi;
        JBiasi.setZero();
        // 误差项rPij, rPhiij, rVij关于顶点4的Bgi,Bai的jacobian
        JBiasi.block<3, 3>(0, 0) = -M.getJPBiasg();        // J_rPij_dbgi
        JBiasi.block<3, 3>(0, 3) = -M.getJPBiasa();        // J_rPij_dbai

        Matrix3d ExprPhiijTrans = Sophus::SO3::exp(rPhiij).inverse().matrix();
        Matrix3d JrBiasGCorr = Sophus::SO3::JacobianR(J_rPhi_dbg * dBgi);
        JBiasi.block<3, 3>(3, 0) = -JrInv_rPhi * ExprPhiijTrans * JrBiasGCorr * J_rPhi_dbg;    //J_rPhiij_dbgi
        JBiasi.block<3, 3>(3, 3) = O3x3;                // J_rPhiij_dbai

        JBiasi.block<3, 3>(6, 0) = -M.getJVBiasg();        // J_rVij_dbgi
        JBiasi.block<3, 3>(6, 3) = -M.getJVBiasa();        // J_rVij_dbai

        _jacobianOplus[0] = JPRi;
        _jacobianOplus[1] = JPRj;
        _jacobianOplus[2] = JVi;
        _jacobianOplus[3] = JVj;
        _jacobianOplus[4] = JBiasi;

    }


    // class EdgeNavStatePRPointXYZ
    void EdgeNavStatePRPointXYZ::linearizeOplus()
    {

        const VertexSBAPointXYZ *vPoint = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
        const VertexNavStatePR *vNavState = static_cast<const VertexNavStatePR *>(_vertices[1]);

        const NavState &ns = vNavState->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();
        const Vector3d &Pw = vPoint->estimate();

        Matrix3d Rcb = Rbc.transpose();
        Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

        double x = Pc[0];
        double y = Pc[1];
        double z = Pc[2];

        // 相机投影模型jacobian
        Matrix<double, 2, 3> Maux;
        Maux.setZero();
        Maux(0, 0) = fx;
        Maux(0, 1) = 0;
        Maux(0, 2) = -x / z * fx;
        Maux(1, 0) = 0;
        Maux(1, 1) = fy;
        Maux(1, 2) = -y / z * fy;
        Matrix<double, 2, 3> Jpi = Maux / z;

        // error = obs - proj(Pc)
        // Pw = Pw+dPw
        // Rwb = Rwb*exp(dtheta)
        // Pwb = Pwb+dPwb

        // 顶点0, 误差对Pw的jacobian
        _jacobianOplusXi = -Jpi * Rcb * Rwb.transpose();

        // 顶点1, 误差对Pwb的jacobian
        Matrix<double, 2, 3> JdPwb = -Jpi * (-Rcb * Rwb.transpose());
        // 顶点1,误差对Rwb的jacobian
        Vector3d Paux = Rcb * Rwb.transpose() * (Pw - Pwb);
        Matrix<double, 2, 3> JdRwb = -Jpi * (Sophus::SO3::hat(Paux) * Rcb);

        Matrix<double, 2, 6> JNavState = Matrix<double, 2, 6>::Zero();
        JNavState.block<2, 3>(0, 0) = JdPwb;
        JNavState.block<2, 3>(0, 3) = JdRwb;

        _jacobianOplusXj = JNavState;

    }


    // class EdgeNavStatePRPointXYZOnlyPose
    void EdgeNavStatePRPointXYZOnlyPose::linearizeOplus()
    {
        const VertexNavStatePR *vNSPR = static_cast<const VertexNavStatePR *>(_vertices[0]);

        const NavState &ns = vNSPR->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();

        Matrix3d Rcb = Rbc.transpose();
        Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

        double x = Pc[0];
        double y = Pc[1];
        double z = Pc[2];

        Matrix<double, 2, 3> Maux;
        Maux.setZero();
        Maux(0, 0) = fx;
        Maux(0, 1) = 0;
        Maux(0, 2) = -x / z * fx;
        Maux(1, 0) = 0;
        Maux(1, 1) = fy;
        Maux(1, 2) = -y / z * fy;
        Matrix<double, 2, 3> Jpi = Maux / z;

        // err = obs - proj(Pc)
        // 顶点, 误差对Pwb的jacobian
        Matrix<double, 2, 3> JdPwb = -Jpi * (-Rcb * Rwb.transpose());
        // 顶点, 误差对Rwb的jacobian
        Vector3d Paux = Rcb * Rwb.transpose() * (Pw - Pwb);
        Matrix<double, 2, 3> JdRwb = -Jpi * (Sophus::SO3::hat(Paux) * Rcb);

        // 更新顶点估计量顺序, dP, dPhi
        Matrix<double, 2, 6> JNavState = Matrix<double, 2, 6>::Zero();
        JNavState.block<2, 3>(0, 0) = JdPwb;
        JNavState.block<2, 3>(0, 3) = JdRwb;


        _jacobianOplusXi = JNavState;

    }


    // class EdgeNavStatePriorPRVBias
    void EdgeNavStatePriorPRVBias::computeError()
    {

        const VertexNavStatePR *vNSPR = static_cast<const VertexNavStatePR *>(_vertices[0]);
        const VertexNavStateV *vNSV = static_cast<const VertexNavStateV *>(_vertices[1]);
        const VertexNavStateBias *vNSBias = static_cast<const VertexNavStateBias *>(_vertices[2]);

        const NavState &nsPRest = vNSPR->estimate();
        const NavState &nsVest = vNSV->estimate();
        const NavState &nsBiasest = vNSBias->estimate();

        // 先验信息
        const NavState &nsprior = _measurement;

        // P V R dg+dgb ba+dba
        Vector15d err = Vector15d::Zero();

        // eP = P_prior - p_est
        err.segment<3>(0) = nsprior.Get_P() - nsPRest.Get_P();

        // eR = log(R_prior^-1*Rest)
        err.segment<3>(3) = (nsprior.Get_R().inverse() * nsPRest.Get_R()).log();

        // eV = V_prior - Vest
        err.segment<3>(6) = nsprior.Get_V() - nsVest.Get_V();

        // ebg = (bg_prior+dbg_prior) - (bg+dbg)
        err.segment<3>(9) = (nsprior.Get_BiasGyr() + nsprior.Get_dBias_Gyr()) -
                            (nsBiasest.Get_BiasGyr() + nsBiasest.Get_dBias_Gyr());

        // eba = (ba_prior+dba_prior) - (ba+dba)
        err.segment<3>(12) = (nsprior.Get_BiasAcc() + nsprior.Get_dBias_Acc()) -
                             (nsBiasest.Get_BiasAcc() + nsBiasest.Get_dBias_Acc());

        _error = err;

    }

    void EdgeNavStatePriorPRVBias::linearizeOplus()
    {

        // 顶点0, error对PRest的jacobian
        Matrix<double, 15, 6> _jacobianOplusPR = Matrix<double, 15, 6>::Zero();
        _jacobianOplusPR.block<3, 3>(0, 0) = -Matrix3d::Identity();                // J_errP_Pest
        _jacobianOplusPR.block<3, 3>(3, 3) = Sophus::SO3::JacobianRInv(_error.segment<3>(3));    // J_errR_Rest

        Matrix<double, 15, 3> _jacobianOplusV = Matrix<double, 15, 3>::Zero();
        _jacobianOplusV.block<3, 3>(6, 6) = -Matrix3d::Identity();                // J_errV_Vest

        Matrix<double, 15, 6> _jacobianOplusBias = Matrix<double, 15, 6>::Zero();
        _jacobianOplusBias.block<3, 3>(9, 0) = -Matrix3d::Identity();                // J_errbg_dBg
        _jacobianOplusBias.block<3, 3>(12, 3) = -Matrix3d::Identity();                // J_errba_dBa

        _jacobianOplus[0] = _jacobianOplusPR;
        _jacobianOplus[1] = _jacobianOplusV;
        _jacobianOplus[2] = _jacobianOplusBias;

    }


    // class EdgeNavStatePVR
    void EdgeNavStatePVR::computeError()
    {

        const VertexNavStatePVR *vPVRi = static_cast<const VertexNavStatePVR *>(_vertices[0]);
        const VertexNavStatePVR *vPVRj = static_cast<const VertexNavStatePVR *>(_vertices[1]);
        const VertexNavStateBias *vBiasi = static_cast<const VertexNavStateBias *>(_vertices[2]);

        // 顶点0, i时刻视觉估计的PVR
        const NavState &NSPVRi = vPVRi->estimate();
        Vector3d Pi = NSPVRi.Get_P();
        Vector3d Vi = NSPVRi.Get_V();
        Sophus::SO3 Ri = NSPVRi.Get_R();

        // 顶点2, i时刻IMU漂移
        const NavState &NSBiasi = vBiasi->estimate();
        Vector3d dBgi = NSBiasi.Get_dBias_Gyr();
        Vector3d dBai = NSBiasi.Get_dBias_Acc();

        // 顶点1, j时刻视觉估计的PVR
        const NavState &NSPVRj = vPVRj->estimate();
        Vector3d Pj = NSPVRj.Get_P();
        Vector3d Vj = NSPVRj.Get_V();
        Sophus::SO3 Rj = NSPVRj.Get_R();

        // IMU预积分后的测量值
        const IMUPreintegrator &M = _measurement;
        double dTij = M.getDeltaTime();
        double dT2 = dTij * dTij;
        Vector3d dPij = M.getDeltaP();
        Vector3d dVij = M.getDeltaV();
        Sophus::SO3 dRij = Sophus::SO3(M.getDeltaR());

        Sophus::SO3 RiT = Ri.inverse();

        // 位置误差
        Vector3d rPij = RiT * (Pj - Pi - Vi * dTij - 0.5 * GravityVec * dT2)
                        - (dPij + M.getJPBiasg() * dBgi + M.getJPBiasa() * dBai);

        // 速度误差
        Vector3d rVij = RiT * (Vj - Vi - GravityVec * dTij)
                        - (dVij + M.getJVBiasg() * dBgi + M.getJVBiasa() * dBai);

        // 旋转误差
        Sophus::SO3 dR_dbg = Sophus::SO3::exp(M.getJRBiasg() * dBgi);
        Sophus::SO3 rRij = (dRij * dR_dbg).inverse() * RiT * Rj;
        Vector3d rPhiij = rRij.log();

        Vector9d err;
        err.setZero();

        err.segment<3>(0) = rPij;
        err.segment<3>(3) = rVij;
        err.segment<3>(6) = rPhiij;

        _error = err;

    }

    void EdgeNavStatePVR::linearizeOplus()
    {

        const VertexNavStatePVR *vPVRi = static_cast<const VertexNavStatePVR *>(_vertices[0]);
        const VertexNavStatePVR *vPVRj = static_cast<const VertexNavStatePVR *>(_vertices[1]);
        const VertexNavStateBias *vBiasi = static_cast<const VertexNavStateBias *>(_vertices[2]);

        // 顶点0状态
        const NavState &NSPVRi = vPVRi->estimate();
        Vector3d Pi = NSPVRi.Get_P();
        Vector3d Vi = NSPVRi.Get_V();
        Matrix3d Ri = NSPVRi.Get_RotMatrix();

        // 顶点2状态
        const NavState &NSBiasi = vBiasi->estimate();
        Vector3d dBgi = NSBiasi.Get_dBias_Gyr();

        // 顶点1状态
        const NavState &NSPVRj = vPVRj->estimate();
        Vector3d Pj = NSPVRj.Get_P();
        Vector3d Vj = NSPVRj.Get_V();
        Matrix3d Rj = NSPVRj.Get_RotMatrix();

        // IMU预积分后的测量值
        const IMUPreintegrator &M = _measurement;
        double dTij = M.getDeltaTime();
        double dT2 = dTij * dTij;

        // 临时变量
        Matrix3d O3x3 = Matrix3d::Zero();
        Matrix3d RiT = Ri.transpose();
        Matrix3d RjT = Rj.transpose();
        Vector3d rPhiij = _error.segment<3>(6);
        // 旋转误差的右逆jacobain
        Matrix3d JrInv_rPhi = Sophus::SO3::JacobianRInv(rPhiij);
        // 预积分的旋转角对dBg的jacobian
        Matrix3d J_rPhi_dbg = M.getJRBiasg();

        // 误差jacobian与误差,顶点顺序有关
        // error order: rP, rV, rR
        // Vertex order: P V R
        // 误差jacobian顺序为
        // J_rP_p, J_rP_v, J_rP_R
        // J_rV_p, J_rV_v, J_rV_R
        // J_rR_p, J_rR_v, J_rR_R

        /*顶点0的估计量PVR的jacobian*/
        Matrix<double, 9, 9> JPVRi;
        JPVRi.setZero();

        // 位置误差rP对P V R的jacobian
        JPVRi.block<3, 3>(0, 0) = -RiT;        // J_rP_Pi
        JPVRi.block<3, 3>(0, 3) = -RiT * dTij;    // J_rP_vi
        JPVRi.block<3, 3>(0, 6) = Sophus::SO3::hat(RiT * (Pj - Pi - Vi * dTij - 0.5 * GravityVec * dT2));    // J_rP_Ri

        // 速度误差rV对P V R的jacobaiin
        JPVRi.block<3, 3>(3, 0) = O3x3;        // J_rV_Pi
        JPVRi.block<3, 3>(3, 3) = -RiT;        // J_rV_Vi
        JPVRi.block<3, 3>(3, 6) = Sophus::SO3::hat(RiT * (Vj - Vi - GravityVec * dTij));    // J_rV_Ri

        // 旋转误差rR对P V R的jacobian
// 	Matrix3d ExprPhiijTrans = Sophus::SO3::exp(rPhiij).inverse().matrix();
// 	Matrix3d JrBiasGCorr = Sophus::SO3::JacobianR(J_rPhi_dbg*dBgi);
        JPVRi.block<3, 3>(6, 0) = O3x3;        // J_rR_Pi
        JPVRi.block<3, 3>(6, 3) = O3x3;        // J_rR_Vi
        JPVRi.block<3, 3>(6, 6) = -JrInv_rPhi * RjT * Ri;    // J_rR_Ri



        /*顶点1的估计量为PVR的jacobian*/
        Matrix<double, 9, 9> JPVRj;
        JPVRj.setZero();

        // 位置误差rP对P V R的jacobian
        JPVRj.block<3, 3>(0, 0) = RiT;        // J_rP_Pj
        JPVRj.block<3, 3>(0, 3) = O3x3;        // J_rP_Vj
        JPVRj.block<3, 3>(0, 6) = O3x3;        // J_rP_Rj

        // 速度误差rV对P V R的jacobian
        JPVRj.block<3, 3>(3, 0) = O3x3;        // J_rV_Pj
        JPVRj.block<3, 3>(3, 3) = RiT;        // J_rV_Vj
        JPVRj.block<3, 3>(3, 6) = O3x3;        // J_rV_Rj

        // 旋转误差rR对P V R的jacobian
        JPVRj.block<3, 3>(6, 0) = O3x3;        // J_rR_Pj
        JPVRj.block<3, 3>(6, 3) = O3x3;        // J_rR_Vj
        JPVRj.block<3, 3>(6, 6) = JrInv_rPhi;    // J_rR_Rj



        /*顶点2的估计量为dBgi,dBai的jacobian*/
        Matrix<double, 9, 6> JBiasi;
        JBiasi.setZero();
        Matrix3d ExprPhiijTrans = Sophus::SO3::exp(rPhiij).inverse().matrix();
        Matrix3d JrBiasGCorr = Sophus::SO3::JacobianR(J_rPhi_dbg * dBgi);

        // 位置误差rP对dBgi,dBai的jacobian
        JBiasi.block<3, 3>(0, 0) = -M.getJPBiasg();    // J_rP_dBgi
        JBiasi.block<3, 3>(0, 3) = -M.getJPBiasa();    // J_rP_dBai

        // 速度误差rV对dBgi,dBai的jacobian
        JBiasi.block<3, 3>(3, 0) = -M.getJVBiasg();    // J_rV_dBgi
        JBiasi.block<3, 3>(3, 3) = -M.getJVBiasa();    // J_rV_dBai

        // 旋转误差rR对dBgi,dBai的jacobian
        JBiasi.block<3, 3>(6, 0) = -JrInv_rPhi * ExprPhiijTrans * JrBiasGCorr * J_rPhi_dbg;    // J_rR_dBgi
        JBiasi.block<3, 3>(6, 3) = O3x3;                            // J_rR_dBai

        _jacobianOplus[0] = JPVRi;
        _jacobianOplus[1] = JPVRj;
        _jacobianOplus[2] = JBiasi;

    }


    // class EdgeNavStateBias
    void EdgeNavStateBias::computeError()
    {
        const VertexNavStateBias *vBiasi = static_cast<const VertexNavStateBias *>(_vertices[0]);
        const VertexNavStateBias *vBiasj = static_cast<const VertexNavStateBias *>(_vertices[1]);

        const NavState &NSi = vBiasi->estimate();
        const NavState &NSj = vBiasj->estimate();

        // IMU preintegration on manifold, Foster paper
        // 陀螺仪漂移误差
        Vector3d rBiasG = (NSj.Get_BiasGyr() + NSj.Get_dBias_Gyr())
                          - (NSi.Get_BiasGyr() + NSi.Get_dBias_Gyr());
        // 加速度计漂移误差
        Vector3d rBiasA = (NSj.Get_BiasAcc() + NSj.Get_dBias_Acc())
                          - (NSi.Get_BiasAcc() + NSi.Get_dBias_Acc());

        Vector6d err;
        err.setZero();

        err.segment<3>(0) = rBiasG;
        err.segment<3>(3) = rBiasA;
        _error = err;

    }

    void EdgeNavStateBias::linearizeOplus()
    {
        // 顶点0: 误差rBiasG, rBiasA关于dBiasG, dBiasA的jacobian
        _jacobianOplusXi = -Matrix<double, 6, 6>::Identity();
        _jacobianOplusXj = Matrix<double, 6, 6>::Identity();

    }


    // class EdgeNavStatePVRPointXYZ
    void EdgeNavStatePVRPointXYZ::linearizeOplus()
    {
        const VertexSBAPointXYZ *vPoint = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
        const VertexNavStatePVR *vNavState = static_cast<const VertexNavStatePVR *>(_vertices[1]);

        const NavState &ns = vNavState->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();
        const Vector3d &Pw = vPoint->estimate();

        Matrix3d Rcb = Rbc.transpose();
        Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

        double x = Pc[0];
        double y = Pc[1];
        double z = Pc[2];

        // 相机投影模型
        Matrix<double, 2, 3> Maux;
        Maux.setZero();
        Maux(0, 0) = fx;
        Maux(0, 1) = 0;
        Maux(0, 2) = -x / z * fx;
        Maux(1, 0) = 0;
        Maux(1, 1) = fy;
        Maux(1, 2) = -y / z * fy;
        Matrix<double, 2, 3> Jpi = Maux / z;

        // error = obs - proj(Pc)

        // 顶点0, 估计量为Pw
        // 误差对Pw的jacobain
        _jacobianOplusXi = -Jpi * Rcb * Rwb.transpose();

        // 顶点1, 估计量为PVR
        // 误差对Pwb的jacobian
        Matrix<double, 2, 3> JdPwb = -Jpi * (-Rcb * Rwb.transpose());

        // 误差对V的jacobian为0

        // 误差对Rwb的jacobian
        Vector3d Paux = Rcb * Rwb.transpose() * (Pw - Pwb);
        Matrix<double, 2, 3> JdRwb = -Jpi * (Sophus::SO3::hat(Paux) * Rcb);

        Matrix<double, 2, 9> JNavState = Matrix<double, 2, 9>::Zero();
        JNavState.block<2, 3>(0, 0) = JdPwb;
        JNavState.block<2, 3>(0, 6) = JdRwb;

        _jacobianOplusXj = JNavState;

    }


    // class EdgeNavStatePVRPointXYZOnlyPose
    void EdgeNavStatePVRPointXYZOnlyPose::linearizeOplus()
    {
        const VertexNavStatePVR *vNSPVR = static_cast<const VertexNavStatePVR *>(_vertices[0]);

        const NavState &ns = vNSPVR->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();

        Matrix3d Rcb = Rbc.transpose();
        Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

        double x = Pc[0];
        double y = Pc[1];
        double z = Pc[2];

        // 相机投影对Pc的jacobian
        Matrix<double, 2, 3> Maux;
        Maux.setZero();
        Maux(0, 0) = fx;
        Maux(0, 1) = 0;
        Maux(0, 2) = -x / z * fx;
        Maux(1, 0) = 0;
        Maux(1, 1) = fy;
        Maux(1, 2) = -y / z * fy;
        Matrix<double, 2, 3> Jpi = Maux / z;

        // error = obs - proj(Pc)

        // 误差对Pwb的jacobian
        Matrix<double, 2, 3> JdPwb = -Jpi * (-Rcb * Rwb.transpose());

        // 误差对V的jacobian为0

        // 误差对Rwb的jacobian
        Vector3d Paux = Rcb * Rwb.transpose() * (Pw - Pwb);
        Matrix<double, 2, 3> JdRwb = -Jpi * (Sophus::SO3::hat(Paux) * Rcb);

        Matrix<double, 2, 9> JNavState = Matrix<double, 2, 9>::Zero();
        JNavState.block<2, 3>(0, 0) = JdPwb;
        JNavState.block<2, 3>(0, 6) = JdRwb;

        _jacobianOplusXi = JNavState;

    }


    // class EdgeNavStatePriorPRVBias
    void EdgeNavStatePriorPVRBias::computeError()
    {

        const VertexNavStatePVR *vNSPVR = static_cast<const VertexNavStatePVR *>(_vertices[0]);
        const VertexNavStateBias *vNSBias = static_cast<const VertexNavStateBias *>(_vertices[1]);

        const NavState &nsPVRest = vNSPVR->estimate();
        const NavState &nsBiasest = vNSBias->estimate();
        const NavState &nsprior = _measurement;

        // 误差, 顺序为P,V,R,bg+dbg, ba+dba
        Vector15d err = Vector15d::Zero();

        // 位置误差
        err.segment<3>(0) = nsprior.Get_P() - nsPVRest.Get_P();

        // 速度误差
        err.segment<3>(3) = nsprior.Get_V() - nsPVRest.Get_V();

        // 旋转误差
        err.segment<3>(6) = (nsprior.Get_R().inverse() * nsPVRest.Get_R()).log();

        // 陀螺仪偏移增量误差
        err.segment<3>(9) = (nsprior.Get_BiasGyr() + nsprior.Get_dBias_Gyr()) -
                            (nsBiasest.Get_BiasGyr() + nsBiasest.Get_dBias_Gyr());

        // 加速度计偏移增量误差
        err.segment<3>(12) = (nsprior.Get_BiasAcc() + nsprior.Get_dBias_Acc()) -
                             (nsBiasest.Get_BiasAcc() + nsBiasest.Get_dBias_Acc());

        _error = err;

    }

    void EdgeNavStatePriorPVRBias::linearizeOplus()
    {
        // 顶点0
        _jacobianOplusXi = Matrix<double, 15, 9>::Zero();
        // 位置误差对位置的jacobian,(对速度和旋转的jacobian=0)
        _jacobianOplusXi.block<3, 3>(0, 0) = -Matrix3d::Identity();
        // 速度误差对速度的jacobian,(对位置和旋转的jacobian=0)
        _jacobianOplusXi.block<3, 3>(3, 3) = -Matrix3d::Identity();
        // 旋转误差对旋转的jacobian, (对位置和速度的jacobian=0)
        _jacobianOplusXi.block<3, 3>(6, 6) = Sophus::SO3::JacobianRInv(_error.segment<3>(6));

        // 顶点1
        _jacobianOplusXj = Matrix<double, 15, 6>::Zero();
        // 陀螺仪误差对陀螺仪误差增量的jacobain, 其他都是0
        _jacobianOplusXj.block<3, 3>(9, 0) = -Matrix3d::Identity();
        // 陀螺仪误差对陀螺仪误差增量的jacobain, 其他都是0
        _jacobianOplusXj.block<3, 3>(12, 3) = -Matrix3d::Identity();

    }


    // class EdgeNavStatePrior
    void EdgeNavStatePrior::computeError()
    {

        const VertexNavState *v = static_cast<const VertexNavState *>(_vertices[0]);
        const NavState &nsest = v->estimate();
        const NavState &nsprior = _measurement;

        // P V R bg+dbg ba+dba
        Vector15d err = Vector15d::Zero();

        // 位置误差
        err.segment<3>(0) = nsprior.Get_P() - nsest.Get_P();

        // 速度误差
        err.segment<3>(3) = nsprior.Get_V() - nsest.Get_V();

        // 旋转误差
        err.segment<3>(6) = (nsprior.Get_R().inverse() * nsest.Get_R()).log();

        // 陀螺仪漂移误差
        err.segment<3>(9) =
                (nsprior.Get_BiasGyr() + nsprior.Get_dBias_Gyr()) - (nsest.Get_BiasGyr() + nsest.Get_dBias_Gyr());

        // 加速度计漂移误差
        err.segment<3>(12) =
                (nsprior.Get_BiasAcc() + nsprior.Get_dBias_Acc()) - (nsest.Get_BiasAcc() + nsest.Get_dBias_Acc());

        _error = err;

    }

    void EdgeNavStatePrior::linearizeOplus()
    {

        _jacobianOplusXi.setZero();
        // 位置误差对P的jacobain, 其他都是0
        _jacobianOplusXi.block<3, 3>(0, 0) = -Matrix3d::Identity();

        // 速度误差对V的jacobain, 其他都是0
        _jacobianOplusXi.block<3, 3>(3, 3) = -Matrix3d::Identity();

        // 旋转误差对R的jacobain, 其他都是0
        _jacobianOplusXi.block<3, 3>(6, 6) = Sophus::SO3::JacobianRInv(_error.segment<3>(6));

        // 陀螺仪漂移误差对dBg的jacobain, 其他都是0
        _jacobianOplusXi.block<3, 3>(9, 9) = -Matrix3d::Identity();

        // 加速度漂移误差对dBa的jacobain, 其他都是0
        _jacobianOplusXi.block<3, 3>(12, 12) = -Matrix3d::Identity();

    }


    // class VertexNavState
    VertexNavState::VertexNavState() : BaseVertex<15, NavState>()
    {

    }

    bool VertexNavState::read(std::istream &is)
    {
        return true;
    }

    bool VertexNavState::write(std::ostream &os) const
    {
        return true;
    }

    void VertexNavState::oplusImpl(const double *update_)
    {
        Eigen::Map<const Vector15d> update(update_);
        _estimate.IncSmall(update);

    }


    // class EdgeNavState
    EdgeNavState::EdgeNavState() : BaseBinaryEdge<15, IMUPreintegrator, VertexNavState, VertexNavState>()
    {


    }

    bool EdgeNavState::read(std::istream &is)
    {
        return true;
    }

    bool EdgeNavState::write(std::ostream &os) const
    {
        return true;
    }

    void EdgeNavState::computeError()
    {
        const VertexNavState *vi = static_cast<const VertexNavState *>(_vertices[0]);
        const VertexNavState *vj = static_cast<const VertexNavState *>(_vertices[1]);

        // 顶点i的状态
        const NavState &NSi = vi->estimate();
        Vector3d Pi = NSi.Get_P();
        Vector3d Vi = NSi.Get_V();
        Sophus::SO3 Ri = NSi.Get_R();
        Vector3d dBgi = NSi.Get_dBias_Gyr();
        Vector3d dBai = NSi.Get_dBias_Acc();

        // 顶点j的状态
        const NavState &NSj = vj->estimate();
        Vector3d Pj = NSj.Get_P();
        Vector3d Vj = NSj.Get_V();
        Sophus::SO3 Rj = NSj.Get_R();

        // IMU预积分的测量值
        const IMUPreintegrator &M = _measurement;
        double dTij = M.getDeltaTime();
        double dT2 = dTij * dTij;
        Vector3d dPij = M.getDeltaP();
        Vector3d dVij = M.getDeltaV();
        Sophus::SO3 dRij = Sophus::SO3(M.getDeltaR());

        Sophus::SO3 RiT = Ri.inverse();


        // 位置误差
        Vector3d rPij = RiT * (Pj - Pi - Vi * dTij - 0.5 * GravityVec * dT2)
                        - (dPij + M.getJPBiasg() * dBgi + M.getJPBiasa() * dBai);

        // 速度误差
        Vector3d rVij = RiT * (Vj - Vi - GravityVec * dTij)
                        - (dVij + M.getJVBiasg() * dBgi + M.getJVBiasa() * dBai);

        // 旋转误差
        Sophus::SO3 dR_dbg = Sophus::SO3::exp(M.getJRBiasg() * dBgi);
        Sophus::SO3 rRij = (dRij * dR_dbg).inverse() * RiT * Rj;
        Vector3d rPhiij = rRij.log();

        // 陀螺仪漂移误差
        Vector3d rBiasG = (NSj.Get_BiasGyr() + NSj.Get_dBias_Gyr())
                          - (NSi.Get_BiasGyr() + NSi.Get_dBias_Gyr());

        // 加速度计漂移误差
        Vector3d rBiasA = (NSj.Get_BiasAcc() + NSj.Get_dBias_Acc())
                          - (NSi.Get_BiasAcc() + NSi.Get_dBias_Acc());


        Vector15d err;
        err.setZero();

        err.segment<3>(0) = rPij;
        err.segment<3>(3) = rVij;
        err.segment<3>(6) = rPhiij;
        err.segment<3>(9) = rBiasG;
        err.segment<3>(12) = rBiasA;

        _error = err;

    }

    // 运动模型jacobian
    void EdgeNavState::linearizeOplus()
    {

        const VertexNavState *vi = static_cast<const VertexNavState *>(_vertices[0]);
        const VertexNavState *vj = static_cast<const VertexNavState *>(_vertices[1]);

        // 顶点0状态
        const NavState &NSi = vi->estimate();
        Vector3d Pi = NSi.Get_P();
        Vector3d Vi = NSi.Get_V();
        Matrix3d Ri = NSi.Get_RotMatrix();
        Vector3d dBgi = NSi.Get_dBias_Gyr();

        // 顶点1状态
        const NavState &NSj = vj->estimate();
        Vector3d Pj = NSj.Get_P();
        Vector3d Vj = NSj.Get_V();
        Matrix3d Rj = NSj.Get_RotMatrix();

        // IMU预积分后的测量值
        const IMUPreintegrator &M = _measurement;
        double dTij = M.getDeltaTime();
        double dT2 = dTij * dTij;

        Matrix3d I3x3 = Matrix3d::Identity();
        Matrix3d O3x3 = Matrix3d::Zero();
        Matrix3d RiT = Ri.transpose();
        Matrix3d RjT = Rj.transpose();
        Vector3d rPhiij = _error.segment<3>(6);
        Matrix3d JrInv_rPhi = Sophus::SO3::JacobianRInv(rPhiij);
        Matrix3d J_rPhi_dbg = M.getJRBiasg();

        /*顶点0 估计量 P V R dBg dBa*/
        _jacobianOplusXi.setZero();

        // 位置误差对P V R dBg dBa的jacobian
        _jacobianOplusXi.block<3, 3>(0, 0) = -RiT;        // J_rP_Pi
        _jacobianOplusXi.block<3, 3>(0, 3) = -RiT * dTij;        // J_rP_Vi
        _jacobianOplusXi.block<3, 3>(0, 6) = Sophus::SO3::hat(
                RiT * (Pj - Pi - Vi * dTij - 0.5 * GravityVec * dT2));    // J_rP_Ri
        _jacobianOplusXi.block<3, 3>(0, 9) = -M.getJPBiasg();    // J_rP_dBgi
        _jacobianOplusXi.block<3, 3>(0, 12) = -M.getJPBiasa();    // J_rP_dBai

        // 速度误差对P V R dBg dBa的jacobian
        _jacobianOplusXi.block<3, 3>(3, 0) = O3x3;        // J_rV_Pi
        _jacobianOplusXi.block<3, 3>(3, 3) = -RiT;        // J_rV_Vi
        _jacobianOplusXi.block<3, 3>(3, 6) = Sophus::SO3::hat(RiT * (Vj - Vi - GravityVec * dTij));    // J_rV_Ri
        _jacobianOplusXi.block<3, 3>(3, 9) = -M.getJVBiasg();    // J_rV_dBgi
        _jacobianOplusXi.block<3, 3>(3, 12) = -M.getJVBiasa();    // J_rV_dBai

        // 旋转误差对P V R dBg dBa的jacobian
        Matrix3d ExprPhiijTrans = Sophus::SO3::exp(rPhiij).inverse().matrix();
        Matrix3d JrBiasGCorr = Sophus::SO3::JacobianR(J_rPhi_dbg * dBgi);
        _jacobianOplusXi.block<3, 3>(6, 0) = O3x3;        // J_rR_Pi
        _jacobianOplusXi.block<3, 3>(6, 3) = O3x3;        // J_rR_Vi
        _jacobianOplusXi.block<3, 3>(6, 6) = -JrInv_rPhi * RjT * Ri;    // J_rR_Ri
        _jacobianOplusXi.block<3, 3>(6, 9) = -JrInv_rPhi * ExprPhiijTrans * JrBiasGCorr * J_rPhi_dbg;    // J_rR_dBgi
        _jacobianOplusXi.block<3, 3>(6, 12) = O3x3;    // J_rR_dBai

        // 陀螺仪漂移误差对P V R dBg dBa的jacobian
        _jacobianOplusXi.block<3, 3>(9, 0) = O3x3;    // J_rBiasG_Pi
        _jacobianOplusXi.block<3, 3>(9, 3) = O3x3;    // J_rBiasG_Vi
        _jacobianOplusXi.block<3, 3>(9, 6) = O3x3;    // J_rBiasG_Ri
        _jacobianOplusXi.block<3, 3>(9, 9) = -I3x3;    // J_rBiasG_dBgi
        _jacobianOplusXi.block<3, 3>(9, 12) = O3x3;    // J_rBiasG_dBai

        // 加速度漂移误差对P V R dBg dBa的jacobian
        _jacobianOplusXi.block<3, 3>(12, 0) = O3x3;    // J_rBiasG_Pi
        _jacobianOplusXi.block<3, 3>(12, 3) = O3x3;    // J_rBiasG_Vi
        _jacobianOplusXi.block<3, 3>(12, 6) = O3x3;    // J_rBiasG_Ri
        _jacobianOplusXi.block<3, 3>(12, 9) = O3x3;    // J_rBiasG_dBgi
        _jacobianOplusXi.block<3, 3>(12, 12) = -I3x3;    // J_rBiasG_dBai



        /*顶点1 估计量 P V R dBg dBa*/
        _jacobianOplusXj.setZero();

        // 位置误差对P V R dBg dBa的jacobian
        _jacobianOplusXj.block<3, 3>(0, 0) = RiT;        // J_rP_Pj
        _jacobianOplusXj.block<3, 3>(0, 3) = O3x3;    // J_rP_Vj
        _jacobianOplusXj.block<3, 3>(0, 6) = O3x3;    // J_rP_Rj
        _jacobianOplusXj.block<3, 3>(0, 9) = O3x3;    // J_rP_dBgi
        _jacobianOplusXj.block<3, 3>(0, 12) = O3x3;    // J_rP_dBai

        // 速度误差对P V R dBg dBa的jacobian
        _jacobianOplusXj.block<3, 3>(3, 0) = O3x3;    // J_rV_Pj
        _jacobianOplusXj.block<3, 3>(3, 3) = RiT;        // J_rV_Vj
        _jacobianOplusXj.block<3, 3>(3, 6) = O3x3;    // J_rV_Rj
        _jacobianOplusXj.block<3, 3>(3, 9) = O3x3;    // J_rV_dBgi
        _jacobianOplusXj.block<3, 3>(3, 12) = O3x3;    // J_rV_dBai

        // 旋转误差对P V R dBg dBa的jacobian
        _jacobianOplusXj.block<3, 3>(6, 0) = O3x3;    // J_rR_Pj
        _jacobianOplusXj.block<3, 3>(6, 3) = O3x3;    // J_rR_Vj
        _jacobianOplusXj.block<3, 3>(6, 6) = JrInv_rPhi;    // J_rR_Rj
        _jacobianOplusXj.block<3, 3>(6, 9) = O3x3;    // J_rR_dBgi
        _jacobianOplusXj.block<3, 3>(6, 12) = O3x3;    // J_rR_dBai

        // 陀螺仪误差对P V R dBg dBa的jacobian
        _jacobianOplusXj.block<3, 3>(9, 0) = O3x3;    // J_rdBg_Pj
        _jacobianOplusXj.block<3, 3>(9, 3) = O3x3;    // J_rdBg_Vj
        _jacobianOplusXj.block<3, 3>(9, 6) = O3x3;    // J_rdBg_Rj
        _jacobianOplusXj.block<3, 3>(9, 9) = I3x3;    // J_rdBg_dBgi
        _jacobianOplusXj.block<3, 3>(9, 12) = O3x3;    // J_rdBg_dBai

        // 加速度误差对P V R dBg dBa的jacobian
        _jacobianOplusXj.block<3, 3>(12, 0) = O3x3;    // J_rdBa_Pj
        _jacobianOplusXj.block<3, 3>(12, 3) = O3x3;    // J_rdBa_Vj
        _jacobianOplusXj.block<3, 3>(12, 6) = O3x3;    // J_rdBa_Rj
        _jacobianOplusXj.block<3, 3>(12, 9) = O3x3;    // J_rdBa_dBgi
        _jacobianOplusXj.block<3, 3>(12, 12) = I3x3;    // J_rdBa_dBai

    }


    // class EdgeNavStatePointXYZ
    EdgeNavStatePointXYZ::EdgeNavStatePointXYZ() : BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexNavState>()
    {

    }

    bool EdgeNavStatePointXYZ::read(std::istream &is)
    {
        return true;
    }

    bool EdgeNavStatePointXYZ::write(std::ostream &os) const
    {
        return true;
    }

    void EdgeNavStatePointXYZ::linearizeOplus()
    {

        const VertexSBAPointXYZ *vPoint = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
        const VertexNavState *vNavState = static_cast<const VertexNavState *>(_vertices[1]);

        const NavState &ns = vNavState->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();
        const Vector3d &Pw = vPoint->estimate();

        Matrix3d Rcb = Rbc.transpose();
        Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

        double x = Pc[0];
        double y = Pc[1];
        double z = Pc[2];

        // 相机投影模型对Pc的jacobian
        Matrix<double, 2, 3> Maux;
        Maux.setZero();
        Maux(0, 0) = fx;
        Maux(0, 1) = 0;
        Maux(0, 2) = -x / z * fx;
        Maux(1, 0) = 0;
        Maux(1, 1) = fy;
        Maux(1, 2) = -y / z * fy;
        Matrix<double, 2, 3> Jpi = Maux / z;

        // error = obs - proj(Pc)
        // 顶点0
        _jacobianOplusXi = -Jpi * Rcb * Rwb.transpose();

        // 顶点1
        // 误差对Pwb的jacobian
        Matrix<double, 2, 3> JdPwb = -Jpi * (-Rcb * Rwb.transpose());

        // 误差对Rwb的jacobian
        Vector3d Paux = Rcb * Rwb.transpose() * (Pw - Pwb);
        Matrix<double, 2, 3> JdRwb = -Jpi * (Sophus::SO3::hat(Paux) * Rcb);

        // NavState 状态顺序 P V R dBg dBa
        Matrix<double, 2, 15> JNavState = Matrix<double, 2, 15>::Zero();
        JNavState.block<2, 3>(0, 0) = JdPwb;
        JNavState.block<2, 3>(0, 6) = JdRwb;

        _jacobianOplusXj = JNavState;

    }


    // class EdgeNavStatePointXYZOnlyPose
    void EdgeNavStatePointXYZOnlyPose::linearizeOplus()
    {
        const VertexNavState *vNavState = static_cast<const VertexNavState *>(_vertices[0]);

        const NavState &ns = vNavState->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();

        Matrix3d Rcb = Rbc.transpose();
        Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

        double x = Pc[0];
        double y = Pc[1];
        double z = Pc[2];

        // 相机投影对Pc的Jacobian
        Matrix<double, 2, 3> Maux;
        Maux.setZero();
        Maux(0, 0) = fx;
        Maux(0, 1) = 0;
        Maux(0, 2) = -x / z * fx;
        Maux(1, 0) = 0;
        Maux(1, 1) = fy;
        Maux(1, 2) = -y / z * fy;
        Matrix<double, 2, 3> Jpi = Maux / z;

        // 顶点0, 误差对Pwb的jacobian
        Matrix<double, 2, 3> JdPwb = -Jpi * (-Rcb * Rwb.transpose());

        // 顶点0, 误差对Rwb的jacobian
        Vector3d Paux = Rcb * Rwb.transpose() * (Pw - Pwb);
        Matrix<double, 2, 3> JdRwb = -Jpi * (Sophus::SO3::hat(Paux) * Rcb);

        // 顶点0是对整个NavState进行更新, 顺序是dP, dV, dPhi, dBiasGyr, dBiasAcc
        Matrix<double, 2, 15> JNavState = Matrix<double, 2, 15>::Zero();
        JNavState.block<2, 3>(0, 0) = JdPwb;
        JNavState.block<2, 3>(0, 6) = JdRwb;

        _jacobianOplusXi = JNavState;

    }


    // class VertexGyrBias
    VertexGyrBias::VertexGyrBias() : BaseVertex<3, Vector3d>()
    {

    }

    bool VertexGyrBias::read(std::istream &is)
    {
        Vector3d est;
        for (int i = 0; i < 3; i++)
            is >> est[i];
        setEstimate(est);

        return true;
    }

    bool VertexGyrBias::write(std::ostream &os) const
    {
        Vector3d est(estimate());
        for (int i = 0; i < 3; i++)
            os << est[i] << " ";

        return os.good();
    }

    void VertexGyrBias::oplusImpl(const double *update_)
    {
        Eigen::Map<const Vector3d> update(update_);
        _estimate += update;

    }


    // class EdgeGryBias
    EdgeGyrBias::EdgeGyrBias() : BaseUnaryEdge<3, Vector3d, VertexGyrBias>()
    {

    }

    bool EdgeGyrBias::read(std::istream &is)
    {
        return true;
    }

    bool EdgeGyrBias::write(std::ostream &os) const
    {
        return true;
    }

    void EdgeGyrBias::computeError()
    {
        const VertexGyrBias *v = static_cast<const VertexGyrBias *>(_vertices[0]);
        Vector3d bg = v->estimate();
        Matrix3d dRbg = Sophus::SO3::exp(J_dR_bg * bg).matrix();

        // 旋转误差 dRij^T * Riw * Rwj
        Sophus::SO3 errR((dRbij * dRbg).transpose() * Rwbi.transpose() * Rwbj);

        _error = errR.log();

    }

    void EdgeGyrBias::linearizeOplus()
    {
        // dRij^T * Riw * Rwj ???
        Sophus::SO3 errR(dRbij.transpose() * Rwbi.transpose() * Rwbj);
        Matrix3d Jlinv = Sophus::SO3::JacobianLInv(errR.log());

        _jacobianOplusXi = -Jlinv * J_dR_bg;

    }


}


