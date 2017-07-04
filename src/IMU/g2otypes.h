#ifndef G2OTYPES_H
#define G2OTYPES_H


#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"

#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

#include <so3.h>
#include <NavState.h>
#include <IMUPreintegrator.h>

namespace g2o
{
    using namespace ORB_SLAM2;
    
    // IDP, inverse depth vertex for MP
    // <D,E> D表示顶点数据的最小维度, E表示顶点的估计量的数据类型
    class VertexIDP:public BaseVertex<1, double>
    {
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	// 构造方法无法被继承,调用子类的构造函数会默认调用父类的无参构造函数
	// 如果子类需要调用父类指定方式的构造方法,需要用初始化父类成员对象的方式.
	// 下面函数的:不是继承,是初始化,后面跟父类需要的构造方法
	VertexIDP():BaseVertex<1, double>() 
	{
	    
	}
	
	bool read(std::istream &is)
	{
	    return true;
	}
	
	bool write(std::ostream &os) const
	{
	    return true;
	}
	
	virtual void setToOriginImpl()
	{
	    _estimate = 1;
	}
	
	virtual void oplusImpl(const double *update_)
	{
	    _estimate += update_[0];
	    if(_estimate < 1e-6)
		_estimate = 1e-6;
	}
	
    };
    
    
    
    // 帧图像中的重投影误差边
    // Vertex 0: MP逆深度
    // Vertex 1: 参考KF的P和R
    // vertex 2: CKF的P和R
    // vertex 3: 相机外参数Tbc(或Tcb)
    class EdgePRIDP:public BaseMultiEdge<2,Vector2d>
    {
	
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	EdgePRIDP():BaseMultiEdge<2, Vector2d>()
	{
	    resize(4);
	}
	
	bool read(std::istream &is)
	{
	    return true;
	}
	
	bool write(std::ostream &os)	const
	{
	    return true;
	}
	
	void computeError();
	
	virtual void linearizeOplus();
	
	void SetParams(double x, double y, double fx_, double fy_, double cx_, double cy_)
	{
	    refnormxy[0] = x;
	    refnormxy[1] = y;
	    fx = fx_;
	    fy = fy_;
	    cx = cx_;
	    cy = cy_;
	}
	

	
	// 归一化相机坐标系的MP坐标
	inline Vector2d project2d (const Vector3d &v) const
	{
	    Vector2d res;
	    res(0) = v(0)/v(2);
	    res(1) = v(1)/v(2);
	    return res;
	}
	
	// 投影相机坐标到2d图像平面
	Vector2d cam_project(const Vector3d &trans_xyz) const
	{
	    Vector2d proj = project2d(trans_xyz);
	    Vector2d res;
	    
	    res[0] = proj[0]*fx+cx;
	    res[1] = proj[1]*fy+cy;
	    
	    return res;
	}
	
	bool isDepthPositive()
	{
	    // 相机坐标系下的MP坐标
	    Vector3d Pc = computePc();
	    return Pc(2) > 0.01;
	}
	
	Vector3d computePc();
	
    protected:
	// [x,y]是MP在参考帧的归一化相机坐标系坐标
	double refnormxy[2];
	double fx, fy, cx, cy;
	
    };
    
    
    
    class VertexNavStatePR:public BaseVertex<6, NavState>
    {
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	VertexNavStatePR(): BaseVertex<6, NavState>()
	{
	    
	}
	
	bool read(std::istream &is)
	{
	    return true;
	}
	
	bool write(std::ostream &os) const
	{
	    return true;
	}
	
	virtual void setToOriginImpl()
	{
	    _estimate = NavState();
	}
	
	virtual void oplusImpl(const double *update_)
	{
	    // 把数据或数组转换成Vector6d
	    Eigen::Map<const Vector6d> update(update_);
	    _estimate.IncSmallPR(update);
	}
	
    };
    
    
    
    class VertexNavStateV: public BaseVertex<3, NavState>
    {
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	VertexNavStateV(): BaseVertex<3, NavState>()
	{
	    
	}
	
	bool read(std::istream &is)
	{
	    return true;
	}
	
	bool write(std::ostream &os)	const
	{
	    return true;
	}
	
	virtual void setToOriginImpl()
	{
	    _estimate = NavState();
	}
	
	virtual void oplusImpl(const double *update_)
	{
	    Eigen::Map<const Vector3d> update(update_);
	    _estimate.IncSmallV(update);
	}
	
    };
    
    
    // 定义误差边
    // 连接五个顶点, PR0, V0, bias0, PR1, V1
    // Vertex 0: PR0
    // Vertex 1: PR1
    // Vertex 2: V0
    // Vertex 3: V1
    // Vertex 4: bias0
    // edge: error_P, error_R, error_V
    // 测量值维度,测量值的数据类型(顶点)
    class EdgeNavStatePRV : public BaseMultiEdge<9, IMUPreintegrator>
    {
	
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	EdgeNavStatePRV(): BaseMultiEdge< 9,IMUPreintegrator>()
	{
	    // 连接Vertex的数量
	    resize(5);
	}
	
	bool read(std::istream &is)
	{
	    return true;
	}
	
	bool write(std::ostream &os)
	{
	    return true;
	}
	
	void computeError();
	
	virtual void linearizeOplus();
	
	void SetParams(const Vector3d &gw)
	{
	    GravityVec = gw;
	}
	
    protected:
	Vector3d GravityVec;
	
    };
    
    
    
    // 边的测量数据的维度, 数据类型, 顶点0数据类型, 顶点1数据类型
    class EdgeNavStatePRPointXYZ: public BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexNavStatePR>
    {
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	EdgeNavStatePRPointXYZ(): BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexNavStatePR>()
	{
	    
	}
	
	bool read(std::istream &is)
	{
	    return true;
	}
	
	bool write(std::ostream &os)	const
	{
	    return true;
	}
	
	void computeError()
	{
	    Vector3d Pc = computePc();
	    Vector2d obs(_measurement);
	    
	    _error = obs - cam_project(Pc);
	}
	
	bool isDepthPositive()
	{
	    Vector3d Pc = computePc();
	    return Pc(2) > 0.0;
	}
	
	Vector3d computePc()
	{
	    
	    const VertexSBAPointXYZ *vPoint = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
	    const VertexNavStatePR *vNavState = static_cast<const VertexNavStatePR *>(_vertices[1]);
	    
	    const NavState &ns = vNavState->estimate();
	    Matrix3d Rwb = ns.Get_RotMatrix();
	    Vector3d Pwb = ns.Get_P();
	    const Vector3d &Pw = vPoint->estimate();
	    
	    // IMU与Cam坐标系没有对齐,涉及两次转换. W=>b, b=>c
	    // W=>b: Pb = Rbw*Pw+tbw; tbw = -Rwb*Pwb
	    // b=>c: Pc = Rcb*Pb+tcb; tcb = -Rcb*Pbc
	    Matrix3d Rcb = Rbc.transpose();
	    Vector3d Pc = Rcb*Rwb.transpose()*(Pw-Pwb) - Rcb*Pbc;
	    
	    return Pc;
	    
	}
	
	inline Vector2d project2d(const Vector3d &v) const
	{
	    Vector2d res;
	    res(0) = v(0)/v(2);
	    res(1) = v(1)/v(2);
	    
	    return res;
	}
	
	Vector2d cam_project(const Vector3d &trans_xyz) const
	{
	    Vector2d proj = project2d(trans_xyz);
	    Vector2d res;
	    
	    res[0] = proj[0]*fx + cx;
	    res[1] = proj[1]*fy + cy;
	    
	    return res;
	}
	
	virtual void linearizeOplus();
	
	void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_, 
		       const Matrix3d &Rbc_, const Vector3d &Pbc_)
	{
	    fx = fx_;
	    fy = fy_;
	    cx = cx_;
	    cy = cy_;
	    Rbc = Rbc_;
	    Pbc = Pbc_;
	}
	
	
    protected:
	double fx, fy, cx, cy;
	
	// IMU-Cam坐标系外参
	Matrix3d Rbc;
	Vector3d Pbc;
	
    };
    
    class EdgeNavStatePRPointXYZOnlyPose:public BaseUnaryEdge<2, Vector2d, VertexNavStatePR>
    {
    public:
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	EdgeNavStatePRPointXYZOnlyPose()
	{
	    
	}
	
	bool read(std::istream &is)
	{
	    return true;
	}
	
	bool write(std::ostream &os)	const
	{
	    return true;
	}
	
	void computeError()
	{
	    Vector3d Pc = computePc();
	    Vector2d obs(_measurement);
	    
	    _error = obs - cam_project(Pc);
	}
	
	bool isDepthPositive()
	{
	    Vector3d Pc = computePc();
	    return Pc(2)>0.0;
	}
	
	Vector3d computePc()
	{
	    const VertexNavStatePR *vNSPR = static_cast<const VertexNavStatePR *>(_vertices[0]);
	    
	    const NavState &ns = vNSPR->estimate();
	    Matrix3d Rwb = ns.Get_RotMatrix();
	    Vector3d Pwb = ns.Get_P();
	    
	    Matrix3d Rcb = Rbc.transpose();
	    Vector3d Pc = Rcb*Rwb.transpose()*(Pw-Pwb) - Rcb*Pbc;
	    
	    return Pc;
	    
	}
	
	inline Vector2d project2d(const Vector3d &v) const
	{
	    Vector2d res;
	    res(0) = v(0)/v(2);
	    res(1) = v(1)/v(2);
	    
	    return res;
	}
	
	Vector2d cam_project(const Vector3d &trans_xyz) const
	{
	    Vector2d proj = project2d(trans_xyz);
	    Vector2d res;
	    res[0] = proj[0]*fx+cx;
	    res[1] = proj[1]*fy+cy;
	    
	    return res;
	}
	
	virtual void linearizeOplus();
	
	void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_, 
		       const Matrix3d &Rbc_, const Vector3d &Pbc_, const Vector3d &Pw_)
	{
	    fx = fx_;
	    fy = fy_;
	    cx = cx_;
	    cy = cy_;
	    Rbc = Rbc_;
	    Pbc = Pbc_;
	    Pw = Pw_;
	}
	
	
    protected:
	
	double fx, fy, cx, cy;
	// IMU-Cam外参
	Matrix3d Rbc;
	Vector3d Pbc;
	// MP位置
	Vector3d Pw;
	
    };
    
    
    
    // 顶点，PR,V,Bias
    // 误差，dP, dV,dBg, dBa
    class EdgeNavStatePriorPRVBias: public BaseMultiEdge<15, NavState>
    {
        
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        EdgeNavStatePriorPRVBias(): BaseMultiEdge<15, NavState>()
        {
            resize(3);
        }
        
        bool read(std::istream &is)
        {
            return true;
        }
        
        bool write(std::ostream &os) const
        {
            return true;
        }
        
        void computeError();
        
        virtual void linearizeOplus();
        
    };
    
    
    
    
    
    // NavState状态量PVR
    class VertexNavStatePVR: public BaseVertex<9, NavState>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        VertexNavStatePVR(): BaseVertex< 9, NavState >()
        {
            
        }
        
        bool read(std::istream &is)
        {
            return true;
        }
        
        bool write(std::ostream &os) const
        {
            return true;
        } 
        
        virtual void setToOriginImpl()
        {
            _estimate = NavState();
        }
        
        virtual void oplusImpl(const double * update_)
        {
            Eigen::Map<const Vector9d> update(update_);
            
            _estimate.IncSmallPVR(update);
        }
        
    };
    
    
    
    // 传感器偏移
    class VertexNavStateBias: public BaseVertex<6, NavState>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        VertexNavStateBias(): BaseVertex< 6, NavState >()
        {
            
        }
        
        bool read(std::istream &is)
        {
            return true;
        }
        
        bool write(std::ostream &os) const
        {
            return true;
        }
        
        virtual void setToOriginImpl()
        {
            _estimate = NavState();
        }
        
        virtual void oplusImpl(const double *update_)
        {
            Eigen::Map<const Vector6d> update(update_);
            _estimate.IncSmallBias(update);
            
        }
        
    };
    
    
    
    // 误差边， 测量值为IMU deltPRV
    // vertex: dP, dR, dV
    class EdgeNavStatePVR: public BaseMultiEdge<9, IMUPreintegrator>
    {
        
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        EdgeNavStatePVR(): BaseMultiEdge<9, IMUPreintegrator>()
        {
            resize(3);
        }
        
        bool read(std::istream &is)
        {
            return true;
        }
        
        bool write(std::ostream &os) const
        {
            return true;
        }
        
        void computeError();
        
        virtual void linearizeOplus();
        
        void SetParams(const Vector3d &gw)
        {
            GravityVec = gw;
        }
        
    protected:
        Vector3d GravityVec;
        
    };
    
    
    
    class EdgeNavStateBias: public BaseBinaryEdge<6, IMUPreintegrator, VertexNavStateBias, VertexNavStateBias>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        EdgeNavStateBias(): BaseBinaryEdge<6, IMUPreintegrator, VertexNavStateBias, VertexNavStateBias>()
        {
            
        }
        
        bool read(std::istream &is)
        {
            return true;
        }
        
        bool write(std::ostream &os) const
        {
            return true;
        }
        
        void computeError();
        
        virtual void linearizeOplus();
        
    };
    
    
    class EdgeNavStatePVRPointXYZ:public BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexNavStatePVR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        EdgeNavStatePVRPointXYZ(): BaseBinaryEdge< 2, Vector2d, VertexSBAPointXYZ, VertexNavStatePVR >()
        {
            
        }
        
        bool read(std::istream &is)
        {
            return true;
        }
        
        bool write(std::ostream &os) const
        {
            return true;
        }
        
        void computeError()
        {
            Vector3d Pc = computePc();
            Vector2d obs(_measurement);
            
            _error = obs-cam_project(Pc);
        }
        
        bool isDepthPositive()
        {
            Vector3d Pc = computePc();
            return Pc(2) > 0.0;
        }
        
        Vector3d computePc()
        {
            const VertexSBAPointXYZ *vPoint = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
            const VertexNavStatePVR *vNavState = static_cast<const VertexNavStatePVR *>(_vertices[1]);
        
            const NavState &ns = vNavState->estimate();
            Matrix3d Rwb = ns.Get_RotMatrix();
            Vector3d Pwb = ns.Get_P();
            const Vector3d &Pw = vPoint->estimate();
            
            Matrix3d Rcb = Rbc.transpose();
            Vector3d Pc = Rcb*Rwb.transpose()*(Pw-Pwb) - Rcb*Pbc;
            
            return Pc;
            
        }
        
        inline Vector2d project2d(const Vector3d &v) const
        {
            Vector2d res;
            res(0) = v(0)/v(2);
            res(1) = v(1)/v(2);
            
            return res;
        }
        
        Vector2d cam_project(const Vector3d &trans_xyz) const
        {
            Vector2d proj = project2d(trans_xyz);
            Vector2d res;
            res[0] = proj[0]*fx+cx;
            res[1] = proj[1]*fy+cy;
            
            return res;
        }
        
        virtual void linearizeOplus();
        
        void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_,
                        const Matrix3d &Rbc_, const Vector3d &Pbc_)
        {
           fx=fx_;
           fy=fy_;
           cx=cx_;
           cy=cy_;
           Rbc=Rbc_;
           Pbc=Pbc_;
        }
        
        
    protected:
        double fx, fy, cx, cy;
        
        Matrix3d Rbc;
        Vector3d Pbc;
        
    };
    
    
    
    class EdgeNavStatePVRPointXYZOnlyPose: public BaseUnaryEdge<2, Vector2d, VertexNavStatePVR>
    {
        
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
		EdgeNavStatePVRPointXYZOnlyPose(){}
        bool read(std::istream &is)
        {
            return true;
        }
        
        bool write(std::ostream &os) const
        {
            return true;
        }
        
        void computeError()
        {
            Vector3d Pc = computePc();
            Vector2d obs(_measurement);
            
            _error = obs-cam_project(Pc);
        }
        
        bool isDepthPositive()
        {
            Vector3d Pc = computePc();
            
            return Pc(2) >0.0;
        }
        
        Vector3d computePc()
        {
            const VertexNavStatePVR *vNSPVR = static_cast<const VertexNavStatePVR *>(_vertices[0]);
            
            const NavState &ns = vNSPVR->estimate();
            Matrix3d Rwb = ns.Get_RotMatrix();
            Vector3d Pwb = ns.Get_P();
            
            Matrix3d Rcb = Rbc.transpose();
            Vector3d Pc = Rcb*Rwb.transpose()*(Pw-Pwb)-Rcb*Pbc;
            
            return Pc;
            
        }
        
        inline Vector2d project2d(const Vector3d &v) const
        {
            Vector2d res;
            res(0) = v(0)/v(2);
            res(1) = v(1)/v(2);
            return res;
        }
        
        Vector2d cam_project(const Vector3d &trans_xyz) const
        {
            Vector2d proj = project2d(trans_xyz);
            Vector2d res;
            res[0] = proj[0]*fx+cx;
            res[1] = proj[1]*fy+cy;
            
            return res;
        }
        
        virtual void linearizeOplus();
        
        void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_, 
		       const Matrix3d &Rbc_, const Vector3d &Pbc_, const Vector3d &Pw_)
        {
            fx = fx_;
            fy = fy_;
            cx = cx_;
            cy = cy_;
            Rbc = Rbc_;
            Pbc = Pbc_;
            Pw = Pw_;
        }
              
        
    protected:
        double fx, fy, cx, cy;
        Matrix3d Rbc;
        Vector3d Pbc;
        
        Vector3d Pw;
        
    };
    
    
    
    class EdgeNavStatePriorPVRBias: public BaseBinaryEdge<15, NavState, VertexNavStatePVR, VertexNavStateBias>
    {
        
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        EdgeNavStatePriorPVRBias(): BaseBinaryEdge<15, NavState, VertexNavStatePVR, VertexNavStateBias>()
        {
            
        }
        
        bool read(std::istream &is)
        {
            return true;
        }
        
        bool write(std::ostream &os) const
        {
            return true;
        }
        
        void computeError();
        
        virtual void linearizeOplus();
        
    };
    
    
    
    // 紧耦合的VISLAM优化
    class VertexNavState : public BaseVertex<15, NavState>
    {
        
    public:
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        VertexNavState();
        
        bool read(std::istream &is);
        
        bool write(std::ostream &os) const;
        
        virtual void setToOriginImpl()
        {
            _estimate = NavState();
        }
        
        virtual void oplusImpl(const double *update_);
        
    };
    
    
    
    class EdgeNavStatePrior: public BaseUnaryEdge<15, NavState, VertexNavState>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        EdgeNavStatePrior()
		{
		
		}
        
        bool read(std::istream &is)
        {
            return true;
        }
        
        bool write(std::ostream &os) const
        {
            return true;
        }
        
        void computeError();
        
        virtual void linearizeOplus();
        
    };
    
    
    
    // edge error is Navstate_i and Navstate_j
    // 测量值是Verteo15d， 9Dof-IMU预积分和6DofIMU 偏移量
    class EdgeNavState: public BaseBinaryEdge<15, IMUPreintegrator, VertexNavState, VertexNavState>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
 		EdgeNavState();
        bool read(std::istream &is);
        
        bool write(std::ostream &os) const;

        void computeError();
        
        virtual void linearizeOplus();
        
        void SetParams(const Vector3d &gw)
        {
            GravityVec = gw;
        }
        
    protected:
        Vector3d GravityVec;
        
    };
    
    
    
    // Vertex NavState and 3D MPs
    // 测量值是2D图像的特征
    class EdgeNavStatePointXYZ: public BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexNavState>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        EdgeNavStatePointXYZ();
        
        bool read(std::istream &is);
        
        bool write(std::ostream &os) const;
        
        void computeError()
        {
            Vector3d Pc = computePc();
            Vector2d obs(_measurement);
            
            _error = obs-cam_project(Pc);
        }
        
        bool isDepthPositive()
        {
            Vector3d Pc = computePc();
            return Pc(2)>0.0;
        }
        
        Vector3d computePc()
        {
            const VertexSBAPointXYZ *vPoint = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
            const VertexNavState *vNavState = static_cast<const VertexNavState *>(_vertices[1]);
            
            const NavState &ns= vNavState->estimate();
	    Matrix3d Rwb = ns.Get_RotMatrix();
	    Vector3d Pwb = ns.Get_P();
	    const Vector3d &Pw = vPoint->estimate();
	    
	    Matrix3d Rcb = Rbc.transpose();
	    Vector3d Pc = Rcb*Rwb.transpose()*(Pw-Pwb) - Rcb*Pbc;
	    
	    return Pc;
	    
        }
        
        inline Vector2d project2d(const Vector3d &v) const
        {
	    Vector2d res;
	    res(0) = v(0)/v(2);
	    res(1) = v(1)/v(2);
	    
	    return res;
	}
	
	Vector2d cam_project(const Vector3d &trans_xyz) const 
	{
	 
	    Vector2d proj= project2d(trans_xyz);
	    Vector2d res;
	    
	    res[0] = proj[0]*fx+cx;
	    res[1] = proj[1]*fy+cy;
	    
	    return res;
	}
	
	virtual void linearizeOplus();
	
	void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_, 
		       const Matrix3d &Rbc_, const Vector3d &Pbc_)
	{
	    fx = fx_;
	    fy = fy_;
	    cx = cx_;
	    cy = cy_;
	    Rbc = Rbc_;
	    Pbc = Pbc_;
	}
	
	
        
        
    protected:
        
        double fx, fy, cx, cy;
        
        Matrix3d Rbc;
        Vector3d Pbc;
        
    };
    
    
    
    class EdgeNavStatePointXYZOnlyPose:public BaseUnaryEdge<2, Vector2d, VertexNavState>
    {
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	EdgeNavStatePointXYZOnlyPose()
	{
	    
	}
	
	bool read(std::istream &is)
        {
            return true;
        }
        
        bool write(std::ostream &os) const
        {
            return true;
        }
        
        void computeError()
	{
	    Vector3d Pc = computePc();
	    Vector2d obs(_measurement);
	    
	    _error = obs-cam_project(Pc);
	}
	
	bool isDepthPositive()
	{
	    Vector3d Pc = computePc();
	    return Pc(2) > 0.0;
	}
	
	Vector3d computePc()
	{
	    const VertexNavState *vNavState = static_cast<const VertexNavState *>(_vertices[0]);
	    
	    const NavState &ns = vNavState->estimate();
	    Matrix3d Rwb = ns.Get_RotMatrix();
	    Vector3d Pwb = ns.Get_P();
	    
	    Matrix3d Rcb = Rbc.transpose();
	    Vector3d Pc = Rcb*Rwb.transpose()*(Pw - Pwb) - Rcb*Pbc;
	    
	    return Pc;
	    
	}
	
	inline Vector2d project2d(const Vector3d &v) const
	{
	    Vector2d res;
	    res(0) = v(0)/v(2);
	    res(1) = v(1)/v(2);
	    
	    return res;
	}
	
	Vector2d cam_project(const Vector3d &trans_xyz) const
	{
	    Vector2d proj = project2d(trans_xyz);
	    Vector2d res;
	    
	    res[0] = proj[0]*fx+cx;
	    res[1] = proj[1]*fy+cy;
	    
	    return res;
	}
	
	virtual void linearizeOplus();
	
	void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_, 
		       const Matrix3d &Rbc_, const Vector3d &Pbc_, const Vector3d &Pw_)
	{
	    fx = fx_;
	    fy = fy_;
	    cx = cx_;
	    cy = cy_;
	    Rbc = Rbc_;
	    Pbc = Pbc_;
	    Pw = Pw_;
	}
	
	
    protected:
	double fx, fy, cx, cy;
	
	Matrix3d Rbc;
	Vector3d Pbc;
	
	Vector3d Pw;
	
    };
    
    
    
    // 用于VI-SLAM初始化, 顶点: 陀螺仪偏移量
    class VertexGyrBias: public BaseVertex<3, Vector3d>
    {
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	VertexGyrBias();
	
	bool read(std::istream &is);
	
	bool write(std::ostream &os) const;
	
	virtual void setToOriginImpl()
	{
	    _estimate.setZero();
	}
	
	virtual void oplusImpl(const double *update_);
	
    };
    
    
    
    // 用于VI-SLAM初始化, 边: 陀螺仪偏移量
    class EdgeGyrBias: public BaseUnaryEdge<3, Vector3d, VertexGyrBias>
    {
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	EdgeGyrBias();
	
	bool read(std::istream &is);
	
	bool write(std::ostream &os) const;
	
	Matrix3d dRbij;
	Matrix3d J_dR_bg;
	Matrix3d Rwbi;
	Matrix3d Rwbj;
	
	void computeError();
	
	virtual void linearizeOplus();
	
    };
    

    
}


#endif


