







#include "Optimizer.h"
#include "Converter.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/batch_stats.h"

#include <Eigen/StdVector>
#include <mutex>

#include "IMU/configparam.h"
#include "IMU/g2otypes.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_cholmod.h"


namespace ORB_SLAM2
{
    
    /**********************VI SLAM***************************/

    // 
    void Optimizer::LocalBAPRVIDP(KeyFrame* pCurKF, const std::list< KeyFrame* >& lLocalKeyFrames, bool* pbStopFlag, Map* pMap, cv::Mat& gw, LocalMapping* pLM)
    {
	
	// 检查CKF是否在local window的KF
	if(pCurKF != lLocalKeyFrames.back())
	    cerr<<"pCurKF != lLocalKeyFrames.back. check"<<endl;
	
	// 外参数
	Matrix4d Tcb = ConfigParam::GetEigT_cb();
	Matrix3d Rcb = Tcb.topLeftCorner(3,3);
	Vector3d Pcb = Tcb.topRightCorner(3,1);
	
	// 重力加速度
	Vector3d GravityVec = Converter::toVector3d(gw);
	
	// 优化所有local window的KF
	for(list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKFi = *lit;
	    if(!pKFi)
		cerr<<"!pKFi. why??????"<<endl;
	    pKFi->mnBALocalForKF = pCurKF->mnId;
	}
	
	// 局部KF中的局部地图点云
	list<MapPoint *> lLocalMapPoints;
	for(list<KeyFrame *>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
	    for(vector<MapPoint *>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
	    {
		MapPoint *pMP = *vit;
		if(pMP)
		{
		    if(!pMP->isBad())
		    {
			if(pMP->mnBALocalForKF != pCurKF->mnId)
			{
			    lLocalMapPoints.push_back(pMP);
			    pMP->mnBALocalForKF = pCurKF->mnId;
			}
		    }
		}
	    }
	}   

	// 观测到局部地图点但是不在local window中的KF
	list<KeyFrame *> lFixedCameras;
	// local windows第一帧KF的前一帧KF
	KeyFrame *pKFPrevLocal = lLocalKeyFrames.front()->GetPrevKeyFrame();
	
	if(pKFPrevLocal)
	{
	    // test log
	    if(pKFPrevLocal->isBad())
		cerr<<"KeyFrame before local window is bad?"<<endl;
		
	    if(pKFPrevLocal->mnBAFixedForKF==pCurKF->mnId)
		cerr<<"KeyFrame before local, has been added to lFixedKF?"<<endl;
		
	    if(pKFPrevLocal->mnBALocalForKF==pCurKF->mnId)
		cerr<<"KeyFrame before local, has been added to lLocalKF?"<<endl;

	    pKFPrevLocal->mnBAFixedForKF = pCurKF->mnId;
		
	    if(!pKFPrevLocal->isBad())
		lFixedCameras.push_back(pKFPrevLocal);
	    else
		cerr<<"pKFPrevLocal is Bad?"<<endl;
	    
	}
	else
	    cerr<<"pKFPrevLocal is NULL?"<<endl;
	    
	// 局部地图点的共视关键帧
	for(list<MapPoint *>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
	{
	    mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = (*lit)->GetObservations();
	    for(mapMapPointObs/*map<KeyFrame*,size_t>*/::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
	    {
		KeyFrame *pKFi = mit->first;
		if(pKFi->mnBALocalForKF!=pCurKF->mnId && pKFi->mnBAFixedForKF!=pCurKF->mnId)
		{
		    pKFi->mnBAFixedForKF = pCurKF->mnId;
		    if(!pKFi->isBad())
			lFixedCameras.push_back(pKFi);
		    
		}
		
	    }
	    
	}
	
	// 设置g2o优化器
	g2o::SparseOptimizer optimizer;
	
	g2o::BlockSolverX::LinearSolverType *linearSolver;    
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
	
	g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
	optimizer.setAlgorithm(solver);    
	
	if(pbStopFlag)
	    optimizer.setForceStopFlag(pbStopFlag);
	    
	int maxKFid = 0;
	
	// 相机与IMU的外参顶点
	int ExtrinsicVertexId = KeyFrame::nNextId*3 + MapPoint::nNextId+1;
	{
	    g2o::VertexNavStatePR *vTcb = new g2o::VertexNavStatePR();
	    NavState tmpNSTcb;
	    tmpNSTcb.Set_Pos(Pcb);
	    tmpNSTcb.Set_Rot(Rcb);
	    
	    vTcb->setEstimate(tmpNSTcb);
	    vTcb->setId(ExtrinsicVertexId);
	    vTcb->setFixed(true);
	    optimizer.addVertex(vTcb);
	}
	
	// 设置局部KF顶点
	for(list<KeyFrame *>::const_iterator lit=lLocalKeyFrames.begin(),lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKFi = *lit;
	    
	    if(!pKFi)
		cerr << "!pKFi when setting local KF vertex???"<<endl;
	    
	    // 顶点PR V
	    int idKF = pKFi->mnId*3;
	    {
		g2o::VertexNavStatePR *vNSPR = new g2o::VertexNavStatePR();
		vNSPR->setEstimate(pKFi->GetNavState());
		vNSPR->setId(idKF);
		vNSPR->setFixed(false);
		optimizer.addVertex(vNSPR);
		
		g2o::VertexNavStateV *vNSV = new g2o::VertexNavStateV();
		vNSV->setEstimate(pKFi->GetNavState());
		vNSV->setId(idKF+1);
		vNSV->setFixed(false);
		optimizer.addVertex(vNSV);
		
	    }
	    
	    // 顶点Bias
	    {
		g2o::VertexNavStateBias *vNSBias = new g2o::VertexNavStateBias();
		vNSBias->setEstimate(pKFi->GetNavState());
		vNSBias->setId(idKF+2);
		vNSBias->setFixed(false);
		optimizer.addVertex(vNSBias);
	    }
	    
	    if(idKF+2 > maxKFid)
		maxKFid = idKF+2;
	    
	}
	
	// 设置固定的KF 顶点
	for(list<KeyFrame *>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKFi = *lit;
	    
	    // PR 顶点
	    int idKF = pKFi->mnId*3;
	    {
		g2o::VertexNavStatePR *vNSPR = new g2o::VertexNavStatePR();
		vNSPR->setEstimate(pKFi->GetNavState());
		vNSPR->setId(idKF);
		vNSPR->setFixed(true);
		optimizer.addVertex(vNSPR);
	    }
	    
	    // 对于Local KF之前的那个KF,添加V,bias顶点
	    if(pKFi==pKFPrevLocal)
	    {
		g2o::VertexNavStateV *vNSV = new g2o::VertexNavStateV();
		vNSV->setEstimate(pKFi->GetNavState());
		vNSV->setId(idKF+1);
		vNSV->setFixed(true);
		optimizer.addVertex(vNSV);
		
		g2o::VertexNavStateBias *vNSBias = new g2o::VertexNavStateBias();
		vNSBias->setEstimate(pKFi->GetNavState());
		vNSBias->setId(idKF+2);
		vNSBias->setFixed(true);
		optimizer.addVertex(vNSBias);
	    }
	    
	    // 防止MapPoint的顶点与KF的顶点ID号重复
	    if(idKF+2 > maxKFid)
		maxKFid = idKF+2;    
	}
	
	
	// 设置边PRV
	vector<g2o::EdgeNavStatePRV *> vpEdgesNavStatePRV;
	vector<g2o::EdgeNavStateBias *> vpEdgesNavStateBias;
	// Use chi2inv() in MATLAB to compute the value corresponding to 0.95/0.99 prob. w.r.t 15DOF:
	// 12.592/16.812 for 0.95/0.99 6DoF
	// 16.919/21.666 for 0.95/0.99 9DoF
	const float thHuberNavStatePRV = sqrt(100*21.666);
	const float thHuberNavStateBias = sqrt(100*16.812);
	
	Matrix<double,6,6> InvCovBgaRW = Matrix<double,6,6>::Identity();
	// Gyroscope bias random walk, covariance INVERSE
	InvCovBgaRW.topLeftCorner(3,3) = Matrix3d::Identity()/IMUData::getGyrBiasRW2();
	
	// Accelerometer bias random walk, covariance INVERSE
	InvCovBgaRW.bottomRightCorner(3,3) = Matrix3d::Identity()/IMUData::getAccBiasRW2();
	
	for(list<KeyFrame *>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKF1 = *lit;
	    if(!pKF1)
		cerr<<"pKF1"<<endl;
	    
	    KeyFrame *pKF0 = pKF1->GetPrevKeyFrame();
	    if(!pKF1 || !pKF0)
		cerr<<"pKF1="<<(size_t)pKF1<<", pKF0="<<(size_t)pKF0<<endl;
	    
		// PRV边
		{
		    // 666, 变量名换成eprv
		    g2o::EdgeNavStatePRV *epvr = new g2o::EdgeNavStatePRV();
		    epvr->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF0->mnId)));
		    epvr->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF1->mnId)));
		    epvr->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF0->mnId+1)));
		    epvr->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF1->mnId+1)));
		    epvr->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF0->mnId+2)));
		    epvr->setMeasurement(pKF1->GetIMUPreInt());			// IMU 测量值
		    
		    Matrix9d CovPRV = pKF1->GetIMUPreInt().getCovPVPhi();	// 是PVR顺序，交换成PRV
		    CovPRV.col(3).swap(CovPRV.col(6));
		    CovPRV.col(4).swap(CovPRV.col(7));
		    CovPRV.col(5).swap(CovPRV.col(8));
		    CovPRV.row(3).swap(CovPRV.row(6));
		    CovPRV.row(4).swap(CovPRV.row(7));
		    CovPRV.row(5).swap(CovPRV.row(8));
		    epvr->setInformation(CovPRV.inverse());
		    
		    epvr->SetParams(GravityVec);
		    
		    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
		    epvr->setRobustKernel(rk);
		    rk->setDelta(thHuberNavStatePRV);
		    
		    optimizer.addEdge(epvr);
		    vpEdgesNavStatePRV.push_back(epvr);    
		}
		
		// Bias 边
		{
		    
		    g2o::EdgeNavStateBias *ebias = new g2o::EdgeNavStateBias();
		    ebias->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF0->mnId+2)));
		    ebias->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF1->mnId+2)));
		    ebias->setMeasurement(pKF1->GetIMUPreInt());
		    
		    ebias->setInformation(InvCovBgaRW/pKF1->GetIMUPreInt().getDeltaTime());
		    
		    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
		    ebias->setRobustKernel(rk);
		    rk->setDelta(thHuberNavStateBias);
		    
		    optimizer.addEdge(ebias);
		    vpEdgesNavStateBias.push_back(ebias);
		}
		
	}
	
	// 设置地图点云顶点
	const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();
	
	// PRIDP 顶点
	vector<g2o::EdgePRIDP *> vpEdgesMono;
	vpEdgesMono.reserve(nExpectedSize);
	
	vector<KeyFrame *> vpEdgeKFMono;
	vpEdgeKFMono.reserve(nExpectedSize);
	
	vector<MapPoint *> vpMapPointEdgeMono;
	vpMapPointEdgeMono.reserve(nExpectedSize);
	
	const float thHuberMono = sqrt(5.991);
	
	vector<Vector2d, Eigen::aligned_allocator<Vector2d> > vRefNormXY;
	vRefNormXY.resize(lLocalMapPoints.size());
	
	vector<bool> vMPGood;
	vMPGood.resize(lLocalMapPoints.size(), false);
	int mpcnt = 0;
	
	// 遍历所有MP
	for(list<MapPoint *>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++, mpcnt++)
	{
	    MapPoint *pMP = *lit;
	    
	    Vector3d Pw = Converter::toVector3d(pMP->GetWorldPos());
	    KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();

	    if(!pRefKF)
		cerr<<("!pRefKF???");
	    
	    // 666 输出应该修改提示
	    if(/*!sLocalFixKFs.count(pRefKF) */pRefKF->isBad())
	    {
		cerr<<pRefKF->mnId<<" pRefKF not in local or fixed window, bBad="<<pRefKF->isBad()<<endl;
		continue;
	    }
	    
	    Matrix3d Rcw = Converter::toMatrix3d(pRefKF->GetRotation());
	    Vector3d tcw = Converter::toVector3d(pRefKF->GetTranslation());
	    Vector3d Pc = Rcw*Pw+tcw;
	    double dc = Pc[2];
	    
	    if(dc < 0.01)
	    {
		std::cerr<<"depth < 0.01, = "<<dc<<std::endl;
		dc = 0.01;
		continue;
	    }
	    
	    // IDP 顶点
	    int mpVertexId = pMP->mnId + maxKFid +1;
	    g2o::VertexIDP *vPoint = new g2o::VertexIDP();
	    vPoint->setEstimate(1.0/dc);
	    vPoint->setId(mpVertexId);
	    vPoint->setMarginalized(true);
	    
	    mapMapPointObs observations = pMP->GetObservations();
	    
	    if(!observations.count(pRefKF))
		cerr<<"!observations.count(pRefKF), why???"<<endl;
	    
	    size_t kpIdxInRefKF = observations[pRefKF];
	    
	    // 投影到归一化相平面
	    const cv::KeyPoint &kpRefUn = pRefKF->mvKeysUn[kpIdxInRefKF];
	    double normx = (kpRefUn.pt.x-pRefKF->cx)/pRefKF->fx;
	    double normy = (kpRefUn.pt.y-pRefKF->cy)/pRefKF->fy;
	    vRefNormXY[mpcnt] << normx, normy; 
	    
	    bool baddmpvertex = false;
	    
	    // 设置边 EdgePRIDP
	    for(mapMapPointObs/*map<KeyFrame*,size_t>*/::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
	    {
		
		KeyFrame *pKFi = mit->first;
		if(pKFi==pRefKF)
		{
		    continue;
		}
		
		if(!pKFi->isBad())
		{
		    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];
		    
		    // 单目观测
		    if(pKFi->mvuRight[mit->second]<0)
		    {
			if(!baddmpvertex)
			{
			    baddmpvertex = true;
			    optimizer.addVertex(vPoint);
			    vMPGood[mpcnt] = true;
			}
			
			g2o::EdgePRIDP *e = new g2o::EdgePRIDP();
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(mpVertexId)));
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pRefKF->mnId)));
			e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKFi->mnId)));
			e->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(ExtrinsicVertexId)));
			e->SetParams(normx, normy, pRefKF->fx, pRefKF->fy, pRefKF->cx, pRefKF->cy);
			
			Eigen::Matrix<double,2,1> obs;
			obs << kpUn.pt.x, kpUn.pt.y;
			e->setMeasurement(obs);
			
			const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
			e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
			
			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(thHuberMono);
			
			optimizer.addEdge(e);
			
			vpEdgesMono.push_back(e);
			vpEdgeKFMono.push_back(pKFi);
			vpMapPointEdgeMono.push_back(pMP);
		    }
		    else
		    {
			cerr<<"Stereo not supported yet, why are you here?? check."<<endl;
		    }
		    
		    baddmpvertex = true;
		    
		}
	    }
	    
	}
	
	if(pbStopFlag)
	    if(*pbStopFlag)
		return;

	// 开始优化
	optimizer.initializeOptimization();
	optimizer.optimize(5);
	
	
	bool bDoMore = true;
	
	if(pbStopFlag)
	    if(*pbStopFlag)
		bDoMore = false;
	    
	    
	if(!bDoMore)
	    cerr<<"Hint: local mapping optimize only 5 iter. Need more computation resource."<<endl;
	
	if(bDoMore)
	{
	    // 检查优化后的地图内点
	    for(size_t i=0, iend=vpEdgesMono.size(); i< iend; i++)
	    {
		g2o::EdgePRIDP *e = vpEdgesMono[i];
		MapPoint *pMP = vpMapPointEdgeMono[i];
		
		if(pMP->isBad())
		    continue;
		
		if(e->chi2()>5.991 || !e->isDepthPositive() || static_cast<g2o::VertexIDP *>(e->vertex(0))->estimate()<2e-6)
		{
		    e->setLevel(1);
		}
		
		e->setRobustKernel(0);
	    }
	    
	    optimizer.initializeOptimization(0);
	    optimizer.optimize(10);
	}    
	
	vector<pair<KeyFrame *, MapPoint *> > vToErase;
	vToErase.reserve(vpEdgesMono.size());
	
	double PosePointchi2 = 0;
	
	for(size_t i=0, iend=vpEdgesMono.size();i<iend; i++)
	{
	    g2o::EdgePRIDP *e = vpEdgesMono[i];
	    MapPoint *pMP = vpMapPointEdgeMono[i];
	    
	    if(pMP->isBad())
		continue;
	    
	    if(e->chi2()>5.991 || !e->isDepthPositive() || static_cast<g2o::VertexIDP *>(e->vertex(0))->estimate()<2e-6 || e->level()!=0)
	    {
		KeyFrame *pKFi = vpEdgeKFMono[i];
		vToErase.push_back(make_pair(pKFi,pMP));
	    }
	    
	    PosePointchi2 += e->chi2();   
	}
	
	// 地图线程锁
	unique_lock<mutex> lock(pMap->mMutexMapUpdate);
	
	// 在地图中剔除外点
	if(!vToErase.empty())
	{
	    for(size_t i=0; i<vToErase.size(); i++)
	    {
		KeyFrame *pKFi = vToErase[i].first;
		MapPoint *pMPi = vToErase[i].second;
		pKFi->EraseMapPointMatch(pMPi);
		pMPi->EraseObservation(pKFi);
	    }    
	}
	
	// 更新优化后的状态
	// 关键帧
	for(list<KeyFrame *>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    
	    KeyFrame *pKFi = *lit;
	    g2o::VertexNavStatePR *vNSPR = static_cast<g2o::VertexNavStatePR *>(optimizer.vertex(3*pKFi->mnId));
	    g2o::VertexNavStateV *vNSV = static_cast<g2o::VertexNavStateV *>(optimizer.vertex(3*pKFi->mnId+1));
	    g2o::VertexNavStateBias *vNSBias = static_cast<g2o::VertexNavStateBias *>(optimizer.vertex(3*pKFi->mnId+2));
	    
	    // 优化后的导航状态
	    const NavState &optPRns = vNSPR->estimate();
	    const NavState &optVns = vNSV->estimate();
	    const NavState &optBiasns = vNSBias->estimate();
	    NavState primaryns = pKFi->GetNavState();
	    
	    pKFi->SetNavStatePos(optPRns.Get_P());
	    pKFi->SetNavStateRot(optPRns.Get_R());
	    pKFi->SetNavStateVel(optVns.Get_V());
	    
	    pKFi->SetNavStateDeltaBg(optBiasns.Get_dBias_Gyr());
	    pKFi->SetNavStateDeltaBa(optBiasns.Get_dBias_Acc());
	    
	    // 更新Tcw
	    pKFi->UpdatePoseFromNS(ConfigParam::GetMatTbc());
	    
	    // test log
	    if( (primaryns.Get_BiasGyr() - optPRns.Get_BiasGyr()).norm() > 1e-6 || (primaryns.Get_BiasGyr() - optBiasns.Get_BiasGyr()).norm() > 1e-6 )
		cerr<<"gyr bias change in optimization?"<<endl;
	    
	    if( (primaryns.Get_BiasAcc() - optPRns.Get_BiasAcc()).norm() > 1e-6 || (primaryns.Get_BiasAcc() - optBiasns.Get_BiasAcc()).norm() > 1e-6 )
		cerr<<"acc bias change in optimization?"<<endl;
	}
	
	// 地图点
	mpcnt = 0;
	for(list<MapPoint *>::const_iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++, mpcnt++)
	{
	    MapPoint *pMP = *lit;
	    if(!pMP)
		cerr<<("!pMP????");
	    
	    KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
	    if(!pRefKF)
		cerr<<("!pRefKF???");
	    
	    if(pRefKF->isBad())
		continue;
	    
	    if(!vMPGood[mpcnt])
		continue;
	    
	    g2o::VertexIDP *vPoint = static_cast<g2o::VertexIDP *>(optimizer.vertex(pMP->mnId+maxKFid+1));
	    const Vector2d &refXY = vRefNormXY[mpcnt];
	    Vector3d Pref;
	    Pref << refXY[0], refXY[1], 1.0;
	    
	    double rho = vPoint->estimate();
	    
	    // 参考帧相机坐标系下的坐标
	    Pref *= 1.0/rho;
	    
	    cv::Mat Rwr = pRefKF->GetRotation().t();
	    cv::Mat twr = pRefKF->GetCameraCenter();
	    
	    pMP->SetWorldPos(Rwr*Converter::toCvMat(Pref)+twr);
	    pMP->UpdateNormalAndDepth();
	}
	
	// 更新Tcb, 没写 
	{
	    
	    g2o::VertexNavStatePR *vTcb = static_cast<g2o::VertexNavStatePR *>(optimizer.vertex(ExtrinsicVertexId));
	    Matrix3d Rcb = vTcb->estimate().Get_RotMatrix();
	    Vector3d tcb = vTcb->estimate().Get_P();
	    
	    if(!vTcb->fixed())
		cerr<<"opt Rcb/tcb:"<<endl<<Rcb<<endl<<tcb.transpose()<<endl;
	}
	
	
	if(pLM)
	{
	    pLM->SetMapUpdateFlagInTracking(true);
	}
	
    }
    
    
    
    // 
    void Optimizer::GlobalBundleAdjustmentNavStatePRV(Map* pMap, const cv::Mat& gw, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
    {
	vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
	vector<MapPoint *> vpMP = pMap->GetAllMapPoints();
	
	// V-I 外参
	Matrix4d Tbc = ConfigParam::GetEigTbc();
	Matrix3d Rbc = Tbc.topLeftCorner(3,3);
	Vector3d Pbc = Tbc.topRightCorner(3,1);
	
	Vector3d GravityVec = Converter::toVector3d(gw);
	
	vector<bool> vbNotIncludedMP;
	vbNotIncludedMP.resize(vpMP.size());
	
	// 构建优化器
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;
	
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
	
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);
	
	if(pbStopFlag)
	    optimizer.setForceStopFlag(pbStopFlag);
	
	long unsigned int maxKFid = 0;
	
	// 设置KF节点
	for(size_t i=0; i<vpKFs.size(); i++)
	{
	    KeyFrame *pKF = vpKFs[i];
	    if(pKF->isBad())
		continue;
	    
	    // PR 顶点
	    g2o::VertexNavStatePR *vNSPR = new g2o::VertexNavStatePR();
	    vNSPR->setEstimate(pKF->GetNavState());
	    vNSPR->setId(pKF->mnId*3);
	    vNSPR->setFixed(pKF->mnId==0);	// 只固定第一帧
	    optimizer.addVertex(vNSPR);
	    
	    // V 顶点
	    g2o::VertexNavStateV *vNSV = new g2o::VertexNavStateV();
	    vNSV->setEstimate(pKF->GetNavState());
	    vNSV->setId(pKF->mnId*3+1);
	    vNSV->setFixed(false); 
	    optimizer.addVertex(vNSV);
	    
	    // Bias 顶点
	    g2o::VertexNavStateBias *vNSBias = new g2o::VertexNavStateBias();
	    vNSBias->setEstimate(pKF->GetNavState());
	    vNSBias->setId(pKF->mnId*3+2);
	    vNSBias->setFixed(pKF->mnId == 0);
	    optimizer.addVertex(vNSBias);
	    
	    if(pKF->mnId*3+2 > maxKFid)
		maxKFid = pKF->mnId*3+2;
	}
	
	
	const float thHuberNavStatePRV = sqrt(100*21.666);
	const float thHuberNavStateBias = sqrt(100*16.812);
	
	Matrix<double,6,6> InvCovBgaRW = Matrix<double,6,6>::Identity();
	InvCovBgaRW.topLeftCorner(3,3) = Matrix3d::Identity()/IMUData::getGyrBiasRW2();
	InvCovBgaRW.bottomRightCorner(3,3) = Matrix3d::Identity()/IMUData::getAccBiasRW2();
	
	for(size_t i=0; i<vpKFs.size(); i++)
	{
	    KeyFrame *pKF1 = vpKFs[i];
	    if(pKF1->isBad())
	    {
		cout<<"pKF is bad in gBA, id "<<pKF1->mnId<<endl;   //Debug log
		continue;
	    }
	    
	    KeyFrame *pKF0 = pKF1->GetPrevKeyFrame();
	    if(!pKF0)
	    {
		if(pKF1->mnId!=0)
		    cerr<<"Previous KeyFrame is NULL?"<<endl;  //Test log
		continue;
	    }
	    
	    // PRV 边
	    {
		// 666, eprv
		g2o::EdgeNavStatePRV *epvr = new g2o::EdgeNavStatePRV();
		
		// 设置边的顶点
		epvr->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF0->mnId)));
		epvr->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF1->mnId)));
		epvr->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF0->mnId+1)));
		epvr->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF1->mnId+1)));
		epvr->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF0->mnId+2)));
		epvr->setMeasurement(pKF1->GetIMUPreInt());
		// 调整顺序为PRV
		Matrix9d CovPRV = pKF1->GetIMUPreInt().getCovPVPhi();
		CovPRV.col(3).swap(CovPRV.col(6));
		CovPRV.col(4).swap(CovPRV.col(7));
		CovPRV.col(5).swap(CovPRV.col(8));
		CovPRV.row(3).swap(CovPRV.row(6));
		CovPRV.row(4).swap(CovPRV.row(7));
		CovPRV.row(5).swap(CovPRV.row(8));
		epvr->setInformation(CovPRV.inverse());
		
		epvr->SetParams(GravityVec);
		
		if(bRobust)
		{
		    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
		    epvr->setRobustKernel(rk);
		    rk->setDelta(thHuberNavStatePRV);
		}
		
		optimizer.addEdge(epvr);
	    }
	    
	    // Bias 边
	    {
		g2o::EdgeNavStateBias *ebias = new g2o::EdgeNavStateBias();
		ebias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF0->mnId+2)));
		ebias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF1->mnId+2)));
		ebias->setMeasurement(pKF1->GetIMUPreInt());
		
		ebias->setInformation(InvCovBgaRW/pKF1->GetIMUPreInt().getDeltaTime());
		
		if(bRobust)
		{
		    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
		    ebias->setRobustKernel(rk);
		    rk->setDelta(thHuberNavStateBias);
		}
		
		optimizer.addEdge(ebias);
	    }
	    
	}
	
	const float thHuber2D = sqrt(5.99);
	
	// 设置MP 顶点
	for(size_t i=0; i<vpMP.size(); i++)
	{
	    
	    MapPoint *pMP = vpMP[i];
	    
	    if(pMP->isBad())
		continue;
	    
	    g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
	    vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
	    const int id = pMP->mnId + maxKFid+1;
	    vPoint->setId(id);
	    vPoint->setMarginalized(true);
	    optimizer.addVertex(vPoint);
	    
	    const mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = pMP->GetObservations();
	    
	    
	    int nEdges = 0; 
	    // 设置边
	    for(mapMapPointObs::const_iterator mit=observations.begin() ; mit!=observations.end(); mit++)
	    {
		
		KeyFrame *pKF = mit->first;
		if(pKF->isBad() || 3*pKF->mnId>maxKFid)
		    continue;
		
		nEdges++;
		
		const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];
		if(pKF->mvuRight[mit->second] < 0)
		{
		    
		    Eigen::Matrix<double,2,1> obs;
		    obs << kpUn.pt.x, kpUn.pt.y;
		    
		    g2o::EdgeNavStatePRPointXYZ *e = new g2o::EdgeNavStatePRPointXYZ();
		    
		    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
		    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF->mnId)));
		    e->setMeasurement(obs);
		    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
		    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
		    
		    if(bRobust)
		    {
			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(thHuber2D);
		    }
		    
		    e->SetParams(pKF->fx, pKF->fy, pKF->cx, pKF->cy, Rbc, Pbc);
		    
		    optimizer.addEdge(e);
		}
		else
		{
		    cerr<<"Stereo not supported"<<endl;
		}
	    }
	    
	    if(nEdges==0)
	    {
		optimizer.removeVertex(vPoint);
		vbNotIncludedMP[i] = true;
	    }
	    else
	    {
		vbNotIncludedMP[i] = false;
	    }
	    
	}
	
	// 开始优化
	optimizer.initializeOptimization();
	optimizer.optimize(nIterations);
	
	
	
	// 更新优化数据
	
	// KF
	for(size_t i=0; i<vpKFs.size(); i++)
	{
	    KeyFrame *pKF = vpKFs[i];
	    if(pKF->isBad())
		continue;
	    
	    g2o::VertexNavStatePR *vNSPR = static_cast<g2o::VertexNavStatePR *>(optimizer.vertex(3*pKF->mnId));
	    g2o::VertexNavStateV *vNSV = static_cast<g2o::VertexNavStateV *>(optimizer.vertex(3*pKF->mnId+1));
	    g2o::VertexNavStateBias *vNSBias = static_cast<g2o::VertexNavStateBias *>(optimizer.vertex(3*pKF->mnId+2));
	    
	    const NavState &nspr = vNSPR->estimate();
	    const NavState &nsv = vNSV->estimate();
	    const NavState &nsbias = vNSBias->estimate();
	    
	    NavState ns_recov = nspr;
	    
	    ns_recov.Set_Pos(nspr.Get_P());
	    ns_recov.Set_Rot(nspr.Get_R());
	    ns_recov.Set_Vel(nsv.Get_V());
	    ns_recov.Set_DeltaBiasGyr(nsbias.Get_dBias_Gyr());
	    ns_recov.Set_DeltaBiasAcc(nsbias.Get_dBias_Acc());
	    
	    // 是否闭环
	    if(nLoopKF == 0)
	    {
		pKF->SetNavState(ns_recov);
		pKF->UpdatePoseFromNS(ConfigParam::GetMatTbc());
	    }
	    else
	    {
		pKF->mNavStateGBA = ns_recov;
		
		pKF->mTcwGBA.create(4,4,CV_32F);
		
		cv::Mat Twb_ = cv::Mat::eye(4,4,CV_32F);
		
		Converter::toCvMat(pKF->mNavStateGBA.Get_RotMatrix()).copyTo(Twb_.rowRange(0,3).colRange(0,3));
		Converter::toCvMat(pKF->mNavStateGBA.Get_P()).copyTo(Twb_.rowRange(0,3).col(3));
		cv::Mat Twc_ = Twb_*ConfigParam::GetMatTbc();
		pKF->mTcwGBA = Converter::toCvMatInverse(Twc_);
		
		pKF->mnBAGlobalForKF = nLoopKF;
	    }
	    
	}
	
	// MPs
	for(size_t i=0; i<vpMP.size(); i++)
	{
	    
	    if(vbNotIncludedMP[i])
		continue;
	    
	    MapPoint *pMP = vpMP[i];
	    
	    if(pMP->isBad())
		continue;
	    g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId+maxKFid+1));
	    
	    if(nLoopKF==0)
	    {
		pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
		pMP->UpdateNormalAndDepth();
	    }
	    else
	    {
		pMP->mPosGBA.create(3,1,CV_32F);
		Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
		pMP->mnBAGlobalForKF = nLoopKF;
	    }
	}
	
    }
    
    
    
    // 
    void Optimizer::LocalBundleAdjustmentNavStatePRV(KeyFrame* pCurKF, const std::list< KeyFrame* >& lLocalKeyFrames, bool* pbStopFlag, Map* pMap, cv::Mat& gw, LocalMapping* pLM)
    {
	
	// 检查CKF是否在local window中
	if(pCurKF != lLocalKeyFrames.back())
	    cerr<<"pCurKF != lLocalKeyFrames.back. check"<<endl;		// test log
	
	// 外参
	Matrix4d Tbc = ConfigParam::GetEigTbc();
	Matrix3d Rbc = Tbc.topLeftCorner(3,3);
	Vector3d Pbc = Tbc.topRightCorner(3,1);
	Vector3d GravityVec  = Converter::toVector3d(gw);
	
	for(list<KeyFrame*>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKFi = *lit;
	    if(!pKFi)
		cerr<<"!pKFi. why??????"<<endl;					//  test log
	    pKFi->mnBALocalForKF = pCurKF->mnId;
	}
	
	list<MapPoint *> lLocalMapPoints;
	for(list<KeyFrame *>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
	    
	    for(vector<MapPoint *>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
	    {
		MapPoint *pMP = *vit;
		if(pMP)
		    if(!pMP->isBad())
			if(pMP->mnBALocalForKF!=pCurKF->mnId)
			{
			    lLocalMapPoints.push_back(pMP);
			    pMP->mnBALocalForKF = pCurKF->mnId;
			}
		
	    }
	    
	}
	
	// 固定的KF
	list<KeyFrame *> lFixedCameras;
	KeyFrame *pKFPrevLocal = lLocalKeyFrames.front()->GetPrevKeyFrame();
	if(pKFPrevLocal)
	{
	    // test log
	    if(pKFPrevLocal->isBad())
		cerr<<"KeyFrame before local window is bad?"<<endl;
	    if(pKFPrevLocal->mnBAFixedForKF==pCurKF->mnId)
		cerr<<"KeyFrame before local, has been added to lFixedKF?"<<endl;
	    if(pKFPrevLocal->mnBALocalForKF==pCurKF->mnId)
		cerr<<"KeyFrame before local, has been added to lLocalKF?"<<endl;
	    
	    pKFPrevLocal->mnBAFixedForKF = pCurKF->mnId;
	    if(!pKFPrevLocal->isBad())
		lFixedCameras.push_back(pKFPrevLocal);
	    else
		cerr<<"pKFPrevLocal is Bad?"<<endl;   
	}
	else
	    cerr<<"pKFPrevLocal is NULL?"<<endl;
	
	// 共视KF
	for(list<MapPoint *>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
	{
	    mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = (*lit)->GetObservations();
	    for(mapMapPointObs/*map<KeyFrame*,size_t>*/::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
	    {
		KeyFrame *pKFi = mit->first;
		
		if(pKFi->mnBALocalForKF!=pCurKF->mnId && pKFi->mnBAFixedForKF!=pCurKF->mnId)
		{
		    pKFi->mnBAFixedForKF = pCurKF->mnId;
		    if(!pKFi->isBad())
			lFixedCameras.push_back(pKFi);
		}
	    }
	}
	
	// 设置优化器
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX:: LinearSolverType *linearSolver;
	
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
	
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);
	
	if(pbStopFlag)
	    optimizer.setForceStopFlag(pbStopFlag);
	
	int maxKFid = 0;
	
	// 顶点 local KF
	for(list<KeyFrame *>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKFi = *lit;
	    
	    if(!pKFi)
		cerr<<"!pKFi when setting local KF vertex???"<<endl;
	    int idKF = pKFi->mnId*3;
	    
	    // 顶点 PR /V
	    {
		
		g2o::VertexNavStatePR *vNSPR = new g2o::VertexNavStatePR();
		vNSPR->setEstimate(pKFi->GetNavState());
		vNSPR->setId(idKF);
		vNSPR->setFixed(false);
		optimizer.addVertex(vNSPR);
		
		g2o::VertexNavStateV *vNSV = new g2o::VertexNavStateV();
		vNSV->setEstimate(pKFi->GetNavState());
		vNSV->setId(idKF+1);
		vNSV->setFixed(false);
		optimizer.addVertex(vNSV);
	    }
	    
	    
	    // 顶点 bias
	    {
		g2o::VertexNavStateBias *vNSBias = new g2o::VertexNavStateBias();
		vNSBias->setEstimate(pKFi->GetNavState());
		vNSBias->setId(idKF+2);
		vNSBias->setFixed(false);
		optimizer.addVertex(vNSBias);
	    }
	    
	    if(idKF+2 > maxKFid)
		maxKFid = idKF+2;
	}
	
	// 顶点 fixed KF
	for(list<KeyFrame *>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKFi = *lit;
	    int idKF = pKFi->mnId*3;
	    
	    {
		g2o::VertexNavStatePR *vNSPR = new g2o::VertexNavStatePR();
		vNSPR->setEstimate(pKFi->GetNavState());
		vNSPR->setId(idKF);
		vNSPR->setFixed(true);
		optimizer.addVertex(vNSPR);
	    }
	    
	    
	    // local window之前的KF
	    if(pKFi == pKFPrevLocal)
	    {
		g2o::VertexNavStateV *vNSV = new g2o::VertexNavStateV();
		vNSV->setEstimate(pKFi->GetNavState());
		vNSV->setId(idKF+1);
		vNSV->setFixed(true);
		optimizer.addVertex(vNSV);
		
		g2o::VertexNavStateBias *vNSBias = new g2o::VertexNavStateBias();
		vNSBias->setEstimate(pKFi->GetNavState());
		vNSBias->setId(idKF+2);
		vNSBias->setFixed(true);
		optimizer.addVertex(vNSBias);
	    }
	    
	    if(idKF+2 > maxKFid)
		maxKFid = idKF+2;
	}
	
	
	vector<g2o::EdgeNavStatePRV *> vpEdgesNavStatePRV;
	vector<g2o::EdgeNavStateBias *> vpEdgesNavStateBias;
	
	const float thHuberNavStatePRV = sqrt(100*21.666);
	const float thHuberNavStateBias = sqrt(100*16.812);
	
	Matrix<double,6,6> InvCovBgaRW = Matrix<double,6,6>::Identity();
	InvCovBgaRW.topLeftCorner(3,3) = Matrix3d::Identity()/IMUData::getGyrBiasRW2();
	InvCovBgaRW.bottomRightCorner(3,3) = Matrix3d::Identity()/IMUData::getAccBiasRW2();
	
	for(list<KeyFrame *>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKF1 = *lit;
	    if(!pKF1)
		cerr<<"pKF1"<<endl;
	    KeyFrame *pKF0 = pKF1->GetPrevKeyFrame();
	    
	    if(!pKF1 || !pKF0)
		cerr<<"pKF1="<<(size_t)pKF1<<", pKF0="<<(size_t)pKF0<<endl;
	    
	    // 边PRV
	    {
		g2o::EdgeNavStatePRV *epvr = new g2o::EdgeNavStatePRV();
		epvr->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF0->mnId)));
		epvr->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF1->mnId)));
		epvr->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF0->mnId+1)));
		epvr->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF1->mnId+1)));
		epvr->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF0->mnId+2)));
		epvr->setMeasurement(pKF1->GetIMUPreInt());
		
		Matrix9d CovPRV = pKF1->GetIMUPreInt().getCovPVPhi();
		CovPRV.col(3).swap(CovPRV.col(6));
		CovPRV.col(4).swap(CovPRV.col(7));
		CovPRV.col(5).swap(CovPRV.col(8));
		CovPRV.row(3).swap(CovPRV.row(6));
		CovPRV.row(4).swap(CovPRV.row(7));
		CovPRV.row(5).swap(CovPRV.row(8));
		epvr->setInformation(CovPRV.inverse());
		
		epvr->SetParams(GravityVec);
		
		g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
		epvr->setRobustKernel(rk);
		rk->setDelta(thHuberNavStatePRV);
		
		optimizer.addEdge(epvr);
		vpEdgesNavStatePRV.push_back(epvr);
	    }
	    
	    // 边 bias
	    {
		g2o::EdgeNavStateBias *ebias = new g2o::EdgeNavStateBias();
		ebias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF0->mnId+2)));
		ebias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKF1->mnId+2)));
		ebias->setMeasurement(pKF1->GetIMUPreInt());
		
		ebias->setInformation(InvCovBgaRW/pKF1->GetIMUPreInt().getDeltaTime());
		
		g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
		ebias->setRobustKernel(rk);
		rk->setDelta(thHuberNavStateBias);
		
		optimizer.addEdge(ebias);
		vpEdgesNavStateBias.push_back(ebias);
	    }
	    
	}
	
	// 顶点 MP
	const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();
	
	vector<g2o::EdgeNavStatePRPointXYZ *> vpEdgesMono;
	vpEdgesMono.reserve(nExpectedSize);
	
	vector<KeyFrame *> vpEdgeKFMono;
	vpEdgeKFMono.reserve(nExpectedSize);
	
	vector<MapPoint *> vpMapPointEdgeMono;
	vpMapPointEdgeMono.reserve(nExpectedSize);
	
	
	const float thHuberMono = sqrt(5.991);
	
	for(list<MapPoint *>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
	{
	    MapPoint *pMP = *lit;
	    g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
	    vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
	    int id = pMP->mnId+maxKFid+1;
	    vPoint->setId(id);
	    vPoint->setMarginalized(true);
	    optimizer.addVertex(vPoint);
	    
	    
	    cv::Mat Pw = pMP->GetWorldPos();
	    
	    const mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = pMP->GetObservations();
	    
	    for(mapMapPointObs/*map<KeyFrame*,size_t>*/::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
	    {
		KeyFrame *pKFi = mit->first;
		
		if(!pKFi->isBad())
		{
		    
		    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];
		    
		    if(pKFi->mvuRight[mit->second] < 0)
		    {
			Eigen::Matrix<double, 2, 1> obs;
			obs << kpUn.pt.x, kpUn.pt.y;
			
			g2o::EdgeNavStatePRPointXYZ *e = new g2o::EdgeNavStatePRPointXYZ();
			
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3*pKFi->mnId)));
			e->setMeasurement(obs);
			const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
			
			e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
			
			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(thHuberMono);
			
			e->SetParams(pKFi->fx, pKFi->fy, pKFi->cx, pKFi->cy, Rbc, Pbc);
			
			optimizer.addEdge(e);
			vpEdgesMono.push_back(e);
			vpEdgeKFMono.push_back(pKFi);
			vpMapPointEdgeMono.push_back(pMP);
			
		    }
		    else
		    {
			cerr<<"Stereo not supported yet, why here?? check."<<endl;
		    }   
		}
	    }
	    
	}
	
	if(pbStopFlag)
	    if(*pbStopFlag)
		return ;
	
	optimizer.initializeOptimization();
	optimizer.optimize(5);
	
	bool bDoMore=true;
	
	if(pbStopFlag)
	    if(*pbStopFlag)
		bDoMore = false;
	
	if(bDoMore)
	{
	    // 检查内点
	    for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
	    {
		g2o::EdgeNavStatePRPointXYZ *e = vpEdgesMono[i];
		MapPoint *pMP = vpMapPointEdgeMono[i];
		
		if(pMP->isBad())
		    continue;
		
		if(e->chi2()>5.991 || !e->isDepthPositive())
		{
		    e->setLevel(1);
		}
		
		e->setRobustKernel(0);
	    }
	    
	    optimizer.initializeOptimization(0);
	    optimizer.optimize(10);
	    
	}
	
	vector<pair<KeyFrame*, MapPoint *>> vToErase;
	vToErase.reserve(vpEdgesMono.size());
	
	double PosePointchi2 = 0;
	
	// 内点记录
	for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
	{
	    g2o::EdgeNavStatePRPointXYZ *e = vpEdgesMono[i];
	    MapPoint *pMP = vpMapPointEdgeMono[i];
	    
	    if(pMP->isBad())
		continue;
	    
	    if(e->chi2()>5.991 || !e->isDepthPositive())
	    {
		KeyFrame *pKFi = vpEdgeKFMono[i];
		vToErase.push_back(make_pair(pKFi, pMP));
	    }
	    
	    PosePointchi2 += e->chi2();
	}
	
	
	
	unique_lock<mutex> lock(pMap->mMutexMapUpdate);
	
	if(!vToErase.empty())
	{
	    for(size_t i=0; i<vToErase.size(); i++)
	    {
		KeyFrame *pKFi  = vToErase[i].first;
		MapPoint *pMPi = vToErase[i].second;
		pKFi->EraseMapPointMatch(pMPi);
		pMPi->EraseObservation(pKFi);
	    }
	}
	
	// 更新优化后的结果
	
	// KF
	for(list<KeyFrame *>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKFi = *lit;
	    g2o::VertexNavStatePR *vNSPR = static_cast<g2o::VertexNavStatePR *>(optimizer.vertex(3*pKFi->mnId));
	    g2o::VertexNavStateV *vNSV = static_cast<g2o::VertexNavStateV *>(optimizer.vertex(3*pKFi->mnId+1));
	    g2o::VertexNavStateBias *vNSBias = static_cast<g2o::VertexNavStateBias *>(optimizer.vertex(3*pKFi->mnId+2));
	    
	    const NavState &optPRns = vNSPR->estimate();
	    const NavState &optVns = vNSV->estimate();
	    const NavState &optBiasns = vNSBias->estimate();
	    NavState primaryns = pKFi->GetNavState();
	    
	    // 更新导航状态
	    pKFi->SetNavStatePos(optPRns.Get_P());
	    pKFi->SetNavStateRot(optPRns.Get_R());
	    pKFi->SetNavStateVel(optVns.Get_V());
	    pKFi->SetNavStateDeltaBg(optBiasns.Get_dBias_Gyr());
	    pKFi->SetNavStateDeltaBa(optBiasns.Get_dBias_Acc());
	    
	    // 跟新相机位姿
	    pKFi-> UpdatePoseFromNS(ConfigParam::GetMatTbc());
	    
	    // test log
	    if( (primaryns.Get_BiasGyr() - optPRns.Get_BiasGyr()).norm() > 1e-6 || (primaryns.Get_BiasGyr() - optBiasns.Get_BiasGyr()).norm() > 1e-6 )
		cerr<<"gyr bias change in optimization?"<<endl;
	    
	    if( (primaryns.Get_BiasAcc() - optPRns.Get_BiasAcc()).norm() > 1e-6 || (primaryns.Get_BiasAcc() - optBiasns.Get_BiasAcc()).norm() > 1e-6 )
		cerr<<"acc bias change in optimization?"<<endl;
	    
	}
	
	
	// MPs
	for(list<MapPoint *>::const_iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
	{
	    MapPoint *pMP = *lit;
	    g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId+maxKFid+1));
	    pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
	    pMP->UpdateNormalAndDepth();
	}
	
	if(pLM)
	{
	    pLM->SetMapUpdateFlagInTracking(true);
	}
	
    }
    
    
    
    // 
    void Optimizer::GlobalBundleAdjustmentNavState(Map* pMap, const cv::Mat& gw, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
    {
	
	vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
	vector<MapPoint *> vpMP = pMap->GetAllMapPoints();
	
	// 外参
	Matrix4d Tbc = ConfigParam::GetEigTbc();
	Matrix3d Rbc = Tbc.topLeftCorner(3,3);
	Vector3d Pbc = Tbc.topRightCorner(3,1);
	
	Vector3d GravityVec = Converter::toVector3d(gw);
	
	vector<bool> vbNotIncludedMP;
	vbNotIncludedMP.resize(vpMP.size());
	
	// 求解器
	g2o::SparseOptimizer optimizer;	
	
	g2o::BlockSolverX::LinearSolverType *linearSolver;
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
	
	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);
	
	if(pbStopFlag)
	    optimizer.setForceStopFlag(pbStopFlag);
	
	long unsigned int maxKFid = 0;
	
	// 顶点KF
	for(size_t i=0; i<vpKFs.size(); i++)
	{
	    KeyFrame *pKF = vpKFs[i];
	    if(pKF->isBad())
		continue;
	    
	    // 顶点 PVR
	    g2o::VertexNavStatePVR *vNSPVR = new g2o::VertexNavStatePVR();
	    vNSPVR->setEstimate(pKF->GetNavState());
	    vNSPVR->setId(pKF->mnId*2);
	    vNSPVR->setFixed(pKF->mnId==0);
	    optimizer.addVertex(vNSPVR);
	    
	    // 顶点Bias
	    g2o::VertexNavStateBias *vNSBias = new g2o::VertexNavStateBias();
	    vNSBias->setEstimate(pKF->GetNavState());
	    vNSBias->setId(pKF->mnId*2+1);
	    vNSBias->setFixed(pKF->mnId==0);
	    optimizer.addVertex(vNSBias);
	    
	    if(pKF->mnId*2+1 > maxKFid)
		maxKFid = pKF->mnId*2+1;
	}
	
	const float thHuberNavStatePVR = sqrt(21.666);
	const float thHuberNavStateBias = sqrt(16.812);
	
	Matrix<double,6,6> InvCovBgaRW = Matrix<double,6,6>::Identity();
	InvCovBgaRW.topLeftCorner(3,3) = Matrix3d::Identity()/IMUData::getGyrBiasRW2();
	InvCovBgaRW.bottomRightCorner(3,3) = Matrix3d::Identity()/IMUData::getAccBiasRW2();
	
	
	for(size_t i=0; i<vpKFs.size(); i++)
	{
	    
	    KeyFrame *pKF1 = vpKFs[i];
	    
	    if(pKF1->isBad())
	    {
		cout<<"pKF is bad in gBA, id "<<pKF1->mnId<<endl;   //Debug log
		continue;
	    }
	    
	    KeyFrame *pKF0 = pKF1->GetPrevKeyFrame();
	    if(!pKF0)
	    {
		if(pKF1->mnId!=0)
		    cerr<<"Previous KeyFrame is NULL?"<<endl;
		continue;
	    }
	    
	    // 边 PVR
	    {
		g2o::EdgeNavStatePVR *epvr = new g2o::EdgeNavStatePVR();
		epvr->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2*pKF0->mnId)));
		epvr->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2*pKF1->mnId)));
		epvr->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2*pKF0->mnId+1)));
		epvr->setMeasurement(pKF1->GetIMUPreInt());
		
		Matrix9d InvCovPVR = pKF1->GetIMUPreInt().getCovPVPhi().inverse();
		epvr->setInformation(InvCovPVR);
		epvr->SetParams(GravityVec);
		
		if(bRobust)
		{
		    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
		    epvr->setRobustKernel(rk);
		    rk->setDelta(thHuberNavStatePVR);
		}
		
		optimizer.addEdge(epvr);
	    }
	    
	    // 边 Bias
	    {
		
		g2o::EdgeNavStateBias *ebias = new g2o::EdgeNavStateBias();
		ebias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2*pKF0->mnId+1)));
		ebias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2*pKF1->mnId+1)));
		ebias->setMeasurement(pKF1->GetIMUPreInt());
		
		ebias->setInformation(InvCovBgaRW/pKF1->GetIMUPreInt().getDeltaTime());
		
		if(bRobust)
		{
		    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
		    ebias->setRobustKernel(rk);
		    rk->setDelta(thHuberNavStateBias);
		}
		
		optimizer.addEdge(ebias);
	    }
	    
	}
	
	
	const float thHuber2D = sqrt(5.99);
	
	// 顶点 MP
	for(size_t i=0; i<vpMP.size(); i++)
	{
	    
	    MapPoint *pMP = vpMP[i];
	    if(pMP->isBad())
		continue;
	    
	    g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
	    vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
	    const int id = pMP->mnId+maxKFid+1;
	    vPoint->setId(id);
	    vPoint->setMarginalized(true);
	    optimizer.addVertex(vPoint);
	    
	    const mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = pMP->GetObservations();
	    
	    int nEdges = 0;
	    for(mapMapPointObs/*map<KeyFrame*,size_t>*/::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
	    {
		KeyFrame *pKF = mit->first;
		if(pKF->isBad() || 2*pKF->mnId>maxKFid)
		    continue;
		
		nEdges++;
		
		const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];
		
		if(pKF->mvuRight[mit->second] < 0)
		{
		    Eigen::Matrix<double,2 ,1> obs;
		    obs << kpUn.pt.x, kpUn.pt.y;
		    
		    // 边 PVR Point
		    g2o::EdgeNavStatePVRPointXYZ *e = new g2o::EdgeNavStatePVRPointXYZ();
		    
		    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
		    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2*pKF->mnId)));
		    e->setMeasurement(obs);
		    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
		    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
		    
		    if(bRobust)
		    {
			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(thHuber2D);
		    }
		    
		    e->SetParams(pKF->fx, pKF->fy, pKF->cx, pKF->cy, Rbc, Pbc);
		    
		    optimizer.addEdge(e);
		}
		else
		{
		    cerr<<"Stereo not supported"<<endl;
		}
	    }
	    
	    if(nEdges == 0)
	    {
		optimizer.removeVertex(vPoint);
		vbNotIncludedMP[i] = true;
	    }
	    else
	    {
		vbNotIncludedMP[i] = false;
	    }
	}
	
	// 优化
	optimizer.initializeOptimization();
	optimizer.optimize(nIterations);
	
	// 更新优化状态
	// KF
	for(size_t i=0; i<vpKFs.size(); i++)
	{
	    
	    KeyFrame *pKF = vpKFs[i];
	    if(pKF->isBad())
		continue;
	    
	    g2o::VertexNavStatePVR *vNSPVR = static_cast<g2o::VertexNavStatePVR *>(optimizer.vertex(2*pKF->mnId));
	    g2o::VertexNavStateBias *vNSBias = static_cast<g2o::VertexNavStateBias *>(optimizer.vertex(2*pKF->mnId+1));
	    const NavState &nspvr = vNSPVR->estimate();
	    const NavState &nsbias = vNSBias->estimate();
	    NavState ns_recov = nspvr;
	    // dBias_Gyr, dBias_Acc 使用Bias边优化的值
	    ns_recov.Set_DeltaBiasGyr(nsbias.Get_dBias_Gyr());
	    ns_recov.Set_DeltaBiasAcc(nsbias.Get_dBias_Acc());
	    
	    if(nLoopKF==0)
	    {
		pKF->SetNavState(ns_recov);
		pKF->UpdatePoseFromNS(ConfigParam::GetMatTbc());
	    }
	    else
	    {
		pKF->mNavStateGBA = ns_recov;
		pKF->mTcwGBA.create(4,4,CV_32F);
		
		cv::Mat Twb_ = cv::Mat::eye(4,4,CV_32F);
		Converter::toCvMat(pKF->mNavStateGBA.Get_RotMatrix()).copyTo(Twb_.rowRange(0,3).colRange(0,3));
		Converter::toCvMat(pKF->mNavStateGBA.Get_P()).copyTo(Twb_.rowRange(0,3).col(3));
		cv::Mat Twc_ = Twb_*ConfigParam::GetMatTbc();
		pKF->mTcwGBA = Converter::toCvMatInverse(Twc_);
		
		pKF->mnBAGlobalForKF = nLoopKF;
	    }
	    
	}
	
	// Points
	for(size_t i=0; i<vpMP.size(); i++)
	{
	    if(vbNotIncludedMP[i])
		continue;
	    
	    MapPoint *pMP = vpMP[i];
	    
	    if(pMP->isBad())
		continue;
	    g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId+maxKFid+1));
	 
	    if(nLoopKF == 0)
	    {
		pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
		pMP->UpdateNormalAndDepth();
	    }
	    else
	    {
		pMP->mPosGBA.create(3,1,CV_32F);
		Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
		pMP->mnBAGlobalForKF = nLoopKF;
	    }
	}
	
    }
    
    
    
    // 
    int Optimizer::PoseOptimization(Frame* pFrame, Frame* pLastFrame, const IMUPreintegrator& imupreint, const cv::Mat& gw, const bool& bComputeMarg)
    {
	
	// 外参
	Matrix4d Tbc = ConfigParam::GetEigTbc();
	Matrix3d Rbc = Tbc.topLeftCorner(3,3);
	Vector3d Pbc = Tbc.topRightCorner(3,1);
	Vector3d GravityVec = Converter::toVector3d(gw);
	
	// 构建优化器
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;
	linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
	
	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);
	
	int nInitialCorrespondences = 0;
	
	const int FramePVRId = 0;
	const int FrameBiasId = 1;
	const int LastFramePVRId = 2;
	const int LastFrameBiasId = 3;
	
	// pFrame顶点 PVR/Bias
	g2o::VertexNavStatePVR *vNSFPVR = new g2o::VertexNavStatePVR();
	{
	    vNSFPVR->setEstimate(pFrame->GetNavState());
	    vNSFPVR->setId(FramePVRId);
	    vNSFPVR->setFixed(false);
	    optimizer.addVertex(vNSFPVR);
	}
	
	g2o::VertexNavStateBias *vNSFBias = new g2o::VertexNavStateBias();
	{
	    vNSFBias->setEstimate(pFrame->GetNavState());
	    vNSFBias->setId(FrameBiasId);
	    vNSFBias->setFixed(false);
	    optimizer.addVertex(vNSFBias);
	}
	
	// LastFrame 顶点 PVR Bias
	g2o::VertexNavStatePVR *vNSFPVRlast = new g2o::VertexNavStatePVR();
	{
	    vNSFPVRlast->setEstimate(pLastFrame->GetNavState());
	    vNSFPVRlast->setId(LastFramePVRId);
	    vNSFPVRlast->setFixed(false);
	    optimizer.addVertex(vNSFPVRlast);
	}
	
	g2o::VertexNavStateBias *vNSFBiaslast = new g2o::VertexNavStateBias();
	{
	    vNSFBiaslast->setEstimate(pLastFrame->GetNavState());
	    vNSFBiaslast->setId(LastFrameBiasId);
	    vNSFBiaslast->setFixed(false);
	    optimizer.addVertex(vNSFBiaslast);
	}
	
	
	// Last Frame边 PriorPVRBias
	g2o::EdgeNavStatePriorPVRBias *eNSPrior = new g2o::EdgeNavStatePriorPVRBias();
	{
	    eNSPrior->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastFramePVRId)));
	    eNSPrior->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastFrameBiasId)));
	    eNSPrior->setMeasurement(pLastFrame->mNavStatePrior);
	    
	    eNSPrior->setInformation(pLastFrame->mMargCovInv);
	    
	    const float thHuberNavState = sqrt(30.5779);
	    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
	    eNSPrior->setRobustKernel(rk);
	    rk->setDelta(thHuberNavState);
	    
	    optimizer.addEdge(eNSPrior);
	}
	
	// 边PVR Frame-LastFrame
	g2o::EdgeNavStatePVR *eNSPVR = new g2o::EdgeNavStatePVR(); 
	{
	    eNSPVR->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastFramePVRId)));
	    eNSPVR->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FramePVRId)));
	    eNSPVR->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastFrameBiasId)));
	    eNSPVR->setMeasurement(imupreint);
	    
	    Matrix9d InvCovPVR = imupreint.getCovPVPhi().inverse();
	    eNSPVR->setInformation(InvCovPVR);
	    
	    eNSPVR->SetParams(GravityVec);
	    
	    const float thHuberNavStatePVR = sqrt(21.666);
	    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
	    eNSPVR->setRobustKernel(rk);
	    rk->setDelta(thHuberNavStatePVR);
	    
	    optimizer.addEdge(eNSPVR);
	}
	
	// 边Bias Frame-LastFrame
	g2o::EdgeNavStateBias *eNSBias = new g2o::EdgeNavStateBias();
	{
	    eNSBias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastFrameBiasId)));
	    eNSBias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FrameBiasId)));
	    eNSBias->setMeasurement(imupreint);
	    
	    Matrix<double,6,6> InvCovBgaRW = Matrix<double,6,6>::Identity();
	    InvCovBgaRW.topLeftCorner(3,3) = Matrix3d::Identity()/IMUData::getGyrBiasRW2();
	    InvCovBgaRW.bottomRightCorner(3,3) = Matrix3d::Identity()/IMUData::getAccBiasRW2();
	    eNSBias->setInformation(InvCovBgaRW/imupreint.getDeltaTime());
	    
	    const float thHuberNavStateBias = sqrt(16.812);
	    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
	    eNSBias->setRobustKernel(rk);
	    rk->setDelta(thHuberNavStateBias);
	    
	    optimizer.addEdge(eNSBias);
	}
	
	// 投影误差边
	const int Ncur = pFrame->N;
	const int Nlast = pLastFrame->N;
	
	vector<g2o::EdgeNavStatePVRPointXYZOnlyPose *> vpEdgesMono;
	vector<size_t> vnIndexEdgeMono;
	vpEdgesMono.reserve(Ncur);
	vnIndexEdgeMono.reserve(Ncur);
	
	vector<g2o::EdgeNavStatePVRPointXYZOnlyPose *> vpEdgesMonoLast;
	vector<size_t> vnIndexEdgeMonoLast;
	vpEdgesMonoLast.reserve(Nlast);
	vnIndexEdgeMonoLast.reserve(Nlast);
	
	const float deltaMono = sqrt(5.991);
	
	{
	    unique_lock<mutex> lock(MapPoint::mGlobalMutex);
	    
	    // pFrame 位姿误差边
	    for(int i=0; i<Ncur; i++)
	    {
		MapPoint *pMP = pFrame->mvpMapPoints[i];
		if(pMP)
		{
		    // Mono
		    if(pFrame->mvuRight[i]<0)
		    {
			nInitialCorrespondences++;
			pFrame->mvbOutlier[i] = false;
			
			Eigen::Matrix<double,2,1> obs;
			const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
			obs << kpUn.pt.x, kpUn.pt.y;
			
			g2o::EdgeNavStatePVRPointXYZOnlyPose *e = new g2o::EdgeNavStatePVRPointXYZOnlyPose();
			
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FramePVRId)));
			e->setMeasurement(obs);
			const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
			e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
			
			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(deltaMono);
			
			e->SetParams(pFrame->fx, pFrame->fy, pFrame->cx, pFrame->cy, Rbc, Pbc, Converter::toVector3d(pMP->GetWorldPos()));
			
			optimizer.addEdge(e);
			
			vpEdgesMono.push_back(e);
			vnIndexEdgeMono.push_back(i);
		    }
		    else
		    {
			cerr<<"stereo shouldn't in poseoptimization"<<endl;
		    }    
		}
		
	    }
	    
	    // LastFrame误差边
	    for(int i=0; i<Nlast; i++)
	    {
		MapPoint *pMP = pLastFrame->mvpMapPoints[i];
		if(pMP)
		{
		    // Mono
		    if(pLastFrame->mvuRight[i] < 0)
		    {
			pLastFrame->mvbOutlier[i] = false;
			
			Eigen::Matrix<double,2,1> obs;
			const cv::KeyPoint &kpUn = pLastFrame->mvKeysUn[i];
			obs << kpUn.pt.x, kpUn.pt.y;
			
			g2o::EdgeNavStatePVRPointXYZOnlyPose *e = new g2o::EdgeNavStatePVRPointXYZOnlyPose();
			
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastFramePVRId)));
			e->setMeasurement(obs);
			const float invSigma2 = pLastFrame->mvInvLevelSigma2[kpUn.octave];
			e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
			
			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(deltaMono);
			
			e->SetParams(pLastFrame->fx, pLastFrame->fy, pLastFrame->cx, pLastFrame->cy, Rbc, Pbc, Converter::toVector3d(pMP->GetWorldPos()));
			
			optimizer.addEdge(e);
			
			vpEdgesMonoLast.push_back(e);
			vnIndexEdgeMonoLast.push_back(i);
		    }
		    else
		    {
			cerr<<"stereo shouldn't in poseoptimization"<<endl;
		    }
		}
	    }
	    
	}
	
	if(nInitialCorrespondences < 3)
	    return 0;
	
	// 执行4次优化，每次执行完成后，分类内点外点，下次优化时，外点不包含，但是优化之后，所有点重新分类
	const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
	const int its[4] = {10, 10, 10, 10};
	
	int nBad = 0;
	int nBadLast = 0;
	for(size_t it=0; it<4; it++)
	{
	    
	    // 重置估计值
	    vNSFPVR->setEstimate(pFrame->GetNavState());
	    vNSFBias->setEstimate(pFrame->GetNavState());
	    vNSFPVRlast->setEstimate(pLastFrame->GetNavState());
	    vNSFBiaslast->setEstimate(pLastFrame->GetNavState());
	    
	    // 优化
	    optimizer.initializeOptimization(0);
	    optimizer.optimize(its[it]);
	    
	    nBad = 0;
	    for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
	    {
		
		g2o::EdgeNavStatePVRPointXYZOnlyPose *e = vpEdgesMono[i];
		const size_t idx = vnIndexEdgeMono[i];
		
		if(pFrame->mvbOutlier[idx])
		{
		    e->computeError();
		}
		
		const float chi2 = e->chi2();
		
		if(chi2 > chi2Mono[it])
		{
		    pFrame->mvbOutlier[idx] = true;
		    e->setLevel(1);
		    nBad++;
		}
		else
		{
		    pFrame->mvbOutlier[idx]  = false;
		    e->setLevel(0);
		}
		
		if(it==2)
		    e->setRobustKernel(0);
	    }
	    
	    nBadLast = 0;
	    for(size_t i=0, iend=vpEdgesMonoLast.size(); i<iend; i++)
	    {
		g2o::EdgeNavStatePVRPointXYZOnlyPose *e = vpEdgesMonoLast[i];
		
		const size_t idx = vnIndexEdgeMonoLast[i];
		
		if(pLastFrame->mvbOutlier[idx])
		{
		    e->computeError();
		}
		
		const float chi2 = e->chi2();
		
		if(chi2>chi2Mono[it])
		{
		    pLastFrame->mvbOutlier[idx] = true;
		    e->setLevel(1);
		    nBadLast++;
		}
		else
		{
		    pLastFrame->mvbOutlier[idx] = false;
		    e->setLevel(0);
		}
		
		if(it==2)
		    e->setRobustKernel(0);
	    }
	    
	    if(optimizer.edges().size()<10)
		break;    
	}
	
	// 更新优化后的状态
	g2o::VertexNavStatePVR *vNSPVR_recov = static_cast<g2o::VertexNavStatePVR *>(optimizer.vertex(FramePVRId));
	const NavState &nsPVR_recov = vNSPVR_recov->estimate();
	g2o::VertexNavStateBias *vNSBias_recov = static_cast<g2o::VertexNavStateBias *>(optimizer.vertex(FrameBiasId));
	const NavState &nsBias_recov = vNSBias_recov->estimate();

	NavState ns_recov = nsPVR_recov;
	ns_recov.Set_DeltaBiasGyr(nsBias_recov.Get_dBias_Gyr());
	ns_recov.Set_DeltaBiasAcc(nsBias_recov.Get_dBias_Acc());
	pFrame->SetNavState(ns_recov);
	pFrame->UpdatePoseFromNS(ConfigParam::GetMatTbc());
	
	// Compute marginalized Hessian H and B, H*x=B
	// H/B can be used as prior for next optimization in PoseOptimization
	if(bComputeMarg)
	{
	    std::vector<g2o::OptimizableGraph::Vertex *> margVerteces;
	    margVerteces.push_back(optimizer.vertex(FramePVRId));
	    margVerteces.push_back(optimizer.vertex(FrameBiasId));
	    
	    g2o::SparseBlockMatrixXd spinv;
	    optimizer.computeMarginals(spinv, margVerteces);
	    
	    Matrix<double,15,15> margCov = Matrix<double,15,15>::Zero();
	    margCov.topLeftCorner(9,9) = spinv.block(0,0)->eval();
	    margCov.topRightCorner(9,6) = spinv.block(0,1)->eval();
	    margCov.bottomLeftCorner(6,9) = spinv.block(1,0)->eval();
	    margCov.bottomRightCorner(6,6) = spinv.block(1,1)->eval();
	    pFrame->mMargCovInv = margCov.inverse();
	    pFrame->mNavStatePrior = ns_recov;
	    
	}
	
	// test log
	if( (nsPVR_recov.Get_BiasGyr()-nsBias_recov.Get_BiasGyr()).norm()>1e-6 || (nsPVR_recov.Get_BiasAcc()-nsBias_recov.Get_BiasAcc()).norm()>1e-6 )
	{
	    std::cerr<<"1 recovered bias gyr not equal for PVR/Bias vertex"<<std::endl<<nsPVR_recov.Get_BiasGyr().transpose()<<" / "<<nsBias_recov.Get_BiasGyr().transpose()<<std::endl;
	    std::cerr<<"1 recovered bias acc not equal for PVR/Bias vertex"<<std::endl<<nsPVR_recov.Get_BiasAcc().transpose()<<" / "<<nsBias_recov.Get_BiasAcc().transpose()<<std::endl;
	}
	
	if( (ns_recov.Get_dBias_Gyr()-nsBias_recov.Get_dBias_Gyr()).norm()>1e-6 || (ns_recov.Get_dBias_Acc()-nsBias_recov.Get_dBias_Acc()).norm()>1e-6 )
	{
	    std::cerr<<"1 recovered delta bias gyr not equal to Bias vertex"<<std::endl<<ns_recov.Get_dBias_Gyr().transpose()<<" / "<<nsBias_recov.Get_dBias_Gyr().transpose()<<std::endl;
	    std::cerr<<"1 recovered delta bias acc not equal to Bias vertex"<<std::endl<<ns_recov.Get_dBias_Acc().transpose()<<" / "<<nsBias_recov.Get_dBias_Acc().transpose()<<std::endl;
	}
	
	return nInitialCorrespondences-nBad;
	
    }
    
    
     // 
    int  Optimizer::PoseOptimization(Frame *pFrame, KeyFrame *pLastKF, const IMUPreintegrator &imupreint, const cv::Mat &gw, const bool &bComputeMarg)
    {
	// 内参
	Matrix4d Tbc = ConfigParam::GetEigTbc();
	Matrix3d Rbc = Tbc.topLeftCorner(3,3);
	Vector3d Pbc = Tbc.topRightCorner(3,1);
	
	Vector3d GravityVec = Converter::toVector3d(gw);
	
	// 构造优化器
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;
	linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
	
	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);
	
	int nInitialCorrespondences = 0;
	
	const int FramePVRId = 0;
	const int FrameBiasId = 1;
	const int LastKFPVRId = 2;
	const int LastKFBiasId = 3;
	
	// 顶点 Frame PVR/Bias
	g2o::VertexNavStatePVR *vNSFPVR = new g2o::VertexNavStatePVR();
	{
	    vNSFPVR->setEstimate(pFrame->GetNavState());
	    vNSFPVR->setId(FramePVRId);
	    vNSFPVR->setFixed(false);
	    optimizer.addVertex(vNSFPVR);
	}
	
	g2o::VertexNavStateBias *vNSFBias = new g2o::VertexNavStateBias();
	{
	    vNSFBias->setEstimate(pFrame->GetNavState());
	    vNSFBias->setId(FrameBiasId);
	    vNSFBias->setFixed(false);
	    optimizer.addVertex(vNSFBias);
	}
	
	// 顶点 KF PVR/Bias
	g2o::VertexNavStatePVR *vNSKFPVR = new g2o::VertexNavStatePVR();
	{
	    vNSKFPVR->setEstimate(pLastKF->GetNavState());
	    vNSKFPVR->setId(LastKFPVRId);
	    vNSKFPVR->setFixed(true);
	    optimizer.addVertex(vNSKFPVR);
	}
	
	g2o::VertexNavStateBias *vNSKFBias = new g2o::VertexNavStateBias();
	{
	    vNSKFBias->setEstimate(pLastKF->GetNavState());
	    vNSKFBias->setId(LastKFBiasId);
	    vNSKFBias->setFixed(true);
	    optimizer.addVertex(vNSKFBias);
	}
	
	
	// 边 KF-Frame PVR
	g2o::EdgeNavStatePVR *eNSPVR = new g2o::EdgeNavStatePVR();
	{
	    eNSPVR->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastKFPVRId)));
	    eNSPVR->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FramePVRId)));
	    eNSPVR->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastKFBiasId)));
	    eNSPVR->setMeasurement(imupreint);
	    
	    Matrix9d InvCovPVR = imupreint.getCovPVPhi().inverse();
	    eNSPVR->setInformation(InvCovPVR);
	    
	    eNSPVR->SetParams(GravityVec);
	    
	    const float thHuberNavStatePVR = sqrt(21.666);
	    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
	    eNSPVR->setRobustKernel(rk);
	    rk->setDelta(thHuberNavStatePVR);
	    
	    optimizer.addEdge(eNSPVR);
	}
	
	// 边 KF-Frame Bias
	g2o::EdgeNavStateBias *eNSBias = new g2o::EdgeNavStateBias();
	{
	    eNSBias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastKFBiasId)));
	    eNSBias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FrameBiasId)));
	    eNSBias->setMeasurement(imupreint);
	    
	    Matrix<double, 6, 6> InvCovBgaRW = Matrix<double,6,6>::Identity();
	    InvCovBgaRW.topLeftCorner(3,3) = Matrix3d::Identity()/IMUData::getGyrBiasRW2();
	    InvCovBgaRW.bottomRightCorner(3,3) = Matrix3d::Identity()/IMUData::getAccBiasRW2();
	    eNSBias->setInformation(InvCovBgaRW/imupreint.getDeltaTime());
	    
	    const float thHuberNavStateBias = sqrt(16.812);
	    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
	    eNSBias->setRobustKernel(rk);
	    rk->setDelta(thHuberNavStateBias);
	    
	    optimizer.addEdge(eNSBias);
	}
	
	const int N = pFrame->N;
	
	vector<g2o::EdgeNavStatePVRPointXYZOnlyPose *> vpEdgesMono;
	vector<size_t> vnIndexEdgeMono;
	vpEdgesMono.reserve(N);
	vnIndexEdgeMono.reserve(N);
	
	const float deltaMono = sqrt(5.991);
	
	
	// 投影误差边
	{
	    unique_lock<mutex> lock(MapPoint::mGlobalMutex);
	    
	    for(int i=0; i<N; i++)
	    {
		MapPoint *pMP = pFrame->mvpMapPoints[i];
		
		if(pMP)
		{
		    if(pFrame->mvuRight[i]<0)
		    {
			nInitialCorrespondences++;
			pFrame->mvbOutlier[i] = false;
			
			Eigen::Matrix<double,2,1> obs;
			const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
			obs << kpUn.pt.x, kpUn.pt.y;
			
			g2o::EdgeNavStatePVRPointXYZOnlyPose *e = new g2o::EdgeNavStatePVRPointXYZOnlyPose();
			
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FramePVRId)));
			e->setMeasurement(obs);
			const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
			e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
			
			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(deltaMono);
			
			e->SetParams(pFrame->fx, pFrame->fy, pFrame->cx, pFrame->cy, Rbc, Pbc, Converter::toVector3d(pMP->GetWorldPos()));
			
			optimizer.addEdge(e);
			
			vpEdgesMono.push_back(e);
			vnIndexEdgeMono.push_back(i);
		    }
		    else
		    {
			cerr<<"stereo shouldn't in poseoptimization"<<endl;
		    }
		}
	    }
	    
	}
	
	
	if(nInitialCorrespondences < 3)
	    return 0;
	
	// 执行4次优化，每次执行完成后，分类内点外点，下次优化时，外点不包含，但是优化之后，所有点重新分类
	const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
	const int its[4] = {10, 10, 10, 10};
	
	int nBad = 0;
	for(size_t it=0; it<4; it++)
	{
	    
	    // 重置估计值
	    vNSFPVR->setEstimate(pFrame->GetNavState());
	    vNSFBias->setEstimate(pFrame->GetNavState());
	    
	    // 优化
	    optimizer.initializeOptimization(0);
	    optimizer.optimize(its[it]);
	    
	    nBad = 0;
	    for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
	    {
		
		g2o::EdgeNavStatePVRPointXYZOnlyPose *e = vpEdgesMono[i];
		const size_t idx = vnIndexEdgeMono[i];
		
		if(pFrame->mvbOutlier[idx])
		{
		    e->computeError();
		}
		
		const float chi2 = e->chi2();
		
		if(chi2 > chi2Mono[it])
		{
		    pFrame->mvbOutlier[idx] = true;
		    e->setLevel(1);
		    nBad++;
		}
		else
		{
		    pFrame->mvbOutlier[idx]  = false;
		    e->setLevel(0);
		}
		
		if(it==2)
		    e->setRobustKernel(0);
	    }
	    
	    if(optimizer.edges().size()<10)
		break;    
	}
	
	// 更新优化后的状态
	g2o::VertexNavStatePVR *vNSPVR_recov = static_cast<g2o::VertexNavStatePVR *>(optimizer.vertex(FramePVRId));
	const NavState &nsPVR_recov = vNSPVR_recov->estimate();
	g2o::VertexNavStateBias *vNSBias_recov = static_cast<g2o::VertexNavStateBias *>(optimizer.vertex(FrameBiasId));
	const NavState &nsBias_recov = vNSBias_recov->estimate();

	NavState ns_recov = nsPVR_recov;
	ns_recov.Set_DeltaBiasGyr(nsBias_recov.Get_dBias_Gyr());
	ns_recov.Set_DeltaBiasAcc(nsBias_recov.Get_dBias_Acc());
	pFrame->SetNavState(ns_recov);
	pFrame->UpdatePoseFromNS(ConfigParam::GetMatTbc());
	
	
	// Compute marginalized Hessian H and B, H*x=B
	// H/B can be used as prior for next optimization in PoseOptimization
	if(bComputeMarg)
	{
	    std::vector<g2o::OptimizableGraph::Vertex *> margVerteces;
	    margVerteces.push_back(optimizer.vertex(FramePVRId));
	    margVerteces.push_back(optimizer.vertex(FrameBiasId));
	    
	    g2o::SparseBlockMatrixXd spinv;
	    optimizer.computeMarginals(spinv, margVerteces);
	    
	    Matrix<double,15,15> margCovInv = Matrix<double,15,15>::Zero();
	    margCovInv.topLeftCorner(9,9) = spinv.block(0,0)->inverse();
	    margCovInv.bottomRightCorner(6,6) = spinv.block(1,1)->inverse();
	    pFrame->mMargCovInv = margCovInv;
	    pFrame->mNavStatePrior = ns_recov;
	    
	}
	
	// test log
	if( (nsPVR_recov.Get_BiasGyr()-nsBias_recov.Get_BiasGyr()).norm()>1e-6 || (nsPVR_recov.Get_BiasAcc()-nsBias_recov.Get_BiasAcc()).norm()>1e-6 )
	{
	    std::cerr<<"recovered bias gyr not equal for PVR/Bias vertex"<<std::endl<<nsPVR_recov.Get_BiasGyr().transpose()<<" / "<<nsBias_recov.Get_BiasGyr().transpose()<<std::endl;
	    std::cerr<<"recovered bias acc not equal for PVR/Bias vertex"<<std::endl<<nsPVR_recov.Get_BiasAcc().transpose()<<" / "<<nsBias_recov.Get_BiasAcc().transpose()<<std::endl;
	}
	
	if( (ns_recov.Get_dBias_Gyr()-nsBias_recov.Get_dBias_Gyr()).norm()>1e-6 || (ns_recov.Get_dBias_Acc()-nsBias_recov.Get_dBias_Acc()).norm()>1e-6 )
	{
	    std::cerr<<"recovered delta bias gyr not equal to Bias vertex"<<std::endl<<ns_recov.Get_dBias_Gyr().transpose()<<" / "<<nsBias_recov.Get_dBias_Gyr().transpose()<<std::endl;
	    std::cerr<<"recovered delta bias acc not equal to Bias vertex"<<std::endl<<ns_recov.Get_dBias_Acc().transpose()<<" / "<<nsBias_recov.Get_dBias_Acc().transpose()<<std::endl;
	}
	
	return nInitialCorrespondences-nBad;
	
	
    }


    
    
    void Optimizer::LocalBundleAdjustmentNavState(KeyFrame *pCurKF, const std::list<KeyFrame*> &lLocalKeyFrames, bool* pbStopFlag, Map* pMap, cv::Mat& gw, LocalMapping* pLM)
    {
	
	// 检查CKF是否在local window中
	if(pCurKF != lLocalKeyFrames.back())
	    cerr<<"pCurKF != lLocalKeyFrames.back. check"<<endl;		// test log
	
	// 外参
	Matrix4d Tbc = ConfigParam::GetEigTbc();
	Matrix3d Rbc = Tbc.topLeftCorner(3,3);
	Vector3d Pbc = Tbc.topRightCorner(3,1);
	Vector3d GravityVec  = Converter::toVector3d(gw);
	
	// local window中的所有关键帧参与优化
	for(list<KeyFrame*>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKFi = *lit;
	    pKFi->mnBALocalForKF = pCurKF->mnId;
	}
	
	// local KF可以观测到的所有MP
	list<MapPoint *> lLocalMapPoints;
	for(list<KeyFrame *>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
	    
	    for(vector<MapPoint *>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
	    {
		MapPoint *pMP = *vit;
		if(pMP)
		    if(!pMP->isBad())
			if(pMP->mnBALocalForKF!=pCurKF->mnId)
			{
			    lLocalMapPoints.push_back(pMP);
			    pMP->mnBALocalForKF = pCurKF->mnId;
			}
		
	    }
	    
	}
	
	// 固定的KF
	list<KeyFrame *> lFixedCameras;
	KeyFrame *pKFPrevLocal = lLocalKeyFrames.front()->GetPrevKeyFrame();
	if(pKFPrevLocal)
	{
	    // test log
	    if(pKFPrevLocal->isBad())
		cerr<<"KeyFrame before local window is bad?"<<endl;
	    if(pKFPrevLocal->mnBAFixedForKF==pCurKF->mnId)
		cerr<<"KeyFrame before local, has been added to lFixedKF?"<<endl;
	    if(pKFPrevLocal->mnBALocalForKF==pCurKF->mnId)
		cerr<<"KeyFrame before local, has been added to lLocalKF?"<<endl;
	    
	    pKFPrevLocal->mnBAFixedForKF = pCurKF->mnId;
	    if(!pKFPrevLocal->isBad())
		lFixedCameras.push_back(pKFPrevLocal);
	    else
		cerr<<"pKFPrevLocal is Bad?"<<endl;   
	}
	else
	    cerr<<"pKFPrevLocal is NULL?"<<endl;
	
	// 共视KF
	for(list<MapPoint *>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
	{
	    mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = (*lit)->GetObservations();
	    for(mapMapPointObs/*map<KeyFrame*,size_t>*/::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
	    {
		KeyFrame *pKFi = mit->first;
		
		if(pKFi->mnBALocalForKF!=pCurKF->mnId && pKFi->mnBAFixedForKF!=pCurKF->mnId)
		{
		    pKFi->mnBAFixedForKF = pCurKF->mnId;
		    if(!pKFi->isBad())
			lFixedCameras.push_back(pKFi);
		}
	    }
	}
	
	// 设置优化器
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX:: LinearSolverType *linearSolver;
	
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
	
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);
	
	if(pbStopFlag)
	    optimizer.setForceStopFlag(pbStopFlag);
	
	int maxKFid = 0;
	
	// 顶点 local KF
	for(list<KeyFrame *>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKFi = *lit;
	    int idKF = pKFi->mnId*2;
	    
	    // 顶点 PVR
	    {
		
		g2o::VertexNavStatePVR *vNSPVR = new g2o::VertexNavStatePVR();
		vNSPVR->setEstimate(pKFi->GetNavState());
		vNSPVR->setId(idKF);
		vNSPVR->setFixed(false);
		optimizer.addVertex(vNSPVR);
	    }
	    
	    
	    // 顶点 bias
	    {
		g2o::VertexNavStateBias *vNSBias = new g2o::VertexNavStateBias();
		vNSBias->setEstimate(pKFi->GetNavState());
		vNSBias->setId(idKF+1);
		vNSBias->setFixed(false);
		optimizer.addVertex(vNSBias);
	    }
	    
	    if(idKF+1 > maxKFid)
		maxKFid = idKF+1;
		// test
		if(pKFi->mnId == 0) 
		cerr<<"pKFi->mnId == 0, shouldn't in LocalBA of NavState"<<endl;
	}
	
	// 顶点 fixed KF
	for(list<KeyFrame *>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKFi = *lit;
	    int idKF = pKFi->mnId*2;
	    
	    {
		g2o::VertexNavStatePVR *vNSPVR = new g2o::VertexNavStatePVR();
		vNSPVR->setEstimate(pKFi->GetNavState());
		vNSPVR->setId(idKF);
		vNSPVR->setFixed(true);
		optimizer.addVertex(vNSPVR);
	    }
	    
	    
	    // local window之前的KF
	    if(pKFi == pKFPrevLocal)
	    {
		
		g2o::VertexNavStateBias *vNSBias = new g2o::VertexNavStateBias();
		vNSBias->setEstimate(pKFi->GetNavState());
		vNSBias->setId(idKF+1);
		vNSBias->setFixed(true);
		optimizer.addVertex(vNSBias);
	    }
	    
	    if(idKF+1 > maxKFid)
		maxKFid = idKF+1;
	}
	
	
	vector<g2o::EdgeNavStatePVR *> vpEdgesNavStatePVR;
	vector<g2o::EdgeNavStateBias *> vpEdgesNavStateBias;
	
	const float thHuberNavStatePVR = sqrt(100*21.666);
	const float thHuberNavStateBias = sqrt(100*16.812);
	
	Matrix<double,6,6> InvCovBgaRW = Matrix<double,6,6>::Identity();
	InvCovBgaRW.topLeftCorner(3,3) = Matrix3d::Identity()/IMUData::getGyrBiasRW2();
	InvCovBgaRW.bottomRightCorner(3,3) = Matrix3d::Identity()/IMUData::getAccBiasRW2();
	
	for(list<KeyFrame *>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKF1 = *lit;
	    KeyFrame *pKF0 = pKF1->GetPrevKeyFrame();
	    
	   
	    
	    // 边PVR
	    {
		g2o::EdgeNavStatePVR *epvr = new g2o::EdgeNavStatePVR();
		epvr->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2*pKF0->mnId)));
		epvr->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2*pKF1->mnId)));
		epvr->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2*pKF0->mnId+1)));
		epvr->setMeasurement(pKF1->GetIMUPreInt());
		
		Matrix9d InvCovPVR = pKF1->GetIMUPreInt().getCovPVPhi().inverse();
		epvr->setInformation(InvCovPVR);

		
		epvr->SetParams(GravityVec);
		
		g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
		epvr->setRobustKernel(rk);
		rk->setDelta(thHuberNavStatePVR);
		
		optimizer.addEdge(epvr);
		vpEdgesNavStatePVR.push_back(epvr);
	    }
	    
	    // 边 bias
	    {
		g2o::EdgeNavStateBias *ebias = new g2o::EdgeNavStateBias();
		ebias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2*pKF0->mnId+1)));
		ebias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2*pKF1->mnId+1)));
		ebias->setMeasurement(pKF1->GetIMUPreInt());
		
		ebias->setInformation(InvCovBgaRW/pKF1->GetIMUPreInt().getDeltaTime());
		
		g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
		ebias->setRobustKernel(rk);
		rk->setDelta(thHuberNavStateBias);
		
		optimizer.addEdge(ebias);
		vpEdgesNavStateBias.push_back(ebias);
	    }
	    
	    // test log
	    if(pKF1->GetIMUPreInt().getDeltaTime() < 1e-3)
	    {
		cerr<<"IMU pre-integrator delta time between 2 KFs too small: "<<pKF1->GetIMUPreInt().getDeltaTime()<<endl;
		cerr<<"No EdgeNavState added"<<endl;
		continue;
	    }
	    
	    if(lit == lLocalKeyFrames.begin())
	    {
		// First KF in Local Window, link (fixed) pKFPrevLocal
		if(pKF0 != pKFPrevLocal) cerr<<"pKF0 != pKFPrevLocal for 1st KF in Local Window, id: "<<pKF0->mnId<<","<<pKFPrevLocal->mnId<<endl;
		
	    }
	    else
	    {
		// KFs in Local Window, link another local KF
	    }
	    
	}
	
	// 顶点 MP
	const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();
	
	vector<g2o::EdgeNavStatePVRPointXYZ *> vpEdgesMono;
	vpEdgesMono.reserve(nExpectedSize);
	
	vector<KeyFrame *> vpEdgeKFMono;
	vpEdgeKFMono.reserve(nExpectedSize);
	
	vector<MapPoint *> vpMapPointEdgeMono;
	vpMapPointEdgeMono.reserve(nExpectedSize);
	
	
	const float thHuberMono = sqrt(5.991);
	
	for(list<MapPoint *>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
	{
	    MapPoint *pMP = *lit;
	    g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
	    vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
	    int id = pMP->mnId+maxKFid+1;
	    vPoint->setId(id);
	    vPoint->setMarginalized(true);
	    optimizer.addVertex(vPoint);
	    
	    
	    cv::Mat Pw = pMP->GetWorldPos();
	    
	    const mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = pMP->GetObservations();
	    
	    for(mapMapPointObs/*map<KeyFrame*,size_t>*/::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
	    {
		KeyFrame *pKFi = mit->first;
		
		if(!pKFi->isBad())
		{
		    
		    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];
		    
		    if(pKFi->mvuRight[mit->second] < 0)
		    {
			Eigen::Matrix<double, 2, 1> obs;
			obs << kpUn.pt.x, kpUn.pt.y;
			
			g2o::EdgeNavStatePVRPointXYZ *e = new g2o::EdgeNavStatePVRPointXYZ();
			
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2*pKFi->mnId)));
			e->setMeasurement(obs);
			const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
			
			e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
			
			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(thHuberMono);
			
			e->SetParams(pKFi->fx, pKFi->fy, pKFi->cx, pKFi->cy, Rbc, Pbc);
			
			optimizer.addEdge(e);
			vpEdgesMono.push_back(e);
			vpEdgeKFMono.push_back(pKFi);
			vpMapPointEdgeMono.push_back(pMP);
			
		    }
		    else
		    {
			cerr<<"Stereo not supported yet, why here?? check."<<endl;
		    }   
		}
	    }
	    
	}
	
	if(pbStopFlag)
	    if(*pbStopFlag)
		return ;
	
	optimizer.initializeOptimization();
	optimizer.optimize(5);
	
	bool bDoMore=true;
	
	if(pbStopFlag)
	    if(*pbStopFlag)
		bDoMore = false;
	
	if(bDoMore)
	{
	    // 检查内点
	    for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
	    {
		g2o::EdgeNavStatePVRPointXYZ *e = vpEdgesMono[i];
		MapPoint *pMP = vpMapPointEdgeMono[i];
		
		if(pMP->isBad())
		    continue;
		
		if(e->chi2()>5.991 || !e->isDepthPositive())
		{
		    e->setLevel(1);
		}
		
		e->setRobustKernel(0);
	    }
	    
	    optimizer.initializeOptimization(0);
	    optimizer.optimize(10);
	    
	}
	
	vector<pair<KeyFrame*, MapPoint *>> vToErase;
	vToErase.reserve(vpEdgesMono.size());
	
	double PosePointchi2 = 0;
	
	// 内点记录
	for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
	{
	    g2o::EdgeNavStatePVRPointXYZ *e = vpEdgesMono[i];
	    MapPoint *pMP = vpMapPointEdgeMono[i];
	    
	    if(pMP->isBad())
		continue;
	    
	    if(e->chi2()>5.991 || !e->isDepthPositive())
	    {
		KeyFrame *pKFi = vpEdgeKFMono[i];
		vToErase.push_back(make_pair(pKFi, pMP));
	    }
	    
	    PosePointchi2 += e->chi2();
	}
	
	
	
	unique_lock<mutex> lock(pMap->mMutexMapUpdate);
	
	if(!vToErase.empty())
	{
	    for(size_t i=0; i<vToErase.size(); i++)
	    {
		KeyFrame *pKFi  = vToErase[i].first;
		MapPoint *pMPi = vToErase[i].second;
		pKFi->EraseMapPointMatch(pMPi);
		pMPi->EraseObservation(pKFi);
	    }
	}
	
	// 更新优化后的结果
	
	// KF
	for(list<KeyFrame *>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKFi = *lit;
	    g2o::VertexNavStatePVR *vNSPVR = static_cast<g2o::VertexNavStatePVR *>(optimizer.vertex(2*pKFi->mnId));
	    g2o::VertexNavStateBias *vNSBias = static_cast<g2o::VertexNavStateBias *>(optimizer.vertex(2*pKFi->mnId+1));
	    
	    const NavState &optPVRns = vNSPVR->estimate();
	    const NavState &optBiasns = vNSBias->estimate();
	    NavState primaryns = pKFi->GetNavState();
	    
	    // 更新导航状态
	    pKFi->SetNavStatePos(optPVRns.Get_P());
	    
	    pKFi->SetNavStateVel(optPVRns.Get_V());
		pKFi->SetNavStateRot(optPVRns.Get_R());
	    pKFi->SetNavStateDeltaBg(optBiasns.Get_dBias_Gyr());
	    pKFi->SetNavStateDeltaBa(optBiasns.Get_dBias_Acc());
	    
	    // 跟新相机位姿
	    pKFi-> UpdatePoseFromNS(ConfigParam::GetMatTbc());
	    
	    // test log
	    if( (primaryns.Get_BiasGyr() - optPVRns.Get_BiasGyr()).norm() > 1e-6 || (primaryns.Get_BiasGyr() - optBiasns.Get_BiasGyr()).norm() > 1e-6 )
		cerr<<"gyr bias change in optimization?"<<endl;
	    
	    if( (primaryns.Get_BiasAcc() - optPVRns.Get_BiasAcc()).norm() > 1e-6 || (primaryns.Get_BiasAcc() - optBiasns.Get_BiasAcc()).norm() > 1e-6 )
		cerr<<"acc bias change in optimization?"<<endl;
	    
	}
	
	
	// MPs
	for(list<MapPoint *>::const_iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
	{
	    MapPoint *pMP = *lit;
	    g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId+maxKFid+1));
	    pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
	    pMP->UpdateNormalAndDepth();
	}
	
	if(pLM)
	{
	    pLM->SetMapUpdateFlagInTracking(true);
	}
	
    }
    
    
    
    // 
    Vector3d Optimizer::OptimizeInitialGyroBias(const std::vector<Frame> &vFrames)
    {
	
	Matrix4d Tbc = ConfigParam::GetEigTbc();
	Matrix3d Rcb = Tbc.topLeftCorner(3,3).transpose();
	
	// 初始化优化器
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
	
	g2o::BlockSolverX *solver_ptr =  new g2o::BlockSolverX(linearSolver);
	g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
	optimizer.setAlgorithm(solver);
	
	
	// 顶点 Gyro Bias
	g2o::VertexGyrBias *vBiasg = new g2o::VertexGyrBias();
	vBiasg->setEstimate(Eigen::Vector3d::Zero());
	vBiasg->setId(0);
	optimizer.addVertex(vBiasg);
	
	// 一元边，gyro bias
	for(size_t i=0; i<vFrames.size(); i++)
	{
	    if(i==0)
		continue;
	    
	    const Frame &Fi = vFrames[i-1];
	    const Frame &Fj = vFrames[i];
	    
	    cv::Mat Tiw = Fi.mTcw;
	    Eigen::Matrix3d Rwci = Converter::toMatrix3d(Tiw.rowRange(0,3).colRange(0,3).t());
	    
	    cv::Mat Tjw = Fj.mTcw;
	    Eigen::Matrix3d Rwcj = Converter::toMatrix3d(Tjw.rowRange(0,3).colRange(0,3).t());
	    
	    
	    IMUPreintegrator imupreint;
	    Fj.ComputeIMUPreIntSinceLastFrame(&Fi, imupreint);
	    
	    g2o::EdgeGyrBias *eBiasg = new g2o::EdgeGyrBias();
	    eBiasg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
	    
	    // 误差函数没有用到Measurement
	    // 设置EdgeGyrBias成员参数
	    eBiasg->dRbij = imupreint.getDeltaR();
	    eBiasg->J_dR_bg = imupreint.getJRBiasg();
	    eBiasg->Rwbi = Rwci*Rcb;
	    eBiasg->Rwbj = Rwcj*Rcb;
	    eBiasg->setInformation(Eigen::Matrix3d::Identity());
	    optimizer.addEdge(eBiasg);
	}
	
	// 开始优化
	optimizer.initializeOptimization();
	optimizer.optimize(1);
	
	g2o::VertexGyrBias *vBgEst = static_cast<g2o::VertexGyrBias *>(optimizer.vertex(0));
	
	return vBgEst->estimate();
    }
    
    
    
    // 
    Vector3d Optimizer::OptimizeInitialGyroBias(const std::list<KeyFrame *> &lLocalKeyFrames)
    {
	return OptimizeInitialGyroBias(std::vector<KeyFrame *>(lLocalKeyFrames.begin(), lLocalKeyFrames.end()));
    }

    
    
    // 
    Vector3d Optimizer::OptimizeInitialGyroBias(const std::vector<KeyFrame *> &vpKFs)
    {
	Matrix4d Tbc = ConfigParam::GetEigTbc();
	Matrix3d Rcb = Tbc.topLeftCorner(3,3).transpose();
	
	//构造优化器
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
	
	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
	g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
	optimizer.setAlgorithm(solver);
	
	g2o::VertexGyrBias *vBiasg = new g2o::VertexGyrBias();
	vBiasg->setEstimate(Eigen::Vector3d::Zero());
	vBiasg->setId(0);
	optimizer.addVertex(vBiasg);
	
	KeyFrame *pPrevKF0 = vpKFs.front();
	for(std::vector<KeyFrame *>::const_iterator lit=vpKFs.begin(), lend=vpKFs.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKF = *lit;   
	    
	    if(pKF == vpKFs.front())
		continue;
	    
	    KeyFrame *pPrevKF = pKF->GetPrevKeyFrame();
	    cv::Mat Twi = pPrevKF->GetPoseInverse();
	    Eigen::Matrix3d Rwci = Converter::toMatrix3d(Twi.rowRange(0,3).colRange(0,3));
	    
	    cv::Mat Twj = pKF->GetPoseInverse();
	    Eigen::Matrix3d Rwcj = Converter::toMatrix3d(Twj.rowRange(0,3).colRange(0,3));
	    
	    const IMUPreintegrator &imupreint = pKF->GetIMUPreInt();
	    g2o::EdgeGyrBias *eBiasg = new g2o::EdgeGyrBias();
	    eBiasg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
	    
	    eBiasg->dRbij = imupreint.getDeltaR();
	    eBiasg->J_dR_bg = imupreint.getJRBiasg();
	    eBiasg->Rwbi = Rwci*Rcb;
	    eBiasg->Rwbj = Rwcj*Rcb;
	    eBiasg->setInformation(Eigen::Matrix3d::Identity());
	    optimizer.addEdge(eBiasg);
	    
	    // test log
	    if(pPrevKF0!=pPrevKF)
		cerr<<"pPrevKF in list != pKF->pPrevKF? in OptimizeInitialGyroBias"<<endl;    
	    pPrevKF0 = pKF;
	    
	}
	
	optimizer.initializeOptimization();
	optimizer.optimize(1);
	
	g2o::VertexGyrBias *vBgEst = static_cast<g2o::VertexGyrBias *>(optimizer.vertex(0));
	
	return vBgEst->estimate();
	
    }
    
    
    
    // 
    Vector3d Optimizer::OptimizeInitialGyroBias(const vector<cv::Mat>& vTwc, const vector<IMUPreintegrator>& vImuPreInt)
    {
	int N= vTwc.size();
	if(vTwc.size()!=vImuPreInt.size())
	    cerr<<"vTwc.size()!=vImuPreInt.size()"<<endl;
	
	Matrix4d Tbc = ConfigParam::GetEigTbc();
	Matrix3d Rcb = Tbc.topLeftCorner(3,3).transpose();
	
	// 构造优化器
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
	
	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
	g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
	optimizer.setAlgorithm(solver);
	
	// 顶点
	g2o::VertexGyrBias *vBiasg = new g2o::VertexGyrBias();
	vBiasg->setEstimate(Eigen::Vector3d::Zero());
	vBiasg->setId(0);
	optimizer.addVertex(vBiasg);
	
	// 1元边
	for(int i=0; i< N; i++)
	{
	    if(i==0)
		continue;
	    
	    // prevKF pose
	    const cv::Mat &Twi = vTwc[i-1];
	    Matrix3d Rwci = Converter::toMatrix3d(Twi.rowRange(0,3).colRange(0,3));
	    
	    // KF pose
	    const cv::Mat &Twj = vTwc[i];
	    Matrix3d Rwcj = Converter::toMatrix3d(Twj.rowRange(0,3).colRange(0,3));
	    
	    const IMUPreintegrator &imupreint = vImuPreInt[i];
	    
	    g2o::EdgeGyrBias *eBiasg = new g2o::EdgeGyrBias();
	    eBiasg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
	    
	    // 设置误差边成员参数
	    eBiasg->dRbij = imupreint.getDeltaR();
	    eBiasg->J_dR_bg = imupreint.getJRBiasg();
	    eBiasg->Rwbi = Rwci*Rcb;
	    eBiasg->Rwbj = Rwcj*Rcb;
	    
	    eBiasg->setInformation(imupreint.getCovPVPhi().bottomRightCorner(3,3).inverse());
	    optimizer.addEdge(eBiasg);
	    
	}
	
	optimizer.initializeOptimization();
	optimizer.optimize(1);
	
	g2o::VertexGyrBias *vBgEst = static_cast<g2o::VertexGyrBias *>(optimizer.vertex(0));
	
	return vBgEst->estimate();
	
    }
    
    
    
    //  
    void Optimizer::LocalBundleAdjustment(KeyFrame* pKF, const std::list< KeyFrame* >& lLocalKeyFrames, bool* pbStopFlag, Map* pMap, LocalMapping* pLM)
    {
	
	// 检查CKF是否在local
	if(pKF != lLocalKeyFrames.back())
	    cerr<<"pKF != lLocalKeyFrames.back. check"<<endl;
	
	// 局部KF
	for(list<KeyFrame *>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKFi = *lit;
	    pKFi->mnBALocalForKF = pKF->mnId;
	}
	
	// 局部MP
	list<MapPoint *> lLocalMapPoints;
	for(list<KeyFrame *>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
	    for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
	    {
		MapPoint *pMP = *vit;
		if(pMP)
		    if(!pMP->isBad())
			if(pMP->mnBALocalForKF!=pKF->mnId)
			{
			    lLocalMapPoints.push_back(pMP);
			    pMP->mnBALocalForKF = pKF->mnId;
			}
	    }
	}
	
	// 固定KF
	list<KeyFrame *> lFixedCameras;
	KeyFrame *pKFPrevLocal = lLocalKeyFrames.front()->GetPrevKeyFrame();
	if(pKFPrevLocal)
	{
	    if(pKFPrevLocal->isBad()) 
		cerr<<"KeyFrame before local window is bad?"<<endl;
	    
	    if(pKFPrevLocal->mnBAFixedForKF==pKF->mnId) 
		cerr<<"KeyFrame before local, has been added to lFixedKF?"<<endl;
	    
	    if(pKFPrevLocal->mnBALocalForKF==pKF->mnId) 
		cerr<<"KeyFrame before local, has been added to lLocalKF?"<<endl;
	    
	    pKFPrevLocal->mnBAFixedForKF = pKF->mnId;
	    
	    if(!pKFPrevLocal->isBad())
		lFixedCameras.push_back(pKFPrevLocal);
	    else
		cerr<<"pKFPrevLocal is Bad?"<<endl;
	    
	}
	
	// 共视KF
	for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
	{
	    mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = (*lit)->GetObservations();
	    
	    for(mapMapPointObs/*map<KeyFrame*,size_t>*/::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
	    {
		KeyFrame *pKFi = mit->first;
		
		if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
		{
		    pKFi->mnBAFixedForKF = pKF->mnId;
		    if(!pKFi->isBad())
			lFixedCameras.push_back(pKFi);
		}
		
	    }
	    
	}
	
	// 构造优化器
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
	
	g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);
	
	if(pbStopFlag)
	    optimizer.setForceStopFlag(pbStopFlag);
	
	unsigned long maxKFid = 0;
	
	// 顶点 Local KF
	for(list<KeyFrame *>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKFi = *lit;
	    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
	    vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
	    vSE3->setId(pKFi->mnId);
	    vSE3->setFixed(pKFi->mnId == 0);
	    optimizer.addVertex(vSE3);
	    
	    if(pKFi->mnId > maxKFid)
		maxKFid = pKFi->mnId;    
	}
	
	// 顶点 fixed KF
	for(list<KeyFrame *>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKFi = *lit;
	    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
	    vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
	    vSE3->setId(pKFi->mnId);
	    vSE3->setFixed(true);
	    optimizer.addVertex(vSE3);
	    
	    if(pKFi->mnId > maxKFid)
		maxKFid = pKFi->mnId;   
	}
	
	
	
	const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();
	
	vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
	vpEdgesMono.reserve(nExpectedSize);

	vector<KeyFrame*> vpEdgeKFMono;
	vpEdgeKFMono.reserve(nExpectedSize);

	vector<MapPoint*> vpMapPointEdgeMono;
	vpMapPointEdgeMono.reserve(nExpectedSize);

	
	vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
	vpEdgesStereo.reserve(nExpectedSize);
	
	vector<KeyFrame*> vpEdgeKFStereo;
	vpEdgeKFStereo.reserve(nExpectedSize);
	
	vector<MapPoint*> vpMapPointEdgeStereo;
	vpMapPointEdgeStereo.reserve(nExpectedSize);
	
	const float thHuberMono = sqrt(5.991);
	const float thHuberStereo = sqrt(7.815);
	
	
	for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
	{
	    // 顶点MP
	    MapPoint *pMP = *lit;
	    g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
	    vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
	    int id = pMP->mnId+maxKFid+1;
	    vPoint->setId(id);
	    vPoint->setMarginalized(true);
	    optimizer.addVertex(vPoint);
	    
	    const mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = pMP->GetObservations();
	    for(mapMapPointObs/*map<KeyFrame*,size_t>*/::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
	    {
		KeyFrame *pKFi = mit->first;
		
		if(!pKFi->isBad())
		{
		    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];
		    
		    // 投影误差边
		    // 单目
		    if(pKFi->mvuRight[mit->second] < 0)
		    {
			Eigen::Matrix<double,2,1> obs;
			obs << kpUn.pt.x, kpUn.pt.y;
			
			g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();
			
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
			e->setMeasurement(obs);
			const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
			e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
			
			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(thHuberMono);
			
			e->fx = pKFi->fx;
			e->fy = pKFi->fy;
			e->cx = pKFi->cx;
			e->cy = pKFi->cy;
			
			optimizer.addEdge(e);
			vpEdgesMono.push_back(e);
			vpEdgeKFMono.push_back(pKFi);
			vpMapPointEdgeMono.push_back(pMP);
		    }
		    else
		    {
			Eigen::Matrix<double,3,1> obs;
			const float kp_ur = pKFi->mvuRight[mit->second];
			obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
			
			g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();
			
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
			e->setMeasurement(obs);
			const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
			Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
			e->setInformation(Info);
			
			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(thHuberStereo);
			
			e->fx = pKFi->fx;
			e->fy = pKFi->fy;
			e->cx = pKFi->cx;
			e->cy = pKFi->cy;
			e->bf = pKFi->mbf;
			
			optimizer.addEdge(e);
			vpEdgesStereo.push_back(e);
			vpEdgeKFStereo.push_back(pKFi);
			vpMapPointEdgeStereo.push_back(pMP);
		    }    
		}
	    }
	    
	}
	
	if(pbStopFlag)
	    if(*pbStopFlag)
		return ;
	    
	optimizer.initializeOptimization();
	optimizer.optimize(5);
	
	bool bDoMore = true;
	
	if(pbStopFlag)
	    if(*pbStopFlag)
		bDoMore = false;

	if(bDoMore)
	{
	    
	    // 检查内点
	    for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
	    {
		g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
		
		MapPoint *pMP = vpMapPointEdgeMono[i];
		
		if(pMP->isBad())
		    continue;
		
		if(e->chi2() > 5.991 || !e->isDepthPositive())
		{
		    e->setLevel(1);
		}
		
		e->setRobustKernel(0);
	    }
	    
	    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
	    {
		g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
		MapPoint* pMP = vpMapPointEdgeStereo[i];
		
		if(pMP->isBad())
		    continue;
		
		if(e->chi2()>7.815 || !e->isDepthPositive())
		{
		    e->setLevel(1);
		}
		
		e->setRobustKernel(0);
	    }
	    
	    optimizer.initializeOptimization(0);
	    optimizer.optimize(10);
	}
	
	vector<pair<KeyFrame *, MapPoint *>> vToErase;
	vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());
	
	for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
	{
	    g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
	    MapPoint *pMP = vpMapPointEdgeMono[i];
	    
	    if(pMP->isBad())
		continue;
	    
	    if(e->chi2()>5.991 || !e->isDepthPositive())
	    {
		KeyFrame* pKFi = vpEdgeKFMono[i];
		vToErase.push_back(make_pair(pKFi,pMP));
	    }
	}
	
	for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
	{
	    g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
	    MapPoint* pMP = vpMapPointEdgeStereo[i];
	    
	    if(pMP->isBad())
		continue;
	    
	    if(e->chi2()>7.815 || !e->isDepthPositive())
	    {
		KeyFrame* pKFi = vpEdgeKFStereo[i];
		vToErase.push_back(make_pair(pKFi,pMP));
	    }
	}
	
	unique_lock<mutex> lock(pMap->mMutexMapUpdate);
	
	if(!vToErase.empty())
	{
	    for(size_t i=0;i<vToErase.size(); i++)
	    {
		KeyFrame *pKFi = vToErase[i].first;
		MapPoint *pMPi = vToErase[i].second;
		pKFi->EraseMapPointMatch(pMPi);
		pMPi->EraseObservation(pKFi);
	    }
	}
	
	// 更新优化数据
	
	// KF
	for(list<KeyFrame *>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
	{
	    KeyFrame *pKF = *lit;
	    g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
	    g2o::SE3Quat SE3quat = vSE3->estimate();
	    pKF->SetPose(Converter::toCvMat(SE3quat));
	}
	
	// MP
	for(list<MapPoint *>::const_iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
	{
	    MapPoint *pMP = *lit;
	    g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId+maxKFid+1));
	    pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
	    pMP->UpdateNormalAndDepth();
	}
	
	if(pLM)
	{
	    pLM->SetMapUpdateFlagInTracking(true);
	}
	
    }

   
    
    /*******************************************************/

    // pMP中所有的MapPoints和关键帧进行全局BA优化。
    // 该GBA在ORB中的两个地方使用。
    // 1. 单目初始化，CreateInitialMapMonocular函数。
    // 2. 闭环完成后优化，RunGlobaBundleAdjustment函数。
    void Optimizer::GlobalBundleAdjustment(Map *pMap, int nIterations, bool  *pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
    {
        // 获取当前地图的所有关键帧和点云。
        vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        vector<MapPoint *> vpMP = pMap->GetAllMapPoints();
        BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);

    }



    /**
    * 3D-2D 最小化重投影误差，e =(u,v)-project(Tcw*Pw)
    * 1.  Vertex(顶点，优化变量)
    *       g2o::VertexSE3Expmap;       // 当前帧的Tcw。
    *       g2o::VertexSBAPointXYZ;     // 当前帧的世界坐标系下的地图点云。
    * 2.  Edge(边，目标函数)
    *       g2o::EdgeSE3ProjectXYZ, BaseBinaryEdge。
    *           连接顶点：待优化当前帧Tcw。
    *                 待优化世界坐标系下的MapPoint。
    *           测量值：  MapPoint在当前帧中对应的图像坐标(u,v)。
    *           信息矩阵：invSigma2(与特征点所在的尺度有关)。
    *
    * Param
    *       vpKFs               关键帧
    *       vpMP                MapPoints
    *       nIterations         迭代次数(20次)
    *       pbStopFlag          是否强制暂停
    *       nLoopKF             关键帧的个数
    *       bRobust             是否使用核函数(代替2范数)
    */
    void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                    int nIterations, bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
    {

        vector<bool> vbNotIncludedMP;
        vbNotIncludedMP.resize(vpMP.size());

        // 步骤1 初始化g2o优化器。
        // 构造求解器。
        g2o::SparseOptimizer optimizer;

        // 选择线性求解器的方法。
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

        // 6×3参数
        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        // L-M下降。
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        if(pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        long unsigned int maxKFid = 0;

        // 步骤2 向优化器添加顶点。

        // 步骤2.1 向优化器添加关键帧位姿顶点。
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if(pKF->isBad())
                continue;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));                // 设置估计量，即迭代初值。
            vSE3->setId(pKF->mnId);
            vSE3->setFixed(pKF->mnId==0);
            optimizer.addVertex(vSE3);
            if(pKF->mnId > maxKFid)
                maxKFid = pKF->mnId;
        }

        const float thHuber2D = sqrt(5.99);
        const float thHuber3D = sqrt(7.815);

        // 步骤2.2 向优化器添加MapPoint顶点。
        for(size_t i=0; i<vpMP.size(); i++)
        {
            MapPoint *pMP = vpMP[i];
            if(pMP->isBad())
                continue;
            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));         // 设置估计量，即迭代初值。
            const int id = pMP->mnId + maxKFid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            // 获取观测该点云的关键帧和点云在KF中的索引。
            const mapMapPointObs/*map<KeyFrame *, size_t>*/ observations = pMP->GetObservations();    

            int nEdges = 0;
            // 步骤3 向优化器添加投影边，即待优化的目标函数。
            // 遍历可以观测到该点云的关键帧，并查找对应的2D图像坐标。
            for(mapMapPointObs/*map<KeyFrame*, size_t>*/::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
            {
                KeyFrame *pKF = mit->first;
                if(pKF->isBad() || pKF->mnId > maxKFid)
                    continue;

                nEdges++;

                const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];
                
                // 单目或RGB-D。
                if(pKF->mvuRight[mit->second] < 0)
                {
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                    // 设置边连接的顶点。
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *> (optimizer.vertex(id)));    
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *> (optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);                                         // 设置观测值，即测量值，误差函数中的(u,v)。
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];    // 提取特征对应图像金字塔的尺度。
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);       // 信息矩阵。

                    // 是否使用核函数。
                    if(bRobust)
                    {
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber2D);
                    }

                    e->fx = pKF->fx;
                    e->fy = pKF->fy;
                    e->cx = pKF->cx;
                    e->cy = pKF->cy;

                    optimizer.addEdge(e);
                }

                // 双目。
                else
                {
                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKF->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    if(bRobust)
                    {
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber3D);
                    }

                    e->fx = pKF->fx;
                    e->fy = pKF->fy;
                    e->cx = pKF->cx;
                    e->cy = pKF->cy;
					e->bf = pKF->mbf;

                    optimizer.addEdge(e);
                }
            }

            if(nEdges == 0)
            {
                optimizer.removeVertex(vPoint);
                vbNotIncludedMP[i] = true;
            }
            else
            {
                vbNotIncludedMP[i] = false;
            }

        }

        // 步骤4 开始优化。
        optimizer.initializeOptimization();
        optimizer.optimize(nIterations);

        // 步骤5 得到优化结果。

        // 关键帧。
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if(pKF->isBad())
                continue;

            // 保存优化后结果。
            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
            g2o::SE3Quat SE3quat= vSE3->estimate();

            if(nLoopKF == 0)
            {
                pKF->SetPose(Converter::toCvMat(SE3quat));
            }
            else
            {
                pKF->mTcwGBA.create(4, 4, CV_32F);
                Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
                pKF->mnBAGlobalForKF = nLoopKF;
            }
        }

        // 地图点云。
        for(size_t i=0; i<vpMP.size(); i++)
        {
            if(vbNotIncludedMP[i])
                continue;

            MapPoint *pMP = vpMP[i];

            if(pMP->isBad())
                continue;

            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId+maxKFid+1));

            if(nLoopKF == 0)
            {
                pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
                pMP->UpdateNormalAndDepth();
            }
            else
            {
                pMP->mPosGBA.create(3, 1, CV_32F);
                Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
                pMP->mnBAGlobalForKF = nLoopKF;
            }
        }

    }



    /*
    * pose 图优化
    *   3D-2D最小化重投影误差 e = (u,v) - project(Tcw*Pw)
    *
    * Vertex(顶点，优化变量)
    *   g2o::VertexSE3Expmap()          // 当前帧的Tcw。
    * Edge(边，目标函数)
    *   g2o::EdgeSE3ProjectXYZOnlyPose, BaseUnaryEdge。
    *       连接顶点：待优化当前帧位姿Tcw。
    *       测量值：当前地图点云MP在当前帧尺度下的图像坐标(u,v)。
    *       信息矩阵：invSigma2(与特征点尺度有关)。
    *   g2o::EdgeStereoSE3ProjectXYZOnlyPose, BaseUnaryEdge。
    *       连接顶点：待优化当前帧位姿Tcw。
    *       测量值： MapPoint在当前帧中的图像坐标(ul,vl, ur)。
    *       信息矩阵：invSigma2(与特征点尺度有关)。
    *
    * Param     pFrame Frame
    * return    inliers数量。
    */
    // 该优化函数主要用于Tracking线程中，运动跟踪，参考帧跟踪，地图跟踪，重定位。
    int Optimizer::PoseOptimization(Frame *pFrame)
    {
        
        // 步骤1 构造g2o优化器。
        g2o::SparseOptimizer optimizer;         // 构造求解器

        // 选择线性模型的求解方法。
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
        // linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
	linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
        // 6×3参数设置。
        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        int nInitialCorrespondences = 0;

        // 步骤2 添加顶点：当前待优化变量Tcw。
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));      // 待优化变量的初始值。
        vSE3->setId(0);
        vSE3->setFixed(false);
        optimizer.addVertex(vSE3);

        // Frame特征点数量。
        const int N = pFrame->N;
        
        // 单目情况。
        vector<g2o::EdgeSE3ProjectXYZOnlyPose *> vpEdgesMono;        // 存储一元边。
        vector<size_t> vnIndexEdgeMono;                             // 存储内点的索引号。
        vpEdgesMono.reserve(N);
        vnIndexEdgeMono.reserve(N);

        // 双目情况。
        vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose *> vpEdgesStereo;
        vector<size_t> vnIndexEdgeStereo;
        vpEdgesStereo.reserve(N);
        vnIndexEdgeStereo.reserve(N);

        const float deltaMono = sqrt(5.991);
        const float deltaStereo = sqrt(7.815);

        // 步骤3 添加一元边：
        {
            unique_lock<mutex> lock(MapPoint::mGlobalMutex);

            for(int i=0; i<N; i++)
            {
                MapPoint *pMP = pFrame->mvpMapPoints[i];
                if(pMP)
                {

                    // 单目。
                    if(pFrame->mvuRight[i] < 0)
                    {
                        nInitialCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        Eigen::Matrix<double, 2, 1> obs;
                        const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                        obs << kpUn.pt.x, kpUn.pt.y;

                        g2o::EdgeSE3ProjectXYZOnlyPose *e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));     // 设置顶点。
                        e->setMeasurement(obs);                                                                 // 设置测量值(u,v)。
                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                        // 使用Huber核函数。
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(deltaMono);

                        e->fx = pFrame->fx;
                        e->fy = pFrame->fy;
                        e->cx = pFrame->cx;
                        e->cy = pFrame->cy;
                        cv::Mat Xw = pMP->GetWorldPos();
                        e->Xw[0] = Xw.at<float>(0);
                        e->Xw[1] = Xw.at<float>(1);
                        e->Xw[2] = Xw.at<float>(2);

                        optimizer.addEdge(e);

                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    }

                    // 双目
                    else
                    {

                        nInitialCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        Eigen::Matrix<double, 3, 1> obs;
                        const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                        const float &kp_ur = pFrame->mvuRight[i];
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                        e->setMeasurement(obs);
                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                        e->setInformation(Info);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(deltaStereo);

                        e->fx = pFrame->fx;
                        e->fy = pFrame->fy;
                        e->cx = pFrame->cx;
                        e->cy = pFrame->cy;
                        e->bf = pFrame->mbf;
                        cv::Mat Xw = pMP->GetWorldPos();
                        e->Xw[0] = Xw.at<float>(0);
                        e->Xw[1] = Xw.at<float>(1);
                        e->Xw[2] = Xw.at<float>(2);

                        optimizer.addEdge(e);

                        vpEdgesStereo.push_back(e);
                        vnIndexEdgeStereo.push_back(i);
                    }
                }
            }
        }

        // 初始特征点配对数量少于3，结束。
        if(nInitialCorrespondences < 3)
            return 0;


        // 步骤4 优化计算。
        // 总共进行4次优化， 每次优化后，将观测点分为outlier和inlier，outlier不参与下次优化。
        // 每次优化后重新判断所有观测点，之前的outlier可能变为inlier，反之亦然。
        // 基于卡方分布计算阈值(假设测量有一个像素偏差)。
        const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
        const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
        const int its[4] = {10, 10, 10, 10};                            // 迭代次数。

        int nBad = 0;
        for(size_t it=0; it<4; it++)
        {
            vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));      // 优化4次，每次优化设置初值。
            optimizer.initializeOptimization(0);                        // 对level为0进行优化。
            optimizer.optimize(its[it]);                                // 运行优化。

            nBad = 0;

            // 遍历所有的边。
            for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
            {
                g2o::EdgeSE3ProjectXYZOnlyPose *e = vpEdgesMono[i];
                const size_t idx = vnIndexEdgeMono[i];

                if(pFrame->mvbOutlier[idx])
                {
                    e->computeError();              // 只计算active edge的误差。
                }
                
                const float  chi2 = e->chi2();
                
                if(chi2 > chi2Mono[it])
                {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);                 // 设置为outlier。
                    nBad++;
                }
                else
                {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0);                 // 设置为inlier。
                }

                if(it == 2)
                    e->setRobustKernel(0);         // 只有前两次优化使用核函数。
            }

            for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
            {
                g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = vpEdgesStereo[i];
                const size_t idx = vnIndexEdgeStereo[i];

                if(pFrame->mvbOutlier[idx])
                {
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if(chi2 > chi2Stereo[it])
                {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBad++;
                }
                else
                {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0);
                }

                if(it == 2)
                    e->setRobustKernel(0);
            }

            if(optimizer.edges().size() < 10)
                break;
        }

        // 步骤5 保存优化后位姿，返回内点数量。
        g2o::VertexSE3Expmap *vSE3_recov = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
        g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
        cv::Mat pose = Converter::toCvMat(SE3quat_recov);
        pFrame->SetPose(pose);

        return nInitialCorrespondences-nBad;
    }



    /*
    * Loal BA优化
    *   顶点：
    *       g2o::VertexSE3Expmap,   LocalKeyFrames，即当前关键帧，与当前关键帧相连关键帧的位姿。
    *       g2o::VertexSE3Expmap,   FixedCameras,   即能观测到Local MapPoints且不属于LocalKeyFrames的关键帧的位姿，在优化中，这些关键帧位姿不变。
    *       g2o::VertexSBAPointXYZ, LocalMapPoints，即LocalKeyFrames能观测到的所有MapPoints的坐标。
    *
    *   边：
    *       g2o::EdgeSE3ProjectXYZ, BaseBinaryEdge
    *           连接顶点：关键帧的位姿Tcw, MapPoint的世界坐标系坐标Pw。
    *           测量值：  MapPoint在关键帧中的图像坐标。
    *           信息矩阵：与特征点尺度有关。
    *   Param:
    *       PKF                 KeyFrame
    *       pbStopFlag          是否停止优化的标志。
    *       pMap                在优化后，更新状态时需要用到Map的互斥量mMutexMapUpdate。
    *
    */
    // 该函数用于LocalMapping线程的局部BA优化。
    void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, LocalMapping *pLM)
    {

        list<KeyFrame *> lLocalKeyFrames;

        // 步骤1 将当前帧加入到lLocalKeyFrames。
        lLocalKeyFrames.push_back(pKF);
        pKF->mnBALocalForKF = pKF->mnId;

        // 步骤2 找到当前关键帧pKF连接的关键帧(一级相连)，加入lLocalKeyFrames。
        const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
        for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
        {
            KeyFrame *pKFi = vNeighKFs[i];
            pKFi->mnBALocalForKF = pKF->mnId;
            if(!pKFi->isBad())
                lLocalKeyFrames.push_back(pKFi);
        }

        // 步骤3 遍历所有lLocalKeyFrames中的关键帧，将观测到的MapPoints加入到lLocalMapPoints。
        list<MapPoint *> lLocalMapPoints;
        for(list<KeyFrame *>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
        {
            vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();    // 获取该帧所有的MapPoints。
            for(vector<MapPoint *>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
            {
                MapPoint *pMP = *vit;
                if(pMP)
                {
                    if(!pMP->isBad())
                        if(pMP->mnBALocalForKF!=pKF->mnId)
                        {
                            lLocalMapPoints.push_back(pMP);
                            pMP->mnBALocalForKF = pKF->mnId;        // 防止冲突添加不同关键帧观测到的相同点云。
                        }
                }
            }
        }

        // 步骤4 获得能观测到局部MapPoints，但不属于局部关键帧的关键帧，这些关键帧在Local中不优化。
        list<KeyFrame *> lFixedCameras;
        for(list<MapPoint *>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
        {
            mapMapPointObs/*map<KeyFrame *, size_t>*/ observations = (*lit)->GetObservations();
            for(mapMapPointObs/*map<KeyFrame *, size_t>*/::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                // pKFi->mnBALocalForKF!=pKF->mnId表示不是局部关键帧。
                if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
                {
                    pKFi->mnBAFixedForKF = pKF->mnId;       // 标志位，防止重复添加。
                    if(!pKFi->isBad())
                        lFixedCameras.push_back(pKFi);
                }
            }
        }

        // 步骤5 构造g2o优化器。
        g2o::SparseOptimizer optimizer;         // g2o求解器。
        
        // 设置线性模型的求解算法。
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

        // 6×3性质。
        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
        
        // L-M下降。
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

		// 传递变量，每次迭代优化时检测，用于终止Local BA
        if(pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        unsigned long maxKFid = 0;

        // 步骤6 添加顶点，局部关键帧位姿。
        for(list<KeyFrame *>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));                           // 设置迭代初值。
            vSE3->setId(pKFi->mnId);
            vSE3->setFixed(pKFi->mnId==0);                                                      // 第一帧位置固定。
            optimizer.addVertex(vSE3);
            if(pKFi->mnId > maxKFid)
                maxKFid = pKFi->mnId;
        }

        // 步骤7 添加顶点， 非LocalKeyFrames关键帧，固定不优化。
        for(list<KeyFrame *>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));                           // 设置迭代初值。
            vSE3->setId(pKFi->mnId);
            vSE3->setFixed(true);
            optimizer.addVertex(vSE3);
            if(pKFi->mnId > maxKFid)
                maxKFid = pKFi->mnId;
        }

        // 步骤8 添加顶点， LocalMapPoints, 3D点坐标。
        const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

		// 预先定义单目边，分配存储空间
        vector<g2o::EdgeSE3ProjectXYZ *> vpEdgesMono;
        vpEdgesMono.reserve(nExpectedSize);

		// 边的关键帧
        vector<KeyFrame *> vpEdgeKFMono;
        vpEdgeKFMono.reserve(nExpectedSize);
		
		// 边的地图点
        vector<MapPoint *> vpMapPointEdgeMono;
        vpMapPointEdgeMono.reserve(nExpectedSize);
		
		// 预先定义双目边，分配存储空间
        vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
        vpEdgesStereo.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFStereo;
        vpEdgeKFStereo.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeStereo;
        vpMapPointEdgeStereo.reserve(nExpectedSize);

        const float thHuberMono = sqrt(5.991);
        const float thHuberStereo = sqrt(7.815);

        // 遍历所有Local MapPoint。
        for(list<MapPoint *>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
        {
            // 添加顶点，局部MapPoints。
            MapPoint *pMP = *lit;
            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));          // 设置迭代初值。
            int id = pMP->mnId+maxKFid+1;                                           // 防止id和LocalKF冲突。
            vPoint->setId(id);
            vPoint->setMarginalized(true);											// 边缘化，方便求解。
            optimizer.addVertex(vPoint);

	    // 地图点云所有观测关键帧
            const mapMapPointObs/*map<KeyFrame *, size_t>*/ observations = pMP->GetObservations();

            // 步骤9 对每一对关联的MapPoint和KF构建边。
            for(mapMapPointObs/*map<KeyFrame *, size_t>*/::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if(!pKFi->isBad())
                {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                    // 单目。
                    if(pKFi->mvuRight[mit->second] < 0)
                    {

                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;            // 地图点云对应的图像特征点位置测量值(u,v)。

                        g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                        // 设置边连接的顶点。
						// 点云和关键帧通过id对应起来。
                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *> (optimizer.vertex(id)));				// 地图点云顶点。
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *> (optimizer.vertex(pKFi->mnId)));		// 关键帧顶点。
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);			// 信息矩阵。

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        e->fx = pKFi->fx;
                        e->fy = pKFi->fy;
                        e->cx = pKFi->cx;
                        e->cy = pKFi->cy;

                        optimizer.addEdge(e);
						
						// 保存边，保存对应的关键帧和地图点云。
                        vpEdgesMono.push_back(e);			
                        vpEdgeKFMono.push_back(pKFi);
                        vpMapPointEdgeMono.push_back(pMP);
                    }

                    // 双目
                    else
                    {
                        Eigen::Matrix<double, 3, 1> obs;
                        const float kp_ur = pKFi->mvuRight[mit->second];
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
						e->setInformation(Info);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        e->fx = pKFi->fx;
                        e->fy = pKFi->fy;
                        e->cx = pKFi->cx;
                        e->cy = pKFi->cy;
                        e->bf = pKFi->mbf;

                        optimizer.addEdge(e);
                        vpEdgesStereo.push_back(e);
                        vpEdgeKFStereo.push_back(pKFi);
                        vpMapPointEdgeStereo.push_back(pMP);
                    }
                }
            }
        }

		// 开始优化前判断是否需要终止。
        if(pbStopFlag)
            if(*pbStopFlag)
                return;

        // 步骤10 开始优化。
        optimizer.initializeOptimization();
        optimizer.optimize(5);                 // 执行优化。

        bool bDoMore = true;

        // 判断是否要求中断BA, pbStopFlag为true表示中断。
        if(pbStopFlag)
            if(*pbStopFlag)
                bDoMore = false;

        if(bDoMore)
        {
            // 步骤11 检测outlier，并设置下次不优化。
            // 单目
            for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
            {
                g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
                MapPoint *pMP = vpMapPointEdgeMono[i];

                if(pMP->isBad())
                    continue;

                // 基于卡方检验计算出的阈值。
                if(e->chi2() > 5.991 || !e->isDepthPositive())
                {
                    e->setLevel(1);         // 不优化。
                }

                e->setRobustKernel(0);     // 下次不使用核函数。
            }

            // 双目
            for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
            {
                g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
                MapPoint *pMP = vpMapPointEdgeStereo[i];

                if(pMP->isBad())
                    continue;

                if(e->chi2() > 7.815 || !e->isDepthPositive())
                {
                    e->setLevel(1);
                }

                e->setRobustKernel(0);
            }

            // 步骤12 剔除outlier后再次优化。
            optimizer.initializeOptimization(0);
            optimizer.optimize(10);
        }

        vector<pair<KeyFrame *, MapPoint *>> vToErase;
        vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

        // 步骤13 重新优化后，记录误差比较大的边连接的KF和MapPoint。
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFMono[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];
            
            if(pMP->isBad())
                continue;

            if(e->chi2()>7.815 || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFStereo[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        // 获取Map线程锁。
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        // 步骤14 剔除投影误差过大的关键帧和地图点，在关键帧中剔除该对MapPoints的观测， 在该MapPoint中剔除该关键帧对其的观测。
        if(!vToErase.empty())
        {
            for(size_t i=0; i<vToErase.size(); i++)
            {
                KeyFrame *pKFi = vToErase[i].first;
                MapPoint *pMPi = vToErase[i].second;
                pKFi->EraseMapPointMatch(pMPi);
                pMPi->EraseObservation(pKFi);
            }
        }

        // 步骤15 更新优化后的关键帧位姿以及MapPoints的位置，平均观测方向等属性。

        // 关键帧属性更新。
        for(list<KeyFrame *>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
        {
            KeyFrame *pKF = *lit;
            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
            g2o::SE3Quat SE3quat = vSE3->estimate();
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }

        // MapPoint属性更新。
        for(list<MapPoint *>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
        {
            MapPoint *pMP = *lit;
            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId+maxKFid+1));
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }

        if(pLM)
	{
	    pLM->SetMapUpdateFlagInTracking(true);
	}
    }



    /*
    * 闭环检测后，进行Essential图优化。
    *
    *   顶点：
    *       g2o::VertexSim3Expmap, Essential图中的关键帧的位姿。
    *   边：
    *       g2o::EdgeSim3(), BaseBinaryEdge。
    *           连接的顶点：关键帧位姿Tcw, MapPoint Pw。
    *           测量值：    经过CorrectLoop函数步骤2后，sim3矫正后的位姿。
    *           信息矩阵：  单位阵。
    *
    *   Param
    *       pMap                    全局地图。
    *       pLoopKF                 闭环匹配上的关键帧。
    *       pCurKF                  当前关键帧。
    *       NonCorrectedSim3        未经过sim3调整过的关键帧位姿。
    *       CorrectedSim3           经过sim3调整过的关键帧位姿。
    *       LoopConnections         因闭环时MapPoints调整重新产生的边。
    *
    **/
    void Optimizer::OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF, 
                                            const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                            const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                            const map<KeyFrame *, set<KeyFrame *> > &LoopConnections, const bool &bFixScale,
					    LoopClosing *pLC)
    {

        // 步骤1 构造优化器。
        g2o::SparseOptimizer optimizer;             // 构造求解器。
        optimizer.setVerbose(false);                // 调试信息不输出。

        // 选择Eigen块求解器作为线性模型的求解方法。
		g2o::BlockSolver_7_3::LinearSolverType *linearSolver = 
        		new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();

        // 构造线性求解器。
        g2o::BlockSolver_7_3 *solver_ptr = new g2o::BlockSolver_7_3(linearSolver);

        // 使用L-M算法进行非线性迭代。
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

        solver->setUserLambdaInit(1e-16);       // L-M中的lambda。
        optimizer.setAlgorithm(solver);

        const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        const vector<MapPoint *> vpMPs = pMap->GetAllMapPoints();

        const unsigned int nMaxKFid = pMap->GetMaxKFid();

        // 经过Sim3调整，未优化的KF位姿。
        vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
        // 经过Sim3调整，优化后的KF位姿。
        vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);

        vector<g2o::VertexSim3Expmap *> vpVertices(nMaxKFid+1);

        const int minFeat = 100;

        // 步骤2 添加顶点：地图中所有KF的pose。
        // 尽量使用经过sim3调整的位姿。
        for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if(pKF->isBad())
                continue;

            g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

            const int nIDi = pKF->mnId;

            LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

            // 如果关键帧在闭环时通过sim3调整过位姿，使用调整位姿。
            if(it!=CorrectedSim3.end())
            {
                vScw[nIDi] = it->second;
                VSim3->setEstimate(it->second);             // 设置迭代初始值。
            }

            // 没有经过sim3调整，使用自身位姿。
            else
            {
                Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
                Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKF->GetTranslation());
                g2o::Sim3 Siw(Rcw, tcw, 1.0);
                vScw[nIDi] = Siw;
                VSim3->setEstimate(Siw);
            }

            // 闭环匹配到的帧不进行位姿优化
            if(pKF==pLoopKF)
                VSim3->setFixed(true);

            VSim3->setId(nIDi);
            VSim3->setMarginalized(false);
            VSim3->_fix_scale = bFixScale;

            optimizer.addVertex(VSim3);

            // 优化前的pose顶点。
            vpVertices[nIDi] = VSim3;
        }
        
        set<pair<long unsigned int, long unsigned int> > sInsertedEdges;
        const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

        // 步骤3 添加边：LoopConnections是闭环时因为MapPoints调整而出现的新关键帧连接关系，不是当前帧与闭环匹配帧之间的连接关系。
        for(map<KeyFrame *, set<KeyFrame *> >::const_iterator mit=LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
        {
            KeyFrame *pKF = mit->first;
            const long unsigned int nIDi = pKF->mnId;
            const set<KeyFrame *> &spConnections = mit->second;
            const g2o::Sim3 Siw = vScw[nIDi];
            const g2o::Sim3 Swi = Siw.inverse();

            for(set<KeyFrame *>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
            {
                const long unsigned int nIDj = (*sit)->mnId;
                if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) && pKF->GetWeight(*sit) < minFeat)
                    continue;

                const g2o::Sim3 Sjw = vScw[nIDj];
                // 得到两个pose之间的sim3变换。
                const g2o::Sim3 Sji = Sjw*Swi;

                g2o::EdgeSim3 *e = new g2o::EdgeSim3();
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *> (optimizer.vertex(nIDj)));    // 添加边的顶点，sp中的帧。
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *> (optimizer.vertex(nIDi)));    // 添加边的顶点，sp树的父节点帧。
                e->setMeasurement(Sji);                                                                     // 边的测量值，两顶点间的相对位姿变换。

                e->information() = matLambda;                   // 单位阵。

                optimizer.addEdge(e);                           // 添加边。

                sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
            }
        }

        // 步骤4 添加跟踪时形成的边，闭环匹配成功形成的边。
        for(size_t i=0, iend=vpKFs.size(); i<iend; i++)            // 遍历地图中的所有关键帧。
        {
            KeyFrame *pKF = vpKFs[i];

            const int nIDi = pKF->mnId;

            g2o::Sim3 Swi;

            LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

            // 尽可能得到没有经过sim3调整的边。
            if(iti!=NonCorrectedSim3.end())
                Swi = (iti->second).inverse();
            else
                Swi = vScw[nIDi].inverse();

            KeyFrame *pParentKF = pKF->GetParent();

            // 步骤4.1 只添加扩展树的边，有父节点的关键帧。
            if(pParentKF)
            {
                int nIDj = pParentKF->mnId;

                g2o::Sim3 Sjw;

                LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

                // 尽可能得到未经过Sim3传播调整的位姿。
                if(itj!=NonCorrectedSim3.end())
                    Sjw = itj->second;
                else
                    Sjw = vScw[nIDj];

                g2o::Sim3 Sji = Sjw*Swi;

                g2o::EdgeSim3 *e = new g2o::EdgeSim3();
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *> (optimizer.vertex(nIDj)));    // 地图中某一关键帧的父节点。
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *> (optimizer.vertex(nIDi)));    // 地图中某一关键帧。
                e->setMeasurement(Sji);             // 测量值，边两个顶点的位姿相对变换。

                e->information() = matLambda;
                optimizer.addEdge(e);
            }

            // 步骤4.2 添加在CorrectLoop函数中AddLoopEdge函数添加的连接边(当前关键帧与闭环帧的连接关系)。
            const set<KeyFrame *> sLoopEdges = pKF->GetLoopEdges();         // 获取闭环帧。

            for(set<KeyFrame *>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
            {
                KeyFrame *pLKF = *sit;
                if(pLKF->mnId < pKF->mnId)
                {
                    g2o::Sim3 Slw;
                    LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

                    // 尽量使用未经过sim3调整的位姿。
                    if(itl!=NonCorrectedSim3.end())
                        Slw = itl->second;
                    else
                        Slw = vScw[pLKF->mnId];

                    g2o::Sim3 Sli = Slw*Swi;
                    g2o::EdgeSim3 *el = new g2o::EdgeSim3();
                    el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *> (optimizer.vertex(pLKF->mnId)));     // 地图中的某帧关键帧的闭环帧。
                    el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *> (optimizer.vertex(nIDi)));           // 地图中的某帧关键帧。
                    el->setMeasurement(Sli);        // 边的两顶点的相对位姿变换。
                    el->information() = matLambda;
                    optimizer.addEdge(el);
                }
            }

            // 步骤4.3 与pKF具有很好共视关系的关键帧也作为边进行优化。
            const vector<KeyFrame *> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
            for(vector<KeyFrame *>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
            {
                
                KeyFrame *pKFn = *vit;
                if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
                {
                    if(!pKFn->isBad() && pKFn->mnId < pKF->mnId)
                    {
                        if(sInsertedEdges.count(make_pair(min(pKF->mnId, pKFn->mnId), max(pKF->mnId, pKFn->mnId)))) // 在步骤3中已经添加过。
                            continue;

                        g2o::Sim3 Snw;
                        LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                        // 尽量使用未经过sim3调整的位姿。
                        if(itn != NonCorrectedSim3.end())
                            Snw = itn->second;
                        else
                            Snw = vScw[pKFn->mnId];

                        g2o::Sim3 Sni = Snw*Swi;         // 相对位姿变换，地图中某帧关键帧和pKFn的位姿相对变换。

                        g2o::EdgeSim3 *en = new g2o::EdgeSim3();
                        en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFn->mnId)));  // 添加边，pKFn。
                        en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));        // 添加边，pKF。
                        en->setMeasurement(Sni);
                        en->information() = matLambda;
                        optimizer.addEdge(en);
                    }
                }
            }
        }

        // 步骤5 开始优化。
        optimizer.initializeOptimization();
        optimizer.optimize(20);

	unique_lock<mutex> lock(pMap->mMutexMapUpdate);
	 
	cv::Mat Tbc = ConfigParam::GetMatTbc();
        // 步骤6 设置优化后的位姿。Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame *pKFi = vpKFs[i];
            const int nIDi = pKFi->mnId;

            g2o::VertexSim3Expmap *VSim3 = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(nIDi));
            g2o::Sim3 CorrectedSiw = VSim3->estimate();
            vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
            Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = CorrectedSiw.translation();
            double s = CorrectedSiw.scale();

            eigt *= (1.0/s);

            cv::Mat Tiw = Converter::toCvSE3(eigR, eigt);

            pKFi->SetPose(Tiw);
	    
	    // Update P/V/R in NavState
	    pKFi->UpdateNavStatePVRFromTcw(Tiw,Tbc);
        }

        // 步骤7 步骤5和步骤6优化得到关键帧位姿后，MapPoints根据参考帧优化前后的相对关系调整自己的位置。
        for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
        {
            MapPoint *pMP = vpMPs[i];

            if(pMP->isBad())
                continue;

            int nIDr;

            // 该MapPoint经过Sim3调整过(CorrectLoop函数的步骤2.2)。
            if(pMP->mnCorrectedByKF == pCurKF->mnId)
            {
                nIDr = pMP->mnCorrectedReference;
            }
            else
            {
                // 通常情况下，MapPoint的参考帧就是它的创建关键帧。
                KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
                nIDr = pRefKF->mnId;
            }

            // 得到MapPoint参考关键帧步骤5优化前的位姿。
            g2o::Sim3 Srw = vScw[nIDr];
            // 得到MapPoint参考关键帧后的位姿。
            g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

            cv::Mat P3Dw = pMP->GetWorldPos();
            Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
            Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));  // 3D坐标 从w->r, r->w，假设3D点在相机坐标下的坐标不变。

            cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
            pMP->SetWorldPos(cvCorrectedP3Dw);

            pMP->UpdateNormalAndDepth();
        }
        
        if(pLC)
	{
	    pLC->SetMapUpdateFlagInTracking(true);
	}
    }



    /*
    * 形成闭环后进行Sim3优化，优化pKF1和pKF2之间的sim3位姿变换。
    *   顶点：
    *       g2o::VertexSim3Expmap, 两个关键帧位姿变换。
    *       g2o::VertexSBAPointXYZ, 两关键帧的MapPoints。
    *   边：
    *       g2o::EdgeSim3ProjectXYZ, BaseBinaryEdge
    *           连接顶点：关键帧的sim3位姿，地图点云Pw。
    *           测量值： MapPoint在关键帧中的图像坐标(u,v)。
    *           信息矩阵：invSigma2(与特征点尺度有关)。
    *       g2o::EdgeInverseSim3ProjectXYZ, BaseBinaryEdge
    *           连接顶点: 关键帧的sim3位姿，地图点云Pw。
    *           测量值：MapPoint在关键帧中的图像坐标(u,v)。
    *           信息矩阵：invSigma2(与特征点尺度有关)。
    *
    *   Param
    *       pKF1, pKF2          KeyFrame。
    *       vpMatches1          两关键帧之间的匹配关系。
    *       g2oS12              两关键帧之间的Sim3变换。
    *       th2                 核函数阈值。
    *       bFixedScale         是否优化尺度，单目优化，双目不优化。
    *
    *   return                  返回内点数量。
    */
    int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
    {
        // 步骤1 初始化g2o求解器。
        g2o::SparseOptimizer optimizer;                             // 构造求解器。

        // 构造线性方程求解器, Hx = -b。
        g2o::BlockSolverX::LinearSolverType *linearSolver;          
        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();     // 使用Dense算法求解器。
        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        // 使用L-M迭代算法。
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        // 相机内参。
        const cv::Mat &K1 = pKF1->mK;
        const cv::Mat &K2 = pKF2->mK;

        // 相机位姿。
        const cv::Mat R1w = pKF1->GetRotation();
        const cv::Mat t1w = pKF1->GetTranslation();
        const cv::Mat R2w = pKF2->GetRotation();
        const cv::Mat t2w = pKF2->GetTranslation();

        // 步骤2 添加顶点(优化变量)。
        
        // 步骤2.1 添加Sim3顶点，两关键帧之间位姿变换。
        g2o::VertexSim3Expmap *vSim3 = new g2o::VertexSim3Expmap();
        vSim3->_fix_scale = bFixScale;
        vSim3->setEstimate(g2oS12);             // 设置待优化变量初值。
        vSim3->setId(0);
        vSim3->setFixed(false);                 // 加入优化。
        vSim3->_principle_point1[0] = K1.at<float>(0,2);            // cx
        vSim3->_principle_point1[1] = K1.at<float>(1,2);            // cy
        vSim3->_focal_length1[0] = K1.at<float>(0,0);               // fx
        vSim3->_focal_length1[1] = K1.at<float>(1,1);               // fy
        vSim3->_principle_point2[0] = K2.at<float>(0,2);
        vSim3->_principle_point2[1] = K2.at<float>(1,2);
        vSim3->_focal_length2[0] = K2.at<float>(0,0);
        vSim3->_focal_length2[1] = K2.at<float>(1,1);
        optimizer.addVertex(vSim3);


        const int N = vpMatches1.size();                // 匹配点数量。
        const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();

        vector<g2o::EdgeSim3ProjectXYZ *> vpEdges12;            // pKF2对应的MapPoints到pKF1的投影。
        vector<g2o::EdgeInverseSim3ProjectXYZ *> vpEdges21;     // pKF1对应的MapPoints到pKF2的投影。
        vector<size_t> vnIndexEdge;

        vnIndexEdge.reserve(2*N);
        vpEdges12.reserve(2*N);
        vpEdges21.reserve(2*N);

        const float deltaHuber = sqrt(th2);

        int nCorrespondences = 0;

        for(int i=0; i<N; i++)
        {
            if(!vpMatches1[i])
                continue;

            // pMP1和pMP2是匹配的MapPoints。
            MapPoint *pMP1 = vpMapPoints1[i];
            MapPoint *pMP2 = vpMatches1[i];

            // 边id。
            const int id1 = 2*i+1;  
            const int id2 = 2*(i+1);            

            const int i2 = pMP2->GetIndexInKeyFrame(pKF2);

            if(pMP1 && pMP2)
            {
                if(!pMP1->isBad() && !pMP2->isBad() && i2>=0)
                {
                    // 步骤2.2 添加PointXYZ顶点
                    g2o::VertexSBAPointXYZ *vPoint1 = new g2o::VertexSBAPointXYZ();
                    cv::Mat P3D1w = pMP1->GetWorldPos();
                    cv::Mat P3D1c = R1w*P3D1w + t1w;
                    vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                    vPoint1->setId(id1);
                    vPoint1->setFixed(true);        // 不参与优化。
                    optimizer.addVertex(vPoint1);

                    g2o::VertexSBAPointXYZ *vPoint2 = new g2o::VertexSBAPointXYZ();
                    cv::Mat P3D2w = pMP2->GetWorldPos();
                    cv::Mat P3D2c = R2w*P3D2w + t2w;
                    vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                    vPoint2->setId(id2);
                    vPoint2->setFixed(true);        // 不参与优化。
                    optimizer.addVertex(vPoint2);
                }
                else 
                    continue;
            }
            else
                continue;

            nCorrespondences++;

            Eigen::Matrix<double, 2, 1> obs1;
            const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
            obs1 << kpUn1.pt.x, kpUn1.pt.y;

            // 步骤2.3 设置边。
            // 从pKF2到pKF1的重投影误差。
            g2o::EdgeSim3ProjectXYZ *e12 = new g2o::EdgeSim3ProjectXYZ();
            e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id2)));
            e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e12->setMeasurement(obs1);          // pKF1的地图点云在KF1的图像坐标。
            const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
            e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

            g2o::RobustKernelHuber *rk1 = new g2o::RobustKernelHuber;
            e12->setRobustKernel(rk1);
            rk1->setDelta(deltaHuber);
            optimizer.addEdge(e12);

            Eigen::Matrix<double, 2, 1> obs2;
            const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
            obs2 << kpUn2.pt.x, kpUn2.pt.y;

            // 从pKF1->pKF2的重投影误差。
            g2o::EdgeInverseSim3ProjectXYZ *e21 = new g2o::EdgeInverseSim3ProjectXYZ();
            e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id1)));
            e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e21->setMeasurement(obs2);
            float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
            e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

            g2o::RobustKernelHuber *rk2 = new g2o::RobustKernelHuber;
            e21->setRobustKernel(rk2);
            rk2->setDelta(deltaHuber);
            optimizer.addEdge(e21);

            vpEdges12.push_back(e12);
            vpEdges21.push_back(e21);
            vnIndexEdge.push_back(i);
        }

        // 步骤3 g2o开始优化，迭代5次。
        optimizer.initializeOptimization();
        optimizer.optimize(5);

        // 步骤4 outliers检查，剔除误差大的边。
        int nBad = 0;
        for(size_t i=0; i<vpEdges12.size(); i++)
        {
            g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
            g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
            if(!e12 || !e21)
                continue;

            // 剔除
            if(e12->chi2()>th2 || e21->chi2()>th2)
            {
                size_t idx = vnIndexEdge[i];
                vpMatches1[idx] = static_cast<MapPoint *>(NULL);
                optimizer.removeEdge(e12);
                optimizer.removeEdge(e21);
                vpEdges12[i] = static_cast<g2o::EdgeSim3ProjectXYZ *>(NULL);
                vpEdges21[i] = static_cast<g2o::EdgeInverseSim3ProjectXYZ *>(NULL);
                nBad++;
            }
        }

        int nMoreIterations;
        if(nBad >0)
            nMoreIterations = 10;
        else
            nMoreIterations = 5;

        // 剩余的内点。
        if(nCorrespondences-nBad < 10)
            return 0;

        // 步骤5 再次优化。
        optimizer.initializeOptimization();
        optimizer.optimize(nMoreIterations);

        int nIn = 0;
        for(size_t i=0; i<vpEdges12.size(); i++)
        {

            g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
            g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
            if(!e12 || !e21)
                continue;

            if(e12->chi2() > th2 || e21->chi2() > th2)
            {
                size_t idx = vnIndexEdge[i];
                vpMatches1[idx] = static_cast<MapPoint *>(NULL);
            }
            else
                nIn++;
        }

        // 步骤6 保存优化后的结果。
        g2o::VertexSim3Expmap *vSim3_recov = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
        g2oS12 = vSim3_recov->estimate();

        return nIn;
    }



}       // namespace ORB_SLAM2










