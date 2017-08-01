/*  输入，Tracking线程创建的关键帧，在队列mlNewKeyFrames
*	线程函数，Run()。
*	处理新加入的关键帧，将关键帧与其地图点云关联起来，剔除新添加的冗余的地图点云。通过三角化新关键帧和它共视图的特征点增加地图点云，融合重复点云，更新公视关系。
*	通过之前的操作，更新map，然后进行局部BA, 局部地图是新插入的关键帧和它的共视图高的关键帧的位姿，新插入关键帧的地图点云位置。
*   剔除冗余关键帧
*   输出，将当前加入的关键帧和关联的地图点云加入通过函数InsertKeyFrame()加入到闭环检测队列mlpLoopKeyFrameQueue。
*/

#include "Converter.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#include <mutex>

namespace ORB_SLAM2
{
using namespace std;
    
    /************************VI SLAM**************************/
    
    class KeyFrameInit
    {
        
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        double mTimeStamp;
        KeyFrameInit *mpPrevKeyFrame;
        cv::Mat Twc;
        IMUPreintegrator mIMUPreInt;
        std::vector<IMUData> mvIMUData;
        Vector3d bg;
        
        
        // 构造函数
        KeyFrameInit(KeyFrame &kf):
            mTimeStamp(kf.mTimeStamp), mpPrevKeyFrame(NULL), Twc(kf.GetPoseInverse().clone()),
            mIMUPreInt(kf.GetIMUPreInt()), mvIMUData(kf.GetVectorIMUData()), bg(0,0,0)
        {
            
        }
        
        
        void ComputePreInt(void)
        {
            if(mpPrevKeyFrame == NULL)
            {
                return;
            }
            else
            {
                // 重置IMUPre
                mIMUPreInt.reset();
                
                if(mvIMUData.empty())
                    return;
                
                // 考虑上一帧关键帧和第一个IMU数据的预积分
                {
                    const IMUData &imu = mvIMUData.front();
                    double dt = std::max(0., imu._t - mpPrevKeyFrame->mTimeStamp);
                    mIMUPreInt.update(imu._g-bg, imu._a, dt);
                }
                
                for(size_t i=0; i<mvIMUData.size(); i++)
                {
                    const IMUData &imu = mvIMUData[i];
                    double nextt;
                    
                    // 最后一帧
                    if(i==mvIMUData.size()-1)
                        nextt = mTimeStamp;
                    else
                        nextt = mvIMUData[i+1]._t;
                    
                    double dt = std::max(0., nextt - imu._t);
                        
                    mIMUPreInt.update(imu._g-bg, imu._a, dt);
                }
            }
        }
        
    };
    
    
    
    //
    bool LocalMapping::GetUpdatingInitPoses(void)
    {
        unique_lock<mutex> lock(mMutexUpdatingInitPoses);
        return mbUpdatingInitPoses;
    }
    
    void LocalMapping::SetUpdatingInitPoses(bool flag)
    {
        unique_lock<mutex> lock(mMutexUpdatingInitPoses);
        mbUpdatingInitPoses = flag;
    }
    
    
    
    KeyFrame *LocalMapping::GetMapUpdateKF()
    {
        unique_lock<mutex> lock(mMutexMapUpdateFlag);
        return mpMapUpdateKF;
    }
    
    bool LocalMapping::GetMapUpdateFlagForTracking()
    {
        unique_lock<mutex> lock(mMutexMapUpdateFlag);
        return mbMapUpdateFlagForTracking;
    }
    
    
    
    void LocalMapping::SetMapUpdateFlagInTracking(bool bflag)
    {
        unique_lock<mutex> lock(mMutexMapUpdateFlag);
        mbMapUpdateFlagForTracking = bflag;
        if(bflag)
        {
            mpMapUpdateKF = mpCurrentKeyFrame;
        }
    }
    
    
    
    bool LocalMapping::GetVINSInited(void)
    {
        unique_lock<mutex> lock(mMutexVINSInitFlag);
        return mbVINSInited;
    }
    
    void LocalMapping::SetVINSInited(bool flag)
    {
        unique_lock<mutex> lock(mMutexVINSInitFlag);
        mbVINSInited = flag;
    }
    
    bool LocalMapping::GetFirstVINSInited(void)
    {
        unique_lock<mutex> lock(mMutexFirstVINSInitFlag);
        return mbFirstVINSInited;
    }
    

    void LocalMapping::SetFirstVINSInited(bool flag)
    {
        unique_lock<mutex> lock(mMutexFirstVINSInitFlag);
        mbFirstVINSInited = flag;
    }
    
    
    cv::Mat LocalMapping::GetGravityVec()
    {
        return mGravityVec.clone();
    }
    
    cv::Mat LocalMapping::GetRwiInit()
    {
        return mRwiInit.clone();
    }

    
    
    // 初始化VINS线程
    void LocalMapping::VINSInitThread()
    {
        unsigned long initedid = 0;
        cerr<<"start VINSInitThread"<<endl;
        
        while(1)
        {
            if(KeyFrame::nNextId > 2)
                if(!GetVINSInited() && mpCurrentKeyFrame->mnId > initedid)
                {
                    initedid = mpCurrentKeyFrame->mnId;
                    
                    bool tmpbool = TryInitVIO();
                    
                    if(tmpbool)
                    {
                        break;
                    }
                }
                
            usleep(3000);
            
            if(isFinished())
                break;
        }
        
    }
    
    
    
    // VIO线程初始化
    bool LocalMapping::TryInitVIO(void)
    {
        
	if(mpMap->KeyFramesInMap() <= mnLocalWindowSize)
	    return false;
	
	static bool fopened = false;
	static ofstream fgw, fscale, fbiasa, fcondnum, ftime, fbiasg;
	string tmpfilepath = ConfigParam::getTmpFilePath();
	
	if(!fopened)
	{
	    // 输入正确的路径
	    fgw.open(tmpfilepath+"gw.txt");
	    fscale.open(tmpfilepath+"scale.txt");
	    fbiasa.open(tmpfilepath+"biasa.txt");
	    fcondnum.open(tmpfilepath+"condnum.txt");
	    ftime.open(tmpfilepath+"computetime.txt");
	    fbiasg.open(tmpfilepath+"biasg.txt");
	    
	    if(fgw.is_open() && fscale.is_open() && fbiasa.is_open() &&
		    fcondnum.is_open() && ftime.is_open() && fbiasg.is_open())
		fopened = true;
	    else
	    {
		cerr<<"file open error in TryInitVIO"<<endl;
		fopened = false;
	    }
	    
	    fgw << std::fixed << std::setprecision(6);
	    fscale << std::fixed << std::setprecision(6);
	    fbiasa << std::fixed << std::setprecision(6);
	    fcondnum << std::fixed << std::setprecision(6);
	    ftime << std::fixed << std::setprecision(6);
	    fbiasg << std::fixed << std::setprecision(6);
	}
	
	Optimizer::GlobalBundleAdjustment(mpMap, 10);
	
	// 外参
	cv::Mat Tbc = ConfigParam::GetMatTbc();
	cv::Mat Rbc = Tbc.rowRange(0,3).colRange(0,3);
	cv::Mat pbc = Tbc.rowRange(0,3).col(3);
	cv::Mat Rcb = Rbc.t();
	cv::Mat pcb = -Rcb*pbc;
	
	if(ConfigParam::GetRealTimeFlag())
	{
	    // 等待KF剔除，如果在运行，等待完成
	    // 关键帧如果被复制，不剔除
	    while(GetFlagCopyInitKFs())
	    {
		usleep(1000);
	    }
	}
	
	SetFlagCopyInitKFs(true);
	
	// 提取地图所有KF
	vector<KeyFrame *> vScaleGravityKF = mpMap->GetAllKeyFrames();
	int N = vScaleGravityKF.size();
	KeyFrame *pNewestKF = vScaleGravityKF[N-1];
	vector<cv::Mat > vTwc;
	vector<IMUPreintegrator> vIMUPreInt;
	
	// 保存初始化需要的关键帧
	vector<KeyFrameInit *> vKFInit;
	for(int i =0; i<N; i++)
	{
	    KeyFrame *pKF = vScaleGravityKF[i];
	    vTwc.push_back(pKF->GetPoseInverse());
	    vIMUPreInt.push_back(pKF->GetIMUPreInt());
	    
	    KeyFrameInit *pkfi = new KeyFrameInit(*pKF);
	    if(i!=0)
	    {
		pkfi->mpPrevKeyFrame = vKFInit[i-1];
	    }
	    vKFInit.push_back(pkfi);
	}
	
	SetFlagCopyInitKFs(false);
	
	// 步骤1 计算gyro偏移
	Vector3d bgest = Optimizer::OptimizeInitialGyroBias(vTwc, vIMUPreInt);
	
	// 更新 gyro bias和KF预积分
	for(int i=0; i<N; i++)
	{
	    vKFInit[i]->bg = bgest;
	}
	for(int i=0; i<N; i++)
	{
	    vKFInit[i]->ComputePreInt();
	}
	
	
	// 步骤2 估计尺度和重力加速度向量(世界坐标系，实际是第一帧的坐标系下) 
	
	// 求解 Ax=B, x=[s,gw]
	cv::Mat A = cv::Mat::zeros(3*(N-2), 4, CV_32F);
	cv::Mat B = cv::Mat::zeros(3*(N-2), 1, CV_32F);
	cv::Mat I3 = cv::Mat::eye(3, 3, CV_32F);
	
	for(int i=0; i<N-2; i++)
	{
	    
	    KeyFrameInit *pKF2 = vKFInit[i+1];
	    KeyFrameInit *pKF3 = vKFInit[i+2];
	    
	    // 关键帧间的时间间隔
	    double dt12 = pKF2->mIMUPreInt.getDeltaTime();
	    double dt23 = pKF3->mIMUPreInt.getDeltaTime();
	    
	    // IMU预积分值
	    cv::Mat dp12 = Converter::toCvMat(pKF2->mIMUPreInt.getDeltaP());
	    cv::Mat dv12 = Converter::toCvMat(pKF2->mIMUPreInt.getDeltaV());
	    cv::Mat dp23 = Converter::toCvMat(pKF3->mIMUPreInt.getDeltaP());
	    
	    // 世界坐标系下相机位姿
	    cv::Mat Twc1 = vTwc[i].clone();		// pKF1
	    cv::Mat Twc2 = vTwc[i+1].clone();		// pKF2
	    cv::Mat Twc3 = vTwc[i+2].clone();		// pKF3
	    
	    // 相机中心坐标
	    cv::Mat pc1 = Twc1.rowRange(0,3).col(3);
	    cv::Mat pc2 = Twc2.rowRange(0,3).col(3);
	    cv::Mat pc3 = Twc3.rowRange(0,3).col(3);
	    
	    // 相机旋转
	    cv::Mat Rc1 = Twc1.rowRange(0,3).colRange(0,3);
	    cv::Mat Rc2 = Twc2.rowRange(0,3).colRange(0,3);
	    cv::Mat Rc3 = Twc3.rowRange(0,3).colRange(0,3);
	    
	    // 求解方程A/B
	    // lambda*s + beta*g = gamma
	    cv::Mat lambda = (pc2-pc1)*dt23+(pc2-pc3)*dt12;
	    cv::Mat beta = 0.5*I3*(dt12*dt12*dt23+dt12*dt23*dt23);
	    cv::Mat gamma = (Rc3-Rc2)*pcb*dt12+(Rc1-Rc2)*pcb*dt23+Rc1*Rcb*dp12*dt23-Rc2*Rcb*dp23*dt12-Rc1*Rcb*dv12*dt12*dt23;
	    lambda.copyTo(A.rowRange(3*i+0, 3*i+3).col(0));
	    beta.copyTo(A.rowRange(3*i+0, 3*i+3).colRange(1,4));
	    gamma.copyTo(B.rowRange(3*i+0, 3*i+3));
	}
	
	// SVD分解
	cv::Mat w,u,vt;
	cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A);
	
	cv::Mat winv = cv::Mat::eye(4,4,CV_32F);
	for(int i=0; i<4; i++)
	{
	    if(fabs(w.at<float>(i))<1e-10)
	    {
		w.at<float>(i) += 1e-10;
		// test log
		cerr<<"w(i) < 1e-10, w="<<endl<<w<<endl;
	    }
	    
	    winv.at<float>(i,i) = 1./w.at<float>(i);
	}
	
	cv::Mat x = vt.t()*winv*u.t()*B;
	
	// x=[s,gw]
	double sstar = x.at<float>(0);
	cv::Mat gwstar = x.rowRange(1,4);
	
	// test log
	if(w.type()!=I3.type() || u.type()!=I3.type() || vt.type()!=I3.type())
	    cerr<<"different mat type, I3,w,u,vt: "<<I3.type()<<","<<w.type()<<","<<u.type()<<","<<vt.type()<<endl;
	
	// 步骤3 
	// gI = [0;0;1], the normalized gravity vector in an inertial frame, NED type with no orientation.
	cv::Mat gI = cv::Mat::zeros(3, 1, CV_32F);
	gI.at<float>(2) = 1;
	cv::Mat gwn = gwstar/cv::norm(gwstar);
	
	// 计算gwstar指向
	cv::Mat gIxgwn = gI.cross(gwn);
	double normgIxgwn = cv::norm(gIxgwn);
	cv::Mat vhat = gIxgwn/normgIxgwn;				// 重力与[0,0,1]旋转单位向量
	double theta = std::atan2(normgIxgwn, gI.dot(gwn));		// 重力与[0,0,1]旋转向量旋转角
	
	Eigen::Vector3d vhateig = Converter::toVector3d(vhat);
	Eigen::Matrix3d RWIeig = Sophus::SO3::exp(vhateig*theta).matrix();	// 重力与[0,0,1]旋转矩阵
	cv::Mat Rwi = Converter::toCvMat(RWIeig);
	cv::Mat GI = gI*ConfigParam::GetG();
	
	// 求解Cx=D, x=[s,dthetaxy, ba]
	cv::Mat C = cv::Mat::zeros(3*(N-2), 6, CV_32F);
	cv::Mat D = cv::Mat::zeros(3*(N-2), 1, CV_32F);
	
	
	for(int i=0; i<N-2; i++)
	{
	    KeyFrameInit *pKF2 = vKFInit[i+1];
	    KeyFrameInit *pKF3 = vKFInit[i+2];
	    
	    // 关键帧间的时间间隔
	    double dt12 = pKF2->mIMUPreInt.getDeltaTime();
	    double dt23 = pKF3->mIMUPreInt.getDeltaTime();
	    
	    // IMU预积分值
	    cv::Mat dp12 = Converter::toCvMat(pKF2->mIMUPreInt.getDeltaP());
	    cv::Mat dv12 = Converter::toCvMat(pKF2->mIMUPreInt.getDeltaV());
	    cv::Mat dp23 = Converter::toCvMat(pKF3->mIMUPreInt.getDeltaP());
	    cv::Mat Jpba12 = Converter::toCvMat(pKF2->mIMUPreInt.getJPBiasa());
	    cv::Mat Jvba12 = Converter::toCvMat(pKF2->mIMUPreInt.getJVBiasa());
	    cv::Mat Jpba23 = Converter::toCvMat(pKF3->mIMUPreInt.getJPBiasa());
	    
	    // 世界坐标系下相机位姿
	    cv::Mat Twc1 = vTwc[i].clone();		// pKF1
	    cv::Mat Twc2 = vTwc[i+1].clone();		// pKF2
	    cv::Mat Twc3 = vTwc[i+2].clone();		// pKF3
	    
	    // 相机中心坐标
	    cv::Mat pc1 = Twc1.rowRange(0,3).col(3);
	    cv::Mat pc2 = Twc2.rowRange(0,3).col(3);
	    cv::Mat pc3 = Twc3.rowRange(0,3).col(3);
	    
	    // 相机旋转
	    cv::Mat Rc1 = Twc1.rowRange(0,3).colRange(0,3);
	    cv::Mat Rc2 = Twc2.rowRange(0,3).colRange(0,3);
	    cv::Mat Rc3 = Twc3.rowRange(0,3).colRange(0,3);
	    
	    // 见论文
	    // lambda*s + phi*dthetaxy + zeta*ba = psi
	    cv::Mat lambda = (pc2-pc1)*dt23 + (pc2-pc3)*dt12;
	    cv::Mat phi = -0.5*(dt12*dt12*dt23+dt12*dt23*dt23)*Rwi*SkewSymmetricMatrix(GI);
	    cv::Mat zeta = Rc2*Rcb*Jpba23*dt12+Rc1*Rcb*Jvba12*dt12*dt23-Rc1*Rcb*Jpba12*dt23;
	    cv::Mat psi = (Rc1-Rc2)*pcb*dt23 + Rc1*Rcb*dp12*dt23 - (Rc2-Rc3)*pcb*dt12
			    -Rc2*Rcb*dp23*dt12 - Rc1*Rcb*dv12*dt23*dt12 - 0.5*Rwi*GI*(dt12*dt12*dt23+dt12*dt23*dt23);
	    
	    lambda.copyTo(C.rowRange(3*i+0, 3*i+3).col(0));
	    // 仅计算2列，第三列dtheta是0
	    phi.colRange(0,2).copyTo(C.rowRange(3*i+0, 3*i+3).colRange(1, 3));
	    zeta.copyTo(C.rowRange(3*i+0, 3*i+3).colRange(3, 6));
	    psi.copyTo(D.rowRange(3*i+0, 3*i+3));
	    
	}
	
	cv::Mat w2, u2, vt2;
	cv::SVDecomp(C, w2, u2, vt2, cv::SVD::MODIFY_A);
	
	cv::Mat w2inv = cv::Mat::eye(6,6,CV_32F);
	for(int i=0; i<6; i++)
	{
	    if(fabs(w2.at<float>(i))<1e-10)
	    {
		w2.at<float>(i) += 1e-10;
		cerr<<"w2(i) < 1e-10, w="<<endl<<w2<<endl;
	    }
	    
	    w2inv.at<float>(i,i) = 1./w2.at<float>(i);
	}
	
	cv::Mat y = vt2.t()*w2inv*u2.t()*D;
	
	double s_ = y.at<float>(0);
	cv::Mat dthetaxy = y.rowRange(1,3);
	cv::Mat dbiasa_ = y.rowRange(3,6);
	Vector3d dbiasa_eig = Converter::toVector3d(dbiasa_);
	
	// dtheta = [dx, dy, 0]
	cv::Mat dtheta = cv::Mat::zeros(3, 1, CV_32F);
	dthetaxy.copyTo(dtheta.rowRange(0,2));
	Eigen::Vector3d dthetaeig = Converter::toVector3d(dtheta);
	
	// Rwi_ = Rwi*exp(dtheta)
	Eigen::Matrix3d Rwieig_ = RWIeig*Sophus::SO3::exp(dthetaeig).matrix();
	cv::Mat Rwi_ = Converter::toCvMat(Rwieig_);
	
	
	// test log
	{
	    
	    cv::Mat gwbefore = Rwi*GI;
	    cv::Mat gwafter = Rwi_*GI;
	    cout<<"Time: "<<mpCurrentKeyFrame->mTimeStamp - mnStartTime<<", sstar: "<<sstar<<", s: "<<s_<<endl;
	    
	    fgw << mpCurrentKeyFrame->mTimeStamp << " "
		<< gwafter.at<float>(0) << " " << gwafter.at<float>(1) << " " << gwafter.at<float>(2) << " "
		<< gwbefore.at<float>(0) << " " << gwbefore.at<float>(1) << " " << gwbefore.at<float>(2) << " "
		<< endl;
		
	    fscale << mpCurrentKeyFrame->mTimeStamp << " "
		    <<s_<<" "<<sstar<<" "<<endl;
	    
	    fbiasa<<mpCurrentKeyFrame->mTimeStamp<<" " 
		    <<dbiasa_.at<float>(0)<<" "<<dbiasa_.at<float>(1)<<" "<<dbiasa_.at<float>(2)<<" "<<endl;
        
	    fcondnum<<mpCurrentKeyFrame->mTimeStamp<<" "
                <<w2.at<float>(0)<<" "<<w2.at<float>(1)<<" "<<w2.at<float>(2)<<" "<<w2.at<float>(3)<<" "
                <<w2.at<float>(4)<<" "<<w2.at<float>(5)<<" "<<endl;
		
// 	    ftime<<mpCurrentKeyFrame->mTimeStamp<<" "
// 		<<(t3-t0)/cv::getTickFrequency()*1000<<" "<<endl;
	    
	    fbiasg<<mpCurrentKeyFrame->mTimeStamp<<" "
		<<bgest(0)<<" "<<bgest(1)<<" "<<bgest(2)<<" "<<endl;
		
	    ofstream fRwi(tmpfilepath+"Rwi.txt");
	    fRwi<<Rwieig_(0,0)<<" "<<Rwieig_(0,1)<<" "<<Rwieig_(0,2)<<" "
		<<Rwieig_(1,0)<<" "<<Rwieig_(1,1)<<" "<<Rwieig_(1,2)<<" "
		<<Rwieig_(2,0)<<" "<<Rwieig_(2,1)<<" "<<Rwieig_(2,2)<<endl;
	    fRwi.close();
		
	}
	
	// 提高初始化状态的可靠性
	bool bVIOInited = false;
	if(mbFirstTry)
	{
	    mbFirstTry = false;
	    mnStartTime = mpCurrentKeyFrame->mTimeStamp;
	}
	
	if(pNewestKF->mTimeStamp - mnStartTime >= ConfigParam::GetVINSInitTime())
	{
	    bVIOInited = true;
	}
	
	if(bVIOInited)
	{
	    // 设置所有关键帧的Navstate, scale和bias
	    double scale = s_;
	    mnVINSInitScale = s_;
	    // 世界坐标系下重力加速坐标系
	    cv::Mat gw = Rwi_*GI;
	    mGravityVec = gw.clone();
	    Vector3d gweig = Converter::toVector3d(gw);
	    mRwiInit = Rwi_.clone();
	    
	    // 更新关键帧NavState
	    if(ConfigParam::GetRealTimeFlag())
	    {
		// 停止Local mapping
		RequestStop();
		
		while(!isStopped() && !isFinished())
		{
		    usleep(1000);
		}
	    }
	    
	    SetUpdatingInitPoses(true);
	    {
		unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
		
		int cnt = 0;
		for(vector<KeyFrame *>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++, cnt++)
		{
		    KeyFrame *pKF = *vit;
		    if(pKF->isBad())
			continue;
		    
		    if(pKF!=vScaleGravityKF[cnt])
			cerr<<"pKF!=vScaleGravityKF[cnt], id: "<<pKF->mnId<<" != "<<vScaleGravityKF[cnt]->mnId<<endl;
		    
		    // vslam PR
		    cv::Mat wPc = pKF->GetPoseInverse().rowRange(0,3).col(3);
		    cv::Mat Rwc = pKF->GetPoseInverse().rowRange(0,3).colRange(0,3);
		    
		    // 设置导航状态
		    
		    // P R
		    cv::Mat wPb = scale*wPc + Rwc*pcb;
		    pKF->SetNavStatePos(Converter::toVector3d(wPb));
		    pKF->SetNavStateRot(Converter::toMatrix3d(Rwc*Rcb));
		    
		    // bias
		    pKF->SetNavStateBiasGyr(bgest);
		    pKF->SetNavStateBiasAcc(dbiasa_eig);
		    // delta_bias只在优化中跟新，设置为0
		    pKF->SetNavStateDeltaBg(Eigen::Vector3d::Zero());
		    pKF->SetNavStateDeltaBa(Eigen::Vector3d::Zero());
		    
		    // 步骤4
		    // 计算速度
		    if(pKF != vScaleGravityKF.back())
		    {
			KeyFrame *pKFnext = pKF->GetNextKeyFrame();
			if(!pKFnext)
			    cerr<<"pKFnext is NULL, cnt="<<cnt<<", pKFnext:"<<pKFnext<<endl;
			if(pKFnext!=vScaleGravityKF[cnt+1])
			    cerr<<"pKFnext!=vScaleGravityKF[cnt+1], cnt="<<cnt<<", id: "<<pKFnext->mnId<<" != "<<vScaleGravityKF[cnt+1]->mnId<<endl;
			
			// IMU预积分
			const IMUPreintegrator &imupreint = pKFnext->GetIMUPreInt();
			double dt = imupreint.getDeltaTime();
			cv::Mat dp = Converter::toCvMat(imupreint.getDeltaP());
			cv::Mat Jpba = Converter::toCvMat(imupreint.getJPBiasa());
			cv::Mat wPcnext = pKFnext->GetPoseInverse().rowRange(0,3).col(3);
			cv::Mat Rwcnext = pKFnext->GetPoseInverse().rowRange(0,3).colRange(0,3);
			
			cv::Mat vel = -1./dt*(scale*(wPc-wPcnext) + (Rwc-Rwcnext)*pcb + Rwc*Rcb*(dp+Jpba*dbiasa_) + 0.5*gw*dt*dt);
			Eigen::Vector3d veleig = Converter::toVector3d(vel);
			pKF->SetNavStateVel(veleig);
		    }
		    else
		    {
			cerr<<"-----------here is the last KF in vScaleGravityKF------------"<<endl;
			
			KeyFrame *pKFprev = pKF->GetPrevKeyFrame();
			if(!pKFprev)
			    cerr<<"pKFprev is NULL, cnt="<<cnt<<endl;
			
			if(pKFprev!=vScaleGravityKF[cnt-1])
			    cerr<<"pKFprev!=vScaleGravityKF[cnt-1], cnt="<<cnt<<", id: "<<pKFprev->mnId<<" != "<<vScaleGravityKF[cnt-1]->mnId<<endl;
			
			const IMUPreintegrator &imupreint_prev_cur = pKF->GetIMUPreInt();
			double dt = imupreint_prev_cur.getDeltaTime();
			Eigen::Matrix3d Jvba = imupreint_prev_cur.getJVBiasa();
			Eigen::Vector3d dv = imupreint_prev_cur.getDeltaV();
			
			Eigen::Vector3d velpre = pKFprev->GetNavState().Get_V();
			Eigen::Matrix3d rotpre = pKFprev->GetNavState().Get_RotMatrix();
			Eigen::Vector3d veleig = velpre+gweig*dt+rotpre*(dv+Jvba*dbiasa_eig);
			pKF->SetNavStateVel(veleig);
		    }
		}
		
		// 重新计算IMU预积分
		for(vector<KeyFrame *>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
		{
		    KeyFrame *pKF = *vit;
		    if(pKF->isBad())
			continue;
		    pKF->ComputePreInt();
		}
		
		// 更新位姿
		vector<KeyFrame *> mspKeyFrames = mpMap->GetAllKeyFrames();
		for(std::vector<KeyFrame *>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
		{
		    KeyFrame *pKF = *sit;
		    cv::Mat Tcw = pKF->GetPose();
		    cv::Mat tcw = Tcw.rowRange(0,3).col(3)*scale;
		    tcw.copyTo(Tcw.rowRange(0,3).col(3));    
		    pKF->SetPose(Tcw);
		}
		
		// 更新地图点
		vector<MapPoint *> mspMapPoints = mpMap->GetAllMapPoints();
		for(std::vector<MapPoint *>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
		{
		    MapPoint *pMP = *sit;
		    pMP->UpdateScale(scale);
		}
		std::cout<<std::endl<<"... Map scale updated ..."<<std::endl<<std::endl;
		
		// 更新NavState
		if(pNewestKF!=mpCurrentKeyFrame)
		{
		    KeyFrame *pKF;
		    
		    // 更新bias
		    pKF = pNewestKF;
		    do
		    {
			pKF = pKF->GetNextKeyFrame();
			
			pKF->SetNavStateBiasGyr(bgest);
			pKF->SetNavStateBiasAcc(dbiasa_eig);
			
			pKF->SetNavStateDeltaBg(Eigen::Vector3d::Zero());
			pKF->SetNavStateDeltaBa(Eigen::Vector3d::Zero());
			
		    }while(pKF!=mpCurrentKeyFrame);
		    
		    // 计算预积分
		    pKF = pNewestKF;
		    do
		    {
			pKF = pKF->GetNextKeyFrame();
			pKF->ComputePreInt();
			
		    }while(pKF!=mpCurrentKeyFrame);
		    
		    // 更新pose
		    pKF = pNewestKF;
		    do
		    {
			pKF = pKF->GetNextKeyFrame();
			
			cv::Mat wPc = pKF->GetPoseInverse().rowRange(0,3).col(3);
			cv::Mat Rwc = pKF->GetPoseInverse().rowRange(0,3).colRange(0,3);
			cv::Mat wPb = wPc + Rwc*pcb;
			pKF->SetNavStatePos(Converter::toVector3d(wPb));
			pKF->SetNavStateRot(Converter::toMatrix3d(Rwc*Rcb));
			
			if(pKF!=mpCurrentKeyFrame)
			{
			    KeyFrame *pKFnext = pKF->GetNextKeyFrame();
			    const IMUPreintegrator &imupreint = pKFnext->GetIMUPreInt();
			    double dt = imupreint.getDeltaTime();
			    cv::Mat dp = Converter::toCvMat(imupreint.getDeltaP());
			    cv::Mat Jpba = Converter::toCvMat(imupreint.getJPBiasa());
				cv::Mat wPcnext = pKFnext->GetPoseInverse().rowRange(0,3).col(3);
			    cv::Mat Rwcnext = pKFnext->GetPoseInverse().rowRange(0,3).colRange(0,3);
			    
			    cv::Mat vel = - 1./dt*( (wPc - wPcnext) + (Rwc - Rwcnext)*pcb + Rwc*Rcb*(dp + Jpba*dbiasa_) + 0.5*gw*dt*dt );
			    Eigen::Vector3d veleig = Converter::toVector3d(vel);
			    pKF->SetNavStateVel(veleig);  
			}
			else
			{
			    // If this is the last KeyFrame, no 'next' KeyFrame exists
			    KeyFrame* pKFprev = pKF->GetPrevKeyFrame();
			    const IMUPreintegrator& imupreint_prev_cur = pKF->GetIMUPreInt();
			    double dt = imupreint_prev_cur.getDeltaTime();
			    Eigen::Matrix3d Jvba = imupreint_prev_cur.getJVBiasa();
			    Eigen::Vector3d dv = imupreint_prev_cur.getDeltaV();
			    //
			    Eigen::Vector3d velpre = pKFprev->GetNavState().Get_V();
			    Eigen::Matrix3d rotpre = pKFprev->GetNavState().Get_RotMatrix();
			    Eigen::Vector3d veleig = velpre + gweig*dt + rotpre*( dv + Jvba*dbiasa_eig );
			    pKF->SetNavStateVel(veleig);
			}
			
		    }while(pKF!=mpCurrentKeyFrame);
		
		}
		std::cout<<std::endl<<"... Map NavState updated ..."<<std::endl<<std::endl;
		SetFirstVINSInited(true);
		SetVINSInited(true);
	    }
	    SetUpdatingInitPoses(false);
	    
	    if(ConfigParam::GetRealTimeFlag())
	    {
		Release();
	    }
	    
	    // 初始化后运行GBA
	    unsigned long nGBAKF = mpCurrentKeyFrame->mnId;
	    Optimizer::GlobalBundleAdjustmentNavStatePRV(mpMap, mGravityVec, 10, NULL, nGBAKF, false);
	    cerr<<"finish global BA after vins init"<<endl;
	    
	    // 更新
	    if(ConfigParam::GetRealTimeFlag())
	    {
		
		RequestStop();
		
		while(!isStopped() && !isFinished())
		{
		    usleep(1000);
		}
		
		cv::Mat cvTbc = ConfigParam::GetMatTbc();
		// 更新状态
		{
		    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
		    
		    // 更新KF
		    list<KeyFrame *> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(), mpMap->mvpKeyFrameOrigins.end());
		    while(!lpKFtoCheck.empty())
		    {
			KeyFrame *pKF = lpKFtoCheck.front();
			const set<KeyFrame *> sChilds = pKF->GetChilds();
			cv::Mat Twc = pKF->GetPoseInverse();
			
			for(set<KeyFrame *>::const_iterator sit=sChilds.begin(); sit!=sChilds.end(); sit++)
			{
			    KeyFrame *pChild = *sit;
			    if(pChild->mnBAGlobalForKF!=nGBAKF)
			    {
				cerr<<"correct KF after gBA in VI init: "<<pChild->mnId<<endl;
				cv::Mat Tchildc = pChild->GetPose()*Twc;
				pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
				pChild->mnBAGlobalForKF=nGBAKF;
				
				// 更新导航状态
				pChild->mNavStateGBA = pChild->GetNavState();
				cv::Mat TwbGBA = Converter::toCvMatInverse(cvTbc*pChild->mTcwGBA);
				Matrix3d RwbGBA = Converter::toMatrix3d(TwbGBA.rowRange(0,3).colRange(0,3));
				Vector3d PwbGBA = Converter::toVector3d(TwbGBA.rowRange(0,3).col(3));
				Matrix3d Rw1 = pChild->mNavStateGBA.Get_RotMatrix();
				Vector3d Vw1 = pChild->mNavStateGBA.Get_V();
				Vector3d Vw2 = RwbGBA*Rw1.transpose()*Vw1;   // bV1 = bV2 ==> Rwb1^T*wV1 = Rwb2^T*wV2 ==> wV2 = Rwb2*Rwb1^T*wV1
				pChild->mNavStateGBA.Set_Pos(PwbGBA);
				pChild->mNavStateGBA.Set_Rot(RwbGBA);
				pChild->mNavStateGBA.Set_Vel(Vw2);
			    }
			    lpKFtoCheck.push_back(pChild);    
			}
			
			pKF->mTcwBefGBA = pKF->GetPose();
			pKF->mNavStateBefGBA = pKF->GetNavState();
			pKF->SetNavState(pKF->mNavStateGBA);
			pKF->UpdatePoseFromNS(cvTbc);
			
			lpKFtoCheck.pop_front();
		    }
		    
		    // 更新MP
		    const vector<MapPoint *> vpMPs = mpMap->GetAllMapPoints();
		    for(size_t i=0; i<vpMPs.size(); i++)
		    {
			MapPoint *pMP = vpMPs[i];
			
			if(pMP->isBad())
			    continue;
			
			if(pMP->mnBAGlobalForKF == nGBAKF)
			{
			    pMP->SetWorldPos(pMP->mPosGBA);
			}
			else
			{
			    // 根据参考KF位姿更新
			    KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
			    
			    if(pRefKF->mnBAGlobalForKF!=nGBAKF)
				continue;
			    
			    // 使用未矫正的位姿投影到相机坐标
			    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
			    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
			    cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;
			    
			    // 使用矫正位姿投影到世界坐标
			    cv::Mat Twc = pRefKF->GetPoseInverse();
			    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
			    cv::Mat twc = Twc.rowRange(0,3).col(3);
			    
			    pMP->SetWorldPos(Rwc*Xc+twc);
			}
		    }
		    
		    cout << "Map updated!" << endl;
		    
		    // 设置地图更新标志
		    SetMapUpdateFlagInTracking(true);
		    
		    Release();
		}
	    }
	    
	    SetFlagInitGBAFinish(true);
	}
	
	for(int i=0; i<N; i++)
	{
	    if(vKFInit[i])
		delete vKFInit[i];
	}
	
	return bVIOInited;
	
    }

    
    
    
    
    
    
    
    
    
    
    
    
    // 增加KF到local window
    void LocalMapping::AddToLocalWindow(KeyFrame* pKF)
    {
        
        mlLocalKeyFrames.push_back(pKF);
        
        if(mlLocalKeyFrames.size()>mnLocalWindowSize)
        {
            mlLocalKeyFrames.pop_front();
        }
        else
        {
            KeyFrame *pKF0 = mlLocalKeyFrames.front();
            while(mlLocalKeyFrames.size()<mnLocalWindowSize && pKF0->GetPrevKeyFrame()!=NULL)
            {
                pKF0 = pKF0->GetPrevKeyFrame();
                mlLocalKeyFrames.push_front(pKF0);
            }
        }
        
    }

    
    
    // 删除LocalWindow中的坏KF
    void LocalMapping::DeleteBadInLocalWindow(void)
    {
        std::list<KeyFrame *>::iterator lit = mlLocalKeyFrames.begin();
        
        while(lit!=mlLocalKeyFrames.end())
        {
            KeyFrame *pKF = *lit;
            
            // test log
            if(!pKF)
                cout<<"pKF null?"<<endl;
            if(pKF->isBad())
            {
                lit = mlLocalKeyFrames.erase(lit);
            }
            else
            {
                lit++;
            }
        }
        
    }
 
    
    
    /*********************************************************/
    
    // 构造函数，成员变量初始化。
    LocalMapping::LocalMapping( Map *pMap, const float bMonocular, ConfigParam *pParams):
        mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
        mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
    {
        mpParams = pParams;
        mnLocalWindowSize = ConfigParam::GetLocalWindowSize();
        cout<<"mnLocalWindowSize:"<<mnLocalWindowSize<<endl;
        
        mbVINSInited = false;
        mbFirstTry = true;
        mbFirstVINSInited = false;
        
        mbUpdatingInitPoses = false;
        mbCopyInitKFs = false;
        mbInitGBAFinish = false;

    }

    // 设置进程间的对象指针，用于数据交互。
    void LocalMapping::SetLoopCloser(LoopClosing *pLoopCloser)
    {
        mpLoopCloser = pLoopCloser;
    }
    void LocalMapping::SetTracker(Tracking *pTracker)
    {
        mpTracker = pTracker;
    }


    // Local Mapping线程入口函数。
    // 1.设置标志位，线程繁忙。
    // 2.队列中有需要插入的关键帧。
    //      局部地图跟踪中与MP匹配的特征点与KF关联，插入地图中。
    //      剔除不合格的MP。
    //      通过帧间运动三角化恢复MP。
    //      融合重复MP。
    //      局部BA优化。
    //      关键帧插入队列mlpLoopKeyFrameQueue。
    // 3.结束线程，设置线程标志位，接收新的KF。
    void LocalMapping::Run()
    {

        mbFinished = false;

        while(1)
        {

            // 告知Tracking线程 Local Mapping线程处于忙碌状态。
            // Local Mapping处理的线程都是Tracking发到mlNewKeyFrames。
            // 没有处理完以前不发送，设置mbAcceptKeyFrames变量表示状态。
            SetAcceptKeyFrames(false);

            // 等待处理的关键帧队列mlNewKeyFrames不能为空。
            if(CheckNewKeyFrames())
            {
                // 步骤1 从队列中取出一帧，计算特征点的BoW向量，将关键帧插入到地图。
                ProcessNewKeyFrame();

                // 步骤2 剔除ProcessNewKeyFrame()中引入的不合格的MapPoints。
                MapPointCulling();

                // 步骤3 运动过程中，与相邻的关键帧三角化恢复出一些MapPoints。
                CreateNewMapPoints();

                // 当前帧是Tracking插入队列中最后一帧关键帧(ProcessNewKeyFrame函数会pop队列)。
                if( !CheckNewKeyFrames() ) 
                {
                    // 检查当前帧与相邻关键帧重复的MapPoints，并融合。
                    SearchInNeighbors();
                }

                mbAbortBA = false;
                
                // 当前帧是队列中的最后一帧，且闭环检测没有发送中断申请。
                if( !CheckNewKeyFrames() && !stopRequested())
                {
                    // 步骤4 局部BA
                    if(mpMap->KeyFramesInMap()>2)
		    {
			if(!GetVINSInited())
			{
			    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpMap, this);
			}
			else
			{
			    Optimizer::LocalBAPRVIDP(mpCurrentKeyFrame, mlLocalKeyFrames, &mbAbortBA, mpMap, mGravityVec, this);
			}
			
		    }
		    
		    // 非实时模式
		    if(!ConfigParam::GetRealTimeFlag())
		    {
			if(!GetVINSInited())
			{
			    bool tmpbool = TryInitVIO();
			    SetVINSInited(tmpbool);
			    if(tmpbool)
			    {
				mpMap->UpdateScale(mnVINSInitScale);
				SetFirstVINSInited(true);
			    }
			}
		    }
		    
                    // 步骤5 剔除冗余关键帧。某关键帧90%的点云被其他关键帧观测，剔除。
                    // Tracking 线程通过InsertKeyFrame函数添加的条件宽松，KF会比较多，便于跟踪。
                    // 在这个删除冗余关键帧。
                    KeyFrameCulling();
                }

                // 将当前关键帧加入闭环检测队列。
                if(GetFlagInitGBAFinish())
		    mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);

            }   // 当前关键帧队列不为空。

            // 停止Local Mapping
            else if(Stop())
            {
                // 收到停止请求，但是进程没有完成。
                while( isStopped() && !CheckFinish() )
                {
                    // 延时3000us
                    std::this_thread::sleep_for(std::chrono::milliseconds(3));
                }
                // 完成Local Mapping。
                if(CheckFinish())
                    break;
            }

            // 是否进行重置。
            ResetIfRequested();

            // 设置线程空闲，用于Tracking查看。
            SetAcceptKeyFrames(true);

            // 进程完成。
            if(CheckFinish())
                break;

            // 延时3000us
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }   // while(1) 

        // Local Mapping线程完成。    
        SetFinish();

    }



    // 插入关键帧。
    // Tracking线程在CreateKeyFrame()中调用。
    // 插入关键帧到mlNewKeyFrames。
    // 仅仅是将关键帧插入到队列，之后从队列中pop。
    void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexNewKFs);

        // 关键帧插入到列表中。
        mlNewKeyFrames.push_back(pKF);
        mbAbortBA = true;
    }



    // 查看队列mlNewKeyFrames中是否还有等待插入的关键帧。
    // return 若不为空，true。
    bool LocalMapping::CheckNewKeyFrames()
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        return (!mlNewKeyFrames.empty());
    }



    // 处理队列中的关键帧。
    // 1.计算BoW，加速匹配和三角化(双目和RGBD)新的MapPoints.
    // 2.新匹配的MapPoints关联当前关键帧帧。
    // 3.插入关键帧，更新Covisibility图和Essential图。
    void LocalMapping::ProcessNewKeyFrame()
    {

        // 步骤1 从队列中取一帧关键帧，队列中关键帧数量变少。
        {
            unique_lock<mutex> lock(mMutexNewKFs);
            mpCurrentKeyFrame = mlNewKeyFrames.front();
            mlNewKeyFrames.pop_front();
        }

        // 步骤2 计算当前关键帧的BoW向量。
        mpCurrentKeyFrame->ComputeBoW();

        // 步骤3 跟踪局部地图过程中新的MapPoints和当前关键帧关联起来。
        // TrackLocalMap()只对局部地图中的MapPoints与当前关键帧进行了匹配，保存了匹配点云，但没有更新MP的属性。
        const vector<MapPoint *> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

        for(size_t i=0; i<vpMapPointMatches.size(); i++)
        {
            MapPoint *pMP = vpMapPointMatches[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    // 为当前帧在tracking过程跟踪到的MapPoint添加属性，观测关键帧， 观测方向， 最佳描述子。
                    if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                    {
                        // 步骤3.1 添加观测。
                        pMP->AddObservation(mpCurrentKeyFrame, i);
                        // 步骤3.2 获得该点的平均观测方向。
                        pMP->UpdateNormalAndDepth();
                        // 步骤3.3 加入关键帧后，更新3D点的最佳描述子。
                        pMP->ComputeDistinctiveDescriptors();
                    }

                    // 只发生在双目或者RGBD跟踪过程中插入的MapPoints
                    else
                    {
                        mlpRecentAddedMapPoints.push_back(pMP);
                    }  
                }
            }
        }

        // 步骤4 更新关键帧间的连接关系，Convisible图和Essential图(tree)
        mpCurrentKeyFrame->UpdateConnections();
        
	// 删除Local window中的bad KF
	DeleteBadInLocalWindow();
	
	// 添加CKF到Local window中
	AddToLocalWindow(mpCurrentKeyFrame);
	
        // 步骤5 将关键帧插入到地图中。
        mpMap->AddKeyFrame(mpCurrentKeyFrame);

    }

    

    // 剔除ProcessingNewKeyFrame()和CreateNewMapPoints()引入的不合格的新MapPoints
    void LocalMapping::MapPointCulling()
    {
        // 检查最近添加的MapPoints。
        // 在CreateNewMapPoints中有新添加点云。
        // 单目在ProcessingNewKeyFrame中没有添加点云，双目RGBD会引入。
        list<MapPoint *>::iterator lit = mlpRecentAddedMapPoints.begin();          
        const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

        int nThObs;
        if(mbMonocular)
            nThObs = 2;
        else
            nThObs = 3;
        const int cnThObs = nThObs;

        // 遍历所有需要检查的MapPoints。
        while(lit != mlpRecentAddedMapPoints.end())
        {
            MapPoint *pMP = *lit;
            // 步骤1 已经是坏点的直接剔除。
            if(pMP->isBad())
            {
                lit = mlpRecentAddedMapPoints.erase(lit);
            }
            
            // 步骤2 跟踪到该MapPoint的Frame数小于预测可跟踪（在帧内，但是不一定有匹配特征点）的帧数的0.25，剔除。
            // IncreaseFound / IncreaseVisible < 25%，不一定是关键帧。
            else if(pMP->GetFoundRatio()<0.25f)
            {
                pMP->SetBadFlag();
                lit = mlpRecentAddedMapPoints.erase(lit);
            }

            // 步骤3 该MapPoint建立开始>=2帧,但是能观测点的关键帧数不超过cnThobs，剔除，单目阈值2。
            else if( ((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
            {
                pMP->SetBadFlag();
                lit = mlpRecentAddedMapPoints.erase(lit);
            }

            // 步骤4 从建立开始，超过3帧，从新添加点云队列中剔除，不参与剔除判断。
            else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
                lit = mlpRecentAddedMapPoints.erase(lit);
            else 
                lit++;
        }

    }



    // 当前关键帧与其共视程度比较高的关键帧通过三角化得到的MapPoints。
	// 目前看，单目除了初始化外，地图点云主要来自于这个函数。
    void LocalMapping::CreateNewMapPoints()
    {
        // 共视帧阈值设置。
        int nn=10;
        if(mbMonocular)
            nn = 20;

        // 步骤1 在Covisible图中找到当前关键帧共视程度最高的前nn帧相邻帧。
        const vector<KeyFrame *> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

        ORBmatcher matcher(0.6, false);

        // 当前关键帧相对初始世界坐标的位姿。
        cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
        cv::Mat Rwc1 = Rcw1.t();
        cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
        cv::Mat Tcw1(3,4,CV_32F);
        Rcw1.copyTo(Tcw1.colRange(0,3));
        tcw1.copyTo(Tcw1.col(3));

        // 得到当前关键帧在世界坐标系中的坐标。
        cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

		// 相机内参
        const float &fx1 = mpCurrentKeyFrame->fx;
        const float &fy1 = mpCurrentKeyFrame->fy;
        const float &cx1 = mpCurrentKeyFrame->cx;
        const float &cy1 = mpCurrentKeyFrame->cy;
        const float &invfx1 = mpCurrentKeyFrame->invfx;
        const float &invfy1 = mpCurrentKeyFrame->invfy;

        const float ratioFactor = 1.5f * mpCurrentKeyFrame->mfScaleFactor;

        int nnew = 0;

        // 极线搜索，匹配约束，三角化。
        // 步骤2 遍历所有相邻关键帧vpNeightKFs。
        for(size_t i=0; i<vpNeighKFs.size(); i++)
        {
			// 还有新插入的关键帧没处理。
           if(i>0 && CheckNewKeyFrames()) 
               return;

           KeyFrame *pKF2 = vpNeighKFs[i];

           // 临接关键帧在世界坐标系中的坐标。
           cv::Mat Ow2 = pKF2->GetCameraCenter();
           // 基线向量，两个关键帧之间的位移。
           cv::Mat  vBaseline = Ow2-Ow1;
           // 基线长度。
		   // 用于判断是否可以三角化，但没有在三角化中直接使用。
           const float baseline = cv::norm(vBaseline);

           // 步骤3 判断基线是否足够长。
           // 立体相机。
           if(!mbMonocular)
           {
               if(baseline<pKF2->mb)
                   continue;
           }
           // 单目。
           else
           {
               // 邻接关键帧的场景深度中值。
               const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
               // 基线与场景深度的比值。
               const float ratioBaselineDepth = baseline/medianDepthKF2;
               // 如果比值特别小，不考虑当前邻接的关键帧，不生成3D点。
               if(ratioBaselineDepth<0.01)
                   continue;
           }
           
           // 步骤4 根据两关键帧之间的位姿计算他们的基本矩阵。 
           cv::Mat F12 = ComputeF12(mpCurrentKeyFrame, pKF2);

           // 步骤5 通过极线约束限制匹配的搜索范围，进行特征点匹配。
           vector<pair<size_t, size_t> > vMatchedIndices;	// 存储两关键帧新的特征匹配点的索引。 
           matcher.SearchForTriangulation(mpCurrentKeyFrame, pKF2, F12, vMatchedIndices, false);

           cv::Mat Rcw2 = pKF2->GetRotation();
           cv::Mat Rwc2 = Rcw2.t();
           cv::Mat tcw2 = pKF2->GetTranslation();
           cv::Mat Tcw2(3, 4, CV_32F);
           Rcw2.copyTo(Tcw2.colRange(0,3));
           tcw2.copyTo(Tcw2.col(3));

          const float &fx2 = pKF2->fx; 
          const float &fy2 = pKF2->fy;
          const float &cx2 = pKF2->cx;
          const float &cy2 = pKF2->cy;
          const float &invfx2 = pKF2->invfx;
          const float &invfy2 = pKF2->invfy;

          // 步骤6 匹配特征通过三角化生成3D点。
          const int nmatches = vMatchedIndices.size();
          for(int ikp = 0; ikp<nmatches; ikp++)
          {
              // 步骤6.1 取出匹配的特征点。

              // 当前匹配特征点在当前关键帧中的索引。
              const int &idx1 = vMatchedIndices[ikp].first;

              // 当前匹配特征点在邻接关键帧中的索引。
              const int &idx2 = vMatchedIndices[ikp].second;

              // 当前匹配特征点在当前关键帧中的特征点。
              const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
              // mvuRight存放着双目的深度值，如果不是双目，为-1。
              const float kp1_ur = mpCurrentKeyFrame->mvuRight[idx1];
              bool bStereo1 = kp1_ur>=0;

              // 当前匹配对在邻接关键帧中的特征点。
              const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
              // mvuRight存放双目深度，不是双目时，为-1。
              const float kp2_ur = pKF2->mvuRight[idx2];
              bool bStereo2 = kp2_ur>=0;

              // 步骤6.2 利用匹配点反投影得到视角差。
              // 特征点反投影到相机坐标系。
              cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
              cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

              // 由相机坐标系转到世界坐标系，得到视角差余弦值。
              cv::Mat ray1 = Rwc1*xn1;
              cv::Mat ray2 = Rwc2*xn2;
              const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

              // 加1是为了让cosParallaxStereo随便初始化为一个很大的值。
              float cosParallaxStereo = cosParallaxRays+1;
              float cosParallaxStereo1 = cosParallaxStereo;
              float cosParallaxStereo2 = cosParallaxStereo;

              // 步骤6.3 对于双目，利用双目得到视差角。
              if(bStereo1)  // 双目，有深度。
                  cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2, mpCurrentKeyFrame->mvDepth[idx1]));
              else if(bStereo2) // 双目，有深度。
                  cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2, pKF2->mvDepth[idx2]));

              // 得到双目观测的视差角。
              cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);

              // 步骤6.4 三角化恢复3D点。
              cv::Mat x3D;
              
              // cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998)表示视角差正常。
              // cosParallaxRays < cosParallaxStereo表示视角小。

              // 视角正常但是小视角。
              if( cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998) && cosParallaxRays < cosParallaxStereo )
              {
                  // 线性三角化方法，见Initialize.cpp的Triangulate函数。
                  cv::Mat A(4, 4, CV_32F);
                  A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                  A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                  A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                  A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                  cv::Mat w, u, vt;
                  cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

                  x3D = vt.row(3).t();

                  if(x3D.at<float>(3) == 0)
                      continue;

                  // 欧式坐标。
                  x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
              }

              else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
              {
                  x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);
              }

              else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
              {
                  x3D = pKF2->UnprojectStereo(idx2);
              }

              // 视差很小。
              else
                  continue;

              cv::Mat x3Dt = x3D.t();

              // 步骤6.5 检测生成的3D点是否在相机前方。
              float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
              if(z1<=0)
                  continue;

              float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
              if(z2<=0)
                  continue;

              // 步骤6.6 计算当前3D点在当前关键帧下的重投影误差。
              const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
              const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
              const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
              const float invz1 = 1.0/z1;

              // 不是双目。
              if(!bStereo1)
              {
                  float u1 = fx1*x1*invz1+cx1;
                  float v1 = fy1*y1*invz1+cy1;
                  float errX1 = u1 - kp1.pt.x;
                  float errY1 = v1 - kp1.pt.y;
                  // 基于卡方检验计算出的阈值，误差服从N(0,1)。
                  if( (errX1*errX1+errY1*errY1)>5.991*sigmaSquare1 )
                      continue;
              }

              // 双目。
              else
              {
                  float u1 = fx1*x1*invz1+cx1;
                  float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                  float v1 = fy1*y1*invz1+cy1;
                  float errX1 = u1 - kp1.pt.x;
                  float errY1 = v1 - kp1.pt.y;
                  float errX1_r = u1_r - kp1_ur;
                  if( (errX1*errX1+errY1*errY1+errX1_r*errX1_r) > 7.8*sigmaSquare1 )
                      continue;
              }

              // 计算3D点在临接关键帧下的重投影误差。
              const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
              const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
              const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
              const float invz2 = 1.0/z2;
              // 非双目。
              if(!bStereo2)
              {
                  float u2 = fx2*x2*invz2+cx2;
                  float v2 = fy2*y2*invz2+cy2;
                  float errX2 = u2 - kp2.pt.x;
                  float errY2 = v2 - kp2.pt.y;
                  if( (errX2*errX2+errY2*errY2)>5.991*sigmaSquare2 )
                      continue;
              }

              // 双目。
              else
              {
                  float u2 = fx2*x2*invz2+cx2;
                  float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                  float v2 = fy2*y2*invz2+cy2;
                  float errX2 = u2 - kp2.pt.x;
                  float errY2 = v2 - kp2.pt.y;
                  float errX2_r = u2_r - kp2_ur;
                  if( (errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2 )
                      continue;
              }

              // 步骤6.7 检查尺度连续性。

              // 世界坐标系下，3D点与相机间的向量，方向由相机指向3D点。
              cv::Mat normal1 = x3D-Ow1;
              float dist1 = cv::norm(normal1);

              cv::Mat normal2 = x3D-Ow2;
              float dist2 = cv::norm(normal2);

              if(dist1==0||dist2==0)
                  continue;

              // ratioDist不考虑金字塔尺度下的距离比例。
              const float ratioDist = dist2/dist1;

              // 金字塔尺度因子。
              const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

              // 尺度连续判断。
              if(ratioDist*ratioFactor<ratioOctave || ratioDist > ratioOctave*ratioFactor)
                  continue;

              // 步骤6.8 三角化生成的3D点成功，构造MapPoint
              MapPoint *pMP = new MapPoint(x3D, mpCurrentKeyFrame, mpMap);

              // 步骤6.9 为该MapPoint添加属性。
              // a. 观测到该MapPoint的关键帧。
              // b. 该MapPoint的描述子。
              // c. 该MapPoint的平均观测方向。
              pMP->AddObservation(mpCurrentKeyFrame, idx1);
              pMP->AddObservation(pKF2, idx2);

              mpCurrentKeyFrame->AddMapPoint(pMP, idx1);
              pKF2->AddMapPoint(pMP, idx2);

              pMP->ComputeDistinctiveDescriptors();
              
              pMP->UpdateNormalAndDepth();

              mpMap->AddMapPoint(pMP);

              // 步骤6.9 将新生程的点放入检测队列中，通过MapPointCulling函数的检验。
              mlpRecentAddedMapPoints.push_back(pMP);

              // 新添加MapPoint数量。
              nnew++;
          } // 步骤6，三角化生成3D点。
        }   // 遍历所有关键帧。

    }



    // 检查并融合当前关键帧与相邻关键帧重复的MapPoints。
    void LocalMapping::SearchInNeighbors()
    {
        // 步骤1 获取当前关键帧在Covisible图中的权重排名前nn的邻接关键帧。
        // 找到当前帧一级相邻与二级相邻关键帧。
        int nn = 10;
        if(mbMonocular)
            nn = 20;
        const vector<KeyFrame *> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

        vector<KeyFrame *> vpTargetKFs;
        for(vector<KeyFrame *>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
        {
            KeyFrame *pKFi = *vit;
            if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
                continue;

            // 加入相邻帧。
            vpTargetKFs.push_back(pKFi);
            // 标记加入，用于判断，防止重复添加。
            pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

            // 扩展到二级相邻帧，就是当前关键帧相邻关键帧在Covisible图中的邻接帧。
            const vector<KeyFrame *> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
            for(vector<KeyFrame *>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
            {
                KeyFrame * pKFi2 = *vit2;
                if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF == mpCurrentKeyFrame->mnId || pKFi2->mnId == mpCurrentKeyFrame->mnId)
                    continue;
                vpTargetKFs.push_back(pKFi2);
            }
        }

        ORBmatcher matcher;

        // 步骤2 将当前关键帧的MapPoint分别与一级二级邻接关键帧进行融合。
		// 其中投影的是当前关键帧的地图点云。
        vector<MapPoint *> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
        for(vector<KeyFrame *>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
        {
            KeyFrame * pKFi = *vit;

            // 将当前关键帧的MapPoint投影到pKFi中，判断是否有重复的MP。
            // 1. 如果当前帧MapPoint能匹配关键帧中的特征点，并且该点对应有MP，将两个MP融合(更新MP的最大可被观测数)。
            // 2. 如果当前帧MapPoint能匹配关键帧中的特征点，并且该点没有对应的MP，添加为MP。
            matcher.Fuse(pKFi, vpMapPointMatches);
        }

        // 用于存储一级邻接和二级邻接关键帧所有MapPoint的集合。
        vector<MapPoint *> vpFuseCandidates;
        vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

        // 步骤3 将一级与二级相邻关键帧的MapPoints分别与当前关键帧的MapPoint进行融合。
		// 其中投影的是临接关键帧的点云。
        // 遍历所有邻接关键帧。
        for(vector<KeyFrame *>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
        {
            KeyFrame *pKFi = *vitKF;

            vector<MapPoint *> vpMapPointsKFi = pKFi->GetMapPointMatches();

            // 遍历邻接关键帧中的MapPoints。
            for(vector<MapPoint *>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
            {
                MapPoint *pMP = *vitMP;
                if(!pMP)
                    continue;

                // 判断pMP是否为坏点，或者是否已经加入vpFuseCandidates。
                if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                    continue;

                // 加入集合，标记。
                pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
                vpFuseCandidates.push_back(pMP);
            }
        }

		// 将临接关键帧的MapPoint投影到当前关键帧中，判断是否有重复的MP。
        matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates);

        // 步骤4 更新当前关键帧的MapPoint的描述子，深度，观测方向等属性。
        vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
        {
            MapPoint * pMP = vpMapPointMatches[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    // 在所有找到pMP的关键帧中，获取最佳描述子。
                    pMP->ComputeDistinctiveDescriptors();
                    
                    // 更新平均观测方向和观测距离。
                    pMP->UpdateNormalAndDepth();
                }
            }
        }

        // 步骤5 更新当前帧与其他帧的连接关系，Covisible图。
		// 因为进行了点云更新，可能会有新的共同点云。
        mpCurrentKeyFrame->UpdateConnections();
    }



    // 根据两关键帧的位姿计算两帧之间的基本矩阵F12。
    cv::Mat LocalMapping::ComputeF12(KeyFrame * &pKF1, KeyFrame * &pKF2)
    {
        // 本征矩阵E=t12 x R12;
        // 基本矩阵F=inv(K1‘t)*E*inv(K2)。

        cv::Mat R1w = pKF1->GetRotation();
        cv::Mat t1w = pKF1->GetTranslation();
        cv::Mat R2w = pKF2->GetRotation();
        cv::Mat t2w = pKF2->GetTranslation();

        // .t表示转置。
        cv::Mat R12 = R1w*R2w.t();
        cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

        cv::Mat t12x = SkewSymmetricMatrix(t12);

        const cv::Mat &K1 = pKF1->mK;
        const cv::Mat &K2 = pKF2->mK;

        return K1.t().inv()*t12x*R12*K2.inv();

    }



    // 发送停止请求。
    void LocalMapping::RequestStop()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopRequested = true;
        unique_lock<mutex> lock2(mMutexNewKFs);
        mbAbortBA = true;
    }

    // 判断是否停止。
    bool LocalMapping::Stop()
    {
        unique_lock<mutex> lock(mMutexStop);
        if(mbStopRequested && !mbNotStop)
        {
            mbStopped = true;
            cout << "Local Mapping STOP" << endl;
            return true;
        }

        return false;
    }

    // 表示线程停止状态。
    // true 线程停止，false 未停止。
   bool LocalMapping::isStopped()
   {
       unique_lock<mutex> lock(mMutexStop);
       return mbStopped;
   } 

   // 表示停止请求状态。
   // true 发送停止请求，fasle未发送停止请求。
   bool LocalMapping::stopRequested()
   {
       unique_lock<mutex> lock(mMutexStop);
       return mbStopRequested;
   }

   // 释放线程
   void LocalMapping::Release()
   {
       unique_lock<mutex> lock(mMutexStop);
       unique_lock<mutex> lock2(mMutexFinish);
       if(mbFinished)
           return;
       mbStopped = false;
       mbStopRequested = false;
       for(list<KeyFrame *>::iterator lit=mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
           delete *lit;
       mlNewKeyFrames.clear();

       cout << "Local Mapping RELEASE" << endl;

   }

   // 是否可以接收关键帧。
   bool LocalMapping::AcceptKeyFrames()
   {
       unique_lock<mutex> lock(mMutexAccept);
       return mbAcceptKeyFrames;
   }

   // 设置是否接收关键帧。
   void LocalMapping::SetAcceptKeyFrames(bool flag)
   {
       unique_lock<mutex> lock(mMutexAccept);
       mbAcceptKeyFrames = flag;
   }

   // 设置不停止线程。
   // 参数，true不停止线程，false停止线程。
   // return true 停止线程，false 不停止线程。
   bool LocalMapping::SetNotStop(bool flag)
   {
       unique_lock<mutex> lock(mMutexStop);

       if(flag && mbStopped)
           return false;

       mbNotStop = flag;

       return true;
   }

   // 中断BA
   void LocalMapping::InterruptBA()
   {
       mbAbortBA = true;
   }



   // 关键帧剔除，不会剔除当前关键帧mpCurrentKeyFrame，剔除的是mpCuKF的共视关键帧。
   // 在Covisiblity图中的关键帧，其中90%以上的MapPoints能被其他至少3个尺度更好的关键帧观测到，则认为冗余，剔除。
   void LocalMapping::KeyFrameCulling()
   {
       
       if(ConfigParam::GetRealTimeFlag())
       {
	   if(GetFlagCopyInitKFs())
	       return;
       }
       SetFlagCopyInitKFs(true);
       
       // 步骤1 根据Covisiblity图提取当前帧的共视关键帧。
       vector<KeyFrame *> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();
       
       KeyFrame *pOldestLocalKF = mlLocalKeyFrames.front();
       KeyFrame *pPrevLocalKF = pOldestLocalKF->GetPrevKeyFrame();
       KeyFrame *pNewestLocalKF = mlLocalKeyFrames.back();
       
       // test log
       if(pOldestLocalKF->isBad()) 
	   cerr<<"pOldestLocalKF is bad, check 1. id: "<<pOldestLocalKF->mnId<<endl;
       
       if(pPrevLocalKF) 
	   if(pPrevLocalKF->isBad()) 
	       cerr<<"pPrevLocalKF is bad, check 1. id: "<<pPrevLocalKF->mnId<<endl;
	   
       if(pNewestLocalKF->isBad()) 
	   cerr<<"pNewestLocalKF is bad, check 1. id: "<<pNewestLocalKF->mnId<<endl;
       
       
       // 遍历局部关键帧。
       for(vector<KeyFrame *>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
       {
           KeyFrame *pKF = *vit;
           if(pKF->mnId==0)
               continue;
           
           // 不剔除pOldestLocalKF和它的前帧
	   if(pKF==pOldestLocalKF || pKF == pPrevLocalKF)
	       continue;
	   
	   // 检查两帧之间时间间隔，小于0.5不剔除
	   KeyFrame *pPrevKF = pKF->GetPrevKeyFrame();
	   KeyFrame *pNextKF = pKF->GetNextKeyFrame();
	   if(pPrevKF && pNextKF && !GetVINSInited())
	   {
	       if(fabs(pNextKF->mTimeStamp - pPrevKF->mTimeStamp) > 0.5)
		   continue;
	   }
	   
	   // 保证当前关键帧的前帧不被剔除
	   if(pKF->GetNextKeyFrame() == mpCurrentKeyFrame)
	       continue;
	   if(pKF->mTimeStamp >= mpCurrentKeyFrame->mTimeStamp - 0.11)
	       continue;
	   
	   if(pPrevKF && pNextKF)
	   {
	       double timegap = 0.51;
	       if(GetVINSInited() && pKF->mTimeStamp < mpCurrentKeyFrame->mTimeStamp -4.0)
		   timegap = 3.01;
	       
	       if(fabs(pNextKF->mTimeStamp - pPrevKF->mTimeStamp) > timegap)
		   continue;
	   }
	   
	   // 步骤2 提取每个共视关键帧的MapPoints。
           const vector<MapPoint *> vpMapPoints = pKF->GetMapPointMatches();

	   // 设置阈值。
           int nObs = 2;
           if(mbMonocular)
               nObs = 3;
           const int thObs=nObs;
           int nRedundantObservations=0;
           int nMPs=0;						// 存储的是参与判断的合格的点云数量。

           // 步骤3 遍历该局部关键帧的MapPoints，判断是否有90%以上的MapPoints能被其至少3个他关键帧观测到。
           for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
           {
               MapPoint *pMP = vpMapPoints[i];
               if(pMP)
               {
                   if(!pMP->isBad())
                   {
                       // 双目，只考虑近处的MapPoints，
                       if(!mbMonocular)
                       {
                           if(pKF->mvDepth[i] > pKF->mThDepth || pKF->mvDepth[i]<0) 
                               continue;
                       }

                       nMPs++;

                       // MapPoints至少被3个关键帧观测到。
                       if(pMP->Observations() > thObs)
                       {
                           const int &scaleLevel = pKF->mvKeysUn[i].octave;
                           const mapMapPointObs/*map<KeyFrame *, size_t>*/ observations = pMP->GetObservations();
                           // 判断该MapPoint是否同时被三个尺度更好关键帧观测到。
                           int nObs=0;

                           // 遍历所有观测到MP的关键帧。
                           for(mapMapPointObs/*map<KeyFrame *, size_t>*/::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                           {
                               KeyFrame *pKFi = mit->first;
                               if(pKFi==pKF)
                                   continue;
                               const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                               // 尺度约束，要求MapPoints在关键帧pKFi的特征尺度近似于关键帧pKF的特征尺度。
                               if(scaleLeveli <= scaleLevel+1)
                               {
                                   nObs++;
                                   // 已经找到三个相同尺度的关键帧可以观测到MapPoint，不需要再找。
                                   if(nObs >= thObs)
                                       break;
                               }
                           }   // 遍历可观测到该MP的关键帧。
                           
                           // 被三个更好尺度的关键帧观测到的点云数量+1。
                           if(nObs>=thObs)
                           {
                               nRedundantObservations++;
                           }
                       }    // 至少三个关键帧观测到。
                   } // !pMP->isBad()。
               } // pMP。
           }    // 步骤3。

           // 步骤4 该局部关键帧90%以上的MapPoints能被其他关键帧（至少3个）观测到，认为冗余
           if(nRedundantObservations>0.9*nMPs)
               pKF->SetBadFlag();
       }    // 遍历共视局部关键帧。

       SetFlagCopyInitKFs(false);
   }



   // 生成反对称矩阵。
   cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
   {
       return (cv::Mat_<float>(3,3) <<  0, -v.at<float>(2), v.at<float>(1),
                v.at<float>(2), 0, -v.at<float>(0),
                -v.at<float>(1), v.at<float>(0), 0);

   }



   // 等待设置重置。
   // 未完成则延时等待，完成退出函数。
   void LocalMapping::RequestReset()
   {
       {
           unique_lock<mutex> lock(mMutexReset);
           mbResetRequested = true;
       }

       while(1)
       {
           {
               unique_lock<mutex> lock2(mMutexReset);
               if(!mbResetRequested)
                   break;
           }

           // 延时3000us
           std::this_thread::sleep_for(std::chrono::milliseconds(3));
       }

   }

   // 设置重置，清空关键帧队列和点云队列。
   void LocalMapping::ResetIfRequested()
   {
       unique_lock<mutex> lock(mMutexReset);
       if(mbResetRequested)
       {
           mlNewKeyFrames.clear();
           mlpRecentAddedMapPoints.clear();
           mbResetRequested=false;
	   
	   mlLocalKeyFrames.clear();
	   
	   // 增加VI初始化
	   mbVINSInited = false;
	   mbFirstTry = true;
       }
   }

   // 重置完成。
   void LocalMapping::RequestFinish()
   {
       unique_lock<mutex> lock(mMutexFinish);
       mbFinishRequested = true;
   }

   // 线程是否完成。
   bool LocalMapping::CheckFinish()
   {
       unique_lock<mutex> lock(mMutexFinish);
       return mbFinishRequested;
   }

   // 设置完成和结束标志位。
   void LocalMapping::SetFinish()
   {
       unique_lock<mutex> lock(mMutexFinish);
       mbFinished = true;
       unique_lock<mutex> lock2(mMutexStop);
       mbStopped = true;
   }

   // 返回线程完成状态。
   bool LocalMapping::isFinished()
   {
       unique_lock<mutex> lock(mMutexFinish);
       return mbFinished;
   }



















}   // namespace ORB_SLAM2

