#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"
#include "MsgSync/MsgSynchronizer.h"
#include "../../../src/IMU/imudata.h"
#include "../../../src/IMU/configparam.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>


using namespace std;

class ImageGrabber
{
public:

    ImageGrabber(ORB_SLAM2::System *pSLAM):mpSLAM(pSLAM)
    {

    }

    void GrabImage(const sensor_msgs::ImageConstPtr &msg);

    ORB_SLAM2::System *mpSLAM;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 VIO path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    ORB_SLAM2::ConfigParam config(argv[2]);


    // 同步获取cam+imu
    double imageMsgDelaySec = config.GetImageDelayToIMU();
    ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);
    
    ros::NodeHandle nh;
    ros::Subscriber imagesub;
    ros::Subscriber imusub;
    if(ORB_SLAM2::ConfigParam::GetRealTimeFlag())
    {
	imagesub = nh.subscribe(config._imageTopic, 2, &ORBVIO::MsgSynchronizer::imageCallback, &msgsync);
	imusub = nh.subscribe(config._imuTopic, 200,  &ORBVIO::MsgSynchronizer::imuCallback, &msgsync);
    }
    
    sensor_msgs::ImageConstPtr imageMsg;
    std::vector<sensor_msgs::ImuConstPtr> vimuMsg;
    
    const double g3dm = 9.80665;
    const bool bAccMultiply98 = config.GetAccMultiply9p8();
    
    ros::Rate r(1000);
    
    // 不是实时运行
    if(!ORB_SLAM2::ConfigParam::GetRealTimeFlag())
    {
	ROS_WARN("Run not-realtime");
	
	std::string bagfile = config._bagfile ;
	rosbag::Bag bag;
	bag.open(bagfile, rosbag::bagmode::Read);
	
	std::vector<std::string> topics;
	std::string imutopic = config._imuTopic;
	std::string imagetopic = config._imageTopic;
	topics.push_back(imagetopic);
	topics.push_back(imutopic);
	
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
	    // 读取imu
	    sensor_msgs::ImuConstPtr simu = m.instantiate<sensor_msgs::Imu>();
	    if(simu!=NULL)
		msgsync.imuCallback(simu);
	    
	    // 读取image
	    sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();
	    if(simage!=NULL)
		msgsync.imageCallback(simage);
	    
	    bool bdata = msgsync.getRecentMsgs(imageMsg, vimuMsg);
	    
	    if(bdata)
	    {
		// 读取imu
		std::vector<ORB_SLAM2::IMUData> vimuData;
		for(unsigned int i=0; i<vimuMsg.size(); i++)
		{
		    sensor_msgs::ImuConstPtr imuMsg = vimuMsg[i];
		    double ax = imuMsg->linear_acceleration.x;
		    double ay = imuMsg->linear_acceleration.y;
		    double az = imuMsg->linear_acceleration.z;
		    if(bAccMultiply98)
		    {
			ax *= g3dm;
			ay *= g3dm;
			az *= g3dm;
		    }
		    
		    ORB_SLAM2::IMUData imudata(imuMsg->angular_velocity.x,imuMsg->angular_velocity.y,imuMsg->angular_velocity.z,
				    ax, ay, az, imuMsg->header.stamp.toSec() );
		    vimuData.push_back(imudata);
		}
		
		// 读取image
		cv_bridge::CvImageConstPtr cv_ptr;
		try
		{
		    cv_ptr = cv_bridge::toCvShare(imageMsg);
		}
		catch(cv_bridge::Exception& e)
		{
		    ROS_ERROR("cv_bridge exception: %s", e.what());
		    return -1;
		}
		
		cv::Mat im = cv_ptr->image.clone();
		{
		    static double startT = -1;
		    if(startT < 0)
			startT = imageMsg->header.stamp.toSec();
		    
		    if(imageMsg->header.stamp.toSec() < startT+config._testDiscardTime)
			im = cv::Mat::zeros(im.rows, im.cols, im.type());
		}
		
		SLAM.TrackMonoVI(im, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
		
		bool bstop = false;
		while(!SLAM.bLocalMapAcceptKF())
		{
		    if(!ros::ok())
		    {
			bstop = true;
		    }
		};
		
		if(bstop)
		    break;
		
	    }
	    
	    ros::spinOnce();
	    r.sleep();
	    if(!ros::ok())
		break;
	    
	}
	
	
    }
    
    // 实时运行
    else
    {
	ROS_WARN("Run realtime");
	
	while(ros::ok())
	{
	    bool bdata = msgsync.getRecentMsgs(imageMsg, vimuMsg);
	    
	    if(bdata)
	    {
		// 读取imu
		std::vector<ORB_SLAM2::IMUData> vimuData;
		for(unsigned int i=0;i<vimuMsg.size();i++)
		{
		    sensor_msgs::ImuConstPtr imuMsg = vimuMsg[i];
		    double ax = imuMsg->linear_acceleration.x;
		    double ay = imuMsg->linear_acceleration.y;
		    double az = imuMsg->linear_acceleration.z; 
		    
		    if(bAccMultiply98)
		    {
			ax *= g3dm;
			ay *= g3dm;
			az *= g3dm;
		    }
		    ORB_SLAM2::IMUData imudata(imuMsg->angular_velocity.x,imuMsg->angular_velocity.y,imuMsg->angular_velocity.z,
					       ax,ay,az,imuMsg->header.stamp.toSec());
		    
		    vimuData.push_back(imudata);   
		}
		
		// 读取图像
		cv_bridge::CvImageConstPtr cv_ptr;
		try
		{
		    cv_ptr = cv_bridge::toCvShare(imageMsg);
		    
		}
		catch (cv_bridge::Exception& e)
		{
		    ROS_ERROR("cv_bridge exception: %s", e.what());
		    return -1;
		    
		}
		
		cv::Mat im = cv_ptr->image.clone();
		{
		    static double startT = -1;
		    if(startT < 0)
			startT = imageMsg->header.stamp.toSec();
		    if(imageMsg->header.stamp.toSec() < startT+config._testDiscardTime)
			im = cv::Mat::zeros(im.rows,im.cols,im.type());
		    
		}
		SLAM.TrackMonoVI(im, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
		
	    }
	    
	    ros::spinOnce();
	    r.sleep();
	    if(!ros::ok())
		break;
	}
		
    }
    
    SLAM.SaveKeyFrameTrajectoryNavState(config._tmpFilePath+"KeyFrameNavStateTrajectory.txt");
    
    cout << endl << endl << "press any key to shutdown" << endl;
    getchar();
    
    SLAM.Shutdown();
    
    ros::shutdown();
    
    return 0;

}





