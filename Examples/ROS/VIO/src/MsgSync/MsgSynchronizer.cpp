#include "MsgSynchronizer.h"
#include "../../../src/IMU/configparam.h"

namespace ORBVIO
{

    MsgSynchronizer::MsgSynchronizer(const double &imagedelay):
        _imageMsgDelaySec(imagedelay), _status(NOTINIT),
		_dataUnsyncCnt(0)
    {
        printf("image delay set as %.1fms\n", _imageMsgDelaySec*1000);
    }

    MsgSynchronizer::~MsgSynchronizer()
    {

    }

    bool MsgSynchronizer::getRecentMsgs(sensor_msgs::ImageConstPtr &imgmsg, std::vector<sensor_msgs::ImuConstPtr> &vimumsgs)
    {

        unique_lock<mutex> lock1(_mutexImageQueue);

        if(_status == NOTINIT || _status == INIT)
        {
            return false;
        }

        if(_imageMsgQueue.empty())
        {
            return false;
        }

        if(_imuMsgQueue.empty())
        {
            return false;
        }

        {
            sensor_msgs::ImageConstPtr imsg;
            sensor_msgs::ImuConstPtr bmsg;

            imsg = _imageMsgQueue.back();
            bmsg = _imuMsgQueue.front();

            // 检查时间差，最大不超过3s
            if(imsg->header.stamp.toSec() - _imageMsgDelaySec + 3.0 < bmsg->header.stamp.toSec())
            {
                ROS_ERROR("Data dis-continuity, > 3 seconds. Buffer cleared");
                clearMsgs();
                return false;
            }

            imsg = _imageMsgQueue.front();
            bmsg = _imuMsgQueue.back();

            // 等待imu数据时间间隔
            if(imsg->header.stamp.toSec() - _imageMsgDelaySec > bmsg->header.stamp.toSec())
            {
                return false;
            }

            if(imsg->header.stamp.toSec() - _imageMsgDelaySec > bmsg->header.stamp.toSec() +3.0)
            {
                ROS_ERROR("Data dis-continuity, > 3 seconds. Buffer cleared");
                clearMsgs();
                return false;
            }

            // 等待imu数据包
            if(_imageMsgQueue.size()<10 && _imuMsgQueue.size()<15
                && imsg->header.stamp.toSec() - _imageMsgDelaySec > bmsg->header.stamp.toSec() )
            {
                return false;
            }
        }

        // 取一帧图像
        imgmsg = _imageMsgQueue.front();
        _imageMsgQueue.pop();

        // 清空imu数据, push所有早于当前时间戳的imu数据
        vimumsgs.clear();
        while(true)
        {
            if(_imuMsgQueue.empty())
                break;

            sensor_msgs::ImuConstPtr tmpimumsg = _imuMsgQueue.front();
            if(tmpimumsg->header.stamp.toSec() < imgmsg->header.stamp.toSec() - _imageMsgDelaySec)
            {
                vimumsgs.push_back(tmpimumsg);

                {
                    unique_lock<mutex> lock(_mutexIMUQueue);
                    _imuMsgQueue.pop();
                }

                _dataUnsyncCnt = 0;
            }

            else
            {
                if(_dataUnsyncCnt++ > 10)
                {
                    _dataUnsyncCnt = 0;
                    clearMsgs();
                    ROS_ERROR("data unsynced many times, reset sync");
                    return false;
                }

                break;
            }
        }

        // 相机20fps, imu为100hz, 两帧之间不应该超过5个imu数据
        if(vimumsgs.size() > 10)
            ROS_WARN("%lu imu messages between images, note", vimumsgs.size());
        if(vimumsgs.size() == 0)
            ROS_ERROR("no imu message between images!");

        return true;
    }



    void MsgSynchronizer::addImuMsg(const sensor_msgs::ImuConstPtr &imumsg)
    {

        unique_lock<mutex> lock(_mutexIMUQueue);
        if(_imageMsgDelaySec >= 0)
        {
            _imuMsgQueue.push(imumsg);

            if(_status == NOTINIT)
            {
                _imuMsgTimeStart = imumsg->header.stamp;
                _status = INIT;
            }
        }

        // 没有图像，只添加imu信息
        else
        {
            if(_status == NOTINIT)
                return;
            else if(_status == INIT)
            {
                if(imumsg->header.stamp.toSec() + _imageMsgDelaySec > _imuMsgTimeStart.toSec())
                {
                    _imuMsgQueue.push(imumsg);
                    _status = NORMAL;
                }
            }
            else
            {
                _imuMsgQueue.push(imumsg);
            }
        }

    }


    void MsgSynchronizer::addImageMsg(const sensor_msgs::ImageConstPtr &imgmsg)
    {
        unique_lock<mutex> lock(_mutexImageQueue);

        if(_imageMsgDelaySec >= 0)
        {
            // 如果没有imu信息，不添加image
            if(_status == NOTINIT)
                return;

            else if(_status == INIT)
            {
                if(imgmsg->header.stamp.toSec() - _imageMsgDelaySec > _imuMsgTimeStart.toSec())
                {
                    _imageMsgQueue.push(imgmsg);
                    _status = NORMAL;
                }
            }

            else
            {
                    _imageMsgQueue.push(imgmsg);
            }
        }
        else
        {

            if(_status == NOTINIT)
            {
                _imuMsgTimeStart = imgmsg->header.stamp;
                _status = INIT;
            }
            else
            {
                _imageMsgQueue.push(imgmsg);
            }
        }

        if(ORB_SLAM2::ConfigParam::GetRealTimeFlag())
        {
            if(_imageMsgQueue.size() > 2)
                _imageMsgQueue.pop();
        }

    }



    void MsgSynchronizer::imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        addImageMsg(msg);
    }



    void MsgSynchronizer::imuCallback(const sensor_msgs::ImuConstPtr &msg)
    {
        addImuMsg(msg);
    }


    void MsgSynchronizer::clearMsgs(void)
    {
        _imuMsgQueue = std::queue<sensor_msgs::ImuConstPtr>();
        _imageMsgQueue  = std::queue<sensor_msgs::ImageConstPtr>();
    }

}

