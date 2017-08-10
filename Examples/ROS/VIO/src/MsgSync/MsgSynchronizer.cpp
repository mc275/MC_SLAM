#include "MsgSynchronizer.h"
#include "../../../src/IMU/configparam.h"

namespace ORBVIO
{

    MsgSynchronizer::MsgSynchronizer(const double &imagedelay) :
            _imageMsgDelaySec(imagedelay), _status(NOTINIT),
            _dataUnsyncCnt(0)
    {
        printf("image delay set as %.1fms\n", _imageMsgDelaySec * 1000);
    }

    MsgSynchronizer::~MsgSynchronizer()
    {

    }

    // 获取订阅的数据
    bool
    MsgSynchronizer::getRecentMsgs(sensor_msgs::ImageConstPtr &imgmsg, std::vector<sensor_msgs::ImuConstPtr> &vimumsgs)
    {

        unique_lock<mutex> lock1(_mutexImageQueue);

        if (_status == NOTINIT || _status == INIT)
        {
            return false;
        }

        if (_imageMsgQueue.empty())
        {
            return false;
        }

        if (_imuMsgQueue.empty())
        {
            return false;
        }

        // 判断是否有异常情况
        {
            sensor_msgs::ImageConstPtr imsg;
            sensor_msgs::ImuConstPtr bmsg;

            // front是当前队列中最先进队列的元素，相当于pop操作， back是最后一个进队列的参数
            // 第二帧图像和最开始的imu数据
            imsg = _imageMsgQueue.back();
            bmsg = _imuMsgQueue.front();

            // 检查时间差，最新读取的图像和IMU数据时间间隔超过3s，清空
            if (imsg->header.stamp.toSec() - _imageMsgDelaySec + 3.0 < bmsg->header.stamp.toSec())
            {
                ROS_ERROR("Data dis-continuity, > 3 seconds. Buffer cleared");
                clearMsgs();
                return false;
            }

            // 第一帧图像和最后的imu数据
            imsg = _imageMsgQueue.front();
            bmsg = _imuMsgQueue.back();

            // 
            if (imsg->header.stamp.toSec() - _imageMsgDelaySec > bmsg->header.stamp.toSec())
            {
                return false;
            }

            if (imsg->header.stamp.toSec() - _imageMsgDelaySec > bmsg->header.stamp.toSec() + 3.0)
            {
                ROS_ERROR("Data dis-continuity, > 3 seconds. Buffer cleared");
                clearMsgs();
                return false;
            }

            // 图像少于10帧， imu数据少于15个
            if (_imageMsgQueue.size() < 10 && _imuMsgQueue.size() < 15
                && imsg->header.stamp.toSec() - _imageMsgDelaySec > bmsg->header.stamp.toSec())
            {
                return false;
            }
        }

        // 读取队列中最早的图像，并从队列中清除
        imgmsg = _imageMsgQueue.front();
        _imageMsgQueue.pop();

        // 清空imu数据, 读取所有早于当前图像时间戳的imu数据
        vimumsgs.clear();
        while (true)
        {
            if (_imuMsgQueue.empty())
                break;

            // 读取队列中最早的imu数据
            sensor_msgs::ImuConstPtr tmpimumsg = _imuMsgQueue.front();
            // imu数据时间戳<图像时间戳，读取到vimumsgs中，并从队列中清除
            if (tmpimumsg->header.stamp.toSec() < imgmsg->header.stamp.toSec() - _imageMsgDelaySec)
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
                if (_dataUnsyncCnt++ > 10)
                {
                    _dataUnsyncCnt = 0;
                    clearMsgs();
                    ROS_ERROR("data unsynced many times, reset sync");
                    return false;
                }

                break;
            }
        }

        // 相机20fps, imu为200hz, 两帧之间不应该超过10个imu数据
        if (vimumsgs.size() > 10)
            ROS_WARN("%lu imu messages between images, note", vimumsgs.size());
        if (vimumsgs.size() == 0)
            ROS_ERROR("no imu message between images!");

        return true;
    }


    void MsgSynchronizer::addImuMsg(const sensor_msgs::ImuConstPtr &imumsg)
    {

        unique_lock<mutex> lock(_mutexIMUQueue);
        // 图像和IMU同步数据的延时关系不同，IMU与图像读取顺序不同
        // 图像滞后于IMU，先读IMU；否则先读取图像
        if (_imageMsgDelaySec >= 0)
        {
            _imuMsgQueue.push(imumsg);

            if (_status == NOTINIT)
            {
                _imuMsgTimeStart = imumsg->header.stamp;
                _status = INIT;
            }
        }
        else
        {
            if (_status == NOTINIT)
                return;
            else if (_status == INIT)
            {
                if (imumsg->header.stamp.toSec() + _imageMsgDelaySec > _imuMsgTimeStart.toSec())
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

        if (_imageMsgDelaySec >= 0)
        {
            if (_status == NOTINIT)
                return;

            else if (_status == INIT)
            {
                if (imgmsg->header.stamp.toSec() - _imageMsgDelaySec > _imuMsgTimeStart.toSec())
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

            if (_status == NOTINIT)
            {
                _imuMsgTimeStart = imgmsg->header.stamp;
                _status = INIT;
            }
            else
            {
                _imageMsgQueue.push(imgmsg);
            }
        }

        // 实时情况下，图像数据的队列只有两帧。
        if (ORB_SLAM2::ConfigParam::GetRealTimeFlag())
        {
            if (_imageMsgQueue.size() > 2)
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
        _imageMsgQueue = std::queue<sensor_msgs::ImageConstPtr>();
    }

}

