#ifndef MSGSYNCHRONIZER_H
#define MSGSYNCHRONIZER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <mutex>

using namespace std;

namespace ORBVIO
{

    class MsgSynchronizer
    {
    public:

        enum Status
        {
            NOTINIT = 0,
            INIT,
            NORMAL
        };

        MsgSynchronizer(const double &imagedelay = 0.);

        ~MsgSynchronizer();

        // 添加消息的回调函数
        void addImageMsg(const sensor_msgs::ImageConstPtr &imgmsg);

        void addImuMsg(const sensor_msgs::ImuConstPtr &imumsg);

        // 主函数循环所有消息的句柄函数
        bool getRecentMsgs(sensor_msgs::ImageConstPtr &imgmsg, std::vector<sensor_msgs::ImuConstPtr> &vimumsgs);

        void clearMsgs(void);

        void imageCallback(const sensor_msgs::ImageConstPtr &msg);

        void imuCallback(const sensor_msgs::ImuConstPtr &msg);

        inline Status getStatus(void)
        {
            return _status;
        }

        double getImageDelaySec(void) const
        {
            return _imageMsgDelaySec;
        }

    private:

        // 图像对IMU的延时(s)
        double _imageMsgDelaySec;
        std::mutex _mutexImageQueue;
        std::queue<sensor_msgs::ImageConstPtr> _imageMsgQueue;
        std::mutex _mutexIMUQueue;
        std::queue<sensor_msgs::ImuConstPtr> _imuMsgQueue;
        ros::Time _imuMsgTimeStart;
        Status _status;
        int _dataUnsyncCnt;

    };


}


#endif

