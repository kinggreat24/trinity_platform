#ifndef TRINITY_UART_PLATFORM_DRIVER_H
#define TRINITY_UART_PLATFORM_DRIVER_H

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "trinity_platform/type.h"
#include "trinity_platform/uart.h"
#include "trinity_platform/coding_conversion.h"


#include <vector>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

//ROS
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/Range.h>

//ROS Custom

//公共命令相关
#include "trinity_platform_msgs/SingleParamSetting.h"

#include "trinity_platform_msgs/TrinityOdom.h"
#include "trinity_platform_msgs/MPU6050PoseData.h"
#include "trinity_platform_msgs/MPU6050RawData.h"
#include "trinity_platform_msgs/TrinityTts.h"

namespace trinity_platform{

class TrinityPlatfromUartDriver
{
public:
	TrinityPlatfromUartDriver(const ros::NodeHandle &nh);
	~TrinityPlatfromUartDriver();

	static void onMessageProcess(const char* buf, const int len);
	static bool cksCheck(const char* inbuffer,const int length);
	static void uartRec(const void *msg, unsigned int msglen, void *user_data);
	

	int trinityPlatformInit(const char *device, int rate, uart_rec_fn uart_rec_cb, void *user_data);

protected:
	void onTwistCb(const geometry_msgs::Twist::ConstPtr &msg);
	void onCmdvelTimerEvent(const ros::TimerEvent &e);
        void onHeartBeatTimerEvent(const ros::TimerEvent &e);
	void onSingleParamSettingCb(const trinity_platform_msgs::SingleParamSetting::ConstPtr &msg);
	void onTrinityTtsCb(const trinity_platform_msgs::TrinityTts::ConstPtr &msg);


public:
	static UART_HANDLE m_uartHd;

private:
	ros::NodeHandle m_nh;
	ros::Subscriber m_subTwist,m_subCommand,m_subSingleParamSetting,m_subTts;

	static ros::Publisher m_pubTrinityOdom, m_pubMpu6050Pose, m_pubMpu6050Raw;

	static std::vector<char> m_bigbuf;        //存放接收到的数据的容器
	static int m_recvIndex;                   //接收到的数据的个数
	static int m_packageLength;

	bool m_isAutoCharging;
	int m_controllerType;
	int m_maxSpeed;
	static boost::mutex m_ttsStatusMutex;
	boost::shared_ptr<const geometry_msgs::Twist> m_current_cmdvel;
	ros::Timer m_cmdvel_timer;
	ros::Timer m_heartbeat_timer;

	TTS_STATUS m_ttsStatus;
	
	static ros::Time lastSyncTime_;             //上次接收到的时间
	uart_rec_fn m_uartRecvCb_;                  //串口接收回调函数

	char* m_portName_;
	int m_baudrate_;
};

}
#endif//TRINITY_UART_PLATFORM_DRIVER_H
