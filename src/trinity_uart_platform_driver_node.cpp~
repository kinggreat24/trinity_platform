#include "trinity_platform/trinity_uart_platform_driver_node.h"
#include "trinity_platform/tool.h"
#include "trinity_platform/type.h"

#include <boost/shared_ptr.hpp>
#include <boost/smart_ptr.hpp>
#include <iostream>


#define SYNC_SECONDS 10

using namespace trinity_platform;

UART_HANDLE TrinityPlatfromUartDriver::m_uartHd;

boost::mutex TrinityPlatfromUartDriver::m_ttsStatusMutex;

std::vector<char> TrinityPlatfromUartDriver::m_bigbuf;

int TrinityPlatfromUartDriver::m_recvIndex = 0;

int TrinityPlatfromUartDriver::m_packageLength = 0;

ros::Time TrinityPlatfromUartDriver::lastSyncTime_;   

ros::Publisher TrinityPlatfromUartDriver::m_pubTrinityOdom, TrinityPlatfromUartDriver::m_pubMpu6050Pose, TrinityPlatfromUartDriver::m_pubMpu6050Raw;


TrinityPlatfromUartDriver::TrinityPlatfromUartDriver(const ros::NodeHandle &nh)
	:m_nh(nh)
{
	//Subscriber
	m_subTwist     = m_nh.subscribe<geometry_msgs::Twist>("cmd_vel",1,&TrinityPlatfromUartDriver::onTwistCb,this);
	m_subSingleParamSetting = m_nh.subscribe<trinity_platform_msgs::SingleParamSetting>("singleParamSetting",1,&TrinityPlatfromUartDriver::onSingleParamSettingCb,this);
	m_subTts       = m_nh.subscribe<trinity_platform_msgs::TrinityTts>("tts",1,&TrinityPlatfromUartDriver::onTrinityTtsCb,this);
	
	//Publisher  
	m_pubTrinityOdom = m_nh.advertise<trinity_platform_msgs::TrinityOdom>("trinityOdom",1);
	m_pubMpu6050Pose = m_nh.advertise<trinity_platform_msgs::MPU6050PoseData>("mpu6050_pose",1);
	m_pubMpu6050Raw = m_nh.advertise<trinity_platform_msgs::MPU6050RawData>("mpu6050_raw",1);



	//TimerEvent
	m_cmdvel_timer = m_nh.createTimer(ros::Duration(0.2),&TrinityPlatfromUartDriver::onCmdvelTimerEvent,this);
	m_heartbeat_timer = m_nh.createTimer(ros::Duration(1),&TrinityPlatfromUartDriver::onHeartBeatTimerEvent,this);


	//Param init
	m_current_cmdvel = boost::shared_ptr<const geometry_msgs::Twist>(new geometry_msgs::Twist());


	m_ttsStatus = FREE;
	m_bigbuf.clear();
	m_isAutoCharging = false;
}


TrinityPlatfromUartDriver::~TrinityPlatfromUartDriver()
{

}



void TrinityPlatfromUartDriver::onMessageProcess(const char* buf, const int len)
{
	if(!cksCheck(buf,len))
		return;

	//解析数据
	unsigned char command_type = buf[3];
	//ROS_INFO("command_type: %d",command_type);
	switch(command_type)
	{
		case 0x33:
		{
			//ROS_INFO("Trinity Odom data");
			trinity_platform_msgs::TrinityOdom trinityOdom;
			trinityOdom.real_current_value[0]  = static_cast<unsigned char>(buf[6])  | static_cast<unsigned char>(buf[7])  << 8;
			trinityOdom.real_current_value[1]  = static_cast<unsigned char>(buf[8])  | static_cast<unsigned char>(buf[9])  << 8;
			trinityOdom.real_velocity_value[0] = static_cast<unsigned char>(buf[10]) | static_cast<unsigned char>(buf[11]) << 8;
			trinityOdom.real_velocity_value[1] = static_cast<unsigned char>(buf[12]) | static_cast<unsigned char>(buf[13]) << 8;
			trinityOdom.real_position_value[0] = static_cast<unsigned char>(buf[14]) | static_cast<unsigned char>(buf[15]) << 8 | static_cast<unsigned char>(buf[16]) << 16 | static_cast<unsigned char>(buf[17]) << 24; 
			trinityOdom.real_position_value[1] = static_cast<unsigned char>(buf[18]) | static_cast<unsigned char>(buf[19]) << 8 | static_cast<unsigned char>(buf[20]) << 16 | static_cast<unsigned char>(buf[21]) << 24; 
	
			trinityOdom.real_online[0]         = buf[22];
			trinityOdom.real_online[1]         = buf[23];

			trinityOdom.real_ctl1_value[0]     = buf[24];
			trinityOdom.real_ctl1_value[1]     = buf[25];

			trinityOdom.real_ctl2_value[0]     = buf[26];
			trinityOdom.real_ctl2_value[1]     = buf[27];

			m_pubTrinityOdom.publish(trinityOdom);
			break;
		}
		
		case 0x35: //MPU6050 raw data
		{		
			//ROS_INFO("Mpu6050 raw data");	
			trinity_platform_msgs::MPU6050RawData mpu6050_raw;

			mpu6050_raw.ax =  buf[6]  << 8 | buf[7];
			mpu6050_raw.ay =  buf[8]  << 8 | buf[9];
			mpu6050_raw.az =  buf[10] << 8 | buf[11];
			
			mpu6050_raw.gx =  buf[12]  << 8 | buf[13];
			mpu6050_raw.gy =  buf[14]  << 8 | buf[15];
			mpu6050_raw.gz =  buf[16]  << 8 | buf[17];

			mpu6050_raw.hx =  buf[18]  << 8 | buf[19];
			mpu6050_raw.hy =  buf[20]  << 8 | buf[21];
			mpu6050_raw.hz =  buf[22]  << 8 | buf[23];
			
			m_pubMpu6050Raw.publish(mpu6050_raw);
			break;
		}
		case 0x36: //MPU6050 pose data
		{
			//ROS_INFO("Mpu6050 pose data");
			trinity_platform_msgs::MPU6050PoseData mpu6050_pose;
			mpu6050_pose.yaw       = static_cast<unsigned char>(buf[6])   << 8 | static_cast<unsigned char>(buf[7]);
			mpu6050_pose.pitch     = static_cast<unsigned char>(buf[8])   << 8 | static_cast<unsigned char>(buf[9]);
			mpu6050_pose.roll      = static_cast<unsigned char>(buf[10])  << 8 | static_cast<unsigned char>(buf[11]);
			mpu6050_pose.tempr     = static_cast<unsigned char>(buf[12])  << 8 | static_cast<unsigned char>(buf[13]);
			mpu6050_pose.press     = static_cast<unsigned char>(buf[14])  << 8 | static_cast<unsigned char>(buf[15]);
			mpu6050_pose.IMUpersec = static_cast<unsigned char>(buf[16])  << 8 | static_cast<unsigned char>(buf[17]);
			m_pubMpu6050Pose.publish(mpu6050_pose);
			break;
		}
		case 0xFE: //heart beat
		{
			ROS_INFO("Recv heart beat.");
			unsigned int secs = static_cast<unsigned char>(buf[6]) | static_cast<unsigned char>(buf[7]) << 8 | static_cast<unsigned char>(buf[8]) << 16 | static_cast<unsigned char>(buf[9]) << 24; 
			unsigned int nsecs = static_cast<unsigned char>(buf[10]) | static_cast<unsigned char>(buf[11]) << 8 | static_cast<unsigned char>(buf[12]) << 16 | static_cast<unsigned char>(buf[13]) << 24; 

			lastSyncTime_ = ros::Time(secs,nsecs);
			break;
		}

		default:
			break;
	}
}



bool TrinityPlatfromUartDriver::cksCheck(const char* buffer,const int length)
{
	uint16_t crc = bytes2int16(buffer+length-3);
	if(crc == getCks((const uint8_t*)buffer,length-3))
	{	
		return true;
	}
	else
	{	
		return false;
	}
}



void TrinityPlatfromUartDriver::uartRec(const void *msg, unsigned int msglen, void *user_data)
{
	if(0 == m_bigbuf.size())
	{
		for(int i=0;i<msglen;i++)
		{
			//ROS_INFO("m_recvIndex: %d",m_recvIndex);
			if(((unsigned char*)msg)[i] == SYNC_FLAG_START)
			{
				//ROS_INFO("Recv header.");
				m_bigbuf.push_back(((unsigned char*)msg)[i]);
				m_recvIndex += 1;
			}
			else
			{
				if(m_recvIndex > 0)
				{
					//已经接收到数据头，开始将数据添加到容器中
					m_bigbuf.push_back(((unsigned char*)msg)[i]);
					m_recvIndex += 1;

					/*if(5 == m_recvIndex)
					{
						m_packageLength = ((unsigned char*)msg)[i];
					}*/
					if(m_recvIndex >= 6 && !m_packageLength)
					{
						m_packageLength = static_cast<unsigned char>(m_bigbuf[4]) | static_cast<unsigned char>(m_bigbuf[5]) << 8;
						//ROS_INFO("package length: %d",m_packageLength);

						if(m_packageLength > MAX_FRAME_LENGTH)
						{
							m_recvIndex = 0;
							m_packageLength = 0;
							m_bigbuf.clear();
							continue;
						}
					}
					if(m_packageLength+9 == m_recvIndex)
					{
						//读到一帧完整的数据
						onMessageProcess(m_bigbuf.data(),m_bigbuf.size());

						//标识符还原
						m_recvIndex = 0;
						m_packageLength = 0;
						m_bigbuf.clear();
					}
				}
				else
				{
					//没有接收到数据头
					//ROS_INFO("Drop this byte: %x",((unsigned char*)msg)[i]);
				}
			}
		}
	}
	else
	{
		for(int i=0;i<msglen;i++)
		{
			m_bigbuf.push_back(((unsigned char*)msg)[i]);
			m_recvIndex += 1;

			/*if(5 == m_recvIndex)
			{
				m_packageLength = ((unsigned char*)msg)[i];
			}*/
			if(m_recvIndex >=6 && !m_packageLength)
			{
				m_packageLength = static_cast<unsigned char>(m_bigbuf[4]) | static_cast<unsigned char>(m_bigbuf[5]) << 8;
				//ROS_INFO("package length: %d",m_packageLength);

				if(m_packageLength > MAX_FRAME_LENGTH)
				{
					m_recvIndex = 0;
					m_packageLength = 0;
					m_bigbuf.clear();
					continue;
				}
			}
			
			if(m_packageLength+9 == m_recvIndex)
			{
				//读到一帧完整的数据
				onMessageProcess(m_bigbuf.data(),m_bigbuf.size());

				//标识符还原
				m_recvIndex = 0;
				m_packageLength = 0;
				m_bigbuf.clear();
			}
		}
	}
}


//
int TrinityPlatfromUartDriver::trinityPlatformInit(const char *device, int rate, uart_rec_fn uart_rec_cb, void *user_data)
{
	m_portName_ = const_cast<char*>(device);
	m_baudrate_ = rate;
	m_uartRecvCb_ = uart_rec_cb;

	return uart_init(&m_uartHd, m_portName_,m_baudrate_,m_uartRecvCb_, NULL);
}



//更新当前的速度
void TrinityPlatfromUartDriver::onTwistCb(const geometry_msgs::Twist::ConstPtr &msg)
{
	//ROS_INFO("update cmdvel");
	m_current_cmdvel = boost::shared_ptr<const geometry_msgs::Twist>(msg);
}



//将当前的速度以一定的频率发送到运动控制器
void TrinityPlatfromUartDriver::onCmdvelTimerEvent(const ros::TimerEvent &e)
{
	
	//ROS_INFO("Real time control mode.");
	boost::shared_array<char> cmdvelBuffer(new char[17]);

	cmdvelBuffer[0] = SYNC_FLAG_START;
	cmdvelBuffer[1] = MOTION_DEVICE_TYPE;
	cmdvelBuffer[2] = MOTION_DEVICE_ADDRESS;

	cmdvelBuffer[3] = 0x32; //实时速度控制

		
	cmdvelBuffer[4] = 0x08; //长度
	cmdvelBuffer[5] = 0x00; //长度

	//最大负载
	//char* max_pwm = trinity_platform::int322bytes(5000);
	//memcpy(cmdvelBuffer.get()+6,max_pwm,4);

	//线速度
	char* linearSpeed = trinity_platform::int322bytes(static_cast<int>(m_current_cmdvel->linear.x*10000));
	memcpy(cmdvelBuffer.get()+6,linearSpeed,4);

	//角速度
	char* angularSpeed = trinity_platform::int322bytes(static_cast<int>(m_current_cmdvel->angular.z*10000));
	memcpy(cmdvelBuffer.get()+10,angularSpeed,4);

	//CRC检校
	uint16_t crcValue = getCks((uint8_t*)(cmdvelBuffer.get()),14);
	char* crc = trinity_platform::int162bytes(crcValue);
	memcpy(cmdvelBuffer.get()+14,crc,2);

	cmdvelBuffer[16] = SYNC_FLAG_END;

	boost::mutex::scoped_lock lock(m_ttsStatusMutex);
	uart_send(m_uartHd,cmdvelBuffer.get(),17);
//#ifdef HANGFA_UART_DEBUG	
	for(int i=0;i<17;i++)
		ROS_INFO("%2X",cmdvelBuffer[i]);
//#endif//HANGFA_UART_DEBUG
}

void TrinityPlatfromUartDriver::onHeartBeatTimerEvent(const ros::TimerEvent &e)
{
	if((ros::Time::now() - lastSyncTime_).toSec() >  SYNC_SECONDS)
	{
		//time out, restart 
		ROS_ERROR("Lost syncing, closing...");
		uart_uninit(&TrinityPlatfromUartDriver::m_uartHd);
		ros::Duration(3.0).sleep();

		
		ROS_ERROR("Lost syncing, restarting...");
		trinityPlatformInit(m_portName_, m_baudrate_, m_uartRecvCb_, NULL);
	}
}







/*设置单个参数设置*/
void TrinityPlatfromUartDriver::onSingleParamSettingCb(const trinity_platform_msgs::SingleParamSetting::ConstPtr &msg)
{
	/*boost::shared_array<char> paramBuffer(new char[13]);

	//Header
	paramBuffer[0] = SYNC_FLAG_START;
 	paramBuffer[1] = MOTION_DEVICE_TYPE;
	paramBuffer[2] = MOTION_DEVICE_ADDRESS;

	//Command
	paramBuffer[3] = 0x1E;

	//Length
	paramBuffer[4] = 0x05;

	//Data
	paramBuffer[5] = msg->index;
	char* paramData = hangfa_platform::int322bytes(msg->data);
	memcpy(paramBuffer.get()+6,paramData,4);

	//CRC check
	uint16_t crcValue = getCks((uint8_t*)(paramBuffer.get()),10);
	char* crc = hangfa_platform::int162bytes(crcValue);
	memcpy(paramBuffer.get()+10,crc,2);

	paramBuffer[12] = SYNC_FLAG_END;

	boost::mutex::scoped_lock lock(m_ttsStatusMutex);
	uart_send(m_uartHd,paramBuffer.get(),13);*/
}

void TrinityPlatfromUartDriver::onTrinityTtsCb(const trinity_platform_msgs::TrinityTts::ConstPtr &msg)
{
	//判断是否是语音合成消息
	if(trinity_platform_msgs::TrinityTts::COMMAND_PLAY != msg->command_type)
	{
		//其它命令消息
		/*int length = 4;
		boost::shared_array<char>buffer = boost::shared_array<char>(new char[length]);
		//帧头(1-byte)
		buffer[0] = SYNC_HEAD;

		//数据区长度(2-bytes),大端字节序
		buffer[1] = 0x00;
		buffer[2] = 0x01;

		//命令字
		buffer[3] = msg->command_type;

		uart_send(m_uartHd,buffer.get(),4);*/
	}
	else
	{
		//将UTF-8编码的转化为gbk编码
		char gbkbuf[1024*4];
		char tmpbuf[1024*4];
		memcpy(tmpbuf,msg->text.c_str(),msg->text.length());
        	CodingConversion::u2g(tmpbuf, msg->text.length(), gbkbuf, sizeof(gbkbuf));
		
		//判断芯片的状态
		boost::mutex::scoped_lock lock(m_ttsStatusMutex);
		if(m_ttsStatus == FREE)         //芯片空闲
		{
			int length = strlen(gbkbuf)/*msg->text.length()*/ + 10;
			ROS_INFO("text length: %d",(int)msg->text.length());
			boost::shared_array<char>buffer = boost::shared_array<char>(new char[length]);
			//帧头(1-byte)
			buffer[0] = SYNC_FLAG_START;
 			buffer[1] = MOTION_DEVICE_TYPE;
			buffer[2] = MOTION_DEVICE_ADDRESS;

			buffer[3] = 0x24;   //TTS Play
			//数据区长度(2-bytes)
			char* lengthBytes = trinity_platform::int162bytes(strlen(gbkbuf) + 1);
			buffer[4] = lengthBytes[0];
			buffer[5] = lengthBytes[1];

			//命令参数
			buffer[6] = msg->param_type;
		
			//待发送文本
			memcpy(buffer.get()+7,gbkbuf,strlen(gbkbuf));//待发送文本

			//CRC检校
			uint16_t crcValue = getCks((uint8_t*)(buffer.get()),strlen(gbkbuf) + 7);
			char* crc = trinity_platform::int162bytes(crcValue);
			memcpy(buffer.get()+7+strlen(gbkbuf),crc,2);
			
			buffer[length-1] = SYNC_FLAG_END;

			/*for(int i=0;i<length;i++)
				ROS_INFO("%2X",buffer[i]);*/

			uart_send(m_uartHd,buffer.get(),length);
			
		}
		else if(m_ttsStatus == STANDBY)
		{
			/*ROS_WARN("Unique tts is sleeping now, wakeup first!");
			//唤醒芯片
			boost::shared_array<char>wakeupBuffer = boost::shared_array<char>(new char[4]);
			
			wakeupBuffer[0] = SYNC_HEAD;//帧头(1-byte)
			wakeupBuffer[1] = 0x00;//数据区长度(2-bytes),大端字节序
			wakeupBuffer[2] = 0x01;
			wakeupBuffer[3] = unique_tts::TrinityTts::COMMAND_WAKEUP;//命令字(唤醒)
			uart_send(m_uartHd,wakeupBuffer.get(),4);

			ros::Duration(0.001).sleep();
			//检查芯片是否唤醒
			

			//发送语音
			int length = strlen(gbkbuf) + 5;
			boost::shared_array<char>buffer = boost::shared_array<char>(new char[length]);
			buffer[0] = SYNC_HEAD;//帧头(1-byte)
			char* lengthBytes = int162bytes(strlen(gbkbuf) + 2,false);//数据区长度(2-bytes)
			buffer[1] = lengthBytes[0];
			buffer[2] = lengthBytes[1];
			buffer[3] = msg->command_type;//命令字
			buffer[4] = msg->param_type;//命令参数
			memcpy(buffer.get()+5,gbkbuf,strlen(gbkbuf));//待发送文本
			uart_send(m_uartHd,buffer.get(),length);*/			
		}
		else//芯片处于忙碌状态
		{
			ROS_WARN("Unique tts is busy!");
			//发送语音
			/*int length = strlen(gbkbuf) + 5;
			boost::shared_array<char>buffer = boost::shared_array<char>(new char[length]);
			buffer[0] = SYNC_HEAD;//帧头(1-byte)
			char* lengthBytes = int162bytes(strlen(gbkbuf) + 2,false);//数据区长度(2-bytes)
			buffer[1] = lengthBytes[0];
			buffer[2] = lengthBytes[1];
			buffer[3] = msg->command_type;//命令字
			buffer[4] = msg->param_type;//命令参数
			memcpy(buffer.get()+5,gbkbuf,strlen(gbkbuf));//待发送文本
			uart_send(m_uartHd,buffer.get(),length);*/	
		}
		
	}
}


