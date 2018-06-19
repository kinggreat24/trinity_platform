#include "trinity_platform/tool.h"
using namespace std;
namespace trinity_platform
{
//============================================================================
// 名称: getCks
// 功能: 计算校验和
// 参数: 
//*ptr:要计算校验和的字符串
//len:数据字节数
// 返回: 
// 说明: 无
//============================================================================
uint16_t getCks(const uint8_t* ptr,uint8_t len)
{
	uint16_t crc = 0;
	uint8_t da = 0;
	while ( len-- != 0 )
	{
		da = (uint8_t)(crc >> 8);
		crc <<= 8;
		crc ^= crc_ta[da ^ *ptr];
		ptr++;
	}
	
	return ( crc );
}


uint16_t bytes2int16(const char* bytes,bool littleOrder)
{
	uint16_t result = 0;
	if(littleOrder)
		result = (uint8_t)bytes[0] | (uint8_t)bytes[1] << 8;
	else
		result = (uint8_t)bytes[0] <<8  | (uint8_t)bytes[1];
	return result;
}

uint32_t bytes2int32(const char* bytes,bool littleOrder)
{
	uint32_t result = 0;
	if(littleOrder)
		result = (uint8_t)bytes[0] | (uint8_t)bytes[1]<<8 | (uint8_t)bytes[2]<<16 | (uint8_t)bytes[3]<<24;
	else
		result = (uint8_t)bytes[0]<<24 | (uint8_t)bytes[1]<<16 | (uint8_t)bytes[2]<<8 | (uint8_t)bytes[3];
	return result;
}

float bytes2float32(const char* bytes,bool littleOrder)
{
	float result = 0;
	if(littleOrder)
		memcpy(&result,bytes,4);
	else
	{
		char tempBytes[4];
		tempBytes[0] = bytes[3];
		tempBytes[1] = bytes[2];
		tempBytes[2] = bytes[1];
		tempBytes[3] = bytes[0];
		memcpy(&result,tempBytes,4);
	}
	return result;
}

char* int162bytes(short length,bool littleOrder)
{
	char* bytes = new char[2];
	if(littleOrder)
	{
		bytes[1] = (length >> 8) & 0xff;
		bytes[0] = length & 0xff;
	}
	else
	{
		bytes[0] = (length >> 8) & 0xff;
		bytes[1] = length & 0xff;
	}
	return bytes;
}

char* int322bytes(int length,bool littleOrder)
{
	char* bytes = new char[4];
	if(littleOrder)
	{
		bytes[3] = (length >> 24) & 0xff;
		bytes[2] = (length >> 16) & 0xff;
		bytes[1] = (length >> 8)  & 0xff;
		bytes[0] = length & 0xff;
	}
	else
	{
		bytes[0] = (length >> 24) & 0xff;
		bytes[1] = (length >> 16) & 0xff;
		bytes[2] = (length >> 8)  & 0xff;
		bytes[3] = length & 0xff;
	}
	return bytes;
}
}
