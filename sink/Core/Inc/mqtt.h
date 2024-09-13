#ifndef __LWMP_H_
#define __LWMP_H_

#include "main.h"
#include "string.h"
#include "usart.h"
#define BYTE0(dwTemp)       (*( char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


#define _DEBUG_MQTT 1

typedef enum
{
	//���� 	    ֵ 			������������ 	����
	M_RESERVED1	=0	,	//	��ֹ	����
	M_CONNECT		,	//	�ͻ��˵������	�ͻ����������ӷ����
	M_CONNACK		,	//	����˵��ͻ���	���ӱ���ȷ��
	M_PUBLISH		,	//	������������	������Ϣ
	M_PUBACK		,	//	������������	QoS 1��Ϣ�����յ�ȷ��
	M_PUBREC		,	//	������������	�����յ�����֤������һ����
	M_PUBREL		,	//	������������	�����ͷţ���֤�����ڶ�����
	M_PUBCOMP		,	//	������������	QoS 2��Ϣ������ɣ���֤������������
	M_SUBSCRIBE		,	//	�ͻ��˵������	�ͻ��˶�������
	M_SUBACK		,	//	����˵��ͻ���	����������ȷ��
	M_UNSUBSCRIBE	,	//	�ͻ��˵������	�ͻ���ȡ����������
	M_UNSUBACK		,	//	����˵��ͻ���	ȡ�����ı���ȷ��
	M_PINGREQ		,	//	�ͻ��˵������	��������
	M_PINGRESP		,	//	����˵��ͻ���	������Ӧ
	M_DISCONNECT	,	//	�ͻ��˵������	�ͻ��˶Ͽ�����
	M_RESERVED2		,	//	��ֹ	����
}_typdef_mqtt_message;



extern const uint8_t parket_connetAck[];
extern const uint8_t parket_subAck[];
	
unsigned int MQTT_Connect(char *Username,char *Password,char *ClientID,unsigned char *mqttPacket);
unsigned int MQTT_SubscribeTopic(char *topic,unsigned char qos,unsigned char whether,unsigned char *mqttPacket);
unsigned int MQTT_PublishData(char *topic, char *message, unsigned char qos,unsigned char *mqttPacket);


#endif
