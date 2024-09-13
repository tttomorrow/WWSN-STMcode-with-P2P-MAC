#include "esp8266.h"
#include "usart.h"
#include "string.h" 
#include "oled.h"
#include <stdio.h>
#include "mqtt.h"
uint8_t dtbuf[200];   								//��ӡ������	
extern unsigned char servernotok;
unsigned int mqttPacketLen = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//ESP8266���������,�����յ���Ӧ��
//str:�ڴ���Ӧ����
//����ֵ:0,û�еõ��ڴ���Ӧ����
//    ����,�ڴ�Ӧ������λ��(str��λ��)
uint8_t* ESP8266_check_cmd(uint8_t *str)
{
	char *strx=0;
	if(USART_RX_STA == REC_OK)		//���յ�һ��������
	{ 
		strx=strstr((const char*)USART_RX_BUF,(const char*)str);
		memset(USART_RX_BUF,0,USART_REC_LEN);
	} 
	return (uint8_t*)strx;
}


//��ESP8266��������
//cmd:���͵������ַ���(����Ҫ��ӻس���),��cmd<0XFF��ʱ��,��������(���緢��0X1A),���ڵ�ʱ�����ַ���.
//ack:�ڴ���Ӧ����,���Ϊ��,���ʾ����Ҫ�ȴ�Ӧ��
//waittime:�ȴ�ʱ��(��λ:10ms)
//����ֵ:0,���ͳɹ�(�õ����ڴ���Ӧ����)
//       1,����ʧ��
uint8_t ESP8266_send_cmd(uint8_t *cmd,uint8_t *ack,uint16_t waittime)
{
	uint8_t res=0; 
	//���֮ǰ�յ�������
	memset(USART_RX_BUF,0,USART_REC_LEN);
	USART_RX_STA=0;
	
	printf("%s\r\n",cmd);
	USART2_printf("AT-->%s\r\n",cmd);  //���Դ�ӡ�ӿڣ�����Ҫ���ڴ�ӡʱ��ȡ��
	if(ack&&waittime)		//��Ҫ�ȴ�Ӧ��
	{
		while(--waittime)
		{
			HAL_Delay(10);
			if(USART_RX_STA == REC_OK)//���յ��ڴ���Ӧ����
			{
				USART2_printf("Recevie-->%s\r\n",(uint8_t *)USART_RX_BUF);	//���͵�����
				if(ESP8266_check_cmd(ack))break;//�õ���Ч���� 
				USART_RX_STA=0;
			} 
		}
		if(waittime==0)res=1; 
	}
	USART_RX_STA=0;
	return res;
} 

//����wifi�͵�Ƭ�����Ӻ�����WiFiģ�����ӵ�·����
uint8_t ESP8266_work_test(uint8_t *SSID,uint8_t *Password)
{
 
	OLED_ShowString(0,2,"               ");
	OLED_ShowString(0,2,"WIFI CONNECT ..");
	//����AT����ʹ��STM32���ӵ� WiFiģ��
	if(ESP8266_send_cmd((uint8_t *)"ATE0",(uint8_t *)"OK",200))
	{
		if(ESP8266_send_cmd((uint8_t *)"ATE0",(uint8_t *)"OK",200)) return WIFI_COMMUNTION_ERR;	//ͨ�Ų���
	}		
	
	OLED_ShowString(0,2,"               ");
	OLED_ShowString(0,2,"WIFI SETMODE ..");
	//����WiFiģ��ΪAP+STAģʽ
	ESP8266_send_cmd((uint8_t *)"AT+CWMODE=3",(uint8_t *)"OK",200);	//����WIFIģʽ
	
	OLED_ShowString(0,2,"               ");
	OLED_ShowString(0,2,"WIFI SETSSID ..");
	
	//����WiFiģ�����ӵ�·����
	memset(dtbuf,0,200);
	OLED_ShowString(0,4,"               ");
	OLED_ShowString(0,4,SSID);
	OLED_ShowString(0,6,"               ");
	OLED_ShowString(0,6,Password);
	sprintf((char *)dtbuf,"AT+CWJAP=\"%s\",\"%s\"",SSID,Password);
	if(ESP8266_send_cmd((uint8_t *)dtbuf,(uint8_t *)"WIFI GOT IP",1000))
	{
		return WIFI_CONN_FAIL;	//�ȴ����ŵ�����
	}	
	return WIFI_OK;
}

uint8_t WIFI_Dect(uint8_t *SSID,uint8_t *Password)
{
	uint8_t res;
	HAL_Delay(500);
	res=ESP8266_work_test(SSID,Password);		
	switch(res)
	{
		case WIFI_OK:
			OLED_ShowString(0,2,"               ");
			OLED_ShowString(0,2,"WIFI SETSSID OK");
			break;
		case WIFI_COMMUNTION_ERR:
			OLED_ShowString(0,2,"               ");
			OLED_ShowString(0,2,"WIFI CONNECT NO");
			break;
		case WIFI_CONN_FAIL:
			OLED_ShowString(0,2,"               ");
			OLED_ShowString(0,2,"WIFI Network NO");
			break;		
		default:
			break;
	}
	return res;
}

//���ӵ�������
uint8_t ESP8266_CONNECT_SERVER(uint8_t *ServerIP,uint8_t *Port)
{	  
		memset(dtbuf,0,200);
		OLED_ShowString(0,2,"               ");
		OLED_ShowString(0,2,"CONNE SERVER ..");
		OLED_ShowString(0,4,"               ");
		OLED_ShowString(0,4,"EMQ MQTT SERVER");
		OLED_ShowString(0,6,"               ");
		OLED_ShowString(0,6,Port);
		sprintf((char *)dtbuf,"AT+CIPSTART=\"TCP\",\"%s\",%s",ServerIP,Port);
		HAL_Delay(5000); //����ʱ��Ҫ��һ�� ��Ȼ�ͻ���busy .. ���������ӵ�wifi����ֱ�ӷ������� 
	  if(ESP8266_send_cmd((uint8_t *)dtbuf,(uint8_t *)"CONNECT",3000))	return 5;		
		//���ö�ģʽ
		if(ESP8266_send_cmd((uint8_t *)"AT+CIPMODE=1",(uint8_t *)"OK",100))	 return 1;
		//����͸��ģʽ
		if(ESP8266_send_cmd((uint8_t *)"AT+CIPSEND",(uint8_t *)">",100))	 return 4;
	
		//printf("***CONNECT SERVER SUCCEED***\r\n");
		USART_RX_STA=0;	
		servernotok=0;
	  return 0;                                                            
}


uint8_t mqtt_Connection(char *Username,char *Password,char *ClientID)
{
		uint8_t cnt=2;
		uint8_t wait;
	  uint8_t js[256];
	  mqttPacketLen=0;
	
	  mqttPacketLen = MQTT_Connect(Username,Password,ClientID,js);
		HAL_UART_Transmit(&huart1,js,mqttPacketLen,100);
		while(cnt--)
		{
			wait=30;//�ȴ�3sʱ��
			while(wait--)
			{
				//CONNECT
				if(USART_RX_BUF[0]==parket_connetAck[0] &&USART_RX_BUF[1]==parket_connetAck[1]) //���ӳɹ�			   
				{
					servernotok = 0;
					return 0;//���ӳɹ�
				}
				HAL_Delay(100);			
			}
		}
		servernotok = 1;
		return 1;		
}

uint8_t mqtt_subscribe(char *Topic)
{
		uint8_t cnt=2;
		uint8_t wait;
	  uint8_t js2[256];
	  mqttPacketLen=0;
	
	  mqttPacketLen = MQTT_SubscribeTopic(Topic,0,1,js2);
		HAL_UART_Transmit(&huart1,js2,mqttPacketLen,100);
		while(cnt--)
		{
			wait=30;//�ȴ�3sʱ��
			while(wait--)
			{
				//CONNECT
				if(USART_RX_BUF[0]==parket_subAck[0] &&USART_RX_BUF[1]==parket_subAck[1]) //���ӳɹ�			   
				{
					servernotok = 0;
					return 0;//���ĳɹ�
				}
				HAL_Delay(100);			
			}
		}
		if(cnt) 
		{
			servernotok = 0;
			return 0;	//���ĳɹ�
		}
		return 1;		
}

//����json����
uint8_t ESP8266_Send_data(char *TopicPost, char *Temp, char *Humi)
{
	  uint8_t js3[256];
	  mqttPacketLen=0;
	
	  memset(dtbuf,0,200); //���
		sprintf((char *)dtbuf,"{\"params\":{\"RelativeHumidity\":%s,\"CurrentTemperature\":%s}}",Humi,Temp); //,EC20_CCID,EC20_CGSN
		
	  mqttPacketLen = MQTT_PublishData(TopicPost,(char *)dtbuf,0,js3);
		HAL_UART_Transmit(&huart1,js3,mqttPacketLen,100);
   	memset(USART_RX_BUF,0,USART_REC_LEN);
		HAL_Delay(300);	
		return 1;		
}




