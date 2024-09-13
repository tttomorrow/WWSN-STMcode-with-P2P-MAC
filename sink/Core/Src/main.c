/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : �������ļ�
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 CSTX.
  * ��������Ȩ����</center></h2>
  *
  * ���������� ST ���� BSD 3-Clause ���Э����Ȩ��
  * ��ֻ�����������Э�������²���ʹ�ø��ļ���
  * ����������µ�ַ��ȡ���Э�鸱����
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include "main.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include "string.h"
#include "oled.h"
#include <stdlib.h>
#include "mqtt.h"
#include "esp8266.h"



////////////////////////////////�����������޸Ĳ����ĵط�(EMQ)////////////////////////////////////////
//#define ServerIP "192.168.31.121"
//#define Port "1883"

//#define ClientID "mqttjs_1309e3e721"    //��Ҫ����Ϊ�û��Լ��Ĳ���
//#define Username   "admin"      //��Ҫ����Ϊ�û��Լ��Ĳ���
//#define Password   "public"  //��Ҫ����Ϊ�û��Լ��Ĳ���

//#define Topic   "testtopic" //��Ҫ����Ϊ�û��Լ��Ĳ���
//#define TopicPost   "testtopic" //��Ҫ����Ϊ�û��Լ��Ĳ���

//#define SSID "Wireless Weak-link Network"
//#define WIFIPassword "18239778101"
/////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////
///////////////������Щ������Ҫ�޸ĵĲ���(TCP)//////////////
#define ServerIP "192.168.31.121" //��������ip��ַ
#define Port "60000"	//�������Ķ˿�

#define SSID "Wireless Weak-link Network" //·���������� 2.4G
#define Password "18239778101" //·����������
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

/**
  * @brief  Ӧ�ó�����ڵ�
  * @retval int
  */
#define SDATA "ToggleLED\n" // �����ַ����������ڴ��ڷ�������
#define ROUTE_REQUEST 0x01
#define ROUTE_REPLY 0x02
#define DATA_PACKET 0x03
#define ACK_PACKET 0x04
#define ROUTING_TABLE_SIZE_INITIAL 10
#define MacH 0x10
#define MacL 0x01
#define channelID 0x09 // �ŵ�ID



// ���ݰ��ṹ��
typedef struct {
    uint8_t destMacH;  // ���ݰ������ֽ�
    uint8_t destMacL;
    uint8_t destchanID;    // Դ�ڵ�ID
    uint8_t sourceMacH;  // ���ݰ������ֽ�
    uint8_t sourceMacL;
    uint8_t sourceID;    // Դ�ڵ�ID
    uint8_t forwardID;   // ת���ڵ�ID
    uint8_t destID;      // Ŀ�Ľڵ�ID
    uint8_t protocol;    // Э������
    char data[20];       // �Զ�������
} DataPacket;

// ·�ɱ�
typedef struct {
    uint8_t destID;      // Ŀ�Ľڵ�ID
    uint8_t nextHopID;   // ��һ���ڵ�ID
    uint8_t macHigh;     // ��һ���ڵ��MAC��ַ
    uint8_t macLow;      // ��һ���ڵ��MAC��ַ
} RoutingEntry;

void SystemClock_Config(void);
void sendRouteRequest(uint8_t destID);
void sendRouteReply(uint8_t destID, uint8_t sourceID);
void sendAddressResolution(uint8_t targetID);
void sendAckPacket(uint8_t destID, uint8_t macH, uint8_t macL);
void handleReceivedPacket(DataPacket* packet);
void processRouteRequest(DataPacket* packet);
void processRouteReply(DataPacket* packet);
void processDataPacket(DataPacket* packet);
void processAckPacket(DataPacket* packet);


uint8_t nodeID = 1; //�ڵ�ID
unsigned char key = 0; // ���ڱ�����յ��ļĴ�������
unsigned char RSSIkey = 0;
unsigned char disOLED[24]; // ����OLED��ʾ������
uint8_t targetID = 1; //��۽ڵ�ID1
char nodeIDStr[8]; // ���ڴ洢�ڵ�ID���ַ���
RoutingEntry *routingTable = NULL;
int routingTableSize = ROUTING_TABLE_SIZE_INITIAL; // ·�ɱ��С
int routingTableCount = 0;
unsigned char packetBUF[sizeof(DataPacket)];
uint32_t previousMillisA0 = 0; //��ʱ�����¶�����
uint32_t previousMillisA1 = 0; //·��������ʱ��
uint8_t sendRoutRequest = 0; //·�������־
uint8_t getRoutReplay = 0; //·�ɻظ���־λ
uint8_t RSSI = 0;
uint8_t res=1;
char *CSTXREVData[3];
unsigned char servernotok=1;

// ͨ�����üĴ��� ����loraģ���ŵ����ַ 
// ��ʼmac��ַΪ�㲥��ַ0x10 0x02�����нڵ�ͳһ�����ŵ�0x09���������ò�ѯ�ֲ�
unsigned char cscxReg[10] = {0xC0, 0x00, 0x07, 0x10, 0x02, 0x01, 0x61, 0x20, 0x09, 0xD0};;	//���Ͳ�ѯ����
// ����ģ���ַ��0x1001���������ַ��0x01,�õ�ַ���ܱ䣩�����ڣ�9600 8N1�������٣�2.4K����
// ���÷���ǰ�ŵ����������ö��㴫�䣨����ǰ�����ֽ�Ϊ��ַ�ߣ���ַ�ͣ��ŵ���һͬ��Ϊ���߷���Ŀ�꣩
//    unsigned char cscxRegCha[4] = {0xC0, 0x05, 0x01, 0x09}; // ����ģ���ŵ�Ϊ0x09
unsigned char csrevReg[12] = {0xC1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};// ���ռĴ�������

void configureModule() {
    cscxReg[3] = MacH;
    cscxReg[4] = MacL;
    // �������ô���...
}

// ���·����Ŀ
void addRoutingEntry(uint8_t destID, uint8_t nextHopID, uint8_t macHigh, uint8_t macLow) {
    if (routingTableCount >= routingTableSize) {
        routingTableSize *= 2;
        routingTable = (RoutingEntry *)realloc(routingTable, routingTableSize * sizeof(RoutingEntry));
    }
    routingTable[routingTableCount].destID = destID;
    routingTable[routingTableCount].nextHopID = nextHopID;
    routingTable[routingTableCount].macHigh = macHigh;
    routingTable[routingTableCount].macLow = macLow;
    routingTableCount++;
}

// ��ӡ·�ɱ�
void printRoutingTable() {
    for (int i = 0; i < routingTableCount; i++) {
        printf("Entry %d: DestID = %02X, NextHopID = %02X, MAC = %02X%02X\n",
               i,
               routingTable[i].destID,
               routingTable[i].nextHopID,
               routingTable[i].macHigh,
               routingTable[i].macLow);
    }
}

// ��ѯ·��
int findRoute(uint8_t destID) {
    printf("Route table:");
    for (int i = 0; i < routingTableCount; i++) {
        printf("%02X?%02X, ", routingTable[i].destID, destID);
        if (routingTable[i].destID == destID) {
            printf("; Routeindex: i=%d\n", i);
            return i;
        }
    }
    return -1; // δ�ҵ�·��
}

void connectServer(){
    //WiFiģ�����ӵ�·���������ֻ��ȵ�
    while(res)
    {
        res=WIFI_Dect((uint8_t *)SSID,(uint8_t *)Password);
        HAL_Delay(200);
    }
    HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET); //LED1���� ��ʾ��Ƭ����wifi���ӳɹ�
    
    res=1;
    while(servernotok)
    {
        res=ESP8266_CONNECT_SERVER((uint8_t *)ServerIP,(uint8_t *)Port);

        HAL_Delay(1000);
    }
}


// ·�ɲ�ѯ
void sendRouteRequest(uint8_t destID) {
    DataPacket packet;
    packet.destMacH = 0xFF;
    packet.destMacL = 0xFF;
    packet.destchanID = channelID;
    packet.sourceMacH = MacH;
    packet.sourceMacL = MacL;
    packet.sourceID = nodeID;
    packet.forwardID = nodeID;
    packet.destID = destID;
    packet.protocol = ROUTE_REQUEST;
    strcpy(packet.data, "RouteReq");

    memcpy(packetBUF, &packet, sizeof(DataPacket));
    USART2_printf("%s\n",packetBUF);
    printf("Route Request sent to find node%d\n", destID);
}

// ·�ɻظ�
void sendRouteReply(uint8_t destID, uint8_t forwardID) {
    DataPacket packet;
    packet.destMacH = 0xFF;
    packet.destMacL = 0xFF;
    packet.destchanID = channelID;
    packet.sourceMacH = MacH;
    packet.sourceMacL = MacL;
    packet.sourceID = nodeID;
    packet.forwardID = forwardID;
    packet.destID = destID;
    packet.protocol = ROUTE_REPLY;
    strcpy(packet.data, "RouteReply");


    memcpy(packetBUF, &packet, sizeof(DataPacket));
    USART2_printf("%s\n",packetBUF);
    for(int i =0;i<sizeof(packet);i++){
    printf("%02x", packetBUF[i]);
    }
    printf("Route Reply sent to %d from node%d\n", destID, nodeID);
}



// ����ACK���ݰ�
void sendAckPacket(uint8_t destID, uint8_t macH, uint8_t macL){
    DataPacket packet;
    packet.destMacH = macH;
    packet.destMacL = macL;
    packet.destchanID = channelID;
    packet.sourceMacH = MacH;
    packet.sourceMacL = MacL;
    packet.sourceID = nodeID;
    packet.forwardID = nodeID;
    packet.destID = destID;
    packet.protocol = ACK_PACKET;
    strcpy(packet.data, "Ack");

    memcpy(packetBUF, &packet, sizeof(packet));
    USART2_printf("%s\n",packetBUF);
    printf("Acknowledgement sent to %d\n", destID);
}

// ������յ������ݰ�
void handleReceivedPacket(DataPacket* packet) {
    switch (packet->protocol) {
        case ROUTE_REQUEST:
            processRouteRequest(packet);
            break;
        case ROUTE_REPLY:
            processRouteReply(packet);
            break;
        case DATA_PACKET:
            processDataPacket(packet);
            break;
        case ACK_PACKET:
            processAckPacket(packet);
            break;
        default:
            printf("Unknown protocol: %d\n", packet->protocol);
            break;
    }
}

void processRouteRequest(DataPacket* packet) {
    // ��ѯ·�ɱ�����Ŀ���ַ��·��
     int routeIndex = findRoute(packet->destID);
    // ����·�������߼�
    // ������͹�·�������Ҹ������ѯ��·���յ���֮ǰ��ѯ��һ�£������κβ���

    // ��ѯ·�ɱ�
    printf("Received Route Request from %d\n", packet->sourceID);
    if (routeIndex != -1) {
    // ·�ɱ����е�ַ
    sendRouteReply(packet->sourceID, routingTable[routeIndex].nextHopID);
    }
    else {
    // ·�ɱ���û��ַ
        if(sendRoutRequest == 1){
        // ���֮ǰ���͹�·�ɲ�ѯ����
        return;
        }
        printf("No route found for destination node %d\n", packet->destID);
        sendRouteRequest(packet->destID);
        previousMillisA1 = HAL_GetTick();
        sendRoutRequest = 1;
    
}
}

void processRouteReply(DataPacket* packet) {
    printf("Received Route Reply from %02X%02X, \npacket sourceID:%02X, \nforwardID:%02X, \ndestID:%02X\n", packet->sourceMacH, 
            packet->sourceMacL, packet->sourceID, packet->forwardID, packet->destID);
    // ����·�ɻظ��߼�
    int routeIndex = findRoute(packet->destID);
    //·�ɱ���û����Ϣ�����в�ͬ��·�������·��
    if (routeIndex == -1 || routingTable[routeIndex].macLow != packet->sourceMacL){
        
    addRoutingEntry(packet->forwardID, packet->sourceID, packet->sourceMacH, packet->sourceMacL);
    getRoutReplay = 1;
    printf("AddRoutingEntry\n");
    }
    

}


void processDataPacket(DataPacket* packet) {
    printf("Received Data Packet from %d\n", packet->sourceID);
    // �������ݰ��߼�
    if(packet->destID == nodeID){
        // �Լ������ݰ����յ�
        memset(disOLED,0,32);
        sprintf((char *)disOLED,"T:%d.%dC H:%d.%dR",USART2_RX_BUF[6],USART2_RX_BUF[7],USART2_RX_BUF[8],USART2_RX_BUF[9]);
        OLED_ShowString(0,6,disOLED);
        // ����ack���ݰ�
        sendAckPacket(packet->sourceID, packet->sourceMacH, packet->sourceMacL);
        for(int i=0;i<64;i++){
        printf("%02X ",USART2_RX_BUF[i]); 
        }//ͨ������������,��ӡ64�ֽڵ����ݰ�
        printf("\n");

       
        // �����ݰ����͵�������������ϻ�۽ڵ�Ĳ�����
        if(servernotok){
        connectServer();
        }
       //�������ݰ���tcp������
        if(!servernotok)
        {
            if(USART2_RX_STA == REC_OK)
            {
                    // ��ӡ���ݰ�����
                printf("sourceMacH: 0x%02X\n", packet->sourceMacH);
                printf("sourceMacL: 0x%02X\n", packet->sourceMacL);
                printf("sourceID: 0x%02X\n", packet->sourceID);
                printf("forwardID: 0x%02X\n", packet->forwardID);
                printf("destID: 0x%02X\n", packet->destID);
                printf("protocol: 0x%02X\n", packet->protocol);
                for(int i = 0; i<sizeof(packet->data);i++){
                printf("data[%d]: %d\n", i, packet->data[i]);
                }
                
                OLED_ShowString(0,2,"               ");
                OLED_ShowString(0,2,"Send SERVER OK ");
            }
        }
        OLED_ShowString(0,6,"                ");
        
        
        // ��ȡ��������RSSI
        // ���M1�Ƿ񱻳ɹ���Ϊ0,M0�Ƿ񱻳ɹ���Ϊ1,WORģʽ

        if (HAL_GPIO_ReadPin(M0_GPIO_Port, M0_Pin) == 0 && HAL_GPIO_ReadPin(M1_GPIO_Port, M1_Pin) == 0){
            //printf(" M1 = 0; M0 = 0 \n");
            
            unsigned char cscxRSSIreq2[6] = {0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x01};
            unsigned char cscxRSSI[4] = {0x00, 0x00, 0x00, 0x00};
            CS_Reg_Send_Data(cscxRSSIreq2, sizeof(cscxRSSIreq2)); // ���� cscxRSSIreq2 ���Ĵ���
            HAL_Delay(300); // �ӳ�  ����
            
            cstx_reg_Receive_Data(cscxRSSI, &RSSIkey); // ���ռĴ���������
            //printf("\nLORA REG CODE %d REG->", RSSIkey); // ��ӡ LORA �Ĵ�������ͼĴ�����Ϣ
//            for (int i = 0; i < 4; i++) // ���ֽڴ�ӡ���յ��ļĴ�������
//                {
//                    printf("%02X", cscxRSSI[i]);
//                    printf(" ");
//                }
            printf("currentChannelNoise: -%ddBm\n", 256-cscxRSSI[3]);
            printf("currentChannelSNR: %ddB\n", RSSI-cscxRSSI[3]);
        }
        memset(USART2_RX_BUF, 0, USART_REC_LEN); // ��ս��ջ�����
        USART2_RX_STA = 0;
        
       
    }
    else{
        // �Լ��������ݰ����յ�
        // ��ѯ·�ɱ�����Ŀ���ַ��·��
        int routeIndex = findRoute(targetID);
        printf("routeIndex: %d\n", routeIndex);
        // �е����ݰ��յ��·�� ��������յ������Լ������ݰ�������Ϊ���Լ���·�ɱ��е��յ��·�ɣ�������Ϊǰһ��������·�ɻظ���
        if (routeIndex != -1) {
        packet->forwardID = nodeID;
        packet->sourceMacH = MacH;
        packet->sourceMacL = MacL;
        packet->destMacH = routingTable[routeIndex].macHigh;
        packet->destMacL = routingTable[routeIndex].macLow;
        
        memcpy(packetBUF, packet, sizeof(DataPacket));
        USART2_printf("%s\n",packetBUF);
        // ����ack���ݰ�
        sendAckPacket(packet->sourceID, packet->sourceMacH, packet->sourceMacL);
        }
        // ���û�е��յ��·�ɣ���ʱûд��ֻ�����Ӷ��˲��ܲ��ɴ��յ㣻�������·�ɲ�ѯ��·��ʧ�ܱ��ģ�
    }
}

void processAckPacket(DataPacket* packet) {
    printf("Received Acknowledgement from %d\n", packet->sourceID);
    packet->destMacH = packet->sourceMacH;
    packet->destMacL = packet->sourceMacL;
    packet->sourceMacH = MacH;
    packet->sourceMacL = MacL;
    
    // ����ȷ�ϰ��߼�
}


// ������
int main(void)
{
    int i;
    /* MCU Configuration--------------------------------------------------------*/

    HAL_Init(); // ��ʼ�� HAL ��
    

    
    SystemClock_Config(); // ����ϵͳʱ��

    MX_GPIO_Init(); // ��ʼ�� GPIO �˿�
    MX_USART1_UART_Init(); // ��ʼ�� USART1 �˿�
    MX_USART2_UART_Init(); // ��ʼ�� USART2 �˿�

    USART_Interupt_Enable(); // ʹ�ܴ��ڽ����жϺͿ����ж�
    
    
    // ��ʼ��·�ɱ�
    // ��ʼ��·�ɱ�
    routingTableSize = 2; // ��ʼ·�ɱ��С
    routingTable = (RoutingEntry *)malloc(routingTableSize * sizeof(RoutingEntry));

    printf("LORA Board LED OK\n"); // ��ӡ LORA ��� LED ״̬��Ϣ
    sprintf(nodeIDStr, "%d", nodeID);
    CS_OLED_Init(); // ��ʼ�� OLED ��ʾ��
    OLED_Clear(); // ��� OLED ��ʾ������
    OLED_ShowString(16, 0, "WWSN NODE"); // �� OLED ��ʾ������ʾ�ַ��� "WWSN NODE"
    printf("WWSN Node %d\n", nodeID); // ��ӡnodeID
    OLED_ShowString(100, 0, (unsigned char*)nodeIDStr); // �� OLED ��ʾ������ʾ�ڵ�ID
    cstxInitialize(); // ��ʼ�� LED ��ʹ����˸


// ͨ�����üĴ��� ����loraģ���ŵ����ַ 
    configureModule();
    if (HAL_GPIO_ReadPin(M1_GPIO_Port, M1_Pin) == 1) // ��� GPIO ���� M1 �ĵ�ƽ״̬��ֻ���ڸߵ�ƽʱ�ſ�ʼ��ȡģ��ļĴ���
    {
        printf("M1 = 1 ; M0 = 0 Register mode \n"); // ��ӡ M1 �� M0 ��״̬��Ϣ����ʾ���ڼĴ���ģʽ
        CS_Reg_Send_Data(cscxReg, sizeof(cscxReg)); // ���� cscxReg ���Ĵ���
        HAL_Delay(300); // �ӳ� 300 ����
        
    
    cstx_reg_Receive_Data(csrevReg, &key); // ���ռĴ���������
    printf("\nLORA REG CODE %d REG->", key); // ��ӡ LORA �Ĵ�������ͼĴ�����Ϣ
    memset(disOLED, 0, 24); // ��� disOLED ����
    
    for (i = 0; i < 12; i++) // ���ֽڴ�ӡ���յ��ļĴ�������
    {
        printf("%02X", csrevReg[i]);
        printf(" ");
    }
    
    OLED_ShowString(0, 2, "Addr:"); // �� OLED ��ʾ������ʾ "Addr:"
    for (i = 3; i < 5; i++) // ��ʾ�Ĵ����ĵ�ַ��Ϣ
    {
        sprintf((char *)disOLED, "%02X", csrevReg[i]);
//        printf("disOLED=%s,", disOLED);
        OLED_ShowString(i * 16 - 4, 2, disOLED);
    }
    
        OLED_ShowString(82, 2, "Ch:"); // �� OLED ��ʾ������ʾ "Ch:"
        sprintf((char *)disOLED, "%02X", csrevReg[8]);
//        printf("disOLED=%s\n", disOLED);
        OLED_ShowString(104, 2, disOLED);
    }
    else
    {
        printf("M1 = 0; M0 = 0 Transparent mode \n"); // ��ӡ M1 �� M0 ��״̬��Ϣ����ʾ����͸��ģʽ
    }

  
  
    HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET); // ʹ��ģ������ M1 0
    HAL_Delay(1000);
    memset(USART2_RX_BUF, 0, USART_REC_LEN); // ��ս��ջ�����
    USART2_RX_STA = 0;
    memset(USART_RX_BUF,0,USART_REC_LEN);
    USART_RX_STA = 0;
    
    
    
    /* ����ѭ�� */
    
    
    ///////////////////////////////////////////////////
    HAL_Delay(300); // �ӳ�  ����

        
    // ѭ����ȡ���������ݲ�����
    
    if (HAL_GPIO_ReadPin(M1_GPIO_Port, M1_Pin) == 0){
        printf("M1 = 0; M0 = 0 Transparent mode \n");
    }
 

    uint32_t currentMillis; //��ȡ��ǰϵͳʱ��
    
    addRoutingEntry(nodeID, nodeID, MacH, MacL);
    DataPacket receivedPacket;
    
    while (1)
    {
        
        
    while(servernotok|res){
    connectServer();
    }
        
    
    if(sendRoutRequest == 1 && getRoutReplay == 0){
    currentMillis = HAL_GetTick(); //��ȡ��ǰϵͳʱ��
    if(currentMillis - previousMillisA1 > 5000 && getRoutReplay == 0){
    sendRouteRequest(targetID);
    previousMillisA1 = HAL_GetTick();
    }
    }
        
    // ����LORA ���͹��������� 
    if (USART2_RX_STA == REC_OK) // ����Ƿ���յ�����
    {
//        printf("receive data with protocol:%02X",  USART2_RX_BUF[5]);
        receivedPacket.sourceMacH = USART2_RX_BUF[0];
        receivedPacket.sourceMacL = USART2_RX_BUF[1];
        receivedPacket.sourceID = USART2_RX_BUF[2];
        receivedPacket.forwardID = USART2_RX_BUF[3];
        receivedPacket.destID = USART2_RX_BUF[4];
        receivedPacket.protocol = USART2_RX_BUF[5];
        memcpy(receivedPacket.data, &USART2_RX_BUF[6], sizeof(receivedPacket) - 6);

        // �����ݰ�ĩβ��ʼ��ǰ������ֱ���ҵ���һ�������ֽ�
        for (int i = sizeof(receivedPacket) - 1; i >= 0; i--) {
            if (USART2_RX_BUF[i] != 0) {
                RSSI = USART2_RX_BUF[i];
                printf("receivedPacketWithRSSI :-%ddBm\n", 256-RSSI);
                break;
            }
        }
//        for(int i = 0; i<sizeof(USART2_RX_BUF) ;i++){
//        printf("%02X", USART2_RX_BUF[i]);
//        }
//        printf("\n");
//        sscanf((char*)USART2_RX_BUF, "%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%s", 
//                                                &receivedPacket.sourceMacH, 
//                                                &receivedPacket.sourceMacL, 
//                                                &receivedPacket.sourceID, 
//                                                &receivedPacket.forwardID, 
//                                                &receivedPacket.destID, 
//                                                &receivedPacket.protocol, 
//                                                receivedPacket.data);
        
        handleReceivedPacket(&receivedPacket);
        printf("\n\n\n");
        memset(USART2_RX_BUF, 0, USART_REC_LEN); // ��ս��ջ�����
        USART2_RX_STA = 0; // ���ý���״̬
    }
    
//    //���Է��͹���������(������)
//    if(USART_RX_STA == REC_OK)
//    {
//        printf("Receive Date from PC and Send to Destination\n"); 
//        USART2_printf("%s\n",USART_RX_BUF); //ͨ��lora���ͳ�ȥ
//        memset(USART_RX_BUF,0,USART_REC_LEN);
//        USART_RX_STA = 0;
//        }
    } // while
    
} // main














/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT CSTX *****END OF FILE****/

