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
#include "dht11.h"
#include <stdio.h>
#include "string.h"
#include "oled.h"
#include <stdlib.h>





/**
  * @brief  Ӧ�ó�����ڵ�
  * @retval int
  */
#define SDATA "ToggleLED\r\n" // �����ַ����������ڴ��ڷ�������
#define ROUTE_REQUEST 0x01
#define ROUTE_REPLY 0x02
#define DATA_PACKET 0x03
#define ACK_PACKET 0x04
#define ROUTING_TABLE_SIZE_INITIAL 10
#define MacH 0x10
#define MacL 0x02 ///////////////////////////////////////////
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


uint8_t nodeID = 2; //�ڵ�ID///////////////////////////////////////////////
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
        printf("\r\nEntry %d: DestID = %02X, NextHopID = %02X, MAC = %02X%02X\n",
               i,
               routingTable[i].destID,
               routingTable[i].nextHopID,
               routingTable[i].macHigh,
               routingTable[i].macLow);
    }
}

// ��ѯ·��
int findRoute(uint8_t destID) {
    printf("\r\nRoute table:");
    for (int i = 0; i < routingTableCount; i++) {
        printf("%02X=%02X, ", routingTable[i].destID, destID);
        if (routingTable[i].destID == destID) {
            printf("; Routeindex: i=%d\r\n", i);
            return i;
        }
    }
    return -1; // δ�ҵ�·��
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
    USART2_printf("%s\r\n",packetBUF);
    printf("\r\nRoute Request sent to find node %d\r\n", destID);
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
    USART2_printf("%s\r\n",packetBUF);
    printf("\r\nRoute Reply sent to %d from node %d\r\n", destID, nodeID);
}



// ����ACK���ݰ�
void sendAckPacket(uint8_t destID, uint8_t macH, uint8_t macL){
    DataPacket packet;
    packet.destMacH = macH;
    packet.destMacL = macL;
    packet.destchanID = channelID;
    packet.sourceID = nodeID;
    packet.forwardID = nodeID;
    packet.destID = destID;
    packet.protocol = ACK_PACKET;
    strcpy(packet.data, "Ack");

    memcpy(packetBUF, &packet, sizeof(packet));
    USART2_printf("%s\r\n",packetBUF);
    printf("\r\nAcknowledgement sent to %d\r\n", destID);
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
            printf("Unknown protocol: %d\r\n", packet->protocol);
            break;
    }
}

void processRouteRequest(DataPacket* packet) {
    // ��ѯ·�ɱ�����Ŀ���ַ��·��
     int routeIndex = findRoute(packet->destID);
    // ����·�������߼�
    // ������͹�·�������Ҹ������ѯ��·���յ���֮ǰ��ѯ��һ�£������κβ���

    // ��ѯ·�ɱ�
    printf("\r\nReceived Route Request from %d\r\n", packet->sourceID);
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
        printf("\r\nNo route found for destination node %d\r\n", packet->destID);
        sendRouteRequest(packet->destID);
        previousMillisA1 = HAL_GetTick();
        sendRoutRequest = 1;
    
}
}

void processRouteReply(DataPacket* packet) {
    printf("\r\nReceived Route Reply from %02X%02X, \r\npacket sourceID:%02X, \r\nforwardID:%02X, \r\ndestID:%02X\r\n", packet->sourceMacH, 
            packet->sourceMacL, packet->sourceID, packet->forwardID, packet->destID);
    // ����·�ɻظ��߼�
    int routeIndex = findRoute(packet->destID);
    //·�ɱ���û����Ϣ�����в�ͬ��·�������·��
    if (routeIndex == -1 || routingTable[routeIndex].macLow != packet->sourceMacL){
        
    addRoutingEntry(packet->forwardID, packet->sourceID, packet->sourceMacH, packet->sourceMacL);
    getRoutReplay = 1;
    printf("\r\nAddRoutingEntry\r\n");
    }
    

}


void processDataPacket(DataPacket* packet) {
    printf("\r\nReceived Data Packet from %d\r\n", packet->sourceID);
    // �������ݰ��߼�
    if(packet->destID == nodeID){
       // �Լ������ݰ����յ�
       memset(disOLED,0,32);
       sprintf((char *)disOLED,"T:%d.%dC H:%d.%dR",USART2_RX_BUF[6],USART2_RX_BUF[7],USART2_RX_BUF[8],USART2_RX_BUF[9]);
       printf("\r\nReceive Date :"); 
       OLED_ShowString(0,6,disOLED);
       for(int i=0;i<16;i++){
       printf("%4X",USART2_RX_BUF[i]); 
       }//ͨ������������
       
       // �����ݰ����͵�������������ϻ�۽ڵ�Ĳ�����
       
       printf("\r\n"); 
       memset(USART2_RX_BUF,0,USART_REC_LEN);
       USART2_RX_STA = 0;
    }
    else{
        // �Լ��������ݰ����յ�
        // ��ѯ·�ɱ�����Ŀ���ַ��·��
        int routeIndex = findRoute(targetID);
        printf("\r\nrouteIndex: %d\r\n", routeIndex);
        // �е����ݰ��յ��·�� ��������յ������Լ������ݰ�������Ϊ���Լ���·�ɱ��е��յ��·�ɣ�������Ϊǰһ��������·�ɻظ���
        if (routeIndex != -1) {
        packet->forwardID = nodeID;
        packet->sourceMacH = MacH;
        packet->sourceMacL = MacL;
        packet->destMacH = routingTable[routeIndex].macHigh;
        packet->destMacL = routingTable[routeIndex].macLow;
        
        memcpy(packetBUF, packet, sizeof(DataPacket));
        USART2_printf("%s\r\n",packetBUF);
        }
        // ���û�е��յ��·�ɣ���ʱûд��ֻ�����Ӷ��˲��ܲ��ɴ��յ㣻�������·�ɲ�ѯ��·��ʧ�ܱ��ģ�
    }
    // ����ack���ݰ�
    sendAckPacket(packet->sourceID, packet->sourceMacH, packet->sourceMacL);
    
    // ��ȡ��������RSSI
    // ���M1�Ƿ񱻳ɹ���Ϊ0,M0�Ƿ񱻳ɹ���Ϊ1,WORģʽ
    if (HAL_GPIO_ReadPin(M0_GPIO_Port, M0_Pin) == 0 && HAL_GPIO_ReadPin(M1_GPIO_Port, M1_Pin) == 0){
        printf("\r\n M1 = 0; M0 = 0 \r\n");
        
        unsigned char cscxRSSIreq2[6] = {0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x01};
        CS_Reg_Send_Data(cscxRSSIreq2, sizeof(cscxRSSIreq2)); // ���� cscxRSSIreq2 ���Ĵ���
        HAL_Delay(300); // �ӳ�  ����
        unsigned char cscxRSSI[4] = {0x00, 0x00, 0x00, 0x00};
        cstx_reg_Receive_Data(cscxRSSI, &RSSIkey); // ���ռĴ���������
        printf("\r\n\r\nLORA REG CODE %d REG->", RSSIkey); // ��ӡ LORA �Ĵ�������ͼĴ�����Ϣ
        for (int i = 0; i < 4; i++) // ���ֽڴ�ӡ���յ��ļĴ�������
            {
                printf("%02X", cscxRSSI[i]);
                printf(" ");
            }
        printf("\n");
        printf("\r\ncurrentChannelNoise: -%ddBm\r\n", 256-cscxRSSI[3]);
        printf("\r\ncurrentChannelSNR: %ddB\r\n", RSSI-cscxRSSI[3]);
    }
}

void processAckPacket(DataPacket* packet) {
    printf("\r\nReceived Acknowledgement from %d\r\n", packet->sourceID);
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
    
    // ��ʼ��·�ɱ�
    // ��ʼ��·�ɱ�
    routingTableSize = 2; // ��ʼ·�ɱ��С
    routingTable = (RoutingEntry *)malloc(routingTableSize * sizeof(RoutingEntry));
    
    SystemClock_Config(); // ����ϵͳʱ��

    MX_GPIO_Init(); // ��ʼ�� GPIO �˿�
    MX_USART1_UART_Init(); // ��ʼ�� USART1 �˿�
    MX_USART2_UART_Init(); // ��ʼ�� USART2 �˿�

    USART_Interupt_Enable(); // ʹ�ܴ��ڽ����жϺͿ����ж�

    printf("LORA Board LED OK\r\n"); // ��ӡ LORA ��� LED ״̬��Ϣ
    sprintf(nodeIDStr, "%d", nodeID);
    CS_OLED_Init(); // ��ʼ�� OLED ��ʾ��
    OLED_Clear(); // ��� OLED ��ʾ������
    OLED_ShowString(16, 0, "WWSN NODE"); // �� OLED ��ʾ������ʾ�ַ��� "WWSN NODE"
    printf("\r\nWWSN Node %d\r\n", nodeID); // ��ӡnodeID
    OLED_ShowString(100, 0, (unsigned char*)nodeIDStr); // �� OLED ��ʾ������ʾ�ڵ�ID
    cstxInitialize(); // ��ʼ�� LED ��ʹ����˸


// ͨ�����üĴ��� ����loraģ���ŵ����ַ 
//    configureModule();
    if (HAL_GPIO_ReadPin(M1_GPIO_Port, M1_Pin) == 1) // ��� GPIO ���� M1 �ĵ�ƽ״̬��ֻ���ڸߵ�ƽʱ�ſ�ʼ��ȡģ��ļĴ���
    {
        printf("\r\nM1 = 1 ; M0 = 0 Register mode \r\n"); // ��ӡ M1 �� M0 ��״̬��Ϣ����ʾ���ڼĴ���ģʽ
        CS_Reg_Send_Data(cscxReg, sizeof(cscxReg)); // ���� cscxReg ���Ĵ���
        HAL_Delay(300); // �ӳ� 300 ����
        printf("\r\n\r\nLORA REG CODE %d UART2->", regConut); // ��ӡ LORA �Ĵ�������� UART2 ��Ϣ
        
    for (i = 0; i < 12; i++) // ���ֽڴ�ӡ USART2 ���ջ�����������
    {
        printf("%02X", USART2_RX_BUF[i]);
        printf(" ");
    }
    
    cstx_reg_Receive_Data(csrevReg, &key); // ���ռĴ���������
    printf("\r\n\r\nLORA REG CODE %d REG->", key); // ��ӡ LORA �Ĵ�������ͼĴ�����Ϣ
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
//        printf("disOLED=%s\r\n", disOLED);
        OLED_ShowString(104, 2, disOLED);
    }
    else
    {
        printf("\r\nM1 = 0; M0 = 0 Transparent mode \r\n"); // ��ӡ M1 �� M0 ��״̬��Ϣ����ʾ����͸��ģʽ
    }

  
  
  
    memset(USART2_RX_BUF, 0, USART_REC_LEN); // ��ս��ջ�����
    USART2_RX_STA = 0;
    /* ����ѭ�� */
    /* USER CODE BEGIN WHILE */
    // ѭ����ȡ���������ݲ�����
    HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET); // ʹ��ģ������ M1 0
    if (HAL_GPIO_ReadPin(M1_GPIO_Port, M1_Pin) == 0){
        printf("\r\nM1 = 0; M0 = 0 Transparent mode \r\n");
    }
    HAL_Init();  

    uint32_t currentMillis; //��ȡ��ǰϵͳʱ��
    
    addRoutingEntry(nodeID, nodeID, MacH, MacL);
    DataPacket receivedPacket;
    DataPacket datapacket;
    while (1)
    {   

        currentMillis = HAL_GetTick();
        if (currentMillis - previousMillisA0 >= 10000 || previousMillisA0 == 0) //��ǰʱ��̼�ȥǰ��ִ�е�ʱ���
        {
            previousMillisA0 = currentMillis; //����ִ��ʱ���

//        OLED_ShowString(0, 4, "Send data ......"); // �� OLED ��ʾ������ʾ "Send data ......"
        
        if (DHT11_READ_DATA() == 1) // ��ȡ DHT11 ����
        {
            
            printf("\r\nRead DHT11 Succeed \r\n");
    //        USART2_printf("A,%d.%d%%,%d.%d", Dht11data[0], Dht11data[1], Dht11data[2], Dht11data[3]); // ��ӡ��ȡ���� DHT11 ����
            datapacket.data[0] = Dht11data[2];
            datapacket.data[1] = Dht11data[3];
            datapacket.data[2] = Dht11data[0];
            datapacket.data[3] = Dht11data[1];
            memset(disOLED, 0, 32); // ��� disOLED ����
            sprintf((char *)disOLED,"T:%d.%dC H:%d.%dR",datapacket.data[0],datapacket.data[1],datapacket.data[2],datapacket.data[3]);
            OLED_ShowString(0, 6, disOLED); // �� OLED ��ʾ������ʾ��ʽ����� DHT11 ����
            
            // ��ѯ·�ɱ�����Ŀ���ַ��·��
            int routeIndex = findRoute(targetID);
            printf("\r\nRouteIndex: %d\r\n", routeIndex);
            if (routeIndex != -1) {
                // �������ݻ�����������
                // �������ݰ�
                datapacket.destMacH = routingTable[routeIndex].macHigh;
                datapacket.destMacL = routingTable[routeIndex].macLow;
                datapacket.destchanID = channelID; // �ŵ�0
                datapacket.sourceMacH = MacH;
                datapacket.sourceMacL = MacL;
                datapacket.sourceID = nodeID;
                datapacket.forwardID = nodeID;
                datapacket.destID = targetID;
                datapacket.protocol = DATA_PACKET;

//                printf("packet: %d, %d, %d, %x, %s\n", 
//                                    (int)packet.sourceID, 
//                                    (int)packet.forwardID, 
//                                    (int)packet.destID, 
//                                     packet.protocol, 
//                                     packet.data);
                printf("\r\nRoute found for destination node\r\n");
                
                memcpy(packetBUF, &datapacket, sizeof(datapacket));
                printf("Send packet to %02X%02X%02X\r\n", packetBUF[0], packetBUF[1], packetBUF[2]);
                
                USART2_printf("%s\r\n", packetBUF);

               
                

                   
            } else {
                printf("\r\nNo route found for destination node %d\r\n", targetID);
                sendRouteRequest(targetID);
                previousMillisA1 = HAL_GetTick();
                sendRoutRequest = 1;
                getRoutReplay = 0;
                
            }
        }
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
//        printf("\r\nReceive data with protocol:%02X\r\n",  USART2_RX_BUF[5]);
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
                printf("\r\nreceivedPacketWithRSSI :-%ddBm\r\n", 256-RSSI);
                break;
            }
        }
        for(int i = 0; i<sizeof(USART2_RX_BUF) ;i++){
        printf("%02X", USART2_RX_BUF[i]);
        }
//        sscanf((char*)USART2_RX_BUF, "%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%s", 
//                                                &receivedPacket.sourceMacH, 
//                                                &receivedPacket.sourceMacL, 
//                                                &receivedPacket.sourceID, 
//                                                &receivedPacket.forwardID, 
//                                                &receivedPacket.destID, 
//                                                &receivedPacket.protocol, 
//                                                receivedPacket.data);
        
        handleReceivedPacket(&receivedPacket);
        memset(USART2_RX_BUF, 0, USART_REC_LEN); // ��ս��ջ�����
        USART2_RX_STA = 0; // ���ý���״̬

      
      
    }
    
    //���Է��͹���������(������)
    if(USART_RX_STA == REC_OK)
    {
        printf("\r\nReceive Date from PC and Send to Destination\r\n"); 
        USART2_printf("%s\r\n",USART_RX_BUF); //ͨ��lora���ͳ�ȥ
        memset(USART_RX_BUF,0,USART_REC_LEN);
        USART_RX_STA = 0;
        }
    } // while
    
} // main














/**
  * @brief ϵͳʱ������
  * @retval ��
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** �������ڲ������������ѹ
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** ��ʼ�� RCC ����������ָ������
  * �� RCC_OscInitTypeDef �ṹ�����á�
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
  /** ��ʼ�� CPU, AHB �� APB ����ʱ��
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
  /** ��ʼ����Χ�豸ʱ��
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
  * @brief  ��������ʱִ�еĺ�����
  * @retval ��
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* �û������ڴ˴�����Լ��Ĵ��������� HAL ���󷵻�״̬ */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  ������Դ����Դ�ļ������кš�
  * @param  file: ָ��Դ�ļ�����ָ��
  * @param  line: ���Դ����к�
  * @retval ��
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* �û������ڴ˴�����Լ��Ĵ����������ļ������кţ�
     ���磺printf("����Ĳ���ֵ: �ļ� %s �� %d ��\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT CSTX *****END OF FILE****/
