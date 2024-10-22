/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序文件
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 CSTX.
  * 保留所有权利。</center></h2>
  *
  * 本软件组件由 ST 依据 BSD 3-Clause 许可协议授权，
  * 你只有在遵守许可协议的情况下才能使用该文件。
  * 你可以在以下地址获取许可协议副本：
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
  * @brief  应用程序入口点
  * @retval int
  */
#define SDATA "ToggleLED\r\n" // 定义字符串常量用于串口发送数据
#define ROUTE_REQUEST 0x01
#define ROUTE_REPLY 0x02
#define DATA_PACKET 0x03
#define ACK_PACKET 0x04
#define ROUTING_TABLE_SIZE_INITIAL 10
#define MacH 0x10
#define MacL 0x02 ///////////////////////////////////////////
#define channelID 0x09 // 信道ID



// 数据包结构体
typedef struct {
    uint8_t destMacH;  // 数据包第三字节
    uint8_t destMacL;
    uint8_t destchanID;    // 源节点ID
    uint8_t sourceMacH;  // 数据包第三字节
    uint8_t sourceMacL;
    uint8_t sourceID;    // 源节点ID
    uint8_t forwardID;   // 转发节点ID
    uint8_t destID;      // 目的节点ID
    uint8_t protocol;    // 协议类型
    char data[20];       // 自定义数据
} DataPacket;

// 路由表
typedef struct {
    uint8_t destID;      // 目的节点ID
    uint8_t nextHopID;   // 下一跳节点ID
    uint8_t macHigh;     // 下一跳节点高MAC地址
    uint8_t macLow;      // 下一跳节点低MAC地址
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


uint8_t nodeID = 2; //节点ID///////////////////////////////////////////////
unsigned char key = 0; // 用于保存接收到的寄存器代码
unsigned char RSSIkey = 0;
unsigned char disOLED[24]; // 用于OLED显示的数组
uint8_t targetID = 1; //汇聚节点ID1
char nodeIDStr[8]; // 用于存储节点ID的字符串
RoutingEntry *routingTable = NULL;
int routingTableSize = ROUTING_TABLE_SIZE_INITIAL; // 路由表大小
int routingTableCount = 0;
unsigned char packetBUF[sizeof(DataPacket)];
uint32_t previousMillisA0 = 0; //计时发送温度数据
uint32_t previousMillisA1 = 0; //路由请求发送时间
uint8_t sendRoutRequest = 0; //路由请求标志
uint8_t getRoutReplay = 0; //路由回复标志位
uint8_t RSSI = 0;

// 通过配置寄存器 设置lora模块信道与地址 
// 初始mac地址为广播地址0x10 0x02，所有节点统一采用信道0x09；具体配置查询手册
unsigned char cscxReg[10] = {0xC0, 0x00, 0x07, 0x10, 0x02, 0x01, 0x61, 0x20, 0x09, 0xD0};;	//发送查询数据
// 配置模块地址（0x1001）、网络地址（0x01,该地址不能变）、串口（9600 8N1）、空速（2.4K）、
// 启用发射前信道监听、启用顶点传输（串口前三个字节为地址高，地址低，信道，一同作为无线发射目标）
//    unsigned char cscxRegCha[4] = {0xC0, 0x05, 0x01, 0x09}; // 配置模块信道为0x09
unsigned char csrevReg[12] = {0xC1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};// 接收寄存器数据

void configureModule() {
    cscxReg[3] = MacH;
    cscxReg[4] = MacL;
    // 其他配置代码...
}

// 添加路由条目
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

// 打印路由表
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

// 查询路由
int findRoute(uint8_t destID) {
    printf("\r\nRoute table:");
    for (int i = 0; i < routingTableCount; i++) {
        printf("%02X=%02X, ", routingTable[i].destID, destID);
        if (routingTable[i].destID == destID) {
            printf("; Routeindex: i=%d\r\n", i);
            return i;
        }
    }
    return -1; // 未找到路由
}


// 路由查询
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

// 路由回复
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



// 发送ACK数据包
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

// 处理接收到的数据包
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
    // 查询路由表，查找目标地址的路由
     int routeIndex = findRoute(packet->destID);
    // 处理路由请求逻辑
    // 如果发送过路由请求且该请求查询的路径终点与之前查询的一致，则不做任何操作

    // 查询路由表
    printf("\r\nReceived Route Request from %d\r\n", packet->sourceID);
    if (routeIndex != -1) {
    // 路由表中有地址
    sendRouteReply(packet->sourceID, routingTable[routeIndex].nextHopID);
    }
    else {
    // 路由表中没地址
        if(sendRoutRequest == 1){
        // 如果之前发送过路由查询报文
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
    // 处理路由回复逻辑
    int routeIndex = findRoute(packet->destID);
    //路由表中没该信息或者有不同的路径则添加路由
    if (routeIndex == -1 || routingTable[routeIndex].macLow != packet->sourceMacL){
        
    addRoutingEntry(packet->forwardID, packet->sourceID, packet->sourceMacH, packet->sourceMacL);
    getRoutReplay = 1;
    printf("\r\nAddRoutingEntry\r\n");
    }
    

}


void processDataPacket(DataPacket* packet) {
    printf("\r\nReceived Data Packet from %d\r\n", packet->sourceID);
    // 处理数据包逻辑
    if(packet->destID == nodeID){
       // 自己是数据包的终点
       memset(disOLED,0,32);
       sprintf((char *)disOLED,"T:%d.%dC H:%d.%dR",USART2_RX_BUF[6],USART2_RX_BUF[7],USART2_RX_BUF[8],USART2_RX_BUF[9]);
       printf("\r\nReceive Date :"); 
       OLED_ShowString(0,6,disOLED);
       for(int i=0;i<16;i++){
       printf("%4X",USART2_RX_BUF[i]); 
       }//通过电脑来接收
       
       // 将数据包发送到服务器（添加上汇聚节点的操作）
       
       printf("\r\n"); 
       memset(USART2_RX_BUF,0,USART_REC_LEN);
       USART2_RX_STA = 0;
    }
    else{
        // 自己不是数据包的终点
        // 查询路由表，查找目标地址的路由
        int routeIndex = findRoute(targetID);
        printf("\r\nrouteIndex: %d\r\n", routeIndex);
        // 有到数据包终点的路由 （如果接收到发给自己的数据包，就以为这自己的路由表有到终点的路由，这是因为前一步进行了路由回复）
        if (routeIndex != -1) {
        packet->forwardID = nodeID;
        packet->sourceMacH = MacH;
        packet->sourceMacL = MacL;
        packet->destMacH = routingTable[routeIndex].macHigh;
        packet->destMacL = routingTable[routeIndex].macLow;
        
        memcpy(packetBUF, packet, sizeof(DataPacket));
        USART2_printf("%s\r\n",packetBUF);
        }
        // 如果没有到终点的路由（暂时没写，只有链接断了才能不可达终点；后续添加路由查询与路由失败报文）
    }
    // 发送ack数据包
    sendAckPacket(packet->sourceID, packet->sourceMacH, packet->sourceMacL);
    
    // 获取环境噪声RSSI
    // 检查M1是否被成功置为0,M0是否被成功置为1,WOR模式
    if (HAL_GPIO_ReadPin(M0_GPIO_Port, M0_Pin) == 0 && HAL_GPIO_ReadPin(M1_GPIO_Port, M1_Pin) == 0){
        printf("\r\n M1 = 0; M0 = 0 \r\n");
        
        unsigned char cscxRSSIreq2[6] = {0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x01};
        CS_Reg_Send_Data(cscxRSSIreq2, sizeof(cscxRSSIreq2)); // 发送 cscxRSSIreq2 到寄存器
        HAL_Delay(300); // 延迟  毫秒
        unsigned char cscxRSSI[4] = {0x00, 0x00, 0x00, 0x00};
        cstx_reg_Receive_Data(cscxRSSI, &RSSIkey); // 接收寄存器的数据
        printf("\r\n\r\nLORA REG CODE %d REG->", RSSIkey); // 打印 LORA 寄存器代码和寄存器信息
        for (int i = 0; i < 4; i++) // 逐字节打印接收到的寄存器数据
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
    
    // 处理确认包逻辑
}





// 主函数
int main(void)
{
    int i;
    /* MCU Configuration--------------------------------------------------------*/

    HAL_Init(); // 初始化 HAL 库
    
    // 初始化路由表
    // 初始化路由表
    routingTableSize = 2; // 初始路由表大小
    routingTable = (RoutingEntry *)malloc(routingTableSize * sizeof(RoutingEntry));
    
    SystemClock_Config(); // 配置系统时钟

    MX_GPIO_Init(); // 初始化 GPIO 端口
    MX_USART1_UART_Init(); // 初始化 USART1 端口
    MX_USART2_UART_Init(); // 初始化 USART2 端口

    USART_Interupt_Enable(); // 使能串口接收中断和空闲中断

    printf("LORA Board LED OK\r\n"); // 打印 LORA 板的 LED 状态信息
    sprintf(nodeIDStr, "%d", nodeID);
    CS_OLED_Init(); // 初始化 OLED 显示屏
    OLED_Clear(); // 清除 OLED 显示屏内容
    OLED_ShowString(16, 0, "WWSN NODE"); // 在 OLED 显示屏上显示字符串 "WWSN NODE"
    printf("\r\nWWSN Node %d\r\n", nodeID); // 打印nodeID
    OLED_ShowString(100, 0, (unsigned char*)nodeIDStr); // 在 OLED 显示屏上显示节点ID
    cstxInitialize(); // 初始化 LED 并使其闪烁


// 通过配置寄存器 设置lora模块信道与地址 
//    configureModule();
    if (HAL_GPIO_ReadPin(M1_GPIO_Port, M1_Pin) == 1) // 检查 GPIO 引脚 M1 的电平状态，只有在高电平时才开始读取模块的寄存器
    {
        printf("\r\nM1 = 1 ; M0 = 0 Register mode \r\n"); // 打印 M1 和 M0 的状态信息，表示处于寄存器模式
        CS_Reg_Send_Data(cscxReg, sizeof(cscxReg)); // 发送 cscxReg 到寄存器
        HAL_Delay(300); // 延迟 300 毫秒
        printf("\r\n\r\nLORA REG CODE %d UART2->", regConut); // 打印 LORA 寄存器代码和 UART2 信息
        
    for (i = 0; i < 12; i++) // 逐字节打印 USART2 接收缓冲区的数据
    {
        printf("%02X", USART2_RX_BUF[i]);
        printf(" ");
    }
    
    cstx_reg_Receive_Data(csrevReg, &key); // 接收寄存器的数据
    printf("\r\n\r\nLORA REG CODE %d REG->", key); // 打印 LORA 寄存器代码和寄存器信息
    memset(disOLED, 0, 24); // 清空 disOLED 数组
    
    for (i = 0; i < 12; i++) // 逐字节打印接收到的寄存器数据
    {
        printf("%02X", csrevReg[i]);
        printf(" ");
    }
    
    OLED_ShowString(0, 2, "Addr:"); // 在 OLED 显示屏上显示 "Addr:"
    for (i = 3; i < 5; i++) // 显示寄存器的地址信息
    {
        sprintf((char *)disOLED, "%02X", csrevReg[i]);
//        printf("disOLED=%s,", disOLED);
        OLED_ShowString(i * 16 - 4, 2, disOLED);
    }
    
        OLED_ShowString(82, 2, "Ch:"); // 在 OLED 显示屏上显示 "Ch:"
        sprintf((char *)disOLED, "%02X", csrevReg[8]);
//        printf("disOLED=%s\r\n", disOLED);
        OLED_ShowString(104, 2, disOLED);
    }
    else
    {
        printf("\r\nM1 = 0; M0 = 0 Transparent mode \r\n"); // 打印 M1 和 M0 的状态信息，表示处于透明模式
    }

  
  
  
    memset(USART2_RX_BUF, 0, USART_REC_LEN); // 清空接收缓冲区
    USART2_RX_STA = 0;
    /* 无限循环 */
    /* USER CODE BEGIN WHILE */
    // 循环读取传感器数据并发送
    HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET); // 使能模块运行 M1 0
    if (HAL_GPIO_ReadPin(M1_GPIO_Port, M1_Pin) == 0){
        printf("\r\nM1 = 0; M0 = 0 Transparent mode \r\n");
    }
    HAL_Init();  

    uint32_t currentMillis; //获取当前系统时间
    
    addRoutingEntry(nodeID, nodeID, MacH, MacL);
    DataPacket receivedPacket;
    DataPacket datapacket;
    while (1)
    {   

        currentMillis = HAL_GetTick();
        if (currentMillis - previousMillisA0 >= 10000 || previousMillisA0 == 0) //当前时间刻减去前次执行的时间刻
        {
            previousMillisA0 = currentMillis; //更新执行时间刻

//        OLED_ShowString(0, 4, "Send data ......"); // 在 OLED 显示屏上显示 "Send data ......"
        
        if (DHT11_READ_DATA() == 1) // 读取 DHT11 数据
        {
            
            printf("\r\nRead DHT11 Succeed \r\n");
    //        USART2_printf("A,%d.%d%%,%d.%d", Dht11data[0], Dht11data[1], Dht11data[2], Dht11data[3]); // 打印读取到的 DHT11 数据
            datapacket.data[0] = Dht11data[2];
            datapacket.data[1] = Dht11data[3];
            datapacket.data[2] = Dht11data[0];
            datapacket.data[3] = Dht11data[1];
            memset(disOLED, 0, 32); // 清空 disOLED 数组
            sprintf((char *)disOLED,"T:%d.%dC H:%d.%dR",datapacket.data[0],datapacket.data[1],datapacket.data[2],datapacket.data[3]);
            OLED_ShowString(0, 6, disOLED); // 在 OLED 显示屏上显示格式化后的 DHT11 数据
            
            // 查询路由表，查找目标地址的路由
            int routeIndex = findRoute(targetID);
            printf("\r\nRouteIndex: %d\r\n", routeIndex);
            if (routeIndex != -1) {
                // 构建数据缓冲区并发送
                // 发送数据包
                datapacket.destMacH = routingTable[routeIndex].macHigh;
                datapacket.destMacL = routingTable[routeIndex].macLow;
                datapacket.destchanID = channelID; // 信道0
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
    currentMillis = HAL_GetTick(); //获取当前系统时间
    if(currentMillis - previousMillisA1 > 5000 && getRoutReplay == 0){
    sendRouteRequest(targetID);
    previousMillisA1 = HAL_GetTick();
    }
    }
        
    // 处理LORA 发送过来的数据 
    if (USART2_RX_STA == REC_OK) // 检查是否接收到数据
    {
//        printf("\r\nReceive data with protocol:%02X\r\n",  USART2_RX_BUF[5]);
        receivedPacket.sourceMacH = USART2_RX_BUF[0];
        receivedPacket.sourceMacL = USART2_RX_BUF[1];
        receivedPacket.sourceID = USART2_RX_BUF[2];
        receivedPacket.forwardID = USART2_RX_BUF[3];
        receivedPacket.destID = USART2_RX_BUF[4];
        receivedPacket.protocol = USART2_RX_BUF[5];
        memcpy(receivedPacket.data, &USART2_RX_BUF[6], sizeof(receivedPacket) - 6);
        
        
        // 从数据包末尾开始向前遍历，直到找到第一个非零字节
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
        memset(USART2_RX_BUF, 0, USART_REC_LEN); // 清空接收缓冲区
        USART2_RX_STA = 0; // 重置接收状态

      
      
    }
    
    //电脑发送过来的数据(测试用)
    if(USART_RX_STA == REC_OK)
    {
        printf("\r\nReceive Date from PC and Send to Destination\r\n"); 
        USART2_printf("%s\r\n",USART_RX_BUF); //通过lora发送出去
        memset(USART_RX_BUF,0,USART_REC_LEN);
        USART_RX_STA = 0;
        }
    } // while
    
} // main














/**
  * @brief 系统时钟配置
  * @retval 无
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** 配置主内部调节器输出电压
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** 初始化 RCC 振荡器，按照指定参数
  * 在 RCC_OscInitTypeDef 结构中配置。
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
  /** 初始化 CPU, AHB 和 APB 总线时钟
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
  /** 初始化外围设备时钟
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
  * @brief  发生错误时执行的函数。
  * @retval 无
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 用户可以在此处添加自己的代码来报告 HAL 错误返回状态 */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  报告断言错误的源文件名和行号。
  * @param  file: 指向源文件名的指针
  * @param  line: 断言错误行号
  * @retval 无
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* 用户可以在此处添加自己的代码来报告文件名和行号，
     例如：printf("错误的参数值: 文件 %s 第 %d 行\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT CSTX *****END OF FILE****/
