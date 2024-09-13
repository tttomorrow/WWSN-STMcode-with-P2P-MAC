#include "dht11.h"
#include "usart.h"
#include "oled.h"
extern UART_HandleTypeDef huart1;
/**
  * @brief  ��ʪ�ȴ�����������
  * @param  void leigong
  * @retval None
  */
	
uint8_t Dht11data[5] = {0};
	
 
/**
  * @brief  ��ʪ�ȴ����������źŷ���
  * @param  void
  * @retval None
  */
void DHT11_START(void)
{
    DHT11_GPIO_MODE_SET(0);                         //  ��������Ϊ���ģʽ
    
    DHT11_PIN_RESET;                                //  ?�������͵�ƽ
    
    HAL_Delay(25);                                  //  �����ȴ� 18 < ms > 30
    
    DHT11_GPIO_MODE_SET(1);                         //  ��������Ϊ����ģʽ,�ȴ�DHT11��Ӧ
}                                                   //  ��Ϊ��������������,GPIO -> 1
 
/**
  * @brief  ��ȡһλ���� 1bit
  * @param  void
  * @retval 0/1
  */
unsigned char DHT11_READ_BIT(void)
{
    while(!DHT11_READ_IO);                          //  �������ݵĵ͵�ƽ 
    
    Coarse_delay_us(40);
 
    if(DHT11_READ_IO)                               //  ��ʱ�����Ϊ�ߵ�ƽ������Ϊ 1
    {
        while(DHT11_READ_IO);                       //  �������ݵĸߵ�ƽ
        return 1;
    }   
    else                                            //  ����ʱΪ����Ϊ 0
    {
        return 0;
    }
}
 
/**
  * @brief  ��ȡһ���ֽ����� 1byte / 8bit
  * @param  void
  * @retval temp
  */
unsigned char DHT11_READ_BYTE(void)
{
    uint8_t i,temp = 0;                             //  ��ʱ�洢����
    
    for(i=0; i<8 ;i++)
    {
        temp <<= 1;                                 
        if(DHT11_READ_BIT())                        //  1byte -> 8bit
        {
            temp |= 1;                              //  0000 0001
        }
    }
    return temp;
}
 
/**
  * @brief  ��ȡ��ʪ�ȴ��������� 5byte / 40bit
  * @param  void leigong
  * @retval 0/1/2
  */
uint8_t DHT11_READ_DATA(void)
{
		uint8_t State;
    uint8_t i;
//    HAL_Delay(60000);
    DHT11_START();                                  //  �������������ź�
    
    if(DHT11_Check())                               //  ���DHT11Ӧ��     
    {   
				
        while(!DHT11_READ_IO);                      //  ����DHT11���źŵĵ͵�ƽ
        while(DHT11_READ_IO);                       //  ����DHT11���źŵĸߵ�ƽ
      
        for(i=0; i<5; i++)
        {                        
            Dht11data[i] = DHT11_READ_BYTE();            //  ��ȡ 5byte
        }
        
        if(Dht11data[0] + Dht11data[1] + Dht11data[2] + Dht11data[3] == Dht11data[4])
        {
            State =  1;                               //  ����У��ͨ��
        }
        else
        {
            State =  0;                               //  ����У��ʧ��
        }
    }
		return State;
}



 
/**
  * @brief  �����ʪ�ȴ������Ƿ����(���DHT11��Ӧ���ź�)
  * @param  void
  * @retval 0/1
  */
unsigned char DHT11_Check(void)
{
 
    Coarse_delay_us(40);
    if(DHT11_READ_IO == 0)                          //  ��⵽DHT11Ӧ��
    {
        return 1;
    }
    else                                            //  ��⵽DHT11��Ӧ��
    {
        return 0;
    }
}
 
/**
  * @brief  ��������ģʽ
  * @param  mode: 0->out, 1->in
  * @retval None
  */
static void DHT11_GPIO_MODE_SET(uint8_t mode)
{
    if(mode)
    {
        /*  ����  */
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = GPIO_PIN_2;                   //  2������
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;             //  ����ģʽ
        GPIO_InitStruct.Pull = GPIO_PULLUP;                 //  ��������
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else 
    {
        /*  ���  */
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.Pin = GPIO_PIN_2;                //  2������
        GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;      //  Push Pull �������ģʽ
        GPIO_InitStructure.Pull = GPIO_PULLUP;              //  ��������
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;    //  ����
        HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);
    }
}
 
void Coarse_delay_us(uint32_t us)
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
    while (delay--)
	{
		;
	}
}

void PY_Delay_us(uint32_t us)
{
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000000);
	HAL_Delay(us-1);
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
}

