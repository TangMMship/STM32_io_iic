/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "retarget.h"
#define SCL GPIO_PIN_8
#define SDA GPIO_PIN_9

#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))

//IO口输出寄存器地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C
//IO口输入寄存器地址映射
#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08

//IO口操作,只对单一的IO口!
//确保n的值为0~15
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define IIC_SCL    PBout(8) //SCL输出配置  对相应的电器属性的寄存器写值
#define IIC_SDA    PBout(9) //SDA输出配置
#define IIC_readSDA PBin(9) //SDA输入设置  由于是开漏输出  所以不用吧引脚配置为输出模式  开漏的引脚状态是由外设决定，所以可以直接读值
                            //而推挽输出引脚状态由单片机PMOS和NMOS以及上下拉电阻决定了，要配置为输入模式禁止mos管才可以读取外部状态
                            //引脚内部  上部分是TTL施密特触发器，用于读取外部电源 01 ，下部分是pmos，nmos组合 pmos打开为高，nmos打开接地  开漏只用nmos
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);//阻塞方式打印,串口1
    return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IIC_Start();
void IIC_Stop();
void IIC_Sendbyte(uint8_t txb);
uint8_t Wait_ack();
uint8_t IIC_Readbyte(unsigned char ack);
void IIC_Sendack();
void IIC_Nsendack();
void Delay_uS(uint16_t uS_Count)
{
    uint16_t counter = 0;                         /*暂存定时器的计数值*/
    __HAL_TIM_SET_AUTORELOAD(&htim1, uS_Count);   /*设置定时器自动加载值，到该值后重新计数*/
    __HAL_TIM_SET_COUNTER(&htim1, 0);             /*设置定时器初始值*/
    HAL_TIM_Base_Start(&htim1);                   /*启动定时器*/
    while(counter < uS_Count)                     /*直到定时器计数从0计数到uS_Count结束循环,刚好uS_Count uS*/
    {
        counter = __HAL_TIM_GET_COUNTER(&htim1);    /*获取定时器当前计数*/
    }
    HAL_TIM_Base_Stop(&htim1);                    /*停止定时器*/
}

/*



*/
#define	AHT20_INIT_COMD  		0xBE
void bmp280();
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */



  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
    RetargetInit(&huart1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      bmp280();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*IIC开始信号
 * 当SCL为高时 SDA跳变为低
 *
 */
void IIC_Start()
{
    IIC_SDA=1;
    Delay_uS(5);
    IIC_SCL=1;
    //修改了Delay_uS(数  此时延时单位us
    Delay_uS(5);
    IIC_SDA=0;
    Delay_uS(5);   //开始信号
    IIC_SCL=0;            //SCL拉低可以传输信号
}
/*IIC停止信号
 * 当SCL为高时 SDA跳变为高
 *因为正常传输信号是SCL为高时SDA都是保持,所以开始结束SDA在此跳变可以区分
 *要保证SCL为高是SDA先是低再是高，我们不能再SCL为高时改变，不然我们直接先把SDA拉低可能就是开始信号了
 *所以先把SCL，SDA拉低，再把SCL拉高，这样SCL高的时候SDA初始状态就是低，此时再拉高
 */
void IIC_Stop()
{
    IIC_SCL=0;
    IIC_SDA=0;
    Delay_uS(4);
    IIC_SCL=1;
    Delay_uS(2);
    IIC_SDA=1;
    Delay_uS(4);
}

/*等待应答
 *
 *
 */
uint8_t Wait_ack()
{
    uint8_t count=0;   //滤波系数  防止电平跳变的影响
    IIC_SDA=1;       //不用SDA时都要为高阻态，只有传输信号的设备SDA可以拉低，因为是开漏，所以SDA整条线都会拉低,因此一次只能有一个设备发送消息
    Delay_uS(1);
    IIC_SCL=1;       //读数据
    Delay_uS(1);
    while(IIC_SDA)
    {
        count++;
        if(count>250)       //一直没应答
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_SCL=0;  //继续接收
    return 0;
}

/*发送数据
 *
 */
void IIC_Sendbyte(uint8_t txb)
{
    uint8_t i;
    IIC_SCL=0;
    for(i=0;i<8;i++)
    {
        IIC_SDA=(txb&0x80)>>7;      //iic从最高位开始传输，取最高位再移动到第一位给SDA赋值
        txb<<=1;
    }
}

/*
 * 接受数据
 * ack为1发送应答
 * ack为0不应答
 */
uint8_t IIC_Readbyte(unsigned char ack)
{
    uint8_t rebyte,i;
    rebyte=0;
    for(i=0;i<8;i++)
    {
        IIC_SCL=0;
        Delay_uS(2);    //一般iic传输速率100kb，差不多1.25us  高速400kb
        IIC_SCL=1;
        rebyte<<=1;
        rebyte|=IIC_SDA;
        Delay_uS(1);
        if(ack)
        {
            IIC_Sendack();
        }
        else
        {
            IIC_Nsendack();
        }
    }
    return rebyte;
}

/*
 *
 */
void IIC_Sendack()
{
    IIC_SCL=0;
    IIC_SDA=0;                  //拉高SDA产生非应答信号
    Delay_uS(4);
    IIC_SCL=1;
    Delay_uS(4);         //完成应答
    IIC_SCL=0;                  //等待下次信号
}
void IIC_Nsendack()
{
    IIC_SCL=0;
    IIC_SDA=1;                  //拉高SDA产生非应答信号
    Delay_uS(4);
    IIC_SCL=1;
    Delay_uS(4);         //完成应答
    IIC_SCL=0;                  //等待下次信号
}

void bmp280()
{
    uint8_t readb;
    for(int i=0;i<10;i++)
    Delay_uS(41);
    IIC_Start();
    IIC_Sendbyte(0x71);
    Wait_ack();
    readb=IIC_Readbyte(1);
    IIC_Sendbyte(0X70);
    Wait_ack();
    IIC_Sendbyte(AHT20_INIT_COMD );
    Wait_ack();
    HAL_Delay(10);
    readb=IIC_Readbyte(1);
    printf("%x%x",readb,IIC_Readbyte(1));


}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
