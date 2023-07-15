

# 1.外部中断

参考资料：[(60条消息) 【STM32】HAL库 STM32CubeMX教程三----外部中断(HAL库GPIO讲解)_hal_gpio_exti_callback_Z小旋的博客-CSDN博客](https://blog.csdn.net/as480133937/article/details/98983268?spm=1001.2014.3001.5502)

基于正点原子stm32F103 精英版



![image-20230711162848422](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230711162848422.png)



<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230711163110640.png" alt="image-20230711163110640" style="zoom:50%;" />



LED

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230711163207077.png" alt="image-20230711163207077" style="zoom: 50%;" />

GPIO函数库讲解

首先打开stm32f1xx_hal_gpio.h 发现一共定义有8个函数

![image-20230711165157274](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230711165157274.png)



在gpio.c中添加这部分代码

```c++
/* USER CODE BEGIN 2 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin==KEY0_Pin)
  {
  
    if(HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin)==0)
    {
      LED0(0);
      LED1(0);
  
    }
    __HAL_GPIO_EXTI_CLEAR_IT(KEY0_Pin);
  }
  else if(GPIO_Pin==KEY1_Pin)
  {
 
    if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==0)
    {
      LED0(1);
      LED1(1);
    
    }
    __HAL_GPIO_EXTI_CLEAR_IT(KEY1_Pin);


}
  }
```



# 2.定时器中断

主要参考：[(60条消息) 【STM32】HAL库 STM32CubeMX教程六----定时器中断_hal_tim_irqhandler_Z小旋的博客-CSDN博客](https://blog.csdn.net/as480133937/article/details/99201209?spm=1001.2014.3001.5502)

定时器溢出时间：

![image-20230711160408900](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230711160408900.png)

关注函数：

**定时器溢出中断回调函数**

-  **void TIM3_IRQHandler(void)**   首先进入中断函数
-  **HAL_TIM_IRQHandler(&htim2)**;之后进入定时器中断处理函数
- 判断产生的是哪一类定时器中断(溢出中断/PWM中断.....) 和定时器通道
-  **void HAL_TIM_PeriodElapsedCallback(&htim2)**;    进入相对应中断回调函数
  在中断回调函数中添加用户代码
  **在HAL库中，中断运行结束后不会立刻退出，而是会先进入相对应的中断回调函数，处理该函数中的代码之后，才会退出中断，**所以在HAL库中我们一般将中断需要处理代码放在中断回调函数中

在main.c函数中初始化定时器2

```c++
  /* USER CODE BEGIN 2 */
    /*使能定时器1中断*/
    HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */
```



在timer.c中添加溢出中断回调函数

```c++
//定时器溢出中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim2))
    {
        ToggLED1();
    }
}
```



其中在main.h中添加LED灯宏定义

```c++
/* USER CODE BEGIN Private defines */
//写GPIO
#define LED0(n) HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, n?GPIO_PIN_SET:GPIO_PIN_RESET )
#define LED1(n) HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, n?GPIO_PIN_SET:GPIO_PIN_RESET )

//GPIO状态取反-即反转
#define ToggLED0() HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin)
#define ToggLED1() HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)
```



**这里有一个问题是，我尝试在中断函数去添加LED闪烁，但没出现效果**

![image-20230711160921863](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230711160921863.png)



# 3.串口（RS-232)

RS232的电平

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230711204347580.png" alt="image-20230711204347580" style="zoom:33%;" />

RS232的数据帧

<img src="C:/Users/su/Desktop/%E5%B5%8C%E5%85%A5%E5%BC%8F/%E9%80%9A%E4%BF%A1%E5%AE%9E%E9%AA%8C/%E9%80%9A%E4%BF%A1%E7%AC%94%E8%AE%B0/%E5%9B%BE%E7%89%87/image-20230711204546403.png" alt="image-20230711204546403" style="zoom: 67%;" />

启动位：必须占一个位长，保持0电平

有效数据位：可选5、6、7、8、9个位长，LSB(最低有效位)在前，MSB（最高有效位）在后

RX232通信示意图

![image-20230711204633604](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230711204633604.png)



# 4.UART串口通信

usart(universal synchronous asynchronous receiver transmitter)  通用同步异步收发器

uart(universal  asynchronous receiver transmitter) 通用异步收发器

都能与外部设备进行全双工异步通信

参考资料：[(61条消息) 【STM32】HAL库 STM32CubeMX教程四---UART串口通信详解_hal_uart_transmit_Z小旋的博客-CSDN博客](https://blog.csdn.net/as480133937/article/details/99073783?spm=1001.2014.3001.5502)

**通过本篇博客您将学到：**

STM32CubeMX创建串口例程

HAL库UATR函数库

重定义printf函数

HAL库，[UART](https://so.csdn.net/so/search?q=UART&spm=1001.2101.3001.7020)中断接收

HAL库UATR接收与发送例程

精英版串口-串口1

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230713095216322.png" alt="image-20230713095216322" style="zoom:50%;" />

注意：使用串口通信时，**编译需要勾选上Use MicroLIB**

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230712092131571.png" alt="image-20230712092131571" style="zoom:67%;" />

**1、串口发送/接收函数**

```c++
HAL_UART_Transmit();串口发送数据，使用超时管理机制 
HAL_UART_Receive();串口接收数据，使用超时管理机制
HAL_UART_Transmit_IT();串口中断模式发送  
HAL_UART_Receive_IT();串口中断模式接收
HAL_UART_Transmit_DMA();串口DMA模式发送
HAL_UART_Transmit_DMA();串口DMA模式接收

```

**2.串口中断函数**

```c++
HAL_UART_IRQHandler(UART_HandleTypeDef *huart);  //串口中断处理函数
HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);  //串口发送中断回调函数
HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);  //串口发送一半中断回调函数（用的较少）
HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);  //串口接收中断回调函数
HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);//串口接收一半回调函数（用的较少）
HAL_UART_ErrorCallback();串口接收错误函数

```

对串口打印进行调试

在usart.c中重写fputc和fgetc函数

**在usart.h需要包含#include<stdio.h>**

```c++
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
int fgetc(FILE * f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1,&ch, 1, 0xffff);
  return ch;
}

```

在main.c中

```c++
   printf("大只测试\n");
    sprintf(str,"今天是个好日子,20%02d-%02d-%02d",15,10,04);  
  /* 调用格式化输出函数打印输出数据 */
  printf("%s\n",str);
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    ch=getchar();
    HAL_UART_Transmit(&huart1,&ch,1,0);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
```



**UART接收中断**
因为中断接收函数只能触发一次接收中断，所以我们需要在中断回调函数中再调用一次中断接收函数

**具体流程：**
1、初始化串口

2、在main中第一次调用接收中断函数

3、进入接收中断，接收完数据  进入中断回调函数

4、修改HAL_UART_RxCpltCallback中断回调函数，处理接收的数据，

5  回调函数中要调用一次HAL_UART_Receive_IT函数，使得程序可以重新触发接收中断

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230712094805765.png" alt="image-20230712094805765" style="zoom:50%;" />

**函数流程图：**

HAL_UART_Receive_IT(启动接收中断接收函数)    ->  USART2_IRQHandler(void)(中断服务函数)    ->    HAL_UART_IRQHandler(UART_HandleTypeDef *huart)(中断处理函数)    ->    UART_Receive_IT(UART_HandleTypeDef *huart) (接收函数)   ->    HAL_UART_RxCpltCallback(huart);(中断回调函数)

HAL_UART_RxCpltCallback函数就是用户要重写在main.c里的回调函数。



**具体而言：当我们在XCOM中发送一个字符时，就会触发USART2_IRQHandler(void)(中断服务函数)**,进而调用有我们编写的中断回调函数

![image-20230712100357610](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230712100357610.png)

main主程序

```c++
int main(void)
{

  uint8_t ch;
  HAL_Init();

  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&c, 1);
  printf("请输入一个英文字符：\r\n\r\n");
  while (1)
  {

    if(flag==1){
			//发送信息
			HAL_UART_Transmit(&huart1, (uint8_t *)&message, strlen(message),0xFFFF); 
			
			//延时
			HAL_Delay(1000);

  }

}
        
    /* 串口数据接收完成回调函数 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  //当输入的指令为0时,发送提示并改变flag
	if(c=='0'){
		flag=0;
		HAL_UART_Transmit(&huart1, (uint8_t *)&tips2, strlen(tips2),0xFFFF); 
	}
	
	//当输入的指令为1时,发送提示并改变flag
	else if(c=='1'){
		flag=1;
		HAL_UART_Transmit(&huart1, (uint8_t *)&tips1, strlen(tips1),0xFFFF); 
	}


	//重新设置中断
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&c, 1); 

}
```



# 5.看门狗（独立看门狗，窗口看门狗）

参考文档：[(61条消息) 【STM32】HAL库 STM32CubeMX教程五----看门狗(独立看门狗,窗口看门狗)_stm32看门狗_Z小旋的博客-CSDN博客](https://blog.csdn.net/as480133937/article/details/99121645?spm=1001.2014.3001.5502)

**前言：看门狗可以有效解决程序的跑飞，在使用过程中比较常见，是防止芯片故障的有效外设**

简单说：**看门狗的本质就是定时计数器**，计数器使能之后一直在累加 而喂狗就是重新写入计数器的值，定时计数器重新累加，

如果在一定时间内没有接收到喂狗信号（表示MCU已经挂了），便实现处理器的自动复位重启（发送复位信号）.**两个看门狗设备（独立看门狗、窗口看门狗）**可以用来检测和解决由软件错误引起的故障。**当计数器达到给定的超时值时，触发一个中断**（仅适用窗口看门狗）或者产生**系统复位。**

- **独立看门狗**（IWDG)由专用的低速时钟（LSI）驱动（40kHz），即使主时钟发生故障它仍有效。独立看门狗适合应用于需要看门狗作为一个在主程序之外 能够完全独立工作，并且对时间精度要求低的场合。
- **窗口看门狗**由从APB1时钟（36MHz）分频后得到时钟驱动。通过可配置的时间窗口来检测应用程序非正常的过迟或过早操作。  窗口看门狗最适合那些要求看门狗在精确计时窗口起作用的程序。

## 5.1 独立看门狗

 IWDG时钟预分频系数 4分频  

 计数器重装载值 4095  **RLR**

 LSI时钟频率：40KHZ

   **超出（溢出）时间计算：**

​                                                                                  **Tout=((4×2^PRER) ×RLR)/LSI时钟频率**

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230712103835745.png" alt="image-20230712103835745" style="zoom:33%;" />



**HAL库独立窗口狗函数库讲解：**

**看门狗初始化：**

```c++
HAL_IWDG_Init(IWDG_HandleTypeDef *hiwdg)
```

**喂狗函数：**

```c++
HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg)
```

现象：系统不断重启，LED闪烁

main函数

![image-20230712120558496](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230712120558496.png)



iwdg.c

```c++
/* IWDG init function */
void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

void iwdg_feed(void)
{
    HAL_IWDG_Refresh(&hiwdg);  /* 重装载计数器 */
}
/* USER CODE END 1 */
```



## 5.2 窗口看门狗

**窗口看门狗的定义**
窗口看门狗跟独立看门狗一样，也是一个递减计数器不断的往下递减计数，当减到一个固定值 0x3F 时还不喂狗的话，产生复位，这个值叫窗口的下限，是固定的值，不能改变。

窗口看门狗之所以称为窗口，就是因为**其喂狗时间是在一个有上下限的范围内（计数器减到某个值~计数器减到0x3F），在这个范围内才可以喂狗，可以通过设定相关寄存器，设定其上限时间（但是下限是固定的0x3F）**




















**看门狗初始化：**

```C++
HAL_WWDG_Init(WWDG_HandleTypeDef *hwwdg)
```

**喂狗：**

```c++
HAL_WWDG_Refresh(WWDG_HandleTypeDef *hwwdg)
```

**看门狗中断处理函数：**

```c++
HAL_WWDG_IRQHandler(WWDG_HandleTypeDef *hwwdg)
```

功能： 判断中断是否正常，并进入中断回调函数

**看门狗中断回调函数**：

```c++
 __weak HAL_WWDG_EarlyWakeupCallback(hwwdg);
```

**在HAL库中，每进行完一个中断，并不会立刻退出，而是会进入到中断回调函数中，**我们主要在这部分函数中添加功能来查看效果





# 6.GPIO八种模式及工作原理详解

参考文档：[(61条消息) 【STM32】STM32F4 GPIO八种模式及工作原理详解_stm32 ft引脚_Z小旋的博客-CSDN博客](https://blog.csdn.net/as480133937/article/details/98063549?spm=1001.2014.3001.5502)

## 6.1 **GPIO的复用**

STM32F4 有很多的内置外设，这些外设的外部引脚都是与 GPIO 共用的。也就是说，一个引脚可以有很多作用，但是默认为IO口，如果想使用一个 GPIO内置外设的功能引脚，就需要GPIO的复用，**那么当这个 GPIO 作为内置外设使用的时候，就叫做复用。    比如说串口  就是GPIO复用为串口**

## 6.2 GPIO的工作模式

1、4种输入模式

**（1）GPIO_Mode_IN_FLOATING 浮空输入**
**（2）GPIO_Mode_IPU 上拉输入**
**（3）GPIO_Mode_IPD 下拉输入**
**（4）GPIO_Mode_AIN 模拟输入**

2、4种输出模式 

**（5）GPIO_Mode_Out_OD 开漏输出（带上拉或者下拉）**
**（6）GPIO_Mode_AF_OD 复用开漏输出（带上拉或者下拉）**
**（7）GPIO_Mode_Out_PP 推挽输出（带上拉或者下拉）**
**（8）GPIO_Mode_AF_PP 复用推挽输出（带上拉或者下拉）**
**3、4种最大输出速度**
**（1）2MHZ  (低速)**
**（2）25MHZ  (中速)**
**（3）50MHZ  (快速)**
**（4）100MHZ  (高速)**



# 7.PWM 脉冲宽度调制

参考资料：[(61条消息) 【STM32】HAL库 STM32CubeMX教程七---PWM输出(呼吸灯)_stm32 hal pwm输出_Z小旋的博客-CSDN博客](https://blog.csdn.net/as480133937/article/details/99231677?spm=1001.2014.3001.5502)



pwm控制电机，就是占空比愈大，那么电机的速度就越快

## PWM工作原理

**SMT32F1系列**共有8个定时器：

**高级定时器（TIM1、TIM8）；通用定时器（TIM2、TIM3、TIM4、TIM5）；基本定时器（TIM6、TIM7）**

**STM32的每个通用定时器都有独立的4个通道可以用来作为：输入捕获、输出比较、PWM输出、单脉冲模式输出等。**

**STM32的定时器除了TIM6和TIM7（基本定时器）之外，其他的定时器都可以产生PWM输出。其中，高级定时器TIM1、TIM8可以同时产生7路PWM输出**

**原理讲解：**

**下图为向上计数模式：**

![image-20230712152333168](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230712152333168.png)

- 在PWM输出模式下，除了CNT（计数器当前值）、ARR（自动重装载值）之外，还多了一个值CCRx（捕获/比较寄存器值）。
- **当CNT小于CCRx时，TIMx_CHx通道输出低电平**；
- **当CNT等于或大于CCRx时，TIMx_CHx通道输出高电平**。

**PWM的一个周期**

- **定时器从0开始向上计数**
- **当0-t1段,定时器计数器TIMx_CNT值小于CCRx值,输出低电平**
- **t1-t2段,定时器计数器TIMx_CNT值大于CCRx值,输出高电平**
- **当TIMx_CNT值达到ARR时,定时器溢出,重新向上计数...循环此过程**
- **至此一个PWM周期完成**



**总结：**

**每个定时器有四个通道,每一个通道都有一个捕获比较寄存器,** 

**将寄存器值和计数器值比较,通过CCRx比较结果输出高低电平,便可以实现脉冲宽度调制模式（PWM信号）**

**TIMx_ARR寄存器确定PWM频率，**

**TIMx_CCRx寄存器确定占空比**



## 计算

在 Parameter Settings 页配置预分频系数为 71，计数周期(自动加载值)为 499，定时器溢出频率，即PWM的周期，就是 
$$
72MHz/(71+1)/(499+1) = 2kHz
$$
PWM频率：

$$
Fpwm =Tclk / ((arr+1)*(psc+1))(单位：Hz)
$$
**arr 是计数器值**
**psc 是预分频值**
占空比：
$$
duty circle = TIM3->CCR1 / arr(单位：%)
$$
**TIM3->CCR1  用户设定值**
比如  定时器频率Tclk = 72Mhz  arr=499   psc=71     那么PWM频率就是720000/500/72=  2000Hz，

即2KHz，arr=499,TIM3->CCR1=250     则pwm的占空比为50%  

**改CCR1可以修改占空比，修改arr可以修改频率**



**重要库函数：**

使能TIM3的PWM Channel1 输出。

```c++
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
```

修改pwm占空比

```c++
__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwmVal);    //修改比较值，修改占空比
//TIM3->CCR1 = pwmVal;    与上方相同
```

在main函数实现

```c++
int main(void)
{
   //呼吸灯例程，需要把正点原子PB5与PA6连在一起
  
    uint16_t pwmVal=0;   //PWM占空比  
    uint8_t dir=1;
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  while (1)
  {
    /* USER CODE END WHILE */
    while (pwmVal< 500)
	  {
		  pwmVal++;
		  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwmVal);    //修改比较值，修改占空比
//		  TIM3->CCR1 = pwmVal;    与上方相同
		  HAL_Delay(1);
	  }
	  while (pwmVal)
	  {
		  pwmVal--;
		  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwmVal);    //修改比较值，修改占空比
//		  TIM3->CCR1 = pwmVal;     与上方相同
		  HAL_Delay(1);
	  }
	  HAL_Delay(200);

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
```

# 8.定时器输入捕获

参考资料：[(61条消息) 【STM32】HAL库 STM32CubeMX教程八---定时器输入捕获_hal_tim_readcapturedvalue_Z小旋的博客-CSDN博客](https://blog.csdn.net/as480133937/article/details/99407485?spm=1001.2014.3001.5502)

**输入捕获概念**

输入捕获模式可以用来测量脉冲宽度或者测量频率。STM32的定时器，除了TIM6、TIM7，其他的定时器都有输入捕获的功能。







# 9.ADC

参考资料：[(61条消息) 【STM32】HAL库 STM32CubeMX教程九---ADC_stm32cubemx adc_Z小旋的博客-CSDN博客](https://blog.csdn.net/as480133937/article/details/99627062?spm=1001.2014.3001.5502)



## 9.1**什么是ADC**

Analog-to-Digital Converter的缩写。指模/数转换器或者模拟/数字转换器。是**指将连续变量的模拟信号转换为离散的数字信号的器件**。

**典型的模拟数字转换器将模拟信号转换为表示一定比例电压值的数字信号。**

简单地说就是将模拟电压值，转换成对应的肉眼可读数值


**12位ADC是一种逐次逼近型模拟数字转换器。**它有，**3个ADC控制器，多达18个通道**，可测量**16个外部和2个内部信号源**。各通道的A/D转换可以单次、连续、扫描或间断模式执行。ADC的结果可以左对齐或右对齐方式存储在16位数据寄存器中。

**12位模拟数字转换器**
就是ADC的数字存储是12位的 也就是说转换器通过采集转换所得到的最大值是4095 “111111111111”=4095 二进制的12位可表示0-4095个数， 对应着所测电压的实际值，转换的电压范围是0v-3.3v的话，转换器就会把0v-3.3v平均分成4096份。设转换器所得到的值为x，所求电压值为y。
$$
y=(x/4096)*3.3V
$$
同理，可以理解8位精度和10位精度,具体的转压范围下面我们会讲



**3个ADC控制器**

ADC1,ADC2,ADC3

**18个通道**

**16个外部通道和2个内部信号源** 具体是哪一个IO 口可以从手册查询到

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230712162145099.png" alt="image-20230712162145099" style="zoom:50%;" />



**16个外部通道**：芯片上有16个引脚是可以接到模拟电压上进行电压值检测的

**2个内部信号源** ： 一个是内部温度传感器，一个是内部参考电压

一共支持**23个引脚**支持ADC，包括21个外部和2个内部信号源



## ADC的转换模式

可看参考资料：



## 左对齐或右对齐

因为ADC得到的数据是12位精度的，但是数据存储在 16 位数据寄存器中，所以ADC的存储结果可以分为**左对齐或右对齐**方式（12位）

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230712164107549.png" alt="image-20230712164107549" style="zoom:50%;" />



## ADC工作内容

**1.电压输入范围**

![image-20230712164520823](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230712164520823.png)

**ADC一般用于采集小电压，其输入值不能超过VDDA**，即ADC输入范围：VREF- ≤ VIN ≤ VREF+。具体的定义见上图。
**一般把VSSA和VREF- 接地， VREF+ 和 VDDA接3V3，那么ADC的输入范围是0~3.3V。**

**2.ADC输入通道**
从**ADCx_INT0-ADCx_INT15** 对应三个ADC的16个外部通道，进行模拟信号转换 此外，还有**两个内部通道：温度检测或者内部电压检测**
选择对应通道之后，便会选择对应GPIO引脚，相关的引脚定义和描述可在开发板的数据手册里找

**3.注入通道，规则通道**
我们看到，在选择了ADC的相关通道引脚之后，在模拟至数字转换器中有两个通道，注入通道，规则通道，**规则通道至多16个，注入通道至多4个**

**规则通道：**

规则通道相当于你正常运行的程序，看它的名字就可以知道，很规矩，就是正常执行程序
**注入通道：**
注入通道可以打断规则通道，听它的名字就知道不安分，如果在规则通道转换过程中，有注入通道进行转换，那么就要先转换完注入通道，等注入通道转换完成后，再回到规则通道的转换流程
![ ](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230712164727440.png)   

可以简单地把注入通道理解为中断形式，可以更好理解



**4.ADC时钟**
图中的**ADC预分频器的ADCCLK是ADC模块的时钟来源**。通常，由时钟控制器提供的**ADCCLK时钟和PCLK2（APB2时钟）同步**。RCC控制器为ADC时钟提供一个专用的可编程预分频器。 分频因子由RCC_CFGR的ADCPRE[1:0]配置，可配置2/4/6/8分频

**STM32的ADC最大的转换速率为1MHz,也就是说最快转换时间为1us，为了保证ADC转换结果的准确性，ADC的时钟最好不超过14M**。
$$
T = 采样时间 + 12.5个周期，其中1周期为1/ADCCLK
$$
例如，当 ADCCLK=14Mhz 的时候，并设置 1.5 个周期的采样时间，则得到： Tcovn=1.5+12.5=14 个周期=1us。



**5.外部触发转换**
**ADC 转换可以由ADC 控制寄存器2: ADC_CR2 的ADON 这个位来控制，写1 的时候开始转换，写0 的时候停止转换**

当然，除了ADC_CR2寄存器的ADON位控制转换的开始与停止，还可以支持**外部事件触发转换（比如定时器捕捉、EXTI线）**

包括**内部定时器触发**和**外部IO触发**。具体的触发源由**ADC_CR2的EXTSEL[2:0]位（规则通道触发源 ）和 JEXTSEL[2:0]位（注入通道触发源）**控制。

同时ADC3的触发源与ADC1/2的触发源有所不同，上图已经给出，

具体查看第五部分框图即可理解


### 6中断

中断触发条件有三个，**规则通道转换结束**，**注入通道转换结束**，或者**模拟看门狗状态位被设置**时都能产生中断，

![image-20230712165745161](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230712165745161.png)

转换结束中断就是正常的ADC完成一次转换，进入中断，这个很好理解

**模拟看门狗中断**
当被ADC转换的模拟电压值低于低阈值或高于高阈值时，便会产生中断。阈值的高低值由ADC_LTR和ADC_HTR配置
模拟看门狗，听他的名字就知道，在ADC的应用中是为了防止读取到的电压值超量程或者低于量程



**DMA**
同时ADC还支持DMA触发，规则和注入通道转换结束后会产生DMA请求，用于将转换好的数据传输到内存。

注意，只有ADC1和ADC3可以产生DMA请求

因为涉及到DMA传输，所以这里我们不再详细介绍，之后几节会更新DMA,一般我们在使用ADC 的时候都会开启DMA 传输。



## ADC主要特征

![image-20230712165919835](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230712165919835.png)

## ADC Hal库函数

开启ADC 3种模式 ( 轮询模式 中断模式 DMA模式 ）

**• HAL_ADC_Start(&hadcx);       //轮询模式开启ADC**
**• HAL_ADC_Start_IT(&hadcx);      //中断轮询模式开启ADC**
**• HAL_ADC_Start_DMA(&hadcx)；       //DMA模式开启ADC**

关闭ADC 3种模式 ( 轮询模式 中断模式 DMA模式 ）

**• HAL_ADC_Stop()**
**• HAL_ADC_Stop_IT()**
**• HAL_ADC_Stop_DMA()**

ADC校准函数 ：

• **HAL_ADCEx_Calibration_Start(&hadcx);**   

读取ADC转换值

**• HAL_ADC_GetValue()**

等待转换结束函数

**• HAL_ADC_PollForConversion(&hadc1, 50);**

第一个参数为那个ADC,第二个参数为最大等待时间

ADC中断回调函数
**• HAL_ADC_ConvCpltCallback()**

转换完成后回调，DMA模式下DMA传输完成后调用

规则通道及看门狗配置

**• HAL_ADC_ConfigChannel() 配置规则组通道**
**• HAL_ADC_AnalogWDGConfig(）**

例程：

在main.c中加上

```c++
  /* USER CODE BEGIN 0 */
	uint16_t ADC_Value;
  /* USER CODE END 0 */
```

在ADC初始化后加入AD校准函数

```c++
  MX_ADC1_Init();
  HAL_ADCEx_Calibration_Start(&hadc1);    //AD校准
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
```

while中加上：

```c++
 HAL_ADC_Start(&hadc1);     //启动ADC转换
 HAL_ADC_PollForConversion(&hadc1, 50);   //等待转换完成，50为最大等待时间，单位为ms
 
 
 if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
 {
  ADC_Value = HAL_ADC_GetValue(&hadc1);   //获取AD值

  printf("ADC1 Reading : %d \r\n",ADC_Value);
  printf("PA3 True Voltage value : %.4f \r\n",ADC_Value*3.3f/4096);
  printf("大只测试\r\n");
}
HAL_Delay(1000);
```



**中断读取**

如果使能了ADC转换结束中断，并且使能了定时器中断，可以这样写：

```c++
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)    //定时器中断回调
{
    HAL_ADC_Start_IT(&hadc1); //定时器中断里面开启ADC中断转换，1ms开启一次采集    
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)    //ADC转换完成回调
{
    HAL_ADC_Stop_IT(&hadc1);    　　　　//关闭ADC
    HAL_TIM_Base_Stop_IT(&htim3);　　  //关闭定时器
    AD_Value=HAL_ADC_GetValue(&hadc1);　　//获取ADC转换的值

    printf("ADC1 Reading : %d \r\n",AD_Value);
    printf("%.4f V\r\n",(AD_Value*3.3/4096));　　　  //串口打印电压信息
    HAL_TIM_Base_Start_IT(&htim3); 　　　　  //开启定时器
}

```



# 10.DAC

参考资料：[(62条消息) 【STM32】HAL库 STM32CubeMX教程十---DAC_Z小旋的博客-CSDN博客](https://blog.csdn.net/as480133937/article/details/102309242?spm=1001.2014.3001.5502)

Digital-to-Analog Converter的缩写。**数模转换器**。又称D/A转换器，简称DAC，是**指将离散的数字信号转换为连续变量的模拟信号的器件**。

典型的数字模拟转换器**将表示一定比例电压值的数字信号转换为模拟信号**。

STM32F1中有两个DAC,可以同时使用STM32的DAC模块是12位数字输入，电压输出型的DAC。

DAC 有两个用途：**输出波形**和**输出固定电压**




# 11. IIC通信（EEPROM)

参考资料：[(62条消息) 【STM32】HAL库 STM32CubeMX教程十二---IIC(读取AT24C02 )_cubemx iic_Z小旋的博客-CSDN博客](https://blog.csdn.net/as480133937/article/details/105259075?spm=1001.2014.3001.5502)

**IIC(Inter－Integrated Circuit)总线**是一种由NXP（原PHILIPS）公司开发的两线式串行总线，用于连接微控制器及其外围设备。多用于主控制器和从器件间的主从通信，在小数据量场合使用，传输距离短，任意时刻只能有一个主机等特性。

在 CPU 与被控 IC 之间、IC 与 IC 之间进行双向传送，高速 IIC 总线一般可达 400kbps 以上。

PS： **这里要注意IIC是为了与低速设备通信而发明的，所以IIC的传输速率比不上SPI**



## 11.1IIC的物理层

**IIC一共有只有两个总线： 一条是双向的数据线ＳＤＡ，一条是串行时钟线ＳＣＬ**

所有接到I2C总线设备上的串行数据SDA都接到总线的SDA上，各设备的时钟线SCL接到总线的SCL上。I2C总线上的每一个设备都对应一个唯一的地址。

![image-20230712191539121](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230712191539121.png)

**IIC起始信号和终止信号：**

- **起始信号**：SCL保持高电平，SDA由高电平变为低电平后，延时(>4.7us)，SCL变为低电平。

- **停止信号**：SCL保持高电平。SDA由低电平变为高电平。

- <img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230712192258171.png" alt="image-20230712192258171" style="zoom:50%;" />

  **数据有效性**
  **IIC信号在数据传输过程中，当SCL=1高电平时，数据线SDA必须保持稳定状态，不允许有电平跳变，只有在时钟线上的信号为低电平期间，数据线上的高电平或低电平状态才允许变化。**

  **SCL=1时 数据线SDA的任何电平变换会看做是总线的起始信号或者停止信号。**

  也就是在IIC传输数据的过程中，SCL时钟线会频繁的转换电平，以保证数据的传输

  <img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230712192703019.png" alt="image-20230712192703019" style="zoom:50%;" />

**应答信号**
每当主机向从机发送完一个字节的数据，主机总是需要等待从机给出一个应答信号，以确认从机是否成功接收到了数据，

**应答信号：主机SCL拉高，读取从机SDA的电平，为低电平表示产生应答**

**应答信号为低电平时，规定为有效应答位（ACK，简称应答位），表示接收器已经成功地接收了该字节；**
**应答信号为高电平时，规定为非应答位（NACK），一般表示接收器接收该字节没有成功。**
<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230712192846815.png" alt="image-20230712192846815" style="zoom:50%;" />

**每发送一个字节**（8个bit）在一个字节传输的8个时钟后的第九个时钟期间，接收器接收数据后必须回一个ACK应答信号给发送器，这样才能进行数据传输。

应答出现在每一次主机完成8个数据位传输后紧跟着的时钟周期，低电平0表示应答，1表示非应答，

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230713091247362.png" alt="image-20230713091247362" style="zoom:50%;" />



精英版

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230713095701789.png" alt="image-20230713095701789" style="zoom:50%;" />

芯片的寻址：
AT24C设备地址为如下，前四位固定为1010，A2~A0为由管脚电平。AT24CXX EEPROM Board模块中默认为接地。所以A2~A0默认为000，最后一位表示读写操作。所以AT24Cxx的读地址为0xA1,写地址为0xA0。

也就是说如果是
**写24C02的时候，从器件地址为10100000（0xA0）；**
**读24C02的时候，从器件地址为10100001（0xA1）。**

## 11.2 IIC的HAL库函数

在i2c.c文件中可以看到IIC初始化函数。在stm32f1xx_hal_i2c.h头文件中可以看到I2C的操作函数。分别对应**轮询，中断和DMA三种控制方式**

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230713101804188.png" alt="image-20230713101804188"  />

上**面的函数看起来多，但是只是发送和接收的方式改变了，函数的参数和本质功能并没有改变**比方说IIC发送函数 还是发送函数，只不过有普通发送，DMA传输，中断 的几种发送模式

这里我们仅介绍下普通发送，其他的只是改下函数名即可



**IIC写函数**

```c++
 HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
```

功能：**IIC写数据**
参数：

- ***hi2c** 设置使用的是那个IIC 例：&hi2c2
- **DevAddress** 写入的地址 设置写入数据的地址 例 0xA0
- ***pData** 需要写入的数据
- **Size** 要发送的字节数
- **Timeout** **最大传输时间**，超过传输时间将自动退出传输函数



**IIC读函数**

```c++
HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
```

举例：

```c++
HAL_I2C_Master_Transmit(&hi2c1,0xA1,(uint8_t*)TxData,2,1000) ；
```



**发送两个字节数据**

**IIC写数据函数**

```c++
HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
```

功能： IIC写多个数据 该函数适用于IIC外设里面还有子地址寄存器的设备，比方说E2PROM,除了设备地址，每个存储字节都有其对应的地址

参数：

- *hi2c： I2C设备号指针，设置使用的是那个IIC 例：&hi2c2

- DevAddress： 从设备地址 从设备的IIC地址 例E2PROM的设备地址 0xA0

- MemAddress： 从机寄存器地址 ，每写入一个字节数据，地址就会自动+1

- MemAddSize： 从机寄存器地址字节长度 8位或16位

- 写入数据的字节类型 8位还是16位
  I2C_MEMADD_SIZE_8BIT
  I2C_MEMADD_SIZE_16BIT

- ***pData：** **需要写入的的数据的起始地址**
- **Size：** 传输数据的大小 多少个字节
- **Timeout：** **最大读取时间**，超过时间将自动退出函数



使用**HAL_I2C_Mem_Write**等于先使用**HAL_I2C_Master_Transmit**传输第一个寄存器地址，再用**HAL_I2C_Master_Transmit**传输写入第一个寄存器的数据。可以传输多个数据

```c++
void Single_WriteI2C(uint8_t REG_Address,uint8_t REG_data)
{
    uint8_t TxData[2] = {REG_Address,REG_data};
    while(HAL_I2C_Master_Transmit(&hi2c1,I2C1_WRITE_ADDRESS,(uint8_t*)TxData,2,1000) != HAL_OK)
    {
        if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
                {
                  Error_Handler();
                }
    }
}
```

**在传输过程，寄存器地址和源数据地址是会自加的。**

至于读函数也是如此，因此用HAL_I2C_Mem_Write和HAL_I2C_Mem_Read，来写读指定设备的指定寄存器数据是十分方便的，让设计过程省了好多步骤。

**举例：**

**8位：**

```c++
HAL_I2C_Mem_Write(&hi2c2, ADDR, i, I2C_MEMADD_SIZE_8BIT,&(I2C_Buffer_Write[i]),8, 1000);

HAL_I2C_Mem_Read(&hi2c2, ADDR, i, I2C_MEMADD_SIZE_8BIT,&(I2C_Buffer_Write[i]),8, 1000);
```

16位

```c++
HAL_I2C_Mem_Write(&hi2c2, ADDR, i, I2C_MEMADD_SIZE_16BIT,&(I2C_Buffer_Write[i]),8, 1000);

HAL_I2C_Mem_Read(&hi2c2, ADDR, i, I2C_MEMADD_SIZE_16BIT,&(I2C_Buffer_Write[i]),8, 1000);
```

**如果只往某个外设中写数据，则用Master_Transmit。　**

**如果是外设里面还有子地址，例如我们的E2PROM，有设备地址，还有每个数据的寄存器存储地址。则用Mem_Write。Mem_Write是2个地址，Master_Transmit只有从机地址**.





## **实验：**

在实现串口通信后（实现步骤这里就不赘述了）

main.c

main函数之前声明

```c++
/* USER CODE BEGIN PV */
#include <string.h>

#define ADDR_24LCxx_Write 0xA0
#define ADDR_24LCxx_Read 0xA1
#define BufferSize 256
uint8_t WriteBuffer[BufferSize],ReadBuffer[BufferSize];
uint16_t i;
/* USER CODE END PV */
```



```c++
  /* USER CODE BEGIN 2 */
	for(i=0; i<256; i++)
    WriteBuffer[i]=i;    /* WriteBuffer init */


		printf("\r\n***************I2C Example Z小旋测试*******************************\r\n");
			for (int j=0; j<32; j++)
        {
                if(HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, 8*j, I2C_MEMADD_SIZE_8BIT,WriteBuffer+8*j,8, 1000) == HAL_OK)
                {
                                printf("\r\n EEPROM 24C02 Write Test OK \r\n");
                        HAL_Delay(20);
                }
                else
                {
                         HAL_Delay(20);
                                printf("\r\n EEPROM 24C02 Write Test False \r\n");
                }
		}
		/*
		// wrinte date to EEPROM   如果要一次写一个字节，写256次，用这里的代码
		for(i=0;i<BufferSize;i++)
		{
		    HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, i, I2C_MEMADD_SIZE_8BIT,&WriteBuffer[i],1，0xff);//使用I2C块读，出错。因此采用此种方式，逐个单字节写入
		  HAL_Delay(5);//此处延时必加，与AT24C02写时序有关
		}
		printf("\r\n EEPROM 24C02 Write Test OK \r\n");
		*/

		HAL_I2C_Mem_Read(&hi2c1, ADDR_24LCxx_Read, 0, I2C_MEMADD_SIZE_8BIT,ReadBuffer,BufferSize, 0xff);

		for(i=0; i<256; i++)
			printf("0x%02X  ",ReadBuffer[i]);
			
  /* USER CODE END 2 */
```

**注意事项：**

- **AT24C02的IIC每次写之后要延时一段时间才能继续写 每次写之后要delay 5ms左右** 不管硬件IIC采用何种形式（DMA，IT），都要确保两次写入的间隔大于5ms;
- 读写函数最后一个超时调整为1000以上 因为我们一次写8个字节，延时要久一点
- AT24C02页写入只支持8个byte，所以需要分32次写入。这不是HAL库的bug，而是AT24C02的限制，其他的EEPROM可以支持更多byte的写入。
- 当然，你也可以每次写一个字节，分成256次写入，也是可以的 那就用注释了的代码即可

- **注意读取AT24C02数据的时候延时也要久一点，否则会造成读的数据不完整**



# 12.SPI通信(FLASH)

参考资料：[(68条消息) 【STM32】HAL库 STM32CubeMX教程十四---SPI_cubemx spi_Z小旋的博客-CSDN博客](https://blog.csdn.net/as480133937/article/details/105849607?spm=1001.2014.3001.5502)

## **什么是SPI**

SPI 是英语**Serial Peripheral interface**的缩写，顾名思义就是**串行外围设备接口**。是Motorola(摩托罗拉)首先在其MC68HCXX系列处理器上定义的。

SPI，是一种高速的，全双工，同步的通信总线，并且在芯片的管脚上只占用四根线，节约了芯片的管脚，同时为PCB的布局上节省空间，提供方便，主要应用在 EEPROM，FLASH，实时时钟，AD转换器，还有数字信号处理器和数字信号解码器之间。



**SPI主从模式**
SPI分为主、从两种模式，一个SPI通讯系统需要包含一个（且只能是一个）主设备，一个或多个从设备。提供时钟的为主设备（Master），接收时钟的设备为从设备（Slave），SPI接口的读写操作，都是由主设备发起。当存在多个从设备时，通过各自的片选信号进行管理。

**SPI是全双工且SPI没有定义速度限制，一般的实现通常能达到甚至超过10 Mbps**



**SPI信号线**
SPI接口一般使用四条信号线通信：
SDI（数据输入），SDO（数据输出），SCK（时钟），CS（片选）

**MISO： 主设备输入/从设备输出引脚。该引脚在从模式下发送数据，在主模式下接收数据。**
**MOSI： 主设备输出/从设备输入引脚。该引脚在主模式下发送数据，在从模式下接收数据。**
**SCLK：串行时钟信号，由主设备产生。**
**CS/SS：从设备片选信号，由主设备控制。它的功能是用来作为“片选引脚”，也就是选择指定的从设备，让主设备可以单独地与特定从设备通讯，避免数据线上的冲突。**

**SPI一对一**

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230713114617872.png" alt="image-20230713114617872" style="zoom:50%;" />



**SPI一对多**

<img src="C:\Users\su\AppData\Roaming\Typora\typora-user-images\image-20230713114643967.png" alt="image-20230713114643967" style="zoom:50%;" />

**SPI数据发送接收**
SPI主机和从机都有一个串行移位寄存器，主机通过向它的SPI串行寄存器写入一个字节来发起一次传输。

1. **首先拉低对应SS信号线，表示与该设备进行通信**
2. **主机通过发送SCLK时钟信号，来告诉从机写数据或者读数据，这里要注意，SCLK时钟信号可能是低电平有效，也可能是高电平有效，因为SPI有四种模式，这个我们在下面会介绍**
3. **主机(Master)将要发送的数据写到发送数据缓存区(Menory)，缓存区经过移位寄存器(0~7)，串行移位寄存器通过MOSI信号线将字节一位一位的移出去传送给从机，，同时MISO接口接收到的数据经过移位寄存器一位一位的移到接收缓存区。**
4. **从机(Slave)也将自己的串行移位寄存器(0~7)中的内容通过MISO信号线返回给主机。同时通过MOSI信号线接收主机发送的数据，这样，两个移位寄存器中的内容就被交换。**

![image-20230713120800936](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230713120800936.png)

**SPI只有主模式和从模式之分，没有读和写的说法，外设的写操作和读操作是同步完成的。如果只进行写操作，主机只需忽略接收到的字节；反之，若主机要读取从机的一个字节，就必须发送一个空字节来引发从机的传输。也就是说，你发一个数据必然会收到一个数据；你要收一个数据必须也要先发一个数据。**



## SPI工作模式

根据时钟极性（CPOL）及相位（CPHA）不同，SPI有四种工作模式。
时钟极性(CPOL)定义了时钟空闲状态电平：

- CPOL=0为时钟空闲时为低电平
- CPOL=1为时钟空闲时为高电平

时钟相位(CPHA)定义数据的采集时间。

- CPHA=0:在时钟的第一个跳变沿（上升沿或下降沿）进行数据采样。
- CPHA=1:在时钟的第二个跳变沿（上升沿或下降沿）进行数据采样。

![image-20230713121443951](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230713121443951.png)



## W25Q128 FLASH芯片介绍

**W25Q128是一款SPI通信的FLASH芯片**，可以通过标准/两线/四线SPI控制，其FLASH的大小为16M，分为 256 个块（Block），每个块大小为 64K 字节，每个块又分为 16个扇区（Sector），每个扇区 4K 个字节。通过SPI通信协议即可实现MCU(STM32)和 W25Q128 之间的通信。实现W25Q128的控制需要通过SPI协议发送相应的控制指令，并满足一定的时序。

![image-20230713144738413](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230713144738413.png)

**写使能(Write Enable) (06h)**

向FLASH发送0x06 写使能命令即可开启写使能，首先CS片选拉低，控制写入字节函数写入命令，CS片选拉高。

**扇区擦除指令(Sector Erase) (0x20h)**

扇区擦除指令，数据写入前必须擦除对应的存储单元，该指令先拉低/CS引脚电平,接着传输“20H”指令和要24位要擦除扇区的地址。

**读命令(Read Data) (03h)**

读数据指令可从存储器依次一个或多个数据字节，该指令通过主器件拉低/CS电平使能设备开始传输，然后传输“03H”指令，接着通过DI管脚传输24位地址，从器件接到地址后，寻址存储器中的数据通过DO引脚输出。每传输一个字节地址自动递增，所以只要时钟继续传输，可以不断读取存储器中的数据。

**状态读取命令(Read Status Register)**

读状态寄存器1(05H)，状态寄存器2（35H）,状态寄存器3（15H），写入命令0x05，即可读取状态寄存器的值。

**写入命令(Page Program) (02h)**

在对W25Q128 FLASH的写入数据的操作中一定要先擦出扇区，在进行写入，否则将会发生数据错误。
W25Q128 FLASH一次性最大写入只有256个字节。
在进行写操作之前，一定要开启写使能(Write Enable)。
当只接收数据时不但能只检测RXNE状态 ，必须同时向发送缓冲区发送数据才能驱动SCK时钟跳变。



STM32(精英版）有硬件NSS(片选信号)，可以选择使能，也可以使用其他IO口接到芯片的NSS上进行代替

**其中SIP2的片选NSS ： SPI2_NSS（PB12）**

如果片选引脚没有连接SPI2_NSS（PB12），则需要选择软件片选

**NSS管脚及我们熟知的片选信号**，**作为主设备NSS管脚为高电平，从设备NSS管脚为低电平**。当NSS管脚为低电平时，该spi设备被选中，可以和主设备进行通信。





## SPI函数详解

在stm32f1xx_hal_spi.h头文件中可以看到spi的操作函数。分别对应**轮询，中断和DMA**三种控制方式。

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230713155806198.png" alt="image-20230713155806198" style="zoom: 67%;" />

- **轮询：** 最基本的发送接收函数，就是正常的发送数据和接收数据
- **中断：** 在SPI发送或者接收完成的时候，会进入SPI回调函数，用户可以编写回调函数，实现设定功能
- **DMA：** DMA传输SPI数据

利用SPI接口发送和接收数据主要调用以下两个函数：

```c++
HAL_StatusTypeDef  HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);//发送数据
HAL_StatusTypeDef  HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);//接收数据
```

参数:

- *hspi: 选择SPI1/2，比如&hspi1，&hspi2
- *pData ： 需要发送的数据，可以为数组
- Size： 发送数据的字节数，1 就是发送一个字节数据
- Timeout： 超时时间，就是执行发送函数最长的时间，超过该时间自动退出发送函数



**SPI接收回调函数：**

```c++
　HAL_SPI_TransmitReceive_IT(&hspi1, TXbuf,RXbuf,CommSize);
```

当SPI上接收出现了 CommSize个字节的数据后，中断函数会调用SPI回调函数：

```c++
　HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
```









# 13.CAN通信

文档

[(59条消息) CAN通信知识梳理及在Stm32上的应用（HAL库）_冬瓜~的博客-CSDN博客](https://blog.csdn.net/weixin_44793491/article/details/107298426)

[(70条消息) cubemx配置can通信教程（stm32）（带项目工程文件）_stm32cubemx can_一个爱茶的工科男的博客-CSDN博客](https://blog.csdn.net/weixin_38800901/article/details/126662080?ops_request_misc=%7B%22request%5Fid%22%3A%22168924944316800180615244%22%2C%22scm%22%3A%2220140713.130102334..%22%7D&request_id=168924944316800180615244&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_click~default-1-126662080-null-null.142^v88^insert_down1,239^v2^insert_chatgpt&utm_term=stm32cubemx can通信&spm=1018.2226.3001.4187)



## 13.1CAN简介

### 显性电平和隐性电平

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230713195948980.png" alt="image-20230713195948980" style="zoom:50%;" />



- **显性电平：逻辑0** CAN_H和CAN_L电压差大
- **隐性电平：逻辑1** CAN_H和CAN_L电压差小
- 在总线上显性电平具有优先权，只要有一个单元输出显性电平，总线上即为显性电平
- 出现连续的11位隐性电平，那么总线就处于空闲状态

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230713200132986.png" alt="image-20230713200132986" style="zoom:50%;" />

### 报文（帧）的类型

- **数据帧**
  发送单元向接收单元传送数据的帧
- **遥控帧**
  接收单元向具有相同 ID 的发送单元请求数据的帧
- **错误帧**
  当检测出错误时向其它单元通知错误的帧
- **过载帧**
  接收单元通知其尚未做好接收准备的帧
- **间隔帧**
  将数据帧及遥控帧与前面的帧分离开来的帧

### 数据帧

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230713200345631.png" alt="image-20230713200345631" style="zoom: 67%;" />

### 位时序

由发送单元在非同步的情况下发送的每秒钟的位数称为位速率，一个位可分为 4 段，每个段又由若干个 Time Quantum（以下称为 Tq） 构成，这称为位时序

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230713200842044.png" alt="image-20230713200842044" style="zoom:50%;" />

![image-20230713200924199](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230713200924199.png)

- 采样点

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230713201002569.png" alt="image-20230713201002569" style="zoom: 67%;" />

### 工作模式

- **静默模式**

![image-20230713201939839](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230713201939839.png)

节点的输出端的逻辑0数据会直接传输到自己的输入端，逻辑1可以被发送到总线，所以**它不能向总线发送显性位(0)， 只能发送隐性位(1)**。输入端可以从总线接收内容。

- **回环模式**

  ![image-20230713202134183](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230713202134183.png)

节点输出端的所有内容都直接传输到自己的输入端，输出端的内容同时也会被传输到总线上，**输入端只接收自己发送端的内容，不接收来自总线上的内容。**
使用回环模式可以进行自检。

- **环回与静默组合模式**

![image-20230713202313285](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230713202313285.png)

节点的输出端的所有内容都直接传输到自己的输入端，输入端不接收来自总线上的内容，不会向总线发送显性位影响总线

## 13.2  CAN通信模型

图片讲解

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230710111004432.png" alt="image-20230710111004432" style="zoom:50%;" />



1.首先进行can的初始化can_init()

2.配置筛选器

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/stm32_picture/image-20230710111409537.png" alt="image-20230710111409537" style="zoom:50%;" />



**发送邮箱**

一共有3个发送邮箱，即最多可以缓存3个待发送的报文

**接收FIFO**

一共有2个接收FIFO，每个FIFO中有3个邮箱，即最多可以缓存6个接收到的报文。当接收到报文时，FIFO的报文计数器会自增，而STM32内部读取FIFO数据之后，报文计数器会自减，通过状态寄存器可获知报文计数器的值，而通过前面主控制寄存器的RFLM位，可设置锁定模式，锁定模式下FIFO溢出时会丢弃新报文，非锁定模式下FIFO溢出时新报文会覆盖旧报文。


## 13.3 实验

参考资料：[(70条消息) 最新CubeMX配置CAN通讯教程，避免踩坑，附全套工程文件_cubemx can_皮皮爹地的博客-CSDN博客](https://blog.csdn.net/prolop87/article/details/122671441?spm=1001.2101.3001.6661.1&utm_medium=distribute.pc_relevant_t0.none-task-blog-2~default~CTRLIST~Rate-1-122671441-blog-126662080.235^v38^pc_relevant_sort_base3&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-2~default~CTRLIST~Rate-1-122671441-blog-126662080.235^v38^pc_relevant_sort_base3&utm_relevant_index=1)

[(70条消息) CAN总线学习笔记 | STM32CubeMX配置CAN环回测试_can 回环测试_安迪西嵌入式的博客-CSDN博客](https://blog.csdn.net/Chuangke_Andy/article/details/127770680?ops_request_misc=&request_id=&biz_id=102&utm_term=stm32cubemx can通信&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-5-127770680.142^v88^insert_down1,239^v2^insert_chatgpt&spm=1018.2226.3001.4187)