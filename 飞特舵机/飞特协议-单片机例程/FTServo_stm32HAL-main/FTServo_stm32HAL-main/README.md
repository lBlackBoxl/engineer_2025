# FTServo-stm32HAL

FEETECH BUS Servo stm32HAL library

1、stm32CubeMx版本6.1.2  
2、使用前先使用stm32CubeMx重生成HAL库  
3、实例默认使用stm32f103(stm32f1.ioc)/stm32f407(stm32f4.ioc)两个芯片型号，可根据实际重配置stm32CubeMx  
4、实例默认使用usart2（引脚默认）波特率115200，可根据实际重配置stm32CubeMx  
5、重配置串口后需要修改\Core\Src\main.c中的ftUart_Send\ftUart_Read函数  
6、实例默认使用usart1进行串口重定向，可根据实际重配置stm32CubeMx与\Core\Src\main.c中的fputc函数  
