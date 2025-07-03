/*
回读所有舵机反馈参数:位置、速度、负载、电压、温度、移动状态、电流；
FeedBack函数回读舵机参数于缓冲区，Readxxx(-1)函数返回缓冲区中相应的舵机状态；
函数Readxxx(ID)，ID=-1返回FeedBack缓冲区参数；ID>=0，通过读指令直接返回指定ID舵机状态,
无需调用FeedBack函数。
*/

#include "main.h"
#include <../../SCSLib/SCServo.h>
#include <stdio.h>

void setup(void)
{
	setEnd(1);//SCSCL舵机为大端存储结构
}

void examples(void)
{
	int Pos;
  int Speed;
  int Load;
  int Voltage;
  int Temper;
  int Move;
  int Current;
	FeedBack(1);
  if(!getLastError()){
    Pos = ReadPos(-1);
    Speed = ReadSpeed(-1);
    Load = ReadLoad(-1);
    Voltage = ReadVoltage(-1);
    Temper = ReadTemper(-1);
    Move = ReadMove(-1);
		Current = ReadCurrent(-1);
		printf("Pos:%d\n", Pos);
		printf("Speed:%d\n", Speed);
		printf("Load:%d\n", Load);
		printf("Voltage:%d\n", Voltage);
		printf("Temper:%d\n", Temper);
		printf("Move:%d\n", Move);
    printf("Current:%d\n", Current);
    HAL_Delay(10);
  }else{
		printf("FeedBack err\n");
    HAL_Delay(2000);
  }
  Pos = ReadPos(1);
  if(!getLastError()){
    printf("Servo position:%d\n", Pos);
    HAL_Delay(10);
  }else{
    printf("read position err\n");
    HAL_Delay(500);
  }
  
  Voltage = ReadVoltage(1);
  if(!getLastError()){
		printf("Servo Voltage:%d\n", Voltage);
    HAL_Delay(10);
  }else{
    printf("read Voltage err\n");
    HAL_Delay(500);
  }
  
  Temper = ReadTemper(1);
  if(!getLastError()){
    printf("Servo temperature:%d\n", Temper);
    HAL_Delay(10);
  }else{
    printf("read temperature err\n");
    HAL_Delay(500);    
  }

  Speed = ReadSpeed(1);
  if(!getLastError()){
    printf("Servo Speed:%d\n", Speed);
    HAL_Delay(10);
  }else{
    printf("read Speed err\n");
    HAL_Delay(500);    
  }
  
  Load = ReadLoad(1);
  if(!getLastError()){
    printf("Servo Load:%d\n", Load);
    HAL_Delay(10);
  }else{
    printf("read Load err\n");
    HAL_Delay(500);    
  }
  
  Current = ReadCurrent(1);
  if(!getLastError()){
    printf("Servo Current:%d\n", Current);
    HAL_Delay(10);
  }else{
    printf("read Current err\n");
    HAL_Delay(500);    
  }

  Move = ReadMove(1);
  if(!getLastError()){
    printf("Servo Move:%d\n", Move);
    HAL_Delay(10);
  }else{
    printf("read Move err\n");
    HAL_Delay(500);    
  }
  printf("\n");
}
