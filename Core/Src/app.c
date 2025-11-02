/*
 * app_rtos.c
 *
 *  Created on: Nov 1, 2025
 *      Author: ekoba
 */

#include "main.h"
#include "cmsis_os.h"

#if IO_USE_CANOPEN
#include "app.h"
#include "can.h"
#include "CO_app_STM32.h"
#include "OD.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} gpio_t;

extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim2;
extern OD_ATTR_RAM OD_RAM_t OD_RAM;

void CanOpenTask(void *argument);
osThreadId_t CanOpenTaskHandle;
const osThreadAttr_t canopen_attributes = {
  .name = "CanOpen",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityHigh,
};

void IOTask(void *argument);
osThreadId_t IOTaskHandle;
const osThreadAttr_t io_attributes = {
  .name = "IO",
  .stack_size = 128 * 2,
  .priority = (osPriority_t) osPriorityAboveNormal5,
};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == canopenNodeSTM32->timerHandle) {
        canopen_app_interrupt();
    }
}

void create_task_canopen(){
	CanOpenTaskHandle = osThreadNew(CanOpenTask, NULL, &canopen_attributes);
	IOTaskHandle = osThreadNew(IOTask, NULL, &io_attributes);
}

void init_io();
void sync_IO();

gpio_t GPIO[16] = {
	{GPIOB, GPIO_PIN_11},
	{GPIOB, GPIO_PIN_10},
	{GPIOB, GPIO_PIN_1},
	{GPIOB, GPIO_PIN_0},

	{GPIOA, GPIO_PIN_7},
	{GPIOA, GPIO_PIN_6},
	{GPIOA, GPIO_PIN_5},
	{GPIOA, GPIO_PIN_4},
	{GPIOA, GPIO_PIN_3},
	{GPIOA, GPIO_PIN_2},
	{GPIOA, GPIO_PIN_1},

	{GPIOC, GPIO_PIN_15},
	{GPIOC, GPIO_PIN_14},
	{GPIOC, GPIO_PIN_13},

	{GPIOB, GPIO_PIN_12},
	{GPIOB, GPIO_PIN_13}
};

uint16_t IO_Value;

void init_io(){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

#if IO_TYPE_DI
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  for (int i = 0; i < 16; i++) {
      GPIO_InitStruct.Pin = GPIO[i].pin;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
      HAL_GPIO_Init(GPIO[i].port, &GPIO_InitStruct);
  }

#elif IO_TYPE_DO
  for (int i = 0; i < 16; i++) {
	  GPIO_InitStruct.Pin = GPIO[i].pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	  HAL_GPIO_Init(GPIO[i].port, &GPIO_InitStruct);
  }

#endif
}

void CanOpenTask(void *argument){
	CANopenNodeSTM32 canOpenNodeSTM32;
	canOpenNodeSTM32.CANHandle = &hcan;
	canOpenNodeSTM32.HWInitFunction = MX_CAN_Init;
	canOpenNodeSTM32.timerHandle = &htim2;
	canOpenNodeSTM32.desiredNodeID = 1;
	canOpenNodeSTM32.baudrate = DEF_BITRATE;
	canopen_app_init(&canOpenNodeSTM32);

	for(;;){
		canopen_app_process();
		osDelay(1);
	}
}

void IOTask(void *argument)
{
    init_io();

    for (;;) {
#if IO_TYPE_DI
        IO_Value = 0;
        for (int i = 0; i < 16; i++) {
            if (HAL_GPIO_ReadPin(GPIO[i].port, GPIO[i].pin) == GPIO_PIN_SET) {
                IO_Value |= (1 << i);
            }
        }
#elif IO_TYPE_DO
        for (int i = 0; i < 16; i++) {
            HAL_GPIO_WritePin(GPIO[i].port,
                              GPIO[i].pin,
                              (IO_Value & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
#endif

        sync_IO();
        osDelay(1);
    }
}

void sync_IO(){
#if IO_TYPE_DI
	OD_RAM.x6000_digitalInput.value= IO_Value;
#elif IO_TYPE_DO
	IO_Value = 0;
#endif
}
#endif
