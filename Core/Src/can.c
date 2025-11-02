/*
 * can.c
 *
 *  Created on: Nov 1, 2025
 *      Author: ekoba
 */
#include "main.h"
#include "can.h"

#if IO_USE_CANOPEN

extern CAN_HandleTypeDef hcan;

void CanSetBitrate(bitrate_t bitrate){
	/**
	 * APB1 Prescaller = 2 @ 36Mhz
	 */
	switch(bitrate){
		case BITRATE_125K:
			hcan.Init.Prescaler = 16;
			break;
		case BITRATE_250K:
			hcan.Init.Prescaler = 8;
			break;
		case BITRATE_500K:
			hcan.Init.Prescaler = 4;
			break;
		case BITRATE_1000K:
			hcan.Init.Prescaler = 2;
			break;
		default:
			break;
	}
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_16TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
void MX_CAN_Init(void)
{
  hcan.Instance = CAN1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;

  CanSetBitrate(DEF_BITRATE);

  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

}
#endif
