/*
 * can.h
 *
 *  Created on: Nov 1, 2025
 *      Author: ekoba
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "main.h"

#if IO_USE_CANOPEN
	void MX_CAN_Init(void);
	void CanSetBitrate(bitrate_t bitrate);
#endif

#endif /* INC_CAN_H_ */
