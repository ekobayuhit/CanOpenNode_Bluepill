/*
 * config.h
 *
 *  Created on: Nov 1, 2025
 *      Author: ekoba
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

/* ============================================
 * Structure Definitions
 * ============================================ */
typedef enum{
	BITRATE_125K = 125,
	BITRATE_250K = 250,
	BITRATE_500K = 500,
	BITRATE_1000K = 1000
}bitrate_t;

/* ============================================
 * Macro Definitions
 * ============================================ */
#define IO_TYPE_DI			(1)
#define IO_USE_CANOPEN		(1)
#define CAN_TEST			(0)

#define IO_USE_CFG_BITRATE	(1)

#define DEF_LED_BLINK		(1000)
#define DEF_BITRATE			(BITRATE_250K)

/* ============================================
 * Macro Logic
 * ============================================ */
#if IO_TYPE_DI
	#define IO_TYPE_DO		(0)
#else
	#define IO_TYPE_DO		(1)
#endif

#if IO_USE_CANOPEN
	#undef  CAN_TEST
	#define CAN_TEST		0
#endif

#endif /* INC_CONFIG_H_ */
