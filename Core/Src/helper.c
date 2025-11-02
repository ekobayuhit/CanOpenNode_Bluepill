/*
 * helper.c
 *
 *  Created on: Nov 1, 2025
 *      Author: ekoba
 */
#include "main.h"
//#include "usbd_cdc_if.h"
//
//char  VCP_Buf[APP_TX_DATA_SIZE];
//int VCP_Printf(char *fmt, ...) {
//	va_list va;
//	va_start(va, fmt);
//
//	int ret = vsprintf(VCP_Buf, fmt, va);
//	va_end(va);
//	unsigned int len = strlen(VCP_Buf);
//	CDC_Transmit_FS((uint8_t *)VCP_Buf, len);
//
//	return ret;
//}

//static void debug_printf(const char *fmt, ...) {
//	char buf[128];
//	va_list ap;
//	va_start(ap, fmt);
//	vsnprintf(buf, sizeof(buf), fmt, ap);
//	va_end(ap);
//	HAL_UART_Transmit(&huart1, (uint8_t*) buf, strlen(buf), HAL_MAX_DELAY);
//}
