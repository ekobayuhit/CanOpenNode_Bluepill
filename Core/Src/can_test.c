/*
 * can_test.c
 *
 *  Created on: Nov 1, 2025
 *      Author: ekoba
 */
#include "main.h"
#include "cmsis_os.h"

#if CAN_TEST
#include "can_test.h"
#define RXCAN_BUF_SIZE     50
#define TXCAN_BUF_SIZE     1

typedef struct {
	CAN_RxHeaderTypeDef hdr;
	uint8_t data[8];
} rxcan_frame_t;

typedef struct {
	CAN_TxHeaderTypeDef hdr;
	uint8_t data[8];
} txcan_frame_t;

extern CAN_HandleTypeDef hcan;
extern int VCP_Printf(char *fmt, ...);

static rxcan_frame_t rx_msg;
static volatile rxcan_frame_t rx_buf[RXCAN_BUF_SIZE];
static volatile txcan_frame_t tx_buf[TXCAN_BUF_SIZE];
static volatile uint16_t rx_head = 0, rx_tail = 0;

void CanTask(void *argument);
osThreadId_t CanTaskHandle;
const osThreadAttr_t can_attributes = {
  .name = "can_test",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

static inline int rb_empty(void) {
	return rx_head == rx_tail;
}
static inline int rb_full(void) {
	return ((rx_head + 1) % RXCAN_BUF_SIZE) == rx_tail;
}
static void rb_push(const CAN_RxHeaderTypeDef *hdr, const uint8_t *data) {
	if (!rb_full()) {
		rx_buf[rx_head].hdr = *hdr;
		memcpy((void*) rx_buf[rx_head].data, data, hdr->DLC);
		rx_head = (rx_head + 1) % RXCAN_BUF_SIZE;
	}
}
static int rb_pop(rxcan_frame_t *out) {
    if (rb_empty()) {
        return 0;
    }
    *out = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RXCAN_BUF_SIZE;
    return 1;
}

static HAL_StatusTypeDef can_tx_blocking(uint32_t std_id, const uint8_t *data,
		uint8_t dlc, uint32_t timeout_ms) {
	CAN_TxHeaderTypeDef tx;
	uint32_t mb;
	tx.StdId = std_id;
	tx.IDE = CAN_ID_STD;
	tx.RTR = CAN_RTR_DATA;
	tx.DLC = dlc;
	uint32_t t0 = HAL_GetTick();
	while ((HAL_GetTick() - t0) < timeout_ms) {
		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
			HAL_StatusTypeDef st = HAL_CAN_AddTxMessage(&hcan, &tx,
					(uint8_t*) data, &mb);
			if (st == HAL_OK)
				return HAL_OK;
		}
		__NOP();
	}
	return HAL_TIMEOUT;
}

void create_task_can(){
	CanTaskHandle = osThreadNew(CanTask, NULL, &can_attributes);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h) {
	CAN_RxHeaderTypeDef hdr;
	uint8_t data[8];
	HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &hdr, data);
	rb_push(&hdr, data);
}

void CanTask(void *argument){
	uint32_t cnt = 0;
	uint32_t mb;
	HAL_StatusTypeDef ret;
	CAN_FilterTypeDef canfilter;

	canfilter.FilterBank = 0;
	canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilter.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilter.FilterIdHigh = 0;
	canfilter.FilterIdLow = 0;
	canfilter.FilterMaskIdHigh = 0;
	canfilter.FilterMaskIdLow = 0;
	canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilter.FilterActivation = ENABLE;
	canfilter.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan, &canfilter);

	if (HAL_CAN_Start(&hcan) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		Error_Handler();
	}

//	VCP_Printf("[CAN] Running Can test program \r\n");
	memset(tx_buf, '\0', sizeof(tx_buf));
	for(;;){
		if (rb_pop(&rx_msg) > 0) {
//		    VCP_Printf("[CAN] Recv id 0x%X with Data: "
//		               "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n",
//		               rx_msg.hdr.ExtId,
//		               rx_msg.data[0], rx_msg.data[1], rx_msg.data[2], rx_msg.data[3],
//		               rx_msg.data[4], rx_msg.data[5], rx_msg.data[6], rx_msg.data[7]);
		}

		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
			//Sending canbus message
			tx_buf->hdr.IDE = CAN_ID_STD;
			tx_buf->hdr.RTR = CAN_RTR_DATA;
			tx_buf->hdr.StdId = 0x151;
			tx_buf->hdr.DLC = 8;
			for(int i =0 ; i < 8; i++){
				tx_buf->data[i] = (cnt > 255) ? (cnt = 0) : cnt++;
			}
			ret = HAL_CAN_AddTxMessage(&hcan, &tx_buf->hdr, tx_buf->data, &mb);
		}

		osDelay(100);
	}
}
#endif
