#ifndef INC_CBUS_H_
#define INC_CBUS_H_

#include "main.h"
#include "string.h"
#include "stdio.h"

#define CBUS_EMU_CAN_ID 0x669

#define CBUS_EMU_CAN_CMD_LEFT 0x01
#define CBUS_EMU_CAN_CMD_RIGHT 0x02
#define CBUS_EMU_CAN_CMD_UP 0x03
#define CBUS_EMU_CAN_CMD_DOWN 0x04
#define CBUS_EMU_CAN_CMD_PLAY_PAUSE 0x05
#define CBUS_EMU_CAN_CMD_1 0x06
#define CBUS_EMU_CAN_CMD_3 0x07
#define CBUS_EMU_CAN_CMD_4 0x08
#define CBUS_EMU_CAN_CMD_5 0x09
#define CBUS_EMU_CAN_CMD_REPEAT 0x10
#define CBUS_EMU_CAN_CMD_SCAN 0x11

#define FC_STATE_RX 0
#define FC_STATE_TX 1

#define SRQ_STATE_IDLE 0
#define SRQ_STATE_SCHEDULED 1
#define SRQ_STATE_READY 2

#define SKIP_CLOCKS_COUNT 10 * 8
#define BURST_GAP_TIME 20 // us

#ifdef __cplusplus
extern "C" {
#endif

void CBUS_main(void);
void CBUS_tick(void);
void EXTI0_1_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_CBUS_H_ */
