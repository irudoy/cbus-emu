#ifndef INC_CBUS_H_
#define INC_CBUS_H_

#include "main.h"
#include "string.h"
#include "stdio.h"

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
