#include "cbus.h"

extern TIM_HandleTypeDef htim2;
extern CAN_HandleTypeDef hcan;

volatile uint8_t FC_state = FC_STATE_RX;
volatile uint8_t incomingByte = 0x00;
volatile uint8_t bitPosition = 0; // 0-7

volatile uint8_t bytesOut[16] = {};
volatile uint8_t bytesOutIndex = 0;
volatile uint8_t bytesOutTotalCount = 0;

volatile uint16_t clockCount = 0xFFFF;
volatile uint8_t shouldResetSdaNext = 0;
volatile uint8_t shouldWaitClock = 0;

volatile uint8_t incomingByteReady = 0;
uint8_t SRQ_started = 0;

uint8_t discNumber = 1;
uint8_t trackNumber = 1;

uint8_t initDone = 0;
uint8_t discChangeStarted = 0;

volatile uint8_t SRQ_riseState = SRQ_STATE_IDLE;
volatile uint8_t SRQ_fallState = SRQ_STATE_IDLE;

CAN_TxHeaderTypeDef CANTxHeader;
uint8_t CANTxData[1];
uint32_t CANTxMailbox;

static void broadcastCommand(uint8_t byte) {
  CANTxData[0] = byte;
  HAL_CAN_AddTxMessage(&hcan, &CANTxHeader, CANTxData, &CANTxMailbox);
}

static void deInit() {
  initDone = 0;
}

static void scheduleTX(uint8_t bytesCount) {
  FC_state = FC_STATE_TX;
  bytesOutTotalCount = bytesCount;
  bytesOutIndex = 0;
}

static void scheduleCommandResponse(uint8_t cmd, uint8_t size, uint8_t *pBytes) {
  bytesOut[0] = cmd;
  bytesOut[1] = size;
  bytesOut[2] = (uint8_t)~size;
  if (pBytes != NULL) {
    for (uint8_t i = 0; i < size; i++) {
      bytesOut[(i * 2) + 3] = pBytes[i];
      bytesOut[(i * 2) + 4] = (uint8_t)~pBytes[i];
    }
  }
  scheduleTX((size * 2) + 3);
}

static void SRQ_startHandshake(void) {
  SRQ_riseState = SRQ_STATE_IDLE;
  SRQ_started = 1;

  CBUS_SRQ_O_GPIO_Port->BSRR = (uint32_t)CBUS_SRQ_O_Pin; // set to HI to turn SRQ LO
}

static void SRQ_endHandshake(void) {
  SRQ_fallState = SRQ_STATE_IDLE;
  SRQ_started = 0;

  CBUS_SRQ_O_GPIO_Port->BRR = (uint32_t)CBUS_SRQ_O_Pin; // set to LO to turn SRQ HI

  bytesOut[0] = 0xF7;
  scheduleTX(1);
}

static void SRQ_scheduleStart(void) {
  SRQ_riseState = SRQ_STATE_SCHEDULED;
}

static void SRQ_scheduleEnd(void) {
  // SRQ, prepare end handshake
  SRQ_fallState = SRQ_STATE_SCHEDULED;
}

static void changeDisc(uint8_t n) {
  if (!initDone) return;

  switch (n) {
    case 1: broadcastCommand(CBUS_EMU_CAN_CMD_1); break;
    case 2: broadcastCommand(CBUS_EMU_CAN_CMD_UP); break;
    case 3: broadcastCommand(CBUS_EMU_CAN_CMD_3); break;
    case 4: broadcastCommand(CBUS_EMU_CAN_CMD_4); break;
    case 5: broadcastCommand(CBUS_EMU_CAN_CMD_5); break;
    case 6: broadcastCommand(CBUS_EMU_CAN_CMD_DOWN); break;
    default:
      break;
  }

  discChangeStarted = 1;
}

static void changeTrack(uint8_t n) {
  if (!initDone || discChangeStarted) return;

  switch (n) {
    case 1: broadcastCommand(CBUS_EMU_CAN_CMD_PLAY_PAUSE); break;
    case 2: broadcastCommand(CBUS_EMU_CAN_CMD_RIGHT); break;
    case 3: broadcastCommand(CBUS_EMU_CAN_CMD_LEFT); break;
    default:
      break;
  }
}

static void handleCmd(uint8_t cmd) {
  switch (cmd) {
    // INIT
    case 0xF7: {
      if (SRQ_started == 1) {
        SRQ_scheduleEnd(); // SRQ success, release
      } else {
        // init seq
        deInit();
        bytesOut[0] = cmd;
        scheduleTX(1);
      }
      break;
    }

    // SRQ
    case 0x00: {
      bytesOut[0] = 0xF7;
      scheduleTX(1);
      break;
    }

    // SRQ
    case 0x11: {
      uint8_t data[3] = { 0x30 + discNumber, 0x78, trackNumber };
      scheduleCommandResponse(cmd, 3, data);
      break;
    }

    // pre de-init
    case 0x44: {
      uint8_t data[3] = { 0x00, 0x09, 0x00 };
      scheduleCommandResponse(cmd, 3, data);
      break;
    }

    // de-init
    case 0x01: {
      deInit();
      scheduleCommandResponse(cmd, 0, NULL);
      break;
    }

    // pause
    case 0x59: {
      broadcastCommand(CBUS_EMU_CAN_CMD_PLAY_PAUSE);
      uint8_t data[3] = { 0x00, 0x01, 0x00 };
      scheduleCommandResponse(cmd, 3, data);
      break;
    }

    // repeat on
    case 0x58: {
      broadcastCommand(CBUS_EMU_CAN_CMD_REPEAT);
      uint8_t data[3] = { 0x00, 0x11, 0x04 };
      scheduleCommandResponse(cmd, 3, data);
      break;
    }

    // repeat off
    case 0x56: {
      broadcastCommand(CBUS_EMU_CAN_CMD_REPEAT);
      uint8_t data[3] = { 0x00, 0x11, 0x04 };
      scheduleCommandResponse(cmd, 3, data);
      break;
    }

    // scan on
    case 0x55: {
      broadcastCommand(CBUS_EMU_CAN_CMD_SCAN);
      uint8_t data[3] = { 0x00, 0x01, 0x00 };
      scheduleCommandResponse(cmd, 3, data);
      break;
    }

    // scan off
    case 0x51: {
      broadcastCommand(CBUS_EMU_CAN_CMD_SCAN);
      uint8_t data[3] = { 0x00, 0x11, 0x00 };
      scheduleCommandResponse(cmd, 3, data);
      break;
    }

    case 0x4B: {
      uint8_t data[2] = { 0x10 + discNumber, trackNumber };
      scheduleCommandResponse(cmd, 2, data);
      break;
    }

    case 0x4D: {
      uint8_t data[3] = { 0x00, 0x01, 0x04 };
      scheduleCommandResponse(cmd, 3, data);
      break;
    }

    case 0x4C: {
      uint8_t data[2] = { 0x00, 0x00 };
      scheduleCommandResponse(cmd, 2, data);
      break;
    }

    // first command of select sequence
    case 0x09:
    case 0x45: {
      uint8_t data[3] = { 0x00, 0x01, 0x00 }; // disc is ready to go
      scheduleCommandResponse(cmd, 3, data);
      break;
    }

    // ???
    case 0x50: {
      uint8_t data[3] = { 0x00, 0x01, 0x1A };
      scheduleCommandResponse(cmd, 3, data);
      break;
    }

    // discs in magazine
    case 0x41: {
      uint8_t data[4] = { 0x1A, 0x55, 0x50, 0x00 };
      scheduleCommandResponse(cmd, 4, data);
      break;
    }

    // disc change pt2
    case 0xF1 ... 0xF6: {
      uint8_t data[3] = { 0x30 + discNumber, 0x78, 0x13 };
      scheduleCommandResponse(cmd, 3, data);
      break;
    }

    // disc change pt3
    case 0xE1 ... 0xE6: {
      uint8_t data[3] = { 0x40 + discNumber, 0x03, 0x01 }; // disc number, tracks count, 0x01
      scheduleCommandResponse(cmd, 3, data);
      break;
    }

    // disc change to 6X
    case 0x61 ... 0x66: {
      changeDisc(cmd - 0x60);
      scheduleCommandResponse(cmd, 0, NULL);
      break;
    }

    // cmd, cmd: 7X, 8Y: track: XY
    // 70 81 - 01
    // 70 82 - 02
    // 70 83 - 03
    // 71 80 - 10

    // track # first N
    case 0x70: {
      scheduleCommandResponse(cmd, 0, NULL);
      break;
    }

    // track # second N
    case 0x81 ... 0x83: {
      changeTrack(cmd - 0x80);
      scheduleCommandResponse(cmd, 0, NULL);
      break;
    }

    case 0x5C: {
      initDone = 1;
      discChangeStarted = 0;
      uint8_t data[3] = { 0x00, 0x01, trackNumber };
      scheduleCommandResponse(cmd, 3, data);
      SRQ_scheduleStart();
      break;
    }

    case 0xFF: {
      // nope
      break;
    }

    default:
      break;
  }
}

void CBUS_main(void) {
  HAL_TIM_Base_Start(&htim2);

  HAL_CAN_Start(&hcan);

  CANTxHeader.DLC = 1;
  CANTxHeader.IDE = CAN_ID_STD;
  CANTxHeader.RTR = CAN_RTR_DATA;
  CANTxHeader.StdId = CBUS_EMU_CAN_ID;
}

void CBUS_tick(void) {
  if (incomingByteReady == 1) {
    incomingByteReady = 0;
    handleCmd(incomingByte);
  }
  if (SRQ_riseState == SRQ_STATE_READY) {
    HAL_Delay(20);
    SRQ_startHandshake();
  }
  if (SRQ_fallState == SRQ_STATE_READY) {
    SRQ_endHandshake();
  }
}

void EXTI0_1_IRQHandler(void) {
  EXTI->PR = CBUS_CLK_Pin;

  if (clockCount == SKIP_CLOCKS_COUNT - 1) {
    CBUS_SDA_O_GPIO_Port->BSRR = (uint32_t)CBUS_SDA_O_Pin;
  }

  if (clockCount == (SKIP_CLOCKS_COUNT + 3)) {
    CBUS_SDA_O_GPIO_Port->BRR = (uint32_t)CBUS_SDA_O_Pin;
    clockCount = 0;
    shouldWaitClock = 0;

    if (FC_state == FC_STATE_RX) {
      if (SRQ_riseState == SRQ_STATE_SCHEDULED) {
        SRQ_riseState = SRQ_STATE_READY;
      }
      if (SRQ_fallState == SRQ_STATE_SCHEDULED) {
        SRQ_fallState = SRQ_STATE_READY;
      }
    }
  }

  // reset SDA after write
  if (shouldResetSdaNext == 1) {
    // SET pin to LO turn SDA HI (or release)
    CBUS_SDA_O_GPIO_Port->BSRR = (uint32_t)CBUS_SDA_O_Pin << 16U;
    shouldResetSdaNext = 0;
  }

  // check if it's a new burst
  // or if we are already in the middle of R/W state
  if ((shouldWaitClock == 0 && (htim2.Instance->CNT > BURST_GAP_TIME)) || bitPosition > 0) {

    if (FC_state == FC_STATE_RX) {
      // Shift a new bit IN
      incomingByte = (incomingByte << 1) | ((CBUS_SDA_GPIO_Port->IDR & CBUS_SDA_Pin) != (uint32_t)GPIO_PIN_RESET);

      // Check that we have read a whole byte
      if (bitPosition == 7) {
        incomingByteReady = 1;
        bitPosition = 0;
        shouldWaitClock = 1;
      } else {
        bitPosition++;
      }
    }

    if (FC_state == FC_STATE_TX) {
      if ((bytesOut[bytesOutIndex] >> (7 - bitPosition)) & 0b00000001) {
        // SET pin to LO turn SDA HI (or reset)
        CBUS_SDA_O_GPIO_Port->BRR = (uint32_t)CBUS_SDA_O_Pin;
      } else {
        // SET pin to HI turn SDA LO
        CBUS_SDA_O_GPIO_Port->BSRR = (uint32_t)CBUS_SDA_O_Pin;
      }

      // Check that we have written a whole byte, and switch to next byte or RX state
      if (bitPosition == 7) {
        // nothing to send, schedule RX
        if (bytesOutIndex == bytesOutTotalCount - 1) {
          FC_state = FC_STATE_RX;

          // reset flags/counters
          bytesOutTotalCount = 0;
          bytesOutIndex = 0;
        } else {
          // if it's a multibyte packet - schedule next byte TX
          bytesOutIndex++;
        }
        shouldResetSdaNext = 1; // reset SDA at the final burst, after TX
        bitPosition = 0; // reset bit counter anyway
        shouldWaitClock = 1; // wait for the next burst
      } else {
        bitPosition++;
      }
    }
  }

  htim2.Instance->CNT = 0; // reset timer anyway

  clockCount++;
}
