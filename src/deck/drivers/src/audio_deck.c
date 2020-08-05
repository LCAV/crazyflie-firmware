#define DEBUG_MODULE "AUDIO_DECK"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "debug.h"
#include "deck.h"
#include "i2cdev.h"
#include "log.h"

#include "audio_deck.h"



#define AUDIO_DECK_ADDRESS 47




static BYTE read_byte;
static bool isInit;



////////////////////////////////////// AUDIO DECK FUNCTIONS /////////////////////////////////

void audio_deckInit(void)
{
  if (isInit)
    return;
  DEBUG_PRINT("AUDIO INIT");
  xTaskCreate(audio_deckTask, AUDIO_TASK_NAME, AUDIO_TASK_STACKSIZE, NULL, AUDIO_TASK_PRI, NULL);

  isInit = true;
}


bool audio_deckTest(void)
{

  if (!isInit)
    return false;
  DEBUG_PRINT("AUDIO TEST_PASSED");
  return true;
}

void audio_deckTask(void* arg){
  systemWaitStart();
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  uint8_t send_byte = 0xFF;
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, M2T(200));
    i2cdevRead(I2C1_DEV, AUDIO_DECK_ADDRESS, 1, &read_byte);
    DEBUG_PRINT("button : %d \n", read_byte);

  }
}

static const DeckDriver audio_deck = {

    .name = "audio_deck",
    .usedGpio = DECK_USING_SDA|DECK_USING_SCL,
    .init =audio_deckInit,
    .test = audio_deckTest,
};

DECK_DRIVER(audio_deck);

LOG_GROUP_START(audio)
LOG_ADD(LOG_UINT8,i2c_byte,&read_byte)
LOG_GROUP_STOP(audio)

