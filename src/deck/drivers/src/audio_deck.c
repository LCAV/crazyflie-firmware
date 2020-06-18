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
#include "deck_spi.h"

#include "audio_deck.h"


#define AUDIO_SHIELD_CS_PIN    		DECK_GPIO_IO1
#define SPI_BEGIN               	spiBegin
#define AUDIO_SHIELD_SPI_BAUDRATE   SPI_BAUDRATE_2MHZ
#define SPI_EXCHANGE            	spiExchange
#define SPI_BEGIN_TRANSACTION   	spiBeginTransaction
#define SPI_END_TRANSACTION     	spiEndTransaction


static void initSPI(void);
static void csLow(void);
static void csHigh(void);
static BYTE xchgSpi(BYTE dat);

static uint16_t spiSpeed;
static BYTE read_byte;
static bool isInit;

//////////////////////////////// LOW LEVEL SPI ///////////////////////////////
static void initSPI(void){
  SPI_BEGIN();   /* Enable SPI function */
  spiSpeed = AUDIO_SHIELD_SPI_BAUDRATE;

  pinMode(AUDIO_SHIELD_CS_PIN, OUTPUT);
  digitalWrite(AUDIO_SHIELD_CS_PIN, 1);

}

static void csHigh(void){
  digitalWrite(AUDIO_SHIELD_CS_PIN, 1);

  // Dummy clock (force DO hi-z for multiple slave SPI)
  // Moved here from fatfs_sd.c to handle bus release
  xchgSpi(0xFF);

  SPI_END_TRANSACTION();
}

static void csLow(void){
  SPI_BEGIN_TRANSACTION(spiSpeed);
  digitalWrite(AUDIO_SHIELD_CS_PIN, 0);
}

/* Exchange a byte */
static BYTE xchgSpi(BYTE dat){
  BYTE receive;

  SPI_EXCHANGE(1, &dat, &receive);
  return (BYTE)receive;
}

////////////////////////////////////// AUDIO DECK FUNCTIONS /////////////////////////////////

void audio_deckInit(void)
{
  if (isInit)
    return;
  initSPI();
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

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, M2T(500));
    csLow();
    read_byte = xchgSpi(0xFF);
    csHigh();
    DEBUG_PRINT("button : %d \n", read_byte);

  }
}

static const DeckDriver audio_deck = {

    .name = "audio_deck",
    .usedGpio = DECK_USING_MISO|DECK_USING_MOSI|DECK_USING_SCK|DECK_USING_IO_1,
    .usedPeriph = DECK_USING_SPI,
    .init =audio_deckInit,
    .test = audio_deckTest,
};

DECK_DRIVER(audio_deck);
