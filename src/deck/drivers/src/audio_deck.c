#define DEBUG_MODULE "AUDIO_DECK"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "stm32fxxx.h"
#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "debug.h"
#include "deck.h"
#include "i2cdev.h"
#include "log.h"
#include "crtp.h"
#include "usec_time.h"

#include "audio_deck.h"



#define AUDIO_DECK_ADDRESS 47
#define CRTP_MAX_PAYLOAD 29
#define FFT_SIZE 32
#define nMic 4
#define ARRAY_SIZE nMic*nMic*FFT_SIZE*4*2
#define NPACKETS (int)ARRAY_SIZE/CRTP_MAX_PAYLOAD

static BYTE corr_array[ARRAY_SIZE];
static bool isInit;


////////////////////////////////////// AUDIO DECK FUNCTIONS /////////////////////////////////

void fillbuffer(uint8_t buffer[],int start_index, uint8_t size){
	for(uint8_t i = 0; i<size; i++)
		buffer[i]=corr_array[i];
}

void con_64_bit_to_8_bit_array(uint64_t data, uint8_t array[]){
	for (uint16_t i = 7;i>=0;i--){
		array[i] = data&0xFF;
		data >>= 8;
	}
}

void send_corr_array(){

	static int start_index = 0;

	static CRTPPacket corr_array_p;
	corr_array_p.header = CRTP_HEADER(CRTP_PORT_AUDIO, 0);
	corr_array_p.size = CRTP_MAX_PAYLOAD;

	static CRTPPacket trans_p;
	trans_p.header = CRTP_HEADER(CRTP_PORT_AUDIO, 1);
	trans_p.size = 1;
	trans_p.data[0]=0;

	crtpSendPacket(&trans_p);

	for(uint16_t i= 0;i<NPACKETS;i++){
		fillbuffer(corr_array_p.data,start_index,CRTP_MAX_PAYLOAD);
		crtpSendPacket(&corr_array_p);
		start_index += CRTP_MAX_PAYLOAD;
	}

	fillbuffer(corr_array_p.data,start_index,ARRAY_SIZE%CRTP_MAX_PAYLOAD);
	crtpSendPacket(&trans_p);
	start_index = 0;

	trans_p.data[0]=1;
	crtpSendPacket(&trans_p);

}

void audio_deckInit(DeckInfo* info)
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
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, F2T(1));
    i2cdevRead(I2C1_DEV, AUDIO_DECK_ADDRESS, ARRAY_SIZE, corr_array);
    send_corr_array();
  }
}

static const DeckDriver audio_deck = {
    .name = "audio_deck",
    .usedGpio = DECK_USING_SDA|DECK_USING_SCL,
    .init = audio_deckInit,
    .test = audio_deckTest,
};

DECK_DRIVER(audio_deck);


