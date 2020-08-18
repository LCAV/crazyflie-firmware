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
#define N_MICS 4
#define ARRAY_SIZE N_MICS*FFT_SIZE*4*2
#define NPACKETS (int)ARRAY_SIZE/CRTP_MAX_PAYLOAD

static BYTE corr_array[ARRAY_SIZE];
static bool isInit;

static bool corr_matrix_sending = 0;



////////////////////////////////////// AUDIO DECK FUNCTIONS /////////////////////////////////

void fillbuffer(uint8_t buffer[],uint8_t packet_count, uint8_t size){
	for(uint8_t i = 0; i<size; i++)
		buffer[i]=corr_array[packet_count*CRTP_MAX_PAYLOAD+i];
}

void con_64_bit_to_8_bit_array(uint64_t data, uint8_t array[]){
	for (uint16_t i = 7;i>=0;i--){
		array[i] = data&0xFF;
		data >>= 8;
	}
}

void send_corr_packet(uint8_t channel){
	static uint8_t packet_count = 0;

	static CRTPPacket corr_array_p;
	corr_array_p.header = CRTP_HEADER(CRTP_PORT_AUDIO, channel);
	corr_array_p.size = CRTP_MAX_PAYLOAD;

	if (packet_count == NPACKETS){
		fillbuffer(corr_array_p.data,packet_count,ARRAY_SIZE%CRTP_MAX_PAYLOAD);
		crtpSendPacket(&corr_array_p);
		corr_matrix_sending = 0;
		packet_count = 0;
	}
	else{
		fillbuffer(corr_array_p.data,packet_count,CRTP_MAX_PAYLOAD);
		crtpSendPacket(&corr_array_p);
		packet_count++;
	}

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
    vTaskDelayUntil(&xLastWakeTime, F2T(100));
    if (!corr_matrix_sending){
    	i2cdevRead(I2C1_DEV, AUDIO_DECK_ADDRESS, ARRAY_SIZE, corr_array);
    	send_corr_packet(1);
    	corr_matrix_sending = 1;
    }
    else{
    	send_corr_packet(0);
    }
  }
}



static const DeckDriver audio_deck = {
    .name = "audio_deck",
    .usedGpio = DECK_USING_SDA|DECK_USING_SCL,
    .init = audio_deckInit,
    .test = audio_deckTest,
};

DECK_DRIVER(audio_deck);


