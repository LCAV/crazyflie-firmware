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

////////////////////////////////////// DEFINES /////////////////////////////////

#define AUDIO_DECK_ADDRESS 47
#define AUDIO_TASK_FREQUENCY 100 // frequency at which packets are sent [Hz]
#define CRTP_MAX_PAYLOAD 29
#define FFT_SIZE 32
#define N_MICS 4
#define FLOAT_PRECISION 4  // float32 = 4 bytes
#define ARRAY_SIZE N_MICS*FFT_SIZE*FLOAT_PRECISION*2  // *2 for complex numbers
#define N_FULL_PACKETS (int)ARRAY_SIZE/CRTP_MAX_PAYLOAD
#define N_PACKETS N_FULL_PACKETS + 1
#define M 5// number of arrays averaged 

////////////////////////////////////// PRIVATE VARIABLES /////////////////////////////////

static BYTE corr_array_sent[ARRAY_SIZE];     // buffer to be sent through CRTP after averaging
static BYTE corr_array_received[ARRAY_SIZE]; // buffer where I2C data is received
static float corr_array_averaged[N_MICS*FFTSIZE*2] // buffer where I2C data is averaged

static bool isInit;

static bool corr_matrix_sending = 0;
static uint8_t packet_count = 0;

////////////////////////////////////// AUDIO DECK FUNCTIONS /////////////////////////////////

void byte_array_to_float(uint8_t input[], float* output)
{
    for (int i = 0;i<FLOAT_PRECISION;i++){
      *((uint8_t*)(output) + i) = input[i];
    }
}

void float_to_byte_array(float input, uint8_t output[]){
  uint32_t temp = *((uint32_t*) &input);
  for (int i = 0;i<FLOAT_PRECISION;i++){
    output[i] = temp&0xFF;
    temp >>= 8;
  }
}

void fillbuffer(uint8_t buffer[],uint8_t packet_count, uint8_t size){
	for(uint8_t i = 0; i<size; i++)
		buffer[i]=corr_array_sent[packet_count*CRTP_MAX_PAYLOAD+i];
}

void byte_array_to_float_array(float float_array[],uint8_t byte_array[]){
  for (int i = 0; i<FFTSIZE*nMic*2; i++){
    byte_array_to_float(&byte_array[i*FLOAT_PRECISION],&float_array[i]);
  }
}

void float_array_to_byte_array(float float_array[],uint8_t byte_array[]){
  for (int i = 0; i<FFTSIZE*nMic*2; i++){
    float_to_byte_array(float_array[i],&byte_array[i*FLOAT_PRECISION]);
  }
}

void send_corr_packet(uint8_t channel){


	static CRTPPacket corr_array_p;
	corr_array_p.header = CRTP_HEADER(CRTP_PORT_AUDIO, channel);
	corr_array_p.size = CRTP_MAX_PAYLOAD;

	if (packet_count == N_FULL_PACKETS){ 	// send last packet and reset counter
		fillbuffer(corr_array_p.data,packet_count,ARRAY_SIZE%CRTP_MAX_PAYLOAD);
		crtpSendPacket(&corr_array_p);
		corr_matrix_sending = 0;
		packet_count = 0;
	}
	else{  // send packet
		fillbuffer(corr_array_p.data,packet_count,CRTP_MAX_PAYLOAD);
		crtpSendPacket(&corr_array_p);
		packet_count++;
	}

}
void receive_audio_deck_array(){
      i2cdevRead(I2C1_DEV, AUDIO_DECK_ADDRESS, ARRAY_SIZE, corr_array_received); // get array from deck
      byte_array_to_float_array()

}

void audio_deckInit(DeckInfo* info){ // deck initialisation
  if (isInit)
    return;
  DEBUG_PRINT("AUDIO INIT");
  xTaskCreate(audio_deckTask, AUDIO_TASK_NAME, AUDIO_TASK_STACKSIZE, NULL, AUDIO_TASK_PRI, NULL);

  isInit = true;
}


bool audio_deckTest(void){  // deck test
  if (!isInit)
    return false;
  DEBUG_PRINT("AUDIO TEST_PASSED");
  return true;
}

void audio_deckTask(void* arg){ // main task
  systemWaitStart();
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, F2T(AUDIO_TASK_FREQUENCY));
    if (packet_count >= N_PACKETS-M){
      receive_audio_deck_array();
    }
    if (!corr_matrix_sending){
    	send_corr_packet(1); // first packet is sent in channel 1 (start condition)
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


