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
#include "param.h"
#include "crtp.h"
#include "usec_time.h"
#include "power_distribution.h"

#include "audio_deck.h"


/////////////////////////////////////////////////////////////////////////////////
/**
 *  Report from debugging session:
 *
 *  send_audio_enable | no  | yes | no
 *  receive_i2c       | yes | yes | yes
 *  send_i2c          | no  | no  | yes
 *
 */

////////////////////////////////////// DEFINES /////////////////////////////////

// SPI stuff
//#define CS_PIN DECK_GPIO_IO1


// debugging
//#define USE_TEST_SIGNALS
#ifdef USE_TEST_SIGNALS
#include "audio_debug_data.h"
#endif

// constants
#define INT16_PRECISION 2 // int16 = 2 bytes
#define FLOAT_PRECISION 4// float32 = 4 bytes
#define CRTP_MAX_PAYLOAD 29

// audio deck parameters
#define FFTSIZE 32
#define N_MICS 4
#define AUDIO_DECK_ADDRESS 47// I2C adress of the deck
#define N_MOTORS 4
#define AUDIO_TASK_FREQUENCY 100 // frequency at which packets are sent [Hz]
#define I2C_REQUEST_RATE 6 // I2C is requested each 6 cycles of main task i.e. 60 ms = 6/100Hz
#define SIZE_OF_PARAM_I2C 3 // in uint16, min_freq = 1, max_freq = 1, snr + propeller enable = 1

// buffer sizes
#define PARAM_N_INTS (N_MOTORS + SIZE_OF_PARAM_I2C) // in uint16, n_motors for the current trhust commands
#define PARAM_N_BYTES (PARAM_N_INTS * INT16_PRECISION) // 14 bytes

#define AUDIO_N_FLOATS N_MICS * FFTSIZE * 2 // *2 for complex numbers
#define AUDIO_N_BYTES (AUDIO_N_FLOATS * FLOAT_PRECISION)
#define AUDIO_N_PACKETS_FULL (int) AUDIO_N_BYTES / CRTP_MAX_PAYLOAD
#define AUDIO_N_PACKETS (AUDIO_N_PACKETS_FULL + 1)

#define FBINS_N_INTS FFTSIZE
#define FBINS_N_BYTES FBINS_N_INTS * INT16_PRECISION
#define FBINS_N_PACKETS_FULL (int) FBINS_N_BYTES / CRTP_MAX_PAYLOAD
#define FBINS_N_PACKETS (FBINS_N_PACKETS_FULL + 1)

#define TOTAL_N_BYTES (FBINS_N_BYTES + AUDIO_N_BYTES)

#define SPI_N_BYTES 1


////////////////////////////////////// PRIVATE VARIABLES /////////////////////////////////
static enum{
  SEND_FIRST_PACKET,
  SEND_AUDIO_PACKET,
  SEND_FBIN_PACKET
} state = SEND_FIRST_PACKET;

// DEBUGGING START
uint32_t t1, t2;
// DEBUGGING END


static BYTE byte_array_CRTP[AUDIO_N_BYTES]; // buffer to be sent through CRTP
static BYTE byte_array_received[TOTAL_N_BYTES]; // buffer where I2C data is received
static float float_array_averaged[AUDIO_N_FLOATS]; // buffer where I2C data is averaged
static float float_array_buffer[AUDIO_N_FLOATS]; // buffer where I2C data is converted to float

static uint16_t I2C_send_packet_int16[PARAM_N_INTS]; // buffer with the current parameters

static bool isInit;

static uint8_t packet_count_audio = 0;
static uint8_t packet_count_fbins = 0;

///////////////////////////////////////// PARAMETERS ////////////////////////////////////

// general parameter
uint8_t debug = 0; // enables DEBUG_PRINT
static bool send_audio_enable = 0; // enables the sending of CRTP packets with the audio data

// frequency selection parameters
static bool filter_propellers_enable = 0;
static bool filter_snr_enable = 0;
static uint16_t min_freq = 200;
static uint16_t max_freq = 10000;

// averaging parameters
static bool use_iir = 0; // put to one to use IIR filter instead of averaging through ma_window buffers
static float alpha_iir = 0.5; // iir moving average parameter
static uint8_t ma_window = 4; // number of arrays averaged, maximum is 6 as 6*6 = 36 = AUDIO_N_PACKETS

////////////////////////////////////// SPI COMMUNICATION / ///////////////////////////////////

// TOTAL_N_BYTES // we have more audio to receive than params to send
static uint16_t spiSpeed = SPI_BAUDRATE_2MHZ;

static uint8_t spiTxBuffer[SPI_N_BYTES];
static uint8_t spiRxBuffer[SPI_N_BYTES];
static uint8_t spi_write_array[SPI_N_BYTES];

static uint8_t spiWrite(const void* data, size_t dataLength)
{
  spiBeginTransaction(spiSpeed);
  memcpy(spiTxBuffer, data, dataLength);
  uint8_t success = spiExchange(dataLength, spiTxBuffer, spiRxBuffer);
  spiEndTransaction();
  return success;
}

static uint8_t spiRead(void* data, size_t dataLength)
{
  spiBeginTransaction(spiSpeed);
  memset(spiTxBuffer, 0, dataLength);
  uint8_t success = spiExchange(dataLength, spiTxBuffer, spiRxBuffer);
  memcpy(data, spiRxBuffer, dataLength);
  spiEndTransaction();
  return success;
}

static uint8_t spiReadWrite(void* dataRead, void* dataWrite, size_t dataLength)
{
  spiBeginTransaction(spiSpeed);
  memcpy(spiTxBuffer, dataWrite, dataLength);
  uint8_t success = spiExchange(dataLength, spiTxBuffer, spiRxBuffer);
  memcpy(dataRead, spiRxBuffer, dataLength);
  spiEndTransaction();
  return success;
}


////////////////////////////////////// AUDIO DECK FUNCTIONS /////////////////////////////////

/**
 *  Fill frequency data after the audio data in the CRTP packet.
 */
void fill_packet_data_fbins(uint8_t buffer[], uint8_t packet_count, uint8_t packet_size){
  for(uint8_t i = 0; i < packet_size; i++) {
    buffer[i] = byte_array_received[AUDIO_N_BYTES + packet_count * CRTP_MAX_PAYLOAD + i];
  }
}

/**
 *  Fill a new CRTP packet with the corresponding data from byte_array_CRTP.
 */
void fill_packet_data_audio(uint8_t packet_data[], uint8_t packet_count, uint8_t packet_size){
  for(uint8_t i = 0; i < packet_size; i++) {
      packet_data[i] = byte_array_CRTP[packet_count * CRTP_MAX_PAYLOAD + i];
  }
}

void byte_array_to_float(uint8_t input[], float* output)
{
  for (int i = 0; i < FLOAT_PRECISION; i++){
      *((uint8_t*)(output) + i) = input[i];
      // TODO(FD) this looks misplaced. Why was it here?
      // state = SEND_AUDIO_PACKET;
  }
}

void byte_array_to_float_array(float float_array[], uint8_t byte_array[], uint16_t n_floats){
  for (int i = 0; i < n_floats; i++){
      byte_array_to_float(&byte_array[i * FLOAT_PRECISION], &float_array[i]);
  }
}


void float_to_byte_array(float input, uint8_t output[]){
  uint32_t temp = *((uint32_t*) &input);
  for (int i = 0; i < FLOAT_PRECISION; i++){
      output[i] = temp&0xFF;
      temp >>= 8;
  }
}

void float_array_to_byte_array(float float_array[], uint8_t byte_array[]){
  for (int i = 0; i < AUDIO_N_FLOATS; i++){
      float_to_byte_array(float_array[i], &byte_array[i * FLOAT_PRECISION]);
  }
}


// DEBUGGING START
void uint_to_byte_array(uint16_t input, uint8_t output[]){
  uint16_t temp = *((uint16_t*) &input);
  for (int i = 0; i < INT16_PRECISION; i++){
      output[i] = temp&0xFF;
      temp >>= 8;
  }
}

void uint_array_to_byte_array(uint16_t uint_array[], uint8_t byte_array[]){
  for (int i = 0; i < FBINS_N_INTS; i++){
      uint_to_byte_array(uint_array[i], &byte_array[AUDIO_N_BYTES + i * INT16_PRECISION]);
      DEBUG_PRINT("%d: uint16 %d, bytes %d, %d \n",
                  i, uint_array[i],
                  byte_array[AUDIO_N_BYTES +  i * INT16_PRECISION],
                  byte_array[AUDIO_N_BYTES +  i * INT16_PRECISION + 1]);
  }
}
// DEBUGGING END


void add_divided_array_to_buffer(float array_to_divide[], float buffer[]){
  for (int i = 0; i < AUDIO_N_FLOATS; i++){
      buffer[i] += array_to_divide[i] / ma_window;
  }
}

void exp_filter(float incoming_buffer[], float buffer[]){
  for (int i = 0; i < AUDIO_N_FLOATS; i++){
      buffer[i]=buffer[i] * (1.0f-alpha_iir)+incoming_buffer[i]*alpha_iir;
  }
}

void erase_buffer(float buffer[], int buffer_size){
  for (int i = 0; i < buffer_size; i++){
      buffer[i] = 0;
  }
}

void copy_buffer(uint16_t input[], uint16_t output[], uint8_t buffer_size){
  for (uint8_t i = 0; i<buffer_size; i++){
      output[i] = input[i];
  }
}

void send_audio_packet(uint8_t channel){
    static CRTPPacket signal_array_p;
    signal_array_p.header = CRTP_HEADER(CRTP_PORT_AUDIO, channel);
    signal_array_p.size = CRTP_MAX_PAYLOAD;

    if (packet_count_audio == AUDIO_N_PACKETS_FULL){ 	// send last packet and reset counter
        fill_packet_data_audio(signal_array_p.data, packet_count_audio, AUDIO_N_BYTES % CRTP_MAX_PAYLOAD);
        crtpSendPacket(&signal_array_p);
        state = SEND_FBIN_PACKET;
        packet_count_audio = 0;
    }
    else { // send full packet
        fill_packet_data_audio(signal_array_p.data, packet_count_audio, CRTP_MAX_PAYLOAD);
        crtpSendPacket(&signal_array_p);
        packet_count_audio++;
    }
}

void send_fbin_packet(){
    static CRTPPacket fbin_array_p;
    fbin_array_p.header = CRTP_HEADER(CRTP_PORT_AUDIO, 2);
    fbin_array_p.size = CRTP_MAX_PAYLOAD;

    if (packet_count_fbins == FBINS_N_PACKETS_FULL){ // send last packet and reset counter
        fill_packet_data_fbins(fbin_array_p.data, packet_count_fbins, FBINS_N_BYTES % CRTP_MAX_PAYLOAD);
        crtpSendPacket(&fbin_array_p);
        packet_count_fbins = 0;
        state = SEND_FIRST_PACKET;
    }
    else{ // send full packet
        fill_packet_data_fbins(fbin_array_p.data, packet_count_fbins, CRTP_MAX_PAYLOAD);
        crtpSendPacket(&fbin_array_p);
        packet_count_fbins++;
    }
}

/** Request current audio data from the audio deck (signals and frequency bins)
 *
 */
void receive_audio_deck_array(){

  #ifdef USE_TEST_SIGNALS
  vTaskDelay(M2T(100)); // ms
  float_array_to_byte_array(audio_data, byte_array_received);
  uint_array_to_byte_array(frequencies, byte_array_received);
  uint8_t success = 1;
  #else
  //uint8_t success = i2cdevRead(I2C1_DEV, AUDIO_DECK_ADDRESS, TOTAL_N_BYTES, byte_array_received);
  //uint8_t success = spiRead(&byte_array_received, TOTAL_N_BYTES);
  uint8_t success = 0;

  #endif

  if (!success && debug) {
      DEBUG_PRINT("Warning: could not read data from audio deck \n");
  }
  else if (debug) {
      DEBUG_PRINT("Success: could read data from audio deck\n");

  }


  byte_array_to_float_array(float_array_buffer, byte_array_received, AUDIO_N_FLOATS);

  if (!use_iir) {
      add_divided_array_to_buffer(float_array_buffer, float_array_averaged);
  }
  else{
      exp_filter(float_array_buffer, float_array_averaged);
  }
}

void send_param_I2C(){
  if (filter_propellers_enable) {
      uint16_t *motorPower_p = get_motor_power();
      copy_buffer(motorPower_p, I2C_send_packet_int16, N_MOTORS);
  }

  I2C_send_packet_int16[N_MOTORS] = min_freq;
  I2C_send_packet_int16[N_MOTORS + 1] = max_freq;

  uint16_t enables = (filter_propellers_enable << 8) | filter_snr_enable;
  I2C_send_packet_int16[N_MOTORS + 2] = enables;

  //uint8_t *I2C_send_packet_byte = (uint8_t*)I2C_send_packet_int16;

  //uint8_t success = i2cdevWrite(I2C1_DEV, AUDIO_DECK_ADDRESS, PARAM_N_BYTES, I2C_send_packet_byte);
  //uint8_t success = spiWrite(&empty_header, HEADER_SIZE, I2C_send_packet_byte, PARAM_N_BYTES);
  uint8_t success = 0;

  if (!success && debug) {
      DEBUG_PRINT("Warning: could not send params to audio deck\n");
  }
  else if (debug) {
      DEBUG_PRINT("Success: could send params to audio deck\n");
  }
}

void audio_deckInit(DeckInfo* info){ // deck initialisation
  if (isInit)
    return;
  DEBUG_PRINT("AUDIO INIT\n");

  spiBegin();

  xTaskCreate(audio_deckTask, AUDIO_TASK_NAME, AUDIO_TASK_STACKSIZE, NULL, AUDIO_TASK_PRI, NULL);

  isInit = true;
}

bool audio_deckTest(void){// deck test
  if (!isInit)
    return false;
  DEBUG_PRINT("AUDIO TEST_PASSED\n");
  return true;
}

void audio_deckTask(void* arg){ // main task
  systemWaitStart();
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  if (use_iir){
      //i2cdevRead(I2C1_DEV, AUDIO_DECK_ADDRESS, TOTAL_N_BYTES, byte_array_received); // get array from deck for initialization
      //spiRead(&byte_array_received, TOTAL_N_BYTES);
      byte_array_to_float_array(float_array_averaged, byte_array_received, AUDIO_N_FLOATS);

  }

  memset(spi_write_array, 0x00, SPI_N_BYTES);

  while (1) {
      vTaskDelayUntil(&xLastWakeTime, F2T(AUDIO_TASK_FREQUENCY));
      vTaskDelay(M2T(300)); // wait for 0.3 seconds;

      if (debug) {
          uint8_t test_readwrite_array[SPI_N_BYTES];
          test_readwrite_array[0] = 0xFF;

          spi_write_array[0] = xTaskGetTickCount() % 0xFF;
          uint8_t success_readwrite = spiReadWrite(&test_readwrite_array, &spi_write_array, SPI_N_BYTES);
          DEBUG_PRINT("SPI %d: read %d, write %d \n", success_readwrite, test_readwrite_array[0], spi_write_array[0]);
      }

      continue;

      uint8_t success_write = spiWrite(&spi_write_array, SPI_N_BYTES);
      DEBUG_PRINT("SPI %d: writing %d \n", success_write, spi_write_array[0]);

      uint8_t test_read_array[1] = {0x00};
      uint8_t success_read = spiRead(&test_read_array, SPI_N_BYTES);
      DEBUG_PRINT("SPI %d: read %d \n", success_read, test_read_array[0]);
      if (state == SEND_FIRST_PACKET){

          if (send_audio_enable) {
            float_array_to_byte_array(float_array_averaged, byte_array_CRTP); // transfer the averaged audio

            if(!use_iir){
                erase_buffer(float_array_averaged, AUDIO_N_FLOATS);
            }
            send_audio_packet(1); // first packet is sent in channel 1 (start condition)

            state = SEND_AUDIO_PACKET;
          }

          send_param_I2C(); // send parameters to the audio deck with I2C
      }
      else if(state == SEND_AUDIO_PACKET){

          // TODO(FD): make sure below works for ma_window=1.
          if ((packet_count_audio % I2C_REQUEST_RATE == 0) &&
              (packet_count_audio >= AUDIO_N_PACKETS - I2C_REQUEST_RATE * ma_window || use_iir)){
              // we average before sending, and calls are separated with I2C_REQUEST_RATE cycles
              receive_audio_deck_array();
          }
          send_audio_packet(0);

      }
      else if (state == SEND_FBIN_PACKET){
          send_fbin_packet();
      }
  }
}

static const DeckDriver audio_deck = {
    // .vid = 0xFF, // write here id for detection of the board
    // .pid = 0xFF,
    .name = "audio_deck",
    .usedGpio = DECK_USING_SDA|DECK_USING_SCL,
    .init = audio_deckInit,
    .test = audio_deckTest,
};

DECK_DRIVER(audio_deck);

PARAM_GROUP_START(audio)
PARAM_ADD(PARAM_UINT8, debug, &debug)
PARAM_ADD(PARAM_UINT8, send_audio_enable, &send_audio_enable)
PARAM_ADD(PARAM_UINT8, use_iir, &use_iir)
PARAM_ADD(PARAM_FLOAT, alpha_iir, &alpha_iir)
PARAM_ADD(PARAM_UINT8, ma_window, &ma_window)
PARAM_ADD(PARAM_UINT8, filter_prop_enable, &filter_propellers_enable)
PARAM_ADD(PARAM_UINT8, filter_snr_enable, &filter_snr_enable)
PARAM_ADD(PARAM_UINT16, min_freq, &min_freq)
PARAM_ADD(PARAM_UINT16, max_freq, &max_freq)
PARAM_GROUP_STOP(audio)
