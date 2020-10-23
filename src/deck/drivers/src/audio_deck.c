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
#include "param.h"
#include "crtp.h"
#include "usec_time.h"
#include "power_distribution.h"

#include "sleepus.h"

#include "audio_deck.h"

////////////////////////////////////// DEFINES /////////////////////////////////

// debugging
//#define USE_TEST_SIGNALS
//#define DEBUG_SPI // set this define to use smaller, fixed buffers.
#ifdef USE_TEST_SIGNALS
#include "audio_debug_data.h"
#endif

//#define SYNCH_CHECK

// constants
#define INT16_PRECISION 2 // int16 = 2 bytes
#define FLOAT_PRECISION 4// float32 = 4 bytes
#define CRTP_MAX_PAYLOAD 29 // data bytes per crtp packet

// audio deck parameters
#define SYNCH_PIN DECK_GPIO_IO4
#define FFTSIZE 32
#define N_MICS 4
#define AUDIO_DECK_ADDRESS 47// I2C adress of the deck
#define N_MOTORS 4


// The maximum update rate we can expect at the ROS level is
// AUDIO_TASK_FREQUENCY / TOTAL_N_PACKETS (currently 36+3=39), so:
// 1000: 25 Hz
// 500: 12 Hz
// 300: 7 Hz
// 100: 2.5 Hz
//
// Observations:
// at 1000 we have packet loss
// at 500 we sometimes have packet loss, with an update rate of ca. 3 Hz.
// at 300 we have an even higher rate of almost 7Hz
#define AUDIO_TASK_FREQUENCY 300 // frequency at which packets are sent [Hz]
#define SIZE_OF_PARAM_I2C 5 // in uint16, min_freq = 1, max_freq = 1, delta_freq = 1, n_average = 1, snr + propeller enable = 1

// buffer sizes
#define PARAM_N_INTS (N_MOTORS + SIZE_OF_PARAM_I2C) // in uint16, n_motors for the current trhust commands
#define PARAM_N_BYTES (PARAM_N_INTS * INT16_PRECISION) // 14 bytes

#define AUDIO_N_FLOATS (N_MICS * FFTSIZE * 2) // *2 for complex numbers
#define AUDIO_N_BYTES (AUDIO_N_FLOATS * FLOAT_PRECISION)
#define AUDIO_N_PACKETS_FULL ((int) AUDIO_N_BYTES / CRTP_MAX_PAYLOAD)
#define AUDIO_N_PACKETS (AUDIO_N_PACKETS_FULL + 1) // 36

// We add the timestamp (float32) at the end of the fbins data.
#define TIMESTAMP_N_BYTES 4
#define FBINS_N_INTS FFTSIZE
#define FBINS_N_BYTES (FBINS_N_INTS * INT16_PRECISION + TIMESTAMP_N_BYTES)
#define FBINS_N_PACKETS_FULL ((int) FBINS_N_BYTES / CRTP_MAX_PAYLOAD)
#define FBINS_N_PACKETS (FBINS_N_PACKETS_FULL + 1) // 3

#define TOTAL_N_BYTES (FBINS_N_BYTES + AUDIO_N_BYTES) // 1092

// because we have more bytes of audio data than parameter data, +1 for checksum
#define CHECKSUM_VALUE 0xAB
#define CHECKSUM_LENGTH 1

#define MULTIPLIER 100 // how often we want to attempt SPI communication (AUDIO_TASK_FREQUENCY / MULTIPLIER)

#ifdef DEBUG_SPI
#define SPI_N_BYTES 100
#else
#define SPI_N_BYTES (TOTAL_N_BYTES + CHECKSUM_LENGTH)
#endif
///////////////////////////////////////// GENERAL    ////////////////////////////////////
static bool isInit;

///////////////////////////////////////// PARAMETERS ////////////////////////////////////

// general parameter
static bool send_audio_enable = false; // enables the sending of CRTP packets with the audio data
static bool new_data_to_send = false;

// frequency selection parameters
static uint16_t min_freq = 100;
static uint16_t max_freq = 10000;
static uint16_t delta_freq = 100;
static uint16_t n_average = 1;
static bool filter_propellers_enable = false;
static bool filter_snr_enable = false;

////////////////////////////////////// CRTP COMMUNICATION /////////////////////////////////
static enum {
	SEND_FIRST_PACKET, SEND_FOLLOWING_PACKET, SEND_FBIN_PACKET
} state = SEND_FIRST_PACKET;

static uint8_t crtp_tx_buffer[SPI_N_BYTES]; // buffer with data to be sent through CRTP
static uint16_t param_buffer_uint16[PARAM_N_INTS]; // buffer with the current parameters

static uint8_t packet_count_audio = 0;
static uint8_t packet_count_fbins = 0;

////////////////////////////////////// SPI COMMUNICATION / ///////////////////////////////////

static uint16_t spi_speed = SPI_BAUDRATE_2MHZ;
static uint8_t spi_tx_buffer[SPI_N_BYTES];
static uint8_t spi_rx_buffer[SPI_N_BYTES];
static uint8_t temp_spi_rx_buffer[SPI_N_BYTES];
static int multiplier = 0;

////////////////////////////////////////// DEBUGGING  ///////////////////////////////////////
uint8_t tx_counter = 0;

/////////////////////////////// AUDIO DECK FUNCTIONS  ///////////////////////////////////////

/**
 *  Fill frequency data after the audio data in the CRTP packet.
 */
void fill_packet_data_fbins(uint8_t packet_data[], uint8_t packet_count, uint8_t packet_size) {
	for (uint8_t i = 0; i < packet_size; i++) {
#ifndef DEBUG_SPI
		packet_data[i] = crtp_tx_buffer[AUDIO_N_BYTES + packet_count * CRTP_MAX_PAYLOAD + i];
#endif
	}
	//memcpy(packet_data, &crtp_tx_buffer[AUDIO_N_BYTES + packet_count * CRTP_MAX_PAYLOAD], packet_size);
}

/**
 *  Fill a new CRTP packet with the corresponding data from crtp_tx_buffer.
 */
void fill_packet_data_audio(uint8_t packet_data[], uint8_t packet_count, uint8_t packet_size) {
	for (uint8_t i = 0; i < packet_size; i++) {
		packet_data[i] = crtp_tx_buffer[packet_count * CRTP_MAX_PAYLOAD + i];
	}
	//memcpy(packet_data, &crtp_tx_buffer[packet_count * CRTP_MAX_PAYLOAD], packet_size);
}

uint8_t send_audio_packet(uint8_t channel) {
	static CRTPPacket signal_array_p;
	signal_array_p.header = CRTP_HEADER(CRTP_PORT_AUDIO, channel);
	signal_array_p.size = CRTP_MAX_PAYLOAD;

	if (packet_count_audio == AUDIO_N_PACKETS_FULL) { // send last packet and reset counter
		fill_packet_data_audio(signal_array_p.data, packet_count_audio,
				AUDIO_N_BYTES % CRTP_MAX_PAYLOAD);
		if (crtpSendPacket(&signal_array_p) == errQUEUE_FULL) {
			DEBUG_PRINT("errQUEUE_FULL audio packet %d\n", packet_count_audio);
			return 0;
		}
		else {
			packet_count_audio = 0;
			return 1;
		}
	} else { // send full packet
		fill_packet_data_audio(signal_array_p.data, packet_count_audio,
				CRTP_MAX_PAYLOAD);
		if (crtpSendPacket(&signal_array_p) == errQUEUE_FULL) {
			DEBUG_PRINT("errQUEUE_FULL audio packet %d\n", packet_count_audio);
			return 0;
		}
		else {
			packet_count_audio++;
			return 0;
		}
	}
}

uint8_t send_fbin_packet() {
	static CRTPPacket fbin_array_p;
	fbin_array_p.header = CRTP_HEADER(CRTP_PORT_AUDIO, 2);
	fbin_array_p.size = CRTP_MAX_PAYLOAD;

	if (packet_count_fbins == FBINS_N_PACKETS_FULL) { // send last packet and reset counter
		fill_packet_data_fbins(fbin_array_p.data, packet_count_fbins,
				FBINS_N_BYTES % CRTP_MAX_PAYLOAD);
		if (crtpSendPacket(&fbin_array_p) == errQUEUE_FULL) {
			DEBUG_PRINT("errQUEUE_FULL fbins packet %d\n", packet_count_fbins);
			return 0;
		}
		else {
			packet_count_fbins = 0;
			return 1;
		}
	} else { // send full packet
		fill_packet_data_fbins(fbin_array_p.data, packet_count_fbins,
				CRTP_MAX_PAYLOAD);
		if (crtpSendPacket(&fbin_array_p) == errQUEUE_FULL) {
			DEBUG_PRINT("errQUEUE_FULL fbins packet %d\n", packet_count_fbins);
			return 0;
		}
		else {
			packet_count_fbins++;
			return 0;
		}
	}
}


void fill_tx_buffer() {
	if (filter_propellers_enable) {
		uint16_t *motorPower_p = get_motor_power();
		memcpy(param_buffer_uint16, motorPower_p, N_MOTORS*2);
	} else {
		memset(param_buffer_uint16, 0x00, N_MOTORS*2);
	}

	param_buffer_uint16[N_MOTORS] = min_freq;
	param_buffer_uint16[N_MOTORS + 1] = max_freq;
	param_buffer_uint16[N_MOTORS + 2] = delta_freq;
	param_buffer_uint16[N_MOTORS + 3] = n_average;

	uint16_t enables = (filter_propellers_enable << 8) | filter_snr_enable;
	param_buffer_uint16[N_MOTORS + 4] = enables;

#ifndef DEBUG_SPI
	memcpy(spi_tx_buffer, (uint8_t*) param_buffer_uint16, PARAM_N_BYTES);
	spi_tx_buffer[SPI_N_BYTES - 1] = CHECKSUM_VALUE;
#endif
}

/** Exchange current audio data from the audio deck (signals and frequency bins)
 *  and parameters.
 */
bool exchange_data_audio_deck() {

#ifdef USE_TEST_SIGNALS
	vTaskDelay(M2T(100)); // ms
	// TODO(FD): check that this conversino works properly.
	memcpy(spi_rx_buffer, audio_data, AUDIO_N_BYTES);
	memcpy(spi_rx_buffer[AUDIO_N_BYTES], frequencies, FBINS_N_BYTES);
	return 1;
#else

	spiBeginTransaction(spi_speed);
	//digitalWrite(SYNCH_PIN, LOW);
	//sleepus(50);

#ifdef SYNCH_CHECK
	//uint8_t tx_synch = 0xDF;
	//uint8_t rx_synch = 0;
	//spiExchange(1, &tx_synch, &rx_synch);

	uint8_t rx_synch = 0;
	uint8_t tx_synch = 0;
	uint32_t waiting = 0;
	uint32_t timeout = 100000;
	while (rx_synch != 0xDF) {
		spiExchange(1, &tx_synch, &rx_synch);
		waiting += 1;
		if (waiting > timeout)
			break;
	}
	waiting = 0;
#endif

	spiExchange(SPI_N_BYTES, spi_tx_buffer, temp_spi_rx_buffer);

	//digitalWrite(SYNCH_PIN, HIGH);
	spiEndTransaction();
	//sleepus(50);

	// Only overwrite previous spi_rx_buffer if the checksum value is verified.
	if (temp_spi_rx_buffer[SPI_N_BYTES - 1] == CHECKSUM_VALUE) {
		memcpy(spi_rx_buffer, temp_spi_rx_buffer, SPI_N_BYTES);
		return true;
	}
	return false;
#endif
}


//////////////////////////// CRAZYFLIE FUNCTIONS  ////////////////////////////////////

void audio_deckInit(DeckInfo *info) { // deck initialisation
	if (isInit)
		return;
	DEBUG_PRINT("AUDIO INIT\n");

	spiBegin();

	pinMode(SYNCH_PIN, INPUT);

	xTaskCreate(audio_deckTask, AUDIO_TASK_NAME, AUDIO_TASK_STACKSIZE, NULL,
			AUDIO_TASK_PRI, NULL);

	isInit = true;
}

bool audio_deckTest(void) { // deck test
	if (!isInit)
		return false;
	DEBUG_PRINT("AUDIO TEST_PASSED\n");
	return true;
}

void audio_deckTask(void *arg) { // main task
	systemWaitStart();
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	DEBUG_PRINT("AUDIO TASK_STARTED \n");

	memset(spi_tx_buffer, 0x00, SPI_N_BYTES);
	fill_tx_buffer();

	memset(spi_rx_buffer, 0x00, SPI_N_BYTES);

#ifdef DEBUG_SPI
	int i = 0;
	for (int j = SPI_N_BYTES; j > 0;  j--) {
		spi_tx_buffer[i] = (uint8_t) (j % 0xFF);
		i++;
	}
	spi_tx_buffer[0] = 0xDF; // 223//(tx_counter++) % 0xFF; //xTaskGetTickCount() % 0xFF;
	spi_tx_buffer[SPI_N_BYTES-1] = CHECKSUM_VALUE;
#endif

	//digitalWrite(SYNCH_PIN, HIGH);

	while (1) {
		vTaskDelayUntil(&xLastWakeTime, F2T(AUDIO_TASK_FREQUENCY));
		multiplier += 1;

		// digitalWrite(SYNCH_PIN, HIGH);

		// fill the parameter buffer with the current parameters,
		// will stay constant throughout the sending of this
		// audio packet.
		fill_tx_buffer();

		//if (multiplier == MULTIPLIER) {
		if (digitalRead(SYNCH_PIN)) {
			if (exchange_data_audio_deck()) {
				new_data_to_send = true;
			}
			else {
				//DEBUG_PRINT("CHECKSUM fail, did not update spi_rx_buffer\n");
			}
			multiplier = 0;
		}

		// send audio data over CRTP
		if (state == SEND_FIRST_PACKET) {
			if (send_audio_enable && new_data_to_send) {
				// fill the crtp_tx_buffer with what we have received so far
				memcpy(crtp_tx_buffer, spi_rx_buffer, SPI_N_BYTES);
				new_data_to_send = false;

				send_audio_packet(1); // first packet is sent in channel 1 (start condition)
				state = SEND_FOLLOWING_PACKET;
			}
		} else if (state == SEND_FOLLOWING_PACKET) {
			if(send_audio_packet(0)){
				state = SEND_FBIN_PACKET;
			}
		} else if (state == SEND_FBIN_PACKET) {
			if(send_fbin_packet()){
				state = SEND_FIRST_PACKET;
			}
		}
	}
	DEBUG_PRINT("WARNING: Exiting loop \n");
}

static const DeckDriver audio_deck = {
		// .vid = 0xFF, // write here id for detection of the board
		// .pid = 0xFF,
		.name = "audio_deck", .usedGpio = DECK_USING_SDA | DECK_USING_SCL,
		.init = audio_deckInit, .test = audio_deckTest, };

DECK_DRIVER(audio_deck);

PARAM_GROUP_START(audio)
PARAM_ADD(PARAM_UINT8, send_audio_enable, &send_audio_enable)
PARAM_ADD(PARAM_UINT16, min_freq, &min_freq)
PARAM_ADD(PARAM_UINT16, max_freq, &max_freq)
PARAM_ADD(PARAM_UINT16, delta_freq, &delta_freq)
PARAM_ADD(PARAM_UINT16, n_average, &n_average)
PARAM_ADD(PARAM_UINT8, filter_prop_enable, &filter_propellers_enable)
PARAM_ADD(PARAM_UINT8, filter_snr_enable, &filter_snr_enable)
PARAM_GROUP_STOP(audio)
