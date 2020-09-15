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

#include "audio_deck.h"

////////////////////////////////////// DEFINES /////////////////////////////////

// debugging
//#define USE_TEST_SIGNALS
#define DEBUG_SPI
#ifdef USE_TEST_SIGNALS
#include "audio_debug_data.h"
#endif

// constants
#define INT16_PRECISION 2 // int16 = 2 bytes
#define FLOAT_PRECISION 4// float32 = 4 bytes
#define CRTP_MAX_PAYLOAD 29

// audio deck parameters
#define SYNCH_PIN DECK_GPIO_IO4
#define FFTSIZE 32
#define N_MICS 4
#define AUDIO_DECK_ADDRESS 47// I2C adress of the deck
#define N_MOTORS 4
#define AUDIO_TASK_FREQUENCY 100 // frequency at which packets are sent [Hz]
#define SIZE_OF_PARAM_I2C 5 // in uint16, min_freq = 1, max_freq = 1, delta_freq = 1, n_average = 1, snr + propeller enable = 1

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

// because we have more bytes of audio data than parameter data, +1 for checksum
#define CHECKSUM_VALUE 0xAB
#define CHECKSUM_LENGTH 1


#ifdef DEBUG_SPI
#define SPI_N_BYTES 100
#else
#define SPI_N_BYTES (TOTAL_N_BYTES + CHECKSUM_LENGTH)
#endif
///////////////////////////////////////// GENERAL    ////////////////////////////////////
static bool isInit;

///////////////////////////////////////// PARAMETERS ////////////////////////////////////

// general parameter
uint8_t debug = 1; // enables DEBUG_PRINT
static bool send_audio_enable = 0; // enables the sending of CRTP packets with the audio data

// frequency selection parameters
static uint16_t min_freq = 100;
static uint16_t max_freq = 10000;
static uint16_t delta_freq = 100;
static uint16_t n_average = 1; // number of arrays averaged, maximum is 6 as 6*6 = 36 = AUDIO_N_PACKETS
static bool filter_propellers_enable = 0;
static bool filter_snr_enable = 0;

////////////////////////////////////// CRTP COMMUNICATION /////////////////////////////////
static enum {
	SEND_FIRST_PACKET, SEND_FOLLOWING_PACKET, SEND_FBIN_PACKET
} state = SEND_FIRST_PACKET;

static uint8_t crtp_tx_buffer[SPI_N_BYTES]; // buffer with data to be sent through CRTP
static uint16_t param_buffer_uint16[PARAM_N_INTS]; // buffer with the current parameters

static uint8_t packet_count_audio = 0;
static uint8_t packet_count_fbins = 0;

////////////////////////////////////// SPI COMMUNICATION / ///////////////////////////////////

//static uint16_t spi_speed = SPI_BAUDRATE_21MHZ;
//static uint16_t spi_speed = SPI_BAUDRATE_6MHZ;
static uint16_t spi_speed = SPI_BAUDRATE_2MHZ;
static uint8_t spi_tx_buffer[SPI_N_BYTES];
static uint8_t spi_rx_buffer[SPI_N_BYTES];

int pin_state = 0;

//TODO(FD) not sure if we need these temporary buffers
static uint8_t temp_spi_tx_buffer[SPI_N_BYTES];
static uint8_t temp_spi_rx_buffer[SPI_N_BYTES];


////////////////////////////////////////// DEBUGGING  ///////////////////////////////////////
uint8_t tx_counter = 0;

/////////////////////////////// AUDIO DECK FUNCTIONS  ///////////////////////////////////////

static uint8_t spiReadWrite(void *data_read, const void *data_write,
		size_t data_length) {
	spiBeginTransaction(spi_speed);
	memcpy(temp_spi_tx_buffer, data_write, data_length);
	uint8_t success = spiExchange(data_length, temp_spi_tx_buffer,
			temp_spi_rx_buffer);
	//uint8_t success = spiExchange(data_length, data_write, data_read);
	spiEndTransaction();
	memcpy(data_read, temp_spi_rx_buffer, data_length);
	return success;
}

static uint8_t spiRead(void *data_read, size_t data_length) {
	spiBeginTransaction(spi_speed);
	memset(temp_spi_tx_buffer, 0, data_length);
	uint8_t success = spiExchange(data_length, temp_spi_tx_buffer,
			temp_spi_rx_buffer);
	spiEndTransaction();
	memcpy(data_read, temp_spi_rx_buffer, data_length);
	return success;
}

static uint8_t spiWrite(const void *data_write,
		size_t data_length) {
	spiBeginTransaction(spi_speed);
	memcpy(temp_spi_tx_buffer, data_write, data_length);
	uint8_t success = spiExchange(data_length, temp_spi_tx_buffer,
			temp_spi_rx_buffer);
	spiEndTransaction();
	return success;
}


/**
 *  Fill frequency data after the audio data in the CRTP packet.
 */
void fill_packet_data_fbins(uint8_t packet_data[], uint8_t packet_count, uint8_t packet_size) {
	for (uint8_t i = 0; i < packet_size; i++) {
#ifndef DEBUG_SPI
		packet_data[i] = crtp_tx_buffer[AUDIO_N_BYTES + packet_count * CRTP_MAX_PAYLOAD + i];
#endif
	}
}

/**
 *  Fill a new CRTP packet with the corresponding data from crtp_tx_buffer.
 */
void fill_packet_data_audio(uint8_t packet_data[], uint8_t packet_count, uint8_t packet_size) {
	for (uint8_t i = 0; i < packet_size; i++) {
		packet_data[i] = crtp_tx_buffer[packet_count * CRTP_MAX_PAYLOAD + i];
	}
}

uint8_t send_audio_packet(uint8_t channel) {
	static CRTPPacket signal_array_p;
	signal_array_p.header = CRTP_HEADER(CRTP_PORT_AUDIO, channel);
	signal_array_p.size = CRTP_MAX_PAYLOAD;

	if (packet_count_audio == AUDIO_N_PACKETS_FULL) { // send last packet and reset counter
		fill_packet_data_audio(signal_array_p.data, packet_count_audio,
				AUDIO_N_BYTES % CRTP_MAX_PAYLOAD);
		crtpSendPacket(&signal_array_p);
		packet_count_audio = 0;
		return 1;
	} else { // send full packet
		fill_packet_data_audio(signal_array_p.data, packet_count_audio,
				CRTP_MAX_PAYLOAD);
		crtpSendPacket(&signal_array_p);
		packet_count_audio++;
		return 0;
	}
}

uint8_t send_fbin_packet() {
	static CRTPPacket fbin_array_p;
	fbin_array_p.header = CRTP_HEADER(CRTP_PORT_AUDIO, 2);
	fbin_array_p.size = CRTP_MAX_PAYLOAD;

	if (packet_count_fbins == FBINS_N_PACKETS_FULL) { // send last packet and reset counter
		fill_packet_data_fbins(fbin_array_p.data, packet_count_fbins,
				FBINS_N_BYTES % CRTP_MAX_PAYLOAD);
		crtpSendPacket(&fbin_array_p);
		packet_count_fbins = 0;
		return 1;
	} else { // send full packet
		fill_packet_data_fbins(fbin_array_p.data, packet_count_fbins,
				CRTP_MAX_PAYLOAD);
		crtpSendPacket(&fbin_array_p);
		packet_count_fbins++;
		return 0;
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
#endif
}

/** Exchange current audio data from the audio deck (signals and frequency bins)
 *  and parameters.
 */
void exchange_data_audio_deck() {

#ifdef USE_TEST_SIGNALS
	vTaskDelay(M2T(100)); // ms
	float_array_to_byte_array(audio_data, spi_rx_buffer);
	uint_array_to_byte_array(frequencies, spi_rx_buffer[AUDIO_N_BYTES]);
	uint8_t success = 1;
#else
	fill_tx_buffer();
	uint8_t success = spiReadWrite(&spi_rx_buffer, &spi_tx_buffer, SPI_N_BYTES);
	if (!success) {
		DEBUG_PRINT("!!!!! AUDIO SPI fail !!!!\n");
	}
//	DEBUG_PRINT(
//			"SPI success %d: read [%4d,%4d,...,%4d] write [%4d,%4d,...,%4d] \n",
//			success, (uint8_t ) spi_rx_buffer[0],
//			(uint8_t ) spi_rx_buffer[1], (uint8_t) spi_rx_buffer[SPI_N_BYTES-1],
//			(uint8_t ) spi_tx_buffer[0], (uint8_t ) spi_tx_buffer[1],
//			(uint8_t) spi_tx_buffer[SPI_N_BYTES-1]);
#endif
}


//////////////////////////// CRAZYFLIE FUNCTIONS  ////////////////////////////////////

void audio_deckInit(DeckInfo *info) { // deck initialisation
	if (isInit)
		return;
	DEBUG_PRINT("AUDIO INIT\n");

	spiBegin();

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
	memset(spi_rx_buffer, 0x00, SPI_N_BYTES);
	uint32_t t1 = 0;
	uint32_t t2 = 0;

	int i = 0;
	for (int j = SPI_N_BYTES; j > 0;  j--) {
		spi_tx_buffer[i] = (uint8_t) (j % 0xFF);
		i++;
	}
	spi_tx_buffer[0] = 0xDF; // 223//(tx_counter++) % 0xFF; //xTaskGetTickCount() % 0xFF;
	spi_tx_buffer[SPI_N_BYTES-1] = CHECKSUM_VALUE;

	while (1) {
		//DEBUG_PRINT("time is %d\n", T2M(xTaskGetTickCount()));
		vTaskDelayUntil(&xLastWakeTime, F2T(AUDIO_TASK_FREQUENCY));

		if (debug) {
			//memset(spi_tx_buffer, 0x04, SPI_N_BYTES);
			memset(spi_rx_buffer, 0xFF, SPI_N_BYTES);

			t1 = xTaskGetTickCount();
			uint8_t counter = 0;
			uint8_t success = 0;
//			while (digitalRead(SYNCH_PIN)) {
//				if (counter == 0) {
//					//success = spiWrite(spi_tx_buffer, SPI_N_BYTES);
//					success = spiRead(spi_rx_buffer, SPI_N_BYTES);
//					//success = spiReadWrite(spi_rx_buffer, spi_tx_buffer, SPI_N_BYTES);
//					//if (spi_rx_buffer[SPI_N_BYTES-1] == CHECKSUM_VALUE) {
//				}
//				counter ++;
//			}
			if (digitalRead(SYNCH_PIN)) {
				success = spiReadWrite(spi_rx_buffer, spi_tx_buffer, SPI_N_BYTES);
			}
			t2 = xTaskGetTickCount();
			if (success) {
				DEBUG_PRINT(
						"pin state 1, SPI success %d, counter: %d, %d: data write [%4d,%4d,%4d,%4d,%4d], data read [%4d,%4d,%4d,%4d,%4d]\n",
						success, counter, T2M(t2-t1),
						spi_tx_buffer[0], spi_tx_buffer[1], spi_tx_buffer[2], spi_tx_buffer[3], spi_tx_buffer[SPI_N_BYTES-1],
						spi_rx_buffer[0], spi_rx_buffer[1], spi_rx_buffer[2], spi_rx_buffer[3], spi_rx_buffer[SPI_N_BYTES-1]);
			}
				//vTaskDelay(M2T(50));
		} else {
			//TODO(FD) remove!!!
			//memset(spi_rx_buffer, 0xFF, SPI_N_BYTES);
			uint8_t success_write = spiWrite(spi_tx_buffer, SPI_N_BYTES);
			uint8_t success_read = spiRead(spi_rx_buffer, SPI_N_BYTES);
			success_write = success_write;
			success_write = success_read;
			//if (spi_rx_buffer[SPI_N_BYTES-1] == CHECKSUM_VALUE) {

			if (pin_state) {
				exchange_data_audio_deck();
			}

			if (state == SEND_FIRST_PACKET) {
				if (send_audio_enable) {

					// fill the crtp_tx_buffer with what we have received so far
					memcpy(crtp_tx_buffer, spi_rx_buffer, SPI_N_BYTES);

				//	DEBUG_PRINT("Sending first audio packet; %d, %d, %d, %d \n",
				//			(uint8_t) crtp_tx_buffer[0],
				//			(uint8_t) crtp_tx_buffer[1],
				//			(uint8_t) crtp_tx_buffer[2],
				//			(uint8_t) crtp_tx_buffer[3]);

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
	}
	DEBUG_PRINT("WARNING: Exiting loop \n");
}

static const DeckDriver audio_deck = {
		// .vid = 0xFF, // write here id for detection of the board
		// .pid = 0xFF,
		.name = "audio_deck", .usedGpio = DECK_USING_SDA | DECK_USING_SCL,
		.init = audio_deckInit, .test = audio_deckTest, };

DECK_DRIVER(audio_deck);

PARAM_GROUP_START(audio) PARAM_ADD(PARAM_UINT8, debug, &debug)
PARAM_ADD(PARAM_UINT8, send_audio_enable, &send_audio_enable)
PARAM_ADD(PARAM_UINT16, min_freq, &min_freq)
PARAM_ADD(PARAM_UINT16, max_freq, &max_freq)
PARAM_ADD(PARAM_UINT16, delta_freq, &delta_freq)
PARAM_ADD(PARAM_UINT16, n_average, &n_average)
PARAM_ADD(PARAM_UINT8, filter_prop_enable, &filter_propellers_enable)
PARAM_ADD(PARAM_UINT8, filter_snr_enable, &filter_snr_enable)
PARAM_GROUP_STOP(audio)
