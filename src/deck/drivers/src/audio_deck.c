#define DEBUG_MODULE "AUDIO_DECK"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "stm32fxxx.h"
#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "debug.h"
#include "deck.h"
#include "log.h"
#include "param.h"
#include "crtp.h"
#include "usec_time.h"
#include "power_distribution.h"
#include "motors.h"
#include "sound.h"

#include "audio_deck.h"

////////////////////////////////////// DEFINES /////////////////////////////////

// debugging
//#define USE_TEST_SIGNALS
//#define DEBUG_SPI // set this define to use smaller, fixed buffers.
#ifdef USE_TEST_SIGNALS
#include "audio_debug_data.h"
#endif

// constants
#define INT16_PRECISION 2 // int16 = 2 bytes
#define FLOAT_PRECISION 4// float32 = 4 bytes
#define CRTP_MAX_PAYLOAD 29 // data bytes per crtp packet
#define SYNCH_PIN DECK_GPIO_IO4
#define FFTSIZE 32
#define N_MICS 4
#define N_MOTORS 4
#define FS 32000
#define N_BUFFER 2048


// The maximum update rate we can expect at the ROS level is
// AUDIO_TASK_FREQUENCY / TOTAL_N_PACKETS (currently 36+3=39), so:
// 1000: 25 Hz, CRTP packet loss
// 500: 12.8 Hz. sometimes CRTP packet loss
// 300: 7.7 Hz, no packet loss
// 100: 2.5 Hz, no packet loss
//
// Note that sending 1089 bytes at the baudrate 6MHz over SPI takes
// ca. 1.67 ms (this includes delays introduced by response time etc,
// and was found with oscilloscope).
// That means we have an effective maximum task frequency of
// at most 1/1.67ms = 600Hz, which would lead to an update rate in ROS of 15Hz.
// However, we do not need that high of a rate because we only have
// new audio data ca. every 50ms, corresponding to 20Hz. Also, setting
// the rate to this maximum would mean there is no time left to do
// anything else, so it is not desirable.
#define AUDIO_TASK_FREQUENCY 300 // frequency at which packets are sent [Hz]

// buffer sizes
// parameters include: current thrusts and min_freq, max_freq, buzzer_freq_idx, delta_freq, n_average, snr, propeller, window_type: total 8
#define PARAM_N_INTS (N_MOTORS + 8)
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

// TODO(FD) this is not a real checksum, but just a fixed value. Should change name.
#define CHECKSUM_VALUE 0xAC

#ifdef DEBUG_SPI
#define SPI_N_BYTES 1100
#else
#define SPI_N_BYTES (TOTAL_N_BYTES + 1) // + 1 for checksum, which is added in the end of buffer
#endif
///////////////////////////////////////// GENERAL    ////////////////////////////////////
static bool isInit;

///////////////////////////////////////// PARAMETERS ////////////////////////////////////

// general parameter
static bool send_audio_enable = false; // enables the sending of CRTP packets with the audio data
static bool new_data_to_send = false;

// frequency selection parameters
static uint16_t min_freq = 0;
static uint16_t max_freq = 0;
static uint16_t buzzer_freq_idx = 0;
static uint16_t delta_freq = 100;
static uint16_t n_average = 1;
static bool filter_prop_enable = false;
static uint16_t filter_snr_enable = 0;
static uint16_t window_type = 1; // 0: none, 1: hann, 2: flattop, 3: tukey(0.2)
static uint16_t motor_power_list[N_MOTORS];

////////////////////////////////////// CRTP COMMUNICATION /////////////////////////////////
static enum {
	SEND_FIRST_PACKET, SEND_FOLLOWING_PACKET, SEND_FBIN_PACKET
} state = SEND_FIRST_PACKET;

static uint8_t crtp_tx_buffer[SPI_N_BYTES]; // buffer with data to be sent through CRTP
static uint16_t param_buffer_uint16[PARAM_N_INTS]; // buffer with the current parameters
static uint8_t packet_count_audio = 0;
static uint8_t packet_count_fbins = 0;

////////////////////////////////////// SPI COMMUNICATION / ///////////////////////////////////

static uint16_t spi_speed = SPI_BAUDRATE_6MHZ;
static uint8_t spi_tx_buffer[SPI_N_BYTES];
static uint8_t spi_rx_buffer[SPI_N_BYTES];
static uint8_t temp_spi_rx_buffer[SPI_N_BYTES];

/////////////////////////////// AUDIO DECK FUNCTIONS  ///////////////////////////////////////

uint8_t send_audio_packet(uint8_t channel) {
	static CRTPPacket signal_array_p;
	signal_array_p.header = CRTP_HEADER(CRTP_PORT_AUDIO, channel);
	signal_array_p.size = CRTP_MAX_PAYLOAD;

	if (packet_count_audio == AUDIO_N_PACKETS_FULL) { // send last packet and reset counter
		memcpy(signal_array_p.data, &crtp_tx_buffer[packet_count_audio * CRTP_MAX_PAYLOAD], AUDIO_N_BYTES % CRTP_MAX_PAYLOAD);

		if (crtpSendPacket(&signal_array_p) == errQUEUE_FULL) {
			DEBUG_PRINT("Warning: errQUEUE_FULL audio packet %d\n", packet_count_audio);
		}

		packet_count_audio = 0;
		return 1;
	} else { // send full packet
		memcpy(signal_array_p.data, &crtp_tx_buffer[packet_count_audio * CRTP_MAX_PAYLOAD], CRTP_MAX_PAYLOAD);

		if (crtpSendPacket(&signal_array_p) == errQUEUE_FULL) {
			DEBUG_PRINT("Warning: errQUEUE_FULL audio packet %d\n", packet_count_audio);
		}

		packet_count_audio++;
		return 0;
	}
}

uint8_t send_fbin_packet() {
	static CRTPPacket fbin_array_p;
	fbin_array_p.header = CRTP_HEADER(CRTP_PORT_AUDIO, 2);
	fbin_array_p.size = CRTP_MAX_PAYLOAD;

	if (packet_count_fbins == FBINS_N_PACKETS_FULL) { // send last packet and reset counter
		memcpy(fbin_array_p.data, &crtp_tx_buffer[AUDIO_N_BYTES + packet_count_fbins * CRTP_MAX_PAYLOAD], FBINS_N_BYTES % CRTP_MAX_PAYLOAD);

		if (crtpSendPacket(&fbin_array_p) == errQUEUE_FULL) {
			DEBUG_PRINT("Warning: errQUEUE_FULL fbins packet %d\n", packet_count_fbins);
		}

		packet_count_fbins = 0;
		return 1;
	} else { // send full packet
		memcpy(fbin_array_p.data, &crtp_tx_buffer[AUDIO_N_BYTES + packet_count_fbins * CRTP_MAX_PAYLOAD], CRTP_MAX_PAYLOAD);

		if (crtpSendPacket(&fbin_array_p) == errQUEUE_FULL) {
			DEBUG_PRINT("Warning: errQUEUE_FULL fbins packet %d\n", packet_count_fbins);
		}

		packet_count_fbins++;
		return 0;
	}
}


void fill_param_buffer() {
	get_motor_power(&motor_power_list[0]);
	memcpy(&param_buffer_uint16[0], &motor_power_list[0], N_MOTORS*INT16_PRECISION);

	buzzer_freq_idx = (uint16_t) round((float) N_BUFFER * soundGetFreq() / FS );

	param_buffer_uint16[N_MOTORS] = min_freq;
	param_buffer_uint16[N_MOTORS + 1] = max_freq;
	param_buffer_uint16[N_MOTORS + 2] = buzzer_freq_idx;
	param_buffer_uint16[N_MOTORS + 3] = delta_freq;
	param_buffer_uint16[N_MOTORS + 4] = n_average;
	param_buffer_uint16[N_MOTORS + 5] = filter_prop_enable;
	param_buffer_uint16[N_MOTORS + 6] = filter_snr_enable;
	param_buffer_uint16[N_MOTORS + 7] = window_type;

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
	digitalWrite(SYNCH_PIN, LOW);

	spiExchange(SPI_N_BYTES, spi_tx_buffer, temp_spi_rx_buffer);

	digitalWrite(SYNCH_PIN, HIGH);
	spiEndTransaction();

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

	pinMode(SYNCH_PIN, OUTPUT);
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
	memset(motor_power_list, 0x00, N_MOTORS*INT16_PRECISION);
	fill_param_buffer();


#ifndef DEBUG_SPI
	memcpy(spi_tx_buffer, (uint8_t*) param_buffer_uint16, PARAM_N_BYTES);
#else
	int i = 0;
	for (int j = SPI_N_BYTES; j > 0;  j--) {
		spi_tx_buffer[i] = (uint8_t) (j % 0xFF); // start at SPI_N_BYTES % 255
		i++;
	}
#endif
	spi_tx_buffer[SPI_N_BYTES - 1] = CHECKSUM_VALUE;

	while (1) {

		// Note that we only delay if necessary, and there is no
		// warning if we have actually taken longer than allowed
		// for one period.
		vTaskDelayUntil(&xLastWakeTime, F2T(AUDIO_TASK_FREQUENCY));

		// fill the parameter buffer with the current parameters, to be sent
		// to audio deck.
		fill_param_buffer();

#ifdef DEBUG_SPI
		spi_tx_buffer[1] = (spi_tx_buffer[1] + 1) % 0xFF;
		if (exchange_data_audio_deck()) {
			spi_tx_buffer[0] = 0x01;
			new_data_to_send = true;
		}
		else {
			spi_tx_buffer[0] = 0x00;
		}
#else
		memcpy(spi_tx_buffer, (uint8_t*) param_buffer_uint16, PARAM_N_BYTES);

		// send parameters from audio deck and receive new parameters.
		new_data_to_send = exchange_data_audio_deck();
#endif

		if (!send_audio_enable) {
			state = SEND_FIRST_PACKET;
			packet_count_fbins = 0;
			packet_count_audio = 0;
		}

		// send audio data over CRTP
		if (state == SEND_FIRST_PACKET) {
			if (send_audio_enable && new_data_to_send) {

				//DEBUG_PRINT("Start sending one chunk of audio data... \n");

				// fill the crtp_tx_buffer with what we have received
				memcpy(crtp_tx_buffer, spi_rx_buffer, SPI_N_BYTES);
				new_data_to_send = false;

				send_audio_packet(1); // first packet is sent in channel 1 (start condition)
				state = SEND_FOLLOWING_PACKET;
			}
			else if (send_audio_enable) {
				DEBUG_PRINT("Want to send new data but don't have any! \n");
			}
		} else if (state == SEND_FOLLOWING_PACKET) {
			if(send_audio_packet(0)){
				state = SEND_FBIN_PACKET;
			}
		} else if (state == SEND_FBIN_PACKET) {
			if(send_fbin_packet()){
				//DEBUG_PRINT("... done sending one chunk of audio data. \n");
				state = SEND_FIRST_PACKET;
			}
		}
	}
	DEBUG_PRINT("WARNING: Exiting loop \n");
}

static const DeckDriver audio_deck = {
		.vid = 0xBC, .pid = 0xFF,
		.name = "audio_deck", .usedGpio = DECK_USING_SDA | DECK_USING_SCL,
		.init = audio_deckInit, .test = audio_deckTest, };

DECK_DRIVER(audio_deck);


LOG_GROUP_START(audio)
LOG_ADD(LOG_UINT8, send_audio_enable, &send_audio_enable)
LOG_ADD(LOG_UINT8, new_data_to_send, &new_data_to_send)
LOG_ADD(LOG_UINT8, first_element, &spi_rx_buffer[0])
LOG_ADD(LOG_UINT16, m1_thrust, &motor_power_list[0])
LOG_ADD(LOG_UINT16, m2_thrust, &motor_power_list[1])
LOG_ADD(LOG_UINT16, m3_thrust, &motor_power_list[2])
LOG_ADD(LOG_UINT16, m4_thrust, &motor_power_list[3])
LOG_GROUP_STOP(audio)


PARAM_GROUP_START(audio)
PARAM_ADD(PARAM_UINT8, send_audio_enable, &send_audio_enable)
PARAM_ADD(PARAM_UINT16, min_freq, &min_freq)
PARAM_ADD(PARAM_UINT16, max_freq, &max_freq)
PARAM_ADD(PARAM_UINT16, delta_freq, &delta_freq)
PARAM_ADD(PARAM_UINT16, n_average, &n_average)
PARAM_ADD(PARAM_UINT8, filter_prop_enable, &filter_prop_enable)
PARAM_ADD(PARAM_UINT8, filter_snr_enable, &filter_snr_enable)
PARAM_ADD(PARAM_UINT8, window_type, &window_type)
PARAM_GROUP_STOP(audio)
