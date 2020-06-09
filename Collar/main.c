/***************************************************************************//**
 * Collar
 ******************************************************************************/

/* Board Headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "infrastructure.h"

/* GATT database */
#include "gatt_db.h"

/* EM library (EMlib) */
#include "em_system.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"

/* Device initialization header */
#include "hal-config.h"

#ifdef FEATURE_BOARD_DETECTED
#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif
#else
#error This sample app only works with a Silicon Labs Board
#endif

#ifdef FEATURE_I2C_SENSOR
#include "i2cspm.h"
#include "si7013.h"
#include "tempsens.h"
#endif

#include "stdio.h"
#include "stdlib.h"
#include "retargetserial.h"

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

/* Gecko configuration parameters (see gecko_configuration.h) */
#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

static const gecko_configuration_t config = { .config_flags = 0,
#if defined(FEATURE_LFXO) || defined(PLFRCO_PRESENT) || defined(LFRCO_PRESENT)
		.sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
#else
		.sleep.flags = 0,
#endif
		.bluetooth.max_connections = MAX_CONNECTIONS, .bluetooth.heap =
				bluetooth_stack_heap, .bluetooth.heap_size =
				sizeof(bluetooth_stack_heap),
#if defined(FEATURE_LFXO)
		.bluetooth.sleep_clock_accuracy = 100, // ppm
#elif defined(PLFRCO_PRESENT) || defined(LFRCO_PRESENT)
		.bluetooth.sleep_clock_accuracy = 500, // ppm
#endif
		.gattdb = &bg_gattdb_data, .ota.flags = 0, .ota.device_name_len = 3,
		.ota.device_name_ptr = "OTA", .pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
		.pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
		.pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
		.rf.flags = GECKO_RF_CONFIG_ANTENNA, /* Enable antenna configuration. */
		.rf.antenna = GECKO_RF_ANTENNA, /* Select antenna path! */
};

/**
 * @brief Function for taking a single temperature measurement with the WSTK Relative Humidity and Temperature (RHT) sensor.
 */

uint32_t max_temperature = 0;
uint32_t temperature = 0;

void temperatureMeasure() {
	int32_t tempData = 10000; /* Stores the Temperature data read from the RHT sensor. */
	uint32_t rhData = 0; /* Dummy needed for storing Relative Humidity data. */

#ifdef FEATURE_I2C_SENSOR
	/* Sensor relative humidity and temperature measurement returns 0 on success, nonzero otherwise */
	if (!Si7013_MeasureRHAndTemp(I2C0, SI7021_ADDR, &rhData, &tempData) != 0)
#endif
			{
		/* Convert sensor data to correct temperature format */
		temperature = FLT_TO_UINT32(tempData, -3);
		//Store maximum temperature
		max_temperature = MAX(max_temperature, temperature);
	}
}

uint32_t simulateAccelerometer() {
	return rand() % (200000);
}

#define SYNC_TOKEN 16777215

void send_data() {
	temperatureMeasure();
	static uint32_t counter = -1; // Keeps track of what data is sent on each call to the function
	uint8_t htmTempBuffer[5]; /* Stores the temperature data in the Health Thermometer (HTM) format. */
	uint8_t flags = 0x00; /* HTM flags set as 0 for Celsius, no time stamp and no temperature type. */
	uint8_t *p = htmTempBuffer; /* Pointer to HTM temperature buffer needed for converting values to bitstream. */
	counter++;
	counter %= 4;
	/* Convert flags to bitstream and append them in the HTM temperature data buffer (htmTempBuffer) */
	UINT8_TO_BITSTREAM(p, flags);
	switch (counter) {
	case 0:
		UINT32_TO_BITSTREAM(p, SYNC_TOKEN)
		;
		break;
	case 1:
		UINT32_TO_BITSTREAM(p, temperature)
		;
		break;
	case 2:
		UINT32_TO_BITSTREAM(p, max_temperature)
		;
		break;
	case 3:
		UINT32_TO_BITSTREAM(p, simulateAccelerometer())
		;
		break;
	}

	/* Send indication of the temperature in htmTempBuffer to all "listening" clients.
	 * This enables the Health Thermometer in the Blue Gecko app to display the temperature.
	 *  0xFF as connection ID will send indications to all connections. */
	gecko_cmd_gatt_server_send_characteristic_notification(0xFF,
	gattdb_temperature_measurement, 5, htmTempBuffer);
	//Once max_temperature has been sent we reset the value and the same should be done with real accelerometer data
	max_temperature = counter ? max_temperature : 0;

}

/**
 * @brief  Main function
 */
int main(void) {
	// Initialize device
	initMcu();
	// Initialize board
	initBoard();
	// Initialize application
	initApp();
	initVcomEnable();
	RETARGET_SerialInit();

	// Initialize stack
	gecko_init(&config);
#ifdef FEATURE_I2C_SENSOR
	// Initialize the Temperature Sensor
	Si7013_Detect(I2C0, SI7021_ADDR, NULL);
#endif
	while (1) {
		/* Event pointer for handling events */
		struct gecko_cmd_packet* evt;

		/* Check for stack event. */
		evt = gecko_wait_event();

		/* Handle events */
		switch (BGLIB_MSG_ID(evt->header)) {
		/* This boot event is generated when the system boots up after reset.
		 * Do not call any stack commands before receiving the boot event.
		 * Here the system is set to start advertising immediately after boot procedure. */
		case gecko_evt_system_boot_id:
			/* Set advertising parameters. 100ms advertisement interval.
			 * The first two parameters are minimum and maximum advertising interval, both in
			 * units of (milliseconds * 1.6). */
			gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);

			/* Start general advertising and enable connections. */
			gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable,
					le_gap_connectable_scannable);
			break;

			/* This event is generated when a connected client has either
			 * 1) changed a Characteristic Client Configuration, meaning that they have enabled
			 * or disabled Notifications or Indications, or
			 * 2) sent a confirmation upon a successful reception of the indication. */
		case gecko_evt_gatt_server_characteristic_status_id:

			/* Check that the characteristic in question is temperature - its ID is defined
			 * in gatt.xml as "temperature_measurement". Also check that status_flags = 1, meaning that
			 * the characteristic client configuration was changed (notifications or indications
			 * enabled or disabled). */
			if ((evt->data.evt_gatt_server_characteristic_status.characteristic
					== gattdb_temperature_measurement)
					&& (evt->data.evt_gatt_server_characteristic_status.status_flags
							== 0x01)) {
				if (evt->data.evt_gatt_server_characteristic_status.client_config_flags
						== 0x02) {
					/* Indications have been turned ON - start the repeating timer. The 1st parameter '32768'
					 * tells the timer to run for 1 second (32.768 kHz oscillator), the 2nd parameter is
					 * the timer handle and the 3rd parameter '0' tells the timer to repeat continuously until
					 * stopped manually.*/
					gecko_cmd_hardware_set_soft_timer(32768, 0, 0);
				} else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags
						== 0x00) {
					/* Indications have been turned OFF - stop the timer. */
					gecko_cmd_hardware_set_soft_timer(0, 0, 0);
				}
			}
			send_data();
			break;

			/* This event is generated when the software timer has ticked. In this example the temperature
			 * is read after every 1 second and then the indication of that is sent to the listening client. */
		case gecko_evt_hardware_soft_timer_id:
			/* Measure the temperature as defined in the function temperatureMeasure() */
			temperatureMeasure();
			break;

		case gecko_evt_le_connection_closed_id:
			/* Stop timer in case client disconnected before indications were turned off */
			gecko_cmd_hardware_set_soft_timer(0, 0, 0);
			/* Restart advertising after client has disconnected */
			gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable,
					le_gap_connectable_scannable);

			break;
		default:
			break;
		}
	}
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
