/*
 * Heavily based on VirtualSerial demo. Original copyright (for that demo) follows.
 *
             LUFA Library
     Copyright (C) Dean Camera, 2012.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2012  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

#include "logger.h"

#include "onewire.h"
#include "ds18x20.h"
#include "i2cmaster.h"

//#include "diskio.h"
//#include "ff.h"
#include <avr/eeprom.h> 

#define MAXSENSORS 2

// Internal EEPROM settings
//#define EXTERNAL_EEPROM 0
//#define MAX_EEPROM_LOCATION 1023

// External EEPROM settings
#define EXTERNAL_EEPROM 1
#define MAX_EEPROM_LOCATION ((16 * 1024) - 1)
// Refer to EEPROM documentation.
#define EXTERNAL_EEPROM_24C 0xA0

uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];

uint16_t uptime_ms = 0;
//FATFS FATFS_Obj;

// SD card functions must be called every 10 ms, which, with a prescaler of /8, is 2000 ticks.
//#define SDCARD_TIMER1_DELAY (65536 - 20000)

// Changed this to a periodic timer every 1/10th of a second (and prescaler of /64)
#define TIMER1_DELAY (65536 - 25000)
#define MS_PER_TICK 100
// Half a second of hysteresis
#define BUTTON_DEBOUNCE 500

#define START_EEPROM_LOCATION 4
//for testing
//#define START_EEPROM_LOCATION 1020
#define DEFAULT_SECS_BETWEEN_READS 15

// Read from EEPROM. Stored in EEPROM as:
// uint8_t secs_between_reads
// uint8_t num_readings_high;
// uint8_t num_readings_low
// uint8_t checksum;
uint16_t ms_between_reads = 0;
uint16_t e_write_ptr;
int16_t most_recent_temperature = 0;
uint8_t temperature_result = DS18X20_ERROR;

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = 0,

				.DataINEndpointNumber           = CDC_TX_EPNUM,
				.DataINEndpointSize             = CDC_TXRX_EPSIZE,
				.DataINEndpointDoubleBank       = false,

				.DataOUTEndpointNumber          = CDC_RX_EPNUM,
				.DataOUTEndpointSize            = CDC_TXRX_EPSIZE,
				.DataOUTEndpointDoubleBank      = false,

				.NotificationEndpointNumber     = CDC_NOTIFICATION_EPNUM,
				.NotificationEndpointSize       = CDC_NOTIFICATION_EPSIZE,
				.NotificationEndpointDoubleBank = false,
			},
	};

/** Standard file stream for the CDC interface when set up, so that the virtual CDC COM port can be
 *  used like any regular character stream in the C APIs
 */
FILE USBSerialStream;

#if EXTERNAL_EEPROM == 1
static uint8_t write_byte(uint8_t *loc, uint8_t val)
{
	uint8_t high, low;

	high = ((uintptr_t)loc) >> 8;
	low  = ((uintptr_t)loc) & 0xff;

	i2c_start_wait(EXTERNAL_EEPROM_24C | I2C_WRITE);
	i2c_write(high);
	i2c_write(low);
	i2c_write(val);
	i2c_stop();

	return 0;
}

static uint8_t read_byte(uint8_t *loc, uint8_t *result)
{
	uint8_t high, low;

	high = ((uintptr_t)loc) >> 8;
	low  = ((uintptr_t)loc) & 0xff;

	i2c_start_wait(EXTERNAL_EEPROM_24C | I2C_WRITE);
	i2c_write(high);
	i2c_write(low);

	i2c_rep_start(EXTERNAL_EEPROM_24C | I2C_READ);
	*result = i2c_readNak();

	i2c_stop();

	return 0;
}

#else /* EXTERNAL_EEPROM == 0 */
#define write_byte eeprom_write_byte
#define read_byte(loc, result) *result = eeprom_read_byte(loc)
#endif

static uint8_t search_sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	
	ow_reset();

	nSensors = 0;
	
	diff = OW_SEARCH_FIRST;
	while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ) {
		DS18X20_find_sensor( &diff, &id[0] );
		
		if( diff == OW_PRESENCE_ERR ) {
			break;
		}
		
		if( diff == OW_DATA_ERR ) {
			break;
		}
		
		for ( i=0; i < OW_ROMCODE_SIZE; i++ )
			gSensorIDs[nSensors][i] = id[i];
		
		nSensors++;
	}
	//fprintf(&USBSerialStream, "Sensors found: %d\r\n", nSensors);
	
	return nSensors;
}

#if 0
static void get_decicelsius(void)
{
	int i = gSensorIDs[0][0]; // family-code for conversion-routine
	DS18X20_start_meas( 0, NULL );
	_delay_ms( DS18B20_TCONV_11BIT );
	temperature_result = DS18X20_read_decicelsius_single( i, &most_recent_temperature );
	/*
	if(res != DS18X20_OK) {
		return -6000;
	}
	*/

	return decicelsius;
}
#endif

static void show_temp(void)
{
	if(temperature_result != DS18X20_OK) {
		fprintf(&USBSerialStream, "Read error\r\n");
	}else{
		fprintf(&USBSerialStream, "Temp: %d.%d C\r\n", most_recent_temperature / 10, most_recent_temperature % 10);
	}
}

static void show_counter(void)
{
	fprintf(&USBSerialStream, "Counter: %d\r\n", uptime_ms);
}

#if 0
static void disk_test(void)
{
	FRESULT f_err_code;

	disk_initialize(0);

	fprintf(&USBSerialStream, "\r\nmount\r\n");
	f_err_code = f_mount(0, &FATFS_Obj);
	
	if(f_err_code != 0) {
		fprintf(&USBSerialStream, "\r\nmount error %d\r\n", f_err_code);
	}

	fprintf(&USBSerialStream, "\r\ncompleted\r\n");
}
#endif

static void
eeprom_test(void)
{
	uint8_t byte, res;

	fprintf(&USBSerialStream, "\r\nWrite byte\r\n");

	res = write_byte((void *)10, 0x37);

	fprintf(&USBSerialStream, "R= %d. \r\n", res);

	res = read_byte((void *)10, &byte);

	fprintf(&USBSerialStream, "R= %d, %x\r\n", res, byte);
}

static void
showOptions(void)
{
	fputs("t    temperature\r\n", &USBSerialStream);
	fputs("c    isr counter\r\n", &USBSerialStream);
	fputs("h    e header\r\n", &USBSerialStream);
	fputs("d    e data\r\n", &USBSerialStream);
	fputs("e    e test\r\n", &USBSerialStream);
	fputs("1    interval: 1 sec\r\n", &USBSerialStream);
	fputs("5    interval: 16 secs\r\n", &USBSerialStream);
	fputs("6    interval: 32 secs\r\n", &USBSerialStream);
	//fputs("\r\nd    disk test\r\n", &USBSerialStream);
}

static void
eeprom_read_header(void)
{
	uint8_t byte;
	uint8_t sum;

	sum = read_byte((uint8_t *)0, &byte);
	sum = byte;

	ms_between_reads = byte * 1000;

	read_byte((uint8_t *)1, &byte);
	e_write_ptr = byte << 8;
	sum += byte;

	read_byte((uint8_t *)2, &byte);
	e_write_ptr |= byte;
	e_write_ptr += START_EEPROM_LOCATION;
	sum += byte;

	read_byte((uint8_t *)3, &byte);

	if(byte != sum) {
		// Reset to default values
		ms_between_reads = DEFAULT_SECS_BETWEEN_READS * 1000;
		e_write_ptr = START_EEPROM_LOCATION;
	}
}

static void
eeprom_write_header(void)
{
	uint8_t byte;
	uint8_t sum;

	byte = sum = (ms_between_reads / 1000);
	write_byte((uint8_t *)0, byte);

	byte = ((e_write_ptr - START_EEPROM_LOCATION) >> 8);
	sum += byte;
	write_byte((uint8_t *)1, byte);

	byte = ((e_write_ptr - START_EEPROM_LOCATION) & 0xff);
	sum += byte;
	write_byte((uint8_t *)2, byte);

	write_byte((uint8_t *)3, sum);
}

static void
eeprom_update_count(void)
{
	eeprom_write_header();
}


static void
show_header(void)
{
	eeprom_read_header();
	fprintf(&USBSerialStream, "Secs: %d; Count: %d\r\n", ms_between_reads / 1000, e_write_ptr - START_EEPROM_LOCATION);
}

static void
show_data(void)
{
	for(unsigned int i = START_EEPROM_LOCATION; i < e_write_ptr; i++) {
		int8_t byte;
		read_byte((uint8_t *)i, (uint8_t *)&byte);
		fprintf(&USBSerialStream, "%d ", byte);
	}
	fputs("\r\nDone\r\n", &USBSerialStream);
}

static void
set_interval_frommenu(int byte)
{
	byte -= '0';

	ms_between_reads = (1 << (byte - 1)) * 1000;
	
	eeprom_write_header();
	fprintf(&USBSerialStream, "secs: %d\r\n", ms_between_reads / 1000);
}

static void
doMenu(int byte)
{
	if(byte == '\n' || byte == '?') {
		showOptions();
	} else if (byte == 't') {
		show_temp();
	} else if (byte == 'c') {
		show_counter();
	} else if (byte == 'h') {
		show_header();
	} else if (byte == 'd') {
		show_data();
	} else if (byte >= '1' && byte <= '9') {
		set_interval_frommenu(byte);
	} else if (byte == 'e') {
		eeprom_test();
	/*} else if (byte == 'd') {
		disk_test();
	*/
	} else {
		fputs("\r\n?\r\n", &USBSerialStream);
	}
}


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

	eeprom_read_header();

	/* Create a regular character stream for the interface so that it can be used with the stdio.h functions */
	CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	sei();

	int16_t byte;
	int16_t last_button_press = 0;
	int16_t last_recording_made = 0;
	int16_t last_temp_measurement_time;
	int8_t button = 0, prevbutton = 0;
	int8_t recording_started = 0;
	int therm_idx;

	search_sensors();

	therm_idx = gSensorIDs[0][0];
	last_temp_measurement_time = uptime_ms + (DS18B20_TCONV_12BIT * 2);

	for (;;)
	{
		byte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
		if(byte > 0) {
			doMenu(byte);
		}

		if (uptime_ms - last_temp_measurement_time > DS18B20_TCONV_12BIT) {
			// Store the most recent temperature and start a new reading.
			temperature_result = DS18X20_read_decicelsius_single(therm_idx, &most_recent_temperature );
			DS18X20_start_meas(DS18X20_POWER_EXTERN, NULL );
			last_temp_measurement_time = uptime_ms;
		}

		button = Buttons_GetStatus() & BUTTONS_BUTTON1;

		if(button && !prevbutton && (uptime_ms - last_button_press > BUTTON_DEBOUNCE)) {
			last_button_press = uptime_ms;
			if(recording_started) {
				// Stop recording
				recording_started = 0;
				LEDs_TurnOffLEDs(LEDS_LED1);
				eeprom_update_count();
			} else {
				// Start recording
				recording_started = 1;
				e_write_ptr = START_EEPROM_LOCATION;
			}
		}

		prevbutton = button;

		if(recording_started) {
			// Log to EEPROM
			if (uptime_ms - last_recording_made >= ms_between_reads) {
				if (e_write_ptr < MAX_EEPROM_LOCATION) {
					int8_t byte;

					if(temperature_result != DS18X20_OK) {
						byte = 127;
					} else {
						byte = most_recent_temperature / 10;
					}
					write_byte((uint8_t*)e_write_ptr, (uint8_t)byte);

					e_write_ptr += 1;
					last_recording_made = uptime_ms;
				} else {
					recording_started = 0;
					eeprom_update_count();
					LEDs_TurnOnLEDs(LEDS_LED1);
				}
			}
		}

		if(recording_started && e_write_ptr < MAX_EEPROM_LOCATION) {
			if(uptime_ms & 0x8)
				LEDs_TurnOnLEDs(LEDS_LED1);
			else
				LEDs_TurnOffLEDs(LEDS_LED1);
		}

		/* Must throw away unused bytes from the host, or it will lock up while waiting for the device */
		//CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
	}
}

ISR(TIMER1_OVF_vect)
{
	TCNT1 = TIMER1_DELAY;
	uptime_ms += MS_PER_TICK;

	//disk_timerproc();
}

/* Set up the SD card timer */
void
initInterrupt(void)
{
	/* Prescaler = 8 */
	//TCCR1B |= 2;

	/* Prescaler = 64 */
	TCCR1B |= 3;

	TCNT1 = TIMER1_DELAY;
	
	/* Overflow */
    TIMSK1 |= (1 << TOIE1);
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

#if EXTERNAL_EEPROM == 1
	i2c_init();
#endif

	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();
	Buttons_Init();
	initInterrupt();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

