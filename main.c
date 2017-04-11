/**
 * Unnecessarily Redundant Care Unit for The Elderly (URCUTE)
 * 2017 (C) JUSTIN NG, LIM HONG WEI [NUS CEG2]
 * MIT License
 */

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_rtc.h"

#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "light.h"
#include "temp.h"
#include "oled.h"
#include "rgb.h"
#include "led7seg.h"
#include "rotary.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/** TIMER **/
volatile uint32_t msTicks = 0;

void SysTick_Handler(void) {
	msTicks++;
}

uint32_t getTicks(void) {
	return msTicks;
}

int timesUpOrNot(uint32_t startTicks, int delayInMs) {
	return (getTicks() - startTicks) >= delayInMs;
}
/* ==================== END TIMER */

/** SEVEN SEGMENT DISPLAY **/
const uint8_t sevenSegChars[] = {
		'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'
};

/** ACCELEROMETER **/
int8_t x = 0, y = 0, z = 0, xoff = 0, yoff = 0, zoff = 0, xLast = 0, yLast = 0, zLast = 0;

/** SYS_MODE **/
typedef enum {
	MODE_STABLE, MODE_MONITOR
} system_mode_t;
volatile system_mode_t mode;

/** SAMPLE_MODE **/
typedef enum {
	PRINT, SEND
} sample_mode_t;
volatile sample_mode_t sample_mode;

/** MOVEMENT **/
typedef enum {
	MOVEMENT_NO, MOVEMENT_DETECTED
} movemen_t;
volatile movemen_t movement;

/** LIGHT **/
typedef enum {
	LIGHT_NORMAL, LIGHT_LOW
} ligh_t;
volatile ligh_t light_flag;

/** FALL **/
typedef enum {
	FALL_NO, FALL_DETECTED
} fall_t;
volatile fall_t fall;

/** EMERGENCY **/
typedef enum {
	EMER_NO, EMER_RAISED, EMER_WAIT, EMER_RESOLVED
} emergency_t;
volatile emergency_t emergency_flag;


/** MISC FLAGS **/
uint8_t cems_message_counter = 0, blink_blue = 0, blink_red = 0, monitor_firstrun = 0, state = 0, mode_button = 1, reading_mode=0, xW = 0, buzz_first=1;
int pitch = 2000;
int pageIndex = 0;

/** INIT PROTOCOLS **/

/**
 * @brief Used for LED array, accelerometer and light sensor.
 */
static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect*/
	// LPC uses I2C2 because EA says so
	// P0.10: SDA2
	// P0.11: SCL2
	PinCfg.Funcnum = 2; // when 10
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000); // clock rate 100000

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

/**
 * @brief Used for 7-seg, OLED
 */
static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK: Serial clock
	 * P0.8 - MISO: Master input
	 * P0.9 - MOSI: Master output
	 * P2.2 - SSEL: Slave select - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

/**
 * @brief Used for SW4, SW3, speaker, temperature sensor
 */
static void init_GPIO(void) {
	//Initialize button sw4 (GPIO interrupt)
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 0; // Using normal GPIO setting for P1.31 when 00.
	PinCfg.OpenDrain = 0; // PUN
	PinCfg.Pinmode = 0; // PUN
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1<<31, 0); // Set input mode

	//Initialize button sw3 (Interrupt)
	PinCfg.Funcnum = 1; // Using EINT0: P2.10 is EINT0 when 01.
	PinCfg.OpenDrain = 0; // PUN
	PinCfg.Pinmode = 0; // PUN
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1 << 10, 0); // Set input mode

	//Initialize buzzer (PWMs)
	GPIO_SetDir(0, 1<<27, 1); // SP-CLK / LM4811-clk
	GPIO_SetDir(0, 1<<28, 1); // SP-UP/DOWN / LM4811-up/dn
	GPIO_SetDir(2, 1<<13, 1); // RGB BLUE or LM4811-shutdn
	GPIO_ClearValue(0, 1<<27); // Set output mode
	GPIO_ClearValue(0, 1<<28);
	GPIO_ClearValue(2, 1<<13);
}
/* ==================== END INIT PROTOCOLS */

/** LIGHT SENSOR CONFIG **/
const uint32_t lightLoLimit = 50, lightHiLimit = 4000;

static void config_light(void) {
	// Setup light limit for triggering interrupt
	light_setRange(LIGHT_RANGE_4000);
	light_setLoThreshold(lightLoLimit);
	light_setHiThreshold(lightHiLimit);
	light_setIrqInCycles(LIGHT_CYCLE_16); // Used 16 for 4 sec activation time.
	light_clearIrqStatus();
	LPC_GPIOINT->IO2IntClr = 1 << 5;
	LPC_GPIOINT->IO2IntEnF |= 1 << 5; // enable falling edge interrupt for P2.5 (irq_out for light sensor)
}
/* ==================== END LIGHT SENSOR CONFIG */

/** INIT UART **/
void pinsel_uart3(void){
	// P0.0: uart1tx
	// P0.1: uart1rx
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);
}

void init_uart(void){
	UART_CFG_Type uartCfg;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;
	pinsel_uart3();	//pin select for uart3
	UART_Init(LPC_UART3, &uartCfg); //supply power & setup working parameters for uart3
	UART_TxCmd(LPC_UART3, ENABLE); //enable transmit for uart3
}
/* ==================== END INIT UART */

/**
 * @brief Send a string to CEMS (UART).
 */
static void sendToCems(unsigned char *string) {
	UART_Send(LPC_UART3, (uint8_t *) string, strlen(string), BLOCKING);
}

/** INTERRUPT HANDLERS **/
void eint_init(void) {
	NVIC_SetPriority(SysTick_IRQn, 1); // Timer has the highest priority.

	// Enable EINT3 interrupt
	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_SetPriority(EINT3_IRQn, 2); // Light has higher priority than button.
	NVIC_EnableIRQ(EINT3_IRQn);

	// Enable EINT0 interrupt with SW3
	LPC_SC->EXTINT = 1; // Clear existing interrupts.
	NVIC_ClearPendingIRQ(EINT0_IRQn);
	NVIC_SetPriority(EINT0_IRQn, 3);
	NVIC_EnableIRQ(EINT0_IRQn);
}

// EINT3 Interrupt Handler
void EINT3_IRQHandler(void) {
	// Determine whether GPIO Interrupt P2.5 has occurred (LIGHT SENSOR) by checking pin
	if ((LPC_GPIOINT->IO2IntStatF>>5)& 0x1) {
		light_flag = LIGHT_LOW;
		LPC_GPIOINT->IO2IntClr = (1<<5); // Clear the interrupt register
		light_clearIrqStatus(); // Clear IRQ otherwise the interrupt will never be issued again.
	}
}

// EINT0 Interrupt Handler (for SW3)
void EINT0_IRQHandler(void)
{
	// Raise emergency flag only if there is already no emergency, and in monitor mode.
	if (mode == MODE_MONITOR && emergency_flag == EMER_NO) {
		emergency_flag = EMER_RAISED;
	}
	// Clear EINT0 (otherwise interrupt will hang the entire system)
	// Write 1 to bit value 0 (EINTx) because it is active low.
	LPC_SC->EXTINT = (1<<0);
}
/* ==================== END INTERRUPT HANDLERS */

/** MOVEMENT LOGIC **/
void isMovementInDarkDetected(uint8_t threshold) {
	if (sqrt(x*x+y*y+z*z) >= threshold && light_flag == LIGHT_LOW) {
		blink_blue = 1;
	}
}

/**
 * FALL LOGIC
 * @brief Serious falls are when the y value changes drastically.
 */
void isFallDetected(uint8_t threshold) {
	if (abs(y - yLast) >= threshold) {
		fall = FALL_DETECTED;
	}
}

static void sampleAcc() {
	acc_read(&x, &y, &z);
	x = x + xoff;
	y = y + yoff;
	z = z + zoff;
	isMovementInDarkDetected(10);
	isFallDetected(18);
}
/* ==================== END MOVEMENT LOGIC */

/**
 * @brief Samples the environment for temp, lux and acc, then prints to OLED. If SEND is specified, send to UART.
 */
static void sampleEnvironmentAnd(sample_mode_t sample_mode) {
	unsigned char temp[15] = "", lux[15] = "", acc[15] = "", warning[50] = "";

	float temperature  = temp_read()/10.0;
	if (temperature >= 45)
		blink_red = 1;

	int light = light_read();

	if (sample_mode == SEND) {
		// NNN_-_T*****_L*****_AX*****_AY*****_AZ*****\r\n
		unsigned char string[50] = "";
		sprintf(string, "%03d_-_T%05.1f_L%05d_AX%05d_AY%05d_AZ%05d\r\n",cems_message_counter++,temperature,light,x,y,z);
		if (blink_red)
			sendToCems("Fire was Detected.\r\n");
		if (blink_blue)
			sendToCems("Movement in darkness was Detected.\r\n");
		if (fall == FALL_DETECTED)
			sendToCems("Serious fall was Detected.\r\n");
		sendToCems(string);
	}

	// Only print values if page is at 0.
	if (pageIndex == 0) {
		sprintf(temp, "%.1f degrees C\r", temperature);
		sprintf(lux, "%d lux", light);
		sprintf(acc, "%dx, %dy, %dz\r", x,y,z);
		sprintf(warning, "%s", blink_red&&blink_blue?"FIRE! DARK!":blink_red?"FIRE!":blink_blue?"DARK!":" ");
		oled_clearScreen(OLED_COLOR_BLACK);
		oled_putString(0,0, "MONITOR", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(0,10, warning, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(0,20, temp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(0,30, lux, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(0,40, acc, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		if (emergency_flag == EMER_RESOLVED) {
			oled_putString(0,55, "HELP IS COMING.", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		} else if (emergency_flag == EMER_WAIT) {
			oled_putString(0,55, "HELP REQUESTED.", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		}
	}
}

static void changePage(uint8_t joyState)
{
	// Ignore up, down joystates
	if ((joyState & JOYSTICK_UP) != 0 || (joyState & JOYSTICK_DOWN) != 0) {
		return;
	}
	oled_clearScreen(OLED_COLOR_BLACK);
	int maxIndex = 3;

	// Reset
	if ((joyState & JOYSTICK_CENTER) != 0) {
		pageIndex = 0;
	} else if ((joyState & JOYSTICK_RIGHT) != 0) {
		pageIndex = (pageIndex+1)%3;
	} else if ((joyState & JOYSTICK_LEFT) != 0) {
		pageIndex = (pageIndex+5)%3;
	}

	if (pageIndex == 0) {
		reading_mode = 0;
		sampleEnvironmentAnd(PRINT);
	} else if (pageIndex == 1) {
		reading_mode = 0;
		oled_putString(0,10,"URCUTE v0.1", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(0,30,"For licensing", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(0,40,"contact Arcana.", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	} else if (pageIndex == 2) {
		reading_mode = 1;
		oled_putString(0,0, "NIGHT", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_fillRect(0, 10, xW, OLED_DISPLAY_HEIGHT, OLED_COLOR_WHITE);
	}
}

#define NOTE_PIN_HIGH() GPIO_SetValue(0, 1<<26)
#define NOTE_PIN_LOW() GPIO_ClearValue(0, 1<<26)

/**
 * @brief Turns on the buzzer using PWM. The lower the note 'frequency', the shorter
 * each of its on/off periods (inversely proportional to frequency) and its the more times it
 * modulates (proportional to duration), resulting in a higher note.
 */
static void playNote(uint32_t note, uint32_t durationMs) {
	uint32_t t = 0;
	if (note>0) {
		while(t<(durationMs*1000)) {
			NOTE_PIN_HIGH();
			Timer0_us_Wait(note/2);
			NOTE_PIN_LOW();
			Timer0_us_Wait(note/2);
			t+=note;
		}
	} else {
		Timer0_Wait(durationMs);
	}
}

void init() {
	SysTick_Config(SystemCoreClock / 1000);
	init_i2c();
	init_ssp();
	init_GPIO();

	acc_init();
	acc_read(&x, &y, &z);
	xoff = 0 - x; // Calculate offsets
	yoff = 0 - y;
	zoff = 0 - z;

	light_enable();
	config_light();

	temp_init(getTicks);
	pca9532_init();
	joystick_init();
	rgb_init(); // Since green LED interferes with OLED, do not use.
	oled_init();
	led7seg_init();
	init_uart();
	RTC_Init(LPC_RTC);
	eint_init();
	rotary_init();
}

void reset() {
	monitor_firstrun = 1; // Toggle first run of monitor mode.
	pitch = 2000; // Pitch is set to 2000 by default.
	led7seg_setChar('{',FALSE); // Clear 7SEG display
	blink_red = 0;	// Clear blink red flag
	blink_blue = 0;	// Clear blink blue flag
	light_flag = LIGHT_NORMAL; // Absolve light warning
	movement = MOVEMENT_NO; // Absolve movement
	fall = FALL_NO;
	emergency_flag = EMER_NO; // Clear emergency
	pca9532_setLeds(0, 0xffff); // Clear floodlights
	GPIO_ClearValue( 2, 1); // Clear red
	GPIO_ClearValue( 0, (1<<26) ); // Clear blue
	oled_clearScreen(OLED_COLOR_BLACK); // cls
}

int main (void) {
	init();

	uint8_t train = 1, data = 0;
	uint32_t startTicks = getTicks();
	uint32_t modeTime=startTicks, trainTime=startTicks, redLightTime= startTicks, rgbTime= startTicks, blueLightTime=startTicks, sevenSegTime=startTicks, emergencyTime=startTicks, buzzerTime=startTicks;

	oled_clearScreen(OLED_COLOR_BLACK); // Initial clear screen

	while (1) {
		// Local mode switching by polling SW4 at P1.31
		if (timesUpOrNot(modeTime, 500)) {
			mode_button = GPIO_ReadValue(1) >> 31 & 0x01;
			if (mode_button == 0) {
				if (mode == MODE_STABLE) {
					reset();
					mode = MODE_MONITOR;
				} else if (mode == MODE_MONITOR) {
					mode = MODE_STABLE;
				}
				mode_button = 1;
			}
			modeTime = getTicks();
		}

		// Wireless mode switching
		UART_Receive(LPC_UART3, &data, 1, NONE_BLOCKING);
		if (data == 's') {
			mode = MODE_STABLE;
			data = 0;
			sendToCems("\r\nEntering STABLE mode by CEMS.\r\n");
		} else if (data == 'm') {
			mode = MODE_MONITOR;
			data = 0;
			sendToCems("\r\nEntering MONITOR mode by CEMS.\r\n");
		} else if (data == 'e' && mode == MODE_MONITOR && emergency_flag == EMER_WAIT) {
			emergency_flag = EMER_RESOLVED;
			data = 0;
			sendToCems("\r\nCEMS responding to emergency...\r\n");
		}

		switch (mode) {
		case MODE_STABLE:
			reset();
			break;

		case MODE_MONITOR:
			if (monitor_firstrun) {
				oled_putString(0,0, "MONITOR", OLED_COLOR_WHITE, OLED_COLOR_BLACK); // Set MONITOR on graphics display
				sendToCems("Entering MONITOR mode.\r\n");
				monitor_firstrun = 0;
				pageIndex = 0;
			}

			if (emergency_flag == EMER_RAISED) {
				sendToCems("[MANUAL OVERRIDE] EMERGENCY ASSISTANCE REQUESTED!\r\n");
				emergency_flag = EMER_WAIT;
			} else if (emergency_flag == EMER_WAIT) {
				oled_putString(0,55, "HELP REQUESTED.", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			}

			// PRINT PAGE NUMBER
			unsigned char index[1] = "";
			sprintf(index, "%d", pageIndex);
			oled_putString(50,0,index, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

			if (timesUpOrNot(sevenSegTime, 1000)) { // 1s period
				// Sample acceleration
				sampleAcc();
				// SSEG display increments by 1 per second (SSP/SSI)
				static int cycler = 16;
				led7seg_setChar(sevenSegChars[++cycler%16], FALSE);
				// Sample the environment every 5 seconds.
				if (cycler%16 == 5 || cycler%16 == 10) {
					sampleEnvironmentAnd(PRINT);
				}
				if (cycler%16 == 15) {
					// Send samples to SEMS
					sampleEnvironmentAnd(SEND);
				}
				sevenSegTime = getTicks();
			}

			// Blink both LEDs for fire detection and movement in the dark (synchronous).
			// Note that we set the pins individually and do not use RGB library because
			// green LED clashes with OLED.
			// Red is P2.0, Blue is P0.26
			if (blink_red == 1 && blink_blue == 1) {
				if (timesUpOrNot(rgbTime, 333)) {
					pca9532_setLeds(0xffff, 0); // Flash floodlights on
					GPIO_SetValue( 2, 1); // Red first
					GPIO_ClearValue( 0, (1<<26)); // Clear blue
				}
				if (timesUpOrNot(rgbTime, 666)) {
					pca9532_setLeds(0, 0xffff); // Flash floodlights off
					GPIO_ClearValue( 2, 1); // Clear red
					GPIO_SetValue( 0, (1<<26)); // Set blue
					rgbTime = getTicks();
				}
			} else if (blink_blue == 1) { // Blink BLUE LED for movement in the dark.
				pca9532_setLeds(0xffff, 0); // On floodlights
				if (timesUpOrNot(rgbTime, 333))
					GPIO_SetValue( 0, (1<<26));
				if (timesUpOrNot(rgbTime, 666)) {
					GPIO_ClearValue( 0, (1<<26));
					rgbTime = getTicks();
				}
			} else if (blink_red == 1) { // Blink RED LED for fire detection.
				if (timesUpOrNot(rgbTime, 333))
					GPIO_SetValue( 2, 1);
				if (timesUpOrNot(rgbTime, 666)) {
					GPIO_ClearValue( 2, 1);
					rgbTime = getTicks();
				}
			}

			// Joystick page control
			if (timesUpOrNot(trainTime, 250)) {
				state = joystick_read();
				if (state != 0 && emergency_flag != EMER_WAIT) {
					changePage(state);
				}
				trainTime = getTicks();
			}

			// Rotary controls for
			// 1) Buzzer tone modification
			// 2) Reading light brightness adjust
			uint8_t rotaryState = rotary_read();
			if (reading_mode == 0 && rotaryState!=ROTARY_WAIT) {
				if (rotaryState == ROTARY_RIGHT) {
					pitch -= 300;
					if (pitch <= 100) {
						pitch = 100;
					}
				} else if (rotaryState == ROTARY_LEFT) {
					pitch += 300;
					if (pitch >= 3000) {
						pitch = 3000;
					}
				}
			}

			// Play buzzer if emergency requested.
			// The blinking of blink_blue and blink_red ceases in this mode
			// and page defaults to 0.
			if (timesUpOrNot(buzzerTime, 1000) && emergency_flag == EMER_WAIT) {
				buzz_first = 0;
				blink_red = 0;
				blink_blue = 0;
				playNote(pitch, 500);
				Timer0_Wait(1);
				buzzerTime = getTicks();
			}

			// Back to main page if emergency triggered.
			if ((pageIndex == 1 || pageIndex == 2) && emergency_flag == EMER_WAIT) {
				reading_mode = 0;
				changePage(0x01);
			}

			// If not in emergency mode, the rotary is enabled to adjust reading mode brightness.
			if (reading_mode == 1 && rotaryState != ROTARY_WAIT) {
				if (rotaryState == ROTARY_RIGHT) {
					xW = xW + 6;
					if (xW > OLED_DISPLAY_WIDTH) {
						xW = OLED_DISPLAY_WIDTH;
					}
				} else if (rotaryState == ROTARY_LEFT) {
					xW = xW - 6;
					if (xW < 6) {
						xW = 6;
					}
				}
				oled_clearScreen(OLED_COLOR_BLACK);
				oled_putString(0,0,"READING", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
				oled_fillRect(0, 10, xW, OLED_DISPLAY_HEIGHT, OLED_COLOR_WHITE);
			}
			break;
		}
	}
}

void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}

