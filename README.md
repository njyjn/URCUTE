# URCUTE
Unnecessarily Redundant Care Unit for The Elderly (CUTE) is a variation of EE2024 CUTE, built for EE2024 by [Justin Ng](http://github.com/njyjn) and [Lim Hong Wei](http://github.com/imhongw) as **Assignment 2**. It is strictly for educational purposes and not to be used on a real elderly patient. Get an Apple Watch for them or something.

## About
URCUTE is designed for the elderly person who wholly entrusts their care to a a Central Elderly Monitoring System (CEMS).

The ease of understanding lies in the fact that URCUTE only runs from a single file. No more complicated header structures and variables that seem to come out of nowhere. All global flags are declared up top, and all timer variables are declared at the top of `void main()`.

For detailed assignment requirements, visit the [EE2024 wiki](http://wiki.nus.edu.sg/display/ee2024).

## Modes
URCUTE has two basic modes: [STABLE](#stable-mode) and [MONITOR](#monitor-mode), two enhanced modes: [NIGHT](#night-mode) and [EMERGENCY](#emergency-mode), and one quasi-mode: [CEMS OVERRIDE](#cems-override-mode).

To view use cases, teleport [there](#use-cases-elderly-person-a).

### Stable Mode
Stable mode is to be used in the presence of a caretaker. There is no environment sampling, no information displayed on the screen, and no emergency warnings.

Stable mode is also the first mode that URCUTE boots into. The boot process is highlighted below. Click [here](#monitor-mode) to skip to **Monitor Mode**.

#### Initialization
The system initializes the clock, I2C, SPI/SSP and GPIO.

`I2C` is used for the LED array, accelerometer and light sensor. As per EA's specifications, the system uses the `I2C2` interface, which takes `P0.10` and `P0.11` as `SDA2` and `SCL2` respectively. For this, we set them to use `Funcnum = 2`, which activates I2C.
```
static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 2; // when 10
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);
	I2C_Init(LPC_I2C2, 100000); // clock rate 100000
	I2C_Cmd(LPC_I2C2, ENABLE);
}
```

`GPIO` is used for `SW4`, `SW3` buttons, the buzzer and the temperature sensor. The following sections of code initializes the buttons `SW3` (`P2.10`) and `SW4` (`P1.31`), and the buzzer (`P2.13`).
```
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
```

`SW4` is activated upon regular polling, with debounce time of 500ms.
```
if (timesUpOrNot(modeTime, 500)) {
  mode_button = GPIO_ReadValue(1) >> 31 & 0x01;
  if (mode_button == 0) {
    if (mode == MODE_STABLE) {
      reset();
      mode = MODE_MONITOR;
    } else if (mode == MODE_MONITOR) {
      printf("Entering STABLE mode.\n");
      mode = MODE_STABLE;
    }
    mode_button = 1;
  }
  modeTime = getTicks();
}
```

`SW3` employs EINT0 (`P2.10`) instead of polling, unlike `SW4`. (This is because `SW4` is on Port 1 where EINT0 is not supported.) To do so we must set `FuncNum = 1`.
```
	//Initialize button sw3 (Interrupt)
	PinCfg.Funcnum = 1; // Using EINT0: P2.10 is EINT0 when 01.
	PinCfg.OpenDrain = 0; // PUN
	PinCfg.Pinmode = 0; // PUN
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1 << 10, 0); // Set input mode
```

Of note is that the buzzer (`LM4811-shutdown`) and the RGB BLUE LED share the same pin `P2.13`.
```
	//Initialize buzzer (Speaker-Amplifier PWMs)
	GPIO_SetDir(0, 1<<27, 1); // SP-CLK / LM4811-clk
	GPIO_SetDir(0, 1<<28, 1); // SP-UP/DOWN / LM4811-up/dn
	GPIO_SetDir(2, 1<<13, 1); // RGB BLUE or LM4811-shutdn
	GPIO_ClearValue(0, 1<<27); // Set output mode
	GPIO_ClearValue(0, 1<<28);
	GPIO_ClearValue(2, 1<<13);
}
```

We also initialize `SPI/SSP` for 7-segment display and OLED, but its initialization is trivial and will not be explained here.

URCUTE then polls for the initial x, y, z accelerometer values and calculates its offsets. This ensures that x,y,z are relative to the position URCUTE was first initialized in.

```
acc_init();
acc_read(&x, &y, &z);
xoff = 0 - x; // Calculate offsets
yoff = 0 - y;
zoff = 0 - z;
```

URCUTE then sets up its light sensors in `light_enable()` (provided in `Lib_EaBaseBoard`) and `config_light()`. Of note is that we are setting up a light IRQ (`P2.5`) as falling edge interrupt. So when the reading falls below or above the specified threshold (in our case, 50 and 5000 respectively), an interrupt flag is written to `P2.5`.

```
static void config_light(void) {
	light_setRange(LIGHT_RANGE_4000);
	light_setLoThreshold(lightLoLimit);
	light_setHiThreshold(lightHiLimit);
	light_setIrqInCycles(LIGHT_CYCLE_16); // Used 16 for 4 sec activation time.
	light_clearIrqStatus();
	LPC_GPIOINT->IO2IntClr = 1 << 5;
	LPC_GPIOINT->IO2IntEnF |= 1 << 5; // enable falling edge interrupt for P2.5 (irq_out for light sensor)
}
```

The system uses an `EINT3` interrupt handler for this purpose. It will first check `P2.5` for the interrupt flag and set `light_flag` accordingly.

```
void EINT3_IRQHandler(void) {
	// Determine whether GPIO Interrupt P2.5 has occurred (LIGHT SENSOR) by checking pin
	if ((LPC_GPIOINT->IO2IntStatF>>5)& 0x1) {
		light_flag = LIGHT_LOW;
		LPC_GPIOINT->IO2IntClr = (1<<5); // Clear the interrupt register
		light_clearIrqStatus(); // Clear IRQ otherwise the interrupt will never be issued again.
	}
}
```

The remaining code initializes the various peripherals on the EaBaseBoard URCUTE is built on. There is nothing significant about them, and we will not be elaborating in detail here.

```
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
```

### Monitor Mode
Monitor mode is used when no caretaker is available. It samples

1. the ambient temperature in _deg C_, every 5 seconds,
2. the movement of the wearer with coordinates _x,y,z_, every 1 second,
3. the ambient light in _lux_, every 5 seconds

and sends these to CEMS with format `NNN_-_T*****_L*****_AX*****_AY*****_AZ*****\r\n` every 5 seconds (or when the seven-segment display shows '5', 'A', or 'F').
```
static void sendToCems(unsigned char *string) {
	UART_Send(LPC_UART3, (uint8_t *) string, strlen(string), BLOCKING);
}
```

In addition to the above factors, the following alarms are activated upon

1. Increase in ambient temperature beyond a threshold of _45 deg C_
```
float temperature  = temp_read()/10.0;
if (temperature >= 45)
  blink_red = 1;
```

2. Movement is detected in the dark (below _50 lux_)
```
void isMovementInDarkDetected(uint8_t threshold) {
	if (sqrt(x*x+y*y+z*z) >= threshold && light_flag == LIGHT_LOW) {
		blink_blue = 1;
	}
}
```

3. Serious fall has occurred
```
void isFallDetected(uint8_t threshold) {
	if (abs(y - yLast) >= threshold) {
		fall = FALL_DETECTED;
	}
}
```

The appropriate LEDs blink according to the emergency accorded:

**RED** during a fire,
```
if (blink_red == 1) { // Blink RED LED for fire detection.
				if (timesUpOrNot(rgbTime, 333))
					GPIO_SetValue( 2, 1);
				if (timesUpOrNot(rgbTime, 666)) {
					GPIO_ClearValue( 2, 1);
					rgbTime = getTicks();
				}
```

**BLUE** during movement in the dark (The 16-LEDs also all turn on)
```
if (blink_blue == 1) { // Blink BLUE LED for movement in the dark.
				pca9532_setLeds(0xffff, 0); // On floodlights
				if (timesUpOrNot(rgbTime, 333))
					GPIO_SetValue( 0, (1<<26));
				if (timesUpOrNot(rgbTime, 666)) {
					GPIO_ClearValue( 0, (1<<26));
					rgbTime = getTicks();
				}
```


and both **RED** and **BLUE** when both have occurred (Also, the 16-LEDs all blink)
```
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
```

The LEDs do not turn off until a caretaker is present and MODE change `SW4` is pressed.

#### Pages
URCUTE supports three different pages in **MONITOR** mode:

0. Main: Displays sampled environment information, updated every 5s.
- `MONITOR   0` is displayed at the top at all times.
1. Information
- `1` is displayed at the top at all times.
2. Nightmode
- `NIGHT     2` is displayed at the top at all times.

```
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
```

### Night Mode
At any time, URCUTE is able to act as a nightlight to facilitate safe movement in the dark. The nightlight is able to be adjusted using the included rotary dial.

Furthermore, turning the rotary adjusts the brightness of the OLED accordingly:
```
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
```

### Emergency Mode
URCUTE is able to interface with the CEMS HQ in case the user needs urgent assistance. When `SW3` is pressed, it sends a message to CEMS. Once CEMS receives it successfully, the screen shows the message `HELP REQUESTED.`
```
if (emergency_flag == EMER_RAISED) {
  sendToCems("[MANUAL OVERRIDE] EMERGENCY ASSISTANCE REQUESTED!\r\n");
  emergency_flag = EMER_WAIT;
} else if (emergency_flag == EMER_WAIT) {
  oled_putString(0,55, "HELP REQUESTED.", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}
```

Until the request is acknowledged, the buzzer buzzes for half a second per second. Note that in this mode, the **RED** led does not blink, while the **BLUE** led does not blink normally, even if there are pending **FIRE** or **DARK** flags.
```
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
```

`playNote()` simply uses PWM to turn the BLUE led on and off in a fixed periodic duration, varying the tone and pitch of the buzzer. The lower the note 'frequency', the shorter each of its on/off periods (inversely proportional to frequency) and its the more times it modulates (proportional to duration), resulting in a higher note.
```
#define NOTE_PIN_HIGH() GPIO_SetValue(0, 1<<26)
#define NOTE_PIN_LOW() GPIO_ClearValue(0, 1<<26)

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
```

The pitch of the buzzer may be adjusted by varying the `pitch` variable being passed into the `playNote(pitch, duration)` method, through turning the rotary left or right. `pitch` here corresponds to the variable `note`, and defaults to 2000.
```
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
```

Whenever this mode is triggered, the page shown on the OLED returns to Page 0, and the joystick, rotary is deactivated for Page 2 (NIGHTLIGHT).
```
if ((pageIndex == 1 || pageIndex == 2) && emergency_flag == EMER_WAIT) {
  reading_mode = 0;
  changePage(0x01);
}
```

When the request is acknowledged by CEMS, the display shows `HELP IS COMING`, and the buzzer switches off.

### CEMS Override Mode
URCUTE may be remotely controlled via an authorized attendant at the CEMS HQ.

<kbd>s</kbd>: Change to STABLE mode.<br>
<kbd>m</kbd>: Change to MONITOR mode.<br>
<kbd>e</kbd>: Acknowledge EMERGENCY mode. Informs URCUTE that help is on its way, so it silences its alarms.<br>

The following hunk of code, using `UART_Receive` library, enables this incoming mode of communication.
```
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

```

## Use Cases: Elderly Person A
### 0. Caretaker sets up URCUTE and heads back to CEMS HQ.
- Caretaker straps URCUTE on A and hits the Mode Change (MC) button, also known as `SW4`.
- The OLED comes to life, showing the word `MONITOR` and environment information.

### 1. A's wig is on fire
- URCUTE polls temperature every 5 seconds.
- If the temperature somehow exceeds 45 degrees Celsius, it will begin to blink RED.
- A warning message `FIRE!` shows up on the screen.
- CEMS is notified every 5 seconds.
- Caretaker arrives, hits the MC and URCUTE returns to STABLE mode. He puts out the fire.
- Before returning to CEMS HQ, he hits the MC. URCUTE returns to MONITOR mode.

### 2. A is caught moving to the kitchen at night for his milk.
- URCUTE polls movement every 1 second, and receives an interrupt when the surrounding lux value falls below 50.
- If so, it will begin to blink BLUE.
- A warning message `DARK!` shows up on the screen.
- CEMS is notified every 5 seconds.
- The Jewel-like Enhanced Warm Emitting Light System (JEWELS) will be activated, and turns on until the alarm is deactivated by a caretaker.
- Caretaker hits the MC and URCUTE returns to STABLE mode. He scolds A and puts him back in bed.
- Before returning to CEMS HQ, he hits the MC. URCUTE returns to MONITOR mode. Good night, A!

#### 2. (a) A does not want to be caught during his milk run this time.
- A flicks the Page Integrated Switch Stick (PISS) to Page 2. URCUTE enters NIGHTLIGHT mode.
- A turns the rotary clockwise to adjust the amount of nightlight through the OLED. Too bright.
- A adjust the rotary anti-clockwise and reaches the goldilocks zone where it is above 50 lux.
- He thanks the creator of URCUTE and gets his milk discretely.
- By the way, how would he go about licensing this technology? He flicks the PISS again, to Page 1.
- URCUTE tells him to look for `Arrchana`.

### 3. A is sleepwalking and knocks over a candle. The carpet catches fire.
- URCUTE notices that there is both a fire and movement in darkness.
- It begins to blink RED and BLUE simultaneously.
- CEMS is notified every 5 seconds.
- A warning message `FIRE! DARK!` shows up on the screen.
- CEMS is notified every 5 seconds.
- The JEWELS will be activated, and flash on and off until the alarm is deactivated by a caretaker.
- Caretaker hits the MC and URCUTE returns to STABLE mode. He puts out the fire, and puts A back in bed.
- Before returning to CEMS HQ, he hits the MC. URCUTE returns to MONITOR mode. Good night, A!

### 4. A has fallen off his bed and is unconscious.
- URCUTE polls movement every 1 second.
- If y-value falls below (or above) threshold at 18, it will send a message to CEMS.

### 5. A notices a stranger at his door. The stranger seems to be holding a suspicious box and a knife.
- A presses the Emergency Response Mode (ERM), otherwise known as `SW3`.
- CEMS receives A's notification immediately. While A waits for a response, his screen shows a `HELP REQUESTED` message and buzzes loudly so that neighbors can also respond.
- While doing so, there is no need for any other distractions, so the BLUE and RED lights do not come on.
- A panics and presses ERM again, and a repeat message is sent to CEMS. URCUTE continues to buzz.
- CEMS acknowledges A's notification by pressing <kbd>e</kbd>. They are on their way. A's screen shows 'HELP IS COMING' message and stops buzzing.
- A goes to his fridge and gets himself a glass of milk.

#### 5. (a) A caretaker has arrived, only to discover that it was another caretaker holding a birthday cake and a plastic knife. It is A's birthday!
- The caretakers hits the MC, returning URCUTE to STABLE state. As expected, all alarms cease to sound or light up.
- He resolves to use the <kbd>m</kbd> key remotely to silence unnecessary warnings in the future.

#### 5. (b) A's neighbor has a hearing aid which only picks up high pitch sounds.
- A turns the rotary clockwise to increase the pitch of the emergency buzzer.
- The neighbor responds and asks A to turn it down because it is hurting her.
- A turns the rotary anti-clockwise to decrease the pitch of the emergency buzzer.
- The person at the door is spooked and decides to leave.

## License

Code released under the [MIT License](https://choosealicense.com/licenses/mit/).

Copyright (c) [2017] [Justin Ng] [Lim Hong Wei], ECE (CEG2) @ NUS

_Stay hungry, stay foolish._
