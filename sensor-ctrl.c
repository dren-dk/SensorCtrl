#include "defines.h"

#include <ctype.h>
#include <inttypes.h>

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <util/twi.h>
#include <avr/sleep.h>

#include "uart.h"
#include "i2csw.h"
#include <avr/wdt.h> 
#include <avr/interrupt.h>
#include <avr/eeprom.h> 
#include <avr/pgmspace.h>


// We don't really care about unhandled interrupts.
EMPTY_INTERRUPT(__vector_default)

// A macro and function to store string constants in flash and only copy them to
// RAM when needed, note the limit on string length.
char stringBuffer[80];

const char *getString(PGM_P src) {
    //assert(strlen_P(src) < sizeof(stringBuffer));
    strcpy_P(stringBuffer, src);
    return stringBuffer;
}

#define PROGSTR(s) getString(PSTR(s))


//-----------------------------------------------------------------------------
// Sleep utility for powering down a little between samples.

// Puts the controller to sleep for about 10ms.
void sleep10ms() {
    // Set the timer to sleep 10 ms and then wake up the mcu with an interrupt.
    TCCR2A = 0;
    TIMSK2 = OCIE2A;                      // Fire interrupt when done
    TCNT2=0;
    OCR2A=195;                            // 10ms or thereabout.
    TCCR2B = 1<<CS20 | 1<<CS21 | 1<<CS22; // Select the slowest clock (20M / 1024)    
    sei();

    // power-down, and wait for the compare interrupt to fire.
    //set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    sleep_enable();
    sleep_cpu();

    // Stop timer again.
    TCCR2B = 0;
}

// Sleep for at least 10 ms at most sleepms + 10 ms 
void sleepMs(unsigned int sleepms) {
    sleepms /= 10;
    do {
      _delay_ms(10);
      //sleep10ms();	
      wdt_reset();
    } while (sleepms--);
}


//-----------------------------------------------------------------------------
// Time one charge/discharge cycle of the sensor.
unsigned int humidityCycle() {

    // Fully discharge.
    PORTD &=~ 1 << PD2; DDRD |= 1 << PD2;
    _delay_ms(1);
    DDRD &=~ 1 << PD2;
    
    // Set up the 16 bit timer to go as fast as possible.
    TCCR1A = 0;
    TCCR1B = 1<<CS10;

    // Wait for charge
    PORTC |= 1 << PC3;
//    _delay_us(20); 
    TCNT1=0;
    loop_until_bit_is_set(PIND, PD2);
    unsigned int res = TCNT1;

    // Fully Charge.
    PORTD |= 1 << PD2; DDRD |= 1 << PD2;
    _delay_ms(1);
    PORTD &=~ 1 << PD2; DDRD &=~ 1 << PD2;

    // Wait for discharge
    PORTC &=~ 1 << PC3;
//    _delay_us(20);
    TCNT1=0;
    loop_until_bit_is_clear(PIND, PD2);
    res += TCNT1;
    TCCR1B = 0;

    return res;
}

#define H_CYCLES 40

//-----------------------------------------------------------------------------
unsigned int humidityOffset;
unsigned int humidityDelta;
unsigned int lastLocalTemp=44; // Updated when sampling the temperature
unsigned int lastRawHumidity;

unsigned int readHumidity() {
    // PC3 = driver, PD2 = sensor.

    DDRC |= 1 << PC3;
    DDRD &=~ 1 << PD2;

    // Reset by charging and then discharging.
    humidityCycle();
    humidityCycle();
    
    int total = 0;
    for (int i=0;i<H_CYCLES;i++) {
	total += humidityCycle();
    }

    DDRC &=~ 1 << PC3;
    PORTC &=~ 1 << PC3;

    total /= H_CYCLES;
    lastRawHumidity = total;

    total -= humidityOffset;
    total -= (lastLocalTemp << 3)/60;
    total *= humidityDelta;
    total >>= 8;

    return total;   
}


//-----------------------------------------------------------------------------
// Get one sample from the ADC.
#define ADC_BUSY_WAIT 128

unsigned int getADC(unsigned char input) {
    _delay_ms(1);
    
    ADMUX = (input & 15) | _BV(REFS0); // AVcc reference + external cap.
    ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADIE);
    if (input & ADC_BUSY_WAIT) {
	ADCSRA |= 1<<ADSC;
	while(ADCSRA & 1<<ADSC) {}

    } else {
	SREG |= _BV(SREG_I);
	//set_sleep_mode(SLEEP_MODE_ADC);
	sleep_enable();
	sleep_mode();
    }
    
    unsigned int result = ADCL | ADCH << 8;

    return result;
}

#define ADC_OVERSAMPLES 6
unsigned int getOsADC(unsigned char input) {
    unsigned int sum = 0;

    for (int i=0;i<_BV(ADC_OVERSAMPLES);i++) {
	sum += getADC(input);
    }
    
    return sum >> ADC_OVERSAMPLES;
}


//-----------------------------------------------------------------------------
unsigned int voltageCalibration[3];

// Sample one analog input and convert the value to a voltage in mv.
// Taking the 1/6 voltage divider and the VCC into account.
unsigned int getVoltage(unsigned char input) {
    // Sample the internal 1.1 V reference, to figure out what the the VCC is.
    unsigned int sbg = getOsADC(14); 
    unsigned int sampleTomvFactor = voltageCalibration[input]/sbg;
    unsigned int raw = getOsADC(input);
//    fprintf(stdout,PROGSTR(" sbg=%d s2v=%d raw=%d"), sbg, sampleTomvFactor, raw);

    return (sampleTomvFactor*raw) >> 1;
}


//-----------------------------------------------------------------------------
// Highlevel printing functions, mainly used for tidyness
void sampleHumidity() {
    unsigned int hum = readHumidity();
    fprintf(stdout, PROGSTR("H:%d h:%d"), hum, lastRawHumidity);	
}

void sampleDoor() {
    DDRD &=~ 1<<PD3; // Door input
    PORTD |= 1<<PD3; // Door pull up enable.

    if (PIND & (1<<PD3)) {
	fprintf(stdout, PROGSTR(" D:open"));
	    
    } else {
	fprintf(stdout, PROGSTR(" D:closed"));				    
    }
}

void sampleVoltages() {
    ADCSRA |= 1<<ADEN;

    // This seems to be needed to allow the ADC to stabilize:
    for (int i=0;i<10;i++) {
	getADC(14);
	getADC(0);
	getADC(1);
	getADC(2);
	_delay_ms(20);
    }

    for (int i=0;i<3;i++) {
	unsigned int voltage = getVoltage(i);
	if (voltage > 0) {
	  fprintf(stdout, PROGSTR(" V%d:%d"), 2-i, voltage); // Renumbering so V1=V1 and V2=V2 on the board.
	}
    }

    ADCSRA &=~ 1<<ADEN;
}

int16_t sampleTemperatures() {
  int16_t maxTemp = -1000;
  for (char i=0;i<8;i++) {
    signed char temp[2];
    char err = i2cReceive(64|8|i, 0, 2, (BYTE *)temp);
    if (err) {
      if (i == 7) {
	fprintf(stdout, PROGSTR(" T%d:e%d"), i, err);			
      }
    } else {
	    
      fprintf(stdout, PROGSTR(" T%d:%d"), i, (signed)temp[0]);
	    
      int16_t tempHalf = temp[0] << 1;
	    
      if (temp[1] & 128) {
	if (temp[0] > 0) {
	  fprintf(stdout, PROGSTR(".5"));
	  tempHalf++;
	}
      } else {
	if (temp[0] < 0) {
	  fprintf(stdout, PROGSTR(".5"));
	  tempHalf--;
	}
      }

      if (i != 7) {
	if (tempHalf > maxTemp) {
	  maxTemp = tempHalf;       
	}
      } else {
	lastLocalTemp = tempHalf;
      }
      
    }
  }

  return maxTemp != -1000 ? maxTemp : lastLocalTemp;
}

char inputInt(char *buffer, char length) {
    unsigned char index = strlen(buffer);
    fprintf(stdout, buffer);

    while (1) {
	int key = fgetc(stdin);
	wdt_reset();
	if (key < 0) continue;
	if (key == '\r') {
	    fprintf(stdout, PROGSTR("\n"));	    
	    return 0;
	}
	if (key >= '0' && key <= '9') {
	    buffer[index] = key;
	    fprintf(stdout, "%c", key);
	    if (index <= length-1) {
		index++;
		buffer[index] = 0;
	    } else {
		fprintf(stdout, "\b");
	    }

	}
	if (key == '\b') {
	    buffer[index] = 0;
	    if (index > 0) {
		index--;
		fprintf(stdout, PROGSTR("\b \b"));
	    } else {
		fprintf(stdout, " \b");
	    }
	} 
   }   
}

#define CAL_MAGIC 0
#define CAL_MAGIC_VALUE 0xbabe
#define CAL_VOLTAGES 2

#define CAL_HUMIDITY_MAGIC 8
#define CAL_HUMIDITY_MAGIC_VALUE 0xbabe
#define CAL_HUMIDITY_OFFSET 10
#define CAL_HUMIDITY_DELTA 12

void calibrateADC() {

    ADCSRA |= 1<<ADEN;
    for (int i=0;i<10;i++) {
	getADC(14);
	getADC(0);
	getADC(1);
	getADC(2);
	_delay_ms(10);
    }

    fprintf(stdout, PROGSTR("\nWelcome to voltage calibration\n"));
    fprintf(stdout, PROGSTR("Please connect a known voltage (5-30V) to V1 and V2, but not pwr\n"));
    

    unsigned int voltage = 0;
    while (!voltage) {
	fprintf(stdout, PROGSTR("Please enter the voltage in mv (or an empty string to quit): "));

	char buffy[10];
	*buffy = 0;
	inputInt(buffy, 6);

	if (!*buffy) {
	    ADCSRA &=~ 1<<ADEN;
	    return;
	}

	voltage = atoi(buffy);

	if (voltage > 30000) {
	    fprintf(stdout, PROGSTR("Error Voltage too high, please use max 30000 mv.\n"));
	    voltage = 0;
	} else if (voltage < 5000) {
	    fprintf(stdout, PROGSTR("Error Voltage too low, please use min 5000 mv.\n"));
	    voltage = 0;
	}
    }
       
    // Calibrate V1 and V2.
    _delay_ms(1);
    unsigned int bg = getOsADC(14);
    
    for (int i=0;i<2;i++) {
	_delay_ms(1);
	unsigned int raw = getOsADC(i);
	unsigned int cal = (2*voltage/raw)*bg;
	voltageCalibration[i] = cal;
	
	fprintf(stdout, PROGSTR("  Calibration for ch %d bg=%d v=%d raw=%d cal=%d\n"),
		2-i, bg, voltage, raw, cal);
    }

    fprintf(stdout, PROGSTR("Please connect pwr as well as V1 and V2 to the same voltage.\n"));
    fprintf(stdout, PROGSTR("Hit n, when ready or q to quit.\n"));
    int key;
    while((key = fgetc(stdin)) != 'n') {
	if (key == 'q') {
	    ADCSRA &=~ 1<<ADEN;
	    return;
	}
	wdt_reset();
	_delay_ms(100);
    }

    // Now use the calibrated V1 and V2 to calibrate PWR
    bg = getOsADC(14);
    voltage = (getVoltage(0)+getVoltage(1)) >> 1;
    fprintf(stdout, PROGSTR(" Calibrating PWR ADC, using %d mv on V1 and V2 as reference\n"), voltage);
    
    _delay_ms(1);
    unsigned int raw = getOsADC(2);
    unsigned int cal = (2*voltage/raw)*bg;
    voltageCalibration[2] = cal;
    
    fprintf(stdout, PROGSTR("  Calibration for ch 0 (pwr) bg=%d v=%d raw=%d cal=%d\n"),  bg, voltage, raw, cal);

    ADCSRA &=~ 1<<ADEN;

    for (int i=0;i<3;i++) {    
	eeprom_write_word(((uint16_t *)CAL_VOLTAGES)+i, voltageCalibration[i]);
	
    }
    eeprom_write_word((uint16_t *)CAL_MAGIC, CAL_MAGIC_VALUE);
    
    fprintf(stdout, PROGSTR("All voltage channels have been calibrated and EEPROM has been updated.\n"));
}

void calibrateHumidity() {

    fprintf(stdout, PROGSTR("\nWelcome to humidity calibration\n"));

    unsigned int offset = 0;
    while (!offset) {
	fprintf(stdout, PROGSTR("Please enter the humidity offset: "));

	char buffy[10];
	sprintf(buffy, "%d", humidityOffset);
	inputInt(buffy, 6);

	if (!*buffy) {
	    return;
	}

	offset = atoi(buffy);

	if (offset > 600) {
	    fprintf(stdout, PROGSTR("Error: Offset must be less than 600.\n"));
	    offset = 0;
	} else if (offset < 300) {
	    fprintf(stdout, PROGSTR("Error: Offset must be larger than 300.\n"));
	    offset = 0;
	}
    }

    unsigned int delta = 0;
    while (!delta) {
	fprintf(stdout, PROGSTR("Please enter the humidity delta: "));

	char buffy[10];
	sprintf(buffy, "%d", humidityDelta);
	inputInt(buffy, 6);

	if (!*buffy) {
	    return;
	}

	delta = atoi(buffy);

	if (delta < 200) {
	    fprintf(stdout, PROGSTR("Error: Delta must be larger than 200.\n"));
	    delta = 0;
	} else if (delta > 400) {
	    fprintf(stdout, PROGSTR("Error: Delta must be less than 400.\n"));
	    delta = 0;
	}
    }

    
    fprintf(stdout, PROGSTR("Humidity calibration: offset=%d, delta=%d, hit 's' to save, 'q' to quit.\n"), 
	    offset, delta);

    char key;
    while((key = fgetc(stdin)) != 's') {
	if (key == 'q') {
	    return;
	}
	wdt_reset();
	_delay_ms(100);
    }

    humidityOffset = offset;
    humidityDelta = delta;
    eeprom_write_word((uint16_t *)CAL_HUMIDITY_MAGIC, CAL_HUMIDITY_MAGIC_VALUE);
    eeprom_write_word((uint16_t *)CAL_HUMIDITY_OFFSET, humidityOffset);
    eeprom_write_word((uint16_t *)CAL_HUMIDITY_DELTA, humidityDelta);

    fprintf(stdout, PROGSTR("Humidity calibration saved to EEPROM.\n"));
}


void loadCalibration() {

    unsigned int magic = eeprom_read_word((uint16_t *)CAL_MAGIC);
    if (magic != CAL_MAGIC_VALUE) {
	fprintf(stdout, PROGSTR("Using inaccurate default voltage calibration, hit v!\n"));
	
	for (int i=0;i<3;i++) {
	    voltageCalibration[i] = 2*6600;
	}
    } else {
	fprintf(stdout, PROGSTR("Voltages are calibrated, hit v to recalibrate.\n"));
	
	for (int i=0;i<3;i++) {
	    voltageCalibration[i] = eeprom_read_word(((uint16_t *)CAL_VOLTAGES)+i);
	}
    }

    // Load humidity calibration here.
    magic = eeprom_read_word((uint16_t *)CAL_HUMIDITY_MAGIC);
    
    if (magic != CAL_HUMIDITY_MAGIC_VALUE) {
	fprintf(stdout, PROGSTR("Using inaccurate default humidity calibration, hit h!\n"));
	humidityOffset = 429;
	humidityDelta = 267;

    } else {
	fprintf(stdout, PROGSTR("Humidity is calibrated, hit h to recalibrate.\n"));
	humidityOffset=eeprom_read_word((uint16_t *)CAL_HUMIDITY_OFFSET);
	humidityDelta=eeprom_read_word((uint16_t *)CAL_HUMIDITY_DELTA);
    }
}

void disableFan(char disable) {
  if (disable) {
    PORTD |= _BV(PD7);  // Fan disable
    PORTB |= _BV(PB3);  // LED on
  } else {
    PORTD &=~ _BV(PD7);  // Fan enable
    PORTB &=~ _BV(PB3);  // LED off
  }
}

#define MAX_FAN 1000000
uint16_t sampleFanRPM() {
  uint8_t timeout = 1;
  uint32_t res = 0;
  while (res++ < MAX_FAN) {
    if (PINB & _BV(PB2)) {
      timeout = 0;
      break;
    }
  }

  if (timeout) {
    return 0;
  }

  TCNT1=0;
  TCCR1A = 0;
  TCCR1B = _BV(CS12) | _BV(CS10);

  res = 0;
  timeout = 1;
  while (res++ < MAX_FAN) {
    if (!(PINB & _BV(PB2))) {
      timeout = 0;
      break;
    }
  }

  if (timeout) {
    return 0;
  }

  timeout = 1;
  while (res++ < MAX_FAN) {
    if (!(PINB & _BV(PB2))) {
      timeout = 0;
      break;
    }
  }

  if (timeout) {
    return 0;
  }

  TCCR1B = 0;

  return  res;
}

void setFanPWM(unsigned char speed) {
  OCR0B = speed;
  disableFan(!speed);
}

void initFanPWM() {
  TCCR0A = _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
  TCCR0B = _BV(CS01) | _BV(WGM02);
  OCR0A = 100; // along with Clock/8 this gives 25 KHz
}

#define FAN_BITS 2
#define MAX_FAN_SPEED (100<<FAN_BITS)
#define TARGET_TEMP 27

uint16_t fanTachoPulses = 0;
void enableFanTacho() {
  PCICR |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT2); // aka. PB2
  fanTachoPulses = 0;
  sei();
}

void disableFanTacho() {
  PCMSK0 &=~ _BV(PCINT2);
}

ISR(PCINT0_vect) {
  fanTachoPulses++;
}

int main(void) {
    wdt_enable(WDTO_4S);

    DDRB |= 1<<PB3;  // LED output
    DDRB |= 1<<PB4;  // LED output
    DDRB |= 1<<PB5;  // LED output

    DDRD |= 1<<PD5;  // FAN PWM
    DDRD |= 1<<PD7;  // FAN disable

    PORTB |= 1<<PB2; // FAN RPM pullup

    uart_init();
    FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
    stdout = stdin = &uart_str;
    fprintf(stdout, PROGSTR("Power up!\n"));
 
    i2cInit();

    loadCalibration();

    disableFan(0);
    initFanPWM();

    int fanSpeed = MAX_FAN_SPEED;
    
    while(1) {
      PORTB |= _BV(PB4);
      enableFanTacho();      
      sleepMs(1000);
      disableFanTacho();      
      PORTB &=~ _BV(PB4);
      
      
      sampleHumidity();

      sampleDoor();
      sampleVoltages();
      
      int16_t maxTemp = sampleTemperatures();
      int16_t errTemp = maxTemp- TARGET_TEMP*2;	
      fprintf(stdout, PROGSTR(" Tm:%d Te:%d"), maxTemp, errTemp);	

      uint16_t fanRpm = fanTachoPulses*30;
      fprintf(stdout, PROGSTR(" F:%d"), fanRpm);
      
      fanSpeed += errTemp;

      if (fanSpeed < 0) {
	fanSpeed = 0;
      } else if (fanSpeed > MAX_FAN_SPEED) {
	fanSpeed = MAX_FAN_SPEED;
      }

      unsigned char fs = fanSpeed >> FAN_BITS;
      setFanPWM(fs);
      fprintf(stdout, PROGSTR(" P:%d"), fs);
      
      fprintf(stdout, PROGSTR("\n"));
      
      int key = fgetc(stdin);
      if (key > 0) {
	fprintf(stdout, PROGSTR("You hit '%c': "), key);
	if (key == 'v') {
	  calibrateADC();
	  
	} else if (key == 'h') {
	  calibrateHumidity();
	  
	} else {
	  fprintf(stdout, PROGSTR("Unknown command.\n"));
	}
      }
      
      wdt_reset();
    }	
}
