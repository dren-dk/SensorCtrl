#CC=/opt/avr-gcc-4.2.1/bin/avr-gcc
CC=avr-gcc
CFLAGS=-Os -Wall -pedantic-errors -Werror -mcall-prologues -mmcu=atmega88 -std=c99
OBJ2HEX=avr-objcopy 

program : sensor-ctrl.hex
	avrdude -c avrispv2 -P usb  -p atmega88 -e -U flash:w:sensor-ctrl.hex

fuses:
	avrdude -c avrispv2 -P usb -p atmega88 -U lfuse:w:0xf6:m -U hfuse:w:0xd4:m

# http://palmavr.sourceforge.net/cgi-bin/fc.cgi?P_PREV=ATmega88&P=ATmega88&V_LOW=F6&V_HIGH=DC&V_EXTENDED=F9&M_LOW_0x3F=0x36&M_LOW_0x40=&M_LOW_0x80=&M_HIGH_0x07=0x04&M_HIGH_0x08=0x00&M_HIGH_0x10=&M_HIGH_0x20=0x00&M_HIGH_0x40=&M_HIGH_0x80=&M_EXTENDED_0x01=&M_EXTENDED_0x06=0x00&B_SPIEN=P&B_CKSEL3=P&B_BOOTSZ1=P&B_BODLEVEL1=P&B_BOOTSZ0=P&B_CKSEL0=P&B_BODLEVEL0=P

sensor-ctrl.obj : sensor-ctrl.c uart.c i2csw.c i2csw.h uart.h defines.h Makefile
	$(CC) $(CFLAGS) uart.c i2csw.c sensor-ctrl.c -o $@

%.hex : %.obj
	$(OBJ2HEX) -R .eeprom -O ihex $< $@

clean :
	rm -f *.hex *.obj *.o

