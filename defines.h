#define F_CPU 20000000UL
#define UART_BAUD  19200

#define TWI_DELAY 500



// Mapping of old fashioned register names to atmega8x names

// For rs232:
#define UBRRL UBRR0L

#define UCSRB UCSR0B
#define TXEN TXEN0
#define RXEN RXEN0
#define UCSRA UCSR0A
#define UDRE UDRE0

#define UDR UDR0
#define RXC RXC0
#define FE FE0
#define DOR DOR0



