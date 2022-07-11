#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "srom_3360_0x05.h"
#include "usb_mouse.h"

#define delay_us(t) __builtin_avr_delay_cycles((t) * (F_CPU/1000000))
#define delay_ms(t) __builtin_avr_delay_cycles((t) * (F_CPU/1000))

// button release latency. units are 125us, so 128 means 16ms.
#define DEBOUNCE_TIME 128

// how long before assuming wheel is in the center of a notch.
#define WHL_MAX_TIMEOUT 16000 // = 2 seconds

#define WHL_LED_ON (PORTB &= ~(1<<4))
#define WHL_LED_OFF (PORTB |= (1<<4))

#define PORT_SPI PORTB
#define DDR_SPI	DDRB

#define DD_SS	0 // aka NCS
#define DD_SCK	1
#define DD_MOSI	2
#define DD_MISO	3

#define SS_LOW	(PORT_SPI &= ~(1<<DD_SS))
#define SS_HIGH	(PORT_SPI |= (1<<DD_SS))

// use this instead of bitshifts or LSB/MSB macros.
union motion_data {
	int16_t all;
	struct { uint8_t lo, hi; };
};


static void pins_init(void)
{
	PORTD |= 0b00001111; // L, R, M, DPI pullup inputs on D0, D1, D2, D5. (active low)
	PORTF |= 0b00110011;

	EICRA = 0b01010101; // generate interrupt request on any edge of D0/D1/D2/D3
	EIMSK = 0; // but don't enable any actual interrupts
	EIFR = 0b00001111; // clear EIFR

	PORTB |= (1<<4);
	DDRB |= (1<<4); // B4 led (active low)
	DDRB &= ~(1<<5); // B5 detector input

	//DDRC |= (1<<2); PORTC |= (1<<2); // C2, 3360 NRESET high output
}


// spi functions
static void spi_init(void)
{
	DDR_SPI |= (1<<DD_MOSI) | (1<<DD_SCK) | (1<<DD_SS); // outputs
	// MISO pullup input is already done in hardware
	// enable spi, master mode, mode 3, clock rate = fck/4 = 2MHz
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<CPOL) | (1<<CPHA);
}

static inline void spi_send(const uint8_t b)
{
	SPDR = b;
	while (!(SPSR & (1<<SPIF)));
}

static inline uint8_t spi_recv(void)
{
	spi_send(0x00);
	return SPDR;
}

static inline void spi_write(const uint8_t addr, const uint8_t data)
{
	spi_send(addr | 0x80);
	spi_send(data);
	delay_us(180); // maximum of t_SWW, t_SWR
}

static inline uint8_t spi_read(const uint8_t addr)
{
	spi_send(addr);
	delay_us(160); // t_SRAD
	uint8_t data = spi_recv();
	delay_us(20);
	return data;
}


static void pmw3360_init(const uint8_t dpi)
{
	SS_HIGH;
	delay_ms(3);

	// shutdown first
	SS_LOW;
	spi_write(0x3b, 0xb6);
	SS_HIGH;
	delay_ms(300);

	// drop and raise ncs to reset spi port
	SS_LOW;
	delay_us(40);
	SS_HIGH;
	delay_us(40);

	// power up reset
	SS_LOW;
	spi_write(0x3a, 0x5a);
	SS_HIGH;
	delay_ms(50);

	// read from 0x02 to 0x06
	SS_LOW;
	spi_read(0x02);
	spi_read(0x03);
	spi_read(0x04);
	spi_read(0x05);
	spi_read(0x06);

	spi_write(0x10, 0x00); // disable rest mode
	spi_write(0x22, 0x00); // ???

	// srom download
	spi_write(0x13, 0x1d);
	SS_HIGH;
	delay_ms(10);
	SS_LOW;
	spi_write(0x13, 0x18);

	const uint8_t *psrom = srom;
	spi_send(0x62 | 0x80);
	for (uint16_t i = 0; i < SROM_LENGTH; i++) {
		delay_us(16);
		spi_send(pgm_read_byte(psrom++));
	}
	delay_us(18);
	SS_HIGH;
	delay_us(200);

	// configuration/settings
	SS_LOW;
	spi_write(0x10, 0x00); // 0x00 disables rest mode use 0x20 for wireless
	spi_write(0x14, 0xff); // how long to wait before going to rest mode. 0xff is max (~10 seconds)
	spi_write(0x17, 0xff);
	spi_write(0x18, 0x00);
	spi_write(0x19, 0x00);
	spi_write(0x1b, 0x00);
	spi_write(0x1c, 0x00);

	// surface tuning default
	spi_write(0x2c, 0x0a);
	spi_write(0x2b, 0x10);
    // calibrated in logitech software with g pro (skypad)
	// spi_write(0x2c, 0x34);
	// spi_write(0x2b, 0xde);

	// configuration/settings
	spi_write(0x0f, dpi);
	spi_write(0x42, 0x00); // no angle snapping
	spi_write(0x63, 0x02); // 2mm lod
	SS_HIGH;
}

// reads the change from the wheel detector.
// see http://www.google.com/patents/US6552716
static inline int8_t whl_read(void)
{
	// read "B"
	PORTB |= (1<<5); // PORT before DDR to avoid outputting a falling edge
	DDRB |= (1<<5);
	delay_us(1); // not sure how long to delay. 1us seems sufficient
	PORTB &= ~(1<<5); // output a falling edge
	delay_us(1);
	DDRB &= ~(1<<5);
	delay_us(1);
	int8_t b = ((PINB & (1<<5)) != 0); // read the pin

	delay_us(1);

	// read "A"
	PORTB |= (1<<5);
	DDRB |= (1<<5);
	delay_us(1);
	PORTB &= ~(1<<5);
	delay_us(1);
	DDRB &= ~(1<<5);
	delay_us(1);
	int8_t a = ((PINB & (1<<5)) != 0);

	return (a - b);
	// toggle wheel led for ~25us after calling this function
}

uint8_t EEMEM stored_dpi_index = 1;
uint8_t btn_dbncd = 0x00;

int main(void)
{
	MCUCR |= _BV(JTD);
	MCUCR |= _BV(JTD); //disable Port F jtag by writing bit twice within 4 cycles.

	// set clock prescaler for 16MHz
	CLKPR = 0x80;
	CLKPR = 0x00;

	pins_init();

	// previous state to compare against for debouncing
	// uint8_t btn_prev = (~PIND) & 0x03; // read L+R
	uint8_t btn_prev = (~PIND) & 0b00001111; // read L, R, M, DPI
	// time (in 125us) button has been unpressed.
	// consider button to be released if this time exceeds DEBOUNCE_TIME.
	// uint8_t btn_time[4] = {0, 0, 0, 0};

	// if dpi button is pressed when plugging in, jump to bootloader
	// see https://www.pjrc.com/teensy/jump_to_bootloader.html
	delay_ms(50);
	if (!(PIND & (1<<5)))
		__asm__ volatile ("jmp 0x7000"); //atmel-dfu
		//__asm__ volatile ("jmp 0x7e00"); // teensy2.0 

	// wheel
	WHL_LED_ON; // initial pulse for first read
	delay_us(25);
	WHL_LED_OFF;
	// ticks are what the detector outputs.
	// for g100s's wheel/detector, 8 ticks = 1 scroll notch.
	// these two variables serve as accumulators, since the wheel is read
	// more frequently than usb data is transmitted
	int8_t whl_notches = 0;
	int8_t whl_ticks = 0;
	// if no wheel motion after this many cycles, reset whl_ticks.
	uint16_t whl_timeout = 0;

	union motion_data x_sum, y_sum; // total motion after last usb transmission
	spi_init();
	// dpi settings
	// uint8_t dpi_index = 1;
	static uint8_t dpi_index;
	uint8_t dpis[] = {3, 7, 15, 31, 63};
	dpi_index = eeprom_read_byte(&stored_dpi_index);
	pmw3360_init(dpis[dpi_index]);
	// begin burst mode
	SS_LOW;
	spi_write(0x50, 0x00);
	SS_HIGH;

	usb_init();
	while (!usb_configured());
	delay_ms(456); // arbitrary

	// set up timer0 to set OCF0A in TIFR0 every 125us
	TCCR0A = 0x02; // CTC
	TCCR0B = 0x02; // prescaler 1/8 = 1us period
	// OCR0A = 124; // = 125 - 1
	OCR0A = 249; // = 16000000 / (8 * 8000) - 1 .. for 16mhz use 249

	cli();
	for (uint8_t i = 0; ; i = (i + 1) % 8) {
	// synchronization to usb frames and 125us intervals
		// polling interrupt flags gives 5 clock cycles or so of
		// jitter. possible to eliminate by going into sleep
		// mode and waking up using interrupts, but whatever.
		if (i == 0) {
			// sync to usb frames (1ms)
			UDINT &= ~(1<<SOFI);
			while(!(UDINT & (1<<SOFI)));
			// reset prescaler phase, not really necessary
			GTCCR |= (1<<PSRSYNC);
			TCNT0 = 0;
		} else {
			// sync to 125us intervals using timer0
			while (!(TIFR0 & (1<<OCF0A)));
		}
		TIFR0 |= (1<<OCF0A); // 0CF0A is cleared by writing 1

	// check wheel timeout
		if (whl_timeout != 0) whl_timeout++;
		if (whl_timeout >= WHL_MAX_TIMEOUT) {
			whl_timeout = 0;
			whl_ticks = 0; // assume wheel is in center of a notch
		}


	// sensor stuff
		union motion_data x, y;
		SS_LOW;
		spi_send(0x50);

		// 3360 needs total of about 35us delay before reading registers
		// do wheel stuff here as a delay
		const int8_t _whl_ticks = whl_read(); // takes ~10us
		WHL_LED_ON;
		delay_us(25); // toggle wheel for next cycle's read
		WHL_LED_OFF;

		spi_send(0x00); // motion, not used
		spi_send(0x00); // observation, not used
		x.lo = spi_recv();
		x.hi = spi_recv();
		y.lo = spi_recv();
		y.hi = spi_recv();
		SS_HIGH;


	// wheel calculations
		whl_ticks += _whl_ticks;
		int8_t _whl_notches = 0;
		if (_whl_ticks != 0) {
			_whl_notches = whl_ticks / 5;
			// same as (whl_ticks > 4) - (whl_ticks < -4)
			whl_ticks -= _whl_notches * 8;
			whl_timeout = 1; //reset timeout
		}

	// button stuff
		//high = not pressed, low = pressed
		//PIND 0 EIFR 0: low, no edges -> is low
		//PIND 0 EIFR 1: low, edge -> is low
		//PIND 1 EIFR 0: high, no edges -> always high during last 125us
		//PIND 1 EIFR 1: high, edge -> low at some point in the last 125us
		// const uint8_t btn_raw = (~PIND) | EIFR;
		// EIFR = 0b00001111; // clear EIFR

		// button debouncing logic
		//          >input<           |        >output<
		//------------------------------------------------------
		// previous    | current      | unclicked  | current
		// dbncd state | actual state | time       | dbncd state
		//-------------+--------------+------------+------------
		//    btn_prev |      btn_raw | btn_time   |   btn_dbncd
		//-------------+--------------+------------+------------
		//           0 |            0 |         =0 |          =0
		//           0 |            1 |         =0 |          =1
		//           1 |            0 |         ++ | (time < DEBOUNCE_TIME)
		//           1 |            1 |         =0 |          =1
		// uint8_t btn_dbncd = 0x00;

		// // manual loop debouncing for every button
		/* #define DEBOUNCE(index) \
		if ((btn_prev & (1<<index)) && !(btn_raw & (1<<index))) { \
			btn_time[index]++; \
			if (btn_time[index] < DEBOUNCE_TIME) \
				btn_dbncd |= (1<<index); \
		} else { \
			btn_time[index] = 0; \
			btn_dbncd |= btn_raw & (1<<index); \
		}*/
		// DEBOUNCE(0); // L
		// DEBOUNCE(1); // R
		// DEBOUNCE(2); // M
		// DEBOUNCE(3); // RSB
		// DEBOUNCE(4); // FSB
		// DEBOUNCE(5); // DPI

		// #undef DEBOUNCE

	// hardware debouncing
		//+left click
		if (!(PIND & (1 << 0))) {
			btn_dbncd |= (1<<0);
		}
		//-left click
		if (!(PINF & (1 << 0))) {
			btn_dbncd &= ~(1<<0);
		}
		//+right click
		if (!(PIND & (1 << 1))) {
			btn_dbncd |= (1<<1);
		}
		//-right click
		if (!(PINF & (1 << 1))) {
			btn_dbncd &= ~(1<<1);
		}
		//+middle click
		if (!(PIND & (1 << 2))) {
			btn_dbncd |= (1<<2);
		}
		//-middle click
		if (!(PINF & (1 << 4))) {
			btn_dbncd &= ~(1<<2);
		}
		//+dpi click
		if (!(PIND & (1 << 3))) {
			btn_dbncd |= (1<<3);
		}
		//-dpi click
		if (!(PINF & (1 << 5))) {
			btn_dbncd &= ~(1<<3);
		}

	// usb
		// first make sure it's configured
		sei();
		while (!usb_configured());
		cli();
		// this stuff is very intricate and confusing
		// i'm fairly certain all of it is correct.
		// there's nothing to do if nothing's changed in this 125us cycle
		if ((btn_dbncd != btn_prev) || x.all || y.all || _whl_notches) {
			UENUM = MOUSE_ENDPOINT;
			if (UESTA0X & (1<<NBUSYBK0)) { // untransmitted data still in bank
				UEINTX |= (1<<RXOUTI); // kill bank; RXOUTI == KILLBK
				while (UEINTX & (1<<RXOUTI));
			} else {
				// transmission's finished, or the data that should be in the
				// bank is exactly the same as what was previously transmitted
				// so that there was nothing worth transmitting before.
				x_sum.all = 0;
				y_sum.all = 0;
				whl_notches = 0;
			}
			x_sum.all += x.all;
			y_sum.all += y.all;
			whl_notches += _whl_notches;
			// only load bank with data if there's something worth transmitting
			if ((btn_dbncd != btn_prev) || x_sum.all || y_sum.all || whl_notches) {
				UEDATX = btn_dbncd;
				UEDATX = x_sum.lo;
				UEDATX = x_sum.hi;
				UEDATX = y_sum.lo;
				UEDATX = y_sum.hi;
				UEDATX = whl_notches; // wheel scrolls
				UEINTX = 0x3a;
				// btn_prev = btn_dbncd;
			}
		}
		// when dpi and right button is pressed cycle dpi and save changes in eeprom
		// if ((btn_dbncd & 0x20) && !(btn_prev & 0x20)) {
		if (!(PIND & (1<<1)) && !(PIND & (1<<5))) {
			delay_ms(500);
			dpi_index = (dpi_index + 1) %
					(sizeof(dpis)/sizeof(dpis[0]));
			SS_LOW; delay_us(1);
			spi_write(0x0f, dpis[dpi_index]);
			SS_HIGH;
			eeprom_write_byte(&stored_dpi_index, dpi_index);
		}

		btn_prev = btn_dbncd;
	}
}
