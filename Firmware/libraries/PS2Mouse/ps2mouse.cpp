/*
 * ps2.cpp - an interface library for ps2 devices.  Common devices are
 * mice, keyboard, barcode scanners etc.  See the examples for mouse and
 * keyboard interfacing.
 * limitations:
 *      we do not handle parity errors.
 *      The timing constants are hard coded from the spec. Data rate is
 *         not impressive.
 *      probably lots of room for optimization.
 */

#include "ps2mouse.h"

//#define readclck (PINK & _BV(PK6))
#define readclck (digitalRead(0))
//#define readdata (PINK & _BV(PK7))
#define readdata (digitalRead(2))

#define timeout ((micros()-timer) > 9999)

unsigned long timer;

// Faster switching
void gohiclck()
{
	//DDRK &= ~_BV(PK6);
    //PORTK |= _BV(PK6); 
	pinMode(0, INPUT);
	digitalWrite(0, HIGH);	
}
void goloclck()
{
	//DDRK |= _BV(PK6);
	//PORTK &= ~_BV(PK6);
	pinMode(0, OUTPUT);
	digitalWrite(0, LOW);	
}
void gohidata()
{
	//DDRK &= ~_BV(PK7);
    //PORTK |= _BV(PK7);
	pinMode(2, INPUT);
	digitalWrite(2, HIGH);	
}
void golodata()
{
	//DDRK |= _BV(PK7);
	//PORTK &= ~_BV(PK7);
	pinMode(2, OUTPUT);
	digitalWrite(2, LOW);	
}

/*
 * the clock and data pins can be wired directly to the clk and data pins
 * of the PS2 connector.  No external parts are needed.
 */
PS2Mouse::PS2Mouse()
{
	gohiclck();
	gohidata();
}

/* write a byte to the PS2 device */
char PS2Mouse::write(unsigned char data)
{
	unsigned char i, parity = 1;
	
	gohidata();
	gohiclck();
	delayMicroseconds(10);	
	goloclck();
	delayMicroseconds(10);
	golodata();
	delayMicroseconds(10);
	gohiclck();  // start bit
	
	// wait for device to take control of clock
	delayMicroseconds(10);
	while (readclck) { if (timeout) { return 0; } }

	// clear to send data
	for (i=0; i < 8; i++) 	{
		if (data & 0x01) {
			gohidata();
		} else {
			golodata();
		}
		// wait for clock
		while (!readclck) { if (timeout) { return 0; } }
		while (readclck)  { if (timeout) { return 0; } }
		parity = parity ^ (data & 0x01);
		data = data >> 1;
	}
	// parity bit
	if (parity)	{
		gohidata();
	} else {
		golodata();
	}
	// clock cycle - like ack.
	while (!readclck) { if (timeout) { return 0; } }
	while (readclck)  { if (timeout) { return 0; } }
	// stop bit
	gohidata();
	delayMicroseconds(10);
	while (readclck) { if (timeout) { return 0; } }
	// mode switch
	while (!readclck || !readdata) { if (timeout) { return 0; } }
	// hold up incoming data
	goloclck();
	return 1;
}


/*
 * read a byte of data from the ps2 device.  Ignores parity.
 */
char PS2Mouse::read(void)
{
	unsigned char data = 0x00;
	unsigned char i;
	unsigned char bit = 0x01;

	// start clock
	gohiclck();
	gohidata();
	delayMicroseconds(10);
	while (readclck)  { if (timeout) { return 0; } }
	delayMicroseconds(10);	// not sure why.
	while (!readclck) { if (timeout) { return 0; } }  // eat start bit
	for (i=0; i < 8; i++)
	{
		while (readclck)  { if (timeout) { return 0; } }
		if (readdata)  {
			data = data | bit;
		}
		while (!readclck) { if (timeout) { return 0; } }
		bit = bit << 1;
	}
	
	// eat parity bit, ignore it.
	while (readclck)  { if (timeout) { return 0; } }
	while (!readclck) { if (timeout) { return 0; } }
	// eat stop bit
	while (readclck)  { if (timeout) { return 0; } }
	while (!readclck)  { if (timeout) { return 0; } }
	goloclck();

	return data;
}

/*
 * get a reading from the mouse and report it back to the
 * host via the serial line.
 */
char PS2Mouse::update(void)
{
	timer = micros();
	if (!write(0xeb)) { // ask for data
		return 0;
	}
    read(); // ignore ack
    status = read();
    x = read();
    y = read();	
	return 1;
}


/*
 * initialize the mouse. Reset it, and place it into remote
 * mode, so we can get the encoder data on demand.
 */
char PS2Mouse::init(void)
{
	timer = micros();
    if (!write(0xff)) { // reset mouse
		return 0;	
	}
    read(); // ack byte
    read(); // blank */
    read(); // blank */
    if (!write(0xf0)) { // remote mode
		return 0;	
	}	
    read(); // ack
	return 1;	
}
