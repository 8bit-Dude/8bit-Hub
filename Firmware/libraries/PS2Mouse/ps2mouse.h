/*
 * ps2.h - a library to interface with ps2 devices. See comments in
 * ps2.cpp.
 * Written by Chris J. Kiick, January 2008.
 * Release into public domain.
 */

#ifndef ps2_h
#define ps2_h

#include "Arduino.h"

class PS2Mouse
{	
	public:
		char x,y;
		char status;	
		PS2Mouse();
		char init(void);
		char update(void);
	private:
		char write(unsigned char data);
		char read(void);
};

#endif /* ps2_h */

