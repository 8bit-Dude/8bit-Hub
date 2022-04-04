/*
 * Copyright (c) 2022 Anthony Beaucamp.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 *   1. The origin of this software must not be misrepresented * you must not
 *   claim that you wrote the original software. If you use this software in a
 *   product, an acknowledgment in the product documentation would be
 *   appreciated but is not required.
 *
 *   2. Altered source versions must be plainly marked as such, and must not
 *   be misrepresented as being the original software.
 *
 *   3. This notice may not be removed or altered from any distribution.
 *
 *   4. The names of this software and/or it's copyright holders may not be
 *   used to endorse or promote products derived from this software without
 *   specific prior written permission.
 */
 
#include "hub.h" 

// Read/Write functions
#if defined(__APPLE2__)
  #include <serial.h>
  #include <peekpoke.h>
  unsigned char __fastcall__ SerialOpen(void* ptr);
  unsigned char __fastcall__ SerialGet(unsigned char* data);
  unsigned char __fastcall__ SerialPut(unsigned char data);
  const struct ser_params comParm = { SER_BAUD_19200, SER_BITS_8, SER_STOP_1, SER_PAR_NONE, SER_HS_HW };
  unsigned char byte;
  unsigned char RecvByte() {
	unsigned int i = 512;
	while (i) {  // Countdown i to 0
		// This code works only in AppleWin (interrupt fires off)
		if (SerialGet(&byte) == SER_ERR_OK) 
			return 1;
		
		// This code works on real hardware
		if (PEEK(0xC089+32)&8) {
			byte = PEEK(0xC088+32);
			return 1;
		}		
		i--;
	}
	return 0;
  }
  unsigned char SendByte() {
	unsigned char i = 255;
	while (i) {  // Countdown i to 0
		if (SerialPut(byte) == SER_ERR_OK) 	// Send byte
			return 1;
		i--;
	}
	return 0;
  }  
#elif defined(__LYNX__)
  #include <serial.h>
  unsigned char __fastcall__ SerialOpen(void* ptr); 
  unsigned char __fastcall__ SerialGet(unsigned char* data);
  unsigned char __fastcall__ SerialPut(unsigned char data);
  unsigned char comParm = 0; 
  unsigned char byte; 
  unsigned char RecvByte() {
	unsigned int i = 512;
	while (i) {  // Countdown i to 0
		if (SerialGet(&byte) == SER_ERR_OK)		// Wait for byte to arrive on serial port
			return 1;
		i--;
	}
	return 0;
  }
  unsigned char SendByte() {
	unsigned char i = 255;
	while (i) {  // Countdown i to 0
		if (SerialPut(byte) == SER_ERR_OK) {	// Send byte
			while (i) {
				if (SerialGet(&byte) == SER_ERR_OK) // Read back byte (sent to oneself)	
					return 1;
				i--;
			}
			return 0;
		}
		i--;
	}
	return 0;
  }
#else
 #ifndef __NES__
  void InputMode();
  void OutputMode(); 
 #endif
 #ifdef __ATMOS__
  void ClosePort();
 #endif  
  unsigned char SendByte();
  unsigned char RecvByte();
  extern unsigned char byte;
#endif

// Hub State
unsigned char hubState[7] = { COM_ERR_OFFLINE, 255, 255, 255, 255, 255, 255 };
unsigned char hubID, hubOutLen, *hubOutBuffer, hubInLen, hubInBuffer[HUB_BUFFER_LEN];

// Hub Functions
unsigned char InitHub(void) 
{
	// Was hub already initialized?
	unsigned char hubVersion = HUB_CLIENT_VER;
	if (hubState[0] == COM_ERR_OFFLINE) {
		// Setup serial port
	  #if defined(__LYNX__) || defined(__APPLE2__)
		SerialOpen(&comParm); 
	  #endif	
	  
		// Setup initial request
		hubOutBuffer = &hubVersion; hubOutLen = 1;
		SendHub(HUB_SYS_RESET);
		
		// Check result
		if (hubState[0] == COM_ERR_OK) {
			hubID = 0;
		} else {
			return 0;
		}
	}
	return 1;
}

unsigned char SendHub(unsigned char cmd)
{
	unsigned char i, j = 0, checksum, acknow = 0;
	
  #if defined(__ATMOS__)
	__asm__("sei");		// Disable interrupts
  #endif	
	
	while (j++ < HUB_SEND_RETRY) {
		/////////////////////
		// Setup for sending
	  #if defined(__LYNX__) || defined(__APPLE2__)
		while (SerialGet(&i) == SER_ERR_OK); // Clear UART Buffer
	  #elif defined(__ATARI__) || defined(__CBM__) || defined(__ATMOS__)
		OutputMode();
	  #endif	

		// Send Header
		byte = 170; if (!SendByte()) goto SendError;
		byte = cmd; if (!SendByte()) goto SendError;
		
		// Send Packet Data (if any)
		checksum = cmd;
		byte = hubOutLen; if (!SendByte()) goto SendError;
		if (hubOutLen) {
			for (i=0; i<hubOutLen; i++) {
				byte = hubOutBuffer[i]; checksum += byte;
				if (!SendByte()) goto SendError;
			}
		}
		
		// Send footer
		byte = checksum; if (!SendByte()) goto SendError;

		///////////////////////
		// Setup for receiving
	  #if defined(__ATARI__) || defined(__CBM__) || defined(__ATMOS__)  
		InputMode();
	  #endif		

		// Check acknowledgment
		if (RecvByte() && byte == 85) {
			acknow = 1;
			hubState[0] = COM_ERR_OK;
			goto SendError; // actually means ok!
		}
	}
	
	SendError:
  #if defined(__ATMOS__)
	ClosePort();
	__asm__("cli");		// Resume interrupts	
  #endif
  
	return acknow;
}

unsigned char RecvHub(unsigned char cmd) 
{
	unsigned char i, ID, checksum, *buf, acknow = 0;

  #if defined(__ATMOS__)
	__asm__("sei");		// Disable interrupts
  #endif	

	/////////////////////
	// Setup for sending
  #if defined(__LYNX__) || defined(__APPLE2__)
	while (SerialGet(&i) == SER_ERR_OK); // Clear UART Buffer
  #elif defined(__ATARI__) || defined(__CBM__) || defined(__ATMOS__)
	OutputMode();
  #endif	
  
	// Make receive request
	byte = 85;    if (!SendByte()) goto RecvError;
	byte = cmd;   if (!SendByte()) goto RecvError;
	byte = hubID; if (!SendByte()) goto RecvError;
  
	///////////////////////
	// Setup for receiving
  #if defined(__ATARI__) || defined(__CBM__) || defined(__ATMOS__)  
	InputMode();
  #endif	
	
	// Check header
	if (!RecvByte()) goto RecvError;
	if (byte != 170) {
		hubState[0] = COM_ERR_HEADER; 
		goto RecvError;
	}
	hubState[0] = COM_ERR_TRUNCAT; 
					
	// Get packet ID
	if (!RecvByte()) goto RecvError; 
	checksum = ID = byte;
	
	// Get buffer length
	if (!RecvByte()) goto RecvError; 
	hubInLen = byte;
	
	// Assign buffer pointer
	if (cmd == HUB_SYS_STATE)
		buf = &hubState[1];
	else
		buf = hubInBuffer;

	// Read buffer data
	for (i=0; i<hubInLen; i++) {
		if (!RecvByte()) goto RecvError; 
		*buf++ = byte;
		checksum += byte;
	}

	// Verify checkum
	if (!RecvByte()) goto RecvError; 
	if (byte != checksum) { 
		hubState[0] = COM_ERR_CORRUPT; 
		goto RecvError; 
	}
	hubState[0] = COM_ERR_OK;
	
	// All good?
	if (cmd == HUB_SYS_STATE) {
		acknow = 1;
		goto RecvError;
	} else
	if (hubID != ID) {
		hubID = ID;
		acknow = 1;
		goto RecvError;
	}

	RecvError:
  #if defined(__ATMOS__)
	ClosePort();
	__asm__("cli");		// Resume interrupts	
  #endif  	
	  
	return acknow;
}
