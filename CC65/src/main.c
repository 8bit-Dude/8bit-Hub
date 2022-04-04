
#include "hub.h"
#include <time.h>

#ifdef __LYNX__
  #include <lynx.h>
  #include <tgi.h>
  #include <6502.h>
#else
  #include <conio.h>
#endif

// Implement missing API function
#if defined(__APPLE2__)
  clock_t clk = 0;
  clock_t clock() {
	return clk++;
  }
#elif defined(__LYNX__)
  void clrscr(void) {
	// TGI Init
	tgi_install(tgi_static_stddrv);
	tgi_init();
	CLI();
	while (tgi_busy());	
  }
  void gotoxy(unsigned char x, unsigned char y) {
	// TGI cursor
	tgi_gotoxy(x,y);
  }
  void cprintf(const char *string) {
	// TGI print
	tgi_outtext(string); 
	tgi_updatedisplay();
  }
#endif

							  //         IP		  | SVR port| CLI port (4*256+210 = 1234)
const unsigned char server[] = { 199, 47, 196, 106, 210,   4, 210,   4 };

void main(void) 
{
	clock_t timeout;

	clrscr();
	
	//////////////////////////	
	// Initialize Hub
	gotoxy(0,0); cprintf("Initializing");
	timeout = clock()+120;
	while (!InitHub()) {
		if (clock() > timeout) {
			gotoxy(0,1); cprintf("Device Timeout");
			while (1);	// Hang forever
		}
	}
	gotoxy(0,1); cprintf("Device OK");
	
	//////////////////////////
	// Open UDP Socket
	hubOutLen = 8;  
	hubOutBuffer = server;  
	SendHub(HUB_UDP_OPEN);	
	
	// Send UDP Packet
	hubOutLen = 13; 
	hubOutBuffer = (unsigned char*)"UDP Received";
	SendHub(HUB_UDP_SEND);
	
	// Wait for Echo from Server
	timeout = clock()+120;
	while(!RecvHub(HUB_UDP_RECV))
		if (clock() > timeout) break; 
	
	// Print result from Echo
	gotoxy(0,2);
	if (hubInLen)
		cprintf((const char*)hubInBuffer);	
	else
		cprintf("UDP Timeout");	
	
	// Close UDP connection
	hubOutLen = 0;
	SendHub(HUB_UDP_CLOSE);	
	
	//////////////////////////
	// Open TCP Socket
	hubOutLen = 8;  
	hubOutBuffer = server;  
	SendHub(HUB_TCP_OPEN);	
	
	// Send TCP Packet
	hubOutLen = 13; 
	hubOutBuffer = (unsigned char*)"TCP Received";
	SendHub(HUB_TCP_SEND);
	
	// Wait for Echo from Server
	timeout = clock()+120;
	while(!RecvHub(HUB_TCP_RECV))
		if (clock() > timeout) break; 
	
	// Print result from Echo	
	gotoxy(0,3);
	if (hubInLen)
		cprintf((const char*)hubInBuffer);	
	else
		cprintf("TCP Timeout");	

	// Close TCP connection
	hubOutLen = 0;
	SendHub(HUB_TCP_CLOSE);	
	
	// Hang forever
	while (1);	
}