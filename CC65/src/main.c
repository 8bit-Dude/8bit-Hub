
#ifdef __LYNX__
  #include <lynx.h>
  #include <tgi.h>
  #include <6502.h>
#else
  #include <conio.h>
#endif

#include <time.h>
#include "hub.h"


#ifdef __LYNX__
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

void main(void) {
									// IP		  SVR port  CLI port (4*256+210 = 1234)
	unsigned char server[] = { 199, 47, 196, 106, 210,   4, 210,   4 };
	clock_t timeout;

	clrscr();
	
	// Initialize Hub
	gotoxy(0,0);
	if (!InitHub()) {
		cprintf("Init Failed");
		while (1);	
	} else {
		cprintf("Init OK");
	}
	
	// Perform UDP test
	hubOutLen = 8;  hubOutBuffer = server;  
	SendHub(HUB_UDP_OPEN);	
	hubOutLen = 13; hubOutBuffer = (unsigned char*)"UDP Received";
	SendHub(HUB_UDP_SEND);
	timeout = clock()+120;
	while(!RecvHub(HUB_UDP_RECV))
		if (clock() > timeout) break; 
	
	gotoxy(0,1);
	if (hubInLen)
		cprintf((const char*)hubInBuffer);	
	else
		cprintf("UDP Timeout");	

	hubOutLen = 0;
	SendHub(HUB_UDP_CLOSE);	
	
	// Perform TCP test
	hubOutLen = 8;  hubOutBuffer = server;  
	SendHub(HUB_TCP_OPEN);	
	hubOutLen = 13; hubOutBuffer = (unsigned char*)"TCP Received";
	SendHub(HUB_TCP_SEND);
	timeout = clock()+120;
	while(!RecvHub(HUB_TCP_RECV))
		if (clock() > timeout) break; 
	
	gotoxy(0,2);
	if (hubInLen)
		cprintf((const char*)hubInBuffer);	
	else
		cprintf("TCP Timeout");	

	hubOutLen = 0;
	SendHub(HUB_TCP_CLOSE);	
	
	// Hang forever
	while (1);	
}