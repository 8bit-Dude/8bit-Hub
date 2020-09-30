
// System libraries
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <picojpeg.h>
#include <SPI.h>
#include <SD.h>

// Debugging
//#define __DEBUG_JOY__
//#define __DEBUG_JPG__
//#define __DEBUG_MOUSE__
//#define __DEBUG_CMD__
//#define __DEBUG_COM__
//#define __DEBUG_PCK__
//#define __DEBUG_TCP__
//#define __DEBUG_UDP__
//#define __DEBUG_WEB__
//#define __DEBUG_HTTP__

// Firmware Version
char espVersion[5] = "?";
char espUpdate[5] = "?";
char megaVersion[5] = "v0.1";
char megaUpdate[5] = "?";
char urlEsp[]  = "http://8bit-unity.com/Hub-ESP8266.bin";
char urlMega[] = "http://8bit-unity.com/Hub-Mega2560.bin";
char urlVer[]  = "http://8bit-unity.com/Hub-Version.txt";

// HUB Commands
#define HUB_SYS_ERROR     0
#define HUB_SYS_RESET     1
#define HUB_SYS_NOTIF     2
#define HUB_SYS_SCAN      3
#define HUB_SYS_CONNECT   4
#define HUB_SYS_IP        5
#define HUB_SYS_MOUSE     6
#define HUB_SYS_VERSION   7
#define HUB_SYS_UPDATE    8
#define HUB_DIR_LS       10  // Todo: Implement for root directory /microSD
#define HUB_DIR_MK       11
#define HUB_DIR_RM       12
#define HUB_DIR_CD       13
#define HUB_FILE_OPEN    21
#define HUB_FILE_SEEK    22
#define HUB_FILE_READ    23
#define HUB_FILE_WRITE   24
#define HUB_FILE_CLOSE   25
#define HUB_UDP_OPEN     30
#define HUB_UDP_RECV     31
#define HUB_UDP_SEND     32
#define HUB_UDP_CLOSE    33
#define HUB_UDP_SLOT     34
#define HUB_TCP_OPEN     40
#define HUB_TCP_RECV     41
#define HUB_TCP_SEND     42
#define HUB_TCP_CLOSE    43
#define HUB_TCP_SLOT     44
#define HUB_WEB_OPEN     50
#define HUB_WEB_RECV     51
#define HUB_WEB_HEADER   52
#define HUB_WEB_BODY     53
#define HUB_WEB_SEND     54
#define HUB_WEB_CLOSE    55
#define HUB_HTTP_GET     60
#define HUB_HTTP_READ    61

// String definitions
#ifdef __DEBUG_CMD__
  const char* cmdString[] = 
  {"SYS_ERROR","SYS_RESET","SYS_NOTIF", "SYS_SCAN",  "SYS_CONNECT","SYS_IP",   "SYS_MOUSE","SYS_VERSION","SYS_UPDATE","",
   "DIR_LS",   "DIR_MK",   "DIR_RM",    "DIR_CD",    "",           "",         "",         "",           "",          "",
   "FILE_OPEN","FILE_SEEK","FILE_READ", "FILE_WRITE","FILE_CLOSE", "",         "",         "",           "",          "",
   "UDP_OPEN", "UDP_RECV", "UDP_SEND",  "UDP_CLOSE", "UDP_SLOT",   "",         "",         "",           "",          "",
   "TCP_OPEN", "TCP_RECV", "TCP_SEND",  "TCP_CLOSE", "TCP_SLOT",   "",         "",         "",           "",          "",
   "WEB_OPEN", "WEB_RECV", "WEB_HEADER","WEB_BODY",  "WEB_SEND",   "WEB_CLOSE","",         "",           "",          "",
   "HTTP_GET", "HTTP_READ","",          "",          "",           "",         "",         "",           "",          ""};
#endif
const char* blank = "                    ";

// Useful macros
#define MIN(a,b) (a>b ? b : a)
#define MAX(a,b) (a>b ? a : b)

// HUB Modes
#define MODE_C64   0
#define MODE_ATARI 1
#define MODE_APPLE 2
#define MODE_ORIC  3
#define MODE_LYNX  4
#define MODE_NES   5
#define MODE_BBC   6
#define HUB_MODES  7
const char* modeString[HUB_MODES] = {"C64/C128", "Atari XL/XE", "Apple //", "Oric", "Lynx", "NES", "BBC"};
byte hubMode = MODE_LYNX;

// COMM Params
#define FILES    16    // Number of file handles
#define SLOTS    16    // Number of connection handles
#define PACKET   256   // Max. packet length (bytes)
#define TIMEOUT  1000  // Packet timout (milliseconds)

// IP Params
char IP[17] = "Not connected...";

// SD Card Paramds
File hubFile[FILES];

// Buffers for data exchange with ESP8266
char serBuffer[PACKET];
unsigned char serLen;

////////////////////////////////
//      PACKET functions      //
////////////////////////////////

// Define packet structure
typedef struct packet {
    unsigned char ID;
    unsigned char cmd;
    unsigned char len;
    unsigned char* data;
    uint32_t timeout;
    struct packet* next;
} packet_t;

packet_t* packetHead = NULL;
unsigned char packetID = 0;

void pushPacket(unsigned char cmd, signed char slot) {
  // Create new packet
    packet_t* packet = malloc(sizeof(packet_t));
    packet->next = NULL;

    // Assign ID & Timeout
    if (++packetID>15) { packetID = 1; }
    packet->ID = packetID;
    packet->timeout = millis() + TIMEOUT;

    // Copy data to packet
    packet->len = serLen+2;
    packet->data = (unsigned char*)malloc(serLen+2);
    packet->data[0] = cmd;
    packet->data[1] = slot;
    memcpy(&packet->data[2], serBuffer, serLen);
    
    // Append packet at tail of linked list
    if (!packetHead) {
        packetHead = packet;
    } else {
        packet_t *packetTail = packetHead;
        while (packetTail->next != NULL) {
            packetTail = packetTail->next;
        }
        packetTail->next = packet;
    }
#ifdef __DEBUG_PCK__
    Serial.print("PUSH: "); Serial.println((byte)packet->ID);
#endif
}

void popPacket(unsigned char ID) {
    if (packetHead && packetHead->ID == ID) { 
        // Remove packet at head of linked list
    #ifdef __DEBUG_PCK__
        Serial.print("POP: "); Serial.println((byte)packetHead->ID);    
    #endif
        packet_t* next = packetHead->next;
        free(packetHead->data); free(packetHead); 
        packetHead = next;      
    }
}

void packetTimeout() {
  // Remove packets that have exceeded timeout
  while (packetHead && millis() > packetHead->timeout) {
      popPacket(packetHead->ID);
  }
}

//////////////////////////
//    List functions    //
//////////////////////////

// Define list structure
typedef struct list {
    char* data;
    struct list* next;
} list_t;

list_t* listHead;

void pushList(char* data) {
    // Create new packet
    list_t* elt = malloc(sizeof(list_t));
    elt->data = data;
    elt->next = NULL;

    // Append packet at tail of linked list
    if (!listHead) {
        listHead = elt;
    } else {
        list_t *tail = listHead;
        while (tail->next != NULL) {
            tail = tail->next;
        }
        tail->next = elt;
    }     
}

unsigned char* getList(unsigned char index) {
    byte i=0;
    list_t* elt = listHead;
    while (elt && i<index) {
        elt = elt->next;
        i++;
    }
    if (elt) {
        return elt->data;
    } else {
        return NULL;
    }
}

unsigned char lenList() {
    byte i=0;
    list_t* elt = listHead;
    while (elt) {
        elt = elt->next;
        i++;
    }
    return i;    
}

void clearList() {
    list_t* elt;
    while (listHead) {
        elt = listHead;
        listHead = elt->next;
        free(elt);
    }
}

////////////////////////////////
//    PERIPHERAL functions    //
////////////////////////////////

// Joystick states
#define JOY_UP    1
#define JOY_DOWN  2
#define JOY_LEFT  4
#define JOY_RIGHT 8
#define JOY_FIRE1 16
#define JOY_FIRE2 32

// Joystick modes
#define JOY_STD  0
#define JOY_PASE 1

// Joystick params
unsigned char joyMode = JOY_STD;
unsigned char joyState[4] = { 255, 255, 255, 255 };
unsigned char joyPins[4][8] = { { 23, 25, 27, 29, 22, 24, 28, 26 },   // U, D, L, R, F1, F2, Alt F2, GND
                                { 31, 33, 35, 37, 30, 32, 36, 34 },
                                { 39, 41, 43, 45, 38, 40, 44, 42 },
                                {  0,  0,  0,  0,  0,  0,  0,  0 } }; // Joy #4 unassigned in current version
unsigned char joyStnd[6] = {   1,  2,  4,  8, 16, 32 };
unsigned char joyPase[6] = {  16,  8,  1,  2,  4,  0 };

// Mouse params
unsigned char mouseState[2] = { 80, 100 };

void setupJOY() {
    // Set I/O pins
    for (byte i=0; i<3; i++) {
        for (byte j=0; j<7; j++) {
            pinMode(joyPins[i][j], INPUT_PULLUP);  // Inputs
        }
        pinMode(joyPins[i][7], OUTPUT);            // Ground
        digitalWrite(joyPins[i][7], LOW);
    }  

    // Be careful with Atari 7800 joypads (Pin 9 causes short-circuit)
    for (byte i=0; i<3; i++) { 
        if (!digitalRead(joyPins[i][6])) joyPins[i][6] = joyPins[i][5];
    }
}

void readJOY() {
    // Read digital pins
    switch (joyMode) {
    case JOY_STD:    
        for (byte i=0; i<3; i++) { 
            joyState[i] |= 63;
            for (byte j=0; j<6; j++) {
                if (!digitalRead(joyPins[i][j])) { joyState[i] &= ~joyStnd[j]; }  
            } 
            if (!digitalRead(joyPins[i][6])) { joyState[i] &= ~joyStnd[5]; } 
        }  
        break;
    case JOY_PASE:
        for (byte i=0; i<2; i++) { 
            joyState[i] = 32;
            for (byte j=0; j<6; j++) {
                if (digitalRead(joyPins[i][j])) { joyState[i] += joyPase[j]; }  
            }
            if (!digitalRead(joyPins[i][6])) { joyState[i] &= ~joyPase[5]; } 
        }
        break;
    }
#ifdef __DEBUG_JOY__
    Serial.print("Joystick: "); 
    for (byte i=0; i<4; i++) { Serial.println(joyState[i]); Serial.print(","); }
#endif    
}

#define MAX(a,b) (a>b?a:b)
#define MIN(a,b) (a>b?b:a)

void readMouse() {
    unsigned char info = readChar();
    signed int x = readChar();
    signed int y = readChar();
    joyState[0] |= 192; joyState[1] |= 192;
    if (info&2) { joyState[0] &= ~64;  }  // L Button
    if (info&4) { joyState[0] &= ~128; }  // R Button
    if (info&8) { joyState[1] &= ~64;  }  // M Button
    if (info&32) { x = -(255-x); }
    if (info&64) { y = -(255-y); }
    x /= 4; y /= -3;
    if (x < 0) { mouseState[0] -= MIN(mouseState[0], -x); }
    if (x > 0) { mouseState[0] += MIN(159-mouseState[0], x); }
    if (y < 0) { mouseState[1] -= MIN(mouseState[1], -y); }
    if (y > 0) { mouseState[1] += MIN(199-mouseState[1], y); }
#ifdef __DEBUG_MOUSE__   
    Serial.print("Mouse: "); Serial.print((info&2)>0); // L but
    Serial.print(","); Serial.print((info&8)>0);       // M but
    Serial.print(","); Serial.print((info&4)>0);       // R but
    Serial.print(","); Serial.print(mouseState[0]);
    Serial.print(","); Serial.println(mouseState[1]);
#endif        
}

////////////////////////////////
//        JPG functions       //
////////////////////////////////
/*
const unsigned char lynxPaletteR[] = {165,  66,  66,  49,  49,  33,  33, 115, 231, 247, 247, 214, 132, 247, 132, 0};
const unsigned char lynxPaletteG[] = { 16,  49, 132, 198, 198, 132,  82,  82, 115, 231, 148,  49,  33, 247, 132, 0};
const unsigned char lynxPaletteB[] = {198, 181, 198, 198,  82,  33,  82,  33,  82,   0, 165,  66,  66, 247, 132, 0};

File jpgFile;
unsigned int jpgSize, jpgOfs;

uint8_t callbackJPG(uint8_t* pBuf, uint8_t buf_size, uint8_t *pBytes_actually_read, void *pCallback_data) {
    unsigned int n = MIN(jpgSize - jpgOfs, buf_size);
    *pBytes_actually_read = (uint8_t)(n);
    jpgFile.read(pBuf,n);
    jpgOfs += n;
    return 0;    
}

void decodeJPG(unsigned char* filename) {
    // Open file from SD card
    uint32_t timer = millis();     

    // Open jpeg file from SD card
    jpgFile = SD.open(filename, FILE_READ);
    jpgSize = jpgFile.size();
    jpgOfs = 0;

    // Initialize decoder
    pjpeg_image_info_t jpgInfo;
    unsigned char status = pjpeg_decode_init(&jpgInfo, callbackJPG, NULL, 0);
    if (status) {
      Serial.print("pjpeg_decode_init() failed with status ");
      Serial.println(status);
      if (status == PJPG_UNSUPPORTED_MODE) {
        Serial.println("Progressive JPEG files are not supported.");
      }
      return;
    }
    
    // Open cache file to store decoded image
    if (SD.exists("cache")) { SD.remove("cache"); }
    File cacheFile = SD.open("cache", FILE_WRITE);
    
    // Scanning variables
    unsigned int xElt, yElt;
    unsigned char xElt8, yElt8;
    unsigned char xMin, xMax, yMin, yMax;
    unsigned int palDist, palMini;
    unsigned char palIndex;
    unsigned char bmp[9][82];

    // Scan through block rows
    for (unsigned int yMCU=0; yMCU<jpgInfo.m_height; yMCU+=jpgInfo.m_MCUHeight) {

        // Computer's Y coordinate range
        yMin = (102 * (yMCU                    )) / jpgInfo.m_height;
        yMax = (102 * (yMCU+jpgInfo.m_MCUHeight)) / jpgInfo.m_height;
        yMax = MIN(yMax, 102);

        // Reset bmp array
        for (unsigned int y=yMin; y<yMax; y++) {
            bmp[y-yMin][0] = 0x52;
            memset(&bmp[y-yMin][1], 0, 81);
        }

        // Scan through block cols
        for (unsigned int xMCU=0; xMCU<jpgInfo.m_width; xMCU+=jpgInfo.m_MCUWidth) {

            // Computer's X coordinate range
            xMin = (160 * (xMCU                   )) / jpgInfo.m_width;
            xMax = (160 * (xMCU+jpgInfo.m_MCUWidth)) / jpgInfo.m_width;
            xMax = MIN(xMax, 160);

            // Decode this block
            pjpeg_decode_mcu();
    
            // Scan Y coordinate range
            for (unsigned int y=yMin; y<yMax; y++) {
                yElt = (y * jpgInfo.m_height)/102 - yMCU;  yElt8 = yElt/8;
                //Serial.print((byte)y); Serial.print(": ");
    
                // Scan X coordinate range
                for (unsigned int x=xMin; x<xMax; x++) {
                    xElt = (x * jpgInfo.m_width)/160 - xMCU;  xElt8 = xElt/8;
                    //Serial.print((byte)x); Serial.print(", ");
    
                    // Compute offset of element
                    unsigned int block_ofs = (xElt8 * 8U) + (yElt8 * 16U);
                    unsigned int elt_oft = 8*(yElt%8) + (xElt%8);
                    const uint8_t *pSrcR = jpgInfo.m_pMCUBufR + block_ofs + elt_oft;
                    const uint8_t *pSrcG = jpgInfo.m_pMCUBufG + block_ofs + elt_oft;
                    const uint8_t *pSrcB = jpgInfo.m_pMCUBufB + block_ofs + elt_oft;
    
                    // Find matching palette color
                    palMini = 3*256;
                    for (byte i=0; i<16; i++) {
                        palDist = abs(lynxPaletteR[i] - *pSrcR) + abs(lynxPaletteG[i] - *pSrcG) + abs(lynxPaletteB[i] - *pSrcB);
                        if (palDist<palMini) {
                            palIndex = i; 
                            palMini = palDist;
                        }
                    }

                    // Save to bmp buffer
                    bmp[y-yMin][x/2+1] |= palIndex << (4*((x+1)%2));
                    //Serial.print(palIndex); Serial.print(",");                
                }
                //Serial.println();
            }
            //Serial.println();
        }

        // Write to char file
        cacheFile.write(&bmp[0][0], 82*(yMax-yMin));
    }

    // Close files
    cacheFile.write((byte)0x00);
    cacheFile.close();  
    jpgFile.close();
#ifdef __DEBUG_JPG__ 
    Serial.println(F("JPEG image info"));
    Serial.print("Width      : "); Serial.println(jpgInfo.m_width);
    Serial.print("Height     : "); Serial.println(jpgInfo.m_height);
    Serial.print("Scan type  : "); Serial.println(jpgInfo.m_scanType);
    Serial.print("MCU/row    : "); Serial.println(jpgInfo.m_MCUSPerRow);
    Serial.print("MCU/col    : "); Serial.println(jpgInfo.m_MCUSPerCol);
    Serial.print("MCU/width  : "); Serial.println(jpgInfo.m_MCUWidth);
    Serial.print("MCU/height : "); Serial.println(jpgInfo.m_MCUHeight);
    Serial.print("Render time: "); Serial.print(millis()-timer); Serial.println(" ms");
#endif  
}
*/
////////////////////////////////
//        SD functions        //
////////////////////////////////

unsigned char sdInserted = 0;

void printDir(File dir, int numTabs) {
    while (true) {
        File entry =  dir.openNextFile();
        if (!entry) {
          // no more files
          break;
        }
        for (uint8_t i=0; i<numTabs; i++) {
          Serial.print('\t');
        }
        Serial.print(entry.name());
        if (entry.isDirectory()) {
          Serial.println("/");
          printDir(entry, numTabs+1);
        } else {
          // files have sizes, directories do not
          Serial.print("\t\t");
          Serial.println(entry.size(), DEC);
        }
        entry.close();
    }
}

void setupSD() {  
    // Try to init SD card
    Serial.println("Initializing SD card...");
    if (!SD.begin(53)) {
        Serial.println("Notif: No Micro-SD card inserted");
        sdInserted = 0;
        return;
    }
    
    // Open root folder
    File root = SD.open("/");
    sdInserted = 1;

    // Debugging
    //decodeJPG("banner.jpg");    
    //printDir(root, 1); 
}

////////////////////////////////
//        LCD functions       //
////////////////////////////////

LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);

void setupLCD() {
    // Setup LCD light
    lcd.begin(20,4);
    lcd.setBacklightPin(3,POSITIVE);
    lcd.setBacklight(HIGH);
    displayHeader();
}

void displayHeader() {
    // Show header
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("    < 8bit-Hub >");
}

void displayMode() {
    lcd.setCursor(0,1);
    lcd.print("Mode:               ");
    lcd.setCursor(5,1);
    lcd.print(modeString[hubMode]);
    Serial.print("Hub Mode: ");
    Serial.println(modeString[hubMode]);    
}

void displaySD() {
    lcd.setCursor(0,2);
    lcd.print("SD:");
    if (sdInserted) {
        lcd.print("Inserted");
    } else {
        lcd.print("Not inserted");
    }
}

void displayIP() {
    lcd.setCursor(0,3);
    lcd.print("IP:                 ");
    lcd.setCursor(3,3);
    lcd.print(IP);
}

//////////////////////
//    COM Packets   //
//////////////////////

#define COM_ERR_OK      0
#define COM_ERR_NODATA  1
#define COM_ERR_HEADER  2 
#define COM_ERR_TRUNCAT 3
#define COM_ERR_CORRUPT 4

volatile unsigned char checksum;
volatile unsigned char hubID, hubLen, *hubBuffer;
volatile unsigned char prevID = 0, comID = 0, comLen = 0, comBuffer[PACKET];
volatile byte comCode = COM_ERR_NODATA;

void setupCOM() {
    // Setup COM Connection
    switch (hubMode) {
    case MODE_LYNX:
        // Setup Serial 2 on pins 16/17
        Serial2.begin(62500, SERIAL_8N2);   // Lynx comm (Bauds 62500, 41666, 9600)
        while (!Serial2) { }
        Serial2.setTimeout(30);
        Serial2.flush();
        Serial2.readString();
        break;
    case MODE_ORIC:
        oricPort(INPUT);
        pinMode(2, OUTPUT);       // ACKNOW
        digitalWrite(2, LOW);          
        pinMode(3, INPUT_PULLUP); // STROBE
        attachInterrupt(digitalPinToInterrupt(3), oricRecvPacket, FALLING);    
    }
}

void preparePacket() {
    // Do we have packet ready to send?
    if (packetHead) {
        hubID = packetHead->ID;
        hubLen = packetHead->len;
        hubBuffer = packetHead->data;
    } else {
        hubLen = 0;
    }
    
    // Pre-calculate checksum
   checksum = (hubID << 4) + (comID & 0x0f);
    for (byte i=0; i<4; i++)   { checksum += joyState[i]; }
    for (byte i=0; i<2; i++)   { checksum += mouseState[i]; }
    for (byte i=0; i<hubLen; i++) { checksum += hubBuffer[i]; }
}

void checkPacket() {
    // Verify checksum
    checksum = comID;
    for (byte i=0; i<comLen; i++)
        checksum += comBuffer[i]; 
    if (comBuffer[comLen] != checksum) { 
        comCode = COM_ERR_CORRUPT;
        return;
    }
        
    // All good: process ID related stuff
    comCode = COM_ERR_OK;
    comBuffer[comLen] = 0;
    if (comID != prevID) {
        popPacket(comID >> 4);
        if (comID & 0x0f == prevID & 0x0f)
            comLen = 0;   // We already got this packet!
        prevID = comID;
    }
}

////////////////////////////////
//     LYNX Communication     //
////////////////////////////////

unsigned char lynxBitPeriod = 16;   // Bauds: 62500=16 / 41666=24 / 9600=104
unsigned long lynxTimer;

void lynxWrite(char value) {
    unsigned char i, parity = 0, mask = 1;

    // Start Bit
    PORTD &= B11111011;
    lynxTimer += lynxBitPeriod; while (micros()-lynxTimer < lynxBitPeriod) ;

    // Value Bits
    for (i=0; i<8; i++) {
        if (value & mask) { 
            PORTD |= B00000100;
            parity++;
        } else { 
            PORTD &= B11111011;
        }
          mask = mask << 1;
          lynxTimer += lynxBitPeriod; while (micros()-lynxTimer < lynxBitPeriod) ;
    }
      
    // Parity Bit
    if (parity % 2) {
        PORTD &= B11111011;
    } else {
        PORTD |= B00000100;
    }
    lynxTimer += lynxBitPeriod; while (micros()-lynxTimer < lynxBitPeriod) ; 

    // Stop Bit
    PORTD |= B00000100;
    for (i=0; i<6; i++) {   // Bauds: 41666,62500 = i<6 / 9600 = i<1
        lynxTimer += lynxBitPeriod; while (micros()-lynxTimer < lynxBitPeriod) ;
    }
}

void lynxSendPacket() {
    // Switch pin to output
    PORTD |= B00000100;
    DDRD |= B00000100;
    lynxTimer = micros();  
    while (micros()-lynxTimer < lynxBitPeriod) ;

    // Send: header, states, length, data, checksum
    lynxWrite(170);
    lynxWrite((hubID << 4) + (comID & 0x0f));
    for (byte i=0; i<4; i++)   { lynxWrite(joyState[i]); }
    for (byte i=0; i<2; i++)   { lynxWrite(mouseState[i]); }
    lynxWrite(hubLen);
    for (byte i=0; i<hubLen; i++) { lynxWrite(hubBuffer[i]); }
    lynxWrite(checksum);

    // Return pin to input and clear data sent to self
    DDRD &= B11111011;
    Serial2.readString();
    //Serial2.readBytes(data, rxLen+3);
}

void lynxRecvPacket() {  
    // Have we got data?
    if (!Serial2.available()) { comCode = COM_ERR_NODATA; return; }

    // Check Header
    if (Serial2.read() != 170) { comCode = COM_ERR_HEADER; return; }

    // Get RecvID
    if (!Serial2.readBytes((unsigned char*)&comID, 1)) { comCode = COM_ERR_TRUNCAT; return; }

    // Get Length
    if (!Serial2.readBytes((unsigned char*)&comLen, 1)) { comCode = COM_ERR_TRUNCAT; return; }

    // Get Buffer+Footer
    if (!Serial2.readBytes((unsigned char*)comBuffer, comLen+1)) { comCode = COM_ERR_TRUNCAT; return; }

    // Check Packet for Corruption
    checkPacket();
}

////////////////////////////////
//     ORIC Communication     //
////////////////////////////////

// Swith data pins status (I/O)
void oricPort(byte state) {
    // Faster pin switching (takes 4us)
    if (state == OUTPUT) {
        DDRG |= _BV(PG5);
        DDRE |= _BV(PE3);
        DDRH |= _BV(PH3);
        DDRH |= _BV(PH4);
        DDRD |= _BV(PD2);
        DDRD |= _BV(PD3);
        DDRH |= _BV(PH0);    
        DDRH |= _BV(PH1);    
    } else {
        DDRG &= ~_BV(PG5);
        DDRE &= ~_BV(PE3);
        DDRH &= ~_BV(PH3);
        DDRH &= ~_BV(PH4);
        DDRD &= ~_BV(PD2);
        DDRD &= ~_BV(PD3);
        DDRH &= ~_BV(PH0);    
        DDRH &= ~_BV(PH1);    
    }
}

// Read from pins
unsigned char oricRead() {
    // Faster pin reading (takes 8us)
    unsigned char input = 0;
    input |= ((PING & _BV(PG5))>0) << 0;
    input |= ((PINE & _BV(PE3))>0) << 1;
    input |= ((PINH & _BV(PH3))>0) << 2;
    input |= ((PINH & _BV(PH4))>0) << 3;
    input |= ((PIND & _BV(PD2))>0) << 4;
    input |= ((PIND & _BV(PD3))>0) << 5;
    input |= ((PINH & _BV(PH0))>0) << 6;
    input |= ((PINH & _BV(PH1))>0) << 7;
    return input;   
}

// Write to pins
void oricWrite(byte value) {
    // Faster pin writing
    if (value &   1) { PORTG |= _BV(PG5); } else { PORTG &= ~_BV(PG5); }
    if (value &   2) { PORTE |= _BV(PE3); } else { PORTE &= ~_BV(PE3); }
    if (value &   4) { PORTH |= _BV(PH3); } else { PORTH &= ~_BV(PH3); }
    if (value &   8) { PORTH |= _BV(PH4); } else { PORTH &= ~_BV(PH4); }
    if (value &  16) { PORTD |= _BV(PD2); } else { PORTD &= ~_BV(PD2); }
    if (value &  32) { PORTD |= _BV(PD3); } else { PORTD &= ~_BV(PD3); }
    if (value &  64) { PORTH |= _BV(PH0); } else { PORTH &= ~_BV(PH0); }
    if (value & 128) { PORTH |= _BV(PH1); } else { PORTH &= ~_BV(PH1); }

    // Blip ACKNOW
    PORTE |= _BV(PE4);
    delayMicroseconds(10);
    PORTE &= ~_BV(PE4);
    delayMicroseconds(250);
}

void oricSendPacket() {
    // Switch pins to output
    oricPort(OUTPUT);
    delayMicroseconds(50);

    // Send: header, states, length, data, checksum
    oricWrite(170);
    oricWrite((hubID << 4) + (comID & 0x0f));
    for (byte i=0; i<4; i++)   { oricWrite(joyState[i]); }
    for (byte i=0; i<2; i++)   { oricWrite(mouseState[i]); }
    oricWrite(hubLen);
    for (byte i=0; i<hubLen; i++) { oricWrite(hubBuffer[i]); }
    oricWrite(checksum);
    
    // Switch pins back to input
    oricPort(INPUT);
}

volatile unsigned char hasHeader, hasID, hasLen, rcvLen; 

void oricRecvPacket() {
    // Read byte from computer  
    unsigned char data = oricRead();

    // Check header
    if (!hasHeader) {
        switch (data) {
        case 85:
            preparePacket();
            oricSendPacket(); 
            return;
        case 170:
            hasHeader = 1; 
            return;
        default:
            return;
        }
    }
  
    // Check ID
    if (!hasID) {
      comID = data;
      hasID = 1;
      return;
    }
  
    // Check for length
    if (!hasLen) {
      comLen = data;
      rcvLen = 0;
      hasLen = 1;
      return;
    }
  
    // Add data to buffer
    comBuffer[rcvLen++] = data;
  
    // Check if packet was fully received (including extra byte for checksum)
    if (rcvLen < comLen+1) { return; }
  
    // Check packet and reset state (it will be processed in main loop)
    checkPacket();
    hasHeader = 0;
    hasID = 0;
    hasLen = 0;
}

/////////////////////////////
//    SERIAL Connection    //
/////////////////////////////

void setupSERIAL() {
    // Setup PC/Wifi Connection
    Serial.begin(115200);               // PC conn.
    Serial3.begin(115200);              // ESP8266 conn.
    while (!Serial) { }
    while (!Serial3) { }
    Serial.setTimeout(10);
    Serial3.setTimeout(10);
    Serial.flush();
    Serial3.flush();
    Serial.readString();
    Serial3.readString();

    // Display Reboot Message
    Serial.println("-System Reboot-");    
}

void writeCMD(unsigned char cmd) {
    Serial3.print("CMD");
    Serial3.write(cmd);
}

void writeChar(unsigned char var) {
    Serial3.write(var);
}

void writeInt(unsigned int var) {
    Serial3.write((char*)&var, 2);
}

void writeBuffer(char* buffer, unsigned char len) {
    Serial3.write(len);
    Serial3.write(buffer, len);
}

unsigned char readChar() {
    // Get char from serial  
    uint32_t timeout = millis()+20;    
    while (!Serial3.available()) {
        if (millis() > timeout)
          return 0;
    }    
    return Serial3.read();
}

void readInt(unsigned int *var) {
    // Get one int from serial
    Serial3.readBytes((char*)var, 2);  
}

unsigned char readBuffer() {
    // Read buffer of known length
    uint32_t timeout;
    unsigned char i = 0;
    unsigned char len = readChar();
    while (i<len) {
        timeout = millis()+10;    
        while (!Serial3.available()) {
            if (millis() > timeout) {
                serLen = 0;
                return 0;
            }
        }    
        serBuffer[i++] = Serial3.read();
    }
    serBuffer[len] = 0;
    serLen = len;
    return len;
}

void readIP() {
    if (readBuffer()) {
        strcpy(IP, serBuffer);
        Serial.print("IP: ");
        Serial.println(serBuffer);
    }  
}

void readNotif() {
    if (readBuffer()) {
        Serial.print("Notif: ");
        Serial.println(serBuffer);
    }
}

void readError() {
    if (readBuffer()) {
        Serial.print("Error: ");
        Serial.println(serBuffer);
    }  
}

////////////////////////////
//  UDP/TCP/WEB functions //
////////////////////////////

void readUdp() {
    // Store data into packet
    char slot = readChar(); // slot
    if (readBuffer()) {
        pushPacket(HUB_UDP_RECV, slot);  
    #if defined(__DEBUG_UDP__)
        Serial.print("UDP RECV: ");
        Serial.write(serBuffer, serLen); 
        Serial.print("\n");
    #endif     
    }
}

void readTcp() {
    // Store data into packet
    char slot = readChar(); // slot
    if (readBuffer()) {
        pushPacket(HUB_TCP_RECV, slot);  
    #if defined(__DEBUG_TCP__)
        Serial.print("TCP RECV: ");
        Serial.write(serBuffer, serLen); 
        Serial.print("\n");
    #endif     
    } 
}

void readWeb() {
    // Store data into packet
    if (readBuffer()) {
        pushPacket(HUB_WEB_RECV, -1);
    #if defined(__DEBUG_WEB__)
        Serial.print("WEB RECV: ");
        Serial.write(serBuffer, serLen); 
        Serial.print("\n");
    #endif     
    }   
}

////////////////////////////
//     HTTP functions     //
////////////////////////////

#define HTTP_NULL   0
#define HTTP_PACKET 1
#define HTTP_UPDATE 2

unsigned long httpSize;
unsigned char httpMode = HTTP_PACKET;

void getHttp() {
    // Store data into packet
    if (readBuffer()) {
        unsigned char len[4];
        memcpy(len, serBuffer, 4);
        httpSize = (unsigned long)len[0]+256L*(unsigned long)len[1]+65536L*(unsigned long)len[2]+16777216L*(unsigned long)len[3];
        if (httpMode == HTTP_PACKET) {
            // Send back file size
            memcpy(serBuffer, (char*)&httpSize, 4); serLen = 4;
            pushPacket(HUB_HTTP_GET, -1);
        }
    #if defined(__DEBUG_HTTP__)
        Serial.print("HTTP GET: ");
        Serial.print(httpSize); Serial.println(" bytes");
    #endif     
    }
}

void readHttp() {
    // Store data to Packet/OTA update
    if (readBuffer()) {
        switch (httpMode) {
        case HTTP_PACKET:
            pushPacket(HUB_HTTP_READ, -1);  
            break;
        case HTTP_UPDATE:
            for (unsigned char i=0; i<serLen; i++)
                InternalStorage.write(serBuffer[i]);
            break;
        }
    #if defined(__DEBUG_HTTP__)
        Serial.print("HTTP READ: ");
        Serial.print(serLen); Serial.println(" bytes");         
    #endif     
    }
}

////////////////////////////////
//     Command Processing     //
////////////////////////////////

char lastEspCMD;
char lastComCMD;

void processEspCMD() {
    lastEspCMD = readChar();
    switch (lastEspCMD) {
    case HUB_SYS_IP:
        readIP();
        displayIP();
        break;
        
    case HUB_SYS_NOTIF:
        readNotif();
        break;
        
    case HUB_SYS_ERROR:
        readError();
        break;
        
    case HUB_SYS_MOUSE:
        readMouse();
        break;

    case HUB_UDP_RECV:
        readUdp();
        break;

    case HUB_TCP_RECV:
        readTcp();
        break;            

    case HUB_WEB_RECV:
        readWeb();
        break;  

    case HUB_HTTP_GET:
        getHttp();
        break;                                    

    case HUB_HTTP_READ:
        readHttp();
        break;                                    
    }  
#ifdef __DEBUG_CMD__
    if (lastEspCMD != HUB_SYS_MOUSE) {
        Serial.print("ESP: "); 
        Serial.print(cmdString[lastEspCMD]); Serial.print(" (");
        Serial.print(serLen, DEC); Serial.println(")");
    }
#endif
}

void processComCMD() {
    unsigned char tmp;
    unsigned int offset;
    unsigned long length;   
    lastComCMD = comBuffer[0];
    switch (lastComCMD) {  
    case HUB_SYS_RESET:
        // Reset packets and files                
        packetID = 0;
        while (packetHead) popPacket(packetHead->ID);
        for (byte i=0; i<FILES; i++) {
            if (hubFile[i]) hubFile[i].close();
        }
        Serial.println("-System Reset-");
        break;

    case HUB_SYS_IP:
        // Send back IP address
        strcpy(serBuffer, IP);
        serLen = strlen(IP);
        pushPacket(HUB_SYS_IP, -1);
        break;

    case HUB_FILE_OPEN:
        // Check if file was previously opened
        if (hubFile[comBuffer[1]])
            hubFile[comBuffer[1]].close();
        
        // Open file (modes are 0:read, 1:write, 2:append)
        switch (comBuffer[2]) {
        case 0:                
            hubFile[comBuffer[1]] = SD.open(&comBuffer[3], FILE_READ); 
            break;
        case 1:
            if (SD.exists(&comBuffer[3])) { SD.remove(&comBuffer[3]); }
            hubFile[comBuffer[1]] = SD.open(&comBuffer[3], FILE_WRITE); 
            break;
        case 2:                
            hubFile[comBuffer[1]] = SD.open(&comBuffer[3], FILE_WRITE);
            break;
        }

        // Send back file size
        length = hubFile[comBuffer[1]].size();
        memcpy(serBuffer, (char*)&length, 4); serLen = 4;
        pushPacket(HUB_FILE_OPEN, comBuffer[1]);
        break;

    case HUB_FILE_SEEK:
        // Seek file position (offset from beginning)
        offset = (comBuffer[3] * 256) + comBuffer[2];
        if (hubFile[comBuffer[1]])
            hubFile[comBuffer[1]].seek(offset);
        break;

    case HUB_FILE_READ:
        if (hubFile[comBuffer[1]]) {
            // Read chunk from file
            serLen = 0;
            while (hubFile[comBuffer[1]].available() && tmp<comBuffer[2])
                serBuffer[serLen++] = hubFile[comBuffer[1]].read();
            pushPacket(HUB_FILE_READ, -1);
        }
        break;

    case HUB_FILE_WRITE:
        if (hubFile[comBuffer[1]]) 
            hubFile[comBuffer[1]].write((unsigned char*)&comBuffer[2], comLen-3);
        break;

    case HUB_FILE_CLOSE:
        if (hubFile[comBuffer[1]]) 
            hubFile[comBuffer[1]].close();
        break;                          
      
    case HUB_UDP_OPEN:
        // Forward CMD to ESP
        writeCMD(HUB_UDP_OPEN);
        writeChar(comBuffer[1]); writeChar(comBuffer[2]);
        writeChar(comBuffer[3]); writeChar(comBuffer[4]);
        writeInt(comBuffer[5]+256*comBuffer[6]); 
        writeInt(comBuffer[7]+256*comBuffer[8]);
        break;

    case HUB_UDP_SEND:
        // Forward CMD to ESP
        writeCMD(HUB_UDP_SEND);
        writeBuffer(&comBuffer[1], comLen-1);
        break;

    case HUB_UDP_CLOSE:
        // Forward CMD to ESP
        writeCMD(HUB_UDP_CLOSE);
        break;
        
    case HUB_TCP_OPEN:
        // Forward CMD to ESP
        writeCMD(HUB_TCP_OPEN);
        writeChar(comBuffer[1]); writeChar(comBuffer[2]);
        writeChar(comBuffer[3]); writeChar(comBuffer[4]);
        writeInt(comBuffer[5]+256*comBuffer[6]); 
        break;

    case HUB_TCP_SEND:
        // Forward CMD to ESP
        writeCMD(HUB_TCP_SEND);
        writeBuffer(&comBuffer[1], comLen-1);
        break;

    case HUB_TCP_CLOSE:
        // Forward CMD to ESP
        writeCMD(HUB_TCP_CLOSE);
        break;

    case HUB_WEB_OPEN:
        // Forward CMD to ESP
        writeCMD(HUB_WEB_OPEN);
        writeInt(comBuffer[1]+256*comBuffer[2]); // Port
        writeInt(comBuffer[3]+256*comBuffer[4]); // TimeOut
        break; 

    case HUB_WEB_HEADER:
        // Forward CMD to ESP
        writeCMD(HUB_WEB_HEADER);
        writeBuffer(&comBuffer[1], comLen-1);                 
        break;

    case HUB_WEB_BODY:
        // Forward CMD to ESP
        writeCMD(HUB_WEB_BODY);
        writeBuffer(&comBuffer[1], comLen-1);        
        break;

    case HUB_WEB_SEND:
        // Forward CMD to ESP
        writeCMD(HUB_WEB_SEND);
        break;
        
    case HUB_WEB_CLOSE:
        // Forward CMD to ESP
        writeCMD(HUB_WEB_CLOSE);
        break;  

    case HUB_HTTP_GET:
        // Forward CMD to ESP
        writeCMD(HUB_HTTP_GET);
        writeBuffer(&comBuffer[1], comLen-1);
        break;                                    

    case HUB_HTTP_READ:
        // Forward CMD to ESP
        writeCMD(HUB_HTTP_READ);
        writeChar(comBuffer[1]);
        break;
    }
#ifdef __DEBUG_CMD__
    Serial.print("COM: "); 
    Serial.print(cmdString[lastComCMD]); Serial.print(" (");
    Serial.print(comLen-1, DEC); Serial.println(")");
#endif
}

////////////////////////////////
//       WIFI functions       //
////////////////////////////////

char ssid[32];
char pswd[64];

void wifiVersion() {
    // Get ESP version
    writeCMD(HUB_SYS_VERSION);    
    lastEspCMD = 0;     
    uint32_t timeout = millis()+3000;
    while (lastEspCMD != HUB_SYS_VERSION) {
        if (Serial3.find("CMD")) 
            processEspCMD();
        if (millis() > timeout) {
            Serial.println("Error: ESP not responding");      
            return;
        }
    }

    // Check version was received correctly
    readBuffer();
    memcpy(espVersion, serBuffer, 4);
    if (strncmp(espVersion, "v", 1)) {
        Serial.println("Error: cannot determine current ESP version."); 
        memcpy(espVersion, "?", 1);
    }
}

unsigned char wifiScan() {
    // Scan and wait for Answer
    writeCMD(HUB_SYS_SCAN);    
    uint32_t timeout = millis()+9000;
    while (lastEspCMD != HUB_SYS_SCAN) {
        if (Serial3.find("CMD"))
            processEspCMD();
        if (millis() > timeout) {
            Serial.println("Error: ESP not responding");      
            return 0;
        }
    }                
    return 1;       
}

unsigned char askUpdate(char *core, char *version, char *update) {
    // Ask user if they want to update
    lcd.setCursor(0,1);
    lcd.print("Update available!");               
    lcd.setCursor(0,2);
    lcd.print(core); lcd.print(": "); 
    lcd.print(version); lcd.print(" -> "); lcd.print(update);
    lcd.setCursor(0,3);
    lcd.print("Press FIRE to update");       

    // Give 5s to decide...
    unsigned char doUpdate = 0;
    uint32_t timer = millis(); 
    while (millis()-timer < 5000) {
        readJOY();
        if (!(joyState[0] & joyStnd[4]) || !(joyState[0] & joyStnd[5])) {
            doUpdate = 1;
            break;
        }
    }

    // Clear LCD and return result
    lcd.setCursor(0,1); lcd.print(blank);
    lcd.setCursor(0,2); lcd.print(blank);
    lcd.setCursor(0,3); lcd.print(blank);
    return doUpdate;
}

void checkUpdate() {
    // Check if we received IP...
    uint32_t timeout = millis()+9000;    
    while (lastEspCMD != HUB_SYS_IP) {
        if (Serial3.find("CMD"))
            lastEspCMD = readChar();
        if (millis() > timeout) {
            Serial.println("Error: ESP not responding");      
            return;
        }
    }
    readIP();
    if (!strcmp(serBuffer, "Not connected..."))
        return;
    
    // Check latest version number
    writeCMD(HUB_HTTP_GET);
    writeBuffer(urlVer, strlen(urlVer)); 
    writeCMD(HUB_HTTP_READ);
    writeChar(20);

    // Wait to receive data
    httpMode = HTTP_NULL;   
    timeout = millis()+9000;
    while (lastEspCMD != HUB_HTTP_READ) {
        if (Serial3.find("CMD")) 
            processEspCMD();
        if (millis() > timeout) {
            Serial.println("Error: ESP not responding");      
            return;
        }            
    }
    memcpy(espUpdate, &serBuffer[4], 4); espUpdate[4] = 0;
    memcpy(megaUpdate, &serBuffer[16], 4); megaUpdate[4] = 0;
    httpMode = HTTP_PACKET;   

    // Check version data was received correctly
    if (strncmp(espVersion, "v", 1)) {
        Serial.println("Error: cannot determine current ESP version.");  
        lcd.setCursor(0,1); lcd.print("Update error...");
        delay(1000);
        return;
    }
    if (strncmp(espUpdate, "v", 1)) {
        Serial.println("Error: cannot determine latest ESP version.");
        lcd.setCursor(0,1); lcd.print("Update error...");
        delay(1000);
        return;
    }    
    if (strncmp(megaUpdate, "v", 1)) {
        Serial.println("Error: cannot determine latest MEGA version.");
        lcd.setCursor(0,1); lcd.print("Update error...");
        delay(1000);
        return;
    }
    
    // Display info
    Serial.print("ESP Firmware: "); Serial.print(espVersion); Serial.print(" -> "); Serial.println(espUpdate); 
    Serial.print("MEGA Firmware: "); Serial.print(megaVersion); Serial.print(" -> "); Serial.println(megaUpdate); 
    if (!strncmp(espVersion, espUpdate, 4) && !strncmp(megaVersion, megaUpdate, 4)) {
        lcd.setCursor(0,1); lcd.print("System up-to-date!");
        delay(1000);
        return;
    }
    
    // Apply ESP update?
    if (strncmp(espVersion, espUpdate, 4)) {
        // Ask user if they want to update
        if (!askUpdate("Wifi", espVersion, espUpdate))
            return;

        // Process with update
        Serial.println("Updating ESP...");
        lcd.setCursor(0,1); lcd.print("Updating Wifi...");
        writeCMD(HUB_SYS_UPDATE);
        writeBuffer(urlEsp, strlen(urlEsp));

        // Wait for ESP to Reboot   
        timeout = millis() + 60000;     
        while (lastEspCMD != HUB_SYS_VERSION) {
            delay(1000);
            writeCMD(HUB_SYS_VERSION);
            if (Serial3.find("CMD")) 
                processEspCMD();
            if (millis() > timeout) {
                Serial.println("Error: ESP not responding");      
                return;
            }                
        }
        readBuffer();

        // Check new ESP version
        if (!strncmp(serBuffer, espUpdate, 4)) {
            lcd.setCursor(0,1); lcd.print("Wifi updated!");
            Serial.println("ESP update complete");              
        } else {
            lcd.setCursor(0,1); lcd.print("Update failed!");
            Serial.println("ESP update failed");                      
        }

        // Reset Wifi, and clear screen
        setupESP();
        while (lastEspCMD != HUB_SYS_IP)
            if (Serial3.find("CMD")) processEspCMD(); 
        lcd.setCursor(0,1); lcd.print(blank);               
        lcd.setCursor(0,2); lcd.print(blank);               
        lcd.setCursor(0,3); lcd.print(blank);               
    }
    
    // Apply MEGA update?
    if (strncmp(megaVersion, megaUpdate, 4)) {
        // Ask user if they want to update
        if (!askUpdate("Core", megaVersion, megaUpdate))
            return;
        
        // Fetch update file on ESP
        lcd.setCursor(0,1); lcd.print("Updating Core...");
        Serial.println("Downloading update...");
        writeCMD(HUB_HTTP_GET);
        writeBuffer(urlMega, strlen(urlMega));

        // Wait to receive file size
        lastEspCMD = 0;            
        while (lastEspCMD != HUB_HTTP_GET)
            if (Serial3.find("CMD")) processEspCMD();

        // Check file has size
        if (!httpSize) {
            lcd.setCursor(0,1); lcd.print("Update failed...");
            Serial.println("Error: cannot download update.");
            delay(2000);
            return;
        }

        // Check there is enough storage for update
        if (!InternalStorage.open(httpSize)) {
            lcd.setCursor(0,1); lcd.print("Update failed...");
            Serial.println("Error: not enough space to store the update.");
            delay(2000);
            return;
        }

        // Get update file from ESP
        httpMode = HTTP_UPDATE;
        unsigned long recvSize = 0;
        while (1) {
            // Request next http packet
            writeCMD(HUB_HTTP_READ);
            writeChar(160);
    
            // Wait to receive packet
            lastEspCMD = 0;
            while (lastEspCMD != HUB_HTTP_READ)
                if (Serial3.find("CMD")) processEspCMD();

            // Check if packet was empty
            if (serLen)
                recvSize += serLen;
            else
                break;
        }
        httpMode = HTTP_PACKET;
        
        // Close the internal storage
        InternalStorage.close();

        // Check size of received data
        if (recvSize == httpSize) {
            Serial.print("Received: ");
            Serial.print(recvSize);
            Serial.print("/");
            Serial.print(httpSize);
            Serial.println(" bytes");
        } else {
            lcd.setCursor(0,1); lcd.print("Update failed...");
            Serial.println("Error: could not download entire file.");
            delay(2000);
            return;
        }

        lcd.setCursor(0,1); lcd.print("Rebooting...");
        Serial.println("Applying MEGA update...");
        Serial.flush();
        InternalStorage.apply();
    }
}

void setupESP() {
    // Start Mouse
    writeCMD(HUB_SYS_MOUSE);
    writeChar(100);   // Refresh period (ms)
  
    // Connect Wifi
    writeCMD(HUB_SYS_CONNECT);
    writeBuffer(ssid, strlen(ssid));
    writeBuffer(pswd, strlen(pswd));
}

//////////////////////////
//     TEST FUNCTIONS   //
//////////////////////////

const char* joyCode = "UDLRAB";

void wifiTest() {
    // Run Wifi Test
    setupESP();
    lcd.setCursor(0,0);
    lcd.print("Wifi:");
    lastEspCMD = 0;     
    while (lastEspCMD != HUB_SYS_IP)
        if (Serial3.find("CMD"))
            lastEspCMD = readChar();
    readBuffer();
    lcd.print(serBuffer);
}

void httpTest() {
    unsigned char url[] = "http://8bit-unity.com/test.txt";
    lcd.setCursor(0,1); 
    lcd.print("HTTP:");    
    Serial.print("HTTP: ");

    // Setup request
    writeCMD(HUB_HTTP_GET);
    writeBuffer(url, strlen(url)); 
    writeCMD(HUB_HTTP_READ);
    writeChar(16);

    // Wait for answer
    uint32_t timeout = millis()+3000;
    while (millis() < timeout) {
        if (Serial3.find("CMD") && readChar() == HUB_HTTP_READ) {
            if (readBuffer()) {
                Serial.println(serBuffer);
                lcd.print(serBuffer);
            }
            break;
        }
    }
    if (millis() >= timeout) {
          Serial.println("Timeout");
          lcd.print("Timeout");
    }
}

void tcpTest() {
    lcd.setCursor(0,1); 
    lcd.print("TCP:");
    Serial.print("TCP: ");
    
    // Setup request    
    writeCMD(HUB_TCP_OPEN);
    writeChar(199); writeChar(47);
    writeChar(196); writeChar(106);
    writeInt(1234);
    writeCMD(HUB_TCP_SEND);
    writeBuffer("Packet received", 16);
    
    // Wait for answer
    uint32_t timeout = millis()+3000;
    while (millis() < timeout) {
        if (Serial3.find("CMD") && readChar() == HUB_TCP_RECV) {
            readChar();
            if (readBuffer()) {
                Serial.println(serBuffer);
                lcd.print(serBuffer);
            }
            break;
        }
    }
    if (millis() >= timeout) {
          Serial.println("Timeout");
          lcd.print("Timeout");
    }
    writeCMD(HUB_TCP_CLOSE);
}

void udpTest() {
    lcd.setCursor(0,2); 
    lcd.print("UDP:");
    Serial.print("UDP: "); 

    // Setup request    
    writeCMD(HUB_UDP_SLOT);
    writeChar(0);
    writeCMD(HUB_UDP_OPEN);
    writeChar(199); writeChar(47);
    writeChar(196); writeChar(106);
    writeInt(1234); writeInt(4321);
    writeCMD(HUB_UDP_SEND);
    writeBuffer("Packet received", 16);
    
    // Wait for answer
    uint32_t timeout = millis()+3000;
    while (millis() < timeout) {
        if (Serial3.find("CMD") && readChar() == HUB_UDP_RECV) {
            readChar();
            if (readBuffer()) { 
                Serial.println(serBuffer);
                lcd.print(serBuffer);
            }
            break;
        }
    }
    if (millis() >= timeout) {
          Serial.println("Timeout");
          lcd.print("Timeout");
    }
    writeCMD(HUB_UDP_CLOSE);
}

// HTML content types
const char ctImg[] = "Content-Type: image/jpg\r\nCache-Control: max-age=999999, public";
const char ctTxt[] = "Content-Type: text/html";

// Hint: max string length is 255, but better to keep below 192 bytes!
const char htmlHead[] = "<html><center>Welcome to 8bit-Unity Web Server<br><br><img src=\"logo.jpg\" width=\"48\"><br><br><a href=\"support\">Supported</a> platforms | <a href=\"future\">Future</a> platforms";
const char htmlSup1[] = "<br><br><table style=\"border:1px solid black;text-align:center\"><tr><td>Apple //</td><td>Atari XL/XE</td><td>Commodore 64</td><td>Oric</td><td>Lynx</td></tr><tr><td><img src=\"apple.jpg\" width=\"64\">";
const char htmlSup2[] = "</td><td><img src=\"atari.jpg\" width=\"64\"></td><td><img src=\"c64.jpg\" width=\"64\"></td><td><img src=\"atmos.jpg\" width=\"64\"></td><td><img src=\"lynx.jpg\" width=\"64\"></td></tr></table>";
const char htmlFut1[] = "<br><br><table style=\"border:1px solid black;text-align:center\"><tr><td>BBC</td><td>NES</td><td>MSX</td><td>CPC</td><td>...</td></tr></table>";
const char htmlFoot[] = "<br></center></html>";

void webTest() {
    lcd.setCursor(0,2); 
    lcd.print("WEB:");
    Serial.println("WEB:");

    // Setup WEB Server
    writeCMD(HUB_WEB_OPEN);
    writeInt(80);
    writeInt(3000);

    // Wait for WEB Request
    while (1) {
        if (Serial3.find("CMD") && readChar() == HUB_WEB_RECV) {
            if (readBuffer()) {
                writeCMD(HUB_WEB_HEADER); writeBuffer((char*)ctTxt, strlen(ctTxt));
                if (!strncmp(serBuffer, "GET / ", 6)) {      
                    writeCMD(HUB_WEB_BODY); writeBuffer((char*)htmlHead, strlen(htmlHead));
                    writeCMD(HUB_WEB_BODY); writeBuffer((char*)htmlSup1, strlen(htmlSup1));
                    writeCMD(HUB_WEB_BODY); writeBuffer((char*)htmlSup2, strlen(htmlSup2));
                    writeCMD(HUB_WEB_BODY); writeBuffer((char*)htmlFoot, strlen(htmlFoot));
                }
                writeCMD(HUB_WEB_SEND); 
                
                Serial.write(serBuffer, serLen);
                Serial.print('\n');
            }
        }
    }
}

void runTests() {  
    // Network Tests
    lcd.clear();
    wifiTest();
    tcpTest();
    udpTest();
    lcd.setCursor(0,3);
    lcd.print("Network Done");
    delay(3000);
    
    // Run Joy Test
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Joy1: , , , , ,");
    lcd.setCursor(0, 1); lcd.print("Joy2: , , , , ,");
    lcd.setCursor(0, 2); lcd.print("Joy3: , , , , ,");
    lcd.setCursor(0, 3); lcd.print("Mous:   ,   , , ,");
    uint32_t timeout = millis()+30000;
    while (millis()<timeout) {
        // Check Joysticks
        readJOY();
        for (byte i=0; i<3; i++) {
           for (byte j=0; j<6; j++) {
              if (!(joyState[i] & joyStnd[j])) { 
                  lcd.setCursor(5+2*j, i);  lcd.print(joyCode[j]);
              }         
           }
        }
        // Check Mouse
        if (Serial3.find("CMD")) {
            char cmd = readChar();        
            if (cmd == HUB_SYS_MOUSE) {
                readMouse();
                lcd.setCursor(5,3);
                lcd.print(mouseState[0]);
                lcd.setCursor(9,3);
                lcd.print(mouseState[1]);
                lcd.setCursor(13,3);
                if (!(joyState[0] & 64))  lcd.print("L");
                lcd.setCursor(15,3);
                if (!(joyState[1] & 64))  lcd.print("M");
                lcd.setCursor(17,3);
                if (!(joyState[0] & 128)) lcd.print("R");
            }
        }
    }
}

//////////////////////////
//    GUI Functions     //
//////////////////////////

boolean upperCase = false;
byte keyRow = 1, keyCol = 1;
char inputCol, inputBuf[32]; 
char* lower[3] = {" 1234567890-=[]\\    ", " abcdefghijklm;'  S ",  " nopqrtsuvwxyz,./ R "};
char* upper[3] = {" !@#$%^&*()_+{}|    ",  " ABCDEFGHIJKLM:\"  S ", " NOPQRSTUVWXYZ<>? R "};

void printKeyboard() {
    for (byte i=0; i<3; i++) {
      lcd.setCursor(0,i+1); 
      if (upperCase) lcd.print(upper[i]); 
      else           lcd.print(lower[i]); 
    }
}

void keyboardInput(char* param) {
    // Special Characters
    lower[0][15] = 164;
    lower[0][18] = 127;
    upper[0][18] = 127;

    // Reset input buffer
    for (byte i=0; i<32; i++) inputBuf[i] = 0;

    // Display Parameter and Current Input
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(param);
    inputCol = strlen(param);
    lcd.setCursor(inputCol, 0);
    lcd.print(inputBuf);
    printKeyboard();

    // Wait for release
    while ((joyState[0] & 63) != 63) readJOY();      
    
    // Get Input from User
    char* letter;
    while (true) {
        // Check Joystick
        readJOY();
        if (!(joyState[0] & joyStnd[0])) { keyRow--; if (keyRow<1)  keyRow = 3; }
        if (!(joyState[0] & joyStnd[1])) { keyRow++; if (keyRow>3)  keyRow = 1; }
        if (!(joyState[0] & joyStnd[2])) { keyCol--; if (keyCol<1)  keyCol = 18; }
        if (!(joyState[0] & joyStnd[3])) { keyCol++; if (keyCol>18) keyCol = 1; }
        if (!(joyState[0] & joyStnd[4]) || !(joyState[0] & joyStnd[5])) {
            if (keyCol == 18) {
                switch (keyRow) {
                case 1:
                    if (strlen(inputBuf)) {
                        // Delete char
                        inputBuf[strlen(inputBuf)-1] = 0;
                        lcd.setCursor(inputCol+strlen(inputBuf), 0);
                        lcd.print(" ");                    
                    }
                    break;      
                case 2:
                    // Switch upper/lower
                    upperCase = !upperCase;
                    printKeyboard();
                    break;                    
                case 3:
                    // Exit keyboard
                    return;
                }               
            } else { 
                // Print character
                if (upperCase) { 
                    letter = upper[keyRow-1][keyCol];
                } else {
                    letter = lower[keyRow-1][keyCol];
                }
                inputBuf[strlen(inputBuf)] = letter;
                inputBuf[strlen(inputBuf)+1] = 0;
                lcd.setCursor(inputCol, 0);
                lcd.print(inputBuf);
            }
        }
          
        // Update Cursor
        lcd.setCursor(keyCol, keyRow);
        lcd.noCursor();
        delay(50);
        lcd.cursor();
        delay(150);        
    }
}

byte listPage = 0;
signed char listRow = 0;

unsigned char listSelection() {
    while(true) {
        // Reset screen
        lcd.clear();
    
        // Select page
        list_t* elt = listHead;
        byte i=0; 
        while (i<listPage) {
            elt = elt->next;
            i++;
        }
    
        // Show elements on current page
        for (i=0; i<4; i++) {
            if (elt) {
                lcd.setCursor(1,i);
                lcd.print(elt->data);        
                elt = elt->next;
            }
        }
    
        // Joystick control
        while (true) {
            // Blink cursor
            lcd.setCursor(0,listRow);
            lcd.print(" ");
            
            // Check Joystick
            readJOY();
            if (!(joyState[0] & joyStnd[0])) { 
                if (listRow>0) {
                    listRow--; 
                } else {
                    if (listPage>0) {
                        listPage -= 4;
                        listRow = 3;
                        break;
                    }
                }
            }
            if (!(joyState[0] & joyStnd[1])) {
                if (listPage+listRow < lenList()-1) {
                    if (listRow<3) {
                        listRow++; 
                    } else {
                        listPage += 4;
                        listRow = 0;
                        break;
                    }
                }
            }
            if (!(joyState[0] & joyStnd[4]) || !(joyState[0] & joyStnd[5])) {
                unsigned char index = listPage+listRow;
                listPage = 0;
                listRow = 0;
                return index;
            }
            
            // Blink cursor
            lcd.setCursor(0,listRow);
            lcd.print(">");
            delay(200);        
        }
    }
}

////////////////////////////////
//      CONFIG functions      //
////////////////////////////////

void readConfig() {
    // Read config from eeprom
    byte i;
    hubMode = 255-EEPROM.read(0); if (hubMode>HUB_MODES) hubMode = 0;
    for (i=0; i<32; i++) ssid[i] = 255-EEPROM.read(32+i);
    for (i=0; i<64; i++) pswd[i] = 255-EEPROM.read(64+i);

    // Override with some defaults
    if (!strlen(ssid) && !strlen(pswd)) {
        strcpy(ssid, "8bit-Unity");
        strcpy(pswd, "0123456789"); 
    }
}

void writeConfig() {
    // Read config from eeprom
    byte i;
    EEPROM.write(0, 255-hubMode);
    for (i=0; i<32; i++) EEPROM.write(32+i, 255-ssid[i]);
    for (i=0; i<64; i++) EEPROM.write(64+i, 255-pswd[i]);
}

byte menu = 1, row = 1;
boolean changes = false;  // Let's be kind on eeprom...
const char* menuHeader[3] = {"   < Hub Config >   ", "  < Wifi Config >   ", "    < Firmware >    "};
const char* menuParam[3][3] = {{"Mode:", "Test:", "Exit:"}, {"Ntwk:", "SSID:", "Pass:"}, {"Wifi:", "Core:", "Updt:"}};

void configMenu() {
    char s=0;
    lcd.setCursor(0,2);
    lcd.print("Entering Config...");
  
    while (true) {
        // Wait for release
        while ((joyState[0] & 63) != 63) readJOY();  
        
        // Show menu
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(menuHeader[menu-1]);
        for (byte param=3; param>0; param--) {
            // Show Param
            lcd.setCursor(0,param);
            lcd.print(menuParam[menu-1][param-1]);
            
            // Show Value
            switch (menu) {
            case 1:
                switch(param) {
                case 1:
                    lcd.print(modeString[hubMode]);
                    break;    
                case 2:
                    lcd.print("Run now");
                    break;    
                case 3:
                    lcd.print("Save & reboot");
                    break;    
                } break;
            case 2:
                switch(param) {
                case 1:
                    lcd.print("Scan now");
                    break;
                case 2:
                    lcd.print(ssid);
                    break;    
                case 3:
                    lcd.print(pswd);
                    break;    
                } break;            
            case 3:
                switch(param) {
                case 1:
                    lcd.print(espVersion);
                    break;
                case 2:
                    lcd.print(megaVersion);
                    break;    
                case 3:
                    lcd.print("Check now");
                    break;
                } break;            
            }
        }
        lcd.setCursor(5,row);
        lcd.cursor();

        // Wait for input
        while ((joyState[0] & 63) == 63) readJOY(); 
        lcd.noCursor();

        // Check Input
        if (!(joyState[0] & joyStnd[0])) { row--; if (row<1) row = 3; }
        if (!(joyState[0] & joyStnd[1])) { row++; if (row>3) row = 1; }
        if (!(joyState[0] & joyStnd[2])) { menu--; if (menu<1)  menu = 3; }
        if (!(joyState[0] & joyStnd[3])) { menu++; if (menu>3)  menu = 1; }
        if (!(joyState[0] & joyStnd[4]) || !(joyState[0] & joyStnd[5])) {   
            switch (menu) {
            case 1:
                switch(row) {
                case 1:
                    // Change Mode
                    delay(200);
                    for (byte i=0; i<HUB_MODES; i++) {
                        pushList(modeString[i]);
                    }
                    hubMode = listSelection();
                    clearList();
                    changes = true;             
                    break;    
                case 2:
                    // Run Tests
                    runTests();
                    break;    
                case 3:
                    // Save Config and Boot
                    if (changes) writeConfig();
                    displayHeader();
                    return;
                } break;
            case 2:         
                switch(row) {
                case 1:
                    // Scan Networks
                    wifiScan(); s=0;
                    while (serBuffer[s] != 0) {
                        while (serBuffer[s] != '\n' && serBuffer[s] != 0) s++;
                        serBuffer[s++] = 0;
                        if (serBuffer[s]) { 
                            pushList(&serBuffer[s]);
                        }
                    }
                    s = listSelection();
                    strcpy(ssid, getList(s));
                    clearList();
                    // Set PSWD
                    keyboardInput(menuParam[menu-1][2]);
                    strcpy(pswd, inputBuf);                    
                    changes = true;
                    break;
                case 2:
                    // Set SSID
                    keyboardInput(menuParam[menu-1][1]);
                    strcpy(ssid, inputBuf);
                    changes = true;
                    break;    
                case 3:
                    // Set PSWD
                    keyboardInput(menuParam[menu-1][2]);
                    strcpy(pswd, inputBuf);
                    changes = true;
                    break;    
                } break;            
            case 3:         
                switch(row) {
                case 3:
                    // Try to Update
                    lcd.setCursor(0,1); lcd.print("Please wait...");
                    lcd.setCursor(0,2); lcd.print(blank);
                    lcd.setCursor(0,3); lcd.print(blank);
                    setupESP();
                    checkUpdate();
                    break;
                } break;            
            }
        }
    }
}

////////////////////////////
//      MEGA Routines     //
////////////////////////////

void setup() {
    // Setup comm.
    setupSERIAL();
    setupLCD();
    setupJOY();
    
    // Read config from EEPROM
    readConfig();
    wifiVersion();
    
    // Chance to enter config (press fire)
    lcd.setCursor(0,1);
    lcd.print("Press FIRE for Conf.");       
    uint32_t timer = millis(); 
    while (millis()-timer < 3000) {
        readJOY();
        if (!(joyState[0] & joyStnd[4]) || !(joyState[0] & joyStnd[5])) {
            configMenu();
        }
    }
        
    // Setup peripherals
    lcd.setCursor(0,1);
    lcd.print("Booting...          ");
    setupESP();
    setupCOM();
    setupSD();

    // Check for Updates
    checkUpdate();

    // Display status info on LCD
    displayMode();
    displaySD();
    displayIP();
}

#ifdef __DEBUG_COM__
  int comCnt, comErr[5];
#endif
long comTime1, comTime2;

void loop() { 
    // Check packets time-out
    packetTimeout();

    // Check Joysticks states
    readJOY(); 
      
    // Process commands from ESP8266
    if (Serial3.find("CMD")) 
        processEspCMD();

    // Process COM I/O
    if (hubMode == MODE_LYNX) {
        // Process Lynx Communication
        comTime1 = micros();
        lynxRecvPacket();
        if (comCode == COM_ERR_OK) {
            // Send reply immediately (almost)
            delayMicroseconds(6*lynxBitPeriod);   // Bauds: 62500=6* / 41666=8* / 9600=2*
            preparePacket();
            lynxSendPacket();
        }  
        comTime2 = micros();
    }    
#ifdef __DEBUG_COM__
    // Display COM Stats
    if (comCode != COM_ERR_NODATA) {
        comErr[comCode] += 1;
        //Serial.print(comCnt++);
        //Serial.print(" Tim="); Serial.print(comTime2-comTime1);
        Serial.print(" Hea="); Serial.print(comErr[COM_ERR_HEADER]);
        Serial.print(" Tru="); Serial.print(comErr[COM_ERR_TRUNCAT]);
        Serial.print(" Cor="); Serial.print(comErr[COM_ERR_CORRUPT]);
        Serial.print(" (Rx)");
        Serial.print(" ID=C");  Serial.print(comID & 0x0f);
        Serial.print(",H"); Serial.print(comID >> 4);
        Serial.print(" Len="); Serial.print(comLen);
        Serial.print(" (Tx)");
        Serial.print(" ID=H"); Serial.print(hubID);
        Serial.print(" Len="); Serial.println(hubLen);
    }
#endif
    // Process commands from COM
    if (comCode == COM_ERR_OK) {
        if (comLen) processComCMD();
    }
    comCode = COM_ERR_NODATA;
}
