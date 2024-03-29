  
// System libraries
#include <FS.h>
#include <ps2mouse.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>

// Debugging
//#define __DEBUG_WIFI__

// Firmware Version
char espVersion[] = "v0.6";

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
#define HUB_SYS_RESEND    9
#define HUB_DIR_LS       10
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
#define HUB_URL_GET      60
#define HUB_URL_READ     61

// ESP Params
#define SLOTS    8     // Number of tcp/udp handles
#define PACKET   256   // Max. packet length (bytes)
#define TIMEOUT  10000 // Wifi connection timeout (msec)

// Wifi connection params
char ssid[32], pswd[64];

// Buffers for data exchange
char serBuffer[PACKET], serLen;

// ESP8266 settings
IPAddress tcpIp[SLOTS];
IPAddress udpIp[SLOTS];
unsigned int tcpPort[SLOTS];
unsigned int udpPort[SLOTS];
unsigned int webPort;
WiFiClient tcp[SLOTS];
WiFiUDP    udp[SLOTS];
WiFiServer webServer(80); 
WiFiClient webClient;
HTTPClient urlClient;
char udpSlot=0, tcpSlot=0;
uint32_t webTimer, webTimeout;
boolean webBusy = false;

// UDP/TCP socket management
uint32_t socketTimer = 0;
unsigned char socketPeriod = 10;
unsigned char socketTimeout = 10;

// Mouse Setting
PS2Mouse mouse(0, 2);
unsigned char mousePeriod = 0;
long mouseTimer = 0;

////////////////////////////////
//      PACKET functions      //
////////////////////////////////

// Define packet structure
typedef struct packet {
    unsigned char cmd;
    signed char slot;
    unsigned char len;
    char buffer[PACKET];
} packet_t;

packet_t lastPacket;

////////////////////////////
//      UDP functions     //
////////////////////////////

void udpOpen(unsigned char slot) {
    // Get data from Mega
    unsigned char ip1, ip2, ip3, ip4;
    unsigned int listenPort;
    ip1 = readChar(); ip2 = readChar();
    ip3 = readChar(); ip4 = readChar();
    udpIp[slot] = IPAddress(ip1, ip2, ip3, ip4);
    readInt(&udpPort[slot]);
    readInt(&listenPort);  

    // Open UDP port
    if (!udp[slot].begin(listenPort))
        reply(HUB_SYS_ERROR, "UDP connection failed");  
}

void udpReceive(unsigned char slot) {
    // Check for incoming packet
    int len = udp[slot].parsePacket();
    if (len) {
        // Wait for all data to arrive (within timeout period)
        uint32_t timeOut = millis()+socketTimeout;
        while (true) {
            if (udp[slot].available() == len) break;
            if (millis() > timeOut) return;
        }

        // Read UDP packet
        len = udp[slot].read(serBuffer, PACKET-1);
        if (len) {
            // Send to mega        
            serLen = len; serBuffer[serLen] = 0;
            writePacket(HUB_UDP_RECV, slot, serBuffer, serLen);
        }
    }
}

void udpSend(unsigned char slot) {
    serLen = readBuffer(serBuffer);
    udp[slot].beginPacket(udpIp[slot], udpPort[slot]);
    udp[slot].write(serBuffer, serLen);
    udp[slot].endPacket();
}

void udpClose(unsigned char slot) {
    udp[slot].stop();
}

////////////////////////////
//      TCP functions     //
////////////////////////////

boolean tcpConnect(unsigned char slot) {
    // Open TCP connection
    if (tcp[slot].connect(tcpIp[slot], tcpPort[slot])) {
        tcp[slot].setNoDelay(true);
        tcp[slot].keepAlive(7200, 75, 9);
        return true;
    } else {
        return false;
    }
}

void tcpOpen(unsigned char slot) {            
    // Get data from Mega
    unsigned char ip1, ip2, ip3, ip4;
    ip1 = readChar(); ip2 = readChar();
    ip3 = readChar(); ip4 = readChar();
    tcpIp[slot] = IPAddress(ip1, ip2, ip3, ip4);
    readInt(&tcpPort[slot]); 
    if (!tcpConnect(slot)) 
        reply(HUB_SYS_ERROR, "TCP connection failed");
}

void tcpReceive(unsigned char slot) {
    if (tcp[slot].available()) {
        // Read data from TCP
        serLen = tcp[slot].read((unsigned char*)serBuffer, PACKET-1);
        serBuffer[serLen] = 0;
        
        // Send it to mega        
        writePacket(HUB_TCP_RECV, slot, serBuffer, serLen);                
    }
}

void tcpSend(unsigned char slot) {
    // Get data and try to send over TCP
    serLen = readBuffer(serBuffer);
    if (!tcp[slot].connected()) 
        if (!tcpConnect(slot))
            return;
    tcp[slot].write(serBuffer, serLen);
}

void tcpClose(unsigned char slot) {
    if (tcp[slot].connected())
        tcp[slot].stop(); 
}

////////////////////////////
//      WEB functions     //
////////////////////////////

void webOpen() {            
    webBusy = false;
    readInt(&webPort); 
    readInt(&webTimeout);
    webServer.begin(webPort);
}

void webReceive() {
    // Check if current client has timed-out
    if (webClient.connected() && millis()>webTimer)
            webClient.stop();

    // Check if someone new else came along...
    if (!webClient.connected()) {
        webClient = webServer.available();
        if (webClient.connected()) {
            webClient.setNoDelay(true);
            webClient.keepAlive(7200, 75, 9);
            webTimer = millis()+webTimeout;   // set overall time-out
            webBusy = false;
        }
    }
    
    if (webClient.connected() && !webBusy) {
        // Setup request reader
        unsigned char webLen = 0;
        char webBuffer[256];

        // Parse request
        while (webClient.available() && millis()<webTimer) {
            // Retrieve incoming message byte-by-byte
            char c = webClient.read();  // read a byte, then
            if (c == '\n') {            // check if it is a newline character...
                // Did we find the GET ... line?
                if (!strncmp(webBuffer, "GET", 3)) {
                    writePacket(HUB_WEB_RECV, -1, webBuffer, webLen);                
                    webBusy = true;
                    return;                   
                }
                webBuffer[0] = 0;
                webLen = 0;          
            } 
            else if (c != '\r') {  // if you got anything else but a carriage return character,
                webBuffer[webLen++] = c;      // add it to the end of the currentLine
            }
            yield();             
        }
    }
}

void webHeader() {
    serLen = readBuffer(serBuffer);
    if (webClient.connected()) {
        webClient.write("HTTP/1.1 200 OK\r\nConnection: close\r\n");
        webClient.write(serBuffer, serLen);
        webClient.write("\r\n\r\n");
    }
}

void webBody() {       
    serLen = readBuffer(serBuffer);
    if (webClient.connected()) 
        webClient.write(serBuffer, serLen);
}

void webSend() {
    if (webClient.connected()) {
        webClient.write("\r\n\r\n");
        webBusy = false;
    }
}

void webClose() {         
    webServer.stop();
}

///////////////////////////
//     URL functions     //
///////////////////////////

File urlFile; 

void urlGet() {
    // Check if url file handle still open?
    if (urlFile) urlFile.close();
  
    // Stream file to flash file system
    unsigned long urlSize;
    serLen = readBuffer(serBuffer);
    urlFile = SPIFFS.open("/url.tmp", "w");
    if (urlFile) {
        if (urlClient.begin(serBuffer)) {
            if (urlClient.GET() == HTTP_CODE_OK) {
                urlClient.writeToStream(&urlFile);
            } else {
                reply(HUB_SYS_ERROR, "URL: not found!");
                return;        
            }
            urlClient.end();
        } else {
            reply(HUB_SYS_ERROR, "URL: cannot connect!");
            return;
        }
        urlFile.close();
    } else {
        reply(HUB_SYS_ERROR, "URL: flash drive error!");
        return;
    }
    
    // Prepare file for reading
    urlFile = SPIFFS.open("/url.tmp", "r");
    urlSize = urlFile.size();

    // Send back file size        
    writeCMD(HUB_URL_GET);    
    Serial.write(4);    
    writeLong(urlSize);       
}

void urlRead() {
    // Stream required number of bytes
    serLen = 0;
    if (urlFile) {
        // Read requested bytes, as long as there are any
        unsigned char requestLen = readChar();
        while (urlFile.available() && serLen < requestLen) {
            serBuffer[serLen++] = urlFile.read();
        }
    }
    
    // Send it to mega        
    writePacket(HUB_URL_READ, -1, serBuffer, serLen);
}

////////////////////////////////
//    SERIAL communication    //
////////////////////////////////

void setupSERIAL() {  
    Serial.setRxBufferSize(256);
    Serial.begin(115200);   // MEGA comm.
    while (!Serial) ;
    Serial.setTimeout(10);
    Serial.flush();  
    Serial.readString();
}

void writeCMD(unsigned char cmd) {
    Serial.print("CMD");
    Serial.write(cmd);
}

void writeChar(unsigned char input) {
    Serial.write(input);
}

void writeInt(unsigned int input) {
    Serial.write((char*)&input, 2);
}

void writeLong(unsigned long input) {
    Serial.write((char*)&input, 4);
}

void writeBuffer(char* buffer, unsigned char len) {
    Serial.write(len);
    Serial.write(buffer, len);
}

void writePacket(unsigned char cmd, signed char slot, char* buffer, unsigned char len) {
    // Make a copy (in case resend is requested)
    lastPacket.cmd = cmd;
    lastPacket.slot = slot;
    lastPacket.len = len;
    memcpy(lastPacket.buffer, buffer, len);

    // Write to Serial
    writeCMD(cmd);    
    if (slot >= 0)
        Serial.write(slot); 
    writeBuffer(buffer, len);
}

void resendPacket() {
    // Write to Serial
    writeCMD(lastPacket.cmd);    
    if (lastPacket.slot >= 0)
        Serial.write(lastPacket.slot); 
    writeBuffer(lastPacket.buffer, lastPacket.len);
}

unsigned char readChar() {
    // Get char from serial  
    uint32_t timeout = millis()+10;    
    while (!Serial.available()) {
        if (millis() > timeout)
          return 0;
    }    
    return Serial.read();
}

void readInt(unsigned int *buffer) {
    // Get one int from serial
    Serial.readBytes((char*)buffer, 2);  
}

unsigned char readBuffer(char *buffer) {
    // Read buffer of known length
    uint32_t timeout;
    unsigned char i = 0;
    unsigned char len = readChar();
    while (i<len) {
        timeout = millis()+10;    
        while (!Serial.available()) {
            if (millis() > timeout)
                return 0;
        }    
        buffer[i++] = Serial.read();
    }
    buffer[len] = 0;
    return len;
}

void reply(unsigned char type, char *message) {
    writeCMD(type);    
    writeBuffer(message, strlen(message));  
}

////////////////////////////////
//        ESP functions       //
////////////////////////////////

boolean connected = false;
char ssidLog[32] = "", pswdLog[64] = "";
unsigned long timeout = 0;

void wifiCheck(char *ssid, char *pswd) {
    // Did SSID/PSWD change?
    if (strcmp(ssid, ssidLog) || strcmp(pswd, pswdLog)) {
        if (WiFi.status() == WL_CONNECTED)
            WiFi.disconnect();  // Cut-out current connection
        strcpy(ssidLog, ssid);  // Update credentials
        strcpy(pswdLog, pswd);
        if (!ssid[0]) return;   // Create new connection?
    #if defined(__DEBUG_WIFI__)
        Serial.println("Updated credentials");            
    #endif
        timeout = millis()+TIMEOUT;
        WiFi.begin(ssid, pswd);  
    }

    // Do we have an SSID?
    if (!ssid[0]) return;

    // Check current status
    if (WiFi.status() != WL_CONNECTED) {
        connected = false;
        if (millis() > timeout) {
            // Update Time-out        
            reply(HUB_SYS_IP, "Not connected..."); 
            timeout = millis()+TIMEOUT;
        }
    } else {
        if (!connected) {
            // Send back IP address
            char ip[17];
            WiFi.localIP().toString().toCharArray(ip, 16);
            reply(HUB_SYS_IP, ip);
            connected = true;
        }
    }
}

void wifiIP() {
    // Send back system IP
    if (WiFi.status() != WL_CONNECTED) {
        reply(HUB_SYS_IP, "Not connected..."); 
    } else {
        // Send back IP address
        char ip[17];
        WiFi.localIP().toString().toCharArray(ip, 16);
        reply(HUB_SYS_IP, ip);
    }
}

void wifiScan() {
    unsigned char len = WiFi.scanNetworks();
    strcpy(serBuffer, "SSID List:\n");
    for (byte i=0; i<len; i++) {
        strcat(serBuffer, WiFi.SSID(i).c_str());
        strcat(serBuffer, "\n");
    }
    reply(HUB_SYS_SCAN, serBuffer);  
}

void updateESP() {
    // Attempt to download update and flash ESP
    ESPhttpUpdate.rebootOnUpdate(true);
    serLen = readBuffer(serBuffer); serBuffer[serLen] = 0;
    t_httpUpdate_return ret = ESPhttpUpdate.update(serBuffer);
}

////////////////////////////////
//     Command Processing     //
////////////////////////////////

char lastCMD;

void processCMD() {
    lastCMD = readChar();
    switch (lastCMD) {

      case HUB_SYS_RESET:
        ESP.reset();
      
      case HUB_SYS_VERSION:
        writeCMD(HUB_SYS_VERSION);
        writeBuffer(espVersion, strlen(espVersion));
        break;
      
      case HUB_SYS_CONNECT:
        readBuffer(ssid);
        readBuffer(pswd);
        socketPeriod = readChar();
        break;

      case HUB_SYS_IP:
        wifiIP();
        break;
        
      case HUB_SYS_MOUSE:
        if (!mouse.initialize()) {
            reply(HUB_SYS_NOTIF, "Mouse not connected");
            mousePeriod = 0;
            readChar();
        } else {
            reply(HUB_SYS_NOTIF, "Mouse connected");
            mousePeriod = readChar();
        }
        break;

      case HUB_SYS_SCAN:
        wifiScan();
        break;

      case HUB_SYS_UPDATE:
        updateESP();          
        break;

      case HUB_SYS_RESEND:
        resendPacket();
        break;

      case HUB_UDP_SLOT:
        udpSlot = readChar();
        break;
        
      case HUB_UDP_OPEN:
        udpOpen(udpSlot);
        break;

      case HUB_UDP_SEND:                        
        udpSend(udpSlot);
        break;

      case HUB_UDP_CLOSE:
        udpClose(udpSlot);
        break;

      case HUB_TCP_SLOT:
        tcpSlot = readChar();
        break;
        
      case HUB_TCP_OPEN:
        tcpOpen(tcpSlot);
        break;    

      case HUB_TCP_SEND:    
        tcpSend(tcpSlot);
        break;

      case HUB_TCP_CLOSE:
        tcpClose(tcpSlot);
        break;

      case HUB_WEB_OPEN:
        webOpen();
        break;  

      case HUB_WEB_HEADER:
        webHeader();
        break;  
                    
      case HUB_WEB_BODY:
        webBody();
        break;  
                    
      case HUB_WEB_SEND:
        webSend();
        break;  
                    
      case HUB_WEB_CLOSE:
        webClose();
        break;  

      case HUB_URL_GET:
        urlGet();
        break;

      case HUB_URL_READ:
        urlRead();
        break;

      default:
        reply(HUB_SYS_ERROR, "ESP received unknown CMD");
    } 
}

////////////////////////////////
//       ESP8266 Routines     //
////////////////////////////////

void setup(void) {
    setupSERIAL();
    SPIFFS.begin();  // Mount Flash File System (for temporary storage of files)
#if defined(__DEBUG_WIFI__)
    Serial.println("\nConnecting to Wifi...");
    strcpy(ssid, "8bit-Unity");
    strcpy(pswd, "0123456789");
#endif
}

void loop(void) {
    // Auto-reconnect (in case connection is lost)
    wifiCheck(ssid, pswd);
    
    // Process commands from MEGA
    if (Serial.find("CMD"))
        processCMD();
        
    // Check UDP/TCP slots and Server
    if (millis() >= socketTimer) {
        socketTimer = millis() + socketPeriod;
        for (byte slot=0; slot<SLOTS; slot++) udpReceive(slot);
        for (byte slot=0; slot<SLOTS; slot++) tcpReceive(slot);
        webReceive();
    }
    
    // Read Mouse State
    if (mousePeriod && (millis()-mouseTimer > mousePeriod) && mouse.update()) {
        writeCMD(HUB_SYS_MOUSE);
        Serial.write(mouse.state.info);
        Serial.write(mouse.state.x);
        Serial.write(mouse.state.y);
        Serial.write(mouse.state.wheel);
        mouseTimer = millis();
    }
}
