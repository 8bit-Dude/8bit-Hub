
// System libraries
#include <FS.h>
#include <ps2mouse.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>

// Firmware Version
char espVersion[] = "v0.1";

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
#define HUB_HTTP_GET     60
#define HUB_HTTP_READ    61

// ESP Params
#define SLOTS    16     // Number of tcp/udp handles
#define PACKET   256    // Max. packet length (bytes)

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
HTTPClient httpClient;
char udpSlot=0, tcpSlot=0;
uint32_t webTimer, webTimeout;
boolean webBusy = false;

// Mouse Setting
PS2Mouse mouse;
char mousePeriod = 0;
long mouseTimer = 0;

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
    if (udp[slot].parsePacket()) {
        // Read data from UDP
        serLen = udp[slot].read(serBuffer, PACKET-1);
        serBuffer[serLen] = 0;
        
        // Send it to mega        
        writeCMD(HUB_UDP_RECV);    
        Serial.write(slot); 
        writeBuffer(serBuffer, serLen);         
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
        writeCMD(HUB_TCP_RECV);    
        Serial.write(slot); 
        writeBuffer(serBuffer, serLen);                 
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
    // Check if client is currently alive
    if (webClient) {
        // Check time-out        
        if (millis()>webTimer) { 
            webClient.stop();
            webClient = webServer.available();
            if (webClient) {
                webClient.setNoDelay(true);
                webClient.keepAlive(7200, 75, 9);
                webTimer = millis()+webTimeout;   // set overall time-out
                webBusy = false;
            }
        }            
    } else {
        // Check if someone else came along...
        webClient = webServer.available();
        if (webClient) {
            webClient.setNoDelay(true);
            webClient.keepAlive(7200, 75, 9);
            webTimer = millis()+webTimeout;   // set overall time-out
            webBusy = false;
        }
    }
    
    if (webClient && !webBusy) {
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
                    writeCMD(HUB_WEB_RECV);
                    writeBuffer(webBuffer, webLen);
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
    if (webClient) {
        webClient.write("HTTP/1.1 200 OK\r\nConnection: close\r\n");
        webClient.write(serBuffer, serLen);
        webClient.write("\r\n\r\n");
    }
}

void webBody() {       
    serLen = readBuffer(serBuffer);
    if (webClient) 
        webClient.write(serBuffer, serLen);
}

void webSend() {
    if (webClient) {
        webClient.write("\r\n\r\n");
        webBusy = false;
    }
}

void webClose() {         
    webServer.stop();
    webServer = NULL;
}

////////////////////////////
//     HTTP functions     //
////////////////////////////

File httpFile; 

void httpGet() {
    // Check if http file handle still open?
    if (httpFile) httpFile.close();
  
    // Stream file to flash file system
    unsigned long httpSize;
    serLen = readBuffer(serBuffer);
    httpFile = SPIFFS.open("/http.tmp", "w");
    if (httpFile) {
        if (httpClient.begin(serBuffer)) {
            if (httpClient.GET() == HTTP_CODE_OK) {
                httpClient.writeToStream(&httpFile);
            } else {
                reply(HUB_SYS_ERROR, "HTTP: url not found!");
                return;        
            }
            httpClient.end();
        } else {
            reply(HUB_SYS_ERROR, "HTTP: cannot connect!");
            return;
        }
        httpFile.close();
    } else {
        reply(HUB_SYS_ERROR, "HTTP: flash drive error!");
        return;
    }
    
    // Prepare file for reading
    httpFile = SPIFFS.open("/http.tmp", "r");
    httpSize = httpFile.size();

    // Send back file size        
    writeCMD(HUB_HTTP_GET);    
    Serial.write(4);    
    writeLong(httpSize);       
}

void httpRead() {
    // Stream required number of bytes
    serLen = 0;
    if (httpFile) {
        // Read requested bytes, as long as there are any
        unsigned char requestLen = readChar();
        while (httpFile.available() && serLen < requestLen) {
            serBuffer[serLen++] = httpFile.read();
        }
    }
    
    // Send it to mega        
    writeCMD(HUB_HTTP_READ);    
    writeBuffer(serBuffer, serLen);   
}

////////////////////////////////
//    SERIAL communication    //
////////////////////////////////

void setupSERIAL() {  
    Serial.setRxBufferSize(256);
    Serial.begin(115200);   // MEGA comm.
    while (!Serial) ;
    Serial.setTimeout(20);
    Serial.flush();  
    Serial.readString();
}

void writeCMD(char cmd) {
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

void writeBuffer(char* buffer, char len) {
    Serial.write(len);
    Serial.write(buffer, len);
}

unsigned char readChar() {
    // Get char from serial  
    uint32_t timeout = millis()+20;    
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
    unsigned char len = readChar();
    Serial.readBytes(buffer, len);
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

void wifiConnect(char *ssid, char *pswd) {
    // Make sure WiFi is out
    WiFi.disconnect();
    
    // Connect to Network
    WiFi.begin(ssid, pswd);
       
    // Wait for Connection
    unsigned long timeout = millis()+9000;
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() > timeout) {
          reply(HUB_SYS_IP, "Not connected..."); 
          return;
        }
        delay(10);
    }  

    // Send back IP address
    char ip[17];
    WiFi.localIP().toString().toCharArray(ip, 16);
    reply(HUB_SYS_IP, ip);
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
        wifiConnect(ssid, pswd);
        break;

      case HUB_SYS_IP:
        wifiIP();
        break;
        
      case HUB_SYS_MOUSE:
        if (!mouse.init()) {
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

      case HUB_HTTP_GET:
        httpGet();
        break;

      case HUB_HTTP_READ:
        httpRead();
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
}

void loop(void) {
    // Auto-reconnect?
    if (WiFi.status() != WL_CONNECTED)
        if (ssid[0]) wifiConnect(ssid, pswd);
    
    // Process commands from MEGA
    if (Serial.find("CMD"))
        processCMD();
        
    // Check UDP/TCP slots and Server
    for (byte slot=0; slot<SLOTS; slot++) udpReceive(slot);
    for (byte slot=0; slot<SLOTS; slot++) tcpReceive(slot);
    webReceive();
    
    // Read Mouse State
    if (mousePeriod && (millis()-mouseTimer > mousePeriod) && mouse.update()) {
        mouseTimer = millis();
        writeCMD(HUB_SYS_MOUSE);
        Serial.write(mouse.status);
        Serial.write(mouse.x);
        Serial.write(mouse.y);
    }
}
