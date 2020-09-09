
// System libraries
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ps2mouse.h>

// Firmware Version
char espVersion[] = "v0.1";

// HUB Commands
#define HUB_SYS_RESET     1
#define HUB_DIR_LS       10
#define HUB_DIR_MK       11
#define HUB_DIR_RM       12
#define HUB_DIR_CD       13
#define HUB_FIL_OPEN     21
#define HUB_FIL_SEEK     22
#define HUB_FIL_READ     23
#define HUB_FIL_WRITE    24
#define HUB_FIL_CLOSE    25
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
#define HUB_WEB_FOOTER   54
#define HUB_WEB_CLOSE    55
#define HUB_ESP_CONNECT 100
#define HUB_ESP_NOTIF   101
#define HUB_ESP_ERROR   112
#define HUB_ESP_IP      103
#define HUB_ESP_MOUSE   104
#define HUB_ESP_SCAN    105
#define HUB_ESP_UPDATE  106
#define HUB_ESP_VERSION 107

// ESP Params
#define SLOTS    16     // Number of connection handles
#define PACKET   1024   // Max. packet length (bytes)

// Wifi connection params
char ssid[32], pswd[64];

// Buffers for data exchange
char megaBuffer[PACKET], megaLen;

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
char udpSlot=0, tcpSlot=0;
uint32_t webTimer, webTimeout;

// Updater Related globals
HTTPClient httpClient;
WiFiClient updateClient;

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
    if (udp[slot].begin(listenPort)) {
        reply(HUB_ESP_NOTIF, "UDP Port Opened");
    } else {
        reply(HUB_ESP_ERROR, "UDP Port Error");  
    }
}

void udpReceive(unsigned char slot) {
    if (udp[slot].parsePacket()) {
        // Read data from UDP
        megaLen = udp[slot].read(megaBuffer, PACKET-1);
        megaBuffer[megaLen] = 0;
        // Send it to mega        
        Serial.print("CMD");
        Serial.write(HUB_UDP_RECV);    
        Serial.write(slot); 
        writeBuffer(megaBuffer, megaLen);         
    }
}

void udpSend(unsigned char slot) {
    megaLen = readBuffer(megaBuffer);
    udp[slot].beginPacket(udpIp[slot], udpPort[slot]);
    udp[slot].write(megaBuffer, megaLen);
    udp[slot].endPacket();
}

void udpClose(unsigned char slot) {
    udp[slot].stop();
    reply(HUB_ESP_NOTIF, "UDP Port Closed");
}

////////////////////////////
//      TCP functions     //
////////////////////////////

void tcpOpen(unsigned char slot) {            
    // Get data from Mega
    unsigned char ip1, ip2, ip3, ip4;
    ip1 = readChar(); ip2 = readChar();
    ip3 = readChar(); ip4 = readChar();
    tcpIp[slot] = IPAddress(ip1, ip2, ip3, ip4);
    readInt(&tcpPort[slot]); 

    // Open TCP connection
    if (tcp[slot].connect(tcpIp[slot], tcpPort[slot])) {
        reply(HUB_ESP_NOTIF, "TCP Connection Opened");
    } else {
        reply(HUB_ESP_ERROR, "TCP Connection Failed");
    }
}

void tcpReceive(unsigned char slot) {
    if (tcp[slot].available()) {
       // Read data from TCP
        megaLen = tcp[slot].read((unsigned char*)megaBuffer, PACKET-1);
        megaBuffer[megaLen] = 0;
        // Send it to mega        
        Serial.print("CMD");
        Serial.write(HUB_TCP_RECV);    
        Serial.write(slot); 
        writeBuffer(megaBuffer, megaLen);                 
    }
}

void tcpSend(unsigned char slot) {        
    megaLen = readBuffer(megaBuffer);
    tcp[slot].write(megaBuffer, megaLen);
}

void tcpClose(unsigned char slot) {
    tcp[slot].stop(); 
    reply(HUB_ESP_NOTIF, "TCP Connection Closed");
}

////////////////////////////
//      WEB functions     //
////////////////////////////

void webOpen() {            
    readInt(&webPort); 
    readInt(&webTimeout);
    webServer.begin(webPort);
    reply(HUB_ESP_NOTIF, "Server Started");
}

void webReceive() {
    // Check if client is currently alive
    if (webClient) {
        if (millis()<webTimer) {
            return;
        } else {
            webClient.stop();
        }
    }

    // Check if someone else came along...
    webClient = webServer.available();
    if (webClient) {
        // Setup request reader
        char currentLine[256];
        unsigned char currentLen = 0;
        megaLen = 0;

        // Parse request
        webTimer = millis()+webTimeout;          // set overall time-out
        while (webClient.connected() && millis() < webTimer) {
            if (webClient.available()) {  
                // Retrieve incoming message byte-by-byte
                char c = webClient.read();             // read a byte, then
                if (c == '\n') {                 // check if it is a newline character...
                    // if the current line is blank, you got two newline characters in a row.
                    // that's the end of the client HTTP request, so forward request
                    if (megaLen) {
                        // Send data to mega        
                        Serial.print("CMD");
                        Serial.write(HUB_WEB_RECV);
                        writeBuffer(megaBuffer, megaLen);
                        return;                       // return to main loop
                    } else {
                        // only keep the HTTP command
                        if (!strncmp(currentLine, "GET", 3)) {
                            megaLen = currentLen;
                            for (byte i=0; i<currentLen; i++) { 
                                megaBuffer[i] = currentLine[i];
                            }
                        }
                        currentLen = 0;                           
                    }
                } else if (c != '\r') {  // if you got anything else but a carriage return character,
                    currentLine[currentLen++] = c;      // add it to the end of the currentLine
                }             
            }
        }

        // Could not process request within time-out...
        webClient.stop();
    }
}

void webHeader() {
    if (webClient) webClient.write("HTTP/1.1 200 OK\r\nConnection: close\r\n\r\n");
}

void webBody() {       
    megaLen = readBuffer(megaBuffer);
    if (webClient) webClient.write(megaBuffer, megaLen);
}

void webFooter() {
    if (webClient) {
        webClient.write("\r\n\r\n");
        webClient.stop();
    }
}

void webClose() {         
    webServer.stop();
    reply(HUB_ESP_NOTIF, "Server Stopped");
}

////////////////////////////////
//      MEGA communication    //
////////////////////////////////

void writeChar(unsigned char input) {
    Serial.write(input);
}

void writeInt(unsigned int input) {
    Serial.write((char*)&input, 2);
}

void writeBuffer(char* buffer, char len) {
    Serial.write(len);
    Serial.write(buffer, len);
}

unsigned char readChar() {
    // Get char from serial  
    while (!Serial.available()) {}
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
    Serial.print("CMD");
    Serial.write(type);    
    writeBuffer(message, strlen(message));  
}

////////////////////////////////
//        Wifi functions      //
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
          reply(HUB_ESP_IP, "Not connected...");
          return;
        }
        delay(10);
    }
  
    // Send back IP address
    char ip[17];
    WiFi.localIP().toString().toCharArray(ip, 16);
    reply(HUB_ESP_IP, ip);
}

void wifiScan() {
    unsigned char len = WiFi.scanNetworks();
    strcpy(megaBuffer, "SSID List:\n");
    for (byte i=0; i<len; i++) {
        strcat(megaBuffer, WiFi.SSID(i).c_str());
        strcat(megaBuffer, "\n");
    }
    reply(HUB_ESP_NOTIF, megaBuffer);  
}

void updateESP() {
    // Fetch version number from update server
    char svrVersion[16] = "";
    if (httpClient.begin(updateClient, "http://8bit-unity.com/Hub-ESP8266.ver")) {
        int httpCode = httpClient.GET();
        if (httpCode == HTTP_CODE_OK) {
            int len = httpClient.getSize();
             WiFiClient *stream = &updateClient;
             stream->readBytes(svrVersion, std::min((size_t)len, sizeof(svrVersion)));
        }
        httpClient.end();        
    }

    // Check if we got a version number
    if (!svrVersion[0]) {
        reply(HUB_ESP_NOTIF, "Server not responding");
        return;
    }

    // Check if latest version is newer?
    char message[64] = "";
    if (!strncmp(espVersion, svrVersion, 4)) {
        reply(HUB_ESP_NOTIF, "Already up-to-date");
        return;
    } else {
        sprintf(message, "Downloading %s...", svrVersion);
        reply(HUB_ESP_NOTIF, message);
    }

    // Attempt to download update and flash ESP
    ESPhttpUpdate.rebootOnUpdate(false);
    t_httpUpdate_return ret = ESPhttpUpdate.update("http://8bit-unity.com/Hub-ESP8266.bin");
    switch(ret) {
    case HTTP_UPDATE_FAILED:
        sprintf(message, "Update failed (%d)", ESPhttpUpdate.getLastError());
        reply(HUB_ESP_NOTIF, message);
        break;                
    case HTTP_UPDATE_NO_UPDATES:
        reply(HUB_ESP_NOTIF, "No update available");
        break;                
    case HTTP_UPDATE_OK:
        reply(HUB_ESP_NOTIF, "Update complete");
        break;  
   default:
        reply(HUB_ESP_NOTIF, "Update error");
        break;
    }    
}

////////////////////////////////
//       ESP8266 Routines     //
////////////////////////////////

void setup(void) {
    // Setup serial port
    Serial.begin(115200);
    while (!Serial) ;
    Serial.setTimeout(10);
    Serial.flush();  
    Serial.readString();
}

void loop(void) {
    // Auto-reconnect?
    if (WiFi.status() != WL_CONNECTED) {
        if (ssid[0]) wifiConnect(ssid, pswd);
    }
    
    // Check commands from MEGA
    char cmd;
    if (Serial.find("CMD")) {
        cmd = readChar();
        switch (cmd) {
          
          case HUB_ESP_VERSION:
            reply(HUB_ESP_NOTIF, espVersion); 
            break;
          
          case HUB_ESP_CONNECT:
            readBuffer(ssid);
            readBuffer(pswd);
            wifiConnect(ssid, pswd);
            break;

          case HUB_ESP_MOUSE:
            if (!mouse.init()) {
                reply(HUB_ESP_NOTIF, "Mouse not connected");
                mousePeriod = 0;
                readChar();
            } else {
                reply(HUB_ESP_NOTIF, "Mouse connected");
                mousePeriod = readChar();
            }
            break;

          case HUB_ESP_SCAN:
            wifiScan();
            break;

          case HUB_ESP_UPDATE:
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
                        
          case HUB_WEB_FOOTER:
            webFooter();
            break;  
                        
          case HUB_WEB_CLOSE:
            webClose();
            break;  
            
          default:
            reply(HUB_ESP_ERROR, "CMD unknown");
        }
    }

    // Check UDP/TCP slots and Server
    for (byte slot=0; slot<SLOTS; slot++) udpReceive(slot);
    for (byte slot=0; slot<SLOTS; slot++) tcpReceive(slot);
    webReceive();
    
    // Read Mouse State
    if (mousePeriod && (millis()-mouseTimer > mousePeriod) && mouse.update()) {
        mouseTimer = millis();
        Serial.print("CMD");
        Serial.write(HUB_ESP_MOUSE);
        Serial.write(mouse.status);
        Serial.write(mouse.x);
        Serial.write(mouse.y);
    }
}
