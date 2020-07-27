
// System libraries
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ps2mouse.h>

// Debugging
//#define __DEBUG_MOUSE__

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
#define HUB_SRV_OPEN     50
#define HUB_SRV_RECV     51
#define HUB_SRV_SEND     52
#define HUB_SRV_CLOSE    53
#define HUB_ESP_CONNECT 100
#define HUB_ESP_NOTIF   101
#define HUB_ESP_ERROR   112
#define HUB_ESP_IP      103
#define HUB_ESP_MOUSE   104
#define HUB_ESP_SCAN    105

// ESP8266 Settings
#define SLOTS 16
IPAddress tcpIp[SLOTS];
IPAddress udpIp[SLOTS];
unsigned int srvPort;
unsigned int tcpPort[SLOTS];
unsigned int udpPort[SLOTS];
WiFiServer srv(80); WiFiClient cli; // Server/Client pair
WiFiClient tcp[SLOTS];
WiFiUDP    udp[SLOTS];
char udpSlot=0, tcpSlot=0;

// Buffer for Data Exchange
char sendBuffer[256], sendLen;
char recvBuffer[256], recvLen;

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

unsigned int udpReceive(unsigned char slot) {
    if (udp[slot].parsePacket()) {
        recvLen = udp[slot].read(recvBuffer, 255);
        recvBuffer[recvLen] = 0;
        return recvLen;
    } else {
        return 0;
    }
}

void udpSend(unsigned char slot) {
    sendLen = readBuffer(sendBuffer);
    udp[slot].beginPacket(udpIp[slot], udpPort[slot]);
    udp[slot].write(sendBuffer, sendLen);
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

unsigned int tcpReceive(unsigned char slot) {
    if (tcp[slot].available()) {
        recvLen = tcp[slot].read((unsigned char*)recvBuffer, 255);
        recvBuffer[recvLen] = 0;
        return recvLen;
    } else {
        return 0;
    }
}

void tcpSend(unsigned char slot) {        
    sendLen = readBuffer(sendBuffer);
    tcp[slot].write(sendBuffer, sendLen);
}

void tcpClose(unsigned char slot) {
    tcp[slot].stop(); 
    reply(HUB_ESP_NOTIF, "TCP Connection Closed");
}

////////////////////////////
//      SRV functions     //
////////////////////////////

void srvOpen() {            
    readInt(&srvPort); 
    srv.begin(srvPort);
    reply(HUB_ESP_NOTIF, "Server Started");
}

unsigned int srvReceive() {
    // Only receive once current client has been handled
    if (cli.available()) return 0;

    // Check if someone else came along...
    cli = srv.available();
    if (cli.available()) {
        recvLen = cli.read((unsigned char*)recvBuffer, 255);
        recvBuffer[recvLen] = 0;
        return recvLen;             
    } else {
        return 0;
    }
}

void srvSend() {        
    sendLen = readBuffer(sendBuffer);
    cli.write(sendBuffer, sendLen);
    cli.stop();
}

void srvClose() {            
    srv.stop();
    reply(HUB_ESP_NOTIF, "Server Stopped");
}

////////////////////////////////
//      MEGA communication    //
////////////////////////////////

void writeBuffer(char* buffer, char len) {
    // Write buffer of known length
    Serial.write(len);
    Serial.write(buffer, len);
}

void writeInt(unsigned int input) {
    // Write int
    Serial.write((char*)&input, 2);
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

char ssid[32], pswd[64];

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
            // Print list of visible networks
            unsigned char len;
            char ssidList[1024];
            strcpy(ssidList, "SSID List:\n");
            len = WiFi.scanNetworks();
            for (byte i=0; i<len; i++) {
                strcat(ssidList, WiFi.SSID(i).c_str());
                strcat(ssidList, "\n");
            }
            reply(HUB_ESP_NOTIF, ssidList);
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

          case HUB_SRV_OPEN:
            srvOpen();
            break;  

          case HUB_SRV_SEND:
            srvSend();
            break;  
                        
          case HUB_SRV_CLOSE:
            srvClose();
            break;  
            
          default:
            reply(HUB_ESP_ERROR, "CMD unknown");
        }
    }

    // Check UDP slots
    for (byte slot=0; slot<SLOTS; slot++) {
        if (udpReceive(slot)) {
            Serial.print("CMD");
            Serial.write(HUB_UDP_RECV);    
            writeBuffer(recvBuffer, recvLen);  
        }
    }
    
    // Check TCP slots
    for (byte slot=0; slot<SLOTS; slot++) {
        if (tcpReceive(slot)) {
            Serial.print("CMD");
            Serial.write(HUB_TCP_RECV);    
            writeBuffer(recvBuffer, recvLen);  
        }    
    }

    // Check Server connections
    if (srvReceive()) {
        Serial.print("CMD");
        Serial.write(HUB_SRV_RECV);    
        writeBuffer(recvBuffer, recvLen);  
    }    
    
    // Read Mouse State
    if (mousePeriod && (millis()-mouseTimer > mousePeriod) && mouse.update()) {
        mouseTimer = millis();
        Serial.print("CMD");
        Serial.write(HUB_ESP_MOUSE);
        Serial.write(mouse.status);
        Serial.write(mouse.x);
        Serial.write(mouse.y);
#ifdef __DEBUG_MOUSE__
        Serial.print("Mouse: "); Serial.print((mouseB&2)>0);   // L but
        Serial.print(","); Serial.print((mouseB&8)>0);   // M but
        Serial.print(","); Serial.print((mouseB&4)>0);   // R but        
        Serial.print(","); Serial.print((signed byte)mouse.x);
        Serial.print(","); Serial.println((signed byte)mouse.y);
#endif        
    }
}

/*
void getBoardInfo() {
  Serial.println("ESP8266 board info:");
  Serial.print("\tChip ID: ");          Serial.println(ESP.getFlashChipId());
  Serial.print("\tCore Version: ");     Serial.println(ESP.getCoreVersion());
  Serial.print("\tChip Real Size: ");   Serial.println(ESP.getFlashChipRealSize());
  Serial.print("\tChip Flash Size: ");  Serial.println(ESP.getFlashChipSize());
  Serial.print("\tChip Flash Speed: "); Serial.println(ESP.getFlashChipSpeed());
  Serial.print("\tChip Speed: ");       Serial.println(ESP.getCpuFreqMHz());
  Serial.print("\tChip Mode: ");        Serial.println(ESP.getFlashChipMode());
  Serial.print("\tSketch Size: ");      Serial.println(ESP.getSketchSize());
  Serial.print("\tSketch Free Space: ");Serial.println(ESP.getFreeSketchSpace());
}
*/
