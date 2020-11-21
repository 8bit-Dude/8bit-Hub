#ifndef MOUSE_H_

#define MOUSE_H_

typedef struct {
    unsigned char x, y;
    unsigned char info;
    unsigned char wheel;
} State;

class PS2Mouse {
private:
    int _clockPin;
    int _dataPin;
    bool _supportsIntelliMouseExtensions;

    void high(int pin);

    void low(int pin);

    char writeAndReadAck(int data);

    char reset();

    void setSampleRate(int rate);

    void checkIntelliMouseExtensions();

    void setResolution(int resolution);

    char getDeviceId();

    void setScaling(int scaling);

    void setRemoteMode();

    char waitForClockState(int expectedState);

    void requestData();

    char readByte();

    int readBit();

    char writeByte(char data);

    char writeBit(int bit);

public:
    PS2Mouse(int clockPin, int dataPin);

    char initialize();

    char update();
	
	State state;
};

#endif // MOUSE_H_