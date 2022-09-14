#ifndef _USB_HOST_SERIAL_H
#define _USB_HOST_SERIAL_H

#include <Arduino.h>

class CdcAcmDevice;

class USBHostSerial : public Stream
{
public:

public:
    USBHostSerial();
    bool begin(unsigned long baud, uint32_t config=SERIAL_8N1);
    void end();
    void updateBaudRate(unsigned long baud);
    int available(void);
    int availableForWrite(void);
    int peek(void);
    int read(void);
    size_t read(uint8_t *buffer, size_t size);
    inline size_t read(char * buffer, size_t size)
    {
      return read((uint8_t*) buffer, size);
    }

    void flush(void);
    void flush( bool txOnly);
    size_t write(uint8_t);
    size_t write(const uint8_t *buffer, size_t size);
    
    inline size_t write(const char * buffer, size_t size)
    {
        return write((uint8_t*) buffer, size);
    }
    inline size_t write(const char * s)
    {
        return write((uint8_t*) s, strlen(s));
    }
    inline size_t write(unsigned long n)
    {
        return write((uint8_t) n);
    }
    inline size_t write(long n)
    {
        return write((uint8_t) n);
    }
    inline size_t write(unsigned int n)
    {
        return write((uint8_t) n);
    }
    inline size_t write(int n)
    {
        return write((uint8_t) n);
    }
    uint32_t baudRate();
    operator bool() const;

    void setDebugOutput(bool);
    
    // void setRxInvert(bool);
    // void setPins(uint8_t rxPin, uint8_t txPin);
    size_t setRxBufferSize(size_t new_size);
private:
    bool setup();
    bool open_VCP_device();
private:
    size_t m_rxBufferSize = 64;
    uint32_t m_baudRate = 256000;
    uint8_t m_stopBit = 0;
    uint8_t m_parityType = 0;
    uint8_t m_dataBits = 8;


    bool m_hostInstalled = false;



};

extern USBHostSerial SerialUSB;

#endif