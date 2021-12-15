#ifndef _USB_HOST_SERIAL_H
#define _USB_HOST_SERIAL_H

#include "esp32-hal-uart.h"
#include "Stream.h"
#include "usb_acm.hpp"
#include "usb_host.hpp"

class USBHostSerial : public Stream
{

public:
    USBHostSerial();

    bool begin(unsigned long baud = 250000);
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
    
    void setRxInvert(bool);
    void setPins(uint8_t rxPin, uint8_t txPin);
    size_t setRxBufferSize(size_t new_size);

private:
};

extern USBHostSerial SerialUSB;

#endif