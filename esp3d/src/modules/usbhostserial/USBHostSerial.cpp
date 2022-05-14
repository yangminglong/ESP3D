#if 1
#include <Arduino.h>
#include "usb/usb_host.h"
#include "USBHostSerial.h"
#include "CircularBuffer.h"
#include "../../include/esp3d_config.h"
#include "usb_acm.hpp"
#include "usb_host.hpp"

USBacmDevice *device = nullptr;   // when USB class is detected from descriptor
USBhost host;           // host is required to detect any device, before USB class is initialized
unsigned long bitRate = 250000;
CircularBuffer<uint8_t, 1024> buffer;

void acm_events(int event, void *data, size_t len)
{
    log_esp3d("acm_events. addr:%x, %d", (int)data, len);
    const char* chData = (const char*)data;
    String str(chData, len);

    // for (size_t i = 0; i < len && data; ++i ) {
    //     printf("%c", *(char*)(data+i));
    // }
    switch (event)
    {

    case CDC_DATA_IN: //1
      {
        log_esp3d("\tCDC_DATA_IN:%d, receive: %s\n", len, str.c_str());
        for (size_t i = 0; i < len ; ++i ) {
          uint8_t ch = *(chData+i);
          buffer.push(ch);
        }
        break;
      }
    case CDC_DATA_OUT: // 2
      {
        log_esp3d("\tCDC_DATA_OUT:%d, send: %s\n", len, str.c_str());
      }
        break;
    case CDC_CTRL_SET_CONTROL_LINE_STATE:
        log_esp3d("CDC_CTRL_SET_CONTROL_LINE_STATE");
        if (device)
          device->setLineCoding(bitRate, 0, 0, 8);
        else 
          log_esp3d("err setLineCoding");
        break;
    case CDC_CTRL_SET_LINE_CODING:
        log_esp3d("CDC_CTRL_SET_LINE_CODING");
        // device->INDATA();
        break;
    case CDC_CTRL_GET_LINE_CODING: 
        // device->INDATA();
        break;
    }
}


void client_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    log_esp3d("client_event_callback.");
    if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV)
    {
        usb_device_info_t info = host.getDeviceInfo();
        log_esp3d("device speed: %s\n device address: %d\n max ep_ctrl size: %d\n config: %d\n\n", info.speed ? "USB_SPEED_FULL" : "USB_SPEED_LOW", info.dev_addr, info.bMaxPacketSize0, info.bConfigurationValue);
        const usb_device_desc_t *dev_desc = host.getDeviceDescriptor();
        printf("idVendor: %d\n idProduct: %d\n iSerialNumber: %d\n", dev_desc->idVendor, dev_desc->idProduct, dev_desc->iSerialNumber );

        int offset = 0;
        for (size_t i = 0; i < dev_desc->bNumConfigurations; i++)
        {
            const usb_config_desc_t *config_desc = host.getConfigurationDescriptor();
            for (size_t n = 0; n < config_desc->bNumInterfaces; n++)
            {
                const usb_intf_desc_t *intf = usb_parse_interface_descriptor(config_desc, n, 0, &offset);
                if (intf->bInterfaceClass == 0x0a || intf->bInterfaceClass == 0xFF) // CDC ACM
                {
                    log_esp3d("new USBacmDevice, offset:", offset);
                    device = new USBacmDevice(config_desc, &host);
                    n = config_desc->bNumInterfaces;
                    if (device)
                    {
                      log_esp3d("device->init()");
                      device->onEvent(acm_events);
                      device->init();
                      device->setControlLine(1, 0);
                      // device->INDATA();

                    }
                }

                printf("config: %d[%d], interface: %d[%d], intf class: %d\n", i, dev_desc->bNumConfigurations, n, config_desc->bNumInterfaces, intf->bInterfaceClass);
            }
        }
    }
    else // USB_HOST_CLIENT_EVENT_DEV_GONE
    {
      log_w("DEVICE gone event"); //USB_HOST_CLIENT_EVENT_DEV_GONE
      // host.close();
      if (device) {
        device->onEvent(nullptr);
        device->release();
        delete device;
        device = nullptr;
        // host.init();
      } 
      buffer.clear();

    }
}

USBHostSerial::USBHostSerial()
{
}


bool USBHostSerial::begin(unsigned long baud)
{
  // if (device == nullptr) {
  //   return false;
  // }

  // bitRate = baud;

  // device->setControlLine(1, 0);
  // device->INDATA();
  // buffer.clear();
  log_i("USBHostSerial::begin"); 

  host.registerClientCb(client_event_callback);
  host.init(true);


  return true;
}

void USBHostSerial::end()
{
  // if (device) {
  //   device->release();
  //   device->onEvent(nullptr);
  //   delete device;
  //   device = nullptr;
  // } 
  // buffer.clear();
}

void USBHostSerial::updateBaudRate(unsigned long baud)
{
  if (device && device->isConnected()) {
    bitRate = baud;
    device->setControlLine(1, 1);
  }
}

int USBHostSerial::available(void)
{
  if (!host.isValid())
    return 0;
  if (device==nullptr)
    return 0;
  if (!device->isConnected())
    return 0;


  // device->INDATA();
  // if (buffer.size() == 0) {
  //   // log_w("INDATA"); 
  //   // log_w("INDATA done"); 
  // }

  // if(buffer.size())
  //   log_esp3d("usb data available: %d\n", buffer.size());

  // return 0;
  return buffer.size();
}

int USBHostSerial::availableForWrite(void)
{
  if (device && device->isConnected()) {
    return 64;
  }
  return 0;
}

size_t USBHostSerial::read(uint8_t *buf, size_t size)
{
  // log_w("enter read"); 
  if (size == 0)
    return 0;
  if (!host.isValid())
    return 0;
  if (device==nullptr)
    return 0;
  if (!device->isConnected())
    return 0;

  // log_w("read"); 
  size_t rdLen = 0;
  // if (available()) {
    // while (buffer.size() < size && buffer.size() < 512) {
    //   // log_w("INDATA"); 
    //   device->INDATA();
    //   // log_w("INDATA done"); 
    // }

    // size_t lenOld = buffer.size();
    rdLen = size < 0 ? buffer.size() :  buffer.size() < size ? buffer.size() : size;
    for (size_t i = 0; i < rdLen; ++i) {
      *(buf+i) = buffer.shift();
    }
    // size_t lenNew = buffer.size();
    // output
    // log_esp3d("lenOld: %d. lenNew: %d. \n", lenOld, lenNew);
  // }
  return rdLen;
}

int USBHostSerial::peek(void)
{
  if (available()) {
    return buffer.first();
  }
  return 0;
}

int USBHostSerial::read(void)
{
  uint8_t val = 0;
  this->read(&val, 1);
  return val;
}

size_t USBHostSerial::readBytes(uint8_t *buffer, size_t length)
{
  return read(buffer, length);
}

void USBHostSerial::flush(void)
{
  
}

void USBHostSerial::flush( bool txOnly)
{
  (void)txOnly;
}

size_t USBHostSerial::write(uint8_t c)
{
  return this->write(&c, 1);
}

size_t USBHostSerial::write(const uint8_t *buffer, size_t size)
{
  if (device == nullptr || !device->isConnected())
    return 0;
  
  size_t wtSize = size > 512 ? 512 : size;
  device->OUTDATA(buffer, wtSize);
  return wtSize;
}

uint32_t USBHostSerial::baudRate()
{
  return bitRate;
}

USBHostSerial::operator bool() const
{
  return device && device->isConnected();
}

size_t USBHostSerial::setRxBufferSize(size_t new_size)
{

  m_bufferSize = new_size > 512 ? 512 : new_size;
  return m_bufferSize;
}


USBHostSerial SerialUSB;

#endif