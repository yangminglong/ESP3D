#include <Arduino.h>
#include "usb/usb_host.h"
#include "USBHostSerial.h"
#include "../CircularBuffer/CircularBuffer.h"

USBacmDevice *device = nullptr;   // when USB class is detected from descriptor
USBhost host;           // host is required to detect any device, before USB class is initialized
unsigned long bitRate = 250000;
CircularBuffer<uint8_t, 128> buffer;

void acm_events(int event, void *data, size_t len)
{
    printf("event: %d, datalen: %d\n", event, len );

    // for (size_t i = 0; i < len && data; ++i ) {
    //     printf("%c", *(char*)(data+i));
    // }
    switch (event)
    {

    case CDC_DATA_IN:
    {
      for (size_t i = 0; i < len ; ++i ) {
        buffer.push(*(((uint8_t*)data)+i));
      }

      break;
    }
    case CDC_DATA_OUT:
        // device->INDATA();
        break;
    case CDC_CTRL_SET_CONTROL_LINE_STATE:
        log_i("CDC_CTRL_SET_CONTROL_LINE_STATE");
        device->setLineCoding(bitRate, 0, 0, 8);
        break;
    case CDC_CTRL_SET_LINE_CODING:
        log_i("CDC_CTRL_SET_LINE_CODING");
        // device->INDATA();
        break;
    case CDC_CTRL_GET_LINE_CODING: 
        // device->INDATA();
        break;
    }
}


void client_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV)
    {
        host.open(event_msg);
        usb_device_info_t info = host.getDeviceInfo();
        log_i("device speed: %s\n device address: %d\n max ep_ctrl size: %d\n config: %d\n\n", info.speed ? "USB_SPEED_FULL" : "USB_SPEED_LOW", info.dev_addr, info.bMaxPacketSize0, info.bConfigurationValue);
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
                    log_i("new USBacmDevice, offset:", offset);
                    device = new USBacmDevice(config_desc, &host);
                    n = config_desc->bNumInterfaces;
                    if (device)
                    {
                      log_i("device->init()");
                      device->init();
                    }
                }

                printf("config: %d[%d], interface: %d[%d], intf class: %d\n", i, dev_desc->bNumConfigurations, n, config_desc->bNumInterfaces, intf->bInterfaceClass);
            }
        }
    }
    else // USB_HOST_CLIENT_EVENT_DEV_GONE
    {
      host.close();
      if (device) {
        delete device;
        device = nullptr;
      } 
      log_w("DEVICE gone event"); //USB_HOST_CLIENT_EVENT_DEV_GONE
    }
}

USBHostSerial::USBHostSerial()
{
  host.registerClientCb(client_event_callback);
  host.init();
}


bool USBHostSerial::begin(unsigned long baud)
{
  if (device == nullptr) {
    return false;
  }

  bitRate = baud;

  device->onEvent(acm_events);
  device->setControlLine(1, 1);
  // device->INDATA();
  buffer.clear();

  return true;
}

void USBHostSerial::end()
{
  device->onEvent(nullptr);
  buffer.clear();
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
  if (device == nullptr || !device->isConnected()) {
    return 0;
  }

  if (buffer.size() == 0) {
    device->INDATA();
  }
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
  size_t rdLen = 0;
  if (available()) {
    if (buffer.size() < size && buffer.size() < 64)
      device->INDATA();
    rdLen = buffer.size() < size ? buffer.size() : size;
    for (size_t i = 0; i < rdLen; ++i) {
      *(buf+i) = buffer.shift();
    }
  }
  return rdLen;
}

int USBHostSerial::peek(void)
{
  if (available()) {
    return buffer[0];
  }
  return 0;
}

int USBHostSerial::read(void)
{
  uint8_t val = 0;
  this->read(&val, 1);
  return val;
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
  
  size_t wtSize = size > 64 ? 64 : size;
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
  (void)new_size;
  return 64;
}


USBHostSerial SerialUSB;