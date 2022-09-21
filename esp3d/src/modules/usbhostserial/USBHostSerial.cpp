#include "../../include/esp3d_config.h"

#include "USBHostSerial.h"
#include "usb/usb_host.h"
#include "CircularBuffer.h"

CircularBuffer<uint8_t, 512> buffer;


#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "cp210x_usb.hpp"
#include "ftdi_usb.hpp"
#include "ch34x_usb.hpp"
#include "usb/cdc_acm_host.h"
#include "usb/usb_host.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#ifdef DISPLAY_DEVICE
#include "../display/display.h"
#endif //DISPLAY_DEVICE

#include "../../core/esp3doutput.h"


CdcAcmDevice *vcp = nullptr;
uint16_t vid = 0;
uint16_t pid = 0;
uint16_t interface_idx = 0;
bool new_dev_cb_called = false;

using namespace esp_usb;

// static portMUX_TYPE buf_lock = portMUX_INITIALIZER_UNLOCKED;
// #define BUFFER_ENTER_CRITICAL()   portENTER_CRITICAL(&buf_lock)
// #define BUFFER_EXIT_CRITICAL()    portEXIT_CRITICAL(&buf_lock)


static void handle_rx(uint8_t *data, size_t data_len, void *arg)
{
  // BUFFER_ENTER_CRITICAL();
  // printf("%.*s", data_len, data);
  for (int i = 0 ; i < data_len; ++i) {
    buffer.push(data[i]);
  }
  // BUFFER_EXIT_CRITICAL();

  // 是否需要释放 data
}

static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
  switch (event->type) 
  {
      case CDC_ACM_HOST_ERROR: // USB 端口错误
        log_esp3d( "CDC-ACM error has occurred, err_no = %d", event->data.error);
        break;
      case CDC_ACM_HOST_DEVICE_DISCONNECTED: // USB 设备断开
        log_esp3d( "Device suddenly disconnected");
        break;
      case CDC_ACM_HOST_SERIAL_STATE: // 串口状态
        log_esp3d( "serial state notif 0x%04X", event->data.serial_state.val);
        break;
      case CDC_ACM_HOST_NETWORK_CONNECTION: // 连接
        log_esp3d( "CDC_ACM_HOST_NETWORK_CONNECTION.");
        break;
      default: 
        break;
  }
}

bool isRunTask = false;
void usb_lib_task(void *arg)
{
    while (1) {
        // Start handling system events
        // 处理usb事件
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        
        // 事件：已没有USB客户端
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all()); // 释放掉被标记为离线的USB客户端
        }

        // 事件：释放已完成
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            log_esp3d( "USB: All devices freed");
            // Continue handling USB events to allow device reconnection
            if (!isRunTask) {
              break;
            }
        }
    }
}


static void new_dev_cb(usb_device_handle_t usb_dev)
{
    const usb_device_desc_t *device_desc;
    ESP_ERROR_CHECK( usb_host_get_device_descriptor(usb_dev, &device_desc));
    vid =  device_desc->idVendor;
    pid =  device_desc->idProduct;

    const usb_config_desc_t *config_desc;
    ESP_ERROR_CHECK( usb_host_get_active_config_descriptor(usb_dev, &config_desc));
    interface_idx = config_desc->bNumInterfaces;

    new_dev_cb_called = true;

    log_esp3d( "new_dev_cb_called");
}

static void dev_gone_cb(usb_device_handle_t usb_dev)
{
    if (vcp) {
        delete vcp;
        vcp = nullptr;
        log_esp3d( "vcp freed");
    }
}


bool USBHostSerial::open_VCP_device()
{
  if (vcp) {
    delete vcp;
    vcp = nullptr;
  }

  const cdc_acm_host_device_config_t dev_config = {
      .connection_timeout_ms = 10000,
      .out_buffer_size = 64,
      .event_cb = handle_event,
      .data_cb = handle_rx,
      .user_arg = NULL,
  };
  
  try {
    switch (vid)
    {
    case FTDI_VID:
        vcp = FT23x::open_ftdi(pid, &dev_config);
        log_esp3d( "open_ftdi");
        break;
    case SILICON_LABS_VID:
        vcp = CP210x::open_cp210x(pid, &dev_config);
        log_esp3d( "open_cp210x");
        break;
    case NANJING_QINHENG_MICROE_VID:
        vcp = CH34x::open_ch34x(pid, &dev_config);
        log_esp3d( "open_ch34x");
        break;
    default:
        return false;
    }
  } catch(esp_err_t err) {
    return false;
  }

#ifdef DISPLAY_DEVICE
  String str = "vid:" + String(vid, 16) + "pid:" + String(pid, 16);
  ESP3DOutput::toScreen(ESP_OUTPUT_STATUS, str.c_str() );
  delay(1000);
  String sc = String(m_baudRate) + "," + String(m_stopBit) + "," + String(m_parityType) + "," + String(m_dataBits);
  ESP3DOutput::toScreen(ESP_OUTPUT_STATUS, sc.c_str());
  delay(1000);
#endif //DISPLAY_DEVICE

  log_esp3d( "open vcp succeed.");

  cdc_acm_line_coding_t line_coding = {
      .dwDTERate   = m_baudRate,
      .bCharFormat = m_stopBit,
      .bParityType = m_parityType,
      .bDataBits   = m_dataBits,
  };

  vcp->line_coding_set(&line_coding);

  return true;
  /*
  ESP_ERROR_CHECK(vcp->set_control_line_state(false, true));
  ESP_ERROR_CHECK(vcp->tx_blocking((uint8_t *)"Test string", 12));
  */

}

USBHostSerial::USBHostSerial()
{

}

bool USBHostSerial::setup()
{
  // 安装USB主机
  log_esp3d( "Installing USB Host");
#ifdef DISPLAY_DEVICE
  ESP3DOutput::toScreen(ESP_OUTPUT_STATUS, "Installing USB Host" );
#endif //DISPLAY_DEVICE

  const usb_host_config_t host_config = {
      .skip_phy_setup = false,
      .intr_flags = ESP_INTR_FLAG_LEVEL1,
  };

  if (ESP_OK == usb_host_install(&host_config)) 
  {
    // Create a task that will handle USB library events 
    // 创建任务：处理USB库事件
    isRunTask = true;
    xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL); 

    // 安装 USB-CDC-ACM 主机
    const cdc_acm_host_driver_config_t driver_config = {
        .driver_task_stack_size = 4960,
        .driver_task_priority = 10,
        .xCoreID = 0,
        .new_dev_cb = new_dev_cb,
        .dev_gone_cb = dev_gone_cb,
    };

    log_esp3d("Installing CDC-ACM driver");
    if ( ESP_OK == cdc_acm_host_install(&driver_config))
      m_hostInstalled = true;
  }
#ifdef DISPLAY_DEVICE
  ESP3DOutput::toScreen(ESP_OUTPUT_STATUS, m_hostInstalled ? "USB Installed." : "Install USB faild." );
  delay(1000);
#endif //DISPLAY_DEVICE

  return m_hostInstalled;
}


bool USBHostSerial::begin(unsigned long baud, uint32_t config)
{
  if (vcp) {
    return true;
  }
  // BUFFER_ENTER_CRITICAL();
  buffer.clear();
  // BUFFER_EXIT_CRITICAL();


  // 安装USB主机驱动，只能调用一次
  if (!m_hostInstalled) 
    if (!setup()) {
      return false;
    }

  m_baudRate = baud;
  // 见 esp32-hal-uart.c #158
  m_dataBits = (config & 0xc) >> 2;
  m_parityType = (config & 0x3);
  m_stopBit = (config & 0x30) >> 4;


#ifdef DISPLAY_DEVICE
  ESP3DOutput::toScreen(ESP_OUTPUT_STATUS, "wait USB device." );
  delay(1000);
#endif //DISPLAY_DEVICE


  static unsigned long timeout = millis() + 1000;
  while (timeout > millis())
  {
    if (new_dev_cb_called ) {
      new_dev_cb_called = false;
#ifdef DISPLAY_DEVICE
        ESP3DOutput::toScreen(ESP_OUTPUT_STATUS, "USB device det." );
        delay(1000);
#endif //DISPLAY_DEVICE

      if (open_VCP_device())
      {
#ifdef DISPLAY_DEVICE
        ESP3DOutput::toScreen(ESP_OUTPUT_STATUS, "USB device opened." );
#endif //DISPLAY_DEVICE
        String cmd = "G28 Y\n";
        vcp->tx_blocking((const uint8_t *)cmd.c_str(), cmd.length());
        return true;
      }

      break;
    }

    delay(1);      
  }
  
#ifdef DISPLAY_DEVICE
  ESP3DOutput::toScreen(ESP_OUTPUT_STATUS, "failed to open USB device." );
  delay(1000);
#endif //DISPLAY_DEVICE
  return false;
}

void USBHostSerial::end()
{  
  if (vcp == nullptr) {
    return;
  }
  
  delete vcp;
  vcp = nullptr;
  // BUFFER_ENTER_CRITICAL();
  buffer.clear();
  // BUFFER_EXIT_CRITICAL();
}

void USBHostSerial::updateBaudRate(unsigned long baud)
{
  m_baudRate = baud;

  if (vcp == nullptr) {
    return;
  }

  log_esp3d("Setting up line coding");

  cdc_acm_line_coding_t line_coding = {
      .dwDTERate   = m_baudRate,
      .bCharFormat = m_stopBit,
      .bParityType = m_parityType,
      .bDataBits   = m_dataBits,
  };
  ESP_ERROR_CHECK(vcp->line_coding_set(&line_coding));
}

int USBHostSerial::available(void)
{
  return buffer.size();
}

int USBHostSerial::availableForWrite(void)
{
    return 64; // ling 64
}

size_t USBHostSerial::read(uint8_t *buf, size_t size)
{
  size_t bfSize = buffer.size();
  // BUFFER_ENTER_CRITICAL();
  size_t rdLen = bfSize < size ? bfSize : size;
  for (size_t i = 0; i < rdLen; ++i) {
    *(buf+i) = buffer.shift();
  }
  // BUFFER_EXIT_CRITICAL();

  return rdLen;
}

int USBHostSerial::peek(void)
{
  uint8_t d = 0;
  if (available()) {
    // BUFFER_ENTER_CRITICAL();
    d = buffer[0];
    // BUFFER_EXIT_CRITICAL();
  }
  return d;
}

int USBHostSerial::read(void)
{
  uint8_t val = 0;
  this->read(&val, 1);
  return val;
}

void USBHostSerial::flush(void)
{
  // empty
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
  if (vcp == nullptr) {
    return 0;
  }

  ESP_ERROR_CHECK(vcp->tx_blocking((uint8_t *)buffer, size));
  
  return size;
}

uint32_t USBHostSerial::baudRate()
{
  return m_baudRate;
}

USBHostSerial::operator bool() const
{
  return vcp != nullptr;
}

size_t USBHostSerial::setRxBufferSize(size_t new_size)
{
  if (vcp) {
      log_esp3d("RX Buffer can't be resized when Serial is already running.\n");
      return 0;
  }

  // if (new_size <= SOC_UART_FIFO_LEN) {
  //     log_e("RX Buffer must be higher than %d.\n", SOC_UART_FIFO_LEN);  // ESP32, S2, S3 and C3 means higher than 128
  //     return 0;
  // }

  m_rxBufferSize = new_size;
  return m_rxBufferSize;
}


USBHostSerial SerialUSB;