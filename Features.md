# V3 Features

* Embedded maintenance page (terminal / local FS update / ESP3D Firmware update)
* WebUI support
* ESP8285 / ESP8266 / ESP32 / ESP32-S2 / ESP32-S3 / ESP32-C3 support
* Wifi / ethernet support
* Raw TCP / serial bridge support (light telnet)
* Boot delay configuration
* Websocket / serial bridge support
* BT Serial support (when supported)
* MKS Serial protocol support
* Serial commands configurations
* Authentication support (admin / user)
* FTP support (limited to 1 connection at once)
* WebDav support
* Local FS support:
    * Little FS (prefered)
    * Fat (ESP32 only)
    * SPIFFS (deprecated)
* SD support
    * File format
      * Native SPI
      * Native SDIO (ESP32 only)
      * SDFat 1.x
      * SDFat 2.x
    * Connection
      * Direct connection
        * e.g.: ESP32cam
      * Sharing connection using hardware switch
        * e.g.: Panucatt Wifi Backpack / Azteeg X5 WiFi
      * MKS fast upload by serial
      * NOT SUPPORTED
        * M28/M29 File transfer protocol
    * USB support
        * planned
* Global FS under FTP / Webdav : SD + Local FS in same directory
* Buzzer support
* Recovery pin support
* ESP32 Camera support
* Basic oled screen support
* Basic tft screen support
* Time synchronization support (manual / internet server)
* Lua interpreter support
* Notifications support
    * WebUI
    * Email
    * Line
    * Telegram
    * PushOver
    * IFTTT
* Sensors support
    * DHT 11/22
    * Analog
    * BMX280
* Auto script support at start
* Basic Host GCODE stream for macros hosted on local FS


