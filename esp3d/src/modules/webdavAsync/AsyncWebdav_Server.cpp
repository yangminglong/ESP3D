/*
  AsyncWebdav_Server.cpp -  webdav server functions class

  Copyright (c) 2014 Luc Lebosse. All rights reserved.

  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with This code; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "../../include/esp3d_config.h"

#if defined (WEBDAV_FEATURE) && WEBDAV_ASYNC
#include <WiFiServer.h>
#include <WiFiClient.h>
#include "AsyncWebdav_Server.h"
#include "../../core/settings_esp3d.h"
#include "../../core/esp3doutput.h"
#include "../../core/commands.h"

AsyncWebdav_Server webdav_server;

void AsyncWebdav_Server::closeClient()
{
    // if(_dav.Client()) {
    //     _dav.Client().stop();
    // }
}

void AsyncWebdav_Server::dir()
{
    // _dav.dir("/", &Serial);
};

bool AsyncWebdav_Server::isConnected()
{
    // if ( !_started) {
    //     return false;
    // }
    // if (_dav.Client()) {
    //     return (_dav.Client().connected());
    // }
    return false;
}

const char* AsyncWebdav_Server::clientIPAddress()
{
    static String res;
    // res = "0.0.0.0";
    // if (_dav.Client() && _dav.Client().connected()) {
    //     res = _dav.Client().remoteIP().toString();
    // }
    return res.c_str();
}


AsyncWebdav_Server::AsyncWebdav_Server()
    :_server(0)
    // , _dav(String(), FILESYSTEM)
{
    _started = false;
    // _port = 0;
}

AsyncWebdav_Server::~AsyncWebdav_Server()
{
    end();
}

/**
 * begin WebDav setup
 */
bool AsyncWebdav_Server::begin()
{
    end();
    // if (Settings_ESP3D::read_byte(ESP_WEBDAV_ON) !=1) {
    //     return true;
    // }
    // _port = Settings_ESP3D::read_uint32(ESP_WEBDAV_PORT);

    // _server.addHandler(&_dav);

    // // start webserver
    // _server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    // _server.begin(_port);

    _started = true;
    return _started;
}
/**
 * End WebDav
 */
void AsyncWebdav_Server::end()
{
    _started = false;
    // _port = 0;
    // closeClient();
    // _server.stop();
    // _dav.end();
}

bool AsyncWebdav_Server::started()
{
    return _started;
}

void AsyncWebdav_Server::handle()
{
    // _dav.handleClient();
}

#endif //WEBDAV_FEATURE

