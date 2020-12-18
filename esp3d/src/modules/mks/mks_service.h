/*
  mks_service.cpp -  mks communication service functions class

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

#ifndef _MKS_SERVICES_H
#define _MKS_SERVICES_H

#define MKS_FRAME_SIZE  1024

class MKSService
{
public:
    static bool begin();
    static bool  sendNetworkFrame();
    static void handle();
    static void end();
    static bool started()
    {
        return _started;
    }
private:
    static void clearFrame();
    static bool canSendFrame();
    static bool _started;
    static char _frame[MKS_FRAME_SIZE];
    static char _moduleId[21];
};


#endif //_SERIAL_SERVICES_H
