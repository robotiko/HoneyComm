//
// HoneyComm - Alternative RFBee firmware to communicate with
//             HR80 radiator controllers.
//
// Copyright (C) 2011 Wladimir Komarow
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#ifndef RFBEESERIAL_H
#define RFBEESERIAL_H 1

#include "globals.h"
#include "CCx.h"
#include "rfBeeCore.h"
#include <avr/pgmspace.h>

#define BUFFLEN CCx_PACKT_LEN




int setMyAddress();
int setPowerAmplifier();
int setCCxConfig();
int changeUartBaudRate();
int setRFBeeMode();



#define TRANSMIT_MODE 1     
#define RECEIVE_MODE 2 




#endif
