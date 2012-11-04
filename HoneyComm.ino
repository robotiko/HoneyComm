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

#include "globals.h"
#include "CCx.h"
#include "CCxCfg.h"
#include "rfBeeSerial.h"
#include "SerialBitstream.h"
#include "ManchesterByteStream.h"

#define PD2 2 // Wired with GDO0 on CC1101
#define SERIAL_DATA PD2
#define PD3 3 // Wired with GDO2 on CC1101
#define SERIAL_CLK PD3

void setup() {

  delay(500);

  CCx.PowerOnStartUp();
  setCCxConfig();

  // Manually control output pin TX
  pinMode(PD1, OUTPUT);
}


//
// Main loop
//
void loop() {

  noInterrupts();

  while(true) {

    // Wait for raising edge of SERIAL_CLK
    while (! (PIND & (1<<SERIAL_CLK)));

    if (PIND & (1<<SERIAL_DATA)) {
      // Set TX to HIGH
      PORTD |= 1<<PD1;
    } 
    else {
      // Set TX to LOW
      PORTD &= ~(1<<PD1);
    }
  }
}













