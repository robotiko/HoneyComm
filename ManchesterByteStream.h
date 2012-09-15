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

#ifndef MANCHESTER_BYTE_STREAM_H
#define MANCHESTER_BYTE_STREAM_H

#define MAN_DECODING_OK      0
#define MAN_DECODING_ERROR   1

#include "Arduino.h"

/**
 * A class for handling Manchester encoding/decoding.
 */
class ManchesterByteStream
{
public:
   void encodeByte(byte *unencoded, byte *encoded);
   byte decodeByte(byte *undecoded, byte *decoded);
   
   byte decode(byte *inputBuffer, byte inputBufferLen, byte *outputBuffer, byte *outputBufferLen);
   
   
};

extern ManchesterByteStream manchesterByteStream;
#endif


