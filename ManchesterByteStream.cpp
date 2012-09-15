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

#include "ManchesterByteStream.h"
#include "SerialBitstream.h"


// Table for encoding 4-bit data into a 8-bit Manchester encoding.
static byte manchEncodeTab[16] = {
  0xAA,  // 0x0 Manchester encoded
  0xA9,  // 0x1 Manchester encoded
  0xA6,  // 0x2 Manchester encoded
  0xA5,  // 0x3 Manchester encoded
  0x9A,  // 0x4 Manchester encoded
  0x99,  // 0x5 Manchester encoded
  0x96,  // 0x6 Manchester encoded
  0x95,  // 0x7 Manchester encoded
  0x6A,  // 0x8 Manchester encoded
  0x69,  // 0x9 Manchester encoded
  0x66,  // 0xA Manchester encoded
  0x65,  // 0xB Manchester encoded
  0x5A,  // 0xC Manchester encoded
  0x59,  // 0xD Manchester encoded
  0x56,  // 0xE Manchester encoded
  0x55}; // 0xF Manchester encoded

// Table for decoding 4-bit Manchester encoded data into 2-bit 
// data. 0xFF indicates invalid Manchester encoding
static byte manchDecodeTab[16] = {
  0xFF, //  Manchester encoded 0x0 decoded
  0xFF, //  Manchester encoded 0x1 decoded
  0xFF, //  Manchester encoded 0x2 decoded
  0xFF, //  Manchester encoded 0x3 decoded
  0xFF, //  Manchester encoded 0x4 decoded
  0x03, //  Manchester encoded 0x5 decoded
  0x02, //  Manchester encoded 0x6 decoded
  0xFF, //  Manchester encoded 0x7 decoded
  0xFF, //  Manchester encoded 0x8 decoded
  0x01, //  Manchester encoded 0x9 decoded
  0x00, //  Manchester encoded 0xA decoded
  0xFF, //  Manchester encoded 0xB decoded
  0xFF, //  Manchester encoded 0xC decoded
  0xFF, //  Manchester encoded 0xD decoded
  0xFF, //  Manchester encoded 0xE decoded
  0xFF}; //  Manchester encoded 0xF decoded


void ManchesterByteStream::encodeByte(byte *unencoded, byte *encoded)
{
  byte data0, data1;

  // Shift to get 4-bit data values
  data1 = (((*unencoded) >> 4) & 0x0F);
  data0 = ((*unencoded) & 0x0F);

  // Perform Manchester encoding
  *encoded       = (manchEncodeTab[data1]);
  *(encoded + 1) = manchEncodeTab[data0];
}


byte ManchesterByteStream::decodeByte(byte *undecoded, byte *decoded)
{
  byte data0, data1, data2, data3;

  // - Shift to get 4 bit data and decode
  data3 = ((*undecoded >> 4) & 0x0F);
  data2 = ( *undecoded       & 0x0F);
  data1 = ((*(undecoded + 1) >> 4) & 0x0F);
  data0 = ((*(undecoded + 1))      & 0x0F);

  // Check for invalid Manchester encoding
  if ( (manchDecodeTab[data3] == 0xFF ) | (manchDecodeTab[data2] == 0xFF ) |
    (manchDecodeTab[data1] == 0xFF ) | (manchDecodeTab[data0] == 0xFF ) )
  {
    return(MAN_DECODING_ERROR);
  }

  // Shift result into a byte
  *decoded = (manchDecodeTab[data3] << 6) | (manchDecodeTab[data2] << 4) |
    (manchDecodeTab[data1] << 2) |  manchDecodeTab[data0];

  return(MAN_DECODING_OK);
}

byte ManchesterByteStream::decode(byte *inputBuffer, byte inputBufferLen, byte *outputBuffer, byte *outputBufferLen) 
{
  for (int i=0 ; i<inputBufferLen ; i+=2) {
    if (MAN_DECODING_ERROR == manchesterByteStream.decodeByte(inputBuffer + i, outputBuffer + (i/2))) {
      Serial.print(F("ERROR: Manchester-Decoding the message: "));
      bitstream.show(inputBuffer, inputBufferLen);
      return 0;
    }
  }

  *outputBufferLen = (byte) (inputBufferLen >> 1);

  return *outputBufferLen;
}

ManchesterByteStream manchesterByteStream = ManchesterByteStream();



