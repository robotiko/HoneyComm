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

#import "SerialBitstream.h"


/**
 * Assumes that the bit stream was sent by over the air using a start- and stop-bit.
 * This method removes these 2 bits from the bit stream and reverses the bit order.
 *
 * @param inputBuffer Points to the undecoded data.
 * @param inputBufferLen Number of bytes of undecoded data.
 * @param outputBuffer Points to the decoded data.
 * @param outputBufferLen Number of bytes of decoded data.
 */
void SerialBitstream::decode(byte *inputBuffer, byte inputBufferLen, byte *outputBuffer, byte *outputBufferLen) {

  // Remove the start- and stop-bits in the bitstream
  // #0123456 7##01234 567##012 34567##0 1234567# (# -> Start/Stop bit)
  // is decoded to:
  // 76543210 76543210 76543210 76543210

  //bitstream.show(inputBuffer, inputBufferLen);

  int j = 0;
  for (int i=0 ; i < (inputBufferLen * 8) ; i++) {

    if (i % 10 == 0) {
      // Start bit. Ignore.
    } 
    else if ((i - 9) % 10 == 0) {
      // Stop bit. Ignore.
    } 
    else {
      int bytepos = i / 8;
      int bitpos = i % 8;

      byte mask = 1 << (7 - bitpos);
      if ((inputBuffer[bytepos] & mask) > 0) {

        bytepos = j / 8;
        bitpos = j % 8;

        outputBuffer[bytepos] |= 1 << bitpos;

      } 
      else {

        bytepos = j / 8;
        bitpos = j % 8;

        outputBuffer[bytepos] &= ~(1 << bitpos);
      }

      j++;
    }
  }

  *outputBufferLen = (j / 8) + 1;

  // bitstream.show(outputBuffer, *outputBufferLen);
}

/**
 * Reverses the bit order of the input data and adds a start bit before and a stop bit
 * after each byte.
 *
 * @param inputBuffer Points to the unencoded data.
 * @param inputBufferLen Number of bytes of unencoded data.
 * @param outputBuffer Points to the encoded data.
 * @param outputBufferLen Number of bytes of encoded data.
 */
void SerialBitstream::encode(byte *inputBuffer, byte inputBufferLen, byte *outputBuffer, byte *outputBufferLen) {

  // Adds a start and stop bit and reverses the bit order.
  // 76543210 76543210 76543210 76543210
  // is encoded to:
  // #0123456 7##01234 567##012 34567##0 1234567# (# -> Start/Stop bit)

  bitstream.show(inputBuffer, inputBufferLen);

  int bytepos;
  int bitpos;

  int j = 0;
  for (int i=0 ; i < (inputBufferLen * 8) ; i++) {

    if (i % 8 == 0) {
      // Insert start bit (0)
      bytepos = j / 8;
      bitpos = j % 8;

      outputBuffer[bytepos] &= ~(1 << (7 - bitpos));
      j++;
    }

    bytepos = i / 8;
    bitpos = i % 8;

    byte mask = 1 << bitpos;
    if ((inputBuffer[bytepos] & mask) > 0) {

      bytepos = j / 8;
      bitpos = 7 - (j % 8);

      outputBuffer[bytepos] |= 1 << bitpos;

    } else {

      bytepos = j / 8;
      bitpos = 7 - (j % 8);

      outputBuffer[bytepos] &= ~(1 << bitpos);
    }

    j++;

    if ((i - 7) % 8 == 0) {
      // Insert stop bit (1)
      bytepos = j / 8;
      bitpos = j % 8;

      outputBuffer[bytepos] |= 1 << (7 - bitpos);
      j++;
    }
  }


  *outputBufferLen = (j / 8) + 1;

  bitstream.show(outputBuffer, *outputBufferLen);

}



/**
 * Displays the buffer in binary/hex format.
 *
 * @param buffer The buffer to use.
 * @param len Number of bytes in the buffer.
 */
void SerialBitstream::show(byte* buffer, int len)
{
#ifdef DEBUG_SHOW_HEX
  for (int i=0 ; i<len ; i++) {
    if (buffer[i] < 0x10) Serial.print(F("0"));
    Serial.print(buffer[i], HEX);
    Serial.print(F(" "));
  }
  Serial.println();
#endif

#ifdef DEBUG_SHOW_BINARY  
  const byte *ptr;
  for ( ptr = buffer; len--; ptr++ )
  {
    byte mask;
    for ( mask = 0x80 ; mask ; mask >>= 1 )
    {
      Serial.print(mask & *ptr ? '1' : '0');
    }
    Serial.print(F(" "));
  }
  Serial.println();
#endif
}

SerialBitstream bitstream = SerialBitstream();

