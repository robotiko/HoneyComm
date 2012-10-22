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

#define PRS_INITIAL 0
#define PRS_PREPARE_RECEIVE 1
#define PRS_RECEIVING_PACKET 2

#define RSSI_OFFSET 74

#define PD2 2
#define PD3 3

volatile byte gdo0;

volatile byte packetReceiveState = PRS_INITIAL;

volatile int cnt;

byte inputBuffer[MAX_PACKET_LENGTH]; // Has to correspond to CC1101 PKTLEN Parameter
byte outputBuffer[MAX_PACKET_LENGTH]; // Has to correspond to CC1101 PKTLEN Parameter

void setup() {

  delay(500);

  Serial.begin(38400);

  CCx.PowerOnStartUp();
  setCCxConfig();

  Serial.println(F("HoneyComm - Copyright (C) 2011 Wladimir Komarow"));
  Serial.println(F("This program comes with ABSOLUTELY NO WARRANTY."));
  Serial.println(F("This is free software, and you are welcome to redistribute it under certain conditions; "));
  Serial.println(F("For details see http://www.gnu.org/licenses/gpl-3.0-standalone.html"));
  delay(500);
}


//
// Main loop
//
void loop() {

  switch (packetReceiveState) {
  case PRS_INITIAL:
    {
      // Check RX state
      byte tmp;
      byte chipStatusByte = CCx.Read(CCx_SNOP, &tmp);

      if ((chipStatusByte & 0x70) > 0) {
        CCx.Strobe(CCx_SIDLE); // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable.
        delay(100);
      }

      packetReceiveState = PRS_PREPARE_RECEIVE;
      break;
    }

  case PRS_PREPARE_RECEIVE:
    {
      CCx.Strobe(CCx_SFRX); // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states.
      CCx.Strobe(CCx_SRX); // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.

      packetReceiveState = PRS_RECEIVING_PACKET;

      break;
    }

  case PRS_RECEIVING_PACKET:
    {
      // Loop until we are in RX state (receive mode)
      cnt = 10000;
      byte status;
      do {
        status = CCx.Strobe(CCx_SNOP) & 0xF0;
        cnt--;
      }
      while (status != 0x10 && cnt>0);

      if (cnt <= 0) {
        packetReceiveState = PRS_PREPARE_RECEIVE;
        break;
      }

      // Loop until GDO0 goes high
      do { 
        gdo0 = digitalRead(PD2);
      }
      while (gdo0 == LOW);

      // Loop until GDO0 goes low
      cnt = 10000;
      do {
        gdo0 = digitalRead(PD2);
        cnt--;
      }
      while ((gdo0 == HIGH) && (cnt > 0));

      // Read number of bytes available in RX FIFO
      byte valueRxBytes;
      CCx.ReadBurst(CCx_RXBYTES, &valueRxBytes, 1);

      byte len = (valueRxBytes & 0x7F);

      // Read the available bytes
      if (len > 0) {
        CCx.ReadBurst(CCx_RXFIFO, inputBuffer, len);

        // Read RSSI status byte
        byte rssiStatusByte;
        CCx.ReadBurst(CCx_RSSI, &rssiStatusByte, 1);

        int rssi = 0;
        if ((rssiStatusByte & 0x80) > 0x00) {
          rssi = (rssiStatusByte - 256) / 2 - RSSI_OFFSET;
        }
        else {
          rssi = rssiStatusByte / 2 - RSSI_OFFSET;
        }

       Serial.print(F("RSSI[dBm]="));
       Serial.println(rssi, DEC);

       //bitstream.show(inputBuffer, len);
       processBitstream(len);

      } // End if length > 0

      packetReceiveState = PRS_PREPARE_RECEIVE;
      break;
    }

  }
}

/**
 * Process the bitstream received. 
 * Looks like the sender sends the bits over RF as if there where transmitted
 * on a RS232 line, including a start bit and a stop bit, and with the 
 * least significant bit appearing first on the line.
 * We have to remove the start- and stop-bits in the bitstream, and then
 * change the byte order.
 *
 * @param len The length of the received bitstream in bytes.
 */
void processBitstream(byte len) {

  byte outputBufferLen;

  // Remove start- and stop-bits, change bit order
  bitstream.decode(inputBuffer, len, outputBuffer, (byte*) &outputBufferLen);

  // RF Preamble is 0xFF 0x00 0x33 0x55 0x53
  // We already configured the CC1101 to use 0xFF 0x00 as 16 bit sync word
  // So we can check here the remaining 3 bytes 0x33 0x55 0x53 of the preamble
  // and ignore a message that does NOT start with this 3 bytes.
  if (outputBuffer[0] == 0x33 && outputBuffer[1] == 0x55 && outputBuffer[2] == 0x53) {

    // We know that there is some kind of RF Postamble 0x35 0x55 ... that marks the end of the message.
    // Find out the position of this message end marker.
    int posMessageEndMarker = -1;
    for (int i=3 ; i<outputBufferLen ; i++) {

      if (outputBuffer[i] == 0x35) {
        posMessageEndMarker = i;

        // Clean up the end of the buffer
        for (int j=i+1 ; j<MAX_PACKET_LENGTH ; j++) {
          outputBuffer[j] = 0x00;
        }

        break; 
      }
    }

    if (posMessageEndMarker < 0) {
      Serial.println(F("ERROR: Cannot find RF postamble."));
      return;
    }

    //bitstream.show(outputBuffer, posMessageEndMarker);

    byte inputBufferLen;
    processByteStream(outputBuffer + 3, posMessageEndMarker - 3, inputBuffer, (byte*) &inputBufferLen);
  }

}

/**
 * The byte stream is manchester-encoded (http://en.wikipedia.org/wiki/Manchester_code).
 * We have to decode the byte stream before we can go on with interpretation of the message.
 *
 * @param inputBuffer Points to the undecoded data.
 * @param inputBufferLen Number of bytes of undecoded data.
 * @param outputBuffer Points to the decoded data.
 * @param outputBufferLen Number of bytes of decoded data.
 */
void processByteStream(byte *inputBuffer, byte inputBufferLen, byte *outputBuffer, byte *outputBufferLen) {

  //bytestream.show(inputBuffer, len);

  if (0 == manchesterByteStream.decode(inputBuffer, inputBufferLen, outputBuffer, (byte*) outputBufferLen)) {
    return; // Something went wrong manchester decoding the message.
  }

  if (0 != verifyChecksum(outputBuffer, *outputBufferLen)) {
    Serial.println(F("ERROR: Wrong checksum in message."));
    return;
  }

  // Manchester-decode the first byte of the message header
  if (0x16 == outputBuffer[0]) {

    if (outputBuffer[5] == 0x1F && outputBuffer[6] == 0xC9) {
      processBindCommand(outputBuffer, *outputBufferLen);
    }
    else if (outputBuffer[5] == 0x11 && outputBuffer[6] == 0x00) {
      process1100Command(outputBuffer, *outputBufferLen);
    }
    else if (outputBuffer[5] == 0x00 && outputBuffer[6] == 0x08) {
      process0008Command(outputBuffer, *outputBufferLen);
    }
    else if (outputBuffer[5] == 0x00 && outputBuffer[6] == 0x09) {
      process0009Command(outputBuffer, *outputBufferLen);
    }
    else if (outputBuffer[5] == 0x1F && outputBuffer[6] == 0x09) {
      process1F09Command(outputBuffer, *outputBufferLen);
    }
    else {
      printDeviceAddress(outputBuffer + 1, 6);
      Serial.print( " 0x16 Header Command ");

      Serial.print(outputBuffer[8], HEX);
      Serial.print(F(" "));

      double temperature = (outputBuffer[9] * 256.0 + outputBuffer[10]) / 100.0;
      Serial.print(F(" T="));
      Serial.println(temperature, 2);

      bitstream.show(outputBuffer, *outputBufferLen);
    }
  }
  else if (0x18 == outputBuffer[0]) {

    if (outputBuffer[7] == 0x23 && outputBuffer[8] == 0x09) {
      processZoneSetpointSettingCommand(outputBuffer, *outputBufferLen);
    }
    else if (outputBuffer[7] == 0x30 && outputBuffer[8] == 0xC9) {
      processZoneTemperatureDistribution(outputBuffer, *outputBufferLen);
    }
    else if (outputBuffer[7] == 0x31 && outputBuffer[8] == 0x50) {
      processHeatDemandTiming(outputBuffer, *outputBufferLen);
    }
    else if (outputBuffer[7] == 0x10 && outputBuffer[8] == 0x60) {
      process1060(outputBuffer, *outputBufferLen);
    }
    else {
      Serial.print(F("ERROR: Unknown command: 0x"));
      Serial.print(outputBuffer[7], HEX);
      Serial.println(outputBuffer[8], HEX);
      bitstream.show(outputBuffer, *outputBufferLen);
      return;
    }
  } 
  else if (0x24 == outputBuffer[0]) {
    // Sent in communication test mode of CM67z
    Serial.println("Communication test mode:");
    bitstream.show(outputBuffer, *outputBufferLen);
  }
  else {
    Serial.print(F("ERROR: Unknown first message header byte: 0x"));
    Serial.println(outputBuffer[0], HEX);
  }
}


/**
 * Verifies the checksum, which is the last byte of the manchester-decoded message.
 *
 * @param inputBuffer Points to the manchester-decoded message.
 * @param len length of the manchester-decoded message.
 *
 * @return 0 if checksum is ok.
 */
byte verifyChecksum(byte *inputBuffer, byte len) {

  byte checksum = 0;
  for (int i=0 ; i<len; i++) {
    checksum += inputBuffer[i];
  }

  return checksum; // Must be 0 after adding up all bytes.
}


void printDeviceAddress(byte *buffer, byte len) {
  for (int i=0 ; i<len ; i++) {
    if (buffer[i] < 0x10) Serial.print(F("0"));
    Serial.print(buffer[i], HEX);
  }
  Serial.print(F(" "));
}  

void processZoneSetpointSettingCommand(byte *buffer, byte len) {
  printDeviceAddress(buffer + 1, 6);

  Serial.print(F(" Zone Setpoint Setting Z="));
  Serial.print(buffer[10], HEX);
  Serial.print(F(" "));

  double temperature = (buffer[11] * 256.0 + buffer[12]) / 100.0;
  Serial.print(F(" T="));
  Serial.println(temperature, 2);
}

void processZoneTemperatureDistribution(byte *buffer, byte len) {
  printDeviceAddress(buffer + 1, 6);

  Serial.print(F(" Zone Temperature Distribution Z="));
  Serial.print(buffer[10], HEX);
  Serial.print(F(" "));

  double temperature = (buffer[11] * 256.0 + buffer[12]) / 100.0;
  Serial.print(F(" T="));
  Serial.println(temperature, 2);
}

void processHeatDemandTiming(byte *buffer, byte len) {
  printDeviceAddress(buffer + 1, 6);

  Serial.print(F(" Heat Demand Timing Related "));
  Serial.print(buffer[10], HEX);
  Serial.print(F(" "));
  Serial.print(buffer[11], HEX);
  Serial.print(F(" "));
  Serial.print(buffer[12], HEX);
  Serial.println(F(" "));
}

/**
 * Command 0x1FC9 "Bind"
 *
 * Device Types:
 * 0x2309 HR80
 * 0x30C9 Relais module R6660D 
 *
 * Possible outputs:
 * 32B44A  Bind Request#B6 zone 1 and device 239 for source device 32B44A / zone 1 and device 30C9 for source device 32B44A 
 * 32B44A  Bind Request#B6 zone 1 and device 239 for source device 32B44A / zone 1 and device 30C9 for source device 32B44A 
 * 32B44A  Bind Request#B6 zone 1 and device 239 for source device 32B44A / zone 1 and device 30C9 for source device 32B44A 
 *
 * 32B44A  Bind Request#B7 zone 1 and device 239 for source device 32B44A / zone 1 and device 30C9 for source device 32B44A 
 * 32B44A  Bind Request#B7 zone 1 and device 239 for source device 32B44A / zone 1 and device 30C9 for source device 32B44A 
 * 32B44A  Bind Request#B7 zone 1 and device 239 for source device 32B44A / zone 1 and device 30C9 for source device 32B44A 
 *
 * 32B44A  Bind Request#8D zone 2 and device 239 for source device 32B44A 
 * 32B44A  Bind Request#8D zone 2 and device 239 for source device 32B44A 
 * 32B44A  Bind Request#8D zone 2 and device 239 for source device 32B44A 
 * 32B44A  Bind Request#8E zone 2 and device 239 for source device 32B44A 
 * 32B44A  Bind Request#8E zone 2 and device 239 for source device 32B44A 
 * 32B44A  Bind Request#8E zone 2 and device 239 for source device 32B44A 
 * 32B44A  Bind Request#8F zone 2 and device 239 for source device 32B44A 
 * 32B44A  Bind Request#8F zone 2 and device 239 for source device 32B44A 
 * 32B44A  Bind Request#8F zone 2 and device 239 for source device 32B44A 
 * 32B44A  Bind Request#90 zone 2 and device 239 for source device 32B44A 
 * 32B44A  Bind Request#90 zone 2 and device 239 for source device 32B44A 
 * 32B44A  Bind Request#90 zone 2 and device 239 for source device 32B44A 
 *
 */
void processBindCommand(byte *buffer, byte len) {
  printDeviceAddress(buffer + 1, 3); // Device address of the sending device

  // Incremented each time the binding signal is triggered by the user by pressing
  // a button combination on the device.
  Serial.print(F(" Bind Request#")); 
  Serial.print(buffer[4], HEX);
  
  // byte[5:6] = 0x1FC9 for "bind" command
  
  Serial.print(F(" zone "));
  Serial.print(buffer[8], HEX);
  Serial.print(F(" and device "));
  Serial.print(buffer[9], HEX);
  Serial.print(buffer[10], HEX);
  Serial.print(F(" for source device "));
  Serial.print(buffer[11], HEX);
  Serial.print(buffer[12], HEX);
  Serial.print(buffer[13], HEX);

  if (buffer[7] == 0x0C) {
    Serial.print(F(" / zone "));
    Serial.print(buffer[14], HEX);
    Serial.print(F(" and device "));
    Serial.print(buffer[15], HEX);
    Serial.print(buffer[16], HEX);
    Serial.print(F(" for source device "));
    Serial.print(buffer[17], HEX);
    Serial.print(buffer[18], HEX);
    Serial.print(buffer[19], HEX);
  }

  Serial.println(F(" "));

  if (buffer[7] != 0x06 && buffer[7] != 0x0C) {
    Serial.println(F("There is something wrong with data length"));
  }


}

void process1100Command(byte *buffer, byte len) {
  printDeviceAddress(buffer + 1, 3);

  if (buffer[7] != 0x08) {
    Serial.println(F("There is something wrong with data length"));
  }

  Serial.print(F(" Command 0x1100 #"));
  Serial.print(buffer[4], HEX);
  Serial.print(F(" Data: "));
  Serial.print(buffer[8], HEX);
  Serial.print(F(" "));
  Serial.print(buffer[9], HEX);
  Serial.print(F(" "));
  Serial.print(buffer[10], HEX);
  Serial.print(F(" "));
  Serial.print(buffer[11], HEX);
  Serial.print(F(" "));
  Serial.print(buffer[12], HEX);
  Serial.print(F(" "));
  Serial.print(buffer[13], HEX);
  Serial.print(F(" "));
  Serial.print(buffer[14], HEX);
  Serial.print(F(" "));
  Serial.print(buffer[15], HEX);
  Serial.println(F(" "));
}

void process0008Command(byte *buffer, byte len) {
  printDeviceAddress(buffer + 1, 3);

  if (buffer[7] != 0x02) {
    Serial.println(F("There is something wrong with data length"));
  }

  Serial.print(F(" Command 0x0008 #"));
  Serial.print(buffer[4], HEX);
  Serial.print(F(" Data: "));
  Serial.print(buffer[8], HEX);
  Serial.print(F(" "));
  Serial.print(buffer[9], HEX);
  Serial.println(F(" "));
}

void process0009Command(byte *buffer, byte len) {
  printDeviceAddress(buffer + 1, 3);

  if (buffer[7] != 0x03) {
    Serial.println(F("There is something wrong with data length"));
  }

  Serial.print(F(" Command 0x0009 #"));
  Serial.print(buffer[4], HEX);
  Serial.print(F(" Data: "));
  Serial.print(buffer[8], HEX);
  Serial.print(F(" "));
  Serial.print(buffer[9], HEX);
  Serial.print(F(" "));
  Serial.print(buffer[10], HEX);
  Serial.println(F(" "));
}

void process1F09Command(byte *buffer, byte len) {
  printDeviceAddress(buffer + 1, 3);

  if (buffer[7] != 0x03) {
    Serial.println(F("There is something wrong with data length"));
  }

  Serial.print(F(" Command 0x1F09 #"));
  Serial.print(buffer[4], HEX);
  Serial.print(F(" Data: "));
  Serial.print(buffer[8], HEX);
  Serial.print(F(" "));
  Serial.print(buffer[9], HEX);
  Serial.print(F(" "));
  Serial.print(buffer[10], HEX);
  Serial.println(F(" "));
}

void process1060(byte *buffer, byte len) {
  printDeviceAddress(buffer + 1, 6);

  Serial.print(F(" Command 0x1016 Data: "));
  Serial.print(buffer[10], HEX);
  Serial.print(F(" "));
  Serial.print(buffer[11], HEX);
  Serial.print(F(" "));
  Serial.print(buffer[12], HEX);
  Serial.println(F(" "));
}










