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

#define PD2 2 // Wired with GDO0 on CC1101
#define SERIAL_DATA PD2
#define PD3 3 // Wired with GDO2 on CC1101
#define SERIAL_CLK PD3


// TODO: Remove
byte inputBuffer[0]; // Has to correspond to CC1101 PKTLEN Parameter
byte outputBuffer[0]; // Has to correspond to CC1101 PKTLEN Parameter


#define BUFFER_SIZE 100
byte buffer[BUFFER_SIZE];

void setup() {

  delay(500);

  CCx.PowerOnStartUp();
  setCCxConfig();

  Serial.begin(115200);

  Serial.println(F("-- HoneyComm - Copyright (C) 2011 Wladimir Komarow"));
  Serial.println(F("-- This program comes with ABSOLUTELY NO WARRANTY."));
  Serial.println(F("-- This is free software, and you are welcome to redistribute it under certain conditions; "));
  Serial.println(F("-- For details see http://www.gnu.org/licenses/gpl-3.0-standalone.html"));
  Serial.println(F("-- THIS DOES NOT WORK PROPERLY YET"));
  delay(500);

  pinMode(PD5, OUTPUT); 
  pinMode(PD6, OUTPUT);
}


//
// Main loop
//
void loop() {

  byte value = 0;
  byte cnt = 0;

  while(true) {

    // Receive the bit stream and look for 4-byte sync word
    syncSearch(0x55, 0x57, 0xfc, 0x01);

    // Now start to read the message from the bitstream.
    cnt = 0;
    do {
      value = receive();
      buffer[cnt++] = value;

    } 
    while (value != 0x35 && cnt < BUFFER_SIZE); // Marker for the end of the message, manchester-breaking.

    if (buffer[0] != 0x33 && buffer[1] != 0x55 && buffer[2] != 0x53) { 
      for (int i=0 ; i<3 ; i++) {
        Serial.print(buffer[i], HEX);
      }
      Serial.println();
    } else {
      for (int i=3 ; i<(cnt-1) ; i+=2) {
        manchesterByteStream.decodeByte(buffer+i, &value);
        Serial.print(value, HEX);
      }
      Serial.println();
    }
  }
}

void syncSearch(byte sync3, byte sync2, byte sync1, byte sync0) {

  byte sync0Received, sync1Received, sync2Received, sync3Received;
  byte SyncNOK;

  do {
    sync3Received = sync3Received << 1;

    if((sync2Received & 0x80) == 0)
      sync3Received &= 0xFE;
    else
      sync3Received |= 0x01;

    sync2Received = sync2Received << 1;

    if((sync1Received & 0x80) == 0)
      sync2Received &= 0xFE;
    else
      sync2Received |= 0x01;

    sync1Received = sync1Received << 1;

    if((sync0Received & 0x80) == 0)
      sync1Received &= 0xFE;
    else
      sync1Received |= 0x01;

    // Wait for raising edge of SERIAL_CLK
    while (! (PIND & (1<<SERIAL_CLK)));

    sync0Received = sync0Received << 1;

    if (PIND & (1<<SERIAL_DATA)) {
      sync0Received |= 0x01; 
    } 
    else {
      sync0Received &= 0xFE;
    }

    // Wait for falling edge of SERIAL_CLK
    while (PIND & (1<<SERIAL_CLK));

    SyncNOK = (
    (sync3Received != sync3) || 
      (sync2Received != sync2) || 
      (sync1Received != sync1) || 
      (sync0Received != sync0));

  } 
  while(SyncNOK);
}

/**
 * Receives a byte from the bit stream, 
 * getting rid of the start- and stop bits.
 */
byte receive() {

  byte value = 0;

  for (int bit=-1 ; bit<9 ; bit++) {

    // Wait for raising edge of SERIAL_CLK
    while (! (PIND & (1<<SERIAL_CLK)));

    // Ignore start- and stop-bit
    if (bit >= 0 && bit <= 7) {

      if (PIND & (1<<SERIAL_DATA)) {
        value |= 1<<bit;
      } 
    }

    // Wait for falling edge of SERIAL_CLK
    while (PIND & (1<<SERIAL_CLK));
  }

  return value;
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

  bitstream.show(outputBuffer, outputBufferLen);
  return;

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

/**
 * Command 0x2309 "Zone Setpoint Setting"
 * Sender: HR80
 *
 * With this message, the HR80 radiator controller broadcasts to which zone it belongs,
 * and what is the current setpoint setting for this zone.
 * Bytes 0-2 of the device address is the unique device address of the HR80.
 * Bytes 3-5 of the device address is the unique device address of the controller to which the HR80 is bound (and got its zone assignment) from.
 *
 * Possible outputs:
 * 00A20132B44A  Zone Setpoint Setting Z=1  T=17.00
 * 00A20132B44A  Zone Setpoint Setting Z=1  T=21.00
 *
 * Device address of the HR80: 00A201
 * Device address of the controller the HR80 is bound to: 32B44A
 * The setpoint for zone 1 (to which this HR80 is belonging to) changed from 17.00 to 21.00 degrees celsius.
 *
 */
void processZoneSetpointSettingCommand(byte *buffer, byte len) {
  printDeviceAddress(buffer + 1, 6);

  Serial.print(F(" Zone Setpoint Setting Z="));
  Serial.print(buffer[10], HEX);
  Serial.print(F(" "));

  double temperature = (buffer[11] * 256.0 + buffer[12]) / 100.0;
  Serial.print(F(" T="));
  Serial.println(temperature, 2);
}

/**
 * Command: 0x30C9 "Zone Temperature Distribution"
 * Sender: HR80
 *
 * With this message, the HR80 radiator controller broadcasts the actual temperature value.
 * The zone information always seems to be Z=0.
 *
 * Bytes 0-2 of the device address is the unique device address of the HR80.
 * Bytes 3-5 of the device address repeat the device address of the HR80.
 *
 * Possible outputs:
 * 00A20100A201  Zone Temperature Distribution Z=0  T=20.57
 *
 * Device address of the HR80: 00A201
 * The actual temperature value is 20.57 degrees celsius.
 *
 */
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
 * Sender: Controlling device
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
















