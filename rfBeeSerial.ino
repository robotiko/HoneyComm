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

#include "rfBeeSerial.h"

int setMyAddress(byte myAddress){

   CCx.Write(CCx_ADDR, myAddress);
   return OK;
} 


int setPowerAmplifier(byte configIndex, byte paIndex){

   CCx.setPA(configIndex, paIndex);
   return OK;
}




int setCCxConfig(){
   // Configuration settings
   byte cfg = 0;
   CCx.Setup(cfg);
   //CCx.ReadSetup();
   // and restore the config settings
   setMyAddress(0);
   setPowerAmplifier(0, 0);


   // setRFBeeMode(RECEIVE_MODE);
   CCx.Strobe(CCx_SIDLE); // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable.
   delay(100);
   CCx.Strobe(CCx_SFRX); // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states.
   CCx.Strobe(CCx_SRX); // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.

   return OK;
}



int blablasetRFBeeMode(byte mode){

   // CCx_MCSM1 is configured to have TX and RX return to proper state on completion or timeout
   switch (mode)
   {
   case TRANSMIT_MODE:
      CCx.Strobe(CCx_SIDLE);
      delay(1);
      CCx.Write(CCx_MCSM1 ,   0x00 );//TXOFF_MODE->stay in IDLE
      CCx.Strobe(CCx_SFTX);
      break;
   case RECEIVE_MODE:
      CCx.Strobe(CCx_SIDLE);
      delay(1);
      CCx.Write(CCx_MCSM1 ,   0x0C );//RXOFF_MODE->stay in RX
      CCx.Strobe(CCx_SFRX);
      CCx.Strobe(CCx_SRX);
      break;

   default:
      break;
   }
   return OK;
}

// read data from CCx and write it to Serial based on the selected output format
void writeSerialData(){

   byte rxData[CCx_PACKT_LEN];
   byte len;
   byte srcAddress;
   byte destAddress;
   char rssi;
   byte lqi;
   int result;
   byte of;


   result=receiveData(rxData, &len, &srcAddress, &destAddress, (byte *)&rssi , &lqi);

   if (result == ERR) {
      Serial.println("Error");
      return;
   }

   if (result == NOTHING)
      return;

   Serial.print(len,DEC); // len is size of data EXCLUDING address bytes !
   Serial.print(',');
   // write source & destination
   Serial.print(srcAddress,DEC);
   Serial.print(',');
   Serial.print(destAddress,DEC);
   Serial.print(',');
   // write data
   Serial.write(rxData,len);
   Serial.print(',');
   // write rssi en lqi
   Serial.print((int)rssi);
   Serial.print(',');
   Serial.println(lqi,DEC);

}







