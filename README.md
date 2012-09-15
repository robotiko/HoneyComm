
  _   _                         ____                           
 | | | | ___  _ __   ___ _   _ / ___| ___  _ __ ___  _ __ ___  
 | |_| |/ _ \| '_ \ / _ \ | | | |    / _ \| '_ ` _ \| '_ ` _ \ 
 |  _  | (_) | | | |  __/ |_| | |___| (_) | | | | | | | | | | |
 |_| |_|\___/|_| |_|\___|\__, |\____|\___/|_| |_| |_|_| |_| |_|
                         |___/                                 

About the project
-----------------

The goal of this project is to provide an alternative firmware for
Seeed Studio's RFBee. 

Using this device connected to a PC or a Microcontroller, it is possible to
listen to the communication between a CM67z Central User Panel and one or more
HR80 radiator controllers. Further, using this alternative firmware, it is
possible to sent commands to the HR80 radiator controllers, e.g. to define
temperature settings for a particular "zone" in a building.

The RFBee is a RF module providing easy and flexible wireless data transmission 
between devices. It is based on a AVR Atmega168 working as a fully functional
Arduino connected via SPI to a TI CC1101 RF transceiver.

Find more information about the RFBee here: http://www.seeedstudio.com


How to contribute
-----------------

The firmware was created and tested in an environment with a CM67z Central User
Panel and several HR80 radiator controllers. Some of the messages received over
the air are well known, while the meaning of other messages remains unknown.

You can contribute by testing this software with other devices of the 
manufacturer of the devices named above and let us know what you found out, 
to fill the gaps in the meaning of some yet unknown messages, or to correct or
refine the meaning of messages we already know. Your help is appreciated.

