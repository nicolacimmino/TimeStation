/////////////////////////////////////////////////////////////////////////////////////////////
//  Time Station.
//  Copyright (C) 2020 Nicola Cimmino
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//   This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see http://www.gnu.org/licenses/.
//
//
/////////////////////////////////////////////////////////////////////////////////////////////

#include <EEPROM.h>
#include <Arduino.h>
#include <SoftwareSerial.h>

/////////////////////////////////////////////////////////////////////////////////////////////
//  Hardware.
//

#define PIN_TIMEMODULE_RX 2
#define PIN_TIMEMODULE_TX 3

//
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
//  Globals.
//

SoftwareSerial tmSerial(PIN_TIMEMODULE_RX, PIN_TIMEMODULE_TX);

//
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
// Setup.



void setup()
{
  tmSerial.begin(9600);
  Serial.begin(9600);
}

//
/////////////////////////////////////////////////////////////////////////////////////////////

void printSlowly(const char *text)
{
  while (*text != 0)
  {
    tmSerial.print((char)*text);
    delay(100);
    text++;
  }
}
void setProtocol()
{
  printSlowly("\r=\rp4\rx\r\0");
  while (tmSerial.available())
  {
    (tmSerial.read());
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Main loop.

void loop()
{
  setProtocol();

  while (true)
  {
    while (Serial.available())
    {
      tmSerial.write(Serial.read());
    }

    while (tmSerial.available())
    {
      Serial.write(tmSerial.read());
    }
  }
}

//
/////////////////////////////////////////////////////////////////////////////////////////////
