/////////////////////////////////////////////////////////////////////////////////////////////
//  Portable Time Station.
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
#include <DCServices.h>

/////////////////////////////////////////////////////////////////////////////////////////////
//  Hardware.
//

#define PIN_TIMEMODULE_RX 2
#define PIN_TIMEMODULE_TX 3
#define PIN_YELLOW_LED 6
#define PIN_FIRST_RED_LED A0

//
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
//  EEPROM.
//

#define EEPROM_BOOT_FLAG 0
#define EEPROM_TIME_PROTOCOL 1

//
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
//  Globals.
//

#define TMMODULE_RX_BUF_SIZE 32
#define TMMODULE_TX_BUF_SIZE 32

SoftwareSerial tmSerial(PIN_TIMEMODULE_RX, PIN_TIMEMODULE_TX);
char tmModuleRXBuffer[TMMODULE_RX_BUF_SIZE];
char tmModuleTxBuffer[TMMODULE_RX_BUF_SIZE];
unsigned long setupStartTime = 0;

DCServices *dcServices;

//
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
// Send a command to the time module and optionally collect the response.

void speakToTmModule(const char *command, char *outputBuffer = NULL, uint8_t outputBufferSize = 0)
{
  // We need to speak really slowly. It seems the time moudule is bit-banging the serial reading
  // and has no buffer at all. It works great when typing from keyboard but not programmatically.
  do
  {
    tmSerial.print(*command ? *command : '\r');
    delay(50);
  } while (*command++ != 0);

  // All responses are in the format "label: xxxxx\n", where xxxx is the part we want.

  bool suppressResponse = true;
  while (tmSerial.available())
  {
    char c = tmSerial.read();

    if (outputBuffer != NULL && !suppressResponse)
    {
      if (c == '\r')
      {
        suppressResponse = true;
        continue;
      }

      *outputBuffer++ = c;
    }

    if (c == ' ')
    {
      suppressResponse = false;
    }
  }
}

void sendTmModuleCommand(const char *command)
{
  memset(tmModuleRXBuffer, 0, TMMODULE_RX_BUF_SIZE);

  speakToTmModule("=");
  speakToTmModule(command, tmModuleRXBuffer, TMMODULE_RX_BUF_SIZE);
  speakToTmModule("x");
}

//
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
// Attempt to get a time signal from from the DataCloud set the internal RTC.
// If the sync is successfull the yellow LED will be lit.

void syncWithTimeSignal()
{
  DateTime *dateTime = new DateTime();

  analogWrite(PIN_YELLOW_LED, 0);

  if (dcServices->receiveTimeBroadcast(dateTime))
  {
    char buffer[16];

    sprintf(buffer, "T%02d%02d%02d", dateTime->hour, dateTime->minute, dateTime->second);
    sendTmModuleCommand(buffer);

    sprintf(buffer, "D%02d%02d%02d", dateTime->day, dateTime->month, dateTime->year);
    sendTmModuleCommand(buffer);

    analogWrite(PIN_YELLOW_LED, 10);
  }

  delete dateTime;
}

//
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
// Show the currently selected time protocol on the LED map.
//

void showTimeProtocol()
{
  for (uint8_t ix = 0; ix < 4; ix++)
  {
    digitalWrite(PIN_FIRST_RED_LED + ix, ix == (EEPROM.read(EEPROM_TIME_PROTOCOL) - 1));
  }
}

//
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
// Initialize the time module for the current selected protocol.
// We always set the flags to TZ 0 and no DTS Region because we always carry current local
// time in the RTC.
//

void initTimeModule()
{
  char buf[8];
  sprintf(buf, "P%d", EEPROM.read(EEPROM_TIME_PROTOCOL));
  sendTmModuleCommand(buf);
  sendTmModuleCommand("Z0");
  sendTmModuleCommand("ROFF");
  sendTmModuleCommand("CON");
}

//
/////////////////////////////////////////////////////////////////////////////////////////////

void processBootFlag()
{
  if (setupStartTime == 0)
  {
    // If the boot flag is set we rebooted less than 10s after the last boot,
    // switch to the next time protocol.
    if (EEPROM.read(EEPROM_BOOT_FLAG) == 1)
    {
      EEPROM.write(EEPROM_TIME_PROTOCOL, 1 + (EEPROM.read(EEPROM_TIME_PROTOCOL) % 4));
    }

    // Write the boot flag, this will be cleared after 10s of activity.
    EEPROM.write(EEPROM_BOOT_FLAG, 1);

    setupStartTime = millis();

    return;
  }

  if (millis() - setupStartTime > 10000 && EEPROM.read(EEPROM_BOOT_FLAG) != 0)
  {
    EEPROM.write(EEPROM_BOOT_FLAG, 0);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Setup.

void setup()
{
  tmSerial.begin(9600);

  for (uint8_t ix = 0; ix < 4; ix++)
  {
    pinMode(PIN_FIRST_RED_LED + ix, OUTPUT);
    digitalWrite(PIN_FIRST_RED_LED + ix, LOW);
  }

  pinMode(PIN_YELLOW_LED, OUTPUT);
  digitalWrite(PIN_YELLOW_LED, LOW);

  dcServices = new DCServices(DC_RADIO_NRF24_V2);

  processBootFlag();
  showTimeProtocol();
  initTimeModule();
  syncWithTimeSignal(); 
}

//
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
// Main loop.

void loop()
{
  processBootFlag();  
}

//
/////////////////////////////////////////////////////////////////////////////////////////////
