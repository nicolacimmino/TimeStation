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

#define TMMODULE_RX_BUF_SIZE 32
#define TMMODULE_TX_BUF_SIZE 32

SoftwareSerial tmSerial(PIN_TIMEMODULE_RX, PIN_TIMEMODULE_TX);
char tmModuleRXBuffer[TMMODULE_RX_BUF_SIZE];
char tmModuleTxBuffer[TMMODULE_RX_BUF_SIZE];

//
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
// Terminal.

#define TERMINAL_BAUDRATE 9600
#define TERMINAL_KEY_ESC 0x1B

//
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
// Setup.

void setup()
{
  tmSerial.begin(9600);
  Serial.begin(TERMINAL_BAUDRATE);
}

//
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
// Send a command to the time module and optionally collect the response.

void speakToTmModule(const char *command, char *outputBuffer = NULL, uint8_t outputBufferSize = 0)
{
  // We need to speak really slowly. It seems the time moudule is bit-banging the serial reading
  // and had no buffer at all. It works great when typing from keyboard but not programmatically.
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

void setProtocol(uint8_t protocol)
{
  sprintf(tmModuleTxBuffer, "p%i", protocol);
  sendTmModuleCommand(tmModuleTxBuffer);
  Serial.println(tmModuleRXBuffer);

  sendTmModuleCommand("z");
  Serial.println(tmModuleRXBuffer);
}

void enterDirectMode()
{
  while (true)
  {
    while (Serial.available())
    {
      if (Serial.peek() == TERMINAL_KEY_ESC)
      {
        Serial.read();
        return;
      }

      tmSerial.write(Serial.read());
    }

    while (tmSerial.available())
    {
      Serial.write(tmSerial.read());
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Main loop.

void loop()
{
  // setProtocol(3);
  if (Serial.available() && Serial.read() == TERMINAL_KEY_ESC)
  {
    enterDirectMode();
  }
}

//
/////////////////////////////////////////////////////////////////////////////////////////////
