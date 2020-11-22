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
#include "src/NRF24L01RadioDriver.h"

/////////////////////////////////////////////////////////////////////////////////////////////
//  Hardware.
//

#define PIN_TIMEMODULE_RX 2
#define PIN_TIMEMODULE_TX 3
#define RADIO_HW_VERSION 2
#define PIN_YELLOW_LED 6

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
NRF24L01RadioDriver *radio;

//
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
// Terminal/Radio Control.

#define TERMINAL_BAUDRATE 9600
#define TERMINAL_KEY_ESC 0x1B

#define RTX_EXTENDED_PREAMBLE 643234
#define CONTROL_RADIO_CHANNEL 85
#define TIME_BROADCAST_RADIO_CHANNEL 32

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
// Serve the serial inteface.
// All traffic is marshalled between the serial interface and the time module serial.
//

void serveSerialInterface()
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

//
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
// Serve radio interface.
// Commands sent on the radio interface are passed directly to the time module and the
//  response is sent back.
//

void serveRadioInterface()
{
  char buffer[64];
  uint8_t dataBufferSize = 64;

  memset(buffer, 0, dataBufferSize);

  radio->setRFChannel(CONTROL_RADIO_CHANNEL);
  if (radio->receive(buffer, &dataBufferSize, 500))
  {
    sendTmModuleCommand(buffer);
    radio->send(tmModuleRXBuffer, strlen(tmModuleRXBuffer));
  }
}

//
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
// Attempt to get a time signal from a BTS24 station and set the internal RTC.
// This call bloks until a valid time signal is received.
//

void syncWithTimeSignal()
{
  char buffer[32];
  uint8_t dataLen = 32;

  radio->setRFChannel(TIME_BROADCAST_RADIO_CHANNEL);

  while (true)
  {
    digitalWrite(PIN_YELLOW_LED, (millis() % 2000) < 1000);

    if (radio->receive(buffer, &dataLen, 1000))
    {
      uint8_t dateParts[6];

      char *token = strtok(buffer, ",");
      if (strcmp(*token, "^T"))
      {
        for (uint8_t ix = 0; ix < 6; ix++)
        {
          token = strtok(NULL, ",");
          dateParts[ix] = atoi(token);
        }

        sprintf(buffer, "T%02d%02d%02d", dateParts[0], dateParts[1], dateParts[2]);
        sendTmModuleCommand(buffer);

        sprintf(buffer, "D%02d%02d%02d", dateParts[5], dateParts[4], dateParts[3]);
        sendTmModuleCommand(buffer);

        digitalWrite(PIN_YELLOW_LED, HIGH);

        return;
      }
    }
  }
}

//
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
// Setup.

void setup()
{
  pinMode(PIN_YELLOW_LED, OUTPUT);
  digitalWrite(PIN_YELLOW_LED, LOW);

  tmSerial.begin(9600);
  Serial.begin(TERMINAL_BAUDRATE);

  radio = new NRF24L01RadioDriver(RADIO_HW_VERSION);
  radio->setRXExtendedPreamble(RTX_EXTENDED_PREAMBLE);
  radio->setTXExtendedPreamble(RTX_EXTENDED_PREAMBLE);
  radio->setTXPower(3);

  sendTmModuleCommand("coff");
  syncWithTimeSignal();
  sendTmModuleCommand("con");
}

//
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
// Main loop.

void loop()
{
  serveSerialInterface();
  serveRadioInterface();
}

//
/////////////////////////////////////////////////////////////////////////////////////////////
