// NR24L01 Driver is part of LoP-RAN , provides control for NRF24L01+ chips.
// This is a custom development of a driver for the NRF24L01+ cips, we use
//  here this chip in particular ways that are not the common application
//  scenarios for them. This is needed as LoP-RAN must be deployable also
//  on different bands where radios with these capabilities are not avaialbe.
// We basically don't make use of any of the automatic retrasmission capabilities
//  since we need to strictly know radio times in order to meet the strict
//  real-time requirements of LoP-RAN TDMA. We also use NRF24L01 "Pipe addresses"
//  as a sort of extended preamble to prevent interslot data leackage.
//
// The code in this module draws some inspiration from the great RF24
//  library Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>, GNU
//  which served also as a good support during the first phases of tests
//  to get things up and running.
//
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

#ifndef __NRF24L01DRIVER_H__
#define __NRF24L01DRIVER_H__

#include "RadioDriver.h"
#include <SPI.h>
#include <EEPROM.h>

#define NRF24L01_SPI_BUFFER_SIZE 8

// HW V1
#define NRF24L01_CS_PIN_V1 10
#define NRF24L01_CE_PIN_V1 9

// HW V2
#define NRF24L01_CS_PIN_V2 8
#define NRF24L01_CE_PIN_V2 7

// Commands
#define NRF24L01_R_REGISTER 0x00
#define NRF24L01_W_REGISTER 0x20
#define NRF24L01_ACTIVATE 0x50
#define NRF24L01_R_RX_PAYLOAD 0x61
#define NRF24L01_W_TX_PAYLOAD 0xA0
#define NRF24L01_FLUSH_TX 0xE1
#define NRF24L01_FLUSH_RX 0xE2

// Register number is part of the W_REGISTER octet, use
//  this mask to combine the command and the register number.
#define REGISTER_MASK 0x1F

// Registers
#define NRF24L01_EN_AA 0x01

#define NRF24L01_CONFIG 0x00
#define NRF24L01_CONFIG_PRIM_RX 0
#define NRF24L01_EN_AA 0x01
#define NRF24L01_EN_RXADDR 0x02
#define NRF24L01_SETUP_AW 0x03
#define NRF24L01_RF_CH 0x05
#define NRF24L01_RF_SETUP 0x06
#define NRF24L01_STATUS 0x07
#define NRF24L01_STATUS_RX_DR 6
#define NRF24L01_OBSERVE_TX 0x08
#define NRF24L01_CD 0x09
#define NRF24L01_RX_ADDR_P0 0x0A
#define NRF24L01_RX_ADDR_P1 0x0B
#define NRF24L01_TX_ADDR 0x10
#define NRF24L01_RX_PW_P0 0x11
#define NRF24L01_RX_PW_P1 0x12
#define NRF24L01_FIFO_STATUS 0x17
#define NRF24L01_FIFO_STATUS_RX_EMPTY 1
#define NRF24L01_FIFO_STATUS_TX_FULL 5
#define NRF24L01_NOP 0xFF

#define NRF24L01_TX_POW_0dBm 3
#define NRF24L01_TX_POW_m6dBm 2
#define NRF24L01_TX_POW_m12dBm 1
#define NRF24L01_TX_POW_m18dBm 0
#define NRF24L01_TX_POW_INVALID 4

#define NRF24L01_PACKET_DATA_SIZE 16
#define NRF24L01_PACKET_HEADER_SIZE 2
#define NRF24L01_PACKET_HEADER_SEQ 0
#define NRF24L01_PACKET_HEADER_DATA_LEN 1

#define NRF24L01_PAYLOAD_SIZE NRF24L01_PACKET_HEADER_SIZE + NRF24L01_PACKET_DATA_SIZE

#define NRF24L01_EXTENDED_PREAMBLE_SIZE 5

class NRF24L01RadioDriver : public RadioDriver
{
private:
    uint8_t spi_buffer[NRF24L01_SPI_BUFFER_SIZE];
    byte csPin;
    byte cePin;

    void initializeRadio();
    void powerDownRadio();
    void powerUpRadio();
    void startReceiving();
    void stopReceiving();
    boolean isDataAvailable();
    uint8_t performSPITransaction(uint8_t command, uint8_t dataLength, char *buffer = NULL);
    uint8_t writeRegister(uint8_t address, uint64_t value, uint8_t registerSize = 1);
    uint8_t readRegister(uint8_t address);

public:
    NRF24L01RadioDriver(byte hardwareVersion);
    void setRXExtendedPreamble(uint64_t preamble);
    void setTXExtendedPreamble(uint64_t preamble);
    void setTXPower(uint8_t power);
    void setRFChannel(uint8_t channel);
    void send(char *buffer, int length);    
    bool receive(char *buffer, uint8_t *bufferDataSize, uint16_t timeoutMilliseconds);
};

#endif
