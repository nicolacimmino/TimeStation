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
#include "NRF24L01RadioDriver.h"

/**********************************************************************************************
 * 
 * 
 */
NRF24L01RadioDriver::NRF24L01RadioDriver(byte hardwareVersion)
{
    switch (hardwareVersion)
    {
    case 1:
        this->csPin = NRF24L01_CS_PIN_V1;
        this->cePin = NRF24L01_CE_PIN_V1;
        break;
    case 2:
        this->csPin = NRF24L01_CS_PIN_V2;
        this->cePin = NRF24L01_CE_PIN_V2;
        break;
    }

    this->initializeRadio();
}
/*
/**********************************************************************************************/

/**********************************************************************************************
 * Sets the extended preamble for TX.
 *
 * In NRF24L01 we implement this using the pipe address feature, which does exactly the same
 *  that is add extra bytes after the preamble that should match for a packet to be accepted. 
 */
void NRF24L01RadioDriver::setTXExtendedPreamble(uint64_t preamble)
{
    this->writeRegister(NRF24L01_TX_ADDR, preamble, NRF24L01_EXTENDED_PREAMBLE_SIZE);
}
/*
/**********************************************************************************************/

/**********************************************************************************************
 * Sets the extended preamble for RX.
 *
 * In NRF24L01 we implement this using the pipe address feature, which does exactly the same
 *  that is add extra bytes after the preamble that should match for a packet to be accepted. 
 */
void NRF24L01RadioDriver::setRXExtendedPreamble(uint64_t preamble)
{
    this->writeRegister(NRF24L01_RX_ADDR_P1, preamble, NRF24L01_EXTENDED_PREAMBLE_SIZE);
}
/*
/**********************************************************************************************/

/**********************************************************************************************
 * Sets the transmit power.
 *
 * In NRF24L01 this is in bits 2-1 of RF_SETUP.
 */
void NRF24L01RadioDriver::setTXPower(uint8_t power)
{
    // TODO: power needs to be a generic value and we convert here to HW specific.
    this->writeRegister(NRF24L01_RF_SETUP, (this->readRegister(NRF24L01_RF_SETUP) & 0b11111001) | ((power & 0b11) << 1));
}
/*
/**********************************************************************************************/

/**********************************************************************************************
 * Sets the RF channel.
 *
 */
void NRF24L01RadioDriver::setRFChannel(uint8_t channel)
{
    this->writeRegister(NRF24L01_RF_CH, channel & 0b01111111);
}
/*
/**********************************************************************************************/

/**********************************************************************************************
 * Sends the supplied buffer.
 */
void NRF24L01RadioDriver::send(char *buffer, int dataLength)
{
    // We can assume radio is in Standby-I when this is invoked as only receive sets
    // it other modes and it does enter Standby-I when done.

    // Flush the TX buffer.
    this->performSPITransaction(NRF24L01_FLUSH_TX, 0);

    // Set TX mode (bit 0 of CONFIG goes to zero)
    // Since we are in Standby-I we need only 130uS for TX to settle.
    this->writeRegister(NRF24L01_CONFIG, (this->readRegister(NRF24L01_CONFIG) & ~_BV(NRF24L01_CONFIG_PRIM_RX)));
    digitalWrite(this->cePin, HIGH);

    // We are here in Standby-II and will move to TX mode as soon as data is pushed to the TX FIFO.

    uint8_t packet[NRF24L01_PAYLOAD_SIZE];

    unsigned long last_pll_lock = millis();

    uint8_t seq = 0;

    for (uint16_t ix = 0; ix < dataLength; ix += NRF24L01_PACKET_DATA_SIZE)
    {
        memcpy(packet + NRF24L01_PACKET_HEADER_SIZE, buffer + ix, NRF24L01_PACKET_DATA_SIZE);
        seq++;
        packet[NRF24L01_PACKET_HEADER_SEQ] = seq;
        packet[NRF24L01_PACKET_HEADER_DATA_LEN] = dataLength;

        this->performSPITransaction(NRF24L01_W_TX_PAYLOAD, NRF24L01_PAYLOAD_SIZE, packet);

        // Wait to push more data if we have filled the FIFO
        while (this->readRegister(NRF24L01_FIFO_STATUS) & _BV(NRF24L01_FIFO_STATUS_TX_FULL))
        {
            delayMicroseconds(100);
        }

        // The PLL during TX mode runs in open loop mode, we cannot stay here more than 4mS as the
        //  oscillator might drift too much. If we go over 4mS we need to enter Standby-II mode
        //  and go back to TX mode in order for the oscillator to lock again. To go to Standby-II
        //  here we pull CE down and wait the TX FIFO to be empty.
        if (millis() - last_pll_lock > 4)
        {         
            // Wait for the FIFO to be empty, so we can enter Standby-II
            // Bit 4 is TX_FIFO_EMPTY
            while (!(this->readRegister(NRF24L01_FIFO_STATUS) & 0b00010000))
            {
                delayMicroseconds(100);
            }            
        }
    }

    // Wait for the FIFO to be empty, so we can enter Standby-II
    while (!(this->readRegister(NRF24L01_FIFO_STATUS) & 0b00010000))
    {
        delayMicroseconds(100);
    }

    // Back to Standby-I
    this->writeRegister(NRF24L01_CONFIG, (this->readRegister(NRF24L01_CONFIG) & 0b11111110));
    digitalWrite(this->cePin, LOW);
}
/*
/**********************************************************************************************/

/**********************************************************************************************
 * Receives.
 *
 */
bool NRF24L01RadioDriver::receive(char *buffer, uint8_t *bufferDataSize, uint16_t timeoutMilliseconds)
{
    unsigned long startTime = millis();
    uint8_t receivedBytes = 0;
    uint8_t expctedSeq = 0;
    uint8_t packet[NRF24L01_PAYLOAD_SIZE];

    this->startReceiving();

    while (true)
    {
        while (!this->isDataAvailable())
        {
            if (millis() - startTime > timeoutMilliseconds)
            {
                this->stopReceiving();
                this->errTOUT++;

                return false;
            }
        }

        expctedSeq++;

        this->performSPITransaction(NRF24L01_R_RX_PAYLOAD, NRF24L01_PAYLOAD_SIZE, packet);

        if (packet[NRF24L01_PACKET_HEADER_SEQ] != expctedSeq)
        {
            this->stopReceiving();
            this->errOOS++;

            return false;
        }

        if (packet[NRF24L01_PACKET_HEADER_DATA_LEN] > *bufferDataSize)
        {
            this->stopReceiving();
            this->errOVFL++;

            return false;
        }

        memcpy(buffer + receivedBytes, packet + NRF24L01_PACKET_HEADER_SIZE, NRF24L01_PACKET_DATA_SIZE);

        receivedBytes += NRF24L01_PACKET_DATA_SIZE;

        if (receivedBytes >= packet[NRF24L01_PACKET_HEADER_DATA_LEN])
        {
            *bufferDataSize = packet[NRF24L01_PACKET_HEADER_DATA_LEN];
            this->stopReceiving();
            return true;
        }
    }
}
/*
/**********************************************************************************************/

/**********************************************************************************************
 * Writes a command plus dataLength bytes to SPI, gets the SPI response and the status reg.
 *
 * @param command       SPI command.
 * @param dataLength    Byes in buffer to send.
 * @param buffer        If provided data is read/written to this buffer, if not the spiBuffer 
 *                      will be used. Max size in this case is NRF24L01_SPI_BUFFER_SIZE. 
 * 
 * @return              SPI Status register.
 * 
 * Note: The command overrides the buffer with the SPI response.
 * 
 */
uint8_t NRF24L01RadioDriver::performSPITransaction(uint8_t command, uint8_t dataLength, char *buffer = NULL)
{
    if (buffer == NULL)
    {
        buffer = spi_buffer;
    }

    // CS low before we can talk.
    digitalWrite(this->csPin, LOW);

    // First we send the command. We also get out the status register.
    uint8_t status = SPI.transfer(command);

    // Then the whole buffer.
    if (dataLength > 0)
    {
        for (uint8_t ix = 0; ix < dataLength; ix++)
        {
            buffer[ix] = SPI.transfer(buffer[ix]);
        }
    }

    // Release the SPI bus.
    digitalWrite(this->csPin, HIGH);

    return status;
}
/*
/**********************************************************************************************/

/**********************************************************************************************
 * Brings up the SPI interface and makes basic settings for to the radio.
 *
 */
void NRF24L01RadioDriver::initializeRadio()
{
    // Initialize SPI related pins.
    pinMode(this->cePin, OUTPUT);
    pinMode(this->csPin, OUTPUT);
    pinMode(SCK, OUTPUT);  // TODO: shouldn't be needed.
    pinMode(MOSI, OUTPUT); // TODO: shouldn't be needed.
    digitalWrite(this->csPin, HIGH);
    digitalWrite(this->cePin, LOW);

    // NRF24L01 PON (Power On Reset) is 10.3mS
    //  according to datasheet. We wait a bit more
    //  just to be on the safe side.
    delay(15);

    // We setup SPI with mode 0, MSB first and use SPI CLOCK
    //  div by 2 to get a 4MHz SPI clock on a 16MHz xtal).
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV2);

    // Disable Auto ACK.
    this->writeRegister(NRF24L01_EN_AA, 0);

    // Set correct payload size.
    this->writeRegister(NRF24L01_RX_PW_P1, NRF24L01_PAYLOAD_SIZE);

    // Enable only pipe1 RX (we don't need zero cause we don't use ACK).
    this->writeRegister(NRF24L01_EN_RXADDR, 0b00000010);

    this->powerUpRadio();

    this->performSPITransaction(NRF24L01_FLUSH_TX, 0);
    this->performSPITransaction(NRF24L01_FLUSH_RX, 0);
}
/*
/**********************************************************************************************/

/**********************************************************************************************
 * Enters receive mode.
 * 
 * Radio must be already in Standby-I when this is invoked.
 *
 */
void NRF24L01RadioDriver::startReceiving()
{
    // Flush RX FIFO and clear RX_DR flag (DataReady).
    uint8_t status = performSPITransaction(NRF24L01_FLUSH_RX, 0);
    this->writeRegister(NRF24L01_STATUS, status | 0b01000000);

    // Set RX mode (bit 0 of CONFIG goes to one)
    // Since we are in Standby-I we need only 130uS for RX to settle.
    this->writeRegister(NRF24L01_CONFIG, (this->readRegister(NRF24L01_CONFIG) | 0b00000001));
    digitalWrite(this->cePin, HIGH);
}
/*
/**********************************************************************************************/

/**********************************************************************************************
 * Terminates receive mode.
 * 
 * Radio will be back to Standby-I when this is done.
 *
 */
void NRF24L01RadioDriver::stopReceiving()
{
    digitalWrite(this->cePin, LOW);
}
/*
/**********************************************************************************************/

/**********************************************************************************************
 * Checks if any data is available in the RX buffer.
 * 
 * Calling this function a second time will return false unless new data has been received.
 *
 * @return      true if data is available to be read.
 */
boolean NRF24L01RadioDriver::isDataAvailable()
{
    // RX_DR Data Ready flag is on bit 6 of STATUS
    // We read status by just sending a NOP as it is faster than the register read command.
    uint8_t status = performSPITransaction(NRF24L01_NOP, 0);
    uint8_t fifoStatus = this->readRegister(NRF24L01_FIFO_STATUS);

    // There is data only if Data Ready and not RX Empty
    if ((status & _BV(NRF24L01_STATUS_RX_DR)) && !(fifoStatus & _BV(NRF24L01_FIFO_STATUS_RX_EMPTY)))
    {
        // Reset Data Ready.
        this->writeRegister(NRF24L01_STATUS, status | _BV(NRF24L01_STATUS_RX_DR));

        return true;
    }

    return false;
}
/*
/**********************************************************************************************/

/**********************************************************************************************
 * Powers down the radio, the SPI interface is still active.
 *
 */
void NRF24L01RadioDriver::powerDownRadio()
{
    // Power is controlled by bit 1 of CONFIG
    this->writeRegister(NRF24L01_CONFIG, (this->readRegister(NRF24L01_CONFIG) & 0b11111101));
}
/*
/**********************************************************************************************/

/**********************************************************************************************
 * Powers up the radio.
 *
 */
void NRF24L01RadioDriver::powerUpRadio()
{
    // Power is controlled by bit 1 of CONFIG
    this->writeRegister(NRF24L01_CONFIG, (this->readRegister(NRF24L01_CONFIG) | 0b00000010));
}
/*
/**********************************************************************************************/

/**********************************************************************************************
 * Writes a multiple bytes register.
 *
 * @param address       Register address.
 * @param value         Register value.
 * @param registerSize  Register size in bytes, if omitted single byte is assumed.
 * 
 * @return              Status register.
 */
uint8_t NRF24L01RadioDriver::writeRegister(uint8_t address, uint64_t value, uint8_t registerSize = 1)
{
    // Write out the value, LSB first.
    for (int ix = 0; ix < registerSize; ix++)
    {
        spi_buffer[ix] = value & 0xFF;
        value = value >> 8;
    }

    return performSPITransaction(NRF24L01_W_REGISTER | (address & REGISTER_MASK), registerSize);
}
/*
/**********************************************************************************************/

/**********************************************************************************************
 * Reads a single byte register.
 *
 * @param address       Register address.
 *
 * @return              Register value.
 * 
 */
uint8_t NRF24L01RadioDriver::readRegister(uint8_t address)
{
    performSPITransaction(NRF24L01_R_REGISTER | (address & REGISTER_MASK), 1);
    return spi_buffer[0];
}
/*
/**********************************************************************************************/
