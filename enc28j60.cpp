/*
 * Copyright (c) 2019 Tobias Jaster
 *
 * Modified by Zoltan Hudak and Yann Charbon
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "cmsis.h"
#include "enc28j60.h"

#ifndef NULL
#define NULL    ((void*)0)
#endif
#define ENC28J60_DEBUG  0

#if ENC28J60_DEBUG
#define ENC28J60_DEBUG_PRINTF(...)  printf(__VA_ARGS__)
#else
#define ENC28J60_DEBUG_PRINTF(...)
#endif
/** Millisec timeout macros */
#define RESET_TIME_OUT_MS       10ms
#define REG_WRITE_TIME_OUT_MS   50ms
#define PHY_RESET_TIME_OUT_MS   100ms
#define INIT_FINISH_DELAY       2000ms

/**
 * \brief TX FIFO Size definitions
 *
 */
#define TX_STATUS_FIFO_SIZE_BYTES       512U    /*< fixed allocation in bytes */
#define TX_DATA_FIFO_SIZE_KBYTES_POS    8U
#define TX_DATA_FIFO_SIZE_KBYTES_MASK   0x0FU
#define KBYTES_TO_BYTES_MULTIPLIER      1024U

/**
 * \brief FIFO Info definitions
 *
 */
#define FIFO_USED_SPACE_MASK        0xFFFFU
#define DATA_FIFO_USED_SPACE_POS    0U
#define STATUS_FIFO_USED_SPACE_POS  16U

#define HW_CFG_REG_RX_FIFO_POS      10U
#define HW_CFG_REG_RX_FIFO_SIZE_ALL 8U
#define HW_CFG_REG_RX_FIFO_SIZE_MIN 2U          /*< Min Rx fifo size in KB */
#define HW_CFG_REG_RX_FIFO_SIZE_MAX 6U          /*< Max Rx fifo size in KB */
#define HW_CFG_REG_RX_FIFO_SIZE     5U          /*< Rx fifo size in KB */

/**
 * @brief
 * @note
 * @param
 * @retval
 */
ENC28J60::ENC28J60(PinName mosi, PinName miso, PinName sclk, PinName cs, uint32_t spi_frequency) :
    _spi(new SPI(mosi, miso, sclk)),
    _cs(cs),
    _bank(0),
    _ready(true),
    _next(ERXST_INI)
{
    init(spi_frequency);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
ENC28J60::ENC28J60(mbed::SPI* spi, PinName cs, uint32_t spi_frequency) :
    _spi(spi),
    _cs(cs),
    _bank(0),
    _ready(true),
    _next(ERXST_INI)
{
    init(spi_frequency);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60::init(uint32_t spi_frequency)
{
    // Initialize SPI interface
    _cs = 1;
    _spi->format(8, 0);         // 8bit, mode 0
    _spi->frequency(spi_frequency);

    // Wait SPI to become stable
    ThisThread::sleep_for(RESET_TIME_OUT_MS);

    // Perform system reset
    writeOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);

    // Check CLKRDY bit to see if reset is complete
    // while(!(readReg(ESTAT) & ESTAT_CLKRDY));
    // The CLKRDY does not work. See Rev. B4 Silicon Errata point.
    // Workaround: Just wait.
    ThisThread::sleep_for(RESET_TIME_OUT_MS);

#if ENC28J60_EVENT_HANDLING == ENC28J60_EVENT_HANDLING_IRQ
    // Initialize interruptions
    init_irqs();
    printf("ENC28J60 Interrupt inited\n");
#endif

    // Set pointers to receive buffer boundaries
    writeRegPair(ERXSTL, ERXST_INI);
    writeRegPair(ERXNDL, ERXND_INI);

    // Set receive pointer. Receive hardware will write data up to,
    // but not including the memory pointed to by ERXRDPT
    writeReg(ERXRDPTL, ERXST_INI);

    // All memory which is not used by the receive buffer is considered the transmission buffer.
    // No explicit action is required to initialize the transmission buffer.
    // TX buffer start.
    writeRegPair(ETXSTL, ETXST_INI);

    // TX buffer end at end of ethernet buffer memory.
    writeRegPair(ETXNDL, ETXND_INI);


    // We want the Ethernet transceiver to forward every packets that are received
    // to the stack. This is needed if the Mbed firmware runs a router-like code
    // such as Wi-SUN border-router. In this situation, packets that have a destination
    // MAC address that is different from the ENC28J60 MAC address may arrive on it. We
    // don't want them to be filtered out as these may be formwared to other nodes
    // within the network.
    //writeReg(ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_PMEN | ERXFCON_BCEN | ERXFCON_MCEN);
    writeReg(ERXFCON, 0);
    writeRegPair(EPMM0, 0x303f);
    writeRegPair(EPMCSL, 0xf7f9);

    // Enable MAC receive and bring MAC out of reset (writes 0x00 to MACON2)
    writeRegPair(MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);

    // Enable automatic padding to 60bytes and CRC operations
    writeOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN);

    // Set inter-frame gap (non-back-to-back)
    writeRegPair(MAIPGL, 0x0C12);

    // Set inter-frame gap (back-to-back)
    writeReg(MABBIPG, 0x12);

    // Set the maximum packet size which the controller will accept
    // Do not send packets longer than MAX_FRAMELEN:
    writeRegPair(MAMXFLL, MAX_FRAMELEN);

    // No loopback of transmitted frames
    phyWrite(PHCON2, PHCON2_HDLDIS);

    // Switch to bank 0
    _setBank(ECON1);

    // Enable interrutps
    writeOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE | EIE_PKTIE);

    // Enable packet reception
    writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);

    // Configure leds
    phyWrite(PHLCON, 0x476);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
enc28j60_error_t ENC28J60::getPacketInfo(packet_t* packet)
{
    enc28j60_error_t    ret;
    uint8_t             nextPacketAddrL;
    uint8_t             nextPacketAddrH;
    uint8_t             status[RX_STAT_LEN];

    if (!_ready)
        return ENC28J60_ERROR_LASTPACKET;

    // Check if a new packet has been received and buffered.
    // The Receive Packet Pending Interrupt Flag (EIR.PKTIF) does not reliably
    // report the status of pending packets. See Rev. B4 Silicon Errata point 6.
    // Workaround: Check EPKTCNT
    if (readReg(EPKTCNT) == 0)
        return ENC28J60_ERROR_NOPACKET;

    _ready = false;

    // Packet pointer in receive buffer is wrapped by hardware.
    packet->addr = _next;

    // Program the receive buffer read pointer to point to this packet.
    writeRegPair(ERDPTL, packet->addr);

    // Read the next packet address
    nextPacketAddrL = readOp(ENC28J60_READ_BUF_MEM, 0);
    nextPacketAddrH = readOp(ENC28J60_READ_BUF_MEM, 0);
    _next = (nextPacketAddrH << 8) | nextPacketAddrL;

    // Read the packet status vector bytes (see datasheet page 43)
    for (uint8_t i = 0; i < RX_STAT_LEN; i++) {
        status[i] = readOp(ENC28J60_READ_BUF_MEM, 0);
    }

    // Get payload length (see datasheet page 43)
    packet->payload.len = (status[1] << 8) | status[0];

    // Remove CRC bytes
    packet->payload.len -= RX_CRC_LEN;

    // Check CRC and symbol errors (see datasheet page 44, table 7-3):
    // The ERXFCON.CRCEN is set by default. Normally we should not
    // need to check this.
    // Bit 7 in byte 3 of receive status vectors indicates that the packet
    // had a valid CRC and no symbol errors.
    if ((status[2] & (1 << 7)) != 0) {
        ret = ENC28J60_ERROR_OK;
    }
    else {
        // Drop faulty packet:
        // Move the receive read pointer to the begin of the next packet.
        // This frees the memory we just read out.
        freeRxBuffer();
        ret = ENC28J60_ERROR_RECEIVE;
    }

    // Decrement the packet counter to indicate we are done with this packet.
    writeOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);

    return ret;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60::readPacket(packet_t* packet)
{
    // Read operation in receive buffer wraps the read pointer automatically.
    // To utilize this feature we program the receive buffer read pointer (ERDPTL)
    // to point to the begin of this packet (see datasheet page 43).
    // Then we read the next packet pointer bytes + packet status vector bytes
    // rather than calculating the payload position and advancing ERDPTL by software.
    writeRegPair(ERDPTL, packet->addr);

    // Advance the read pointer to the payload by reading bytes out of interest.
    for (uint8_t i = 0; i < (RX_NEXT_LEN + RX_STAT_LEN); i++)
        readOp(ENC28J60_READ_BUF_MEM, 0);

    // The receive buffer read pointer is now pointing to the correct place.
    // We can read packet payload bytes.
    readBuf(packet->payload.buf, packet->payload.len);
}

/**
 * @brief   Frees the memory occupied by last packet.
 * @note    Programs the Receive Pointer (ERXRDPT)to point to the next
 *          packet address. Receive hardware will write data up to,
 *          but not including the memory pointed to by ERXRDPT.
 * @param
 * @retval
 */
void ENC28J60::freeRxBuffer(void)
{
    // Compensate for the errata rev B7, point 11:
    // The receive hardware may corrupt the circular
    // receive buffer (including the Next Packet Pointer
    // and receive status vector fields) when an even value
    // is programmed into the ERXRDPTH:ERXRDPTL registers.
    // Workaround: Never write an even address!
    if ((_next - 1 < ERXST_INI) || (_next - 1 > ERXND_INI))
        writeRegPair(ERXRDPTL, ERXND_INI);
    else
        writeRegPair(ERXRDPTL, _next - 1);

    _ready = true;  // ready for next packet
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
enc28j60_error_t ENC28J60::loadPacketInTxBuffer(packet_t* packet)
{
    uint8_t             controlByte = 0;
    enc28j60_error_t    error = ENC28J60_ERROR_OK;
    uint16_t            packetLen = TX_CTRL_LEN + packet->payload.len + TX_STAT_LEN;

    if (packetLen > ETXND_INI - ETXST_INI) {
        error = ENC28J60_ERROR_FIFOFULL;
        return error;
    }

    // Reset TX logic
    writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
    writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
    writeOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXERIF | EIR_TXIF);

    setWritePrt(ETXST_INI, 0);

    //_tx_packet.payload.len = data_len;
    writeBuf(&controlByte, sizeof(controlByte));
    error = setWritePrt(ETXST_INI, sizeof(controlByte));
    writeBuf(packet->payload.buf, packet->payload.len);

    return error;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
enc28j60_error_t ENC28J60::softReset(void)
{
    writeOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);

    ThisThread::sleep_for(1ms);
    return ENC28J60_ERROR_OK;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60::setRxBufSize(uint32_t size_kb)
{
    if (size_kb >= HW_CFG_REG_RX_FIFO_SIZE_MIN && size_kb <= HW_CFG_REG_RX_FIFO_SIZE_MAX) {
        writeRegPair(ERXSTL, ERXST_INI);

        // set receive pointer address
        writeRegPair(ERXRDPTL, ERXST_INI);

        // RX end
        writeRegPair(ERXNDL, 0x1FFF - (size_kb << HW_CFG_REG_RX_FIFO_POS));
    }
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
enc28j60_error_t ENC28J60::resetPhy(void)
{
    enc28j60_error_t    error = ENC28J60_ERROR_OK;
    uint16_t            phcon1 = 0;
    error = phyRead(PHCON1, &phcon1);
    if (error)
        return ENC28J60_ERROR_TIMEOUT;
    error = phyWrite(PHCON1, (phcon1 | PHCON1_PRST));
    if (error)
        return ENC28J60_ERROR_TIMEOUT;
    ThisThread::sleep_for(PHY_RESET_TIME_OUT_MS);
    error = phyRead(PHCON1, &phcon1);
    if (error || (phcon1 & PHCON1_PRST) != 0) {
        return ENC28J60_ERROR_TIMEOUT;
    }

    return ENC28J60_ERROR_OK;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60::enableMacRecv(void)
{
    writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60::disableMacRecv(void)
{
    writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_RXEN);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
enc28j60_error_t ENC28J60::readMacAddr(char* mac)
{
    if (!mac) {
        return ENC28J60_ERROR_PARAM;
    }

    _setBank(MAADR0);

    mac[0] = readReg(MAADR5);
    mac[1] = readReg(MAADR4);
    mac[2] = readReg(MAADR3);
    mac[3] = readReg(MAADR2);
    mac[4] = readReg(MAADR1);
    mac[5] = readReg(MAADR0);

    return ENC28J60_ERROR_OK;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
enc28j60_error_t ENC28J60::writeMacAddr(char* mac)
{
    if (!mac) {
        return ENC28J60_ERROR_PARAM;
    }

    _setBank(MAADR0);

    writeReg(MAADR5, mac[0]);
    writeReg(MAADR4, mac[1]);
    writeReg(MAADR3, mac[2]);
    writeReg(MAADR2, mac[3]);
    writeReg(MAADR1, mac[4]);
    writeReg(MAADR0, mac[5]);

    return ENC28J60_ERROR_OK;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
enc28j60_error_t ENC28J60::setWritePrt(uint16_t position, uint16_t offset)
{
    uint32_t    start = position + offset > ETXND_INI ? position + offset - ETXND_INI + ETXST_INI : position + offset;

    writeRegPair(EWRPTL, start);

    return ENC28J60_ERROR_OK;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
enc28j60_error_t ENC28J60::transmitPacket(packet_t* packet)
{
    // Set Transmit Buffer Start pointer
    writeRegPair(ETXSTL, ETXST_INI);

    // Set Transmit Buffer End pointer
    writeRegPair(ETXNDL, ETXST_INI + TX_CTRL_LEN + packet->payload.len);

    // Enable transmittion
    writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);

    // Wait until transmission is completed
    //while (((readReg(ECON1) & ECON1_TXRTS) != 0) && (!_tx_error_occured)) { }

    uint16_t count = 0;
    while ((readReg(EIR) & (EIR_TXIF | EIR_TXERIF)) == 0 && ++count < 1000U);

    if (!(readReg(EIR) & EIR_TXERIF) && count < 1000U) {
        return ENC28J60_ERROR_OK;
    }

    return ENC28J60_ERROR_NEXTPACKET;

    //// Chek whether the transmission was successfull
    //if ((readReg(ESTAT) & ESTAT_TXABRT) == 0)
    //    return ENC28J60_ERROR_OK;
    //else
    //    return ENC28J60_ERROR_NEXTPACKET;
}


/**
 * @brief
 * @note
 * @param
 * @retval
 */
uint32_t ENC28J60::getRxBufFreeSpace(void)
{
    uint16_t    readPointer = getRecvPointer();
    uint16_t    writePointer = getWritePointer();
    uint32_t    freeSpace = 0;
    if (writePointer > readPointer) {
        freeSpace = (uint32_t) (ERXND_INI - ERXST_INI) - (writePointer - readPointer);
    }
    else
    if (writePointer == readPointer) {
        freeSpace = (ERXND_INI - ERXST_INI);
    }
    else {
        freeSpace = readPointer - writePointer - 1;
    }

    return freeSpace;
}

/**
 * @brief   Sets Ethernet buffer read pointer
 * @note    Points to a location in receive buffer to read from.
 * @param
 * @retval
 */
enc28j60_error_t ENC28J60::setRxBufReadPtr(uint16_t position)
{
    //
    // Wrap the start pointer of received data when greater than end of receive buffer
    if (position > ERXND_INI)
        position = ERXST_INI + (position - ERXND_INI - 1);

    writeRegPair(ERDPTL, position);

    return ENC28J60_ERROR_OK;
}

/**
 * @brief   Sets receive pointer.
 * @note    Receive hardware will write received data up to, but not including
 *          the memory pointed to by the receive pointer.
 * @param
 * @retval
 */
uint16_t ENC28J60::getRecvPointer(void)
{
    return readRegPair(ERXRDPTL);
}

/**
 * @brief   Gets receive buffer's write pointer.
 * @note    Location within the receive buffer where the hardware will write bytes that it receives.
 *          The pointer is read-only and is automatically updated by the hardware whenever
 *          a new packet is successfully received.
 * @param
 * @retval
 */
uint16_t ENC28J60::getWritePointer(void)
{
    uint16_t    count_pre = readReg(EPKTCNT);
    uint16_t    writePointer = readRegPair(ERXWRPTL);
    uint16_t    count_post = readReg(EPKTCNT);
    while (count_pre != count_post) {
        count_pre = count_post;
        writePointer = readRegPair(ERXWRPTL);
        count_post = readReg(EPKTCNT);
    }

    ENC28J60_DEBUG_PRINTF("[ENC28J60] rx_getWritePointer: %d", writePointer);
    return writePointer;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60::readBuf(uint8_t* data, uint16_t len)
{
    _read(ENC28J60_READ_BUF_MEM, data, len, false);
    data[len] = '\0';
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60::writeBuf(uint8_t* data, uint16_t len)
{
    _write(ENC28J60_WRITE_BUF_MEM, data, len, false);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
uint8_t ENC28J60::readReg(uint8_t address)
{
    _setBank(address);

    // do the read
    return readOp(ENC28J60_READ_CTRL_REG, address);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
uint16_t ENC28J60::readRegPair(uint8_t address)
{
    uint16_t    temp;

    _setBank(address);

    // do the read
    temp = (uint16_t) (readOp(ENC28J60_READ_CTRL_REG, address + 1)) << 8;
    temp |= readOp(ENC28J60_READ_CTRL_REG, address);

    return temp;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60::writeReg(uint8_t address, uint8_t data)
{
    _setBank(address);

    // do the write
    writeOp(ENC28J60_WRITE_CTRL_REG, address, data);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60::writeRegPair(uint8_t address, uint16_t data)
{
    _setBank(address);

    // do the write
    writeOp(ENC28J60_WRITE_CTRL_REG, address, (data & 0xFF));
    writeOp(ENC28J60_WRITE_CTRL_REG, address + 1, (data) >> 8);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
enc28j60_error_t ENC28J60::phyRead(uint8_t address, uint16_t* data)
{
    uint8_t timeout = 0;
    writeReg(MIREGADR, address);
    writeReg(MICMD, MICMD_MIIRD);

    // wait until the PHY read completes
    while (readReg(MISTAT) & MISTAT_BUSY) {
        wait_us(15);
        timeout++;
        if (timeout > 10)
            return ENC28J60_ERROR_TIMEOUT;
    }   //and MIRDH

    writeReg(MICMD, 0);
    *data = (readReg(MIRDL) | readReg(MIRDH) << 8);
    return ENC28J60_ERROR_OK;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
enc28j60_error_t ENC28J60::phyWrite(uint8_t address, uint16_t data)
{
    uint8_t timeout = 0;
    // set the PHY register address

    writeReg(MIREGADR, address);

    // write the PHY data
    writeRegPair(MIWRL, data);

    // wait until the PHY write completes
    while (readReg(MISTAT) & MISTAT_BUSY) {
        wait_us(15);
        timeout++;
        if (timeout > 10)
            return ENC28J60_ERROR_TIMEOUT;
    }

    return ENC28J60_ERROR_OK;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
bool ENC28J60::linkStatus(void)
{
    uint16_t    data;
    phyRead(PHSTAT2, &data);
    return(data & 0x0400) > 0;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
uint8_t ENC28J60::readOp(uint8_t op, uint8_t address)
{
    uint8_t result;
    uint8_t data[2];

    // issue read command

    if (address & 0x80) {
        _read((op | (address & ADDR_MASK)), &data[0], 2, false);
        result = data[1];
        return result;
    }
    else {
        _read((op | (address & ADDR_MASK)), &data[0], 1, false);
    }

    result = data[0];
    return result;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60::writeOp(uint8_t op, uint8_t address, uint8_t data)
{
    // issue write command

    _write(op | (address & ADDR_MASK), &data, 1, false);
}

#if ENC28J60_EVENT_HANDLING == ENC28J60_EVENT_HANDLING_IRQ
void ENC28J60::init_irqs(void) {
    disable_all_interrupts();
    clear_all_interrupts();

    enable_interrupt(ENC28J60_INTERRUPT_RX_PENDING_ENABLE);
    //enable_interrupt(ENC28J60_INTERRUPT_LINK_STATE_ENABLE);
    //phyWrite(PHIE, PHIE_PLNKIE | PHIE_PGEIE);
    //enable_interrupt(ENC28J60_INTERRUPT_TX_ENABLE);
    //enable_interrupt(ENC28J60_INTERRUPT_TX_ERROR_ENABLE);
    enable_interrupt(ENC28J60_INTERRUPT_RX_ERROR_ENABLE);
}

void ENC28J60::enable_interrupt(enc28j60_interrupt_source source) {
	writeOp(ENC28J60_BIT_FIELD_SET, EIE, source);
}

void ENC28J60::disable_interrupt(enc28j60_interrupt_source source) {
	writeOp(ENC28J60_BIT_FIELD_CLR, EIE, source);
}

void ENC28J60::disable_all_interrupts(void) {
	writeReg(EIE, 0x00);
}

void ENC28J60::clear_interrupt(enc28j60_interrupt_source source) {
	writeOp(ENC28J60_BIT_FIELD_CLR, EIR, source);
}

void ENC28J60::clear_all_interrupts(void) {
    writeReg(EIR, 0x00);
}

uint8_t ENC28J60::get_interrupts(void) {
	return readOp(ENC28J60_READ_CTRL_REG, EIR);
}

bool ENC28J60::get_interrupt(enc28j60_interrupt_source source) {
    return (get_interrupts()&source)>0 ? true : false;
}
#endif

/**
 * @brief Reset the RX components
*/
void ENC28J60::reset_rx_logic() {
    writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_RXEN);
    writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXRST);
    writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_RXRST);
    writeRegPair(ERXSTL, ERXST_INI);
    writeRegPair(ERXNDL, ERXND_INI);
    writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, EIR_RXERIF);
    writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60::_setBank(uint8_t address)
{
    if ((address & BANK_MASK) != _bank) {
        writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1 | ECON1_BSEL0));
        writeOp(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK) >> 5);
        _bank = (address & BANK_MASK);
    }
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60::_read(uint8_t cmd, uint8_t* buf, uint16_t len, bool blocking)
{
#ifndef ENC28J60_READWRITE
    _SPIMutex.lock();
    _cs = 0;

    // issue read command
    _spi->write((int)cmd);
    while (len) {
        len--;

        // read data
        *buf = _spi->write(0x00);
        buf++;
    }

    _cs = 1;
    _SPIMutex.unlock();
#else
    uint8_t*    dummy = NULL;
    _readwrite(cmd, buf, dummy, len, blocking);
#endif
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60::_write(uint8_t cmd, uint8_t* buf, uint16_t len, bool blocking)
{
#ifndef ENC28J60_READWRITE
    _SPIMutex.lock();
    _cs = 0;

    // issue read command
    _spi->write((int)cmd);
    while (len) {
        len--;

        // read data
        _spi->write((int) *buf);
        buf++;
    }

    _cs = 1;
    _SPIMutex.unlock();
#else
    uint8_t*    dummy = NULL;
    _readwrite(cmd, dummy, buf, len, blocking);
#endif
}

#ifdef ENC28J60_READWRITE

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60::_readwrite(uint8_t cmd, uint8_t* readbuf, uint8_t* writebuf, uint16_t len, bool blocking)
{
    _SPIMutex.lock();
    _cs = 0;

    // issue read command
    _spi->write((int)cmd);
    while (len) {
        len--;

        if (readbuf == NULL) {
            _spi->write((int) *writebuf);
            writebuf++;
        }
        else
        if (writebuf == NULL) {
            *readbuf = _spi->write(0x00);
            readbuf++;
        }
        else {
            break;
        }
    }

    _cs = 1;
    _SPIMutex.unlock();
}
#endif
