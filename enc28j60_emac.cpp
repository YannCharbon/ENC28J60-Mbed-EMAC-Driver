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
#include "mbed_interface.h"
#include "mbed_wait_api.h"
#include "mbed_assert.h"
#include "netsocket/nsapi_types.h"
#include "mbed_shared_queues.h"
#include "enc28j60_emac.h"

#if ENC28J60_EVENT_HANDLING == ENC28J60_EVENT_HANDLING_TIMER

ENC28J60_EMAC::ENC28J60_EMAC() :
    _enc28j60(new ENC28J60(ENC28J60_MOSI, ENC28J60_MISO, ENC28J60_SCK, ENC28J60_CS, ENC28J60_SPI_FREQ)),
    _prev_link_status_up(PHY_STATE_LINK_DOWN),
    _link_status_task_handle(0),
    _receive_task_handle(0),
    _memory_manager(NULL)
{ }

#else

ENC28J60_EMAC::ENC28J60_EMAC() :
    _enc28j60(new ENC28J60(ENC28J60_MOSI, ENC28J60_MISO, ENC28J60_SCK, ENC28J60_CS, ENC28J60_SPI_FREQ)),
    _prev_link_status_up(PHY_STATE_LINK_DOWN),
    _int(ENC28J60_INT),
    _memory_manager(NULL)
{
    _irq_thread = new Thread(IRQ_THREAD_PRIORITY,(uint32_t)IRQ_THREAD_STACKSIZE);
	_rx_thread = new Thread(RX_THREAD_PRIORITY,(uint32_t)RX_THREAD_STACKSIZE);
    _int.fall(callback(this, &ENC28J60_EMAC::Interrupt_Handler));
}

/** \brief Ethernet receive interrupt handler
 *
 *  This function handles the receive interrupt.
 */
void ENC28J60_EMAC::Interrupt_Handler(void) {
	_irq_thread->flags_set(FLAG_IRQ);
}

#endif

/**
 * @brief
 * @note    Allocates buffer chain from a pool and filles it with incoming packet.
 * @param
 * @retval  Data from incoming packet
 */
emac_mem_buf_t* ENC28J60_EMAC::low_level_input()
{
    emac_mem_buf_t*     chain;
    emac_mem_buf_t*     buf;
    packet_t            packet;

    if (_enc28j60->getPacketInfo(&packet) != ENC28J60_ERROR_OK) {
        return NULL;
    }

    if (packet.payload.len == 0) {
        return NULL;
    }

    // Allocate a buffer chain from the memory pool.
    chain = _memory_manager->alloc_pool(packet.payload.len, ENC28J60_BUFF_ALIGNMENT);
    buf = chain;

    // Iterate through the buffer chain and fill it with packet payload.
    while (buf != NULL) {
        packet.payload.buf = (uint8_t*)_memory_manager->get_ptr(buf);
        packet.payload.len = (uint16_t) (_memory_manager->get_len(buf));

        // Fill the ethernet stack buffer with packet payload received by ENC28J60.
        _enc28j60->readPacket(&packet);

        buf = _memory_manager->get_next(buf);
    }

    // Return the buffer chain filled with packet payload.
    return chain;
}

#if ENC28J60_EVENT_HANDLING == ENC28J60_EVENT_HANDLING_TIMER

/**
 * @brief   Receive task.
 * @note    Passes packet payload received by ENC28J60 to the ethernet stack
 * @param
 * @retval
 */
void ENC28J60_EMAC::receive_task()
{
    emac_mem_buf_t*     payload;

    _ethLockMutex.lock();
    while (_enc28j60->readReg(EPKTCNT) > 0) {
        payload = low_level_input();
        if (payload != NULL) {
            if (_emac_link_input_cb) {
                //printf("ETH packet input\n");
                _emac_link_input_cb(payload);   // pass packet payload to the ethernet stack
            }
        }
        _enc28j60->freeRxBuffer();  // make room in ENC28J60 receive buffer for new packets
    }

    _ethLockMutex.unlock();
}

#else

/** \brief  Receiver thread.
 *
 * Woken by thread flags to receive packets or clean up transmit
 *
 *  \param[in] params pointer to the interface data
 */
void ENC28J60_EMAC::receiver_thread_function(void* params) {
    ENC28J60_EMAC *enc28j60_enet = static_cast<ENC28J60_EMAC *>(params);
    emac_mem_buf_t*     payload;

    while(1) {
        uint32_t flags = ThisThread::flags_wait_any(FLAG_RX);

        if (flags & FLAG_RX) {
            enc28j60_enet->_ethLockMutex.lock();
            while (enc28j60_enet->_enc28j60->readReg(EPKTCNT) > 0) {
                payload = enc28j60_enet->low_level_input();
                if (payload != NULL) {
                    if (enc28j60_enet->_emac_link_input_cb) {
                        enc28j60_enet->_emac_link_input_cb(payload);   // pass packet payload to the ethernet stack
                        //printf("ETH packet input\n");
                    }
                }
                enc28j60_enet->_enc28j60->freeRxBuffer();  // make room in ENC28J60 receive buffer for new packets
            }

            enc28j60_enet->_ethLockMutex.unlock();
        }
        enc28j60_enet->_enc28j60->enable_interrupt(ENC28J60_INTERRUPT_ENABLE);
    }
}

void ENC28J60_EMAC::interrupt_thread_function(void* params) {

	ENC28J60_EMAC *enc28j60_enet = static_cast<ENC28J60_EMAC *>(params);
	volatile uint32_t flags;
	uint8_t estat = 0;
	uint8_t eir = 0;

    while(1) {
        flags = ThisThread::flags_wait_any(FLAG_IRQ);

        if (flags & FLAG_IRQ) {
            flags = 0;
            estat = enc28j60_enet->_enc28j60->readOp(ENC28J60_READ_CTRL_REG, ESTAT);
            if ((estat & 0x80) > 0) {
                enc28j60_enet->_enc28j60->disable_interrupt(ENC28J60_INTERRUPT_ENABLE);
                enc28j60_enet->_enc28j60->clear_interrupt(ENC28J60_INTERRUPT_ENABLE);
                eir = enc28j60_enet->_enc28j60->get_interrupts();
                if ((eir & 0x40) > 0){
                    enc28j60_enet->_enc28j60->clear_interrupt(ENC28J60_INTERRUPT_RX_PENDING_ENABLE);
                    enc28j60_enet->_rx_thread->flags_set(FLAG_RX);
                    //printf("ENC28J60_INTERRUPT_RX_PENDING_ENABLE\n");
                    //break;
                }
                else if ((eir & 0x20) > 0){
                    enc28j60_enet->_enc28j60->clear_interrupt(ENC28J60_INTERRUPT_DMA_ENABLE);
                    //printf("ENC28J60_INTERRUPT_DMA_ENABLE\n");
                    enc28j60_enet->_enc28j60->enable_interrupt(ENC28J60_INTERRUPT_ENABLE);
                    //break;
                }
                else if ((eir & 0x10) > 0){
                    enc28j60_enet->_enc28j60->clear_interrupt(ENC28J60_INTERRUPT_LINK_STATE_ENABLE);
                    enc28j60_enet->link_status_task();
                    //printf("ENC28J60_INTERRUPT_LINK_STATE_ENABLE\n");
                    enc28j60_enet->_enc28j60->enable_interrupt(ENC28J60_INTERRUPT_ENABLE);
                    //break;
                }
                else if ((eir & 0x08) > 0){
                    //enc28j60_enet->_enc28j60->clear_interrupt(ENC28J60_INTERRUPT_TX_ENABLE);
                    ////printf("ENC28J60_INTERRUPT_TX_ENABLE\n");
                    //enc28j60_enet->_enc28j60->enable_interrupt(ENC28J60_INTERRUPT_ENABLE);
                    //break;
                }
                else if ((eir & 0x02) > 0){
                    //enc28j60_enet->_enc28j60->clear_interrupt(ENC28J60_INTERRUPT_TX_ERROR_ENABLE);
                    //printf("ENC28J60: TX error\n");
                    //enc28j60_enet->_enc28j60->enable_interrupt(ENC28J60_INTERRUPT_ENABLE);
                    //break;
                }
                else if ((eir & 0x01) > 0){
                    enc28j60_enet->_enc28j60->clear_interrupt(ENC28J60_INTERRUPT_RX_ERROR_ENABLE);
                    printf("ENC28J60: RX error\n");
                    enc28j60_enet->_enc28j60->freeRxBuffer();
                    enc28j60_enet->_enc28j60->reset_rx_logic();
                    enc28j60_enet->_enc28j60->enable_interrupt(ENC28J60_INTERRUPT_ENABLE);
                    //break;
                }
                else {
                    enc28j60_enet->_enc28j60->enable_interrupt(ENC28J60_INTERRUPT_ENABLE);
                    //break;
                }
            }
        }
    }
}

#endif


/**
 * @brief
 * @note    Passes a packet payload from the ethernet stack to ENC28J60 for transmittion
 * @param
 * @retval
 */
bool ENC28J60_EMAC::link_out(emac_mem_buf_t* buf)
{
    emac_mem_buf_t*     chain = buf;
    packet_t            packet;
    enc28j60_error_t    error;

    if (buf == NULL) {
        return false;
    }

    //printf("ENC28J60_EMAC::link_out entry\n");

    _ethLockMutex.lock();
    // Iterate through the buffer chain and fill the packet with payload.
    while (buf != NULL) {
        packet.payload.buf = (uint8_t *) (_memory_manager->get_ptr(buf));
        packet.payload.len = _memory_manager->get_len(buf);
        error = _enc28j60->loadPacketInTxBuffer(&packet);
        if (error != ENC28J60_ERROR_OK) {
            _memory_manager->free(chain);
            _ethLockMutex.unlock();
            printf("ETH packet could not be loaded in TX buffer\n");
            return false;
        }

        error = _enc28j60->transmitPacket(&packet);
        if (error != ENC28J60_ERROR_OK) {
            _memory_manager->free(chain);
            _ethLockMutex.unlock();
            printf("ETH packet output faillure\n");
            return false;
        }

        //printf("ETH packet output success\n");

        buf = _memory_manager->get_next(buf);
    }

    _memory_manager->free(chain);
    _ethLockMutex.unlock();

    return true;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60_EMAC::link_status_task()
{
    uint16_t    phy_basic_status_reg_value = 0;
    bool        current_link_status_up = false;

    /* Get current status */

    _ethLockMutex.lock();
    _enc28j60->phyRead(PHSTAT2, &phy_basic_status_reg_value);

    current_link_status_up = (bool) ((phy_basic_status_reg_value & PHSTAT2_LSTAT) != 0);

    /* Compare with previous state */
    if (current_link_status_up != _prev_link_status_up) {
        if (_emac_link_state_cb) {
            printf("ETH link %s\n", (current_link_status_up ? "UP" : "DOWN"));
            _emac_link_state_cb(current_link_status_up);
        }

        _prev_link_status_up = current_link_status_up;
    }

    _ethLockMutex.unlock();
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
bool ENC28J60_EMAC::power_up()
{
    volatile uint32_t   timeout = 500;
    _enc28j60->writeOp(ENC28J60_BIT_FIELD_CLR, ECON2, ECON2_PWRSV);
    while (_enc28j60->readReg(ESTAT_CLKRDY) == 0) {
        ThisThread::sleep_for(1ms);
        timeout--;
        if (timeout == 0) {
            return false;
        }
    }

#if ENC28J60_EVENT_HANDLING == ENC28J60_EVENT_HANDLING_TIMER
    /* Trigger thread to deal with any RX packets that arrived
     * before receiver_thread was started */
    _receive_task_handle = mbed::mbed_event_queue()->call_every
        (
            RECEIVE_TASK_PERIOD_MS,
            mbed::callback(this, &ENC28J60_EMAC::receive_task)
        );

    _prev_link_status_up = PHY_STATE_LINK_DOWN;
    mbed::mbed_event_queue()->call(mbed::callback(this, &ENC28J60_EMAC::link_status_task));

    /* Allow the Link Status task to detect the initial link state */
    ThisThread::sleep_for(10ms);
    _link_status_task_handle = mbed::mbed_event_queue()->call_every
        (
            LINK_STATUS_TASK_PERIOD_MS,
            mbed::callback(this, &ENC28J60_EMAC::link_status_task)
        );


#else
	_rx_thread = new Thread(RX_THREAD_PRIORITY,(uint32_t)RX_THREAD_STACKSIZE);
	_rx_thread->start(callback(&ENC28J60_EMAC::receiver_thread_function, this));

    /* enable interrupts */
    _enc28j60->enable_interrupt(ENC28J60_INTERRUPT_ENABLE);

    /* Trigger thread to deal with any RX packets that arrived
     * before receiver_thread was started */
	_rx_thread->flags_set(FLAG_RX);
    _prev_link_status_up = PHY_STATE_LINK_DOWN;
    mbed::mbed_event_queue()->call(mbed::callback(this, &ENC28J60_EMAC::link_status_task));

    /* Allow the Link Status task to detect the initial link state */
    ThisThread::sleep_for(10ms);
    mbed::mbed_event_queue()->call_every(LINK_STATUS_TASK_PERIOD_MS,
                              mbed::callback(this, &ENC28J60_EMAC::link_status_task));

    _irq_thread = new Thread(IRQ_THREAD_PRIORITY,(uint32_t)IRQ_THREAD_STACKSIZE);
	_irq_thread->start(callback(&ENC28J60_EMAC::interrupt_thread_function, this));
#endif

    return true;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
uint32_t ENC28J60_EMAC::get_mtu_size() const
{
    return ENC28J60_ETH_MTU_SIZE;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
uint32_t ENC28J60_EMAC::get_align_preference() const
{
    return ENC28J60_BUFF_ALIGNMENT;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60_EMAC::get_ifname(char* name, uint8_t size) const
{
    memcpy(name, ENC28J60_ETH_IF_NAME, (size < sizeof(ENC28J60_ETH_IF_NAME)) ? size : sizeof(ENC28J60_ETH_IF_NAME));
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
uint8_t ENC28J60_EMAC::get_hwaddr_size() const
{
    return ENC28J60_HWADDR_SIZE;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
bool ENC28J60_EMAC::get_hwaddr(uint8_t* addr) const
{
    enc28j60_error_t    error = _enc28j60->readMacAddr((char*)addr);
    if (error == ENC28J60_ERROR_OK) {
        return true;
    }
    else {
        return false;
    }
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60_EMAC::set_hwaddr(const uint8_t* addr)
{
    if (!addr) {
        return;
    }

    memcpy(_hwaddr, addr, sizeof _hwaddr);
    _ethLockMutex.lock();

    enc28j60_error_t    error = _enc28j60->writeMacAddr((char*)addr);
    _ethLockMutex.unlock();
    if (error) {
        return;
    }
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60_EMAC::set_link_input_cb(emac_link_input_cb_t input_cb)
{
    _emac_link_input_cb = input_cb;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60_EMAC::set_link_state_cb(emac_link_state_change_cb_t state_cb)
{
    _emac_link_state_cb = state_cb;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60_EMAC::add_multicast_group(const uint8_t* addr)
{
    // No action for now
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60_EMAC::remove_multicast_group(const uint8_t* addr)
{
    // No action for now
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60_EMAC::set_all_multicast(bool all)
{
    // No action for now
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60_EMAC::power_down()
{
    _enc28j60->disableMacRecv();
    if (_enc28j60->readReg(ESTAT_RXBUSY) != 0) {
        _enc28j60->enableMacRecv();
        return;
    }

    if (_enc28j60->readReg(ECON1_TXRTS) != 0) {
        _enc28j60->enableMacRecv();
        return;
    }

    _enc28j60->writeOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_VRPS);
    _enc28j60->writeOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PWRSV);

#if ENC28J60_EVENT_HANDLING == ENC28J60_EVENT_HANDLING_IRQ
    _irq_thread->terminate();
	_rx_thread->terminate();
    _irq_thread->join();
	_rx_thread->join();
	delete _irq_thread;
	delete _rx_thread;
#endif
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void ENC28J60_EMAC::set_memory_manager(EMACMemoryManager& mem_mngr)
{
    _memory_manager = &mem_mngr;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
ENC28J60_EMAC& ENC28J60_EMAC::get_instance()
{
    static ENC28J60_EMAC    emac;
    return emac;
}

EMAC& EMAC::get_default_instance()
{
    return ENC28J60_EMAC::get_instance();
}
