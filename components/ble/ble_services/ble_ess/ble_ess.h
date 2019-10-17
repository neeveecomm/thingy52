/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __BLE_ESS_H__
#define __BLE_ESS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BLE_ESS_BLE_OBSERVER_PRIO
#define BLE_ESS_BLE_OBSERVER_PRIO 2
#endif
void ble_ess_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

#define BLE_ESS_DEF(_name)                                                                          \
static ble_ess_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_ESS_BLE_OBSERVER_PRIO,                                                     \
                     ble_ess_on_ble_evt, &_name)

#define ESS_UUID_SERVICE          0x181A
#define ESS_UUID_TEMPERATURE_CHAR 0x2A6E
#define ESS_UUID_HUMIDITY_CHAR    0x2A6F

#define ESS_UPDATE_INTERVAL        2000

typedef struct ble_ess_s ble_ess_t;
typedef void (*ble_ess_notif_write_handler_t) (uint16_t uuid_char, uint8_t new_state);

/**@brief Environment Sensing Service structure. This structure contains various status information for the service. */
struct ble_ess_s
{
    uint16_t                    service_handle;       /**< Handle of Environment Sensing Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    temp_char_handles;    /**< Handles related to the Temperature Characteristic. */
    ble_gatts_char_handles_t    humid_char_handles;   /**< Handles related to the Button Characteristic. */
    uint8_t                     uuid_type;            /**< UUID type for the Environment Sensing Service. */
    ble_ess_notif_write_handler_t  notif_write_handler; 
};

uint32_t ble_ess_init(ble_ess_t * p_ess);
uint32_t ble_ess_notify_temp(uint16_t conn_handle, ble_ess_t * p_ess, uint16_t temp);
uint32_t ble_ess_notify_humid(uint16_t conn_handle, ble_ess_t * p_ess, uint16_t humid);

void ess_notif_timers_create(app_timer_timeout_handler_t timeout_handler);
void ess_notif_timer_start(uint32_t msec);
void ess_notif_timer_stop(void);

uint32_t ble_ess_update_temp(ble_ess_t * p_ess, uint16_t temp);
uint32_t ble_ess_update_humid(ble_ess_t * p_ess, uint16_t humid);
#ifdef __cplusplus
}
#endif

#endif