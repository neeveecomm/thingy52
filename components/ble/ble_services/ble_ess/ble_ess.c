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

#include "sdk_common.h"
#include "ble_ess.h"
#include "ble_srv_common.h"
#include "app_timer.h"

APP_TIMER_DEF(ess_notif_timer);

static void on_write(ble_ess_t *p_ess, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_ess->temp_char_handles.cccd_handle) && (p_ess->notif_write_handler != NULL)) {
            p_ess->notif_write_handler(ESS_UUID_TEMPERATURE_CHAR, p_evt_write->data[0]);
    } else if ((p_evt_write->handle == p_ess->humid_char_handles.cccd_handle) && (p_ess->notif_write_handler != NULL)) {
            p_ess->notif_write_handler(ESS_UUID_HUMIDITY_CHAR, p_evt_write->data[0]);
    }
}

void ble_ess_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_ess_t * p_ess = (ble_ess_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_write(p_ess, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

uint32_t ble_ess_init(ble_ess_t * p_ess)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;
    
    BLE_UUID_BLE_ASSIGN(ble_uuid, ESS_UUID_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_ess->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add Button characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = ESS_UUID_TEMPERATURE_CHAR;
    add_char_params.uuid_type         = p_ess->uuid_type;
    add_char_params.init_len          = sizeof(int16_t);
    add_char_params.max_len           = sizeof(int16_t);
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.notify = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_ess->service_handle,
                                  &add_char_params,
                                  &p_ess->temp_char_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Button characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = ESS_UUID_HUMIDITY_CHAR;
    add_char_params.uuid_type         = p_ess->uuid_type;
    add_char_params.init_len          = sizeof(uint16_t);
    add_char_params.max_len           = sizeof(uint16_t);
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.notify = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_ess->service_handle,
                                  &add_char_params,
                                  &p_ess->humid_char_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return 0;
}

void ess_notif_timers_create(app_timer_timeout_handler_t timeout_handler)
{
    ret_code_t err_code;
    err_code = app_timer_create(&ess_notif_timer, APP_TIMER_MODE_REPEATED, timeout_handler);
    APP_ERROR_CHECK(err_code);
}

void ess_notif_timer_start(uint32_t msec)
{
    ret_code_t err_code = app_timer_start(ess_notif_timer, APP_TIMER_TICKS(msec), NULL);
    APP_ERROR_CHECK(err_code);
}

void ess_notif_timer_stop(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_stop(ess_notif_timer);
    APP_ERROR_CHECK(err_code);
}

uint32_t ble_ess_notify_temp(uint16_t conn_handle, ble_ess_t * p_ess, uint16_t temp)
{
    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(temp);
    uint8_t tempval[2];

    tempval[0] = (temp & 0xFF);
    tempval[1] = ((temp>>8) & 0xFF);

    memset(&params, 0, sizeof(params));
    params.type   = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_ess->temp_char_handles.value_handle;
    params.p_data = tempval;
    params.p_len  = &len;

    return sd_ble_gatts_hvx(conn_handle, &params);
}

uint32_t ble_ess_notify_humid(uint16_t conn_handle, ble_ess_t * p_ess, uint16_t humid)
{
    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(humid);
    uint8_t humidval[2];

    humidval[0] = (humid & 0xFF);
    humidval[1] = ((humid>>8) & 0xFF);

    memset(&params, 0, sizeof(params));
    params.type   = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_ess->humid_char_handles.value_handle;
    params.p_data = humidval;
    params.p_len  = &len;

    return sd_ble_gatts_hvx(conn_handle, &params);
}

uint32_t ble_ess_update_temp(ble_ess_t * p_ess, uint16_t temp)
{
    ret_code_t         err_code = NRF_SUCCESS;
    ble_gatts_value_t  gatts_value;
    uint8_t tempval[2];

    tempval[0] = (temp & 0xFF);
    tempval[1] = ((temp>>8) & 0xFF);

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(temp);
    gatts_value.offset  = 0;
    gatts_value.p_value = tempval;

    // Update database.
    err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
                                      p_ess->temp_char_handles.value_handle,
                                      &gatts_value);

    return err_code;
}

uint32_t ble_ess_update_humid(ble_ess_t * p_ess, uint16_t humid)
{
    ret_code_t         err_code = NRF_SUCCESS;
    ble_gatts_value_t  gatts_value;
    uint8_t humidval[2];

    humidval[0] = (humid & 0xFF);
    humidval[1] = ((humid>>8) & 0xFF);

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(humid);
    gatts_value.offset  = 0;
    gatts_value.p_value = humidval;

    // Update database.
    err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
                                      p_ess->humid_char_handles.value_handle,
                                      &gatts_value);

    return err_code;
}
