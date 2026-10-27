/**
 * @file  ble_tlm_svc.h
 * @brief "ACwing telemetry" GATT service on the STM32WB BLE stack (STM32_WPAN).
 *
 *   Service  7E4A0001-2C6B-4F8E-9A5D-3B1C0E1D2F30
 *     TLM    7E4A0002-...  Notify            telemetry byte stream (same frames as UART)
 *     CMD    7E4A0003-...  Write w/o resp.   commands (tlm_ble.h: TLM_BLE_CMD_*)
 *
 * Three hooks go into the CubeMX-generated STM32_WPAN/App/app_ble.c
 * (USER CODE sections, they survive regeneration):
 *
 *   APP_BLE_Init()            USER CODE BEGIN APP_BLE_Init_3   ble_tlm_svc_init();
 *   SVCCTL_App_Notification() USER CODE BEGIN HCI_EVT_LE_CONN_COMPLETE
 *                                 ble_tlm_on_connect(p_connection_complete_event->Connection_Handle);
 *                             USER CODE BEGIN EVT_DISCONN_COMPLETE
 *                                 ble_tlm_on_disconnect();
 *
 * plus `#include "ble_tlm_svc.h"` in USER CODE BEGIN Includes.
 * With APP_TLM_BLE = 0 the functions exist but do nothing.
 */
#ifndef BLE_TLM_SVC_H
#define BLE_TLM_SVC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** After SVCCTL_Init() and before advertising starts (APP_BLE_Init_3) */
void ble_tlm_svc_init(void);
void ble_tlm_on_connect(uint16_t conn_handle);
void ble_tlm_on_disconnect(void);

#ifdef __cplusplus
}
#endif
#endif /* BLE_TLM_SVC_H */
