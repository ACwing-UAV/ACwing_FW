/**
 * @file  ble_tlm_svc.c
 * @brief "ACwing telemetry" GATT service + TX thread on STM32_WPAN (see ble_tlm_svc.h).
 *
 * Protocol v2 (HANDOVER_FW.md §2): CMD writes are only recorded (app_link.c),
 * the TX thread drains the two priority queues of tlm_ble.c.
 *
 * Event flow
 *   HCI user-event thread (stack)  -> TlmSvc_EventHandler(): only flags and
 *                                     counters, never an ACI call, never blocks
 *   BleTxThread (ours)             -> every APP_BLE_FLUSH_MS, or when woken:
 *                                     connection parameters, then queued bytes
 *                                     as notifications of up to ATT_MTU-3 bytes
 *
 * ACI calls from this thread are safe: in the FreeRTOS version of app_ble.c,
 * BLE_StatusNot() takes MtxHciId around every HCI command (HCI_TL_CmdBusy /
 * HCI_TL_CmdAvailable), so commands from different threads are serialised.
 *
 * Flow control: aci_gatt_update_char_value() returns
 * BLE_STATUS_INSUFFICIENT_RESOURCES when the TX pool is full (the link is
 * slower than the data). The chunk is NOT consumed: we wait for
 * ACI_GATT_TX_POOL_AVAILABLE (or the next tick) and retry. If the link stays
 * slower than the production for long, the ring fills and TelemetryTask drops
 * whole frames (STATUS.ble_drop): the stream stays consistent.
 */
#include "ble_tlm_svc.h"
#include "app_config.h"

#if APP_TLM_BLE

#if defined(__has_include)
#if !__has_include("common_blesvc.h")
#error "APP_TLM_BLE = 1 requires STM32_WPAN (BLE) enabled in CubeMX: see the README, section 'BLE telemetry (phase 2)'"
#endif
#endif

#include "common_blesvc.h"
#include "cmsis_os2.h"
#include "tlm_ble.h"
#include "app_link.h"
#include "main.h"
#include <string.h>

#ifndef CFG_BLE_MAX_ATT_MTU
#define CFG_BLE_MAX_ATT_MTU 23
#endif
#define TLM_CHAR_MAX_LEN  (CFG_BLE_MAX_ATT_MTU - 3)   /* one notification          */
#define CMD_CHAR_MAX_LEN  20

#define FLAG_KICK   0x0001u    /* pool available / subscription changed / connected */

/* UUIDs written MSB first, as printed (7E4A0001-2C6B-...); the stack wants them LSB first */
static const uint8_t UUID_SVC[16] = { 0x7E,0x4A,0x00,0x01, 0x2C,0x6B, 0x4F,0x8E,
                                      0x9A,0x5D, 0x3B,0x1C,0x0E,0x1D,0x2F,0x30 };
static const uint8_t UUID_TLM[16] = { 0x7E,0x4A,0x00,0x02, 0x2C,0x6B, 0x4F,0x8E,
                                      0x9A,0x5D, 0x3B,0x1C,0x0E,0x1D,0x2F,0x30 };
static const uint8_t UUID_CMD[16] = { 0x7E,0x4A,0x00,0x03, 0x2C,0x6B, 0x4F,0x8E,
                                      0x9A,0x5D, 0x3B,0x1C,0x0E,0x1D,0x2F,0x30 };

static uint16_t          s_svcHdl, s_tlmHdl, s_cmdHdl;
static volatile uint16_t s_conn = 0xFFFFu;
static volatile uint8_t  s_notify;          /* CCCD of TLM: notifications enabled */
static volatile uint8_t  s_paramsPending;   /* request connection parameters/DLE  */
static osThreadId_t      s_txThread;
static uint8_t           s_chunk[TLM_CHAR_MAX_LEN];

static void uuid_lsb_first(uint8_t *dst, const uint8_t *msb_first)
{
    for (int i = 0; i < 16; ++i) dst[i] = msb_first[15 - i];
}

static void kick(void)
{
    if (s_txThread) (void)osThreadFlagsSet(s_txThread, FLAG_KICK);
}

/* ------------------------------------------------------------------ events
 * Runs in the stack's HCI user-event thread: flags only. */
static SVCCTL_EvtAckStatus_t TlmSvc_EventHandler(void *Event)
{
    SVCCTL_EvtAckStatus_t ret = SVCCTL_EvtNotAck;
    hci_event_pckt *ev = (hci_event_pckt *)(((hci_uart_pckt *)Event)->data);
    if (ev->evt != HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE) return ret;

    evt_blecore_aci *ble = (evt_blecore_aci *)ev->data;
    switch (ble->ecode) {
    case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE: {
        aci_gatt_attribute_modified_event_rp0 *am =
            (aci_gatt_attribute_modified_event_rp0 *)ble->data;
        if (am->Attr_Handle == (uint16_t)(s_tlmHdl + 2u)) {          /* CCCD of TLM */
            ret = SVCCTL_EvtAckFlowEnable;
            s_conn = am->Connection_Handle;
            if (am->Attr_Data_Length > 0u && (am->Attr_Data[0] & COMSVC_Notification)) {
                tlm_ble_set_state(TLM_BLE_STREAMING);
                s_notify = 1u;
                s_paramsPending = 1u;
            } else {
                s_notify = 0u;
                tlm_ble_set_state(TLM_BLE_CONNECTED);
            }
            kick();
        } else if (am->Attr_Handle == (uint16_t)(s_cmdHdl + 1u)) {   /* CMD value */
            ret = SVCCTL_EvtAckFlowEnable;
            /* Only the arrival is recorded here (app_link.h): time from the 1 kHz
             * HAL tick, RC values, or a copy into the command queue drained by
             * the AppTask. No command is executed in the BLE context. */
            if (!g_dbg_ble_stall) {
                appl_on_cmd_write(am->Attr_Data, am->Attr_Data_Length, HAL_GetTick());
                tlm_ble_on_command(am->Attr_Data, am->Attr_Data_Length);
            }
        }
        break;
    }
    case ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE: {
        aci_att_exchange_mtu_resp_event_rp0 *mr =
            (aci_att_exchange_mtu_resp_event_rp0 *)ble->data;
        /* effective ATT_MTU = min(ours, peer's) */
        uint16_t mtu = mr->Server_RX_MTU;
        if (mtu > CFG_BLE_MAX_ATT_MTU) mtu = CFG_BLE_MAX_ATT_MTU;
        tlm_ble_set_mtu(mtu);
        break;                         /* not ours only: let app_ble.c see it too */
    }
    case ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE:
        kick();
        break;
    default:
        break;
    }
    return ret;
}

/* ------------------------------------------------------------------ TX thread */
static void request_link_params(uint16_t conn)
{
    /* 15-30 ms, no latency, 4 s supervision: accepted by iOS (min 15 ms,
     * max >= min + 15 ms) and Windows/macOS. The central has the last word. */
    tBleStatus st = aci_l2cap_connection_parameter_update_req(conn,
                        APP_BLE_CONN_INT_MIN, APP_BLE_CONN_INT_MAX, 0u,
                        APP_BLE_SUPERV_TIMEOUT);
    if (st != BLE_STATUS_SUCCESS) tlm_ble_set_error(st);
    /* MTU exchange started by us as well: iOS and Windows usually start it
     * themselves, but if the central does not, we would stay at 23 bytes.
     * The result arrives as ACI_ATT_EXCHANGE_MTU_RESP (handled above). */
    (void)aci_gatt_exchange_config(conn);
    /* Data Length Extension: 251-byte link-layer packets -> one notification
     * per packet with large MTUs. Not fatal if the central refuses. */
    (void)hci_le_set_data_length(conn, 251u, 2120u);
}

static void BleTxThread(void *arg)
{
    (void)arg;
    for (;;) {
        (void)osThreadFlagsWait(FLAG_KICK, osFlagsWaitAny, APP_BLE_FLUSH_MS);
        if (g_dbg_ble_stall) continue;            /* bench test: BLE path stalled on purpose */

        if (s_paramsPending && s_conn != 0xFFFFu) {
            s_paramsPending = 0u;
            request_link_params(s_conn);
        }
        if (!s_notify) { tlm_ble_discard(); continue; }

        uint16_t maxc = tlm_ble_chunk_max();
        if (maxc > TLM_CHAR_MAX_LEN) maxc = TLM_CHAR_MAX_LEN;
        for (;;) {
            const uint16_t n = tlm_ble_peek(s_chunk, maxc);
            if (n == 0u) break;
            const tBleStatus st = aci_gatt_update_char_value(s_svcHdl, s_tlmHdl, 0u,
                                                             (uint8_t)n, s_chunk);
            if (st == BLE_STATUS_SUCCESS) {
                tlm_ble_consume(n);
            } else if (st == BLE_STATUS_INSUFFICIENT_RESOURCES || st == BLE_STATUS_BUSY) {
                break;                    /* link full: retry on TX_POOL_AVAILABLE */
            } else {
                tlm_ble_set_error(st);    /* e.g. disconnected meanwhile */
                tlm_ble_discard();
                break;
            }
        }
    }
}

/* ------------------------------------------------------------------ public */
extern volatile uint8_t g_cpu2_ready;           /* platform/cfg_flash_wb.c */

void ble_tlm_svc_init(void)
{
    Char_UUID_t uuid;
    tBleStatus  st;

    g_cpu2_ready = 1u;    /* called from APP_BLE_Init(): the wireless stack is running */

    tlm_ble_reset();
    SVCCTL_RegisterSvcHandler(TlmSvc_EventHandler);

    /* records: 1 service + TLM (decl, value, CCCD) + CMD (decl, value) = 6 -> 8 */
    uuid_lsb_first(uuid.Char_UUID_128, UUID_SVC);
    st = aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t *)&uuid, PRIMARY_SERVICE,
                              8u, &s_svcHdl);
    if (st != BLE_STATUS_SUCCESS) { tlm_ble_set_error(st); return; }

    uuid_lsb_first(uuid.Char_UUID_128, UUID_TLM);
    st = aci_gatt_add_char(s_svcHdl, UUID_TYPE_128, &uuid, TLM_CHAR_MAX_LEN,
                           CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE,
                           GATT_DONT_NOTIFY_EVENTS, 0x10u, CHAR_VALUE_LEN_VARIABLE,
                           &s_tlmHdl);
    if (st != BLE_STATUS_SUCCESS) { tlm_ble_set_error(st); return; }

    uuid_lsb_first(uuid.Char_UUID_128, UUID_CMD);
    st = aci_gatt_add_char(s_svcHdl, UUID_TYPE_128, &uuid, CMD_CHAR_MAX_LEN,
                           CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_WRITE,
                           ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
                           0x10u, CHAR_VALUE_LEN_VARIABLE, &s_cmdHdl);
    if (st != BLE_STATUS_SUCCESS) { tlm_ble_set_error(st); return; }

    static const osThreadAttr_t a = { .name = "bletx", .priority = osPriorityBelowNormal,
                                      .stack_size = 1024 };
    s_txThread = osThreadNew(BleTxThread, NULL, &a);
    tlm_ble_set_state(s_txThread ? TLM_BLE_READY : TLM_BLE_OFF);
}

void ble_tlm_on_connect(uint16_t conn_handle)
{
    appl_on_connect();
    s_conn = conn_handle;
    s_notify = 0u;
    tlm_ble_set_mtu(TLM_BLE_MTU_DEFAULT);
    tlm_ble_set_state(TLM_BLE_CONNECTED);
}

void ble_tlm_on_disconnect(void)
{
    appl_on_disconnect();     /* may only make the app-link failsafe start earlier */
    s_conn = 0xFFFFu;
    s_notify = 0u;
    s_paramsPending = 0u;
    tlm_ble_set_state(TLM_BLE_READY);     /* app_ble.c restarts advertising */
    kick();                               /* the TX thread empties the queue */
}

#else  /* APP_TLM_BLE == 0: hooks compile to nothing */

void ble_tlm_svc_init(void) {}
void ble_tlm_on_connect(uint16_t conn_handle) { (void)conn_handle; }
void ble_tlm_on_disconnect(void) {}

#endif /* APP_TLM_BLE */
