/**
 * @file  usb_cdc.c
 * @brief USB CDC binding of the USB link (see usb_cdc.h).
 */
#include "usb_cdc.h"
#include "usb_link.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_desc.h"

static USBD_HandleTypeDef s_dev;
static uint8_t s_rx[CDC_DATA_FS_MAX_PACKET_SIZE];
static uint8_t s_tx[512];
static USBD_CDC_LineCodingTypeDef s_line = { 115200u, 0u, 0u, 8u };

static int8_t itf_init(void)
{
    USBD_CDC_SetTxBuffer(&s_dev, s_tx, 0);
    USBD_CDC_SetRxBuffer(&s_dev, s_rx);
    return USBD_OK;
}

static int8_t itf_deinit(void) { return USBD_OK; }

static int8_t itf_control(uint8_t cmd, uint8_t *p, uint16_t len)
{
    (void)len;
    /* line coding is stored and echoed only: the PC tools open the port at any baud rate */
    if (cmd == CDC_SET_LINE_CODING) {
        s_line.bitrate = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
        s_line.format = p[4]; s_line.paritytype = p[5]; s_line.datatype = p[6];
    } else if (cmd == CDC_GET_LINE_CODING) {
        p[0] = (uint8_t)s_line.bitrate; p[1] = (uint8_t)(s_line.bitrate >> 8);
        p[2] = (uint8_t)(s_line.bitrate >> 16); p[3] = (uint8_t)(s_line.bitrate >> 24);
        p[4] = s_line.format; p[5] = s_line.paritytype; p[6] = s_line.datatype;
    }
    return USBD_OK;
}

/* USB interrupt: bytes from the PC */
static int8_t itf_receive(uint8_t *buf, uint32_t *len)
{
    usbl_rx_isr(buf, *len);
    USBD_CDC_SetRxBuffer(&s_dev, s_rx);
    USBD_CDC_ReceivePacket(&s_dev);
    return USBD_OK;
}

static int8_t itf_tx_done(uint8_t *buf, uint32_t *len, uint8_t ep)
{
    (void)buf; (void)len; (void)ep;
    return USBD_OK;
}

static USBD_CDC_ItfTypeDef s_itf = { itf_init, itf_deinit, itf_control, itf_receive, itf_tx_done };

int usb_cdc_init(void)
{
    if (USBD_Init(&s_dev, &ACW_FS_Desc, DEVICE_FS) != USBD_OK) return -1;
    if (USBD_RegisterClass(&s_dev, &USBD_CDC) != USBD_OK) return -1;
    if (USBD_CDC_RegisterInterface(&s_dev, &s_itf) != USBD_OK) return -1;
    if (USBD_Start(&s_dev) != USBD_OK) return -1;
    return 0;
}

uint8_t usb_cdc_configured(void) { return (uint8_t)(s_dev.dev_state == USBD_STATE_CONFIGURED); }

void usb_cdc_pump(void)
{
    if (!usb_cdc_configured()) {
        /* nobody listening: drop what was queued */
        uint8_t b[64];
        uint32_t n;
        while ((n = usbl_tx_peek(b, sizeof b)) != 0u) usbl_tx_consume(n);
        return;
    }
    USBD_CDC_HandleTypeDef *h = (USBD_CDC_HandleTypeDef *)s_dev.pClassData;
    if (!h || h->TxState != 0u) return;                       /* previous transfer still running */
    const uint32_t n = usbl_tx_peek(s_tx, sizeof s_tx);
    if (!n) return;
    USBD_CDC_SetTxBuffer(&s_dev, s_tx, n);
    if (USBD_CDC_TransmitPacket(&s_dev) == USBD_OK) usbl_tx_consume(n);
}
