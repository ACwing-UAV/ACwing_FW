/**
 * @file  usbd_desc.c
 * @brief USB descriptors of the ACwing FC (CDC virtual COM port).
 *
 * VID/PID: 0x0483/0x5740, ST's "Virtual COM Port" pair used by the STM32Cube
 * CDC examples, so the standard CDC-ACM drivers bind on every OS. It is
 * fine for development; a product needs its own VID/PID.
 * Written after the ST template usbd_desc_template.c.
 */
#include "usbd_core.h"
#include "usbd_conf.h"
#include "usbd_desc.h"

#define USBD_VID            0x0483u
#define USBD_PID            0x5740u
#define USBD_LANGID         0x0409u
#define STR_MANUFACTURER    "ACwing"
#define STR_PRODUCT         "ACwing FC"
#define STR_CONFIG          "CDC Config"
#define STR_INTERFACE       "CDC Interface"
#define USB_SIZ_STRING_SERIAL 0x1Au

static uint8_t *dev_desc(USBD_SpeedTypeDef s, uint16_t *len);
static uint8_t *lang_desc(USBD_SpeedTypeDef s, uint16_t *len);
static uint8_t *mfr_desc(USBD_SpeedTypeDef s, uint16_t *len);
static uint8_t *prod_desc(USBD_SpeedTypeDef s, uint16_t *len);
static uint8_t *serial_desc(USBD_SpeedTypeDef s, uint16_t *len);
static uint8_t *config_desc(USBD_SpeedTypeDef s, uint16_t *len);
static uint8_t *itf_desc(USBD_SpeedTypeDef s, uint16_t *len);

USBD_DescriptorsTypeDef ACW_FS_Desc = {
    dev_desc, lang_desc, mfr_desc, prod_desc, serial_desc, config_desc, itf_desc,
#if (USBD_LPM_ENABLED == 1U)
    NULL,
#endif
};

static uint8_t s_dev[USB_LEN_DEV_DESC] __attribute__((aligned(4))) = {
    0x12, USB_DESC_TYPE_DEVICE, 0x00, 0x02,
    0x02, 0x02, 0x00,                   /* class CDC, subclass ACM */
    USB_MAX_EP0_SIZE,
    LOBYTE(USBD_VID), HIBYTE(USBD_VID), LOBYTE(USBD_PID), HIBYTE(USBD_PID),
    0x00, 0x02,                         /* bcdDevice 2.00 */
    USBD_IDX_MFC_STR, USBD_IDX_PRODUCT_STR, USBD_IDX_SERIAL_STR,
    USBD_MAX_NUM_CONFIGURATION
};

static uint8_t s_lang[USB_LEN_LANGID_STR_DESC] __attribute__((aligned(4))) = {
    USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING, LOBYTE(USBD_LANGID), HIBYTE(USBD_LANGID)
};

static uint8_t s_str[USBD_MAX_STR_DESC_SIZ] __attribute__((aligned(4)));
static uint8_t s_serial[USB_SIZ_STRING_SERIAL] __attribute__((aligned(4))) = { USB_SIZ_STRING_SERIAL, USB_DESC_TYPE_STRING };

static uint8_t *dev_desc(USBD_SpeedTypeDef s, uint16_t *len)  { (void)s; *len = sizeof s_dev;  return s_dev; }
static uint8_t *lang_desc(USBD_SpeedTypeDef s, uint16_t *len) { (void)s; *len = sizeof s_lang; return s_lang; }
static uint8_t *mfr_desc(USBD_SpeedTypeDef s, uint16_t *len)  { (void)s; USBD_GetString((uint8_t *)STR_MANUFACTURER, s_str, len); return s_str; }
static uint8_t *prod_desc(USBD_SpeedTypeDef s, uint16_t *len) { (void)s; USBD_GetString((uint8_t *)STR_PRODUCT, s_str, len); return s_str; }
static uint8_t *config_desc(USBD_SpeedTypeDef s, uint16_t *len) { (void)s; USBD_GetString((uint8_t *)STR_CONFIG, s_str, len); return s_str; }
static uint8_t *itf_desc(USBD_SpeedTypeDef s, uint16_t *len)  { (void)s; USBD_GetString((uint8_t *)STR_INTERFACE, s_str, len); return s_str; }

static void hex_unicode(uint32_t v, uint8_t *p, uint8_t n)
{
    for (uint8_t i = 0; i < n; ++i) {
        const uint8_t d = (uint8_t)(v >> 28);
        p[2u * i] = (uint8_t)(d < 10u ? '0' + d : 'A' + d - 10u);
        p[2u * i + 1u] = 0;
        v <<= 4;
    }
}

/* serial: 12 hex digits from the MCU unique id */
static uint8_t *serial_desc(USBD_SpeedTypeDef s, uint16_t *len)
{
    (void)s;
    const uint32_t w0 = HAL_GetUIDw0() + HAL_GetUIDw2(), w1 = HAL_GetUIDw1();
    hex_unicode(w0, &s_serial[2], 8);
    hex_unicode(w1, &s_serial[18], 4);
    *len = USB_SIZ_STRING_SERIAL;
    return s_serial;
}
