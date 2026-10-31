/**
 * @file  usb_cdc.h
 * @brief USB CDC (virtual COM port) binding of the USB link
 *        (telemetry/usb_link.h).
 *
 * usb_cdc_init() starts the USB device. Received bytes go to usbl_rx_isr()
 * from the USB interrupt. usb_cdc_pump(), called by the LogTask, moves queued
 * TX bytes to the IN endpoint whenever the previous transfer has finished.
 * The line coding set by the PC (baud rate, ...) is ignored: CDC runs at the
 * USB rate.
 */
#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdint.h>

int     usb_cdc_init(void);       /**< 0 = started */
void    usb_cdc_pump(void);
uint8_t usb_cdc_configured(void); /**< enumerated by a host */

#endif /* USB_CDC_H */
