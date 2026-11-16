/**
 * @file  boot_slot.h
 * @brief MCUboot slots from the application (MCUboot build only, APP_MCUBOOT): see boot_slot.c.
 */
#ifndef BOOT_SLOT_H
#define BOOT_SLOT_H

/** marks the running image as confirmed (MCUboot will not revert it); 0 = ok. RTOS context, disarmed. */
int boot_slot_confirm(void);

#endif /* BOOT_SLOT_H */
