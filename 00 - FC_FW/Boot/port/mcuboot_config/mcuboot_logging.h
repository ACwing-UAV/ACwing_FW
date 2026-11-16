/* MCUboot logging: disabled in the ACwing bootloader (MCUBOOT_HAVE_LOGGING not defined). */
#ifndef MCUBOOT_LOGGING_H
#define MCUBOOT_LOGGING_H
#define MCUBOOT_LOG_LEVEL_OFF      0
#define MCUBOOT_LOG_LEVEL_ERROR    1
#define MCUBOOT_LOG_LEVEL_WARNING  2
#define MCUBOOT_LOG_LEVEL_INFO     3
#define MCUBOOT_LOG_LEVEL_DEBUG    4
#define MCUBOOT_LOG_LEVEL MCUBOOT_LOG_LEVEL_OFF
#define MCUBOOT_LOG_MODULE_DECLARE(...)
#define MCUBOOT_LOG_MODULE_REGISTER(...)
#define MCUBOOT_LOG_ERR(...) do { } while (0)
#define MCUBOOT_LOG_WRN(...) do { } while (0)
#define MCUBOOT_LOG_INF(...) do { } while (0)
#define MCUBOOT_LOG_DBG(...) do { } while (0)
#define MCUBOOT_LOG_SIM(...) do { } while (0)
#endif
