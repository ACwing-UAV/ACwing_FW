/* Host stubs: RAM flash pages for cfg_store, expansion sensor bits. */
#include "cfg_store.h"
#include <string.h>
#include <stdint.h>

static uint8_t  s_page[CFG_PAGES][4096];
int             g_flash_fail;
int             g_flash_writes;
uint16_t        g_ext_sensors;

const void *cfg_flash_page(int p) { return s_page[p]; }
int cfg_flash_write(int p, const void *d, uint32_t len)
{
    if (g_flash_fail || len > sizeof s_page[0]) return -1;
    memset(s_page[p], 0xFF, sizeof s_page[0]);
    memcpy(s_page[p], d, len);
    g_flash_writes++;
    return 0;
}
void host_flash_erase(void) { memset(s_page, 0xFF, sizeof s_page); }

uint16_t ext_sensors(void) { return g_ext_sensors; }
