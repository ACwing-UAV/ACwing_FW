/**
 * Host test of the bootloader: the real MCUboot (boot_go, swap-using-move, ECDSA P-256 with tinycrypt) and the ACwing
 * flash map (Boot/port/flash_map.c) on a 512 KB flash simulated in RAM, with images signed by imgtool with the
 * development key (make boot_test builds them, boot_img/).
 *
 *   1. slot 0 valid, slot 1 empty: boots slot 0
 *   2. new image in slot 1, set pending (test): swapped in, boots it
 *   3. not confirmed, reset: reverted to the old image
 *   4. pending again, boot, confirmed (boot_set_confirmed, as the application does): stays after more resets
 *   5. slot 1 signed with another key: refused, the running image stays
 *   6. slot 1 corrupted after signing: refused
 *   7. power lost at every 7th flash operation during a swap: the swap resumes and completes, image intact
 *   8. empty flash: no image to boot
 *   9. the configuration pages (0x0807E000) are never touched
 *  10. boot_confirm.c: when the application confirms (60 s healthy, disarmed, retry after a failed write)
 */
#include "bootutil/bootutil.h"
#include "bootutil/bootutil_public.h"
#include "bootutil/image.h"
#include "flash_map_backend/flash_map_backend.h"
#include "sysflash/sysflash.h"
#include "acwing_flash_layout.h"
#include "boot_confirm.h"
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int s_fail, s_pass;
#define CHECK(c, ...) do { if (c) s_pass++; else { s_fail++; printf("FAIL %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---- simulated flash ---- */
static uint8_t  s_flash[ACW_AREA_END];
static long     s_ops_left = -1;       /* > 0: power lost when it reaches 0 */
static jmp_buf  s_power;
static unsigned s_cfg_touched;

static void power_tick(void)
{
    if (s_ops_left > 0 && --s_ops_left == 0) longjmp(s_power, 1);
}
static int rd(uint32_t off, void *dst, uint32_t len) { memcpy(dst, &s_flash[off], len); return 0; }
static int prog(uint32_t off, const void *src, uint32_t len)
{
    power_tick();
    if (off >= ACW_CFG_OFF) s_cfg_touched++;
    const uint8_t *s = src;
    for (uint32_t i = 0; i < len; ++i) {
        if (s_flash[off + i] != 0xFF && s[i] != s_flash[off + i]) return -1;   /* program over programmed bits */
        s_flash[off + i] &= s[i];
    }
    return 0;
}
static int erase(uint32_t off)
{
    power_tick();
    if (off >= ACW_CFG_OFF) s_cfg_touched++;
    memset(&s_flash[off], 0xFF, ACW_FLASH_PAGE);
    return 0;
}
static const acw_flash_ops_t s_ops = { rd, prog, erase };

/* ---- images ---- */
typedef struct { uint8_t *d; long n; } img_t;
static img_t load(const char *name)
{
    char p[256];
    snprintf(p, sizeof p, "boot_img/%s", name);
    FILE *f = fopen(p, "rb");
    img_t im = { NULL, 0 };
    if (!f) { printf("missing %s (make boot_test builds it)\n", p); exit(2); }
    fseek(f, 0, SEEK_END); im.n = ftell(f); fseek(f, 0, SEEK_SET);
    im.d = malloc((size_t)im.n);
    if (fread(im.d, 1, (size_t)im.n, f) != (size_t)im.n) exit(2);
    fclose(f);
    return im;
}
static void put(uint32_t off, uint32_t size, const img_t *im)
{
    memset(&s_flash[off], 0xFF, size);
    memcpy(&s_flash[off], im->d, (size_t)im->n);
}
/* the image in slot 0 is `im` (header + payload, up to the protected TLVs) */
static int slot0_is(const img_t *im)
{
    const struct image_header *h = (const struct image_header *)(const void *)im->d;
    const uint32_t body = h->ih_hdr_size + h->ih_img_size;
    return memcmp(&s_flash[ACW_SLOT0_OFF], im->d, body) == 0;
}

/* one reset: run MCUboot, return 0 and the image offset, -1 = nothing to boot */
static int boot(uint32_t *off)
{
    struct boot_rsp rsp;
    memset(&rsp, 0, sizeof rsp);
    FIH_DECLARE(rc, FIH_FAILURE);
    FIH_CALL(boot_go, rc, &rsp);
    if (FIH_NOT_EQ(rc, FIH_SUCCESS)) return -1;
    if (off) *off = rsp.br_image_off;
    return 0;
}

static void test_confirm_logic(void)
{
    bc_t b; bc_init(&b);
    bc_in_t ok = { 1, 1, 0 }, sick = { 1, 0, 0 }, armed = { 1, 1, 1 };
    uint32_t t = 1000;
    CHECK(!bc_step(&b, &ok, t), "not at once");
    CHECK(!bc_step(&b, &ok, t + BC_HEALTHY_MS - 1), "not before 60 s");
    CHECK(!bc_step(&b, &sick, t + 30000), "estimate lost: the minute starts again");
    CHECK(!bc_step(&b, &ok, t + 31000) && !bc_step(&b, &ok, t + 31000 + BC_HEALTHY_MS - 1), "restart counted from the recovery");
    CHECK(!bc_step(&b, &armed, t + 31000 + BC_HEALTHY_MS), "armed: deferred (no flash write in flight)");
    uint32_t now = t + 31000 + BC_HEALTHY_MS + 10;
    CHECK(bc_step(&b, &ok, now), "healthy 60 s, disarmed: confirm");
    bc_result(&b, -1, now);
    CHECK(!bc_step(&b, &ok, now + BC_RETRY_MS - 1) && bc_step(&b, &ok, now + BC_RETRY_MS), "failed write: retried after 5 s");
    bc_result(&b, 0, now + BC_RETRY_MS);
    CHECK(!bc_step(&b, &ok, now + 100000), "done once per boot");
    /* the clock wraps during the minute */
    bc_init(&b);
    CHECK(!bc_step(&b, &ok, 0xFFFFF000u) && bc_step(&b, &ok, 0xFFFFF000u + BC_HEALTHY_MS), "uint32 wrap of the ms clock");
}

int main(void)
{
    test_confirm_logic();
    acw_flash_set_ops(&s_ops);
    const img_t v1 = load("v1.bin"), v2 = load("v2.bin"), v3 = load("v3.bin"), badkey = load("badkey.bin");
    uint32_t off = 0;

    /* 8 empty flash */
    memset(s_flash, 0xFF, sizeof s_flash);
    CHECK(boot(&off) == -1, "empty flash: nothing to boot");

    /* 1 */
    memset(s_flash, 0xFF, sizeof s_flash);
    memset(&s_flash[ACW_CFG_OFF], 0x5A, ACW_AREA_END - ACW_CFG_OFF);      /* configuration in use */
    put(ACW_SLOT0_OFF, ACW_SLOT0_SIZE, &v1);
    CHECK(boot(&off) == 0 && off == ACW_SLOT0_OFF && slot0_is(&v1), "boots slot 0 (v1)");
    CHECK((ACW_FLASH_BASE + off + ACW_IMAGE_HEADER_SIZE) == ACW_APP_VECTORS, "vectors at slot 0 + header");

    /* 2 */
    put(ACW_SLOT1_OFF, ACW_SLOT1_SIZE, &v2);
    CHECK(boot_set_pending(0) == 0, "v2 pending (test)");
    CHECK(boot(&off) == 0 && slot0_is(&v2), "v2 swapped in for the test boot");

    /* 3 */
    CHECK(boot(&off) == 0 && slot0_is(&v1), "not confirmed: reverted to v1 at the next reset");
    CHECK(boot(&off) == 0 && slot0_is(&v1), "v1 stays");

    /* 4 */
    put(ACW_SLOT1_OFF, ACW_SLOT1_SIZE, &v2);
    CHECK(boot_set_pending(0) == 0 && boot(&off) == 0 && slot0_is(&v2), "v2 test boot again");
    CHECK(boot_set_confirmed() == 0, "the application confirms v2");
    CHECK(boot(&off) == 0 && slot0_is(&v2) && boot(&off) == 0 && slot0_is(&v2), "confirmed: v2 stays");

    /* 5 */
    put(ACW_SLOT1_OFF, ACW_SLOT1_SIZE, &badkey);
    (void)boot_set_pending(0);
    CHECK(boot(&off) == 0 && slot0_is(&v2), "image signed with another key: refused, v2 stays");

    /* 6 */
    put(ACW_SLOT1_OFF, ACW_SLOT1_SIZE, &v3);
    s_flash[ACW_SLOT1_OFF + ACW_IMAGE_HEADER_SIZE + 1000] ^= 0x01;
    (void)boot_set_pending(0);
    CHECK(boot(&off) == 0 && slot0_is(&v2), "one bit flipped in slot 1: refused, v2 stays");

    /* 7 power loss during the swap of v3, at many points; then permanent */
    int resumed = 0, lost = 0;
    for (long every = 3; every < 2500; every += 97) {
        put(ACW_SLOT1_OFF, ACW_SLOT1_SIZE, &v3);
        (void)boot_set_pending(1);                       /* permanent: no revert expected */
        s_ops_left = every;
        if (setjmp(s_power) == 0) { (void)boot(&off); s_ops_left = -1; }
        else { lost++; s_ops_left = -1; }
        const int ok = boot(&off) == 0 && slot0_is(&v3);
        resumed += ok;
        if (!ok) { printf("  power cut after %ld operations: not v3\n", every); break; }
        /* back to v2 for the next round */
        put(ACW_SLOT1_OFF, ACW_SLOT1_SIZE, &v2);
        (void)boot_set_pending(1);
        (void)boot(&off);
    }
    CHECK(lost > 5 && resumed == 26, "power lost %d times during a swap: resumed and completed every time (%d)", lost, resumed);

    /* 9 */
    CHECK(s_cfg_touched == 0, "configuration pages never written (%u)", s_cfg_touched);
    int cfg_ok = 1;
    for (uint32_t i = ACW_CFG_OFF; i < ACW_AREA_END; ++i) cfg_ok &= (s_flash[i] == 0x5A);
    CHECK(cfg_ok, "configuration content intact");

    free(v1.d); free(v2.d); free(v3.d); free(badkey.d);
    printf("boot: %d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
