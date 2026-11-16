/**
 * @file  keys.c
 * @brief The public key MCUboot checks the images against. The key file is chosen at build time (Makefile KEY):
 *        keys/dev_pubkey.c for local and CI builds (development key: its private half is in keys/ and is NOT a
 *        secret), the production key for releases (its private half is a GitHub Actions secret, ADR 0015 §1).
 */
#include "bootutil/sign_key.h"
#include KEY_C

const struct bootutil_key bootutil_keys[] = { { ecdsa_pub_key, &ecdsa_pub_key_len } };
const int bootutil_key_cnt = 1;
