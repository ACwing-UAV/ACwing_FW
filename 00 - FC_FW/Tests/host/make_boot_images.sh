#!/bin/sh
# Signed test images for boot_test (MCUboot on the host): three versions with the development key, one with
# another key. Payloads are random bytes; MCUboot checks the header, the hash and the signature, not the content.
set -e
OUT=boot_img
KEY=../../Boot/keys/dev-ecdsa-p256.pem
mkdir -p "$OUT"
sign() { imgtool sign -k "$1" --header-size 0x400 --pad-header --align 8 --slot-size 0x3B000 --max-sectors 60 \
         --version "$2" "$OUT/$3.raw" "$OUT/$3.bin" >/dev/null; }
for v in 1 2 3; do head -c $((90000 + v * 7000)) /dev/urandom > "$OUT/v$v.raw"; sign "$KEY" "0.$v.0" "v$v"; done
imgtool keygen -k "$OUT/other.pem" -t ecdsa-p256
cp "$OUT/v3.raw" "$OUT/badkey.raw"; sign "$OUT/other.pem" 0.9.0 badkey
