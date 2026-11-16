# Host tests of the protocol-v2 firmware modules (no HAL, no RTOS).
#   make        build and run
APP  = ../../App
SRC  = $(APP)/telemetry/telemetry.c $(APP)/telemetry/proto_v2.c $(APP)/telemetry/ack_cache.c \
       $(APP)/telemetry/app_link.c $(APP)/telemetry/tlm_ble.c \
       $(APP)/app/params.c $(APP)/app/cfg_store.c $(APP)/app/fc_shared.c $(APP)/app/app_task.c \
       $(APP)/control/fc_core.c $(APP)/control/calib_mgr.c $(APP)/control/cal_motors.c $(APP)/control/checks.c \
       $(APP)/control/battery.c $(APP)/app/batt_task.c $(APP)/log/log_queue.c $(APP)/log/log_feed.c \
       $(APP)/log/log_store.c $(APP)/app/log_task.c $(APP)/telemetry/usb_link.c \
       $(APP)/filters/altitude_kf/alt_est.c $(APP)/control/alt_ctrl.c \
       $(APP)/control/att_ctrl.c $(APP)/control/mixer.c $(APP)/control/motors.c $(APP)/app/ctrl_loop.c \
       $(APP)/filters/hover_est/hover_est.c $(APP)/control/hover_track.c \
       $(APP)/control/rc_input.c $(APP)/drivers/rc/rc_crsf.c $(APP)/drivers/rc/rc_sbus.c $(APP)/drivers/rc/rc_ibus.c \
       $(APP)/drivers/rc/rc_dsm.c $(APP)/control/input_arb.c $(APP)/control/radio_fs.c $(APP)/control/cal_sticks.c $(APP)/control/rc_switches.c $(APP)/control/pos_ctrl.c $(APP)/control/launch.c $(APP)/control/flip.c $(APP)/control/ctl_io.c $(APP)/control/ctl_law_cascade.c \
       $(APP)/drivers/vl53l1x/vl53l1x.c $(APP)/drivers/flow_bridge/flow_bridge.c \
       $(APP)/drivers/ublox/ubx.c $(APP)/app/gnss_nav.c $(APP)/app/boot_confirm.c \
       host_stubs.c test_main.c
INC  = -I$(APP)/log -I$(APP)/drivers/w25q -I$(APP)/telemetry -I$(APP)/app -I$(APP)/control -I$(APP)/bsp -I$(APP)/filters/altitude_kf -I$(APP)/filters/hover_est -I$(APP)/drivers/rc \
       -I$(APP)/drivers/common -I$(APP)/drivers/vl53l1x -I$(APP)/drivers/flow_bridge \
       -I$(APP)/drivers/ublox -I$(APP)/drivers/iis2mdc
CFLAGS = -std=c11 -O1 -g -Wall -Wextra -Wno-unused-parameter -DHOST_TEST \
         -DAPP_BLE_STATE_DECIM_DEFAULT=4 -fsanitize=address,undefined

FLOW = ../../../01 - FLOW_FW
GNSS_SRC = $(APP)/app/ext_sens.c $(APP)/drivers/vl53l1x/vl53l1x.c $(APP)/drivers/flow_bridge/flow_bridge.c \
           $(APP)/drivers/ublox/ubx.c $(APP)/drivers/ublox/m10.c $(APP)/drivers/iis2mdc/iis2mdc.c \
           $(APP)/app/gnss_nav.c $(APP)/control/checks.c test_gnss.c
EXT_SRC = $(APP)/app/ext_sens.c $(APP)/drivers/vl53l1x/vl53l1x.c $(APP)/drivers/flow_bridge/flow_bridge.c \
          $(APP)/drivers/ublox/ubx.c $(APP)/drivers/ublox/m10.c $(APP)/drivers/iis2mdc/iis2mdc.c \
          "$(FLOW)/Core/Src/bridge.c" test_ext.c

PWR_SRC = $(filter-out test_main.c,$(SRC)) test_power.c
LOG_SRC = $(filter-out test_main.c,$(SRC)) \
          $(APP)/log/log_flash_ram.c $(APP)/drivers/w25q/w25q.c test_log.c
LINK_SRC = $(filter-out test_main.c,$(SRC)) $(APP)/log/log_flash_ram.c test_link.c

CTRL_SRC = $(APP)/control/att_ctrl.c $(APP)/control/mixer.c test_ctrl.c
RC_SRC = $(APP)/drivers/rc/rc_crsf.c $(APP)/drivers/rc/rc_sbus.c $(APP)/drivers/rc/rc_ibus.c $(APP)/drivers/rc/rc_dsm.c \
         $(APP)/control/rc_input.c $(APP)/control/input_arb.c $(APP)/control/radio_fs.c $(APP)/control/cal_sticks.c $(APP)/control/rc_switches.c test_rc.c
HOVER_SRC = $(APP)/filters/hover_est/hover_est.c $(APP)/control/hover_track.c $(APP)/control/alt_ctrl.c test_hover.c

BOOT = ../../Boot
MCB  = $(BOOT)/third_party/mcuboot
BOOT_SRC = $(BOOT)/port/flash_map.c $(BOOT)/src/keys.c \
           $(addprefix $(MCB)/boot/bootutil/src/,boot_record.c bootutil_area.c bootutil_find_key.c bootutil_img_hash.c \
             bootutil_img_security_cnt.c bootutil_loader.c bootutil_misc.c bootutil_public.c caps.c \
             fault_injection_hardening.c image_ecdsa.c image_validate.c loader.c swap_misc.c swap_move.c tlv.c) \
           $(addprefix $(MCB)/ext/tinycrypt/lib/source/,ecc.c ecc_dsa.c sha256.c utils.c ecc_platform_specific.c) \
           $(MCB)/ext/mbedtls-asn1/src/asn1parse.c $(MCB)/ext/mbedtls-asn1/src/platform_util.c \
           $(APP)/app/boot_confirm.c test_boot.c
BOOT_INC = -I$(APP)/app -I$(BOOT)/port -I$(MCB)/boot/bootutil/include -I$(MCB)/boot/bootutil/src \
           -I$(MCB)/ext/tinycrypt/lib/include -I$(MCB)/ext/mbedtls-asn1/include
BOOT_CFLAGS = -std=gnu11 -O1 -g -Wall -Wno-unused-function -DHOST_TEST -fsanitize=address,undefined \
              -ffunction-sections -Wl,--gc-sections \
              -DKEY_C='"$(abspath $(BOOT)/keys/dev_pubkey.c)"' -DMBEDTLS_CONFIG_FILE='"mbedtls/mbedtls_config.h"'

IEKF_SRC = $(APP)/filters/iekf/iekf.c $(APP)/filters/iekf/so3.c $(APP)/filters/iekf/calib.c $(APP)/filters/iekf/accwin.c \
           $(APP)/app/est_pipeline.c $(APP)/app/est_config.c $(APP)/control/control_hooks.c test_iekf.c
IEKF_INC = -Istubs -I$(APP)/app -I$(APP)/filters/iekf -I$(APP)/control -I$(APP)/drivers/imu_lsm6dsv16x \
           -I$(APP)/drivers/common -I$(APP)/third_party/st_lsm6dsv16x
IEKF_CFLAGS = -std=c11 -O2 -g -Wall -Wextra -Wno-unused-parameter -DHOST_TEST -fsanitize=address,undefined

all: run
host_test: $(SRC) $(wildcard $(APP)/*/*.h)
	$(CC) $(CFLAGS) $(INC) $(SRC) -lm -o $@
run: host_test ext_test gnss_test boot_test power_test log_test link_test iekf_test ctrl_test hover_test rc_test pos_test launch_test flip_test
	./host_test
	./ext_test
	./gnss_test
	./boot_test
	./power_test
	./log_test
	./link_test
	./iekf_test
	./ctrl_test
	./pos_test
	./launch_test
	./flip_test
	./hover_test
	./rc_test
rc_test: $(RC_SRC) $(wildcard $(APP)/drivers/rc/*.h $(APP)/control/rc_input.h)
	$(CC) $(CFLAGS) -I$(APP)/drivers/rc -I$(APP)/control -I$(APP)/telemetry $(RC_SRC) -lm -o $@
hover_test: $(HOVER_SRC) $(wildcard $(APP)/control/*.h $(APP)/filters/hover_est/*.h)
	$(CC) $(CFLAGS) -I$(APP)/control -I$(APP)/filters/hover_est $(HOVER_SRC) -lm -o $@
FLIP_SRC = $(APP)/control/flip.c $(APP)/control/att_ctrl.c $(APP)/control/mixer.c test_flip.c
flip_test: $(FLIP_SRC) $(APP)/control/flip.h plant6.h
	$(CC) $(CFLAGS) -I$(APP)/control -I$(APP)/telemetry $(FLIP_SRC) -lm -o $@
LAUNCH_SRC = $(APP)/control/launch.c $(APP)/control/att_ctrl.c $(APP)/control/mixer.c test_launch.c
launch_test: $(LAUNCH_SRC) $(APP)/control/launch.h plant6.h
	$(CC) $(CFLAGS) -I$(APP)/control $(LAUNCH_SRC) -lm -o $@
POS_SRC = $(APP)/control/pos_ctrl.c test_pos.c
pos_test: $(POS_SRC) $(APP)/control/pos_ctrl.h
	$(CC) $(CFLAGS) -I$(APP)/control $(POS_SRC) -lm -o $@
ctrl_test: $(CTRL_SRC) $(wildcard $(APP)/control/*.h)
	$(CC) $(CFLAGS) -I$(APP)/control $(CTRL_SRC) -lm -o $@
log_test: $(LOG_SRC) $(wildcard $(APP)/*/*.h)
	$(CC) $(CFLAGS) $(INC) $(LOG_SRC) -lm -o $@
link_test: $(LINK_SRC) $(wildcard $(APP)/*/*.h)
	$(CC) $(CFLAGS) $(INC) $(LINK_SRC) -lm -o $@
power_test: $(PWR_SRC) $(wildcard $(APP)/*/*.h)
	$(CC) $(CFLAGS) $(INC) $(PWR_SRC) -lm -o $@
iekf_test: $(sort $(IEKF_SRC)) $(wildcard $(APP)/*/*.h $(APP)/filters/iekf/*.h)
	$(CC) $(IEKF_CFLAGS) $(IEKF_INC) $(sort $(IEKF_SRC)) -lm -o $@
ext_test:
	$(CC) $(CFLAGS) $(INC) -I"$(FLOW)/Core/Inc" $(EXT_SRC) -lm -o $@
boot_test: $(BOOT_SRC) $(BOOT)/port/acwing_flash_layout.h make_boot_images.sh
	./make_boot_images.sh
	$(CC) $(BOOT_CFLAGS) $(BOOT_INC) $(BOOT_SRC) -o $@
gnss_test: $(GNSS_SRC) $(wildcard $(APP)/drivers/ublox/*.h $(APP)/drivers/iis2mdc/*.h $(APP)/app/ext_sens.h $(APP)/app/gnss_nav.h)
	$(CC) $(CFLAGS) $(INC) $(GNSS_SRC) -lm -o $@
clean:
	rm -rf boot_img; rm -f host_test ext_test gnss_test boot_test power_test log_test link_test iekf_test ctrl_test hover_test rc_test pos_test launch_test flip_test frames.bin power_frames.bin log_sample.bin
.PHONY: all run clean ext_test
