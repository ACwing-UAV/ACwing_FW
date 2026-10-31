# Host tests of the protocol-v2 firmware modules (no HAL, no RTOS).
#   make        build and run
APP  = ../../App
SRC  = $(APP)/telemetry/telemetry.c $(APP)/telemetry/proto_v2.c $(APP)/telemetry/ack_cache.c \
       $(APP)/telemetry/app_link.c $(APP)/telemetry/tlm_ble.c \
       $(APP)/app/params.c $(APP)/app/cfg_store.c $(APP)/app/fc_shared.c $(APP)/app/app_task.c \
       $(APP)/control/fc_core.c $(APP)/control/calib_mgr.c $(APP)/control/checks.c \
       $(APP)/control/battery.c $(APP)/app/batt_task.c $(APP)/log/log_queue.c $(APP)/log/log_feed.c \
       $(APP)/filters/altitude_kf/alt_est.c $(APP)/control/alt_ctrl.c \
       $(APP)/drivers/vl53l1x/vl53l1x.c $(APP)/drivers/flow_bridge/flow_bridge.c \
       host_stubs.c test_main.c
INC  = -I$(APP)/log -I$(APP)/drivers/w25q -I$(APP)/telemetry -I$(APP)/app -I$(APP)/control -I$(APP)/bsp -I$(APP)/filters/altitude_kf \
       -I$(APP)/drivers/common -I$(APP)/drivers/vl53l1x -I$(APP)/drivers/flow_bridge
CFLAGS = -std=c11 -O1 -g -Wall -Wextra -Wno-unused-parameter -DHOST_TEST \
         -DAPP_BLE_STATE_DECIM_DEFAULT=4 -fsanitize=address,undefined

FLOW = ../../../01 - FLOW_FW
EXT_SRC = $(APP)/app/ext_sens.c $(APP)/drivers/vl53l1x/vl53l1x.c $(APP)/drivers/flow_bridge/flow_bridge.c \
          "$(FLOW)/Core/Src/bridge.c" test_ext.c

PWR_SRC = $(filter-out test_main.c,$(SRC)) test_power.c
LOG_SRC = $(filter-out test_main.c,$(SRC)) $(APP)/log/log_store.c \
          $(APP)/log/log_flash_ram.c $(APP)/drivers/w25q/w25q.c $(APP)/app/log_task.c $(APP)/telemetry/usb_link.c test_log.c

all: run
host_test: $(SRC) $(wildcard $(APP)/*/*.h)
	$(CC) $(CFLAGS) $(INC) $(SRC) -lm -o $@
run: host_test ext_test power_test log_test
	./host_test
	./ext_test
	./power_test
	./log_test
log_test: $(LOG_SRC) $(wildcard $(APP)/*/*.h)
	$(CC) $(CFLAGS) $(INC) $(LOG_SRC) -lm -o $@
power_test: $(PWR_SRC) $(wildcard $(APP)/*/*.h)
	$(CC) $(CFLAGS) $(INC) $(PWR_SRC) -lm -o $@
ext_test:
	$(CC) $(CFLAGS) $(INC) -I"$(FLOW)/Core/Inc" $(EXT_SRC) -lm -o $@
clean:
	rm -f host_test ext_test power_test log_test frames.bin power_frames.bin log_sample.bin
.PHONY: all run clean ext_test
