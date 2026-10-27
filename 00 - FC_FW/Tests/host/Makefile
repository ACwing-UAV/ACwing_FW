# Host tests of the protocol-v2 firmware modules (no HAL, no RTOS).
#   make        build and run
APP  = ../../App
SRC  = $(APP)/telemetry/telemetry.c $(APP)/telemetry/proto_v2.c $(APP)/telemetry/ack_cache.c \
       $(APP)/telemetry/app_link.c $(APP)/telemetry/tlm_ble.c \
       $(APP)/app/params.c $(APP)/app/cfg_store.c $(APP)/app/fc_shared.c $(APP)/app/app_task.c \
       $(APP)/control/fc_core.c $(APP)/control/calib_mgr.c $(APP)/control/checks.c \
       $(APP)/filters/altitude_kf/alt_est.c $(APP)/control/alt_ctrl.c \
       $(APP)/drivers/vl53l1x/vl53l1x.c $(APP)/drivers/flow_bridge/flow_bridge.c \
       host_stubs.c test_main.c
INC  = -I$(APP)/telemetry -I$(APP)/app -I$(APP)/control -I$(APP)/bsp -I$(APP)/filters/altitude_kf \
       -I$(APP)/drivers/common -I$(APP)/drivers/vl53l1x -I$(APP)/drivers/flow_bridge
CFLAGS = -std=c11 -O1 -g -Wall -Wextra -Wno-unused-parameter -DHOST_TEST \
         -DAPP_BLE_STATE_DECIM_DEFAULT=4 -fsanitize=address,undefined

all: run
host_test: $(SRC) $(wildcard $(APP)/*/*.h)
	$(CC) $(CFLAGS) $(INC) $(SRC) -lm -o $@
run: host_test
	./host_test
clean:
	rm -f host_test frames.bin
.PHONY: all run clean
