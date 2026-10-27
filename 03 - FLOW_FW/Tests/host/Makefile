# Host tests of the flow bridge: register map vs the FC driver, I2C target
# protocol, accumulation, PMW3901 sequences. make -> build and run.
FC  = ../../../00 - STM32_FW/App/drivers
SRC = ../../Core/Src/bridge.c ../../Core/Src/pmw3901.c test_bridge.c "$(FC)/flow_bridge/flow_bridge.c"
INC = -I../../Core/Inc -I"$(FC)/flow_bridge" -I"$(FC)/common"
CFLAGS = -std=c11 -O1 -g -Wall -Wextra -fsanitize=address,undefined

run: host_test
	./host_test
host_test:
	$(CC) $(CFLAGS) $(INC) $(SRC) -o $@
clean:
	rm -f host_test
.PHONY: run clean host_test   # paths with spaces: always rebuild (small)
