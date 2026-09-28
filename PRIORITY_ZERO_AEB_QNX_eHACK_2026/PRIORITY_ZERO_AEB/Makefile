# =============================================================================
#  Makefile  —  Autonomous Emergency Braking Controller
#  Team PRIORITY ZERO · QNX eHACK 2026 · Problem Statement #2
#
#  BUILD (on a host with QNX SDP 8.0 installed)
#      source ~/qnx800/qnxsdp-env.sh
#      make                 # builds both aeb and hwtest
#      make aeb             # controller only
#      make hwtest          # hardware bring-up tester only
#      make clean
#
#  DEPLOY
#      make deploy TARGET=192.168.1.50
#
#  Default target architecture is aarch64 (Raspberry Pi 5 / Pi 4 64-bit).
#  For a 32-bit image use:   make ARCH=gcc_ntoarmv7le
# =============================================================================

ARCH    ?= gcc_ntoaarch64le
TARGET  ?= 192.168.1.50
DESTDIR ?= /data/home/root

CC      := qcc
CFLAGS  := -V$(ARCH) -O2 -Wall -Wextra -std=gnu11 -D_QNX_SOURCE
LDFLAGS := -V$(ARCH)
LIBS    := -lm

SRC_DIR   := src
TOOLS_DIR := tools
BIN_DIR   := bin

AEB_BIN    := $(BIN_DIR)/aeb
HWTEST_BIN := $(BIN_DIR)/hwtest

.PHONY: all aeb hwtest clean deploy help

all: aeb hwtest

aeb: $(AEB_BIN)
hwtest: $(HWTEST_BIN)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(AEB_BIN): $(SRC_DIR)/aeb.c | $(BIN_DIR)
	$(CC) $(CFLAGS) $< -o $@ $(LIBS)
	@echo "built  $@   (AEB controller — 8 SCHED_FIFO threads)"

$(HWTEST_BIN): $(TOOLS_DIR)/hwtest.c | $(BIN_DIR)
	$(CC) $(CFLAGS) $< -o $@ $(LIBS)
	@echo "built  $@   (component bring-up tester)"

deploy: all
	scp $(AEB_BIN) $(HWTEST_BIN) root@$(TARGET):$(DESTDIR)/
	@echo "deployed to root@$(TARGET):$(DESTDIR)/"

clean:
	rm -rf $(BIN_DIR) aeb_log.csv
	@echo "cleaned"

help:
	@echo "make            build aeb + hwtest"
	@echo "make aeb        build the controller only"
	@echo "make hwtest     build the hardware tester only"
	@echo "make deploy TARGET=<ip>   build and scp to the board"
	@echo "make clean      remove binaries and logs"
	@echo "make ARCH=gcc_ntoarmv7le  build for 32-bit targets"
