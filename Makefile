# ════════════════════════════════════════════════════════════════════════════
# Modbus TCP/RTU library – transport + application API layers.
#
# Usage:
#   make                 → build library for ARM (default)
#   make ARCH=arm        → build library for ARM
#   make ARCH=x86        → build library for x86 (native)
#
#   make examples        → build example binaries for selected ARCH
#   make test            → run end-to-end tests (x86 only; ARM binaries cannot
#                          run on an x86 host without QEMU)
#   make clean           → remove build outputs for the selected ARCH
#   make clean-all       → remove all build outputs (all architectures)
# ════════════════════════════════════════════════════════════════════════════

# ── Architecture selection ─────────────────────────────────────────────────
# Override on the command line:  make ARCH=x86
ARCH ?= arm

ifeq ($(ARCH), arm)
    CROSS_PREFIX := aarch64-linux-gnu-
    CC           := $(CROSS_PREFIX)gcc
    AR           := $(CROSS_PREFIX)ar
else ifeq ($(ARCH), x86)
    CROSS_PREFIX :=
    CC           := gcc
    AR           := ar
else
    $(error Unknown ARCH="$(ARCH)". Supported values: arm  x86)
endif

$(info [Makefile] ARCH=$(ARCH)   CC=$(CC))

# ── Compiler flags ─────────────────────────────────────────────────────────
CFLAGS   := -Wall -Wextra -std=c11 -O2 -fPIC
CPPFLAGS := -Iinclude

# ── Output directories (separated per architecture) ───────────────────────
BUILD_DIR := build/$(ARCH)
LIB_NAME  := modbus_protocol
LIB       := $(BUILD_DIR)/lib$(LIB_NAME).a

# ── Source files and their compiled objects ────────────────────────────────
# Transport layer (low-level TCP, framing helpers)
SRC_TRANSPORT := src/modbus_tcp.c
OBJ_TRANSPORT := $(BUILD_DIR)/modbus_tcp.o

# Application API: server (slave) role
SRC_SERVER    := src/modbus_tcp_server.c
OBJ_SERVER    := $(BUILD_DIR)/modbus_tcp_server.o

# Application API: client (master) role
SRC_CLIENT    := src/modbus_tcp_client.c
OBJ_CLIENT    := $(BUILD_DIR)/modbus_tcp_client.o

# RTU transport and application APIs
SRC_RTU_TRANSPORT := src/modbus_rtu.c
OBJ_RTU_TRANSPORT := $(BUILD_DIR)/modbus_rtu.o
SRC_RTU_SERVER    := src/modbus_rtu_server.c
OBJ_RTU_SERVER    := $(BUILD_DIR)/modbus_rtu_server.o
SRC_RTU_CLIENT    := src/modbus_rtu_client.c
OBJ_RTU_CLIENT    := $(BUILD_DIR)/modbus_rtu_client.o

ALL_OBJS := $(OBJ_TRANSPORT) $(OBJ_SERVER) $(OBJ_CLIENT) \
            $(OBJ_RTU_TRANSPORT) $(OBJ_RTU_SERVER) $(OBJ_RTU_CLIENT)

# ── Common headers every translation unit depends on ──────────────────────
HDRS_COMMON := include/modbus_defines.h

# ── Test and example paths ─────────────────────────────────────────────────
TEST_DIR := test
TEST_SRC := $(TEST_DIR)/test_modbus_tcp.c
TEST_BIN := $(BUILD_DIR)/test_modbus_tcp
TEST_RTU_SRC := $(TEST_DIR)/test_modbus_rtu.c
TEST_RTU_BIN := $(BUILD_DIR)/test_modbus_rtu

EXAMPLE_DIR    := examples
EXAMPLE_SERVER := $(BUILD_DIR)/tcp_server_example
EXAMPLE_CLIENT := $(BUILD_DIR)/tcp_client_example
EXAMPLE_RTU_SERVER := $(BUILD_DIR)/rtu_server_example
EXAMPLE_RTU_CLIENT := $(BUILD_DIR)/rtu_client_example

# ════════════════════════════════════════════════════════════════════════════
.PHONY: all test examples clean clean-all info

all: $(LIB)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# ── Library objects ────────────────────────────────────────────────────────
$(OBJ_TRANSPORT): $(SRC_TRANSPORT) include/modbus_tcp.h $(HDRS_COMMON) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $(SRC_TRANSPORT) -o $(OBJ_TRANSPORT)

$(OBJ_SERVER): $(SRC_SERVER) include/modbus_tcp_server.h include/modbus_tcp.h $(HDRS_COMMON) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $(SRC_SERVER) -o $(OBJ_SERVER)

$(OBJ_CLIENT): $(SRC_CLIENT) include/modbus_tcp_client.h include/modbus_tcp.h $(HDRS_COMMON) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $(SRC_CLIENT) -o $(OBJ_CLIENT)

$(OBJ_RTU_TRANSPORT): $(SRC_RTU_TRANSPORT) include/modbus_rtu.h $(HDRS_COMMON) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $(SRC_RTU_TRANSPORT) -o $(OBJ_RTU_TRANSPORT)

$(OBJ_RTU_SERVER): $(SRC_RTU_SERVER) include/modbus_rtu_server.h include/modbus_rtu.h $(HDRS_COMMON) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $(SRC_RTU_SERVER) -o $(OBJ_RTU_SERVER)

$(OBJ_RTU_CLIENT): $(SRC_RTU_CLIENT) include/modbus_rtu_client.h include/modbus_rtu.h $(HDRS_COMMON) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $(SRC_RTU_CLIENT) -o $(OBJ_RTU_CLIENT)

$(LIB): $(ALL_OBJS)
	$(AR) rcs $(LIB) $(ALL_OBJS)
	@echo "[Makefile] Library ready: $(LIB)"

# ── Test binary ────────────────────────────────────────────────────────────
$(TEST_BIN): $(TEST_SRC) $(LIB) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(TEST_SRC) -L$(BUILD_DIR) -l$(LIB_NAME) -lpthread -o $(TEST_BIN)

$(TEST_RTU_BIN): $(TEST_RTU_SRC) $(LIB) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(TEST_RTU_SRC) -L$(BUILD_DIR) -l$(LIB_NAME) -lpthread -o $(TEST_RTU_BIN)

test: $(TEST_BIN) $(TEST_RTU_BIN)
	@if [ "$(ARCH)" = "arm" ]; then \
	    echo "[Makefile] NOTE: ARM binaries cannot run on x86 host."; \
	    echo "           Use 'make test ARCH=x86' to execute tests locally."; \
	    echo "           For ARM testing, deploy to target board and run $(TEST_BIN) / $(TEST_RTU_BIN)"; \
	else \
	    echo "Running TCP validation ($(ARCH))..."; \
	    $(TEST_BIN) && \
	    echo "Running RTU validation ($(ARCH))..." && \
	    $(TEST_RTU_BIN) && echo "All tests passed." || echo "Some tests FAILED."; \
	fi

# ── Example binaries ───────────────────────────────────────────────────────
$(EXAMPLE_SERVER): $(EXAMPLE_DIR)/tcp_server_example.c $(LIB) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $< -L$(BUILD_DIR) -l$(LIB_NAME) -lpthread -o $@

$(EXAMPLE_CLIENT): $(EXAMPLE_DIR)/tcp_client_example.c $(LIB) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $< -L$(BUILD_DIR) -l$(LIB_NAME) -lpthread -o $@

$(EXAMPLE_RTU_SERVER): $(EXAMPLE_DIR)/rtu_server_example.c $(LIB) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $< -L$(BUILD_DIR) -l$(LIB_NAME) -lpthread -o $@

$(EXAMPLE_RTU_CLIENT): $(EXAMPLE_DIR)/rtu_client_example.c $(LIB) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $< -L$(BUILD_DIR) -l$(LIB_NAME) -lpthread -o $@

examples: $(EXAMPLE_SERVER) $(EXAMPLE_CLIENT) $(EXAMPLE_RTU_SERVER) $(EXAMPLE_RTU_CLIENT)
	@echo "[Makefile] Examples ready ($(ARCH)):"
	@echo "  $(EXAMPLE_SERVER)  – Modbus TCP server (slave)"
	@echo "  $(EXAMPLE_CLIENT)  – Modbus TCP client (master)"
	@echo "  $(EXAMPLE_RTU_SERVER)  – Modbus RTU server (slave)"
	@echo "  $(EXAMPLE_RTU_CLIENT)  – Modbus RTU client (master)"

# ── Clean ──────────────────────────────────────────────────────────────────
clean:
	rm -rf $(BUILD_DIR)
	@echo "[Makefile] Cleaned build/$(ARCH)/"

clean-all:
	rm -rf build/
	@echo "[Makefile] Cleaned build/ (all architectures)"

# ── Info (show current configuration) ─────────────────────────────────────
info:
	@echo "ARCH         = $(ARCH)"
	@echo "CC           = $(CC)"
	@echo "AR           = $(AR)"
	@echo "CFLAGS       = $(CFLAGS)"
	@echo "BUILD_DIR    = $(BUILD_DIR)"
	@echo "LIB          = $(LIB)"
