ETHERLAB_DIR ?= /usr/local

CC      ?= gcc
AR      ?= ar
CFLAGS  ?= -O2 -Wall -Wextra -g -std=c99
CPPFLAGS = -Iinclude -I$(ETHERLAB_DIR)/include
LDFLAGS  = -L$(ETHERLAB_DIR)/lib -Wl,-rpath,$(ETHERLAB_DIR)/lib
LDLIBS   = -lethercat -lm

BUILD_DIR = build
LIB_STATIC = $(BUILD_DIR)/libjointsdk.a
LIB_SHARED = $(BUILD_DIR)/libjointsdk.so
EXAMPLE = $(BUILD_DIR)/csp_single_sdk
DIAG_EXAMPLE = $(BUILD_DIR)/diag_csp_single
DIAG_DUAL_EXAMPLE = $(BUILD_DIR)/diag_csp_dual
CSV_DUAL_EXAMPLE = $(BUILD_DIR)/diag_csv_dual
CSV_EXAMPLE = $(BUILD_DIR)/diag_csv_single
CST_EXAMPLE = $(BUILD_DIR)/diag_cst_single
CST_DUAL_EXAMPLE = $(BUILD_DIR)/diag_cst_dual
SDO_EXAMPLE = $(BUILD_DIR)/sdo_diag
FAULT_EXAMPLE = $(BUILD_DIR)/fault_diag
PHYS_EXAMPLE = $(BUILD_DIR)/phys_csp_single

SDK_SRCS = \
	src/transport.c \
	src/cia402.c \
	src/profile_cyberbeast_joint_module.c \
	src/esi_parser.c \
	src/drive_model.c \
	src/dc_timing_conf.c
SDK_OBJS = $(patsubst src/%.c,$(BUILD_DIR)/%.o,$(SDK_SRCS))

.PHONY: all clean example diag dual csv csv-dual cst cst-dual sdo fault phys dc-setup

all: $(LIB_STATIC) $(LIB_SHARED) example diag dual csv csv-dual cst cst-dual sdo fault phys dc-setup

example: $(EXAMPLE)
diag: $(DIAG_EXAMPLE)
dual: $(DIAG_DUAL_EXAMPLE)
csv: $(CSV_EXAMPLE)
csv-dual: $(CSV_DUAL_EXAMPLE)
cst: $(CST_EXAMPLE)
cst-dual: $(CST_DUAL_EXAMPLE)
sdo: $(SDO_EXAMPLE)
fault: $(FAULT_EXAMPLE)
phys: $(PHYS_EXAMPLE)
dc-setup: $(BUILD_DIR)/dc_timing_setup

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -fPIC -c -o $@ $<

$(LIB_STATIC): $(SDK_OBJS)
	$(AR) rcs $@ $^

$(LIB_SHARED): $(SDK_OBJS)
	$(CC) -shared -o $@ $^ $(LDFLAGS) $(LDLIBS)

$(EXAMPLE): examples/csp_single_sdk.c $(LIB_STATIC)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(LIB_STATIC) $(LDFLAGS) $(LDLIBS)

$(DIAG_EXAMPLE): examples/diag_csp_single.c $(LIB_STATIC)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(LIB_STATIC) $(LDFLAGS) $(LDLIBS)

$(DIAG_DUAL_EXAMPLE): examples/diag_csp_dual.c $(LIB_STATIC)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(LIB_STATIC) $(LDFLAGS) $(LDLIBS)

$(CSV_DUAL_EXAMPLE): examples/diag_csv_dual.c $(LIB_STATIC)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(LIB_STATIC) $(LDFLAGS) $(LDLIBS)

$(CSV_EXAMPLE): examples/diag_csv_single.c $(LIB_STATIC)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(LIB_STATIC) $(LDFLAGS) $(LDLIBS)

$(CST_EXAMPLE): examples/diag_cst_single.c $(LIB_STATIC)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(LIB_STATIC) $(LDFLAGS) $(LDLIBS)

$(CST_DUAL_EXAMPLE): examples/diag_cst_dual.c $(LIB_STATIC)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(LIB_STATIC) $(LDFLAGS) $(LDLIBS)

$(SDO_EXAMPLE): examples/sdo_diag.c $(LIB_STATIC)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(LIB_STATIC) $(LDFLAGS) $(LDLIBS)

$(FAULT_EXAMPLE): examples/fault_diag.c $(LIB_STATIC)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(LIB_STATIC) $(LDFLAGS) $(LDLIBS)

$(PHYS_EXAMPLE): examples/phys_csp_single.c $(LIB_STATIC)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(LIB_STATIC) $(LDFLAGS) $(LDLIBS) -lm

$(BUILD_DIR)/dc_timing_setup: examples/dc_timing_setup.c $(LIB_STATIC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(LIB_STATIC) $(LDFLAGS) $(LDLIBS)

clean:
	rm -rf $(BUILD_DIR)
