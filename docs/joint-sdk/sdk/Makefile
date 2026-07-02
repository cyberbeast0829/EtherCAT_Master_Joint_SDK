ETHERLAB_DIR ?= /opt/etherlab

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
CSV_EXAMPLE = $(BUILD_DIR)/diag_csv_single
CST_EXAMPLE = $(BUILD_DIR)/diag_cst_single
SDO_EXAMPLE = $(BUILD_DIR)/sdo_diag

SDK_SRCS = \
	src/transport.c \
	src/cia402.c \
	src/profile_cyberbeast_joint_module.c \
	src/esi_parser.c
SDK_OBJS = $(patsubst src/%.c,$(BUILD_DIR)/%.o,$(SDK_SRCS))

.PHONY: all clean example diag csv cst sdo

all: $(LIB_STATIC) $(LIB_SHARED) example diag csv cst sdo

example: $(EXAMPLE)
diag: $(DIAG_EXAMPLE)
csv: $(CSV_EXAMPLE)
cst: $(CST_EXAMPLE)
sdo: $(SDO_EXAMPLE)

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

$(CSV_EXAMPLE): examples/diag_csv_single.c $(LIB_STATIC)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(LIB_STATIC) $(LDFLAGS) $(LDLIBS)

$(CST_EXAMPLE): examples/diag_cst_single.c $(LIB_STATIC)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(LIB_STATIC) $(LDFLAGS) $(LDLIBS)

$(SDO_EXAMPLE): examples/sdo_diag.c $(LIB_STATIC)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(LIB_STATIC) $(LDFLAGS) $(LDLIBS)

clean:
	rm -rf $(BUILD_DIR)
