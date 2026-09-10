ETHERLAB_DIR ?= /opt/etherlab
PREFIX ?= /usr/local
VERSION = 0.5.0

CC      ?= gcc
CXX     ?= g++
AR      ?= ar
CFLAGS  ?= -O2 -Wall -Wextra -g -std=c99
CXXFLAGS ?= -O2 -Wall -Wextra -g -std=c++11
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
MULTI_EXAMPLE = $(BUILD_DIR)/multi_axis_csp
CPP_EXAMPLE = $(BUILD_DIR)/cpp_group_csp

SDK_SRCS = \
	src/transport.c \
	src/cia402.c \
	src/profile_cyberbeast_joint_module.c \
	src/esi_parser.c \
	src/drive_model.c \
	src/dc_timing_conf.c
SDK_OBJS = $(patsubst src/%.c,$(BUILD_DIR)/%.o,$(SDK_SRCS))

.PHONY: all clean example diag dual csv csv-dual cst cst-dual sdo fault phys multi cpp dc-setup install uninstall

all: $(LIB_STATIC) $(LIB_SHARED) example diag dual csv csv-dual cst cst-dual sdo fault phys dc-setup multi cpp dc-setup

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
multi: $(MULTI_EXAMPLE)
cpp: $(CPP_EXAMPLE)
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

$(MULTI_EXAMPLE): examples/multi_axis_csp.c $(LIB_STATIC)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(LIB_STATIC) $(LDFLAGS) $(LDLIBS) -lm

$(CPP_EXAMPLE): examples/cpp_group_csp.cpp $(LIB_STATIC)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -o $@ $< $(LIB_STATIC) $(LDFLAGS) $(LDLIBS) -pthread

$(BUILD_DIR)/dc_timing_setup: examples/dc_timing_setup.c $(LIB_STATIC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(LIB_STATIC) $(LDFLAGS) $(LDLIBS)

# ---- 安装 ----

PC_FILE = $(BUILD_DIR)/joint-sdk.pc

$(PC_FILE): joint-sdk.pc.in | $(BUILD_DIR)
	sed -e 's|@PREFIX@|$(PREFIX)|g' -e 's|@VERSION@|$(VERSION)|g' $< > $@

install: $(LIB_STATIC) $(LIB_SHARED) $(PC_FILE)
	install -d $(DESTDIR)$(PREFIX)/lib
	install -d $(DESTDIR)$(PREFIX)/include/joint_sdk
	install -d $(DESTDIR)$(PREFIX)/lib/pkgconfig
	install -m 755 $(LIB_SHARED) $(DESTDIR)$(PREFIX)/lib/
	install -m 644 $(LIB_STATIC) $(DESTDIR)$(PREFIX)/lib/
	install -m 644 include/joint_sdk/joint_sdk.h $(DESTDIR)$(PREFIX)/include/joint_sdk/
	install -m 644 include/joint_sdk/cia402.h $(DESTDIR)$(PREFIX)/include/joint_sdk/
	install -m 644 include/joint_sdk/joint_group.hpp $(DESTDIR)$(PREFIX)/include/joint_sdk/
	install -m 644 $(PC_FILE) $(DESTDIR)$(PREFIX)/lib/pkgconfig/
	@echo "Installed joint-sdk $(VERSION) to $(DESTDIR)$(PREFIX)"

uninstall:
	rm -f $(DESTDIR)$(PREFIX)/lib/libjointsdk.so
	rm -f $(DESTDIR)$(PREFIX)/lib/libjointsdk.a
	rm -f $(DESTDIR)$(PREFIX)/lib/pkgconfig/joint-sdk.pc
	rm -rf $(DESTDIR)$(PREFIX)/include/joint_sdk
	@echo "Uninstalled joint-sdk"

clean:
	rm -rf $(BUILD_DIR)
