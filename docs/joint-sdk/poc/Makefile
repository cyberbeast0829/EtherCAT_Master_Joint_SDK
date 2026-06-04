# 守护兽关节单轴 CSP PoC — 构建
#
# 依赖已安装的 IgH EtherCAT Master 用户库 (libethercat) 与头文件 (ecrt.h)。
# 默认安装前缀为 /opt/etherlab 或 /usr/local，按实际修改 ETHERLAB_DIR。

ETHERLAB_DIR ?= /opt/etherlab

CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra -g
CPPFLAGS = -I$(ETHERLAB_DIR)/include
LDFLAGS  = -L$(ETHERLAB_DIR)/lib -Wl,-rpath,$(ETHERLAB_DIR)/lib
LDLIBS   = -lethercat -lm

TARGET  = csp_single
SRC     = csp_single.c

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $^ $(LDFLAGS) $(LDLIBS)

clean:
	rm -f $(TARGET)
