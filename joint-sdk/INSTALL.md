# 安装与依赖

## 1. 依赖

### 1.1 IgH EtherCAT Master

本 SDK 基于 IgH EtherCAT Master 用户空间库。需要先编译安装：

```bash
git clone https://gitlab.com/etherlab.org/ethercat.git
cd ethercat
./bootstrap
./configure --sysconfdir=/etc --enable-generic   # 按需加 --enable-e1000e 等
make all modules
sudo make modules_install install
sudo depmod
```

安装后确认：

```bash
ls /opt/etherlab/lib/libethercat.so       # 库（默认前缀 /opt/etherlab）
ls /opt/etherlab/include/ecrt.h           # 头文件
```

若安装前缀不同（如 `/usr/local`），编译 SDK 时指定 `ETHERLAB_DIR`。

### 1.2 内核与实时性

| 场景 | 建议 |
|------|------|
| 仅验证链路 | 普通内核即可 |
| 稳定 1 kHz DC | **PREEMPT_RT** 内核 + Intel 独立网卡（i210 等，走原生驱动） |
| 多轴高频 | PREEMPT_RT + BIOS 关 C-State/SpeedStep + CPU 隔离 |

### 1.3 网卡绑定

EtherCAT 网卡必须**独占**（不接普通网络）：

```bash
sudo /etc/init.d/ethercat start    # 或 systemctl start ethercat
ethercat slaves                     # 应能看到守护兽从站
```

## 2. 编译

```bash
cd joint-sdk
make ETHERLAB_DIR=/opt/etherlab
```

产物在 `build/` 下：

| 文件 | 说明 |
|------|------|
| `libjointsdk.a` | 静态库 |
| `libjointsdk.so` | 动态库 |
| `csp_single_sdk` | 单轴 CSP 示例 |
| `diag_csp_single` / `diag_csv_single` / `diag_cst_single` | 三模式诊断 |
| `sdo_diag` | 异步 SDO 诊断 |
| `fault_diag` | 故障诊断 |
| `phys_csp_single` | 物理单位示例 |
| `multi_axis_csp` | 多轴示例 |
| `cpp_group_csp` | C++ 外观层示例 |

## 3. 安装

```bash
sudo make install PREFIX=/usr/local ETHERLAB_DIR=/opt/etherlab
```

安装内容：

| 文件 | 位置 |
|------|------|
| `libjointsdk.so` / `libjointsdk.a` | `$(PREFIX)/lib` |
| `joint_sdk.h` / `cia402.h` / `joint_group.hpp` | `$(PREFIX)/include/joint_sdk` |
| `joint-sdk.pc` | `$(PREFIX)/lib/pkgconfig` |

卸载：`sudo make uninstall PREFIX=/usr/local`

## 4. 在客户项目中使用

### 方式 A：pkg-config

```bash
# 编译客户程序
gcc my_app.c $(pkg-config --cflags --libs joint-sdk) -o my_app
```

### 方式 B：手动链接

```bash
gcc my_app.c -I/usr/local/include -L/usr/local/lib -ljointsdk -lethercat -lm -o my_app
```

C++ 项目额外加 `-pthread`。

### 方式 C：直接引用 SDK 源码目录

```bash
gcc my_app.c -Ijoint-sdk/include -Ljoint-sdk/build -ljointsdk -lethercat -lm -o my_app
```

## 5. ESI 文件约定

- 守护兽各型号 ESI 文件（如 `ECAT_CIA402.xml`）由公司提供。
- 客户运行目录需能访问 ESI 文件，示例中使用相对路径 `./ECAT_CIA402.xml`。
- 也可以使用内置 profile 名 `JSDK_PROFILE_CYBERBEAST_JOINT_MODULE`（编译期静态 profile，无需文件）。
