# CSP / CSV / CST 操作说明（joint-sdk 诊断程序）

面向本机 **ISVD90RC** + IgH EtherCAT Master + `joint-sdk` 诊断示例。  
本文档位于 `joint-sdk/`，以下命令均在该目录下执行。

相关文档：
- 架构：[`ARCHITECTURE.zh-CN.md`](ARCHITECTURE.zh-CN.md)
- 协议手册：[`守护兽驱动ETHERCAT协议手册.md`](守护兽驱动ETHERCAT协议手册.md)

---

## 0. 每次上电 / 主站异常时先做这些

```bash
cd joint-sdk   # 若已在本目录可省略

sudo /usr/local/sbin/ethercatctl restart
sudo chmod 666 /dev/EtherCAT0

ethercat slaves
# 应看到类似：0  ...  PREOP/OP  +  ISVD90RC-...
```

编译（改过代码才需要）：

```bash
make diag dual csv csv-dual cst cst-dual dc-setup ETHERLAB_DIR=/usr/local
```

程序位置：

| 模式 | 可执行文件 |
|------|------------|
| CSP 单轴 | `./build/diag_csp_single` |
| CSP 双轴 | `./build/diag_csp_dual` |
| CSV 单轴 | `./build/diag_csv_single` |
| CSV 双轴 | `./build/diag_csv_dual` |
| CST 单轴 | `./build/diag_cst_single` |
| CST 双轴 | `./build/diag_cst_dual` |
| DC 时序配置 | `./build/dc_timing_setup` |

**注意：**

- 运行诊断程序时不要同时开另一个占用 `/dev/EtherCAT0` 的进程。
- `sched_setscheduler: Operation not permitted` 可忽略（未拿到实时优先级）。
- 表中出现短暂 `unknown` / `al=04` 多为掉同步，一般会回到 `al=08`（OP）。
- **不要用** `ethercat download` 去改 PDO 里的 `0x6040/0x6060/0x607A/0x60FF/0x6071`；模式与目标由程序周期写。  
  SDK 会在 PREOP 自动写 `0x2002:1`、`0x6072`、限速等相关 SDO。

### 表头常见列

| 列 | 含义 |
|----|------|
| `mode` | `8`=CSP，`9`=CSV，`10`=CST |
| `wc` / `slv` / `al` | 工作计数、从站数、AL 状态；正常约 `wc=3`，`al=08`（OP） |
| `ok` / `PASS` | 程序自检是否在容差内 |

编码器口径：**约 16384 counts = 1 电机转**。

---

## 一、CSP — 周期同步位置（`diag_csp_single`）

### 1.1 手册 / TwinCAT 对应关系

| 步骤 | 对象 | 值 | 说明 |
|------|------|-----|------|
| 1 | `0x6060` | **8** | 周期同步位置 |
| 2 | `0x2002:1` | **3** | 滤波位置（POS_FILTER） |
| 3 | `0x6040` | 使能至 **0x000F** | 伺服运行 |
| 4 | `0x607A` | 目标位置 | counts；例 81920 ≈ 5 转 |

反馈：`0x6064` 实际位置，`0x606C` 实际速度，`0x6061` 模式显示应为 **8**。

> **现场说明：** TwinCAT 上原生 CSP 可用。IgH 侧若出现「目标钉住、actual 仍顶限速跑」，属已知问题；接口仍按上表使用。需要可靠走位时可暂用下文 CSV 做速度逼近，或继续对照 TwinCAT 的周期/Sync0 配置。

### 1.2 参数说明

| 参数 | 作用 |
|------|------|
| `--hold` | 停住：目标 = 使能瞬间位置 |
| `--delta N` | **相对**移动：目标 = home + N（**推荐**） |
| `--pos N` | **绝对**目标，直接写 `0x607A=N`（必须靠近当前 actual） |
| `--speed N` | 限速，写入 `0x6080`（默认约 30000） |
| `--seconds N` | 跑 N 秒后自动停 |
| `--log file.csv` | 每秒采样写日志 |

### 1.3 绝对值怎么取

**不要**随便写 `30000` 这类远离当前位置的数。

```bash
# 1) 先 hold，看打印的 home=
./build/diag_csp_single --hold --seconds 3

# 2) 假设 home=39704760，再绝对走位
./build/diag_csp_single --pos $((39704760 + 4096))  --speed 20000 --seconds 12   # ≈1/4 转
./build/diag_csp_single --pos $((39704760 + 16384)) --speed 20000 --seconds 12   # ≈1 转
./build/diag_csp_single --pos $((39704760 + 81920)) --speed 30000 --seconds 15   # ≈5 转（手册例）
```

| 想动多少 | 相对 home 大约加 |
|----------|------------------|
| ¼ 转 | `+4096` |
| ½ 转 | `+8192` |
| 1 转 | `+16384` |
| 5 转 | `+81920` |

更省事：直接用相对位移，不必手算绝对值：

```bash
./build/diag_csp_single --delta 16384 --speed 20000 --seconds 12
```

### 1.4 推荐操作顺序

```bash
sudo /usr/local/sbin/ethercatctl restart
sudo chmod 666 /dev/EtherCAT0
make diag ETHERLAB_DIR=/usr/local

# 先确认能站住
./build/diag_csp_single --hold --seconds 6

# 相对小步 → 再大一点
./build/diag_csp_single --delta 4096  --speed 20000 --seconds 10
./build/diag_csp_single --delta 16384 --speed 30000 --seconds 12
```

### 1.5 一句话备忘（CSP）

```bash
./build/diag_csp_single --hold --seconds 6
./build/diag_csp_single --delta 16384 --speed 30000 --seconds 12
```

---

## 二、CSV — 周期同步速度（`diag_csv_single`）

### 2.1 模式对应

| 项 | 值 |
|----|-----|
| `0x6060` | **9**（CSV） |
| `0x2002:1` | SDK 默认 **PASSTHROUGH(1)**（周期发速度用） |
| 目标 PDO | `0x60FF` |
| 反馈 | `0x606C` 实际速度（counts/s 量级） |

### 2.2 现场标定（重要）

本机实测：

```text
act_vel ≈ 20.5 × (写入 0x60FF 的值)
```

因此：

- **不要**写 `--vel 50000`（会被当成极大指令 → 顶限速飞车，常见 `act≈±30万`）。
- **推荐**用 `--rps`：程序按标定系数换算后再写 `0x60FF`。

要约 **1 转/秒**（期望 `act≈16384`）：

```bash
./build/diag_csv_single --rps 1 --seconds 10
# 内部约写 0x60FF ≈ 799
```

### 2.3 参数说明

| 参数 | 作用 |
|------|------|
| `--hold` | 目标速度 = 0（零速保持）**先跑这个** |
| `--rps N` | **推荐** 目标 N 转/秒（已按 20.5 标定换算） |
| `--vel N` | 直接写 `0x60FF=N`（需自己懂量纲） |
| `--turns N` | 直接写整数 N 到 `0x60FF`（探测用） |
| `--invert` | PREOP 写 `0x607E` 速度极性反相 |
| `--vmax N` | 可选，PREOP 写 `0x6080` |
| `--seconds N` | 自动停 |
| `--log file.csv` | 写日志 |

### 2.4 推荐操作顺序

```bash
make csv ETHERLAB_DIR=/usr/local

# 1) 零速
./build/diag_csv_single --hold --seconds 5

# 2) 1 转/秒（已验证可用）
./build/diag_csv_single --rps 1 --seconds 10 --log build/csv_logs/csv_rps1.csv

# 3) 其它转速
./build/diag_csv_single --rps 0.5 --seconds 10
./build/diag_csv_single --rps 2   --seconds 10
```

方向反了再加 `--invert`。

### 2.5 一句话备忘（CSV）

```bash
./build/diag_csv_single --hold --seconds 5
./build/diag_csv_single --rps 1 --seconds 10
```

---

## 三、CST — 周期同步力矩（`diag_cst_single`）

### 3.1 模式对应

| 项 | 值 |
|----|-----|
| `0x6060` | **10**（CST） |
| `0x2002:1` | SDK 默认 **PASSTHROUGH(1)** |
| 目标 PDO | `0x6071`（0.1% 额定；额定 10 N·m 时 **100 = 1 N·m**） |
| 反馈 | `0x6077`（量纲可能与目标不一致，先看是否限速） |
| 限速 | SDK 对 CST 打开 `0x200A:12`，并设默认 `0x6080≈80000` |

**恒力矩会持续加速**，靠 `0x6080` 把速度顶在限速附近（本机约 7～8 万），避免飞到 ±30 万。

### 3.2 参数说明

| 参数 | 作用 |
|------|------|
| `--hold` | **CSV 零速制动**（从高速刹停；不是纯 CST 力矩=0） |
| `--nm X` | **推荐** 目标 X N·m → `0x6071 = X*100`（按 10 N·m 额定） |
| `--trq N` | 直接写 `0x6071=N` |
| `--vmax N` | 更严速度上限（写入 `0x6080`） |
| `--seconds N` | 自动停 |
| `--log file.csv` | 写日志 |

### 3.3 推荐操作顺序

```bash
make cst ETHERLAB_DIR=/usr/local

# 1) 先刹停
./build/diag_cst_single --hold --seconds 8

# 2) 小力矩（已验证 0.2～1.0 N·m 可 PASS）
./build/diag_cst_single --nm 0.3 --seconds 8 --log build/csv_logs/cst_nm03.csv
./build/diag_cst_single --nm 0.5 --seconds 8
./build/diag_cst_single --nm 1.0 --seconds 8

# 3) 测完再 hold 刹停
./build/diag_cst_single --hold --seconds 5
```

判定要点：`mode=10`，`act_vel` 顶在限速附近且不飞车即可；`act_trq` 与 `tgt_trq` 数值对不上时可先不管。

### 3.4 一句话备忘（CST）

```bash
./build/diag_cst_single --hold --seconds 8
./build/diag_cst_single --nm 0.3 --seconds 8
```

---

## 四、三模式对照

| | CSP | CSV | CST |
|--|-----|-----|-----|
| 程序 | `diag_csp_single` | `diag_csv_single` | `diag_cst_single` |
| `0x6060` | 8 | 9 | 10 |
| `0x2002:1`（SDK） | **3** 滤波位置 | **1** 直通 | **1** 直通 |
| 目标对象 | `0x607A` 位置 | `0x60FF` 速度 | `0x6071` 力矩 |
| 推荐指令 | `--delta 16384` | `--rps 1` | `--nm 0.3` |
| 站住 | `--hold` | `--hold` | `--hold`（CSV 零速刹） |
| 禁忌 | 远离实际的 `--pos` | 大 `--vel`（如 50000） | 无限速的大力矩长时间跑 |

---

## 五、排障速查

| 现象 | 处理 |
|------|------|
| `Failed to reserve master` / Device busy | 结束其它 `diag_*`，或 `ethercatctl restart` |
| 整段 `al=04` / 进不了 OP | `sudo ethercatctl restart && sudo chmod 666 /dev/EtherCAT0` |
| CSV 飞到 ±30 万 | 改用 `--rps`，勿用大 `--vel` |
| CST 飞车 | 确认用的是当前 SDK（已开 `0x200A:12`）；先 `--hold` |
| CSP 目标不动、actual 狂跑 | IgH 侧原生 CSP 已知问题；核对 TwinCAT 周期/DC；或改用相对小 `--delta` 观察 |
| `chmod` / `ethercatctl` 要密码 | 用 `sudo`；Agent 无密码时需你在本机终端执行 |

---

## 六、快速复制区

```bash
sudo /usr/local/sbin/ethercatctl restart
sudo chmod 666 /dev/EtherCAT0
make diag dual csv csv-dual cst cst-dual ETHERLAB_DIR=/usr/local

# --- CSV ---
./build/diag_csv_single --hold --seconds 5
./build/diag_csv_single --rps 1 --seconds 10
sudo chrt -f 80 ./build/diag_csv_single --fw 8.1.44 --rps 5 --seconds 20

# --- CST（跟 drive_model；双轴在线时勿强求 bus al==08）---
./build/diag_cst_single --hold --seconds 8
./build/diag_cst_single --fw 8.1.44 --nm 0.3 --seconds 8
./build/diag_cst_single --fw 8.1.44 --nm 1 --vmax 50000 --seconds 10

# --- CSP ---
./build/diag_csp_single --hold --seconds 6
./build/diag_csp_single --delta 16384 --speed 30000 --seconds 12

# 带固件型号 / 实时调度
sudo chrt -f 80 ./build/diag_csp_single --fw 8.1.50 --delta 8192 --speed 4000 --seconds 15
sudo chrt -f 80 ./build/diag_csp_single --fw 8.1.50 --hold --seconds 5

# 双轴 CSV（未写 --seconds 时默认约 12 秒）
sudo chrt -f 80 ./build/diag_csv_dual --fw0 8.1.50 --fw1 8.1.60 --rps 2 --seconds 14
sudo chrt -f 80 ./build/diag_cst_dual --fw0 8.1.50 --fw1 8.1.44 --nm 1 --vmax 50000 --seconds 10
```

---

## 七、双轴 DC 时序（与运动指令分离）

DC 在 **configure / PREOP→SAFEOP** 下发（`ecrt_slave_config_dc`），**不要**写进 `diag_csp/csv/cst_*` 运动命令行。

| 步骤 | 做什么 |
|------|--------|
| 1 | `./build/dc_timing_setup --shared …` 或 `--per-joint --fw …` |
| 2 | 再跑 `diag_*_dual` / single（只负责运动） |

没有 conf 文件时：用 `JSDK_DC_TIMING_MODE_DEFAULT` + `src/drive_model.c`。

### 代码默认开关

文件：`include/joint_sdk/drive_model.h`

```c
#define JSDK_DC_TIMING_SHARED     0   /* 相同 DC：整总线一套 */
#define JSDK_DC_TIMING_PER_JOINT  1   /* 不同 DC：每轴跟固件表/配置段 */
#define JSDK_DC_TIMING_MODE_DEFAULT JSDK_DC_TIMING_SHARED
```

改默认后需 `make`。表内默认数值在 `src/drive_model.c`。

### 单独配置命令（推荐）：`dc_timing_setup`

先配置、再跑运动。配置写入 `config/dc_timing.conf`（可用环境变量 `JSDK_DC_TIMING_CONF` 改路径）。运动程序 `create` 时自动读入。

```bash
make dc-setup

# 查看当前配置 / 代码默认
./build/dc_timing_setup --show

# --- 相同 DC（SHARED）---
./build/dc_timing_setup --shared \
  --period 1000000 --sync0-cycle 1000000 \
  --sync0-shift 250000 --wait-ms 100

# --- 不同 DC（PER_JOINT）：每个固件写一次 ---
./build/dc_timing_setup --per-joint --period 1000000
./build/dc_timing_setup --per-joint --fw 8.1.50 \
  --sync0-cycle 1000000 --sync0-shift 250000 --wait-ms 100
./build/dc_timing_setup --per-joint --fw 8.1.60 \
  --sync0-cycle 1000000 --sync0-shift 0 --wait-ms 100

# 然后再跑运动（不要带 DC 参数）
sudo chrt -f 80 ./build/diag_csp_dual --fw0 8.1.50 --fw1 8.1.60 \
  --no-invert0 --invert1 --delta 18192 --seconds 20
```
