# 守护兽关节 单轴 CSP PoC

验证 IgH EtherCAT Master 与本公司机器人关节（CyberBeast FL90BLW14）的
CiA402 周期同步位置（CSP）交互链路。

> 仅用于 PoC / 联调，非生产代码。文件：[csp_single.c](csp_single.c)

## 这个 PoC 做了什么

1. 申请 master0、创建 domain。
2. 按 ESI 校验从站身份：vendor `0x000C0B00` / product `0x00080153`。
3. 配置 **0x1600 RxPDO + 0x1A00 TxPDO** 全功能映射（含 ESI 中的 8 位对齐填充）。
4. 配置期 SDO 预置 0x6060 = CSP。
5. 配置 **DC**：AssignActivate=`0x300`，SYNC0=周期，shift=周期/4。
6. 激活后进入 1 kHz 实时循环：
   - 自动引导 **DS402 状态机**至 Operation Enabled（含故障自动复位）；
   - 使能后下发正弦 **目标位置 0x607A**，读回 **实际位置 0x6064**；
   - 每周期维护 DC 时钟同步；
   - 1 Hz 打印 status / mode / 实际位置诊断。
7. Ctrl-C 退出时清控制字停机并释放 master。

## ⚠ 联调前务必确认/调整

- **0x6060 模式编码**：代码用 CiA402 标准 `8=CSP`。若实物不同，改 `MODE_CSP`。
- **轨迹幅值/换算因子**：`TRAJ_AMPLITUDE` 暂用默认编码器分辨率 16384，按实际换算因子调整，**首次务必空载、小幅值、低频测试**。
- **周期**：默认 1 ms，远高于设备最小 DC 周期（单轴≈41 µs），稳定后可缩短。
- **alias/position**：默认 `0/0`（环上第一个从站）。多轴或带耦合器时需调整。

## 构建

需已安装 IgH EtherCAT Master 的用户库与头文件。

```bash
# 按实际安装前缀修改（configure 默认 /opt/etherlab）
make ETHERLAB_DIR=/opt/etherlab
```

## 运行

```bash
# 确认主站内核模块已加载、网卡已绑定
sudo /etc/init.d/ethercat start    # 或 systemctl start ethercat
ethercat slaves                    # 应能看到 FL90BLW14，状态 PREOP

# 需要 root（实时优先级 + mlockall + 访问 /dev/EtherCAT0）
sudo ./csp_single
```

正常时应依次看到：`1 slave(s) responding` → `AL states: 0x08`(OP) →
`Joint: operational` → `Enabled. Home position = ...`，电机开始小幅正弦摆动。

## 排障要点

- **看不到从站 / WC 不增长**：检查网卡绑定（`ethercat master`）、网线、从站供电。
- **卡在 SAFEOP、报 0x1A 同步错误**：DC 配置问题。确认 shift=周期/4，主站周期抖动是否过大；
  必要时换更实时的内核（见架构文档第 12 节 D3）。
- **status 一直 0x0008（故障）且复位无效**：读 0x603F / 0x203F 看具体故障码（用 `ethercat upload`）。
- **模式不切换（mode_disp ≠ 8）**：确认 0x6060 模式编码值是否与本设备一致。
