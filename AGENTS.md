# IgH EtherCAT Master — Agent Guide

Open-source **EtherCAT master** for Linux. C kernel modules + userspace C library + C++ CLI tool. See [README.md](README.md) and [FEATURES.md](FEATURES.md) for the project overview.

## Architecture

```
Kernel space                         User space
  master/   ec_master.ko  ◄── ioctl/mmap ──►  lib/  libethercat.so ──► apps (ecrt_* API)
    │  (/dev/EtherCATn, char dev)                    └── tool/ ethercat (C++ CLI)
  devices/  ec_device_*.ko (NIC frame I/O)
```

- **master/** — kernel master module: bus scanning, datagrams, domains, mailbox protocols (CoE/SoE/FoE/EoE/VoE), realtime core.
- **lib/** — userspace C library wrapping the master via `ioctl` + `mmap` (`lib/master.c`).
- **tool/** — C++ `ethercat` CLI; one `Command*` subclass per subcommand, dispatched from `tool/main.cpp`, talks to the master through `tool/MasterDevice.cpp`.
- **devices/** — patched native Ethernet drivers (8139too, e100, e1000(e), r8169) plus the `generic` driver; `*-orig.c` is the unmodified kernel source, `*-ethercat.c` the EtherCAT port. Keep diffs minimal between the two.
- **include/** — public API: [include/ecrt.h](include/ecrt.h) (realtime application interface) and `include/ectty.h`. Treat these as a stable contract.
- Other dirs: `examples/`, `script/`, `tty/` (serial-over-EtherCAT), `fake_lib/` (dry-run mock, see [fake_lib/README.md](fake_lib/README.md)), `documentation/` (LaTeX handbook).

## Conventions

Follow [CodingStyle.md](CodingStyle.md) for all C **except** `devices/*` Ethernet drivers, which use **Linux kernel coding style** to minimize diffs. Key rules: max 78-char lines, 4-space indent (no tabs), K&R braces, no trailing whitespace, `do { } while (0)` for multi-statement macros, CAPITALS for macros/defines.

Naming (match existing code):
- `ec_` — kernel-internal types/functions (`ec_master_t`, `ec_slave_t`, `ec_datagram_t`).
- `ecrt_` — public realtime API in `ecrt.h` (`ecrt_master_activate()`, `ecrt_domain_process()`).
- `EC_` — macros/constants (`EC_MAX_FMMUS`, `EC_IOCTL_TYPE`).

**Finite state machines**: `master/fsm_*.c/h` drive bus/protocol logic. Each FSM is a struct holding a `void (*state)(ec_fsm_x_t *)` function pointer; `ec_fsm_x_init()` sets it up, `ec_fsm_x_exec()` calls the current state handler, and state handlers reassign `fsm->state` to transition. FSMs nest (e.g. `fsm_master` embeds `fsm_coe`, `fsm_sii`). Use `master/fsm_master.c` as the reference pattern when adding states.

**ioctl interface** (`master/ioctl.h`, `master/cdev.c`): bump `EC_IOCTL_VERSION_MAGIC` whenever the ioctl struct layout or command set changes — the library checks it against the kernel module.

**Realtime safety**: respect master phases and calling contexts (idle vs. operation, blocking vs. rt_safe) documented in [master/api_usage_notes.md](master/api_usage_notes.md). Config calls (`ecrt_slave_config_*`) are idle/blocking only; process-data calls (`ecrt_domain_queue/process`, `ecrt_master_send/receive`) must stay rt-safe (no sleeping/allocation).

## Build

Autotools + kernel Kbuild hybrid. Full instructions in [INSTALL.md](INSTALL.md).

```bash
./bootstrap                      # only if building from the repo (generates ./configure)
./configure --sysconfdir=/etc    # add feature flags as needed (see below)
make all modules                 # userspace + kernel modules
sudo make modules_install install && sudo depmod
```

Common `./configure` flags (defined in `configure.ac`): `--with-linux-dir=<DIR>`, `--enable-generic`, `--enable-8139too|e100|e1000|e1000e|r8169`, `--enable-eoe`, `--with-xenomai-dir=<DIR>`, `--with-rtai-dir=<DIR>`. Generated artifacts: `master/ec_master.ko`, `devices/ec_device_*.ko`, `lib/libethercat.so`, `tool/ethercat`.

Docs: `cd documentation && make` (PDF handbook); `make doc` (Doxygen, requires `git submodule update --init`).

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). Never report security issues in the public tracker — email `fp@igh.de`. The canonical repo is on GitLab (`gitlab.com/etherlab.org/ethercat`).
