# The IgH EtherCAT Master

[[_TOC_]]

## General Information

This is the README.md file of the IgH EtherCAT Master.

This is an open-source EtherCAT master implementation for Linux 2.6 or newer.

See the [features file](FEATURES.md) for a list of features. For more
information, see [etherlab.org/ethercat](https://etherlab.org/ethercat).

or contact

> Dipl.-Ing. (FH) [Florian Pose](mailto:fp@igh.de)\
> Ingenieurgemeinschaft IgH\
> Nordsternstraße 66\
> D-45329 Essen\
> [igh.de](http://igh.de)

## Documentation

### Handbook

The PDF documentation is generated via LaTeX and can be build with the
following steps:

```bash
cd documentation
make
```

The PDF is automatically held up-to-date and can be [downloaded from
GitLab](https://gitlab.com/etherlab.org/ethercat/-/jobs/artifacts/stable-1.6/raw/pdf/ethercat_doc.pdf?job=pdf).

### Doxygen

To generate the Doxygen documentation, the following commands can be used.
Therefore, the configure script must have run (see the [install
file](INSTALL.md)).

```bash
git submodule update --init
make doc
```

An up-to-date Doxygen output can be found on
[docs.etherlab.org](https://docs.etherlab.org/ethercat/1.6/doxygen/index.html).

## Requirements

### Software requirements

Configured sources for the Linux 2.6 (or newer) kernel are required to build
the EtherCAT master.

### Hardware requirements

A table of supported hardware can be found at
[docs.etherlab.org](https://docs.etherlab.org/ethercat/1.6/doxygen/devicedrivers.html).

## Building and installing

See the [install file](INSTALL.md).

## Dry-run and Field Simulation

A limited set of the userspace API is available in `libfakeethercat`,
a library which can be used to run an userspace application
without an EtherCAT master or with emulated EtherCAT slaves.
Please find some details in the [Fakelib README](fake_lib/README.md).

## Robot Joint SDK (`joint-sdk`)

`joint-sdk` is a CiA402 (CANopen over EtherCAT / CoE) SDK for quickly
integrating and testing EtherCAT-based robot joints. It encapsulates
the CiA402 state machine, PDO mapping, and SDO diagnostics, allowing
you to drive a joint with just a few lines of C code.

The SDK is built on top of IgH EtherCAT Master's userspace library
(`libethercat.so`) and targets real-time control loops.

### Quick Start

#### Prerequisites

- IgH EtherCAT Master installed (default prefix `/opt/etherlab`);
  set `ETHERLAB_DIR` if installed elsewhere.
- GCC, GNU Make.

#### Build

```bash
cd joint-sdk
make
```

This produces:
- `build/libjointsdk.a` — static library
- `build/libjointsdk.so` — shared library
- `build/csp_single_sdk` — basic CSP example
- `build/diag_csp_single` — CSP with diagnostic output
- `build/diag_csv_single` — CSV mode diagnostic example
- `build/diag_cst_single` — CST mode diagnostic example
- `build/sdo_diag` — SDO parameter read/write example
- `build/fault_diag` — fault detection & recovery example
- `build/phys_csp_single` — CSP with physical unit（角度/力矩）demo

#### Run an Example

```bash
sudo ./build/csp_single_sdk
```

The example activates the master, configures the first detected joint,
and runs a sinusoidal CSP（Cyclic Synchronous Position）trajectory.

### Integrating into Your Application

#### Build Flags

```makefile
CFLAGS  += -I$(JOINT_SDK_DIR)/include -I$(ETHERLAB_DIR)/include
LDFLAGS += -L$(JOINT_SDK_DIR)/build -L$(ETHERLAB_DIR)/lib \
           -Wl,-rpath,$(ETHERLAB_DIR)/lib
LDLIBS  += -ljointsdk -lethercat -lm
```

#### Minimal Code (CSP mode)

```c
#include <joint_sdk/joint_sdk.h>

jsdk_context_config_t cfg = { .master_index = 0, .period_ns = 1000000 };
jsdk_context_t *ctx;
jsdk_joint_t *joint;

jsdk_context_create(&cfg, &ctx);
jsdk_context_add_joint(ctx, 0, NULL, &joint);  // alias=0, auto-detect
jsdk_context_activate(ctx);

// Real-time loop
while (running) {
    jsdk_context_cycle_begin(ctx);
    jsdk_joint_set_mode(joint, JSDK_MODE_CSP);
    jsdk_joint_set_target_position(joint, target_pos);
    jsdk_context_cycle_end(ctx);
}
```

### API Overview

| Layer | Header | Purpose |
|-------|--------|---------|
| Context & Joint | `joint_sdk/joint_sdk.h` | Master lifecycle, joint config, cycle I/O |
| CiA402 Types | `joint_sdk/cia402.h` | Mode enums, status/control structures |

The SDK supports three CiA402 operating modes:
- **CSP** (Cyclic Synchronous Position, `JSDK_MODE_CSP = 8`)
- **CSV** (Cyclic Synchronous Velocity, `JSDK_MODE_CSV = 9`)
- **CST** (Cyclic Synchronous Torque, `JSDK_MODE_CST = 10`)

For architecture details and device profile configuration, see
[`joint-sdk/ARCHITECTURE.zh-CN.md`](joint-sdk/ARCHITECTURE.zh-CN.md).

## Realtime and Tuning

Realtime patches for the Linux kernel are supported, but not required. The
realtime processing has to be done by the calling module (see API
documentation). The EtherCAT master code itself is passive (except for the
idle mode and EoE).

To avoid frame timeouts, deactivating DMA access for hard drives is
recommended (`hdparm -d0 <DEV>`).

## License

Copyright (C) 2006-2023  Florian Pose, Ingenieurgemeinschaft IgH

This file is part of the IgH EtherCAT Master.

The IgH EtherCAT Master is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License version 2, as
published by the Free Software Foundation.

The IgH EtherCAT Master is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
details.

You should have received a copy of the GNU General Public License along with
the IgH EtherCAT Master; if not, write to the Free Software Foundation, Inc.,
51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

## I have a question / I want to contribute

Please see the [contributing document](CONTRIBUTING.md).

## Coding Style

Developers shall use the coding style rules in the [coding style
file](CodingStyle.md).

There is a [cpplint configuration](CPPLINT.cfg) included as well that is
automatically checked via [pre-commit hooks](.pre-commit-config.yaml). So
please install the pre-commit hooks via

```bash
pre-commit install
```
