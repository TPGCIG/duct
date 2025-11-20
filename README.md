# duct: High-Performance, Zero-Copy PCI Driver

## Summary
*duct* (like the tape) is a network interface card for connecting your computer to *Ductnet*, which is a simplified, packet-switched network with a bus topology. It acts as a bridge between the OS and the network, handing the transmission and reception of packets using Direct Memory Access (DMA) to offload work from the CPU.

This project demonstrates a production-ready implementation of kernel-space concurrency, interrupt handling (MSI-X) and user-kernel boundary management via `ioctl` and `kqueue`.

## Architecture
The driver utilises a ring buffer to receive and transmit RX/TX packets from the network. The rings, RX/TX packet descriptors, and bounce buffers for descriptors are held in DMA maps. The CPU allocates DMA-safe memory regions and passes physical addresses to the device through PCI BARs. Ownership of buffer descriptors is toggled via an `owner` bit to ensure race-free synchronisation between the Host and Device without locking in the data path.

## Key Features
- **Zero-Copy DMA:** Implements `bus_dma` scatter-gather mapping to allow the hardware to write directly to RAM, bypassing CPU copy operations.
- **SMP Concurrency:** Utilises extensive mutex locking (`mtx_enter`/`mtx_leave`) and atomic memory barriers to prevent race conditions in a symmetric multiprocessing environment. The entire driver is completely thread-safe.
- **Interrupt Handling:** Implements MSI-X vector support for distinct Event and Error handling.
- **Asynchronous I/O:** Full support of non-blocking I/O via `kqueue` allowing userland applications to sleep until hardware events finish.
- **Hardware Filtering:** Since every device on the network can receive transmitted packets, there is a custom `ioctl` interface for programming hardware multicast filters.

## Repository Structure
The project was completely done within the OpenBSD kernel (via virtual machine) so the structure tries to represent that.
```
duct-pci-driver/
├── patches/
│   ├── GENERIC.diff         # Kernel config patch to enable the driver
│   └── files.pci.diff       # Build system patch to register source files
├── sys/
│   └── dev/
│       └── pci/
│           └── duct.c       # Main driver source code
└── userland/
    ├── ductclient.c         # User-space CLI tool for testing the driver over the network.
    └── ductctl.c            # User-space CLI tool for driver testing locally.
```

## Build Instructions
The driver is designed to be compiled directly into the kernel source tree.

### 1. Prepare the source tree
```
$ cp sys/dev/pci/duct.c /usr/src/sys/dev/pci/
```

### 2. Apply Patches
Apply the patches to register the driver in the kernel.
```
$ cd /usr/src
$ patch -p0 < patches/files.pci.diff
$ patch -p0 < patches/GENERIC.diff
$ patch -p0 < patches/conf.c.diff
```

### 3. Compile the Kernel
Rebuild the kernel.
```
$ cd /usr/src/sys/arch/amd64/conf
$ config GENERIC
$ cd ../compile/GENERIC.MP
$ make clean && make
$ make install
```

### 4. Build Userland Tools
Compile the userland tools so you can test the driver.
```
$ clang -O2 -o ductctl ductctl.c
$ clang -O2 -o ductclient ductclient.c
```

## Usage & Verification

Once you've rebooted and rebuilt the kernel, the device should attach as `duct0`.

```
$ dmesg | grep duct
nvme0 at pci0 dev 1 function 0 unknown vendor 0xfb5d product 0x0a0a rev 0x00: msix, NVMe 1.4
duct0 at pci0 dev 6 function 0 unknown vendor 0x3301 product 0x2000 rev 0x00duct0: duct device v2.1 hwaddr=0x34565cab
duct0: MSI-X vec0 (events) at msix
duct0: MSI-X vec1 (fatal) at msix
duct0: attach successful!
```

Once rebooted, create the UNIX device node files. The Major Number for the device was designed as 102 however you can edit this in `conf.c`.
```
$ doas mknod -m 666 /dev/duct0 c 102 0
```

Then, you can use the provided tools to test it out.

```
$ ductctl
/dev/duct0: duct v2.1, hwaddr: 34565cab
$ ductclient -q
“THERE ARE NO MODAL DIALOGS IN SPARTA.”
$ ductclient -e
hello duct and world!
hello duct and world!
```




