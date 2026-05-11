# Realtek r8125 DKMS

![Static Badge](https://img.shields.io/badge/RELEASE-V9.014.01-blue)

This repository started as an investigation into several hangs/crashes (wrong fragment count with lots of small packets) and occasional data corruption but ended as a comprehensive rewrite of the official Realtek r8125 driver. While highly stable, the mainline r8169 driver does not fully support more advanced features of the 8125 family, such as RSS, PTP, multi-TX queues, MSIX interrupts etc.  

- Implement fixes for data corruption and hangs, some borrowed from the mainstream r8169 driver (6.8+).
- Differentiate from the mainline r8169 based driver through support for multi-TX (2 queues), receive side scaling (aka RSS, 4 queues), MSIX and PTP support while offering slightly higher performance at reduced system load.
- Fix PAGE_REUSE and add support for large pages (2M)
- Improve performance: achieve 2.5Gb speeds in upload and download for both single and parallel transfers using multiple receive and transmit queues. It requires an MTU larger than 1500 bytes to achieve the full 2.5Gb. With the default MTU of 1500, speeds of around 2.35Gb are be obtained.
- Reduce the use of system resources (System CPU) by about 50% compared to official driver, based on a 6 minute stress test (~20s versus ~38s).
- Reduce code size and memory footprint, depending on the compile time options up to 50%
- Simplify the code by removing support for pre 4.x Linux kernels
- Remove support for unknown r8125 chipsets: the driver will now exit when unknown variants of the Realtek 8125A, 8125B, 8125D and 8125BP are discovered.
- Integrate changes/fixes from the official Realtek releases, follow Realtek 8125 driver version numbering
- Leverage the excellent DKMS work from [awesometic](https://github.com/awesometic/realtek-r8125-dkms/)
- Enable HW checksumming for jumbo packets as it was found to be stable and offer equal performance at reduced system loads. HW TSO for MTU's larger than 1500 bytes has been left disabled as it negatively impacts performance.

**NOTE:**
This driver is provided as-is, support will be on voluntary basis ("best-effort") via GitHub "issues".  The rewrite effort was started because the existing r8125 failed stress testing (data corruptions) while the mainline driver lacked features to fully support the 8125 family.
Testing was done solely on a cluster of RK3588 8-core arm64 systems (Orange PI 5+) running dual Realtek 8125B NICS on 10G ethernet switches on busy and quiet networks and using MTU's of 1500 and 9000 bytes, Ubuntu 24.04 (Noble) and 6.1+ kernels. While unintentional, it is possible that support for other 8125 variants is broken.

## Kernel versions
In principle, all kernel versions between 4.0 and 6.12 should work. The last version that was effectively stress tested was 6.12.8.
Versions above 6.15 will definately NOT work, due to changes in the IOMMU. A new version is in developement and being tested on 6.18.x.

## Known Issues
- PTP is enabled but testing has not succeeded. Some extra logging has been added and a few kernel hangs eliminated. It seems 8125B chips can't timestamp TX packets that have the minor version set to a non-zero value. Compiling linuxptp 4.4 (instead of versions with Ubuntu Noble (4.0) or Debian 12 (3.1.1)) and setting ptp_minor_version to 0 provides some progress...

## Before use

This DKMS package is for Realtek RTL8125 (r8125 in module name) Ethernet, which is designed for the PCI interface.

## Installation

There are 3 ways to install this DKMS module. Choose one as your tastes.

Those are not interfering with each other. So you can do all 3 methods but absolutely you don't need to.

Installation using the Debian package is recommended for the sake of getting the newer driver.

### Debian package

#### Released package file

Download the latest Debian package from the Release tab on the Github repository.

Then enter the following command.

```bash
sudo dpkg -i realtek-r8125-dkms*.deb
```

> If multiple files selected by the wild card, you should type the specific version of the file.
>
> ```bash
> sudo dpkg -i realtek-r8125-dkms_9.014.01-1_amd64.deb
> ```

If dependency error occurs, try to fix that with `apt` command.

```bash
sudo apt install --fix-broken
```

### autorun.sh

Using the `autorun.sh` script that Realtek provides on their original driver package. This is **not installed as a DKMS**, only efforts to the current kernel.

Download or clone this repository and move to the extracted directory, then run the script.

```bash
sudo ./autorun.sh
```

Note that the `autorun` script will stop the `r8169` module if present.  Hence, do not connect remotely, at least not using this route, as you will be disconnected and the script will be terminated.   

### dkms-install.sh

This script is from aircrack-ng team. You can install the DKMS module by a simple command.

Download or clone this repository and move to the extracted directory, then run the script.

```bash
sudo ./dkms-install.sh
```

## Verify the module is loaded successfully

After installing the DKMS package, you may not be able to use the new `r8125` module on the fly. This because the existing `r8169` module will be loaded priority to `r8125` so that it prevents working of the `r8125` module.

Check if the `r8169` module loaded currently.

```bash
lsmod | grep -i r8169
```

If there is a result, maybe the `r8125` module wasn't loaded properly. You can check out modules currently in use via `lspci -k` or `dmesg` too.

To use the `r8125` module explicitly you can add the `r8169` module to not be loaded by adding it to a blacklist file.

Enter the following command to configure the blacklist.

```bash
sudo tee -a /etc/modprobe.d/blacklist-r8169.conf > /dev/null <<EOT
# To use r8125 driver explicitly
blacklist r8169
EOT
```

To apply the new blacklist to your kernel, update your initramfs via

```bash
sudo update-initramfs -u
```

Finally, reboot to take effect.

> - If you need to load both r8169 and r8125, maybe removing r8125 firmware could make it work. Please enter `sudo rm -f /lib/firmware/rtl_nic/rtl8125*` to remove all the r8125 firmwares on the system. But it is just a workaround, you should have to do this every time installing the new kernel version or new Linux firmware.
> - In the case of the Debian package, I will update the scripts to make it do this during the installation.

## Debian package build

You can build yourself this after installing some dependencies including `dkms`.

```bash
sudo apt install devscripts debmake debhelper build-essential dkms dh-dkms
```

```bash
dpkg-buildpackage -b -rfakeroot -us -uc
```

## LICENSE

GPL-2 on Realtek driver and the debian packaing.

## References

- [Realtek r8125 driver release page](https://www.realtek.com/Download/List?cate_id=584)
- [ParrotSec's realtek-rtl88xxau-dkms, where got hint from](https://github.com/ParrotSec/realtek-rtl88xxau-dkms)
