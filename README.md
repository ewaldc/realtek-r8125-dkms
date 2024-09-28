# Realtek r8125 DKMS

![GitHub release (latest SemVer)](https://img.shields.io/github/v/release/awesometic/realtek-r8125-dkms?sort=semver&style=for-the-badge)

This provides the Realtek r8125 driver in a DKMS way so that you can keep the latest driver even after a kernel upgrade.
The official Realtek 8125 driver has been rewritten with the following objections:
- Implement data corruption fixes, some borrowed from the upstream mainstream r8169 driver (6.8+).
- Improve performance: achieve 2.5Gb speeds in upload and download for both single and parallel transfers using multiple receive and transmit queues. It requires an MTU larger than 1500 bytes to achieve the full 2.5Gb. With the default MTU, speeds of around 2.3Gb can be obtained.
- Reduce the use of system resource: almost 2x less system CPU used based on a 6 minute stress test (~20s versus ~38s).
- Reduce code size (depending on the options up to 50%)
- Simplify the code by removing support for pre 4.x Linux kernels as well as unknown r8125 chipsets: the driver will now exit when unknown variants of the Realtek 8125A, 8125B, 8125D and 8125BP are discovered.
- Differentiate from the mainline r8169 based drive through RSS/PTP support, higher performance at reduced system load.

** NOTE: **
This driver is provided as-is, support will be on voluntary basis ("best-effort") via GitHub "issues".  The rewrite effort was started because the existing r8125 failed stress testing (data corruptions) while the mainline driver lacked features and performance to support a clustered file server.
Testing was done solely on a cluster of 8-core arm64 systems running dual Realtek 8125B NICS on 10G ethernet switches, Ubuntu 24.04 (Noble) and 6.1+ kernels,

## Before use

This DKMS package is for Realtek RTL8125 (r8125 in module name) Ethernet, which is designed for the PCI interface.

If you are searching for the Realtek 2.5 Gbits **USB Ethernet**, which may use RTL8156 (r8152 in module name), you are in the wrong place. Please refer to another DKMS project for that Realtek driver.

- [Realtek R8152 DKMS](https://github.com/awesometic/realtek-r8152-dkms)

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
> sudo dpkg -i realtek-r8125-dkms_9.013.02-2_amd64.deb
> ```

If dependency error occurs, try to fix that with `apt` command.

```bash
sudo apt install --fix-broken
```

#### Launchpad PPA (Recommended)

Add the Launchpad PPA.

```bash
sudo add-apt-repository ppa:awesometic/ppa
```

Then install the package using `apt` tool.

```bash
sudo apt install realtek-r8125-dkms
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
sudo apt install devscripts debmake debhelper build-essential dkms
```

```bash
dpkg-buildpackage -b -rfakeroot -us -uc
```

## LICENSE

GPL-2 on Realtek driver and the debian packaing.

## References

- [Realtek r8125 driver release page](https://www.realtek.com/Download/List?cate_id=584)
- [ParrotSec's realtek-rtl88xxau-dkms, where got hint from](https://github.com/ParrotSec/realtek-rtl88xxau-dkms)
