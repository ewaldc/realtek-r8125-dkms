// SPDX-License-Identifier: GPL-2.0-only
/*
################################################################################
#
# r8125 is the Linux device driver released for Realtek 2.5 Gigabit Ethernet
# controllers with PCI-Express interface.
#
# Copyright(c) 2024 Realtek Semiconductor Corp. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation; either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, see <http://www.gnu.org/licenses/>.
#
# Author:
# Realtek NIC software team <nicfae@realtek.com>
# No. 2, Innovation Road II, Hsinchu Science Park, Hsinchu 300, Taiwan
#
################################################################################
*/

/************************************************************************************
 *  This product is covered by one or more of the following patents:
 *  US6,570,884, US6,115,776, and US6,327,625.
 ***********************************************************************************/

#include <linux/module.h>
#include <linux/version.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/mii.h>
#include <linux/if_vlan.h>
#include <linux/crc32.h>
#include <linux/interrupt.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <net/ip6_checksum.h>
#include <linux/tcp.h>
#include <linux/init.h>
#include <linux/rtnetlink.h>
#include <linux/completion.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,4,0)
#include <linux/pci-aspm.h>
#endif
#include <linux/prefetch.h>
#include <linux/dma-mapping.h>
#include <linux/dma-direct.h>
#include <linux/pm_runtime.h>
#include <linux/moduleparam.h>
#include <linux/mdio.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,4,10)
#include <net/gso.h>
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(6,4,10) */

#include <asm/io.h>
#include <asm/irq.h>

#include "r8125.h"
#include "rtl_eeprom.h"
#include "rtltool.h"
#include "r8125_firmware.h"
#include "r8125_ptp.h"
#include "r8125_rss.h"

#ifdef ENABLE_R8125_PROCFS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#endif

#define FIRMWARE_8125A_3        "rtl_nic/rtl8125a-3.fw"
#define FIRMWARE_8125B_2        "rtl_nic/rtl8125b-2.fw"
#define FIRMWARE_8125BP_1       "rtl_nic/rtl8125bp-1.fw"
#define FIRMWARE_8125BP_2       "rtl_nic/rtl8125bp-2.fw"
#define FIRMWARE_8125D_1        "rtl_nic/rtl8125d-1.fw"
#define FIRMWARE_8125D_2        "rtl_nic/rtl8125d-2.fw"

static void rtl8125_hw_phy_config_8125a_1(struct rtl8125_private *tp);
static void rtl8125_hw_phy_config_8125a_2(struct rtl8125_private *tp);
static void rtl8125_hw_phy_config_8125b_1(struct rtl8125_private *tp);
static void rtl8125_hw_phy_config_8125b_2(struct rtl8125_private *tp);
static void rtl8125_hw_phy_config_8125bp_1(struct rtl8125_private *tp);
static void rtl8125_hw_phy_config_8125bp_2(struct rtl8125_private *tp);
static void rtl8125_hw_phy_config_8125d_1(struct rtl8125_private *tp);
static void rtl8125_hw_phy_config_8125d_2(struct rtl8125_private *tp);

#ifdef DISABLE_MULTI_MSIX_VECTOR
#define MAX_MSIX_VECTOR 1
#else
#define MAX_MSIX_VECTOR 32
#endif

#define _R(NAME,FWNAME,MAC,RCR,MASK,HW_PHY_CFG,SW_RAMCODE_VER,CSI_FUN0_BYTE, \
        LINK_CHG_SHIFT, ISR_VER,INT_MITI_VER,MIN_IRQ_NVECS,MAX_IRQ_NVECS) \
        { .name = NAME, .mcfg = MAC, .RxConfig = RCR, .RxConfigMask = MASK, \
        .rtl8125_hw_phy_config = HW_PHY_CFG, .sw_ram_code_ver = SW_RAMCODE_VER, \
        .org_pci_offset_180 = (u8)CSI_FUN0_BYTE, .linkChgShift = (u8)LINK_CHG_SHIFT, \
        .HwSuppIsrVer = (u8)ISR_VER, .HwSuppIntMitiVer = (u8)INT_MITI_VER, \
        .min_irq_nvecs = (u8)MIN_IRQ_NVECS, .max_irq_nvecs = (u8)MAX_IRQ_NVECS }

static const struct {
        const char *name;
        const char *fw_name;
        u8 mcfg;
        u32 RxConfig;
        u32 RxConfigMask;   /* Clears the bits supported by this chip */
        void (*rtl8125_hw_phy_config)(struct rtl8125_private *);
        u16 sw_ram_code_ver;
        u8 org_pci_offset_180, linkChgShift;
        u8 HwSuppIsrVer, HwSuppIntMitiVer;
        u8 min_irq_nvecs, max_irq_nvecs;
} rtl_chip_info[] = {
        _R("RTL8125A", "",
        CFG_METHOD_2,
        Rx_Fetch_Number_8 | EnableInnerVlan | EnableOuterVlan | (RX_DMA_BURST_256 << RxCfgDMAShift),
        0xff7e5880, rtl8125_hw_phy_config_8125a_1, NIC_RAMCODE_VERSION_CFG_METHOD_2, 0x264, 5, 1, 3, 1, 1),

        _R("RTL8125A", FIRMWARE_8125A_3,
        CFG_METHOD_3,
        Rx_Fetch_Number_8 | EnableInnerVlan | EnableOuterVlan | (RX_DMA_BURST_256 << RxCfgDMAShift),
        0xff7e5880, rtl8125_hw_phy_config_8125a_2, NIC_RAMCODE_VERSION_CFG_METHOD_3, 0x264, 5, 1, 3, 1, 1),

        _R("RTL8125B", "",
        CFG_METHOD_4,
        Rx_Fetch_Number_8 | RxCfg_pause_slot_en | EnableInnerVlan | EnableOuterVlan | (RX_DMA_BURST_unlimited << RxCfgDMAShift),
        0xff7e5880, rtl8125_hw_phy_config_8125b_1, NIC_RAMCODE_VERSION_CFG_METHOD_4, 0x214, 21, 2, 4, 22, MAX_MSIX_VECTOR),

        _R("RTL8125B", FIRMWARE_8125B_2,
        CFG_METHOD_5,
        Rx_Fetch_Number_8 | RxCfg_pause_slot_en | EnableInnerVlan | EnableOuterVlan | (RX_DMA_BURST_unlimited << RxCfgDMAShift),
        0xff7e5880, rtl8125_hw_phy_config_8125b_2, NIC_RAMCODE_VERSION_CFG_METHOD_5, 0x214, 21, 2, 4, 22, MAX_MSIX_VECTOR),

        _R("RTL8162", "", CFG_METHOD_UNKNOWN, (RX_DMA_BURST_256 << RxCfgDMAShift), 0xff7e5880, NULL, 0, 0, 0, 0, 0, 1, 1), // Not used

        _R("RTL8162", "", CFG_METHOD_UNKNOWN, (RX_DMA_BURST_512 << RxCfgDMAShift), 0xff7e5880, NULL, 0, 0, 0, 0, 0, 1, 1), // Not used

        _R("RTL8125BP", FIRMWARE_8125BP_1,
        CFG_METHOD_8,
        Rx_Fetch_Number_8 | Rx_Close_Multiple | RxCfg_pause_slot_en | EnableInnerVlan | EnableOuterVlan | (RX_DMA_BURST_256 << RxCfgDMAShift),
        0xff7e5880, rtl8125_hw_phy_config_8125bp_1, NIC_RAMCODE_VERSION_CFG_METHOD_8, 0x210, 29, 4, 6, 31, MAX_MSIX_VECTOR),

        _R("RTL8125BP", FIRMWARE_8125BP_2,
        CFG_METHOD_9,
        Rx_Fetch_Number_8 | Rx_Close_Multiple | RxCfg_pause_slot_en | EnableInnerVlan | EnableOuterVlan | (RX_DMA_BURST_256 << RxCfgDMAShift),
        0xff7e5880, rtl8125_hw_phy_config_8125bp_2, NIC_RAMCODE_VERSION_CFG_METHOD_9, 0x210, 29, 4, 6, 31, MAX_MSIX_VECTOR),

        _R("RTL8125D", FIRMWARE_8125D_1,
        CFG_METHOD_10,
        Rx_Fetch_Number_8 | Rx_Close_Multiple | RxCfg_pause_slot_en | EnableInnerVlan | EnableOuterVlan | (RX_DMA_BURST_256 << RxCfgDMAShift),
        0xff7e5880, rtl8125_hw_phy_config_8125d_1, NIC_RAMCODE_VERSION_CFG_METHOD_10, 0x210, 18, 5, 7, 20, MAX_MSIX_VECTOR),

        _R("RTL8125D", FIRMWARE_8125D_2,
        CFG_METHOD_11,
        Rx_Fetch_Number_8 | Rx_Close_Multiple | RxCfg_pause_slot_en | EnableInnerVlan | EnableOuterVlan | (RX_DMA_BURST_256 << RxCfgDMAShift),
        0xff7e5880, rtl8125_hw_phy_config_8125d_2, NIC_RAMCODE_VERSION_CFG_METHOD_11, 0x210, 18, 5, 7, 20, MAX_MSIX_VECTOR),

        _R("Unknown", "", CFG_METHOD_UNKNOWN, (RX_DMA_BURST_512 << RxCfgDMAShift), 0xff7e5880, NULL, 0, 0, 0, 0, 0, 1, 1)
};
#undef _R


#ifndef PCI_VENDOR_ID_DLINK
#define PCI_VENDOR_ID_DLINK 0x1186
#endif

static struct pci_device_id rtl8125_pci_tbl[] = {
        { PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8125), },
        { PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8162), },
        { PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x3000), },
        {0,},
};

MODULE_DEVICE_TABLE(pci, rtl8125_pci_tbl);

static int use_dac = 1;
static int timer_count = 0x2600;
static int timer_count_v2 = (0x2600 / 0x100);

static struct {
        u8 msg_enable;
} debug = { -1 };

static unsigned int speed_mode = SPEED_2500;
static unsigned int duplex_mode = DUPLEX_FULL;
static unsigned int autoneg_mode = AUTONEG_ENABLE;
#ifdef CONFIG_ASPM
static int aspm = 1;
#else
static int aspm = 0;
#endif
#ifdef ENABLE_S5WOL
static int s5wol = 1;
#else
static int s5wol = 0;
#endif
#ifdef ENABLE_S5_KEEP_CURR_MAC
static int s5_keep_curr_mac = 1;
#else
static int s5_keep_curr_mac = 0;
#endif
#ifdef ENABLE_EEE
static int eee_enable = 1;
#else
static int eee_enable = 0;
#endif
#ifdef CONFIG_SOC_LAN
static ulong hwoptimize = HW_PATCH_SOC_LAN;
#else
static ulong hwoptimize = 0;
#endif
#ifdef ENABLE_S0_MAGIC_PACKET
static int s0_magic_packet = 1;
#else
static int s0_magic_packet = 0;
#endif
#ifdef ENABLE_PTP_SUPPORT
#ifdef ENABLE_PTP_MASTER_MODE
static int enable_ptp_master_mode = 1;
#else
static int enable_ptp_master_mode = 0;
#endif
#endif

#ifdef ENABLE_DOUBLE_VLAN
static int enable_double_vlan = 1;
#else
static int enable_double_vlan = 0;
#endif

MODULE_AUTHOR("Realtek and the Linux r8125 crew <netdev@vger.kernel.org>");
MODULE_DESCRIPTION("Realtek r8125 Ethernet controller driver");

module_param(speed_mode, uint, 0);
MODULE_PARM_DESC(speed_mode, "force phy operation. Deprecated by ethtool (8).");

module_param(duplex_mode, uint, 0);
MODULE_PARM_DESC(duplex_mode, "force phy operation. Deprecated by ethtool (8).");

module_param(autoneg_mode, uint, 0);
MODULE_PARM_DESC(autoneg_mode, "force phy operation. Deprecated by ethtool (8).");

module_param(aspm, int, 0);
MODULE_PARM_DESC(aspm, "Enable ASPM.");

module_param(s5wol, int, 0);
MODULE_PARM_DESC(s5wol, "Enable Shutdown Wake On Lan.");

module_param(s5_keep_curr_mac, int, 0);
MODULE_PARM_DESC(s5_keep_curr_mac, "Enable Shutdown Keep Current MAC Address.");

module_param(use_dac, int, 0);
MODULE_PARM_DESC(use_dac, "Enable PCI DAC. Unsafe on 32 bit PCI slot.");

module_param(timer_count, int, 0);
MODULE_PARM_DESC(timer_count, "Timer Interrupt Interval.");

module_param(eee_enable, int, 0);
MODULE_PARM_DESC(eee_enable, "Enable Energy Efficient Ethernet.");

module_param(hwoptimize, ulong, 0);
MODULE_PARM_DESC(hwoptimize, "Enable HW optimization function.");

module_param(s0_magic_packet, int, 0);
MODULE_PARM_DESC(s0_magic_packet, "Enable S0 Magic Packet.");

#ifdef ENABLE_PTP_SUPPORT
module_param(enable_ptp_master_mode, int, 0);
MODULE_PARM_DESC(enable_ptp_master_mode, "Enable PTP Master Mode.");
#endif

module_param(enable_double_vlan, int, 0);
MODULE_PARM_DESC(enable_double_vlan, "Enable Double VLAN.");

module_param_named(debug, debug.msg_enable, byte, 0);
MODULE_PARM_DESC(debug, "Debug verbosity level (0=none, ..., 16=all)");

MODULE_LICENSE("GPL");
#ifdef ENABLE_USE_FIRMWARE_FILE
MODULE_FIRMWARE(FIRMWARE_8125A_3);
MODULE_FIRMWARE(FIRMWARE_8125B_2);
MODULE_FIRMWARE(FIRMWARE_8125BP_1);
MODULE_FIRMWARE(FIRMWARE_8125BP_2);
MODULE_FIRMWARE(FIRMWARE_8125D_1);
MODULE_FIRMWARE(FIRMWARE_8125D_2);
#endif

MODULE_VERSION(RTL8125_VERSION);

static netdev_tx_t rtl8125_start_xmit(struct sk_buff * const skb, struct net_device *const dev);
static irqreturn_t rtl8125_interrupt(const int irq, void * const dev_instance);
static irqreturn_t rtl8125_interrupt_msix(const int irq, void * const dev_instance);
static void rtl8125_set_rx_mode(struct net_device *dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
static void rtl8125_tx_timeout(struct net_device *dev, unsigned int txqueue);
#else
static void rtl8125_tx_timeout(struct net_device *dev);
#endif
static int rtl8125_rx_interrupt(struct rtl8125_private * const, const u8, int);
static void rtl8125_tx_interrupt(struct rtl8125_private * const, const u8, const int);

static int rtl8125_change_mtu(struct net_device *dev, int new_mtu);
static void rtl8125_down(struct rtl8125_private *tp, struct net_device *dev);

static __always_inline void rtl_pci_commit(struct rtl8125_private *tp)
{
        /* Read an arbitrary register to commit a preceding PCI write */
        RTL_R8(tp, ChipCmd);
}

static __always_inline struct RxDesc*
rtl8125_get_rxdesc(const struct rtl8125_private *tp, const struct RxDesc *RxDescBase, const u16 entry)
{
        return (struct RxDesc*)((u8*)RxDescBase + (entry * (tp->RxDescType & 0x3F)));
}

static __always_inline void
rtl8125_disable_hw_interrupt_v2(const struct rtl8125_private *tp, const u8 message_id)
{
        RTL_W32(tp, IMR_V2_CLEAR_REG_8125, BIT(message_id));
}

static __always_inline void
rtl8125_enable_hw_interrupt_v2(const struct rtl8125_private *tp, const u8 message_id)
{
        RTL_W32(tp, IMR_V2_SET_REG_8125, BIT(message_id));
}

static void rtl8125_hw_config(struct rtl8125_private *tp, struct net_device *dev);
static void rtl8125_hw_start(struct rtl8125_private *tp);
static void rtl8125_rx_clear_rings(struct rtl8125_private * const);
static void rtl8125_tx_clear_rings(struct rtl8125_private * const);
static int rtl8125_close(struct net_device *dev);
static int rtl8125_open(struct net_device *dev);
static int rtl8125_init_rings(struct rtl8125_private * const);

static int rtl8125_set_mac_address(struct net_device *dev, void *p);
static void rtl8125_rar_set(struct rtl8125_private *tp, const u8 *);
static void rtl8125_desc_addr_fill(const struct rtl8125_private * const);
static void rtl8125_tx_desc_init(struct rtl8125_private * const);
static void rtl8125_rx_desc_init(struct rtl8125_private * const);

static inline void rtl8125_mdio_set_page(struct rtl8125_private * const, const u16);
static inline u32 rtl8125_mdio_read(const struct rtl8125_private * const, const u16);

static void rtl8125_clear_and_set_eth_phy_ocp_bit(const struct rtl8125_private * const, const u16, const u16, const u16);
static void rtl8125_clear_eth_phy_ocp_bit(const struct rtl8125_private * const, const u16, const u16);
static void rtl8125_set_eth_phy_ocp_bit(const struct rtl8125_private * const tp, const u16 addr, const u16 mask);
static u16 rtl8125_get_hw_phy_mcu_code_ver(const struct rtl8125_private * const tp);
static void rtl8125_phy_power_up(struct rtl8125_private *const tp);
static void rtl8125_phy_power_down(struct rtl8125_private *const tp);
static int rtl8125_set_speed(struct rtl8125_private *tp, u8 autoneg, u32 speed, u8 duplex, u64 adv);
static bool rtl8125_set_phy_mcu_patch_request(struct rtl8125_private *tp);
static bool rtl8125_clear_phy_mcu_patch_request(struct rtl8125_private *tp);

static int rtl8125_poll(struct napi_struct *napi, int budget);

static void rtl8125_reset_task(struct work_struct *work);
#ifdef ENABLE_ESD
static void rtl8125_schedule_esd_work(struct rtl8125_private *tp);
#endif
static void rtl8125_init_all_schedule_work(struct rtl8125_private *tp);
static void rtl8125_cancel_all_schedule_work(struct rtl8125_private *tp);

#if ((LINUX_VERSION_CODE < KERNEL_VERSION(4,7,0) && \
     LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,00)))
void ethtool_convert_legacy_u32_to_link_mode(unsigned long *dst, u32 legacy_u32)
{
        bitmap_zero(dst, __ETHTOOL_LINK_MODE_MASK_NBITS);
        dst[0] = legacy_u32;
}

bool ethtool_convert_link_mode_to_legacy_u32(u32 *legacy_u32,
                const unsigned long *src)
{
        bool retval = true;

        /* TODO: following test will soon always be true */
        if (__ETHTOOL_LINK_MODE_MASK_NBITS > 32) {
                __ETHTOOL_DECLARE_LINK_MODE_MASK(ext);

                bitmap_zero(ext, __ETHTOOL_LINK_MODE_MASK_NBITS);
                bitmap_fill(ext, 32);
                bitmap_complement(ext, ext, __ETHTOOL_LINK_MODE_MASK_NBITS);
                if (bitmap_intersects(ext, src,
                                      __ETHTOOL_LINK_MODE_MASK_NBITS)) {
                        /* src mask goes beyond bit 31 */
                        retval = false;
                }
        }
        *legacy_u32 = src[0];
        return retval;
}
#endif

static int rtl8125_dump_tally_counter(struct rtl8125_private *tp, dma_addr_t paddr)
{
        u32 cmd, WaitCnt = 0;

        RTL_W32(tp, CounterAddrHigh, (u64)paddr >> 32);
        cmd = (u64)paddr & DMA_BIT_MASK(32);
        RTL_W32(tp, CounterAddrLow, cmd);
        RTL_W32(tp, CounterAddrLow, cmd | CounterDump);

        while (RTL_R32(tp, CounterAddrLow) & CounterDump) {
                udelay(10);
                if (++WaitCnt > 20)
                        return -1;
        }
        return 0;
}

static u32 rtl8125_convert_link_speed(u16 status)
{
        if (likely(status & LinkStatus)) {
                if (status & _2500bpsF)
                        return SPEED_2500;
                else if (status & _1000bpsF)
                        return SPEED_1000;
                else if (status & _100bps)
                        return SPEED_100;
                else if (status & _10bps)
                        return SPEED_10;
        }
        return SPEED_UNKNOWN;
}

#ifdef ENABLE_R8125_PROCFS
static u32
rtl8125_hw_clo_ptr(struct rtl8125_private *tp, const u8 ring_index)
{
        return is_8125AB(tp) ?  RTL_R16(tp, hw_clo_ptr_reg_v3(ring_index)) :
                                RTL_R32(tp, hw_clo_ptr_reg_v6(ring_index));
}

static u32
rtl8125_sw_tail_ptr(struct rtl8125_private *tp, const u8 ring_index)
{
        return is_8125AB(tp) ?  RTL_R16(tp, sw_tail_ptr_reg_v3(ring_index)) :
                                RTL_R32(tp, sw_tail_ptr_reg_v6(ring_index));
}

static u32 rtl8125_read_thermal_sensor(struct rtl8125_private *tp)
{
        if (is_8125A(tp) || is_8125D(tp))
                return rtl8125_mdio_direct_read_phy_ocp(tp, 0xBD84) & 0x3ff;
        else
                return 0xffff;
}


static bool
rtl8125_sysfs_testmode_on(struct rtl8125_private *tp)
{
#ifdef ENABLE_R8125_SYSFS
        return rtl8125_flag_to_bool(tp, TestMode);
#else
        return 1;
#endif
}

static void rtl8125_mdi_swap(struct rtl8125_private *tp)
{
        u16 reg, val, mdi_reverse;
        u16 tps_p0, tps_p1, tps_p2, tps_p3, tps_p3_p0;

        //if (rtl8125_flag_to_bool(tp, IsMcfg236))
        if (is_8125A(tp))
                reg = 0x8284;
        else if (is_8125B(tp)) //(tp->mcfg <= 7)
                reg = 0x81aa;
        else
                return;

        tps_p3_p0 = rtl8125_mac_ocp_read(tp, 0xD440) & 0xF000;
        tps_p3 = !!(tps_p3_p0 & BIT_15);
        tps_p2 = !!(tps_p3_p0 & BIT_14);
        tps_p1 = !!(tps_p3_p0 & BIT_13);
        tps_p0 = !!(tps_p3_p0 & BIT_12);
        mdi_reverse = rtl8125_mac_ocp_read(tp, 0xD442);

        if ((mdi_reverse & BIT_5) && tps_p3_p0 == 0xA000)
                return;

        if (!(mdi_reverse & BIT_5))
                val = tps_p0 << 8 | tps_p1 << 9 | tps_p2 << 10 | tps_p3 << 11;
        else
                val = tps_p3 << 8 | tps_p2 << 9 | tps_p1 << 10 | tps_p0 << 11;

        for (int i = 8; i < 12; i++) {
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, reg);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, BIT(i), val & BIT(i));
        }
}

static int _rtl8125_vcd_test(struct rtl8125_private *tp)
{
        u16 val;
        u32 wait_cnt;
        int ret = -1;

        rtl8125_mdi_swap(tp);

        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA422, BIT(0));
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA422, 0x00F0);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA422, BIT(0));

        wait_cnt = 0;
        do {
                mdelay(1);
                val = rtl8125_mdio_direct_read_phy_ocp(tp, 0xA422);
                wait_cnt++;
        } while (!(val & BIT_15) && (wait_cnt < 5000));

        if (wait_cnt == 5000)
                goto exit;

        ret = 0;

exit:
        return ret;
}

static int rtl8125_vcd_test(struct rtl8125_private *tp, bool poe_mode)
{
        int ret;

        if (is_8125B(tp)) {
                /* update rtct threshold for poe mode */
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8FE1);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, poe_mode ? 0x0A44 : 0x0000);

                /* enable rtct poe mode */
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8FE3);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, poe_mode ? 0x0100 : 0x0000);

                ret = _rtl8125_vcd_test(tp);

                /* disable rtct poe mode */
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8FE3);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);

                /* restore rtct threshold */
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8FE1);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        } else
                ret = _rtl8125_vcd_test(tp);

        return ret;
}

static void rtl8125_get_cp_len(struct rtl8125_private *tp, int cp_len[RTL8125_CP_NUM])
{
        int i;
        u16 status;
        int cplen = 0;

        status = RTL_R16(tp, PHYstatus);
        if (status & LinkStatus) {
                if (status & _10bps) {
                        cplen = -1;
                } else if (status & (_100bps | _1000bpsF)) {
                        rtl8125_mdio_set_page(tp, 0x0a88);
                        cplen = rtl8125_mdio_read(tp, 0x10);
                } else if (status & _2500bpsF) {
                        if (is_8125A(tp)) {
                                rtl8125_mdio_set_page(tp, 0x0ac5);
                                cplen = rtl8125_mdio_read(tp, 0x14) >> 4;
                        } else {
                                rtl8125_mdio_set_page(tp, 0x0acb);
                                cplen = rtl8125_mdio_read(tp, 0x15) >> 2;
                        }
                }
        }

        cplen &= 0xff;
        for (i = 0; i < RTL8125_CP_NUM; i++)
                cp_len[i] = cplen;

        rtl8125_mdio_set_page(tp, 0x0000);
        for (i=0; i<RTL8125_CP_NUM; i++)
                if (cp_len[i] > RTL8125_MAX_SUPPORT_CP_LEN)
                        cp_len[i] = RTL8125_MAX_SUPPORT_CP_LEN;
        return;
}

static int __rtl8125_get_cp_status(u16 val)
{
        switch (val) {
        case 0x0060:
                return rtl8125_cp_normal;
        case 0x0048:
                return rtl8125_cp_open;
        case 0x0050:
                return rtl8125_cp_short;
        case 0x0042:
        case 0x0044:
                return rtl8125_cp_mismatch;
        default:
                return rtl8125_cp_normal;
        }
}

static int _rtl8125_get_cp_status(struct rtl8125_private *tp, u8 pair_num)
{
        u16 val;
        int cp_status = rtl8125_cp_unknown;

        if (pair_num > 3)
                goto exit;

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8027 + 4 * pair_num);
        val = rtl8125_mdio_direct_read_phy_ocp(tp, 0xA438);

        cp_status = __rtl8125_get_cp_status(val);

exit:
        return cp_status;
}

static const char * rtl8125_get_cp_status_string(int cp_status)
{
        switch(cp_status) {
        case rtl8125_cp_normal:
                return "normal  ";
        case rtl8125_cp_short:
                return "short   ";
        case rtl8125_cp_open:
                return "open    ";
        case rtl8125_cp_mismatch:
                return "mismatch";
        default:
                return "unknown ";
        }
}

static u16 rtl8125_get_cp_pp(struct rtl8125_private *tp, u8 pair_num)
{
        u16 pp = 0;

        if (pair_num > 3)
                goto exit;

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8029 + 4 * pair_num);
        pp = rtl8125_mdio_direct_read_phy_ocp(tp, 0xA438);

        pp &= 0x3fff;
        pp /= 80;

exit:
        return pp;
}

static void rtl8125_get_cp_status(struct rtl8125_private *tp,
        int cp_status[RTL8125_CP_NUM], bool poe_mode)
{
        u16 status;
        int i;

        status = RTL_R16(tp, PHYstatus);
        if (status & LinkStatus && !(status & (_10bps | _100bps))) {
                for (i=0; i<RTL8125_CP_NUM; i++)
                        cp_status[i] = rtl8125_cp_normal;
        } else {
                /* cannot do vcd when link is on */
                rtl8125_vcd_test(tp, poe_mode);

                for (i=0; i<RTL8125_CP_NUM; i++)
                        cp_status[i] = _rtl8125_get_cp_status(tp, i);
        }

        if (poe_mode) {
                for (i=0; i<RTL8125_CP_NUM; i++) {
                        if (cp_status[i] == rtl8125_cp_mismatch)
                                cp_status[i] = rtl8125_cp_normal;
                }
        }
}

/****************************************************************************
*   -----------------------------PROCFS STUFF-------------------------
*****************************************************************************
*/

static struct proc_dir_entry *rtl8125_proc;
static int proc_init_num = 0;

static int proc_get_driver_variable(struct seq_file *m, void *v)
{
        struct net_device *dev = m->private;
        struct rtl8125_private *tp = netdev_priv(dev);
        struct pci_dev *pdev = tp->pci_dev;
        int chipset = chipset(tp);
        u8 org_pci_offset_80, org_pci_offset_81;
        pci_read_config_byte(pdev, 0x80, &org_pci_offset_80);
        pci_read_config_byte(pdev, 0x81, &org_pci_offset_81);

        seq_puts(m, "\nDump Driver Variable\n");

        rtnl_lock();

        seq_puts(m, "Variable\tValue\n----------\t-----\n");
        seq_printf(m, "MODULENAME\t%s\n", MODULENAME);
        seq_printf(m, "driver version\t%s\n", RTL8125_VERSION);
        seq_printf(m, "mcfg\t%d\n", tp->mcfg);
        seq_printf(m, "chipset\t%d\n", chipset);
        seq_printf(m, "chipset_name\t%s\n", rtl_chip_info[chipset].name);
        seq_printf(m, "mtu\t%d\n", dev->mtu);
        seq_printf(m, "NUM_RX_DESC\t0x%x\n", NUM_RX_DESC);
        seq_printf(m, "cur_rx0\t0x%x\n", tp->rx_ring[0].cur_rx);
        seq_printf(m, "cur_rx1\t0x%x\n", tp->rx_ring[1].cur_rx);
        seq_printf(m, "cur_rx2\t0x%x\n", tp->rx_ring[2].cur_rx);
        seq_printf(m, "cur_rx3\t0x%x\n", tp->rx_ring[3].cur_rx);
        seq_printf(m, "NUM_TX_DESC\t0x%x\n", NUM_TX_DESC);
        seq_printf(m, "cur_tx0\t0x%x\n", tp->tx_ring[0].cur_tx);
        seq_printf(m, "dirty_tx0\t0x%x\n", tp->tx_ring[0].dirty_tx);
        seq_printf(m, "cur_tx1\t0x%x\n", tp->tx_ring[1].cur_tx);
        seq_printf(m, "dirty_tx1\t0x%x\n", tp->tx_ring[1].dirty_tx);
        seq_printf(m, "rx_buf_sz\t0x%x\n", RX_BUF_SIZE);
        seq_printf(m, "R8125_RX_ALIGN\t0x%x\n", R8125_RX_ALIGN);
#ifdef ENABLE_ESD
        seq_printf(m, "esd_flag\t0x%x\n", tp->esd_flag);
        seq_printf(m, "pci_cfg_is_read\t0x%x\n", tp->pci_cfg_is_read);
#endif
        seq_printf(m, "rx_config\t0x%x\n", tp->rx_config);
        seq_printf(m, "cp_cmd\t0x%x\n", tp->cp_cmd);
        seq_printf(m, "intr_mask\t0x%x\n", tp->intr_mask);
        seq_printf(m, "timer_intr_mask\t0x%x\n", tp->timer_intr_mask);
        seq_printf(m, "wol_enabled\t0x%x\n", tp->wol_enabled);
        seq_printf(m, "wol_opts\t0x%x\n", tp->wol_opts);
        seq_printf(m, "autoneg\t0x%x\n", rtl8125_flag_to_bool(tp, AutoNegMode));
        seq_printf(m, "duplex\t0x%x\n", tp->duplex);
        seq_printf(m, "speed\t%d\n", tp->speed);
        seq_printf(m, "advertising\t0x%llx\n", tp->advertising);
 #ifdef ENABLE_R8125_EEPROM
        seq_printf(m, "eeprom_type\t%s\n", (rtl8125_flag_is_set(tp, EEPROM_TYPE_93C46)) ? "93C46" :
                (rtl8125_flag_is_set(tp, EEPROM_TYPE_93C56)) ? "9356": "NONE");
        seq_printf(m, "eeprom_len\t0x%x\n", rtl_get_eeprom_len(dev));
#endif
        seq_printf(m, "cur_page\t0x%x\n", tp->cur_page);
        seq_printf(m, "flags%lx\n", tp->flags);
        seq_printf(m, "org_pci_offset_99\t0x%x\n", tp->org_pci_offset_99);
        seq_printf(m, "org_pci_offset_180\t0x%x\n", tp->org_pci_offset_180);
        //seq_printf(m, "issue_offset_99_event\t0x%x\n", tp->issue_offset_99_event);
        seq_printf(m, "org_pci_offset_80\t0x%x\n", org_pci_offset_80);
        seq_printf(m, "org_pci_offset_81\t0x%x\n", org_pci_offset_81);
        seq_printf(m, "use_timer_interrupt\t0x%x\n", rtl8125_flag_to_bool(tp, UseIntrTimer));
        seq_printf(m, "HwHasWrRamCodeToMicroP\t0x%x\n", rtl8125_flag_to_bool(tp, HwHasWrRamCodeToMicroP));
        seq_printf(m, "sw_ram_code_ver\t0x%x\n", tp->sw_ram_code_ver);
        seq_printf(m, "hw_ram_code_ver\t0x%x\n", tp->hw_ram_code_ver);
#ifdef ENABLE_RTL_TOOL
        seq_printf(m, "rtk_enable_diag\t0x%x\n", tp->rtk_enable_diag);
#endif
        seq_printf(m, "NicCustLedValue\t0x%x\n", RTL_R16(tp, CustomLED));
        seq_printf(m, "RequiredPfmPatch\t0x%x\n", is_8125D(tp) ? 1 : 0);

        seq_printf(m, "speed_mode\t0x%x\n", speed_mode);
        seq_printf(m, "duplex_mode\t0x%x\n", duplex_mode);
        seq_printf(m, "autoneg_mode\t0x%x\n", autoneg_mode);
        seq_printf(m, "aspm\t0x%x\n", aspm);
        seq_printf(m, "s5wol\t0x%x\n", s5wol);
        seq_printf(m, "s5_keep_curr_mac\t0x%x\n", s5_keep_curr_mac);
        seq_printf(m, "eee_enable\t0x%x\n", tp->eee.eee_enabled);
        seq_printf(m, "hwoptimize\t0x%lx\n", hwoptimize);
        seq_printf(m, "proc_init_num\t0x%x\n", proc_init_num);
        seq_printf(m, "s0_magic_packet\t0x%x\n", s0_magic_packet);
        //seq_printf(m, "disable_wol_support\t0x%x\n", disable_wol_support);
        seq_printf(m, "enable_double_vlan\t0x%x\n", enable_double_vlan);
        seq_printf(m, "HwSuppLinkChgWakeUpVer\t0x%x\n", 3);
        seq_printf(m, "HwSuppD0SpeedUp\%s\n", HW_SUPP_D0_SPEED_UP(tp) ? "true" : "false");
        seq_printf(m, "D0SpeedUpSpeed\t0x%x\n", tp->D0SpeedUpSpeed);
        seq_printf(m, "HwSuppTxNoCloseVer\t0x%x\n", is_8125AB(tp) ? 3 : 6);
        seq_printf(m, "BeginHwDesCloPtr0\t0x%x\n", tp->tx_ring[0].BeginHwDesCloPtr);
        seq_printf(m, "hw_clo_ptr_reg0\t0x%x\n", rtl8125_hw_clo_ptr(tp, 0));
        seq_printf(m, "sw_tail_ptr_reg0\t0x%x\n", rtl8125_sw_tail_ptr(tp, 0));
        seq_printf(m, "BeginHwDesCloPtr1\t0x%x\n", tp->tx_ring[1].BeginHwDesCloPtr);
        seq_printf(m, "hw_clo_ptr_reg1\t0x%x\n", rtl8125_hw_clo_ptr(tp, 1));
        seq_printf(m, "sw_tail_ptr_reg1\t0x%x\n", rtl8125_sw_tail_ptr(tp, 1));
        seq_printf(m, "RxDescType\t0x%x\n", tp->RxDescType);
        seq_printf(m, "RxDescLength\t0x%x\n", tp->RxDescType & 0x3F);
        seq_printf(m, "num_rx_rings\t0x%x\n", tp->num_rx_rings);
        seq_printf(m, "num_tx_rings\t0x%x\n", tp->num_tx_rings);
        seq_printf(m, "tot_rx_rings\t0x%x\n", rtl8125_tot_rx_rings(tp));
        seq_printf(m, "tot_tx_rings\t0x%x\n", rtl8125_tot_tx_rings(tp));
        seq_printf(m, "HwSuppNumRxQueues\t0x%x\n", HW_SUPP_NUM_RX_QUEUES(tp));
        seq_printf(m, "HwSuppNumTxQueues\t0x%x\n", HW_SUPP_NUM_TX_QUEUES(tp));
#if defined(ENABLE_RSS_SUPPORT) || defined(ENABLE_PTP_SUPPORT)
        seq_printf(m, "EnableRss\t0x%x\n", rtl8125_flag_to_bool(tp, EnableRss));
#endif
#ifdef ENABLE_PTP_SUPPORT
        seq_printf(m, "EnablePtp\t0x%x\n", rtl8125_flag_to_bool(tp, EnablePtp));
        seq_printf(m, "ptp_master_mode\t0x%x\n", rtl8125_flag_to_bool(tp, PtpMasterMode));
        seq_printf(m, "tx_hwtstamp_timeouts\t0x%x\n", tp->tx_hwtstamp_timeouts);
        seq_printf(m, "tx_hwtstamp_skipped\t0x%x\n", tp->tx_hwtstamp_skipped);
#endif
        seq_printf(m, "min_irq_nvecs\t0x%x\n", rtl_chip_info[chipset].min_irq_nvecs);
        seq_printf(m, "max_irq_nvecs\t0x%x\n", rtl_chip_info[chipset].max_irq_nvecs);
        seq_printf(m, "irq_nvecs\t0x%x\n", tp->irq_nvecs);
        seq_printf(m, "HwSuppIsrVer\t0x%x\n", rtl_chip_info[chipset].HwSuppIsrVer);
        seq_printf(m, "HwCurrIsrVer\t0x%x\n", tp->HwCurrIsrVer);
        seq_printf(m, "RandomMac\t0x%x\n", rtl8125_flag_to_bool(tp, RandomMac));
        seq_printf(m, "org_mac_addr\t%pM\n", tp->org_mac_addr);
        seq_printf(m, "perm_addr\t%pM\n", dev->perm_addr);
        seq_printf(m, "dev_addr\t%pM\n", dev->dev_addr);

        rtnl_unlock();

        seq_putc(m, '\n');
        return 0;
}

static int proc_get_tally_counter(struct seq_file *m, void *v)
{
        struct net_device *dev = m->private;
        struct rtl8125_private *tp = netdev_priv(dev);
        struct rtl8125_counters *counters;
        dma_addr_t paddr;

        seq_puts(m, "\nDump Tally Counter\n");

        rtnl_lock();

        counters = tp->tally_vaddr;
        paddr = tp->tally_paddr;
        if (!counters) {
                seq_puts(m, "\nDump Tally Counter Fail\n");
                goto out_unlock;
        }

        rtl8125_dump_tally_counter(tp, paddr);

        seq_puts(m, "Statistics\tValue\n----------\t-----\n");
        seq_printf(m, "tx_packets\t%lld\n", le64_to_cpu(counters->tx_packets));
        seq_printf(m, "rx_packets\t%lld\n", le64_to_cpu(counters->rx_packets));
        seq_printf(m, "tx_errors\t%lld\n", le64_to_cpu(counters->tx_errors));
        seq_printf(m, "rx_errors\t%d\n", le32_to_cpu(counters->rx_errors));
        seq_printf(m, "rx_missed\t%d\n", le16_to_cpu(counters->rx_missed));
        seq_printf(m, "align_errors\t%d\n", le16_to_cpu(counters->align_errors));
        seq_printf(m, "tx_one_collision\t%d\n", le32_to_cpu(counters->tx_one_collision));
        seq_printf(m, "tx_multi_collision\t%d\n", le32_to_cpu(counters->tx_multi_collision));
        seq_printf(m, "rx_unicast\t%lld\n", le64_to_cpu(counters->rx_unicast));
        seq_printf(m, "rx_broadcast\t%lld\n", le64_to_cpu(counters->rx_broadcast));
        seq_printf(m, "rx_multicast\t%d\n", le32_to_cpu(counters->rx_multicast));
        seq_printf(m, "tx_aborted\t%d\n", le16_to_cpu(counters->tx_aborted));
        seq_printf(m, "tx_underrun\t%d\n", le16_to_cpu(counters->tx_underrun));

        seq_printf(m, "tx_octets\t%lld\n", le64_to_cpu(counters->tx_octets));
        seq_printf(m, "rx_octets\t%lld\n", le64_to_cpu(counters->rx_octets));
        seq_printf(m, "rx_multicast64\t%lld\n", le64_to_cpu(counters->rx_multicast64));
        seq_printf(m, "tx_unicast64\t%lld\n", le64_to_cpu(counters->tx_unicast64));
        seq_printf(m, "tx_broadcast64\t%lld\n", le64_to_cpu(counters->tx_broadcast64));
        seq_printf(m, "tx_multicast64\t%lld\n", le64_to_cpu(counters->tx_multicast64));
        seq_printf(m, "tx_pause_on\t%d\n", le32_to_cpu(counters->tx_pause_on));
        seq_printf(m, "tx_pause_off\t%d\n", le32_to_cpu(counters->tx_pause_off));
        seq_printf(m, "tx_pause_all\t%d\n", le32_to_cpu(counters->tx_pause_all));
        seq_printf(m, "tx_deferred\t%d\n", le32_to_cpu(counters->tx_deferred));
        seq_printf(m, "tx_late_collision\t%d\n", le32_to_cpu(counters->tx_late_collision));
        seq_printf(m, "tx_all_collision\t%d\n", le32_to_cpu(counters->tx_all_collision));
        seq_printf(m, "tx_aborted32\t%d\n", le32_to_cpu(counters->tx_aborted32));
        seq_printf(m, "align_errors32\t%d\n", le32_to_cpu(counters->align_errors32));
        seq_printf(m, "rx_frame_too_long\t%d\n", le32_to_cpu(counters->rx_frame_too_long));
        seq_printf(m, "rx_runt\t%d\n", le32_to_cpu(counters->rx_runt));
        seq_printf(m, "rx_pause_on\t%d\n", le32_to_cpu(counters->rx_pause_on));
        seq_printf(m, "rx_pause_off\t%d\n", le32_to_cpu(counters->rx_pause_off));
        seq_printf(m, "rx_pause_all\t%d\n", le32_to_cpu(counters->rx_pause_all));
        seq_printf(m, "rx_unknown_opcode\t%d\n", le32_to_cpu(counters->rx_unknown_opcode));
        seq_printf(m, "rx_mac_error\t%d\n", le32_to_cpu(counters->rx_mac_error));
        seq_printf(m, "tx_underrun32\t%d\n", le32_to_cpu(counters->tx_underrun32));
        seq_printf(m, "rx_mac_missed\t%d\n", le32_to_cpu(counters->rx_mac_missed));
        seq_printf(m, "rx_tcam_dropped\t%d\n", le32_to_cpu(counters->rx_tcam_dropped));
        seq_printf(m, "tdu\t%d\n", le32_to_cpu(counters->tdu));
        seq_printf(m, "rdu\t%d\n", le32_to_cpu(counters->rdu));

        seq_putc(m, '\n');

out_unlock:
        rtnl_unlock();

        return 0;
}

static int proc_get_registers(struct seq_file *m, void *v)
{
        struct net_device *dev = m->private;
        u16 max, begin; // = R8125_MAC_REGS_SIZE;
        u8 byte_rd;
        const u16 max_value[] = {R8125_MAC_REGS_SIZE, 0xB00, 0xD40, 0x2840};
        const u16 begin_value[] = {0x0000, 0x0A00, 0x0D00, 0x2800};
        struct rtl8125_private *tp = netdev_priv(dev);
        void __iomem *ioaddr = tp->mmio_addr;

        seq_puts(m, "\nDump MAC Registers\n");
        seq_puts(m, "Offset\tValue\n------\t-----\n");

        rtnl_lock();
        for (u16 k = 0; k < 4; k++) {
                max = max_value[k];
                begin = begin_value[k];
                for (u16 n = begin; n < max;) {
                        seq_printf(m, "\n0x%04x:\t", n);

                        for (u16 i = 0; i < 16 && n < max; i++, n++) {
                                byte_rd = readb(ioaddr + n);
                                seq_printf(m, "%02x ", byte_rd);
                        }
                }
        }
        rtnl_unlock();

        seq_putc(m, '\n');
        return 0;
}

static int proc_get_all_registers(struct seq_file *m, void *v)
{
        struct net_device *dev = m->private;
        int i, n, max;
        u8 byte_rd;
        struct rtl8125_private *tp = netdev_priv(dev);
        void __iomem *ioaddr = tp->mmio_addr;
        struct pci_dev *pdev = tp->pci_dev;

        seq_puts(m, "\nDump All MAC Registers\n");
        seq_puts(m, "Offset\tValue\n------\t-----\n");

        rtnl_lock();
        max = pci_resource_len(pdev, 2);
        max = min(max, 0x8000);
        for (n = 0; n < max;) {
                seq_printf(m, "\n0x%04x:\t", n);
                for (i = 0; i < 16 && n < max; i++, n++) {
                        byte_rd = readb(ioaddr + n);
                        seq_printf(m, "%02x ", byte_rd);
                }
        }

        rtnl_unlock();
        seq_printf(m, "\nTotal length:0x%X", max);
        seq_putc(m, '\n');
        return 0;
}

static int proc_get_pcie_phy(struct seq_file *m, void *v)
{
        struct net_device *dev = m->private;
        int i, n, max = R8125_EPHY_REGS_SIZE/2;
        u16 word_rd;
        struct rtl8125_private *tp = netdev_priv(dev);

        seq_puts(m, "\nDump PCIE PHY\n");
        seq_puts(m, "\nOffset\tValue\n------\t-----\n ");

        rtnl_lock();
        for (n = 0; n < max;) {
                seq_printf(m, "\n0x%02x:\t", n);
                for (i = 0; i < 8 && n < max; i++, n++) {
                        word_rd = rtl8125_ephy_read(tp, n);
                        seq_printf(m, "%04x ", word_rd);
                }
        }
        rtnl_unlock();

        seq_putc(m, '\n');
        return 0;
}

static int proc_get_eth_phy(struct seq_file *m, void *v)
{
        struct net_device *dev = m->private;
        int i, n, max = R8125_PHY_REGS_SIZE/2;
        unsigned long flags;
        u16 word_rd;
        struct rtl8125_private *tp = netdev_priv(dev);

        seq_puts(m, "\nDump Ethernet PHY\n");
        seq_puts(m, "\nOffset\tValue\n------\t-----\n ");

        spin_lock_irqsave(&tp->phy_lock, flags);
        seq_puts(m, "\n####################page 0##################\n ");
        rtl8125_mdio_set_page(tp, 0x0000);
        for (n = 0; n < max;) {
                seq_printf(m, "\n0x%02x:\t", n);

                for (i = 0; i < 8 && n < max; i++, n++) {
                        word_rd = rtl8125_mdio_read(tp, n);
                        seq_printf(m, "%04x ", word_rd);
                }
        }

        seq_puts(m, "\n####################extra reg##################\n ");
        n = 0xA400;
        seq_printf(m, "\n0x%02x:\t", n);
        for (i = 0; i < 8; i++, n+=2) {
                word_rd = rtl8125_mdio_direct_read_phy_ocp(tp, n);
                seq_printf(m, "%04x ", word_rd);
        }

        n = 0xA410;
        seq_printf(m, "\n0x%02x:\t", n);
        for (i = 0; i < 3; i++, n+=2) {
                word_rd = rtl8125_mdio_direct_read_phy_ocp(tp, n);
                seq_printf(m, "%04x ", word_rd);
        }

        n = 0xA434;
        seq_printf(m, "\n0x%02x:\t", n);
        word_rd = rtl8125_mdio_direct_read_phy_ocp(tp, n);
        seq_printf(m, "%04x ", word_rd);

        n = 0xA5D0;
        seq_printf(m, "\n0x%02x:\t", n);
        for (i = 0; i < 4; i++, n+=2) {
                word_rd = rtl8125_mdio_direct_read_phy_ocp(tp, n);
                seq_printf(m, "%04x ", word_rd);
        }

        n = 0xA61A;
        seq_printf(m, "\n0x%02x:\t", n);
        word_rd = rtl8125_mdio_direct_read_phy_ocp(tp, n);
        seq_printf(m, "%04x ", word_rd);

        n = 0xA6D0;
        seq_printf(m, "\n0x%02x:\t", n);
        for (i = 0; i < 3; i++, n+=2) {
                word_rd = rtl8125_mdio_direct_read_phy_ocp(tp, n);
                seq_printf(m, "%04x ", word_rd);
        }

        spin_unlock_irqrestore(&tp->phy_lock, flags);
        seq_putc(m, '\n');
        return 0;
}

static int proc_get_pci_registers(struct seq_file *m, void *v)
{
        struct net_device *dev = m->private;
        int i, n, max = R8125_PCI_REGS_SIZE;
        u32 dword_rd;
        struct rtl8125_private *tp = netdev_priv(dev);

        seq_puts(m, "\nDump PCI Registers\n");
        seq_puts(m, "\nOffset\tValue\n------\t-----\n ");

        rtnl_lock();

        for (n = 0; n < max;) {
                seq_printf(m, "\n0x%03x:\t", n);

                for (i = 0; i < 4 && n < max; i++, n+=4) {
                        pci_read_config_dword(tp->pci_dev, n, &dword_rd);
                        seq_printf(m, "%08x ", dword_rd);
                }
        }

        n = 0x110;
        pci_read_config_dword(tp->pci_dev, n, &dword_rd);
        seq_printf(m, "\n0x%03x:\t%08x ", n, dword_rd);
        n = 0x70c;
        pci_read_config_dword(tp->pci_dev, n, &dword_rd);
        seq_printf(m, "\n0x%03x:\t%08x ", n, dword_rd);

        rtnl_unlock();

        seq_putc(m, '\n');
        return 0;
}

static int proc_get_temperature(struct seq_file *m, void *v)
{
        struct net_device *dev = m->private;
        struct rtl8125_private *tp = netdev_priv(dev);
        u16 ts_digout, tj, fah;

        if (is_8125B(tp) || is_8125D(tp))
                seq_puts(m, "\nChip Temperature\n");
        else
                return -EOPNOTSUPP;

        rtnl_lock();
        if (!rtl8125_sysfs_testmode_on(tp)) {
                seq_puts(m, "\nPlease turn on ""/sys/class/net/<iface>/rtk_adv/testmode"".\n\n");
                rtnl_unlock();
                return 0;
        }

        netif_testing_on(dev);
        ts_digout = rtl8125_read_thermal_sensor(tp);
        netif_testing_off(dev);
        rtnl_unlock();

        tj = ts_digout / 2;
        if (ts_digout <= 512) {
                tj = ts_digout / 2;
                seq_printf(m, "Cel:%d\n", tj);
                fah = tj * (9/5) + 32;
                seq_printf(m, "Fah:%d\n", fah);
        } else {
                tj = (512 - ((ts_digout / 2) - 512)) / 2;
                seq_printf(m, "Cel:-%d\n", tj);
                fah = tj * (9/5) + 32;
                seq_printf(m, "Fah:-%d\n", fah);
        }

        seq_putc(m, '\n');
        return 0;
}

static int _proc_get_cable_info(struct seq_file *m, void *v, bool poe_mode)
{
        int i;
        u16 status;
        int cp_status[RTL8125_CP_NUM];
        int cp_len[RTL8125_CP_NUM] = {0};
        struct net_device *dev = m->private;
        struct rtl8125_private *tp = netdev_priv(dev);
        const char *pair_str[RTL8125_CP_NUM] = {"1-2", "3-6", "4-5", "7-8"};
        unsigned long flags;
        int ret;

        if (!is_8125AB(tp)) {
                ret = -EOPNOTSUPP;
                goto error_out;
        }

        spin_lock_irqsave(&tp->phy_lock, flags);
        if (!rtl8125_sysfs_testmode_on(tp)) {
                seq_puts(m, "\nPlease turn on ""/sys/class/net/<iface>/rtk_adv/testmode"".\n\n");
                ret = 0;
                goto error_unlock;
        }

        rtl8125_mdio_set_page(tp, 0x0000);
        if (rtl8125_mdio_read(tp, MII_BMCR) & BMCR_PDOWN) {
                ret = -EIO;
                goto error_unlock;
        }

        netif_testing_on(dev);

        status = RTL_R16(tp, PHYstatus);
        if (status & LinkStatus)
                seq_printf(m, "\nlink speed:%d",
                           rtl8125_convert_link_speed(status));
        else
                seq_puts(m, "\nlink status:off");

        rtl8125_get_cp_len(tp, cp_len);

        rtl8125_get_cp_status(tp, cp_status, poe_mode);

        seq_puts(m, "\npair\tlength\tstatus   \tpp\n");

        for (i=0; i<RTL8125_CP_NUM; i++) {
                if (cp_len[i] < 0)
                        seq_printf(m, "%s\t%s\t%s\t", pair_str[i], "none",
                                   rtl8125_get_cp_status_string(cp_status[i]));
                else
                        seq_printf(m, "%s\t%d\t%s\t", pair_str[i], cp_len[i],
                                   rtl8125_get_cp_status_string(cp_status[i]));
                if (cp_status[i] == rtl8125_cp_normal)
                        seq_printf(m, "none\n");
                else
                        seq_printf(m, "%dm\n", rtl8125_get_cp_pp(tp, i));
        }

        netif_testing_off(dev);
        seq_putc(m, '\n');
        ret = 0;

error_unlock:
        spin_unlock_irqrestore(&tp->phy_lock, flags);

error_out:
        return ret;
}

static int proc_get_cable_info(struct seq_file *m, void *v)
{
        return _proc_get_cable_info(m, v, 0);
}

static int proc_get_poe_cable_info(struct seq_file *m, void *v)
{
        return _proc_get_cable_info(m, v, 1);
}

static void _proc_dump_desc(struct seq_file *m, void *desc_base, u32 alloc_size)
{
        u32 *pdword;

        if (desc_base == NULL || alloc_size == 0)
                return;

        pdword = (u32*)desc_base;
        for (int i=0; i<(alloc_size/4); i++) {
                if (!(i % 4))
                        seq_printf(m, "\n%04x ", i);
                seq_printf(m, "%08x ", pdword[i]);
        }

        seq_putc(m, '\n');
        return;
}

static int proc_dump_rx_desc(struct seq_file *m, void *v)
{
        struct net_device *dev = m->private;
        struct rtl8125_private *tp = netdev_priv(dev);

        rtnl_lock();
        for (int i = 0; i < tp->num_rx_rings; i++) {
                struct rtl8125_rx_ring *ring = &tp->rx_ring[i];

                if (!ring)
                        continue;
                seq_printf(m, "\ndump rx %d desc:%d\n", i, NUM_RX_DESC);
                _proc_dump_desc(m, (void*)ring->RxDescArray, RX_DESC_ALLOC_SIZE(tp)); //ring->RxDescAllocSize)
        }
        rtnl_unlock();

        seq_putc(m, '\n');
        return 0;
}

static int proc_dump_tx_desc(struct seq_file *m, void *v)
{
        struct net_device *dev = m->private;
        struct rtl8125_private *tp = netdev_priv(dev);
        int i;

        rtnl_lock();
        for (i = 0; i < tp->num_tx_rings; i++) {
                struct rtl8125_tx_ring *ring = &tp->tx_ring[i];

                if (!ring)
                        continue;
                seq_printf(m, "\ndump tx %d desc:%d\n", i, NUM_TX_DESC);
                _proc_dump_desc(m, (void*)ring->TxDescArray, TX_DESC_ALLOC_SIZE); //ring->TxDescAllocSize);
        }
        rtnl_unlock();

        seq_putc(m, '\n');
        return 0;
}

static int proc_dump_msix_tbl(struct seq_file *m, void *v)
{
        struct net_device *dev = m->private;
        struct rtl8125_private *tp = netdev_priv(dev);
        int i, j, max_irq_nvecs = rtl_chip_info[chipset(tp)].max_irq_nvecs;
        void __iomem *ioaddr;

        /* ioremap MMIO region */
        ioaddr = ioremap(pci_resource_start(tp->pci_dev, 4), pci_resource_len(tp->pci_dev, 4));
        if (!ioaddr)
                return -EFAULT;

        rtnl_lock();

        seq_printf(m, "\ndump MSI-X Table. Total Entry %d. \n", max_irq_nvecs);

        for (i=0; i<max_irq_nvecs; i++) {
                seq_printf(m, "\n%04x ", i);
                for (j=0; j<4; j++)
                        seq_printf(m, "%08x ", readl(ioaddr + i*0x10 + 4*j));
        }

        rtnl_unlock();

        iounmap(ioaddr);

        seq_putc(m, '\n');
        return 0;
}

static void rtl8125_proc_module_init(void)
{
        //create /proc/net/r8125
        rtl8125_proc = proc_mkdir(MODULENAME, init_net.proc_net);
        if (!rtl8125_proc)
                dprintk("cannot create %s proc entry \n", MODULENAME);
}

/*
 * seq_file wrappers for procfile show routines.
 */
static int rtl8125_proc_open(struct inode *inode, struct file *file)
{
        struct net_device *dev = proc_get_parent_data(inode);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,17,0)
        int (*show)(struct seq_file *, void *) = pde_data(inode);
#else
        int (*show)(struct seq_file *, void *) = PDE_DATA(inode);
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(5,17,0)

        return single_open(file, show, dev);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
static const struct proc_ops rtl8125_proc_fops = {
        .proc_open           = rtl8125_proc_open,
        .proc_read           = seq_read,
        .proc_lseek          = seq_lseek,
        .proc_release        = single_release,
};
#else
static const struct file_operations rtl8125_proc_fops = {
        .open           = rtl8125_proc_open,
        .read           = seq_read,
        .llseek         = seq_lseek,
        .release        = single_release,
};
#endif

/*
 * Table of proc files we need to create.
 */
struct rtl8125_proc_file {
        char name[16];
        int (*show)(struct seq_file *, void *);
};

// Read with `cat /proc/net/r8125/<ethernet link device>/debug/<file>`
static const struct rtl8125_proc_file rtl8125_debug_proc_files[] = {
        { "driver_var", &proc_get_driver_variable },
        { "tally", &proc_get_tally_counter },
        { "registers", &proc_get_registers },
        { "registers2", &proc_get_all_registers },
        { "pcie_phy", &proc_get_pcie_phy },
        { "eth_phy", &proc_get_eth_phy },
        { "pci_regs", &proc_get_pci_registers },
        { "tx_desc", &proc_dump_tx_desc },
        { "rx_desc", &proc_dump_rx_desc },
        { "msix_tbl", &proc_dump_msix_tbl },
        { "", NULL }
};

static const struct rtl8125_proc_file rtl8125_test_proc_files[] = {
        { "temp", &proc_get_temperature },
        { "cdt", &proc_get_cable_info },
        { "cdt_poe", &proc_get_poe_cable_info },
        { "", NULL }
};

#define R8125_PROC_DEBUG_DIR "debug"
#define R8125_PROC_TEST_DIR "test"

static void rtl8125_proc_init(struct net_device *dev)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        const struct rtl8125_proc_file *f;
        struct proc_dir_entry *dir;

        if (!rtl8125_proc)
                return;

        if (tp->proc_dir_debug || tp->proc_dir_test)
                return;

        dir = proc_mkdir_data(dev->name, 0, rtl8125_proc, dev);
        if (!dir) {
                printk("Unable to initialize /proc/net/%s/%s\n",
                       MODULENAME, dev->name);
                return;
        }
        tp->proc_dir = dir;
        proc_init_num++;

        /* create debug entry */
        dir = proc_mkdir_data(R8125_PROC_DEBUG_DIR, 0, tp->proc_dir, dev);
        if (!dir) {
                printk("Unable to initialize /proc/net/%s/%s/%s\n",
                       MODULENAME, dev->name, R8125_PROC_DEBUG_DIR);
                return;
        }

        tp->proc_dir_debug = dir;
        for (f = rtl8125_debug_proc_files; f->name[0]; f++) {
                if (!proc_create_data(f->name, S_IFREG | S_IRUGO, dir,
                                      &rtl8125_proc_fops, f->show)) {
                        printk("Unable to initialize /proc/net/%s/%s/%s/%s\n",
                               MODULENAME, dev->name, R8125_PROC_DEBUG_DIR, f->name);
                        return;
                }
        }

        /* create test entry */
        dir = proc_mkdir_data(R8125_PROC_TEST_DIR, 0, tp->proc_dir, dev);
        if (!dir) {
                printk("Unable to initialize /proc/net/%s/%s/%s\n",
                       MODULENAME, dev->name, R8125_PROC_TEST_DIR);
                return;
        }

        tp->proc_dir_test = dir;
        for (f = rtl8125_test_proc_files; f->name[0]; f++) {
                if (!proc_create_data(f->name, S_IFREG | S_IRUGO, dir,
                                      &rtl8125_proc_fops, f->show)) {
                        printk("Unable to initialize /proc/net/%s/%s/%s/%s\n",
                               MODULENAME, dev->name, R8125_PROC_TEST_DIR, f->name);
                        return;
                }
        }
}

static void rtl8125_proc_remove(struct net_device *dev)
{
        struct rtl8125_private *tp = netdev_priv(dev);

        if (tp->proc_dir) {
                remove_proc_subtree(dev->name, rtl8125_proc);
                proc_init_num--;

                tp->proc_dir_debug = NULL;
                tp->proc_dir_test = NULL;
                tp->proc_dir = NULL;
        }
}

#endif //ENABLE_R8125_PROCFS

#ifdef ENABLE_R8125_SYSFS
/****************************************************************************
*   -----------------------------SYSFS STUFF-------------------------
*****************************************************************************
*/
static ssize_t testmode_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
        struct net_device *netdev = to_net_dev(dev);
        struct rtl8125_private *tp = netdev_priv(netdev);

        sprintf(buf, "%u\n", rtl8125_flag_to_bool(tp, TestMode));

        return strlen(buf);
}

static ssize_t testmode_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t count)
{
        struct net_device *netdev = to_net_dev(dev);
        struct rtl8125_private *tp = netdev_priv(netdev);
        u32 testmode;

        if (sscanf(buf, "%u\n", &testmode) != 1)
                return -EINVAL;

        rtnl_lock();
        if (testmode)
                rtl8125_set_flag(tp, TestMode);
        else
                rtl8125_clear_flag(tp, TestMode);
        rtnl_unlock();

        return count;
}

static DEVICE_ATTR_RW(testmode);

static struct attribute *rtk_adv_attrs[] = {
        &dev_attr_testmode.attr,
        NULL
};

static struct attribute_group rtk_adv_grp = {
        .name = "rtl_adv",
        .attrs = rtk_adv_attrs,
};

static inline void rtl8125_sysfs_init(struct rtl8125_private *tp, struct net_device *dev)
{
        /* init rtl_adv */
        rtl8125_set_flag(tp, TestMode);
        WARN_ON(sysfs_create_group(&dev->dev.kobj, &rtk_adv_grp));
}

static void rtl8125_sysfs_remove(struct net_device *dev)
{
        sysfs_remove_group(&dev->dev.kobj, &rtk_adv_grp);
}
#endif //ENABLE_R8125_SYSFS

static bool
rtl8125_timed_wait(void __iomem *addr, bool b, enum RTL8125_register_content flag, int delay, int count)
{
        for (int i = 0; i < count; i++) {
                udelay(delay);
                if ((_RTL_R32(addr) & flag) == b)
                        return 1;
        }
        return 0;
}

static __always_inline void rtl8125_mdio_set_page(struct rtl8125_private * const tp, const u16 value)
{
        tp->cur_page = value;
}

static __always_inline u16 map_phy_ocp_addr(u16 page, u16 reg)
{
        return (page == 0) ? OCP_STD_PHY_BASE_PAGE + (reg << 1): 0;
}

void
rtl8125_mdio_direct_write_phy_ocp(const struct rtl8125_private *const tp, const u16 reg, const u16 value)
{
#ifdef ENABLE_RTL_TOOL
        if (unlikely(tp->rtk_enable_diag))
                return;
#endif
        RTL_W32(tp, PHYOCP, OCPR_Write | (reg << 15) | value);
        rtl8125_timed_wait(RTL_R32_ADDR(tp, PHYOCP), false, OCPR_Flag, 5, 50);
        udelay(5);
}

u32 rtl8125_mdio_direct_read_phy_ocp(const struct rtl8125_private * const tp, const u16 reg)
{
#ifdef ENABLE_RTL_TOOL
        if (unlikely(tp->rtk_enable_diag))
                return 0xffffffff;
#endif

        RTL_W32(tp, PHYOCP, reg << 15);
        rtl8125_timed_wait(RTL_R32_ADDR(tp, PHYOCP), true, OCPR_Flag, 5, 50);
        return RTL_R32(tp, PHYOCP) & OCPDR_Data_Mask;
}

static inline void rtl8125_mdio_write(struct rtl8125_private * const tp, const u16 reg, const u16 value)
{
#ifdef ENABLE_RTL_TOOL
        if (tp->rtk_enable_diag)
                return;
#endif
        rtl8125_mdio_direct_write_phy_ocp(tp,
                map_phy_ocp_addr(tp->cur_page, reg), value);
}

static inline u32 rtl8125_mdio_read(const struct rtl8125_private * const tp, const u16 reg)
{
        return rtl8125_mdio_direct_read_phy_ocp(tp, map_phy_ocp_addr(tp->cur_page, reg));
}

static inline void
rtl8125_clear_and_set_eth_phy_ocp_bit(const struct rtl8125_private * const tp,
        const u16 reg, const u16 clearmask, const u16 setmask)
{
        rtl8125_mdio_direct_write_phy_ocp(tp, reg,
                (rtl8125_mdio_direct_read_phy_ocp(tp, reg) & ~clearmask) | setmask);
}

static inline void rtl8125_clear_eth_phy_ocp_bit(const struct rtl8125_private * const tp, const u16 reg, const u16 clearmask)
{
        rtl8125_mdio_direct_write_phy_ocp(tp, reg,
                rtl8125_mdio_direct_read_phy_ocp(tp, reg) & ~clearmask);
}

static inline void rtl8125_set_eth_phy_ocp_bit(const struct rtl8125_private *const tp,  const u16 reg, const u16 setmask)
{
        rtl8125_mdio_direct_write_phy_ocp(tp, reg,
                rtl8125_mdio_direct_read_phy_ocp(tp, reg) | setmask);
}

static inline void rtl8125_mac_ocp_write(const struct rtl8125_private * const tp, const u16 reg, const u16 value)
{
        WARN_ON_ONCE(reg & 0x0001); // Uneven register address means trouble
        //WARN_ONCE(reg_addr & 0xffff0001, "Invalid ocp reg %x!\n", reg);
        RTL_W32(tp, MACOCP, OCPR_Write | (reg << 15) | value);
}

/* Moved to r8125.h
static inline u16 rtl8125_mac_ocp_read(struct rtl8125_private *tp, u16 reg_addr)
{
        RTL_W32(tp, MACOCP, reg_addr << 15);
        return (u16)RTL_R32(tp, MACOCP);
}
*/

#ifdef ENABLE_USE_FIRMWARE_FILE
static void mac_mcu_write(struct rtl8125_private *const tp, const u16 reg, const u16 value)
{
        if (reg == 0x1f) {
                tp->ocp_base = value << 4;
                return;
        }
        rtl8125_mac_ocp_write(tp, tp->ocp_base + reg, value);
}

static u32 mac_mcu_read(const struct rtl8125_private * const tp, const u16 reg)
{
        return rtl8125_mac_ocp_read(tp, tp->ocp_base + reg);
}
#endif

void
rtl8125_clear_set_mac_ocp_bit(const struct rtl8125_private * const tp, const u16 addr, const u16 clearmask, const u16 setmask)
{
        u16 PhyRegValue = rtl8125_mac_ocp_read(tp, addr);
        rtl8125_mac_ocp_write(tp, addr, (PhyRegValue & ~clearmask) | setmask);
}

void rtl8125_ephy_write(const struct rtl8125_private * const tp, const u8 reg, const u16 value)
{
        RTL_W32(tp, EPHYAR, EPHYAR_Write | (reg & EPHYAR_Reg_Mask_v2) << EPHYAR_Reg_shift |
                (value & EPHYAR_Data_Mask));
        rtl8125_timed_wait(RTL_R32_ADDR(tp, EPHYAR), false, EPHYAR_Flag, 10, 100);
        udelay(10);
}

u16 rtl8125_ephy_read(const struct rtl8125_private * const tp, const int reg)
{
        RTL_W32(tp, EPHYAR, EPHYAR_Read | (reg & EPHYAR_Reg_Mask_v2) << EPHYAR_Reg_shift);
        return rtl8125_timed_wait(RTL_R32_ADDR(tp, EPHYAR), true, EPHYAR_Flag, 10, 100) ?
                (u16) (RTL_R32(tp, EPHYAR) & EPHYAR_Data_Mask) : ~0;
}

static inline void
ClearAndSetPCIePhyBit(const struct rtl8125_private * const tp, const u8 addr, const u16 clearmask, const u16 setmask)
{
        rtl8125_ephy_write(tp, addr, (rtl8125_ephy_read(tp, addr) & ~clearmask) | setmask);
}

static inline void ClearPCIePhyBit(const struct rtl8125_private * const tp, const u8 addr, const u16 mask)
{
        rtl8125_ephy_write(tp, addr, rtl8125_ephy_read(tp, addr) & ~mask);
}

static inline void SetPCIePhyBit(const struct rtl8125_private * const tp, const u8 addr, const u16 mask)
{
        rtl8125_ephy_write(tp, addr, rtl8125_ephy_read(tp, addr) | mask);
}

static u32
rtl8125_csi_fun_read(const struct rtl8125_private * const tp, u8 func, const u32 addr)
{
        u32 rc;
        func &= 0x07;
        RTL_W32(tp, CSIAR, CSIAR_Read | CSIAR_ByteEn << CSIAR_ByteEn_shift |
                (addr & CSIAR_Addr_Mask) | func << 16);
        rc = rtl8125_timed_wait(RTL_R32_ADDR(tp, CSIAR), true, CSIAR_Flag, 5, 200) ?
                (u32)(RTL_R32(tp, CSIDR)) : 0xFFFFFFFF;
        udelay(5);
        return rc;
}

static void
rtl8125_csi_fun_write(const struct rtl8125_private * const tp, u8 func, const u32 addr, const u32 value)
{
        RTL_W32(tp, CSIDR, value);
        func &= 0x07;
        RTL_W32(tp, CSIAR, CSIAR_Write | CSIAR_ByteEn << CSIAR_ByteEn_shift |
                (addr & CSIAR_Addr_Mask) | func << 16);
        rtl8125_timed_wait(RTL_R32_ADDR(tp, CSIAR), false, CSIAR_Flag, 5, 200);
        udelay(5);
}

static __always_inline u32
rtl8125_csi_read(const struct rtl8125_private * const tp, const u32 addr)
{
        return rtl8125_csi_fun_read(tp, (u8)0, addr);
}

static __always_inline void
rtl8125_csi_write(const struct rtl8125_private * const tp, const u32 addr, const u32 value)
{
        rtl8125_csi_fun_write(tp, (u8)0, addr, value);
}

static u8
rtl8125_csi_fun0_read_byte(const struct rtl8125_private * const tp, const u32 addr)
{
        u32 tmp_u32 = rtl8125_csi_fun_read(tp, 0, addr & ~0x3);
        tmp_u32 >>= (addr & 0x3) << 3; // (8*ShiftByte);
        return (u8)tmp_u32;
}

static void
rtl8125_csi_fun0_write_byte(const struct rtl8125_private * const tp, u32 addr, const u8 value)
{
        u32 tmp_u32;
        u8 shift = (addr & 0x3) * 8;
        addr &= ~(0x3); // align to 32 bit
        tmp_u32 = rtl8125_csi_fun_read(tp, 0, addr);
        tmp_u32 &= ~(0xFF << shift) | (value << shift);
        rtl8125_csi_fun_write(tp, 0, addr, tmp_u32);
}

static inline void
rtl8125_enable_rxdvgate(const struct rtl8125_private * const tp)
{
        RTL_W8(tp, 0xF2, RTL_R8(tp, 0xF2) | BIT_3);
        mdelay(2);
}

static inline void
rtl8125_disable_rxdvgate(const struct rtl8125_private * const tp)
{
        RTL_W8(tp, 0xF2, RTL_R8(tp, 0xF2) & ~BIT_3);
        mdelay(2);
}

static u8
rtl8125_is_gpio_low(const struct rtl8125_private * const tp)
{
        if (!(rtl8125_mac_ocp_read(tp, 0xDC04) & BIT_13)){
                dprintk("gpio is low.\n");
                return TRUE;
        }
        return FALSE;
}

static u8
rtl8125_is_phy_disable_mode_enabled(const struct rtl8125_private * const tp)
{
        if (RTL_R8(tp, 0xF2) & BIT_5) {
                dprintk("phy disable mode enabled.\n");
                return TRUE;
        }
        return FALSE;
}

static u8
rtl8125_is_in_phy_disable_mode(const struct rtl8125_private * const tp)
{
        if (rtl8125_is_phy_disable_mode_enabled(tp) && rtl8125_is_gpio_low(tp)) {
                dprintk("Hardware is in phy disable mode.\n");
                return TRUE;
        }
        return FALSE;
}

static inline void rtl8125_stop_all_request(const struct rtl8125_private * const tp)
{
        RTL_W8(tp, ChipCmd, RTL_R8(tp, ChipCmd) | StopReq);
        //if (rtl8125_flag_is_set(tp, IsMcfg236)) {
        if (is_8125A(tp)) {
                for (int i = 0; i < 20; i++) {
                        udelay(10);
                        if (!(RTL_R8(tp, ChipCmd) & StopReq))
                                goto out;
                }
                return;
        } else // other chips
                udelay(200);
out:
        RTL_W8(tp, ChipCmd, RTL_R8(tp, ChipCmd) & (CmdTxEnb | CmdRxEnb));
}

void rtl8125_wait_txrx_fifo_empty(const struct rtl8125_private * const tp)
{
        //bool isNotMcfg236 = !(rtl8125_flag_is_set(tp, IsMcfg236));
        const bool is_not_8125A = !is_8125A(tp);
        int i;

        if (is_not_8125A)        // Stop all requests
                RTL_W8(tp, ChipCmd, RTL_R8(tp, ChipCmd) | StopReq);

        for (i = 0; i < 3000; i++) {
                udelay(50);
                if ((RTL_R8(tp, MCUCmd_reg) & (Txfifo_empty | Rxfifo_empty)) == (Txfifo_empty | Rxfifo_empty))
                                break;
        }

        if (is_not_8125A)
                for (i = 0; i < 3000; i++) {
                        udelay(50);
                        if ((RTL_R16(tp, RxTxFifo) & (BIT_0 | BIT_1 | BIT_8)) == (BIT_0 | BIT_1 | BIT_8))  // 0x0103
                                break;
                }
}

static void
rtl8125_enable_hw_linkchg_interrupt(const struct rtl8125_private * const tp)
{
        if (tp->LinkChgShift != 5)
                RTL_W32(tp, IMR_V2_SET_REG_8125, 1 << tp->LinkChgShift);
        else
                RTL_W32(tp, IMR0_8125, LinkChg | RTL_R32(tp, IMR0_8125));

}

static inline void
rtl8125_enable_hw_interrupt(const struct rtl8125_private * const tp)
{
        if (likely(tp->HwCurrIsrVer > 1))
                RTL_W32(tp, IMR_V2_SET_REG_8125, tp->intr_mask);
        else {
                RTL_W32(tp, IMR0_8125, tp->intr_mask);
                for (u16 i = 1, imr_reg = IMR1_8125; i < tp->num_rx_rings; i++, imr_reg += 4)
                        RTL_W16(tp, imr_reg, other_q_intr_mask); // tp->imr_reg[i]
        }
}

static __always_inline void
rtl8125_clear_hw_isr_v2(const struct rtl8125_private * const tp, const u8 message_id)
{
        RTL_W32(tp, ISR_V2_8125, BIT(message_id));
}

static __always_inline void
rtl8125_disable_hw_interrupt(const struct rtl8125_private * const tp)
{
        if (likely(tp->HwCurrIsrVer > 1)) {
                RTL_W32(tp, IMR_V2_CLEAR_REG_8125, 0xFFFFFFFF);
                if (tp->HwCurrIsrVer > 3)
                        RTL_W32(tp, IMR_V4_L2_CLEAR_REG_8125, 0xFFFFFFFF);
        } else {
                RTL_W32(tp, IMR0_8125, 0x0000);
                for (u16 i = 1, imr_reg = IMR1_8125; i < tp->num_rx_rings; i++, imr_reg += 4)
                        RTL_W16(tp, imr_reg, 0);
                //        for (int i=1; i<tp->num_rx_rings; i++) RTL_W16(tp, tp->imr_reg[i], 0);
        }
}

static __always_inline void
rtl8125_switch_to_hw_interrupt(const struct rtl8125_private * const tp)
{
        RTL_W32(tp, TIMER_INT0_8125, 0x0000);
        rtl8125_enable_hw_interrupt(tp);
}

static __always_inline void
rtl8125_switch_to_timer_interrupt(const struct rtl8125_private * const tp)
{
        if (rtl8125_flag_is_set(tp, UseIntrTimer)) {
                RTL_W32(tp, TIMER_INT0_8125, timer_count);
                RTL_W32(tp, TCTR0_8125, timer_count);
                RTL_W32(tp, IMR0_8125, tp->timer_intr_mask);
        } else
                rtl8125_switch_to_hw_interrupt(tp);
}

static void
rtl8125_irq_mask_and_ack(const struct rtl8125_private * const tp)
{
        //static const u16 isr_reg[] = { ISR0_8125, ISR1_8125, ISR2_8125, ISR3_8125 };
        u16 isr_reg = ISR1_8125;
        rtl8125_disable_hw_interrupt(tp);

        if (tp->HwCurrIsrVer > 1) {
                RTL_W32(tp, ISR_V2_8125, 0xFFFFFFFF);
                if (tp->HwCurrIsrVer > 3)
                        RTL_W32(tp, ISR_V4_L2_8125, 0xFFFFFFFF);
        } else {
                RTL_W32(tp, ISR0_8125, RTL_R32(tp, ISR0_8125));
                for (u16 i = 1; i < tp->num_rx_rings; i++, isr_reg += 4)
                        RTL_W16(tp, isr_reg, RTL_R16(tp, isr_reg));
        }
}

static void
rtl8125_disable_rx_packet_filter(const struct rtl8125_private * const tp)
{
        RTL_W32(tp, RxConfig, RTL_R32(tp, RxConfig) & ~(AcceptErr | AcceptRunt |
                AcceptBroadcast | AcceptMulticast | AcceptMyPhys |  AcceptAllPhys));
}

static void
rtl8125_nic_reset(const struct rtl8125_private * const tp)
{
        rtl8125_disable_rx_packet_filter(tp);
        rtl8125_enable_rxdvgate(tp);
        rtl8125_stop_all_request(tp);
        rtl8125_wait_txrx_fifo_empty(tp);
        mdelay(2);

        /* Soft reset the chip. */
        RTL_W8(tp, ChipCmd, CmdReset);

        /* Check that the chip has finished the reset. */
        for (int i = 100; i > 0; i--) {
                udelay(100);
                if ((RTL_R8(tp, ChipCmd) & CmdReset) == 0)
                        break;
        }

        /* reset rcr */
        RTL_W32(tp, RxConfig, (RX_DMA_BURST_512 << RxCfgDMAShift));
}

static void
rtl8125_hw_set_interrupt_type(const struct rtl8125_private * const tp)
{
        u8 cfg = RTL_R8(tp, INT_CFG0_8125);

        switch (rtl_chip_info[chipset(tp)].HwSuppIsrVer) {
        case 4 ... 5:
                cfg &= ~INT_CFG0_MSIX_ENTRY_NUM_MODE;
                fallthrough;
        case 2: // ... 3:  does not exist
                cfg = (tp->HwCurrIsrVer > 1) ? cfg | INT_CFG0_ENABLE_8125 : cfg & ~(INT_CFG0_ENABLE_8125);
                RTL_W8(tp, INT_CFG0_8125, cfg);
        }
}

static void
rtl8125_hw_clear_int_timer(const struct rtl8125_private * const tp)
{
        RTL_W32(tp, TIMER_INT0_8125, 0x0000);
        RTL_W32(tp, TIMER_INT1_8125, 0x0000);
        RTL_W32(tp, TIMER_INT2_8125, 0x0000);
        RTL_W32(tp, TIMER_INT3_8125, 0x0000);
}

/* values for HwSuppIntMitiVer are 3,4 6 and 7 */
static void
rtl8125_hw_clear_int_miti_timer(const struct rtl8125_private *const tp)
{
        //  IntMITI_0-IntMITI_15 or IntMITI_31
        for (int i = 0xA00; i < 0xB00; i += 4) {
                RTL_W32(tp, i, 0x0000);
                if (i == 0xA80 && (rtl_chip_info[chipset(tp)].HwSuppIntMitiVer == 4)) { // stop at IntMITI_15
                        RTL_W8(tp, INT_CFG0_8125, RTL_R8(tp, INT_CFG0_8125) &
                                ~(INT_CFG0_TIMEOUT0_BYPASS_8125 | INT_CFG0_MITIGATION_BYPASS_8125));
                        RTL_W16(tp, INT_CFG1_8125, 0x0000);
                        break;
                }
        }
}

static void
rtl8125_hw_set_int_miti_timer(const struct rtl8125_private *const tp, const u8 message_id, const u8 timer)
{
        u8 HwSuppIntMitiVer = rtl_chip_info[chipset(tp)].HwSuppIntMitiVer;
        //printk(KERN_INFO "r8125 hw_set_int_miti_timer %d %d\n", timer, message_id);

        //if ((tp->HwCurrIsrVer == 2) && (message_id < R8125_MAX_RX_QUEUES_VEC_V3))
        //        timer_intmiti_val = 0;
        //ROK
        if (message_id < R8125_MAX_RX_QUEUES_VEC_V3) {
                RTL_W8(tp, INT_MITI_V2_0_RX + 8 * message_id,
                        (tp->HwCurrIsrVer == 2) ? 0: timer);
                if (HwSuppIntMitiVer == 6 && message_id < tp->num_tx_rings) // RTL8125BP
                        RTL_W8(tp,INT_MITI_V2_0_TX + 8 * message_id, timer);
        //TOK
        } else if (HwSuppIntMitiVer == 4) { // RTL8125B
                if (message_id == 16)
                        RTL_W8(tp, INT_MITI_V2_0_TX, timer);
                if (message_id == 18 && tp->num_tx_rings > 1)
                        RTL_W8(tp, INT_MITI_V2_1_TX, timer);
        } else if (HwSuppIntMitiVer == 7) {  // RTL8125D
                if (message_id == 16)
                        RTL_W8(tp, INT_MITI_V2_0_TX, timer);
                if (message_id == 17 && tp->num_tx_rings > 1)
                        RTL_W8(tp, INT_MITI_V2_1_TX, timer);
        }
}

static void
rtl8125_hw_reset(const struct rtl8125_private *const tp)
{
        /* Disable interrupts */
        rtl8125_irq_mask_and_ack(tp);
        rtl8125_hw_clear_int_timer(tp);
        rtl8125_nic_reset(tp);
}

static inline unsigned int
rtl8125_xmii_link_ok(const struct rtl8125_private *const tp)
{
        const u16 status = RTL_R16(tp, PHYstatus);
        if (status == 0xffff)
                return 0;

        return (status & LinkStatus) ? 1 : 0;
}

static int
rtl8125_wait_phy_reset_complete(const struct rtl8125_private *const tp)
{
        for (int i = 0; i < 2500; i++) {
                if (!(rtl8125_mdio_read(tp, MII_BMCR) & BMCR_RESET))
                        return 0;
                mdelay(1);
        }
        return -1;
}

static void
rtl8125_xmii_reset_enable(struct rtl8125_private *const tp)
{
        if (rtl8125_is_in_phy_disable_mode(tp))
                return;

        rtl8125_mdio_set_page(tp, 0x0000);
        rtl8125_mdio_write(tp, MII_ADVERTISE, rtl8125_mdio_read(tp, MII_ADVERTISE) &
                           ~(ADVERTISE_10HALF | ADVERTISE_10FULL |
                             ADVERTISE_100HALF | ADVERTISE_100FULL));
        rtl8125_mdio_write(tp, MII_CTRL1000, rtl8125_mdio_read(tp, MII_CTRL1000) &
                           ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL));
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA5D4, rtl8125_mdio_direct_read_phy_ocp(tp, 0xA5D4) &
                                          ~RTK_ADVERTISE_2500FULL);
        rtl8125_mdio_write(tp, MII_BMCR, BMCR_RESET | BMCR_ANENABLE);

        if (!rtl8125_wait_phy_reset_complete(tp))
                if (netif_msg_link(tp))
                        printk(KERN_ERR "%s: PHY reset failed.\n", tp->dev->name);
}

void
rtl8125_init_ring_indexes(struct rtl8125_private *const tp)
{
        int i, hwSuppNumTxQueues = HW_SUPP_NUM_TX_QUEUES(tp);
        int hwSuppNumRxQueues = HW_SUPP_NUM_RX_QUEUES(tp);

        for (i = 0; i < hwSuppNumTxQueues; i++) {
                struct rtl8125_tx_ring * const ring = tp->tx_ring + i;
                ring->dirty_tx = ring->cur_tx = 0;
                ring->BeginHwDesCloPtr = 0;
                /* reset BQL for queue */
                netdev_tx_reset_queue(netdev_get_tx_queue(tp->dev, i));
        }

        for (i = 0; i < hwSuppNumRxQueues; i++)
                tp->rx_ring[i].cur_rx = 0;
}

static void
rtl8125_issue_offset_99_event(const struct rtl8125_private *const tp)
{
        rtl8125_set_mac_ocp_bit(tp, 0xE09A, BIT_0);
}

static void rtl8125_enable_eee_plus(const struct rtl8125_private *const tp)
{
        rtl8125_set_mac_ocp_bit(tp, 0xE080, BIT_1);
}

static void rtl8125_disable_eee_plus(const struct rtl8125_private *const tp)
{
        rtl8125_clear_mac_ocp_bit(tp, 0xE080, BIT_1);
}

#ifdef ENABLE_DOUBLE_VLAN
static void rtl8125_enable_double_vlan(const struct rtl8125_private *const tp)
{
        RTL_W16(tp, DOUBLE_VLAN_CONFIG, 0xf002);
}
#else
static void rtl8125_disable_double_vlan(const struct rtl8125_private *const tp)
{
        RTL_W16(tp, DOUBLE_VLAN_CONFIG, 0);
}
#endif

static void
rtl8125_set_pfm_patch(const struct rtl8125_private *const tp, const bool enable)
{
        if (enable) {
                rtl8125_set_mac_ocp_bit(tp, 0xD3F0, BIT_0);
                rtl8125_set_mac_ocp_bit(tp, 0xD3F2, BIT_0);
                rtl8125_set_mac_ocp_bit(tp, 0xE85A, BIT_6);
        } else {
                rtl8125_clear_mac_ocp_bit(tp, 0xD3F0, BIT_0);
                rtl8125_clear_mac_ocp_bit(tp, 0xD3F2, BIT_0);
                rtl8125_clear_mac_ocp_bit(tp, 0xE85A, BIT_6);
        }
}

static void
rtl8125_link_up_patch(struct rtl8125_private *const tp, struct net_device *const dev)
{
        u8 supports_10bps;
        unsigned long flags;
        rtl8125_hw_config(tp, dev);

        if ((tp->mcfg == CFG_METHOD_2) &&
            netif_running(dev)) {
                if (RTL_R16(tp, PHYstatus)&FullDup)
                        RTL_W32(tp, TxConfig, (RTL_R32(tp, TxConfig) | (BIT_24 | BIT_25)) & ~BIT_19);
                else
                        RTL_W32(tp, TxConfig, (RTL_R32(tp, TxConfig) | BIT_25) & ~(BIT_19 | BIT_24));
        }

        //if (likely(tp->mcfg <= CFG_METHOD_9) && (RTL_R8(tp, PHYstatus) & _10bps))
        //        rtl8125_enable_eee_plus(tp);

        supports_10bps = RTL_R8(tp, PHYstatus) & _10bps;
        if (is_8125D(tp)) // (tp->mcfg >= CFG_METHOD_10)
                rtl8125_set_pfm_patch(tp, supports_10bps);
        else if (supports_10bps)
                rtl8125_enable_eee_plus(tp);

        rtl8125_hw_start(tp);
        netif_carrier_on(dev);
        netif_tx_wake_all_queues(dev);

        spin_lock_irqsave(&tp->phy_lock, flags);
        tp->phy_reg_aner = rtl8125_mdio_read(tp, MII_EXPANSION);
        tp->phy_reg_anlpar = rtl8125_mdio_read(tp, MII_LPA);
        tp->phy_reg_gbsr = rtl8125_mdio_read(tp, MII_STAT1000);
        tp->phy_reg_status_2500 = rtl8125_mdio_direct_read_phy_ocp(tp, 0xA5D6);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
}

static void
rtl8125_link_down_patch(struct rtl8125_private *const tp, struct net_device *const dev)
{
        tp->phy_reg_aner = 0;
        tp->phy_reg_anlpar = 0;
        tp->phy_reg_gbsr = 0;
        tp->phy_reg_status_2500 = 0;

        if (is_8125D(tp))
                rtl8125_set_pfm_patch(tp, 1);
        else
                rtl8125_disable_eee_plus(tp);

        netif_carrier_off(dev);
        netif_tx_disable(dev);
        rtl8125_hw_reset(tp);
        rtl8125_tx_clear_rings(tp);
        rtl8125_rx_clear_rings(tp);
        rtl8125_init_rings(tp);
        rtl8125_enable_hw_linkchg_interrupt(tp);

        //rtl8125_set_speed(dev, tp->autoneg, tp->speed, tp->duplex, tp->advertising);

}

static inline void
_rtl8125_check_link_status(struct rtl8125_private *const tp, struct net_device *const dev)
{
        if (rtl8125_xmii_link_ok(tp)) {
                rtl8125_link_up_patch(tp, dev);
                if (netif_msg_ifup(tp))
                        printk(KERN_INFO PFX "%s: link up\n", dev->name);
        } else {
                if (netif_msg_ifdown(tp))
                        printk(KERN_INFO PFX "%s: link down\n", dev->name);
                rtl8125_link_down_patch(tp, dev);
        }
}

static void
rtl8125_check_link_status(struct net_device *const dev)
{
        struct rtl8125_private * const tp = netdev_priv(dev);
        unsigned int link_status_on = rtl8125_xmii_link_ok(tp);
        rtl8125_clear_flag(tp, ResumeNoSpeedChange);
        if (netif_carrier_ok(dev) == link_status_on)
                rtl8125_enable_hw_linkchg_interrupt(tp);
        else
                _rtl8125_check_link_status(tp, dev);
}

static inline bool
rtl8125_is_autoneg_mode_valid(const u32 autoneg)
{
        return ((autoneg | 0x1) == 0x1);
}

static bool
rtl8125_is_speed_mode_valid(const u32 speed)
{
        switch(speed) {
        case SPEED_2500:
        case SPEED_1000:
        case SPEED_100:
        case SPEED_10:
                return true;
        }
        return false;
}

static inline bool
rtl8125_is_duplex_mode_valid(const u8 duplex)
{
        return ((duplex | 0x1) == 0x1);
}

static void
rtl8125_set_link_option(struct rtl8125_private *const tp, const u8 autoneg, u32 speed,
        u8 duplex, const enum rtl8125_fc_mode fc)
{
        if (!rtl8125_is_speed_mode_valid(speed))
                speed = SPEED_2500;

        if (!rtl8125_is_duplex_mode_valid(duplex))
                duplex = DUPLEX_FULL;

        if (!rtl8125_is_autoneg_mode_valid(autoneg) || autoneg)
                rtl8125_set_flag(tp, AutoNegMode);
        else
                rtl8125_clear_flag(tp, AutoNegMode);

        //speed = min(speed, tp->HwSuppMaxPhyLinkSpeed);
        speed = min(speed, (u32)(rtl8125_flag_is_set(tp, HwSuppPhyLinkSpeed_2500) ? 2500 : 1000));

        tp->advertising = ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full |
                ADVERTISED_100baseT_Half | ADVERTISED_100baseT_Full |
                ADVERTISED_1000baseT_Half | ADVERTISED_1000baseT_Full;
        if (speed == SPEED_2500)
                tp->advertising |= ADVERTISED_2500baseX_Full;

        tp->speed = speed;
        tp->duplex = duplex;
        tp->fcpause = fc;
}

/*
static void
rtl8125_enable_ocp_phy_power_saving(struct net_device *dev)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        u16 val;

        if (rtl8125_flag_is_set(tp, IsMcfg236)) {
                val = rtl8125_mdio_direct_read_phy_ocp(tp, 0xC416);
                if (val != 0x0050) {
                        rtl8125_set_phy_mcu_patch_request(tp);
                        rtl8125_mdio_direct_write_phy_ocp(tp, 0xC416, 0x0000);
                        rtl8125_mdio_direct_write_phy_ocp(tp, 0xC416, 0x0050);
                        rtl8125_clear_phy_mcu_patch_request(tp);
                }
        }
}
*/

static void
rtl8125_disable_ocp_phy_power_saving(struct rtl8125_private * const tp)
{
        if (rtl8125_mdio_direct_read_phy_ocp(tp, 0xC416) != 0x0500) {
                rtl8125_set_phy_mcu_patch_request(tp);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xC416, 0x0000);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xC416, 0x0500);
                rtl8125_clear_phy_mcu_patch_request(tp);
        }
}

static void
rtl8125_wait_ll_share_fifo_ready(const struct rtl8125_private * const tp)
{
        for (int i = 0; i < 10; i++) {
                udelay(100);
                if (RTL_R16(tp, 0xD2) & BIT_9)
                        return;
        }
}

static void
rtl8125_disable_pci_offset_99(const struct rtl8125_private * const tp)
{
        rtl8125_clear_mac_ocp_bit(tp, 0xE032, (BIT_0 | BIT_1));
        rtl8125_csi_fun0_write_byte(tp, 0x99, 0x00);
}

static void
rtl8125_enable_pci_offset_99(const struct rtl8125_private * const tp)
{
        u32 csi_tmp;
        u8 offset_99 = tp->org_pci_offset_99;

        rtl8125_csi_fun0_write_byte(tp, 0x99, offset_99);
        csi_tmp = rtl8125_mac_ocp_read(tp, 0xE032) & ~(BIT_0 | BIT_1);
        if (offset_99 & (BIT_5 | BIT_6))
                csi_tmp |= BIT_1;
        if (offset_99 & BIT_2)
                csi_tmp |= BIT_0;
        rtl8125_mac_ocp_write(tp, 0xE032, csi_tmp);
}

static void
rtl8125_init_pci_offset_99(const struct rtl8125_private * const tp)
{
        rtl8125_mac_ocp_write(tp, 0xCDD0, 0x9003);
        rtl8125_set_mac_ocp_bit(tp, 0xE034, BIT_15 | BIT_14);
        rtl8125_mac_ocp_write(tp, 0xCDD2, 0x889C);
        rtl8125_mac_ocp_write(tp, 0xCDD8, 0x9003);
        rtl8125_mac_ocp_write(tp, 0xCDD4, 0x8C30);
        rtl8125_mac_ocp_write(tp, 0xCDDA, 0x9003);
        rtl8125_mac_ocp_write(tp, 0xCDD6, 0x9003);
        rtl8125_mac_ocp_write(tp, 0xCDDC, 0x9003);
        rtl8125_mac_ocp_write(tp, 0xCDE8, 0x883E);
        rtl8125_mac_ocp_write(tp, 0xCDEA, 0x9003);
        rtl8125_mac_ocp_write(tp, 0xCDEC, 0x889C);
        rtl8125_mac_ocp_write(tp, 0xCDEE, 0x9003);
        rtl8125_mac_ocp_write(tp, 0xCDF0, 0x8C09);
        rtl8125_mac_ocp_write(tp, 0xCDF2, 0x9003);
        rtl8125_set_mac_ocp_bit(tp, 0xE032, BIT_14);
        rtl8125_set_mac_ocp_bit(tp, 0xE0A2, BIT_0);
        rtl8125_enable_pci_offset_99(tp);
}

static void
rtl8125_disable_pci_offset_180(const struct rtl8125_private * const tp)
{
        rtl8125_mac_ocp_write(tp, 0xE092, rtl8125_mac_ocp_read(tp, 0xE092) & 0xFF00);
}

static void
rtl8125_enable_pci_offset_180(const struct rtl8125_private * const tp)
{
        rtl8125_clear_mac_ocp_bit(tp, 0xE094, 0xFF00); // ~0x00FF
        rtl8125_clear_set_mac_ocp_bit(tp, 0xE092, 0x00FF, BIT_2); // ~0xFF00
}

static void
rtl8125_init_pci_offset_180(const struct rtl8125_private * const tp)
{
        rtl8125_enable_pci_offset_180(tp);
}

static void
rtl8125_set_pci_99_180_exit_driver_para(const struct rtl8125_private * const tp)
{
        if (tp->org_pci_offset_99 & BIT_2)
                rtl8125_issue_offset_99_event(tp);
        rtl8125_disable_pci_offset_99(tp);
        rtl8125_disable_pci_offset_180(tp);
}

static void
rtl8125_unlock_config_regs(const struct rtl8125_private *tp)
{
        RTL_W8(tp, Cfg9346, RTL_R8(tp, Cfg9346) | Cfg9346_Unlock);
}

static inline void
rtl8125_lock_config_regs(const struct rtl8125_private *tp)
{
        RTL_W8(tp, Cfg9346, RTL_R8(tp, Cfg9346) & ~Cfg9346_Unlock);
}

static void
rtl8125_disable_exit_l1_mask(const struct rtl8125_private *tp)
{
        //(1)ERI(0xD4)(OCP 0xC0AC).bit[7:12]=6'b000000, L1 Mask
        rtl8125_clear_mac_ocp_bit(tp, 0xC0AC, (BIT_7 | BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12));
}

static void
rtl8125_enable_extend_tally_couter(const struct rtl8125_private *tp)
{
        rtl8125_set_mac_ocp_bit(tp, 0xEA84, (BIT_1 | BIT_0));
}

static void
rtl8125_disable_extend_tally_couter(const struct rtl8125_private *tp)
{
        rtl8125_clear_mac_ocp_bit(tp, 0xEA84, (BIT_1 | BIT_0));
}

static void
rtl8125_enable_force_clkreq(const struct rtl8125_private *tp, const bool enable)
{
        if (enable)
                RTL_W8(tp, 0xF1, RTL_R8(tp, 0xF1) | BIT_7);
        else
                RTL_W8(tp, 0xF1, RTL_R8(tp, 0xF1) & ~BIT_7);
}

static void
rtl8125_enable_aspm_clkreq_lock(const struct rtl8125_private *tp, const bool enable)
{
        rtl8125_unlock_config_regs(tp);
        if (enable) {
                RTL_W8(tp, Config2, RTL_R8(tp, Config2) | BIT_7);
                RTL_W8(tp, Config5, RTL_R8(tp, Config5) | BIT_0);
        } else {
                RTL_W8(tp, Config2, RTL_R8(tp, Config2) & ~BIT_7);
                RTL_W8(tp, Config5, RTL_R8(tp, Config5) & ~BIT_0);
        }
        rtl8125_lock_config_regs(tp);
}

static void
rtl8125_hw_d3_para(struct rtl8125_private *tp)
{
        RTL_W16(tp, RxMaxSize, RX_BUF_SIZE + 1);

        rtl8125_enable_force_clkreq(tp, 0);
        rtl8125_enable_aspm_clkreq_lock(tp, 0);
        rtl8125_disable_exit_l1_mask(tp);

#ifdef ENABLE_REALWOW_SUPPORT
        rtl8125_set_realwow_d3_para(tp->dev);
#endif
        rtl8125_set_pci_99_180_exit_driver_para(tp);

        /*disable ocp phy power saving*/
        //if (rtl8125_flag_is_set(tp, IsMcfg236))
        if (is_8125A(tp))
                rtl8125_disable_ocp_phy_power_saving(tp);

        rtl8125_disable_rxdvgate(tp);
        rtl8125_disable_extend_tally_couter(tp);
}

static void
rtl8125_set_magic_packet(const struct rtl8125_private *tp, const bool enable)
{
        //if (tp->mcfg == CFG_METHOD_8 || tp->mcfg == CFG_METHOD_9)
        if (! rtl8125_flag_is_set(tp, WAKEUP_MAGIC_PACKET_V3))
                return;
        if (enable)
                rtl8125_set_mac_ocp_bit(tp, 0xC0B6, BIT_0);
        else
                rtl8125_clear_mac_ocp_bit(tp, 0xC0B6, BIT_0);
}

static void
rtl8125_set_linkchg_wakeup(const struct rtl8125_private *tp, const bool enable)
{
        if (enable) {
                RTL_W8(tp, Config3, RTL_R8(tp, Config3) | LinkUp);
                rtl8125_clear_set_mac_ocp_bit(tp, 0xE0C6, (BIT_5 | BIT_3 | BIT_2), (BIT_4 | BIT_1 | BIT_0));
        } else {
                RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~LinkUp);
                rtl8125_clear_mac_ocp_bit(tp, 0xE0C6,  (BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0));
        }
}

#define WAKE_ANY (WAKE_PHY | WAKE_MAGIC | WAKE_UCAST | WAKE_BCAST | WAKE_MCAST)

/* 8125 Hardware WOL at boot is typically g (Magic Packet, 0x20) */
static u32
rtl8125_get_hw_wol(struct rtl8125_private *tp)
{
        u8 options;
        u32 wol_opts = 0;

        /* Power management should always be enabled on chips that support WOL */
        if (!(RTL_R8(tp, Config1) & PMEnable))
                goto out;

        options = RTL_R8(tp, Config3);
        if (options & LinkUp)
                wol_opts |= WAKE_PHY;

        if (rtl8125_flag_is_set(tp, WAKEUP_MAGIC_PACKET_V3)
                && (rtl8125_mac_ocp_read(tp, 0xC0B6) & BIT_0))
                wol_opts |= WAKE_MAGIC;

        options = RTL_R8(tp, Config5);
        if (options & UWF)
                wol_opts |= WAKE_UCAST;
        if (options & BWF)
                wol_opts |= WAKE_BCAST;
        if (options & MWF)
                wol_opts |= WAKE_MCAST;

out:
        return wol_opts;
}

static void
rtl8125_disable_d0_speedup(struct rtl8125_private *tp)
{
        if (likely(HW_SUPP_D0_SPEED_UP(tp)))
                RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) & ~BIT_3);
}

static void
rtl8125_set_hw_wol(struct net_device *dev, u32 wol_opts)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        static struct {
                u32 opt;
                u16 reg;
                u8  mask;
        } cfg[] = {
                { WAKE_PHY,   Config3, LinkUp },
                { WAKE_UCAST, Config5, UWF },
                { WAKE_BCAST, Config5, BWF },
                { WAKE_MCAST, Config5, MWF },
                { WAKE_ANY,   Config5, LanWake },
                { WAKE_MAGIC, Config3, MagicPacket },
        };
        unsigned int i, cfgSize = ARRAY_SIZE(cfg) - 1;
        rtl8125_set_magic_packet(tp, wol_opts & WAKE_MAGIC); // enable/disable magic packet

        rtl8125_unlock_config_regs(tp);

        for (i = 0; i < cfgSize; i++) {
                u8 options = RTL_R8(tp, cfg[i].reg) & ~cfg[i].mask;
                if (wol_opts & cfg[i].opt)
                        options |= cfg[i].mask;
                RTL_W8(tp, cfg[i].reg, options);
        }

        RTL_W8(tp, Config2, RTL_R8(tp, Config2) | PMSTS_En); // PME_SIGNAL
        rtl8125_set_linkchg_wakeup(tp, wol_opts & WAKE_PHY);
        printk(KERN_INFO "r8125: Wake On Lan options:0x%x\n", wol_opts);
        rtl8125_lock_config_regs(tp);
}

static void
rtl8125_enable_d0_speedup(struct rtl8125_private *tp)
{
        u16 clearmask;
        u16 setmask;

        if (unlikely(!HW_SUPP_D0_SPEED_UP(tp)))
                return;

        if (tp->D0SpeedUpSpeed == D0_SPEED_UP_SPEED_DISABLE)
                return;

        clearmask = (BIT_10 | BIT_9 | BIT_8 | BIT_7);
        if (tp->D0SpeedUpSpeed == D0_SPEED_UP_SPEED_2500)
                setmask = BIT_7;
        else
                setmask = 0;
        rtl8125_clear_set_mac_ocp_bit(tp, 0xE10A, clearmask, setmask);

        //speed up flowcontrol
        clearmask = (BIT_15 | BIT_14);
        //if (tp->HwSuppD0SpeedUpVer == 2)
        if (is_8125D(tp)) // HW_SUPPORT_D0_SPEED_UP version 2
                clearmask |= BIT_13;

        if (tp->fcpause == rtl8125_fc_full) {
                setmask = (BIT_15 | BIT_14);
                //if (tp->HwSuppD0SpeedUpVer == 2)
                if (is_8125D(tp)) // HW_SUPPORT_D0_SPEED_UP version 2
                        setmask |= BIT_13;
        } else
                setmask = 0;
        rtl8125_clear_set_mac_ocp_bit(tp, 0xE860, clearmask, setmask);
        RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) | BIT_3);
}

static void
rtl8125_phy_restart_nway(struct rtl8125_private *tp)
{
        if (rtl8125_is_in_phy_disable_mode(tp))
                return;
        rtl8125_mdio_set_page(tp, 0x0000);
        rtl8125_mdio_write(tp, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);
}

static void
rtl8125_phy_setup_force_mode(struct rtl8125_private *tp, u32 speed, u8 duplex)
{
        u16 bmcr_true_force = 0;

        if (rtl8125_is_in_phy_disable_mode(tp))
                return;

        if ((speed == SPEED_10) && (duplex == DUPLEX_HALF)) {
                bmcr_true_force = BMCR_SPEED10;
        } else if ((speed == SPEED_10) && (duplex == DUPLEX_FULL)) {
                bmcr_true_force = BMCR_SPEED10 | BMCR_FULLDPLX;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_HALF)) {
                bmcr_true_force = BMCR_SPEED100;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_FULL)) {
                bmcr_true_force = BMCR_SPEED100 | BMCR_FULLDPLX;
        } else {
                netif_err(tp, drv, tp->dev, "Failed to set phy force mode!\n");
                return;
        }

        rtl8125_mdio_set_page(tp, 0x0000);
        rtl8125_mdio_write(tp, MII_BMCR, bmcr_true_force);
}

static void
rtl8125_set_pci_pme(struct rtl8125_private *tp, int set)
{
        struct pci_dev *pdev = tp->pci_dev;
        u16 pmc;

        if (!pdev->pm_cap)
                return;

        pci_read_config_word(pdev, pdev->pm_cap + PCI_PM_CTRL, &pmc);
        pmc |= PCI_PM_CTRL_PME_STATUS;
        if (set)
                pmc |= PCI_PM_CTRL_PME_ENABLE;
        else
                pmc &= ~PCI_PM_CTRL_PME_ENABLE;
        pci_write_config_word(pdev, pdev->pm_cap + PCI_PM_CTRL, pmc);
}

static int
rtl8125_set_wol_link_speed(struct net_device *dev)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        int giga_ctrl, ctrl_2500, auto_nego = 0;
        u64 adv;
        u16 anlpar, gbsr, aner, status_2500;
        unsigned long flags;

        spin_lock_irqsave(&tp->phy_lock, flags);
        if (!rtl8125_flag_is_set(tp, AutoNegMode))
                goto exit;

        rtl8125_mdio_set_page(tp, 0x0000);

        auto_nego = rtl8125_mdio_read(tp, MII_ADVERTISE);
        auto_nego &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL
                       | ADVERTISE_100HALF | ADVERTISE_100FULL);

        giga_ctrl = rtl8125_mdio_read(tp, MII_CTRL1000);
        giga_ctrl &= ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);

        ctrl_2500 = rtl8125_mdio_direct_read_phy_ocp(tp, 0xA5D4);
        ctrl_2500 &= ~RTK_ADVERTISE_2500FULL;

        aner = tp->phy_reg_aner;
        anlpar = tp->phy_reg_anlpar;
        gbsr = tp->phy_reg_gbsr;
        status_2500 = tp->phy_reg_status_2500;
        if (rtl8125_xmii_link_ok(tp)) {
                aner = rtl8125_mdio_read(tp, MII_EXPANSION);
                anlpar = rtl8125_mdio_read(tp, MII_LPA);
                gbsr = rtl8125_mdio_read(tp, MII_STAT1000);
                status_2500 = rtl8125_mdio_direct_read_phy_ocp(tp, 0xA5D6);
        }

        adv = tp->advertising;
        if ((aner | anlpar | gbsr | status_2500) == 0) {
                int auto_nego_tmp = 0;
                if (adv & ADVERTISED_10baseT_Half)
                        auto_nego_tmp |= ADVERTISE_10HALF;
                if (adv & ADVERTISED_10baseT_Full)
                        auto_nego_tmp |= ADVERTISE_10FULL;
                if (adv & ADVERTISED_100baseT_Half)
                        auto_nego_tmp |= ADVERTISE_100HALF;
                if (adv & ADVERTISED_100baseT_Full)
                        auto_nego_tmp |= ADVERTISE_100FULL;

                if (auto_nego_tmp == 0)
                        goto exit;

                auto_nego |= auto_nego_tmp;
                goto skip_check_lpa;
        }
        if (!(aner & EXPANSION_NWAY))
                goto exit;

        if ((adv & ADVERTISED_10baseT_Half) && (anlpar & LPA_10HALF))
                auto_nego |= ADVERTISE_10HALF;
        else if ((adv & ADVERTISED_10baseT_Full) && (anlpar & LPA_10FULL))
                auto_nego |= ADVERTISE_10FULL;
        else if ((adv & ADVERTISED_100baseT_Half) && (anlpar & LPA_100HALF))
                auto_nego |= ADVERTISE_100HALF;
        else if ((adv & ADVERTISED_100baseT_Full) && (anlpar & LPA_100FULL))
                auto_nego |= ADVERTISE_100FULL;
        else if (adv & ADVERTISED_1000baseT_Half && (gbsr & LPA_1000HALF))
                giga_ctrl |= ADVERTISE_1000HALF;
        else if (adv & ADVERTISED_1000baseT_Full && (gbsr & LPA_1000FULL))
                giga_ctrl |= ADVERTISE_1000FULL;
        else if (adv & ADVERTISED_2500baseX_Full && (status_2500 & RTK_LPA_ADVERTISE_2500FULL))
                ctrl_2500 |= RTK_ADVERTISE_2500FULL;
        else
                goto exit;

skip_check_lpa:
#ifdef CONFIG_DOWN_SPEED_100
        auto_nego |= (ADVERTISE_100FULL | ADVERTISE_100HALF | ADVERTISE_10HALF | ADVERTISE_10FULL);
#endif

        rtl8125_mdio_write(tp, MII_ADVERTISE, auto_nego);
        rtl8125_mdio_write(tp, MII_CTRL1000, giga_ctrl);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA5D4, ctrl_2500);
        rtl8125_phy_restart_nway(tp);
exit:
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        return auto_nego;
}

static bool
rtl8125_keep_wol_link_speed(struct net_device *dev, u8 from_suspend)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        return (from_suspend && rtl8125_xmii_link_ok(tp) && (tp->wol_opts & WAKE_PHY)) ? 1 : 0;
}

static void
rtl8125_powerdown_pll(struct net_device *dev, u8 from_suspend)
{
        struct rtl8125_private *tp = netdev_priv(dev);

        rtl8125_clear_flag(tp, CheckKeepLinkSpeed);
        if (tp->wol_enabled == WOL_ENABLED) {
                int auto_nego;

                rtl8125_set_hw_wol(dev, tp->wol_opts);

                /* Enable the PME and clear the status */
                rtl8125_set_pci_pme(tp, 1);

                if (rtl8125_keep_wol_link_speed(dev, from_suspend))
                        rtl8125_set_flag(tp, CheckKeepLinkSpeed);
                else {
                        if (tp->D0SpeedUpSpeed != D0_SPEED_UP_SPEED_DISABLE) {
                                rtl8125_enable_d0_speedup(tp);
                                rtl8125_set_flag(tp, CheckKeepLinkSpeed);;
                        }

                        auto_nego = rtl8125_set_wol_link_speed(dev);

                        if (is_8125D(tp))   // PFM patch is required;
                                rtl8125_set_pfm_patch(tp,
                                        (auto_nego & (ADVERTISE_10HALF | ADVERTISE_10FULL)) ? 1 : 0);
                }

                RTL_W32(tp, RxConfig, RTL_R32(tp, RxConfig) | AcceptBroadcast | AcceptMulticast | AcceptMyPhys);

                return;
        }

        rtl8125_phy_power_down(tp);
        RTL_W8(tp, PMCH, RTL_R8(tp, PMCH) & ~D3COLD_NO_PLL_DOWN); // power down d3 pll
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,11,0)
        dev->ethtool->wol_enabled = tp->wol_opts ? 1 : 0;
#else
        dev->wol_enabled = tp->wol_opts ? 1 : 0;
#endif
        RTL_W8(tp, PPSW, RTL_R8(tp, PPSW) & ~PFM_D3COLD_EN);
}

static void rtl8125_powerup_pll(struct rtl8125_private *tp)
{
        RTL_W8(tp, PMCH, RTL_R8(tp, PMCH) | D3_NO_PLL_DOWN); // power up d3 pll
        if (rtl8125_flag_is_set(tp, ResumeNoSpeedChange))
                return;
        rtl8125_phy_power_up(tp);
}

static void
rtl8125_get_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
        struct rtl8125_private *tp = netdev_priv(dev);

        wol->wolopts = 0;
        wol->supported = WAKE_ANY;

        if (!(RTL_R8(tp, Config1) & PMEnable))
                return;

        wol->wolopts = tp->wol_opts;
}

static int
rtl8125_set_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
        struct rtl8125_private *tp = netdev_priv(dev);

        tp->wol_opts = wol->wolopts;
        tp->wol_enabled = (tp->wol_opts) ? WOL_ENABLED : WOL_DISABLED;
        device_set_wakeup_enable(tp->device, wol->wolopts);
        return 0;
}

static void
rtl8125_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        struct rtl8125_fw *rtl_fw = tp->rtl_fw;

        strscpy(info->driver, MODULENAME, sizeof(info->driver));
        strscpy(info->version, RTL8125_VERSION, sizeof(info->version));
        strscpy(info->bus_info, pci_name(tp->pci_dev), sizeof(info->bus_info));
        info->regdump_len = R8125_REGS_DUMP_SIZE;
        info->eedump_len = tp->flags & (EEPROM_TYPE_93C46 | EEPROM_TYPE_93C56);
        BUILD_BUG_ON(sizeof(info->fw_version) < sizeof(rtl_fw->version));
        if (rtl_fw)
                strscpy(info->fw_version, rtl_fw->version,
                        sizeof(info->fw_version));
}

static int
rtl8125_get_regs_len(struct net_device *dev)
{
        return R8125_REGS_DUMP_SIZE;
}

static void
rtl8125_set_d0_speedup_speed(struct rtl8125_private *tp)
{
        if (likely(HW_SUPP_D0_SPEED_UP(tp))) {
                tp->D0SpeedUpSpeed = D0_SPEED_UP_SPEED_DISABLE;
                if (rtl8125_flag_is_set(tp, AutoNegMode)) {
                        if (tp->speed == SPEED_2500)
                                tp->D0SpeedUpSpeed = D0_SPEED_UP_SPEED_2500;
                        else if (tp->speed == SPEED_1000)
                                tp->D0SpeedUpSpeed = D0_SPEED_UP_SPEED_1000;
                }
        }
}

static int
rtl8125_set_speed(struct rtl8125_private *tp, u8 autoneg, u32 speed, u8 duplex, u64 adv)
{
        int auto_nego = 0, giga_ctrl = 0, ctrl_2500 = 0;
        unsigned long flags;

        if (rtl8125_flag_is_set(tp, ResumeNoSpeedChange))
                return 0;

        spin_lock_irqsave(&tp->phy_lock, flags);
        //Disable Giga Lite
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA428, BIT_9);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA5EA, BIT_0);

        if (!rtl8125_is_speed_mode_valid(speed)) {
                speed = SPEED_2500;
                duplex = DUPLEX_FULL;
                adv |= tp->advertising;
        }

        giga_ctrl = rtl8125_mdio_read(tp, MII_CTRL1000);
        giga_ctrl &= ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);
        ctrl_2500 = rtl8125_mdio_direct_read_phy_ocp(tp, 0xA5D4);
        ctrl_2500 &= ~RTK_ADVERTISE_2500FULL;

        if (autoneg == AUTONEG_ENABLE) {
                /*n-way force*/
                auto_nego = rtl8125_mdio_read(tp, MII_ADVERTISE);
                auto_nego &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL |
                               ADVERTISE_100HALF | ADVERTISE_100FULL |
                               ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);

                if (adv & ADVERTISED_10baseT_Half)
                        auto_nego |= ADVERTISE_10HALF;
                if (adv & ADVERTISED_10baseT_Full)
                        auto_nego |= ADVERTISE_10FULL;
                if (adv & ADVERTISED_100baseT_Half)
                        auto_nego |= ADVERTISE_100HALF;
                if (adv & ADVERTISED_100baseT_Full)
                        auto_nego |= ADVERTISE_100FULL;
                if (adv & ADVERTISED_1000baseT_Half)
                        giga_ctrl |= ADVERTISE_1000HALF;
                if (adv & ADVERTISED_1000baseT_Full)
                        giga_ctrl |= ADVERTISE_1000FULL;
                if (adv & ADVERTISED_2500baseX_Full)
                        ctrl_2500 |= RTK_ADVERTISE_2500FULL;

                //flow control
                if (tp->fcpause == rtl8125_fc_full)
                        auto_nego |= ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM;

                tp->phy_auto_nego_reg = auto_nego;
                tp->phy_1000_ctrl_reg = giga_ctrl;

                tp->phy_2500_ctrl_reg = ctrl_2500;

                rtl8125_mdio_set_page(tp, 0x0000);
                rtl8125_mdio_write(tp, MII_ADVERTISE, auto_nego);
                rtl8125_mdio_write(tp, MII_CTRL1000, giga_ctrl);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA5D4, ctrl_2500);
                rtl8125_phy_restart_nway(tp);
                rtl8125_set_flag(tp, AutoNegMode);
                mdelay(20);
        } else {
                /*true force*/
                rtl8125_clear_flag(tp, AutoNegMode);
                if (speed == SPEED_10 || speed == SPEED_100)
                        rtl8125_phy_setup_force_mode(tp, speed, duplex);
                else {
                        spin_unlock_irqrestore(&tp->phy_lock, flags);
                        return -EINVAL;
                }
        }

        //tp->autoneg = autoneg;
        tp->speed = speed;
        tp->duplex = duplex;
        tp->advertising = adv;

        rtl8125_set_d0_speedup_speed(tp);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        return 0;
}

/*
static inline void rtl8125_set_speed(struct rtl8125_private *tp, u8 autoneg,
                  u32 speed, u8 duplex, u64 adv)
{
        if (!rtl8125_flag_is_set(tp, ResumeNoSpeedChange))
                rtl8125_set_speed_xmii(tp, autoneg, speed, duplex, adv);

        //ret = tp->set_speed(tp, autoneg, speed, duplex, adv);
}
*/

static int
rtl8125_set_settings(struct net_device *dev,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0)
                     struct ethtool_cmd *cmd)
{
        u8 autoneg = cmd->autoneg;
        u32 speed = cmd->speed;
        u8 duplex = cmd->duplex;
        u64 supported = cmd->supported;
        u64 advertising = cmd->advertising;
#else
                     const struct ethtool_link_ksettings *cmd)
{
        const struct ethtool_link_settings *base = &cmd->base;
        u8 autoneg = base->autoneg;
        u32 speed = base->speed;
        u8 duplex = base->duplex;
        u64 supported = 0, advertising = 0;
        ethtool_convert_link_mode_to_legacy_u32((u32*)&supported,
                                                cmd->link_modes.supported);
        ethtool_convert_link_mode_to_legacy_u32((u32*)&advertising,
                                                cmd->link_modes.advertising);
        if (test_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
                     cmd->link_modes.supported))
                supported |= ADVERTISED_2500baseX_Full;
        if (test_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
                     cmd->link_modes.advertising))
                advertising |= ADVERTISED_2500baseX_Full;
#endif
        if (advertising & ~supported)
                return -EINVAL;

        // Returns 0 (= success) or a negative value (= error)
        return rtl8125_set_speed(netdev_priv(dev), autoneg, speed, duplex, advertising);
}

static __always_inline u32
rtl8125_rx_desc_opts1(struct rtl8125_private *tp, struct RxDesc *desc)
{
#if defined(ENABLE_RSS_SUPPORT) || defined(ENABLE_PTP_SUPPORT)
        if (tp->RxDescType == RX_DESC_RING_TYPE_3)
                return READ_ONCE(((struct RxDescV3 *)desc)->RxDescNormalDDWord4.opts1);
#endif
#ifdef ENABLE_RSS_SUPPORT
        if (likely(tp->RxDescType == RX_DESC_RING_TYPE_4))
                return READ_ONCE(((struct RxDescV4 *)desc)->RxDescNormalDDWord2.opts1);
#endif
        return READ_ONCE(desc->opts1);
}

/*
static __always_inline void
rtl8125_set_rx_desc_opts1(struct rtl8125_private *tp, struct RxDesc *desc, u32 opts)
{
        //dma_wmb();
#if defined(ENABLE_RSS_SUPPORT) || defined(ENABLE_PTP_SUPPORT)
        if (tp->RxDescType == RX_DESC_RING_TYPE_3)
                ((struct RxDescV3 *)desc)->RxDescNormalDDWord4.opts1 = opts;
        else
#endif
#ifdef ENABLE_RSS_SUPPORT
        if (likely(tp->RxDescType == RX_DESC_RING_TYPE_4))
                ((struct RxDescV4 *)desc)->RxDescNormalDDWord2.opts1 = opts;
        else
#endif
        desc->opts1 = opts;
}
*/

static __always_inline u32
rtl8125_rx_desc_opts2(struct rtl8125_private *tp, struct RxDesc *desc)
{
#if defined(ENABLE_RSS_SUPPORT) || defined(ENABLE_PTP_SUPPORT)
        if (tp->RxDescType == RX_DESC_RING_TYPE_3)
                return ((struct RxDescV3 *)desc)->RxDescNormalDDWord4.opts2;
#endif
#ifdef ENABLE_RSS_SUPPORT
        if (likely(tp->RxDescType == RX_DESC_RING_TYPE_4))
                return ((struct RxDescV4 *)desc)->RxDescNormalDDWord2.opts2;
#endif
        return desc->opts2;
}

static __always_inline void
rtl8125_set_rx_desc_opts2(struct rtl8125_private *tp, struct RxDesc *desc, u32 opts)
{
#if defined(ENABLE_RSS_SUPPORT) || defined(ENABLE_PTP_SUPPORT)
        if (tp->RxDescType == RX_DESC_RING_TYPE_3)
               ((struct RxDescV3 *)desc)->RxDescNormalDDWord4.opts2 = opts;
        else
#endif
#ifdef ENABLE_RSS_SUPPORT
        if (likely(tp->RxDescType == RX_DESC_RING_TYPE_4))
                ((struct RxDescV4 *)desc)->RxDescNormalDDWord2.opts2 = opts;
        else
#endif
        desc->opts2 = opts;
}

#define rtl8125_clear_rx_desc_opts2(tp,desc) rtl8125_set_rx_desc_opts2(tp,desc,0)

static __always_inline u64
rtl8125_rx_desc_addr(struct rtl8125_private *tp, struct RxDesc *desc)
{
#if defined(ENABLE_RSS_SUPPORT) || defined(ENABLE_PTP_SUPPORT)
        if (tp->RxDescType == RX_DESC_RING_TYPE_3)
                return le64_to_cpu(((struct RxDescV3 *)desc)->addr);
#endif
#ifdef ENABLE_RSS_SUPPORT
        if (likely(tp->RxDescType == RX_DESC_RING_TYPE_4))
                return le64_to_cpu(((struct RxDescV4 *)desc)->addr);
#endif
        return le64_to_cpu(desc->addr);
}

#ifdef CONFIG_R8125_VLAN

static __always_inline u32
rtl8125_tx_vlan_tag(struct rtl8125_private *tp, struct sk_buff *skb)
{
        return (skb_vlan_tag_present(skb)) ?
               TxVlanTag | swab16(skb_vlan_tag_get(skb)) : 0x00;
}

static __always_inline void
rtl8125_rx_vlan_skb(struct rtl8125_private *tp, struct RxDesc *desc, struct sk_buff *skb)
{
        u32 opts2 = le32_to_cpu(rtl8125_rx_desc_opts2(tp, desc));
        if (opts2 & RxVlanTag)
                __vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), swab16(opts2 & 0xffff));

        rtl8125_clear_rx_desc_opts2(tp, desc);
}

#else /* !CONFIG_R8125_VLAN */

static __always_inline u32
rtl8125_tx_vlan_tag(struct rtl8125_private *tp, struct sk_buff *skb)
{
        return 0;
}

static void
rtl8125_rx_vlan_skb(struct rtl8125_private *tp, struct RxDesc *desc, struct sk_buff *skb)
{
}

#endif

static netdev_features_t
rtl8125_fix_features(struct net_device *dev, netdev_features_t features)
{
        if (dev->mtu > MSS_MAX)
                features &= ~NETIF_F_ALL_TSO;

        /*
        if (dev->mtu > ETH_DATA_LEN)
                features &= ~(NETIF_F_ALL_TSO | NETIF_F_CSUM_MASK);

#ifndef CONFIG_R8125_VLAN
        features &= ~NETIF_F_CSUM_MASK;
#endif
        */
        return features;
}

static void
rtl8125_hw_set_features(struct rtl8125_private *tp, netdev_features_t features)
{
        u32 rx_config = RTL_R32(tp, RxConfig);
        if (features & NETIF_F_RXALL) {
                tp->rx_config |= (AcceptErr | AcceptRunt);
                rx_config |= (AcceptErr | AcceptRunt);
        } else {
                tp->rx_config &= ~(AcceptErr | AcceptRunt);
                rx_config &= ~(AcceptErr | AcceptRunt);
        }

        if (features & NETIF_F_HW_VLAN_CTAG_RX) {
                tp->rx_config |= (EnableInnerVlan | EnableOuterVlan);
                rx_config |= (EnableInnerVlan | EnableOuterVlan);
        } else {
                tp->rx_config &= ~(EnableInnerVlan | EnableOuterVlan);
                rx_config &= ~(EnableInnerVlan | EnableOuterVlan);
        }

        RTL_W32(tp, RxConfig, rx_config);

        if (features & NETIF_F_RXCSUM)
                tp->cp_cmd |= RxChkSum;
        else
                tp->cp_cmd &= ~RxChkSum;

        RTL_W16(tp, CPlusCmd, tp->cp_cmd);
              /* Read an arbitrary register to commit a preceding PCI write */
        rtl_pci_commit(tp);
}

static int
rtl8125_set_features(struct net_device *dev, netdev_features_t features)
{
        features &= NETIF_F_RXALL | NETIF_F_RXCSUM | NETIF_F_HW_VLAN_CTAG_RX;
        rtl8125_hw_set_features(netdev_priv(dev), features);
        return 0;
}

static inline u8 rtl8125_get_mdi_status(struct rtl8125_private *tp)
{
        if (unlikely(!rtl8125_xmii_link_ok(tp)))
                return ETH_TP_MDI_INVALID;

        if (rtl8125_mdio_direct_read_phy_ocp(tp, 0xA444) & BIT_1)
                return ETH_TP_MDI;
        else
                return ETH_TP_MDI_X;
}

static inline void rtl8125_gset_xmii(struct net_device *dev,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0)
        struct ethtool_cmd *cmd)
#else
        struct ethtool_link_ksettings *cmd)
#endif
{
        struct rtl8125_private *tp = netdev_priv(dev);
        unsigned long flags;
        u16 aner = tp->phy_reg_aner;
        u16 anlpar = tp->phy_reg_anlpar;
        u16 gbsr = tp->phy_reg_gbsr;
        u16 status_2500 = tp->phy_reg_status_2500;
        u64 supported, advertising, lpa_adv = 0;
        u16 status, bmcr;
        u8 autoneg, duplex, report_lpa = 0;
        u32 speed = 0;

        supported = SUPPORTED_10baseT_Half | SUPPORTED_10baseT_Full | SUPPORTED_100baseT_Half |
                    SUPPORTED_100baseT_Full | SUPPORTED_1000baseT_Full | SUPPORTED_2500baseX_Full |
                    SUPPORTED_Autoneg | SUPPORTED_TP | SUPPORTED_Pause | SUPPORTED_Asym_Pause;

        if (!HW_SUPP_PHY_LINK_SPEED_2500M(tp))
                supported &= ~SUPPORTED_2500baseX_Full;

        advertising = tp->advertising;
        if (tp->phy_auto_nego_reg || tp->phy_1000_ctrl_reg ||
            tp->phy_2500_ctrl_reg) {
                advertising = 0;
                if (tp->phy_auto_nego_reg & ADVERTISE_10HALF)
                        advertising |= ADVERTISED_10baseT_Half;
                if (tp->phy_auto_nego_reg & ADVERTISE_10FULL)
                        advertising |= ADVERTISED_10baseT_Full;
                if (tp->phy_auto_nego_reg & ADVERTISE_100HALF)
                        advertising |= ADVERTISED_100baseT_Half;
                if (tp->phy_auto_nego_reg & ADVERTISE_100FULL)
                        advertising |= ADVERTISED_100baseT_Full;
                if (tp->phy_1000_ctrl_reg & ADVERTISE_1000FULL)
                        advertising |= ADVERTISED_1000baseT_Full;
                if (tp->phy_2500_ctrl_reg & RTK_ADVERTISE_2500FULL)
                        advertising |= ADVERTISED_2500baseX_Full;
        }

        spin_lock_irqsave(&tp->phy_lock, flags);
        rtl8125_mdio_set_page(tp, 0x0000);
        bmcr = rtl8125_mdio_read(tp, MII_BMCR);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        if (bmcr & BMCR_ANENABLE) {
                autoneg = AUTONEG_ENABLE;
                advertising |= ADVERTISED_Autoneg;
        } else {
                autoneg = AUTONEG_DISABLE;
        }

        advertising |= ADVERTISED_TP;

        status = RTL_R16(tp, PHYstatus);
        if (netif_running(dev) && (status & LinkStatus))
                report_lpa = 1;

        if (report_lpa) {
                /*link on*/
                speed = rtl8125_convert_link_speed(status);

                if (status & TxFlowCtrl)
                        advertising |= ADVERTISED_Asym_Pause;

                if (status & RxFlowCtrl)
                        advertising |= ADVERTISED_Pause;

                duplex = ((status & (_1000bpsF | _2500bpsF)) ||
                          (status & FullDup)) ?
                         DUPLEX_FULL : DUPLEX_HALF;

                /*link partner*/
                if (aner & EXPANSION_NWAY)
                        lpa_adv |= ADVERTISED_Autoneg;
                if (anlpar & LPA_10HALF)
                        lpa_adv |= ADVERTISED_10baseT_Half;
                if (anlpar & LPA_10FULL)
                        lpa_adv |= ADVERTISED_10baseT_Full;
                if (anlpar & LPA_100HALF)
                        lpa_adv |= ADVERTISED_100baseT_Half;
                if (anlpar & LPA_100FULL)
                        lpa_adv |= ADVERTISED_100baseT_Full;
                if (anlpar & LPA_PAUSE_CAP)
                        lpa_adv |= ADVERTISED_Pause;
                if (anlpar & LPA_PAUSE_ASYM)
                        lpa_adv |= ADVERTISED_Asym_Pause;
                if (gbsr & LPA_1000HALF)
                        lpa_adv |= ADVERTISED_1000baseT_Half;
                if (gbsr & LPA_1000FULL)
                        lpa_adv |= ADVERTISED_1000baseT_Full;
                if (status_2500 & RTK_LPA_ADVERTISE_2500FULL)
                        lpa_adv |= ADVERTISED_2500baseX_Full;
        } else {
                /*link down*/
                speed = SPEED_UNKNOWN;
                duplex = DUPLEX_UNKNOWN;
                lpa_adv = 0;
        }

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0)
        cmd->supported = (u32)supported;
        cmd->advertising = (u32)advertising;
        cmd->autoneg = autoneg;
        cmd->speed = speed;
        cmd->duplex = duplex;
        cmd->port = PORT_TP;
        cmd->lp_advertising = (u32)lpa_adv;
        cmd->eth_tp_mdix = rtl8125_get_mdi_status(tp);
#else
        ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.supported,
                                                supported);
        ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.advertising,
                                                advertising);
        ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.lp_advertising,
                                                lpa_adv);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,0,0)
        if (supported & SUPPORTED_2500baseX_Full) {
                linkmode_mod_bit(ETHTOOL_LINK_MODE_2500baseX_Full_BIT,
                                 cmd->link_modes.supported, 0);
                linkmode_mod_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
                                 cmd->link_modes.supported, 1);
        }
        if (advertising & ADVERTISED_2500baseX_Full) {
                linkmode_mod_bit(ETHTOOL_LINK_MODE_2500baseX_Full_BIT,
                                 cmd->link_modes.advertising, 0);
                linkmode_mod_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
                                 cmd->link_modes.advertising, 1);
        }
        if (report_lpa) {
                if (lpa_adv & ADVERTISED_2500baseX_Full) {
                        linkmode_mod_bit(ETHTOOL_LINK_MODE_2500baseX_Full_BIT,
                                         cmd->link_modes.lp_advertising, 0);
                        linkmode_mod_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
                                         cmd->link_modes.lp_advertising, 1);
                }
        }
#endif
        cmd->base.autoneg = autoneg;
        cmd->base.speed = speed;
        cmd->base.duplex = duplex;
        cmd->base.port = PORT_TP;
        cmd->base.eth_tp_mdix = rtl8125_get_mdi_status(tp);
#endif
}

static int
rtl8125_get_settings(struct net_device *dev,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0)
                     struct ethtool_cmd *cmd
#else
                     struct ethtool_link_ksettings *cmd
#endif
                    )
{
        rtl8125_gset_xmii(dev, cmd);
        return 0;
}

static void rtl8125_get_regs(struct net_device *dev, struct ethtool_regs *regs, void *p)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        void __iomem *ioaddr = tp->mmio_addr;
        unsigned int i;
        u8 *data = p;

        if (regs->len < R8125_REGS_DUMP_SIZE)
                return /* -EINVAL */;

        memset(p, 0, regs->len);

        for (i = 0; i < R8125_MAC_REGS_SIZE; i++)
                *data++ = readb(ioaddr + i);
        data = (u8*)p + 256;

        rtl8125_mdio_set_page(tp, 0x0000);
        for (i = 0; i < R8125_PHY_REGS_SIZE/2; i++) {
                *(u16*)data = rtl8125_mdio_read(tp, i);
                data += 2;
        }
        data = (u8*)p + 256 * 2;

        for (i = 0; i < R8125_EPHY_REGS_SIZE/2; i++) {
                *(u16*)data = rtl8125_ephy_read(tp, i);
                data += 2;
        }
        data = (u8*)p + 256 * 3;

}

static void
rtl8125_get_pauseparam(struct net_device *dev, struct ethtool_pauseparam *pause)
{
        struct rtl8125_private *tp = netdev_priv(dev);

        pause->autoneg = rtl8125_flag_to_bool(tp, AutoNegMode);
        if (tp->fcpause == rtl8125_fc_rx_pause)
                pause->rx_pause = 1;
        else if (tp->fcpause == rtl8125_fc_tx_pause)
                pause->tx_pause = 1;
        else if (tp->fcpause == rtl8125_fc_full) {
                pause->rx_pause = 1;
                pause->tx_pause = 1;
        }
}

static int
rtl8125_set_pauseparam(struct net_device *dev, struct ethtool_pauseparam *pause)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        enum rtl8125_fc_mode newfc;

        if (pause->tx_pause || pause->rx_pause)
                newfc = rtl8125_fc_full;
        else
                newfc = rtl8125_fc_none;

        if (tp->fcpause != newfc) {
                tp->fcpause = newfc;
                rtl8125_set_speed(tp, rtl8125_flag_to_bool(tp, AutoNegMode), tp->speed, tp->duplex, tp->advertising);
        }
        return 0;
}

static u32
rtl8125_get_msglevel(struct net_device *dev)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        return (u32)(tp->msg_enable);
}

static void
rtl8125_set_msglevel(struct net_device *dev, u32 value)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        tp->msg_enable = (u8)(value & 0xFF);
}

static const char rtl8125_gstrings[][ETH_GSTRING_LEN] = {
        /* legacy */
        "tx_packets",
        "rx_packets",
        "tx_errors",
        "rx_errors",
        "rx_missed",
        "align_errors",
        "tx_single_collisions",
        "tx_multi_collisions",
        "unicast",
        "broadcast",
        "multicast",
        "tx_aborted",
        "tx_underrun",

        /* extended */
        "tx_octets",
        "rx_octets",
        "rx_multicast64",
        "tx_unicast64",
        "tx_broadcast64",
        "tx_multicast64",
        "tx_pause_on",
        "tx_pause_off",
        "tx_pause_all",
        "tx_deferred",
        "tx_late_collision",
        "tx_all_collision",
        "tx_aborted32",
        "align_errors32",
        "rx_frame_too_long",
        "rx_runt",
        "rx_pause_on",
        "rx_pause_off",
        "rx_pause_all",
        "rx_unknown_opcode",
        "rx_mac_error",
        "tx_underrun32",
        "rx_mac_missed",
        "rx_tcam_dropped",
        "tdu",
        "rdu",
};

static int rtl8125_get_sset_count(struct net_device *dev, int sset)
{
        return (sset == ETH_SS_STATS) ? ARRAY_SIZE(rtl8125_gstrings) : -EOPNOTSUPP;
}

/*
static void
rtl8125_set_ring_size(struct rtl8125_private *tp, u32 rx, u32 tx)
{
        int i;

        for (i = 0; i < R8125_MAX_RX_QUEUES; i++)
                tp->rx_ring[i].num_rx_desc = rx;

        for (i = 0; i < R8125_MAX_TX_QUEUES; i++)
                tp->tx_ring[i].num_tx_desc = tx;
}
*/

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,17,0)
static void rtl8125_get_ringparam(struct net_device *dev,
                                  struct ethtool_ringparam *ring,
                                  struct kernel_ethtool_ringparam *kernel_ring,
                                  struct netlink_ext_ack *extack)
#else
static void rtl8125_get_ringparam(struct net_device *dev,
                                  struct ethtool_ringparam *ring)
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(5,17,0)
{
        //struct rtl8125_private *tp = netdev_priv(dev);

        ring->rx_max_pending = MAX_NUM_TX_DESC;
        ring->tx_max_pending = MAX_NUM_RX_DESC;
        ring->rx_pending = NUM_RX_DESC; //tp->rx_ring[0].num_rx_desc;
        ring->tx_pending = NUM_TX_DESC; //tp->tx_ring[0].num_tx_desc;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,17,0)
static int rtl8125_set_ringparam(struct net_device *dev,
                                 struct ethtool_ringparam *ring,
                                 struct kernel_ethtool_ringparam *kernel_ring,
                                 struct netlink_ext_ack *extack)
#else
static int rtl8125_set_ringparam(struct net_device *dev,
                                 struct ethtool_ringparam *ring)
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(5,17,0)
{
        return 0; // do nothing

/* For performance reasons keep ring_tx/rx_count to maximum size
        struct rtl8125_private *tp = netdev_priv(dev);
        u32 new_rx_count, new_tx_count;
        int rc = 0;

        if ((ring->rx_mini_pending) || (ring->rx_jumbo_pending))
                return -EINVAL;

        new_tx_count = clamp_t(u32, ring->tx_pending,
                               MIN_NUM_TX_DESC, MAX_NUM_TX_DESC);

        new_rx_count = clamp_t(u32, ring->rx_pending,
                               MIN_NUM_RX_DESC, MAX_NUM_RX_DESC);

        if ((new_rx_count == tp->rx_ring[0].num_rx_desc) &&
            (new_tx_count == tp->tx_ring[0].num_tx_desc))
                return 0;

        if (netif_running(dev)) {
                rtl8125_wait_for_quiescence(tp);
                rtl8125_enable_napi(tp);
                rtl8125_close(dev);
        }

        rtl8125_set_ring_size(tp, new_rx_count, new_tx_count);

        if (netif_running(dev))
                rc = rtl8125_open(dev);

        return rc;
*/
}

static void
rtl8125_get_ethtool_stats(struct net_device *dev, struct ethtool_stats *stats, u64 *data)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        struct rtl8125_counters *counters;
        dma_addr_t paddr;

        ASSERT_RTNL();

        counters = tp->tally_vaddr;
        paddr = tp->tally_paddr;
        if (!counters)
                return;

        rtl8125_dump_tally_counter(tp, paddr);

        data[0] = le64_to_cpu(counters->tx_packets);
        data[1] = le64_to_cpu(counters->rx_packets);
        data[2] = le64_to_cpu(counters->tx_errors);
        data[3] = le32_to_cpu(counters->rx_errors);
        data[4] = le16_to_cpu(counters->rx_missed);
        data[5] = le16_to_cpu(counters->align_errors);
        data[6] = le32_to_cpu(counters->tx_one_collision);
        data[7] = le32_to_cpu(counters->tx_multi_collision);
        data[8] = le64_to_cpu(counters->rx_unicast);
        data[9] = le64_to_cpu(counters->rx_broadcast);
        data[10] = le32_to_cpu(counters->rx_multicast);
        data[11] = le16_to_cpu(counters->tx_aborted);
        data[12] = le16_to_cpu(counters->tx_underrun);

        data[13] = le64_to_cpu(counters->tx_octets);
        data[14] = le64_to_cpu(counters->rx_octets);
        data[15] = le64_to_cpu(counters->rx_multicast64);
        data[16] = le64_to_cpu(counters->tx_unicast64);
        data[17] = le64_to_cpu(counters->tx_broadcast64);
        data[18] = le64_to_cpu(counters->tx_multicast64);
        data[19] = le32_to_cpu(counters->tx_pause_on);
        data[20] = le32_to_cpu(counters->tx_pause_off);
        data[21] = le32_to_cpu(counters->tx_pause_all);
        data[22] = le32_to_cpu(counters->tx_deferred);
        data[23] = le32_to_cpu(counters->tx_late_collision);
        data[24] = le32_to_cpu(counters->tx_all_collision);
        data[25] = le32_to_cpu(counters->tx_aborted32);
        data[26] = le32_to_cpu(counters->align_errors32);
        data[27] = le32_to_cpu(counters->rx_frame_too_long);
        data[28] = le32_to_cpu(counters->rx_runt);
        data[29] = le32_to_cpu(counters->rx_pause_on);
        data[30] = le32_to_cpu(counters->rx_pause_off);
        data[31] = le32_to_cpu(counters->rx_pause_all);
        data[32] = le32_to_cpu(counters->rx_unknown_opcode);
        data[33] = le32_to_cpu(counters->rx_mac_error);
        data[34] = le32_to_cpu(counters->tx_underrun32);
        data[35] = le32_to_cpu(counters->rx_mac_missed);
        data[36] = le32_to_cpu(counters->rx_tcam_dropped);
        data[37] = le32_to_cpu(counters->tdu);
        data[38] = le32_to_cpu(counters->rdu);
}

static void
rtl8125_get_strings(struct net_device *dev, u32 stringset, u8 *data)
{
        if (stringset == ETH_SS_STATS)
                memcpy(data, rtl8125_gstrings, sizeof(rtl8125_gstrings));
}

#ifdef ENABLE_R8125_EEPROM
static int rtl_get_eeprom_len(struct net_device *dev)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        return tp->flags & (EEPROM_TYPE_93C46 | EEPROM_TYPE_93C56);
}

static int rtl_get_eeprom(struct net_device *dev, struct ethtool_eeprom *eeprom, u8 *buf)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        int ret, start_w, end_w;
        int VPD_addr = 0xD2, VPD_data = 0xD4;
        u32 *eeprom_buff;
        u16 tmp;
        u8 eeprom_len = rtl_get_eeprom_len(dev); // length of EEPROM

        if (eeprom->len == 0 || (eeprom->offset+eeprom->len) > eeprom_len) {
                netdev_printk(KERN_DEBUG, dev, "Invalid parameter\n");
                return -EINVAL;
        }
        start_w = eeprom->offset >> 2;
        end_w = (eeprom->offset + eeprom->len - 1) >> 2;

        eeprom_buff = kmalloc(sizeof(u32)*(end_w - start_w + 1), GFP_KERNEL);
        if (!eeprom_buff)
                return -ENOMEM;

        rtl8125_unlock_config_regs(tp);
        ret = -EFAULT;
        for (int i = start_w; i <= end_w; i++) {
                pci_write_config_word(tp->pci_dev, VPD_addr, (u16)i*4);
                ret = -EFAULT;
                for (int j = 0; j < 10; j++) {
                        udelay(400);
                        pci_read_config_word(tp->pci_dev, VPD_addr, &tmp);
                        if (tmp & 0x8000) {
                                ret = 0;
                                break;
                        }
                }

                if (ret)
                        break;

                pci_read_config_dword(tp->pci_dev, VPD_data, &eeprom_buff[i-start_w]);
        }
        rtl8125_lock_config_regs(tp);

        if (!ret)
                memcpy(buf, (u8 *)eeprom_buff + (eeprom->offset & 3), eeprom->len);

        kfree(eeprom_buff);

        return ret;
}
#endif

#undef ethtool_op_get_link
#define ethtool_op_get_link _kc_ethtool_op_get_link
static u32 _kc_ethtool_op_get_link(struct net_device *dev)
{
        return netif_carrier_ok(dev) ? 1 : 0;
}

static inline void
rtl8125_set_eee_lpi_timer(struct rtl8125_private *tp)
{
        RTL_W16(tp, EEE_TXIDLE_TIMER_8125, tp->eee.tx_lpi_timer);
}

static inline bool rtl8125_is_adv_eee_enabled(struct rtl8125_private *tp)
{
        return (!is_8125D(tp) && (rtl8125_mdio_direct_read_phy_ocp(tp, 0xA430) & BIT_15));
}

static void _rtl8125_disable_adv_eee(struct rtl8125_private *tp)
{
        bool lock = rtl8125_is_adv_eee_enabled(tp);
        if (lock)
                rtl8125_set_phy_mcu_patch_request(tp);
        rtl8125_clear_mac_ocp_bit(tp, 0xE052, BIT_0);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA442, BIT_12 | BIT_13);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA430, BIT_15);

        if (lock)
                rtl8125_clear_phy_mcu_patch_request(tp);
}

static void rtl8125_disable_adv_eee(struct rtl8125_private *tp)
{
        _rtl8125_disable_adv_eee(tp);
}

static int rtl8125_enable_eee(struct rtl8125_private *tp)
{
        struct ethtool_keee *eee = &tp->eee;
        u16 eee_adv_cap1_t = rtl8125_ethtool_adv_to_mmd_eee_adv_cap1_t(eee->advertised);
        u16 eee_adv_cap2_t = rtl8125_ethtool_adv_to_mmd_eee_adv_cap2_t(eee->advertised);

        rtl8125_set_mac_ocp_bit(tp, 0xE040, (BIT_1|BIT_0));
        if (is_8125A(tp))
                rtl8125_set_mac_ocp_bit(tp, 0xEB62, (BIT_2|BIT_1));
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA432, BIT_4);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA5D0,
                MDIO_EEE_100TX | MDIO_EEE_1000T, eee_adv_cap1_t);

        if (is_8125A(tp))
                rtl8125_clear_eth_phy_ocp_bit(tp, 0xA6D4, MDIO_EEE_2_5GT);
        else
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA6D4, MDIO_EEE_2_5GT, eee_adv_cap2_t);

        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA6D8, BIT_4);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA428, BIT_7);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA4A2, BIT_9);
        /*Advanced EEE*/
        rtl8125_disable_adv_eee(tp);
        return 0;
}

static int rtl8125_disable_eee(struct rtl8125_private *tp)
{
        rtl8125_clear_mac_ocp_bit(tp, 0xE040, (BIT_1|BIT_0));
        //if (rtl8125_flag_is_set(tp, IsMcfg236)) {
        if (is_8125A(tp)) {
                rtl8125_clear_mac_ocp_bit(tp, 0xEB62, (BIT_2|BIT_1));
                rtl8125_clear_eth_phy_ocp_bit(tp, 0xA432, BIT_4);
        } else
                rtl8125_set_eth_phy_ocp_bit(tp, 0xA432, BIT_4);

        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA5D0, (MDIO_EEE_100TX | MDIO_EEE_1000T));
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA6D4, MDIO_EEE_2_5GT);  // BIT_0
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA6D8, BIT_4);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA428, BIT_7);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA4A2, BIT_9);

        /*Advanced EEE*/
        rtl8125_disable_adv_eee(tp);
        return 0;
}

static int rtl_nway_reset(struct net_device *dev)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        int bmcr;

#ifdef ENABLE_RTL_TOOL
        if (unlikely(tp->rtk_enable_diag))
                return -EBUSY;
#endif
        /* if autoneg is off, it's an error */
        rtl8125_mdio_set_page(tp, 0x0000);
        bmcr = rtl8125_mdio_read(tp, MII_BMCR);

        if (bmcr & BMCR_ANENABLE) {
                rtl8125_mdio_write(tp, MII_BMCR, bmcr | BMCR_ANRESTART);
                return 0;
        }
        return -EINVAL;
}

static u32
rtl8125_device_lpi_t_to_ethtool_lpi_t(struct rtl8125_private *tp , u32 lpi_timer)
{
        u32 to_us;
        u16 status;

        to_us = lpi_timer * 80;
        status = RTL_R16(tp, PHYstatus);
        if (status & LinkStatus) {
                /*link on*/
                //2.5G : lpi_timer * 3.2ns
                //Giga: lpi_timer * 8ns
                //100M : lpi_timer * 80ns
                if (status & _2500bpsF)
                        to_us = (lpi_timer * 32) / 10;
                else if (status & _1000bpsF)
                        to_us = lpi_timer * 8;
        }

        //ns to us
        to_us /= 1000;

        return to_us;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,9,0)
static void
rtl8125_adv_to_linkmode(unsigned long *mode, u64 adv)
{
        linkmode_zero(mode);

        if (adv & ADVERTISED_10baseT_Half)
                linkmode_set_bit(ETHTOOL_LINK_MODE_10baseT_Half_BIT, mode);
        if (adv & ADVERTISED_10baseT_Full)
                linkmode_set_bit(ETHTOOL_LINK_MODE_10baseT_Full_BIT, mode);
        if (adv & ADVERTISED_100baseT_Half)
                linkmode_set_bit(ETHTOOL_LINK_MODE_100baseT_Half_BIT, mode);
        if (adv & ADVERTISED_100baseT_Full)
                linkmode_set_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT, mode);
        if (adv & ADVERTISED_1000baseT_Half)
                linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseT_Half_BIT, mode);
        if (adv & ADVERTISED_1000baseT_Full)
                linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT, mode);
        if (adv & ADVERTISED_2500baseX_Full)
                linkmode_set_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT, mode);
}

static int
rtl8125_ethtool_get_eee(struct net_device *net, struct ethtool_keee *edata)
{
        __ETHTOOL_DECLARE_LINK_MODE_MASK(common);
        struct rtl8125_private *tp = netdev_priv(net);
        struct ethtool_keee *eee = &tp->eee;
        u32 tx_lpi_timer;
        u16 val;

#ifdef ENABLE_RTL_TOOL
        if (unlikely(tp->rtk_enable_diag))
                return -EBUSY;
#endif

        /* Get LP advertisement EEE */
        val = rtl8125_mdio_direct_read_phy_ocp(tp, 0xA5D2);
        mii_eee_cap1_mod_linkmode_t(edata->lp_advertised, val);
        val = rtl8125_mdio_direct_read_phy_ocp(tp, 0xA6D0);
        mii_eee_cap2_mod_linkmode_sup_t(edata->lp_advertised, val);

        /* Get EEE Tx LPI timer*/
        tx_lpi_timer = rtl8125_device_lpi_t_to_ethtool_lpi_t(tp, eee->tx_lpi_timer);

        val = rtl8125_mac_ocp_read(tp, 0xE040);
        val &= BIT_1 | BIT_0;

        edata->eee_enabled = !!val;
        linkmode_copy(edata->supported, eee->supported);
        linkmode_copy(edata->advertised, eee->advertised);
        edata->tx_lpi_enabled = edata->eee_enabled;
        edata->tx_lpi_timer = tx_lpi_timer;
        linkmode_and(common, edata->advertised, edata->lp_advertised);
        edata->eee_active = !linkmode_empty(common);

        return 0;
}

static int
rtl8125_ethtool_set_eee(struct net_device *dev, struct ethtool_keee *edata)
{
        __ETHTOOL_DECLARE_LINK_MODE_MASK(advertising);
        __ETHTOOL_DECLARE_LINK_MODE_MASK(tmp);
        struct rtl8125_private *tp = netdev_priv(dev);
        struct ethtool_keee *eee = &tp->eee;
        int rc = 0;

        if (!HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
                return -EOPNOTSUPP;

#ifdef ENABLE_RTL_TOOL
        if (unlikely(tp->rtk_enable_diag)) {
                netdev_printk(KERN_WARNING, dev, "Diag Enabled\n");
                rc = -EBUSY;
                goto out;
        }
#endif
        if (!rtl8125_flag_is_set(tp, AutoNegMode)) {
                netdev_printk(KERN_WARNING, dev, "EEE requires autoneg\n");
                rc = -EINVAL;
                goto out;
        }

        /*
        if (edata->tx_lpi_enabled) {
        if (edata->tx_lpi_timer > tp->max_jumbo_frame_size ||
            edata->tx_lpi_timer < ETH_MIN_MTU) {
                netdev_printk(KERN_WARNING, dev, "Valid LPI timer range is %d to %d. \n",
                           ETH_MIN_MTU, MAX_JUMBO_FRAME_SIZE);
                rc = -EINVAL;
                goto out;
        }
        }
        */

        rtl8125_adv_to_linkmode(advertising, tp->advertising);
        if (linkmode_empty(edata->advertised)) {
                linkmode_and(edata->advertised, advertising, eee->supported);
        } else if (linkmode_andnot(tmp, edata->advertised, advertising)) {
                netdev_printk(KERN_WARNING, dev, "advertised must be a subset of autoneg advertised speeds\n");
                rc = -EINVAL;
                goto out;
        }

        if (linkmode_andnot(tmp, edata->advertised, eee->supported)) {
                netdev_printk(KERN_WARNING, dev, "EEE advertised must be a subset of supported\n");
                rc = -EINVAL;
                goto out;
        }

        //tp->eee.eee_enabled = edata->eee_enabled;
        //tp->eee_adv_t = rtl8125_ethtool_adv_to_mmd_eee_adv_cap1_t(edata->advertised);

        linkmode_copy(eee->advertised, edata->advertised);
        //eee->tx_lpi_enabled = edata->tx_lpi_enabled;
        //eee->tx_lpi_timer = edata->tx_lpi_timer;
        eee->eee_enabled = edata->eee_enabled;

        if (eee->eee_enabled)
                rtl8125_enable_eee(tp);
        else
                rtl8125_disable_eee(tp);

        rtl_nway_reset(dev);

out:
        return rc;
}
#else
static int
rtl8125_ethtool_get_eee(struct net_device *net, struct ethtool_eee *edata)
{
        struct rtl8125_private *tp = netdev_priv(net);
        struct ethtool_eee *eee = &tp->eee;
        u32 lp, adv, tx_lpi_timer, supported = 0;
        u16 val;

#ifdef ENABLE_RTL_TOOL
        if (unlikely(tp->rtk_enable_diag))
                return -EBUSY;
#endif
        /* Get Supported EEE */
        //val = rtl8125_mdio_direct_read_phy_ocp(tp, 0xA5C4);
        //supported = mmd_eee_cap_to_ethtool_sup_t(val);
        supported = eee->supported;

        /* Get advertisement EEE */
        adv = eee->advertised;

        /* Get LP advertisement EEE */
        val = rtl8125_mdio_direct_read_phy_ocp(tp, 0xA5D2);
        lp = mmd_eee_adv_to_ethtool_adv_t(val);
        val = rtl8125_mdio_direct_read_phy_ocp(tp, 0xA6D0);
        if (val & RTK_LPA_EEE_ADVERTISE_2500FULL)
                lp |= ADVERTISED_2500baseX_Full;

        /* Get EEE Tx LPI timer*/
        tx_lpi_timer = rtl8125_device_lpi_t_to_ethtool_lpi_t(tp, eee->tx_lpi_timer);

        val = rtl8125_mac_ocp_read(tp, 0xE040) & (BIT_1 | BIT_0);
        edata->eee_enabled = !!val;
        edata->eee_active = !!(supported & adv & lp);
        edata->supported = supported;
        edata->advertised = adv;
        edata->lp_advertised = lp;
        edata->tx_lpi_enabled = edata->eee_enabled;
        edata->tx_lpi_timer = tx_lpi_timer;
        return 0;
}

static int
rtl8125_ethtool_set_eee(struct net_device *dev, struct ethtool_eee *edata)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        struct ethtool_eee *eee = &tp->eee;
        u64 advertising;
        int rc = 0;

        if (!HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
                return -EOPNOTSUPP;

#ifdef ENABLE_RTL_TOOL
        if (unlikely(tp->rtk_enable_diag)) {
                netdev_printk(KERN_WARNING, dev, "Diag Enabled\n");
                rc = -EBUSY;
                goto out;
        }
#endif

        if (!rtl8125_flag_is_set(tp, AutoNegMode)) {
                netdev_printk(KERN_WARNING, dev, "EEE requires autoneg\n");
                rc = -EINVAL;
                goto out;
        }

        /*
        if (edata->tx_lpi_enabled) {
        if (edata->tx_lpi_timer > MAX_JUMBO_FRAME_SIZE ||
            edata->tx_lpi_timer < ETH_MIN_MTU) {
                netdev_printk(KERN_WARNING, dev, "Valid LPI timer range is %d to %d. \n",
                           ETH_MIN_MTU, MAX_JUMBO_FRAME_SIZE);
                rc = -EINVAL;
                goto out;
        }
        }
        */

        advertising = tp->advertising;
        if (!edata->advertised) {
                edata->advertised = advertising & eee->supported;
        } else if (edata->advertised & ~advertising) {
                netdev_printk(KERN_WARNING, dev, "EEE advertised %x must be a subset of autoneg advertised speeds %llu\n",
                           edata->advertised, advertising);
                rc = -EINVAL;
                goto out;
        }

        if (edata->advertised & ~eee->supported) {
                netdev_printk(KERN_WARNING, dev, "EEE advertised %x must be a subset of support %x\n",
                           edata->advertised, eee->supported);
                rc = -EINVAL;
                goto out;
        }

        //tp->eee.eee_enabled = edata->eee_enabled;
        //tp->eee_adv_t = rtl8125_ethtool_adv_to_mmd_eee_adv_cap1_t(edata->advertised);
        netdev_printk(KERN_WARNING, dev, "EEE tx_lpi_timer %x must be a subset of support %x\n",
                   edata->tx_lpi_timer, eee->tx_lpi_timer);

        eee->advertised = edata->advertised;
        //eee->tx_lpi_enabled = edata->tx_lpi_enabled;
        //eee->tx_lpi_timer = edata->tx_lpi_timer;
        eee->eee_enabled = edata->eee_enabled;

        if (eee->eee_enabled)
                rtl8125_enable_eee(tp);
        else
                rtl8125_disable_eee(tp);
        rtl_nway_reset(dev);

out:
        return rc;
}
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(6,9,0) */

static void rtl8125_get_channels(struct net_device *dev,
                                 struct ethtool_channels *channel)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        channel->max_rx = HW_SUPP_NUM_RX_QUEUES(tp);
        channel->max_tx = HW_SUPP_NUM_TX_QUEUES(tp);
        channel->rx_count = tp->num_rx_rings;
        channel->tx_count = tp->num_tx_rings;
}

static const struct ethtool_ops rtl8125_ethtool_ops = {
        .get_drvinfo        = rtl8125_get_drvinfo,
        .get_regs_len       = rtl8125_get_regs_len,
        .get_link           = ethtool_op_get_link,
        .get_ringparam      = rtl8125_get_ringparam,
        .set_ringparam      = rtl8125_set_ringparam,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0)
        .get_settings       = rtl8125_get_settings,
        .set_settings       = rtl8125_set_settings,
#else
        .get_link_ksettings = rtl8125_get_settings,
        .set_link_ksettings = rtl8125_set_settings,
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0)
        .get_pauseparam     = rtl8125_get_pauseparam,
        .set_pauseparam     = rtl8125_set_pauseparam,
        .get_msglevel       = rtl8125_get_msglevel,
        .set_msglevel       = rtl8125_set_msglevel,
        .get_regs           = rtl8125_get_regs,
        .get_wol            = rtl8125_get_wol,
        .set_wol            = rtl8125_set_wol,
        .get_strings        = rtl8125_get_strings,
        .get_sset_count     = rtl8125_get_sset_count,
        .get_ethtool_stats  = rtl8125_get_ethtool_stats,
#ifdef ENABLE_R8125_EEPROM
        .get_eeprom         = rtl_get_eeprom,
        .get_eeprom_len     = rtl_get_eeprom_len,
#endif
#ifdef ENABLE_RSS_SUPPORT
        .get_rxnfc                = rtl8125_get_rxnfc,
        .set_rxnfc                = rtl8125_set_rxnfc,
        .get_rxfh_indir_size        = rtl8125_rss_indir_size,
        .get_rxfh_key_size        = rtl8125_get_rxfh_key_size,
        .get_rxfh                = rtl8125_get_rxfh,
        .set_rxfh                = rtl8125_set_rxfh,
#endif //ENABLE_RSS_SUPPORT
#ifdef ENABLE_PTP_SUPPORT
        .get_ts_info        = rtl8125_get_ts_info,
#else
        .get_ts_info        = ethtool_op_get_ts_info,
#endif //ENABLE_PTP_SUPPORT
        .get_eee            = rtl8125_ethtool_get_eee,
        .set_eee            = rtl8125_ethtool_set_eee,
        .get_channels            = rtl8125_get_channels,
        .nway_reset         = rtl_nway_reset,

};

static void rtl8125_get_mac_version(struct rtl8125_private *tp)
{
        u32 val32 = RTL_R32(tp, TxConfig);
        u32 reg = val32 & 0x7c800000;
        u32 ICVerID = val32 & 0x00700000;
        struct pci_dev *pdev = tp->pci_dev;

        switch (reg) {
        case 0x60800000:
                if (ICVerID == 0x00000000)
                        tp->mcfg = CFG_METHOD_2;        // 8125A_1
                else if (ICVerID == 0x100000)
                        tp->mcfg = CFG_METHOD_3;        // 8125A_2
                else
                        tp->mcfg = CFG_METHOD_UNKNOWN;
                break;
        case 0x64000000:
                if (ICVerID == 0x00000000)
                        tp->mcfg = CFG_METHOD_4;        // 8125B_1
                else if (ICVerID == 0x100000)
                        tp->mcfg = CFG_METHOD_5;        // 8125B_2 (RTL8125BG(S)-CG)
                else
                        tp->mcfg = CFG_METHOD_UNKNOWN;
                break;
        case 0x68000000:
                if (ICVerID == 0x00000000)
                        tp->mcfg = CFG_METHOD_8;        // 8125BP_1
                else if (ICVerID == 0x100000)
                        tp->mcfg = CFG_METHOD_9;        // 8125BP_2
                else
                        tp->mcfg = CFG_METHOD_UNKNOWN;
                break;
        case 0x68800000:
                if (ICVerID == 0x00000000)
                        tp->mcfg = CFG_METHOD_10;       // 8125D_1
                else if (ICVerID == 0x100000)
                        tp->mcfg = CFG_METHOD_11;       // 8125D_2
                else
                        tp->mcfg = CFG_METHOD_UNKNOWN;
                break;
        default:
                tp->mcfg = CFG_METHOD_UNKNOWN;
                break;
        }

        /* Remove CFG_METHOD_6/7 8125_A/B limited to 1000Mb/s versus 2500Mb/s */
        rtl8125_set_flag(tp, HwSuppPhyLinkSpeed_2500);
        //tp->HwSuppMaxPhyLinkSpeed = 2500;

        // 0x8162 type devices are limited to 1000Mb/s
        if (pdev->device == 0x8162 &&
                (tp->mcfg == CFG_METHOD_3 || tp->mcfg == CFG_METHOD_5))
                rtl8125_clear_flag(tp, HwSuppPhyLinkSpeed_2500);
                //tp->HwSuppMaxPhyLinkSpeed = 1000;
}

static void
rtl8125_print_mac_version(struct rtl8125_private *tp)
{
        printk(KERN_INFO "Realtek r8125 Ethernet controller driver (%s chipset, version %s)\n",
                rtl_chip_info[chipset(tp)].name, RTL8125_VERSION);
}

static void
rtl8125_tally_counter_addr_fill(struct rtl8125_private *tp)
{
        if (!tp->tally_paddr)
                return;
        RTL_W32(tp, CounterAddrHigh, (u64)tp->tally_paddr >> 32);
        RTL_W32(tp, CounterAddrLow, (u64)tp->tally_paddr & (DMA_BIT_MASK(32)));
}

static void
rtl8125_tally_counter_clear(struct rtl8125_private *tp)
{
        if (!tp->tally_paddr)
                return;
        RTL_W32(tp, CounterAddrHigh, (u64)tp->tally_paddr >> 32);
        RTL_W32(tp, CounterAddrLow, ((u64)tp->tally_paddr & (DMA_BIT_MASK(32))) | CounterReset);
}

static void
rtl8125_clear_phy_ups_reg(struct rtl8125_private *tp)
{
        //if (!rtl8125_flag_is_set(tp, IsMcfg236))
        if (!is_8125A(tp))
                rtl8125_clear_eth_phy_ocp_bit(tp, 0xA466, BIT_0);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA468, BIT_3 | BIT_1);
}

static inline int
rtl8125_is_ups_resume(struct rtl8125_private *tp)
{
        return (rtl8125_mac_ocp_read(tp, 0xD42C) & BIT_8);
}

static inline void
rtl8125_clear_ups_resume_bit(struct rtl8125_private *tp)
{
        rtl8125_clear_mac_ocp_bit(tp, 0xD42C, BIT_8);
}

static inline u8
rtl8125_get_phy_state(struct rtl8125_private *tp)
{
        return (rtl8125_mdio_direct_read_phy_ocp(tp, 0xA420) & 0x7);
}

static void
rtl8125_wait_phy_ups_resume(struct rtl8125_private * const tp, const u16 PhyState)
{
        for (int i = 0; i < 100; i++) {
                if (rtl8125_get_phy_state(tp) == PhyState)
                        return;
                mdelay(1);
        }
        WARN_ON_ONCE(1);
}

void
rtl8125_enable_now_is_oob(const struct rtl8125_private * const tp)
{
        RTL_W8(tp, MCUCmd_reg, RTL_R8(tp, MCUCmd_reg) | Now_is_oob);
}

void
rtl8125_disable_now_is_oob(const struct rtl8125_private * const tp)
{
        RTL_W8(tp, MCUCmd_reg, RTL_R8(tp, MCUCmd_reg) & ~Now_is_oob);
}

static void
rtl8125_exit_oob(struct rtl8125_private * const tp)
{
        u16 data16;
        rtl8125_disable_rx_packet_filter(tp);

#ifdef ENABLE_REALWOW_SUPPORT
        rtl8125_realwow_hw_init(tp->dev);
#else
        //Disable realwow  function
        rtl8125_mac_ocp_write(tp, 0xC0BC, 0x00FF);

#endif //ENABLE_REALWOW_SUPPORT

        rtl8125_nic_reset(tp);
        rtl8125_disable_now_is_oob(tp);
        //rtl8125_clear_mac_ocp_bit(tp, 0xE8DE, BIT_14);

        data16 = rtl8125_mac_ocp_read(tp, 0xE8DE) & ~BIT_14;
        rtl8125_mac_ocp_write(tp, 0xE8DE, data16);
        rtl8125_wait_ll_share_fifo_ready(tp);
        rtl8125_mac_ocp_write(tp, 0xC0AA, 0x07D0);
        rtl8125_mac_ocp_write(tp, 0xC0A6, 0x01B5);
        rtl8125_mac_ocp_write(tp, 0xC01E, 0x5555);
        rtl8125_wait_ll_share_fifo_ready(tp);

        //wait ups resume (phy state 2)
        if (rtl8125_is_ups_resume(tp)) {
                rtl8125_wait_phy_ups_resume(tp, 2);
                rtl8125_clear_ups_resume_bit(tp);
                rtl8125_clear_phy_ups_reg(tp);
        }
}

void
rtl8125_hw_disable_mac_mcu_bps(const struct rtl8125_private * const tp)
{
        rtl8125_enable_aspm_clkreq_lock(tp, 0);
        rtl8125_mac_ocp_write(tp, 0xFC48, 0x0000);
        for (u16 regAddr = 0xFC28; regAddr < 0xFC48; regAddr += 2) {
                rtl8125_mac_ocp_write(tp, regAddr, 0x0000);
        }

        mdelay(3);
        rtl8125_mac_ocp_write(tp, 0xFC26, 0x0000);
}

#ifndef ENABLE_USE_FIRMWARE_FILE
static void
rtl8125_switch_mac_mcu_ram_code_page(const struct rtl8125_private *tp, const u16 page)
{
        rtl8125_clear_set_mac_ocp_bit(tp, 0xE446, (BIT_1 | BIT_0), page & (BIT_1 | BIT_0));
}

static void
rtl8125_write_mac_mcu_ram_code(const struct rtl8125_private *const tp, const u16 *entry, const u16 entry_cnt)
{
        u16 offset, page = 0;
        if (entry == NULL || entry_cnt == 0)
                return;

        for (u16 i = 0; i < entry_cnt; i++) {
                offset = i & (RTL8125_MAC_MCU_PAGE_SIZE - 1);
                if (offset == 0)
                        rtl8125_switch_mac_mcu_ram_code_page(tp, page++);
                rtl8125_mac_ocp_write(tp, 0xF800 + offset * 2, entry[i]);
        }
        //_rtl8125_write_mac_mcu_ram_code_with_page(tp, entry, entry_cnt, RTL8125_MAC_MCU_PAGE_SIZE);
}

static void
rtl8125_set_mac_mcu_8125a_1(struct rtl8125_private *tp)
{
        rtl8125_hw_disable_mac_mcu_bps(tp);
}

static void
rtl8125_set_mac_mcu_8125a_2(struct rtl8125_private *tp)
{
        static const u16 mcu_patch_code_8125a_2[] = {
                0xE010, 0xE012, 0xE022, 0xE024, 0xE029, 0xE02B, 0xE094, 0xE09D, 0xE09F,
                0xE0AA, 0xE0B5, 0xE0C6, 0xE0CC, 0xE0D1, 0xE0D6, 0xE0D8, 0xC602, 0xBE00,
                0x0000, 0xC60F, 0x73C4, 0x49B3, 0xF106, 0x73C2, 0xC608, 0xB406, 0xC609,
                0xFF80, 0xC605, 0xB406, 0xC605, 0xFF80, 0x0544, 0x0568, 0xE906, 0xCDE8,
                0xC602, 0xBE00, 0x0000, 0x48C1, 0x48C2, 0x9C46, 0xC402, 0xBC00, 0x0A12,
                0xC602, 0xBE00, 0x0EBA, 0x1501, 0xF02A, 0x1500, 0xF15D, 0xC661, 0x75C8,
                0x49D5, 0xF00A, 0x49D6, 0xF008, 0x49D7, 0xF006, 0x49D8, 0xF004, 0x75D2,
                0x49D9, 0xF150, 0xC553, 0x77A0, 0x75C8, 0x4855, 0x4856, 0x4857, 0x4858,
                0x48DA, 0x48DB, 0x49FE, 0xF002, 0x485A, 0x49FF, 0xF002, 0x485B, 0x9DC8,
                0x75D2, 0x4859, 0x9DD2, 0xC643, 0x75C0, 0x49D4, 0xF033, 0x49D0, 0xF137,
                0xE030, 0xC63A, 0x75C8, 0x49D5, 0xF00E, 0x49D6, 0xF00C, 0x49D7, 0xF00A,
                0x49D8, 0xF008, 0x75D2, 0x49D9, 0xF005, 0xC62E, 0x75C0, 0x49D7, 0xF125,
                0xC528, 0x77A0, 0xC627, 0x75C8, 0x4855, 0x4856, 0x4857, 0x4858, 0x48DA,
                0x48DB, 0x49FE, 0xF002, 0x485A, 0x49FF, 0xF002, 0x485B, 0x9DC8, 0x75D2,
                0x4859, 0x9DD2, 0xC616, 0x75C0, 0x4857, 0x9DC0, 0xC613, 0x75C0, 0x49DA,
                0xF003, 0x49D0, 0xF107, 0xC60B, 0xC50E, 0x48D9, 0x9DC0, 0x4859, 0x9DC0,
                0xC608, 0xC702, 0xBF00, 0x3AE0, 0xE860, 0xB400, 0xB5D4, 0xE908, 0xE86C,
                0x1200, 0xC409, 0x6780, 0x48F1, 0x8F80, 0xC404, 0xC602, 0xBE00, 0x10AA,
                0xC010, 0xEA7C, 0xC602, 0xBE00, 0x0000, 0x740A, 0x4846, 0x4847, 0x9C0A,
                0xC607, 0x74C0, 0x48C6, 0x9CC0, 0xC602, 0xBE00, 0x13FE, 0xE054, 0x72CA,
                0x4826, 0x4827, 0x9ACA, 0xC607, 0x72C0, 0x48A6, 0x9AC0, 0xC602, 0xBE00,
                0x07DC, 0xE054, 0xC60F, 0x74C4, 0x49CC, 0xF109, 0xC60C, 0x74CA, 0x48C7,
                0x9CCA, 0xC609, 0x74C0, 0x4846, 0x9CC0, 0xC602, 0xBE00, 0x2480, 0xE092,
                0xE0C0, 0xE054, 0x7420, 0x48C0, 0x9C20, 0x7444, 0xC602, 0xBE00, 0x12F8,
                0x1BFF, 0x46EB, 0x1BFF, 0xC102, 0xB900, 0x0D5A, 0x1BFF, 0x46EB, 0x1BFF,
                0xC102, 0xB900, 0x0E2A, 0xC602, 0xBE00, 0x0000, 0xC602, 0xBE00, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x6486,
                0x0B15, 0x090E, 0x1139
        };

        rtl8125_hw_disable_mac_mcu_bps(tp);
        rtl8125_write_mac_mcu_ram_code(tp, mcu_patch_code_8125a_2, ARRAY_SIZE(mcu_patch_code_8125a_2));

        rtl8125_mac_ocp_write(tp, 0xFC26, 0x8000);
        rtl8125_mac_ocp_write(tp, 0xFC2A, 0x0540);
        rtl8125_mac_ocp_write(tp, 0xFC2E, 0x0A06);
        rtl8125_mac_ocp_write(tp, 0xFC30, 0x0EB8);
        rtl8125_mac_ocp_write(tp, 0xFC32, 0x3A5C);
        rtl8125_mac_ocp_write(tp, 0xFC34, 0x10A8);
        rtl8125_mac_ocp_write(tp, 0xFC40, 0x0D54);
        rtl8125_mac_ocp_write(tp, 0xFC42, 0x0E24);
        rtl8125_mac_ocp_write(tp, 0xFC48, 0x307A);
}

static void
rtl8125_set_mac_mcu_8125b_1(struct rtl8125_private *tp)
{
        rtl8125_hw_disable_mac_mcu_bps(tp);
}

static void
rtl8125_set_mac_mcu_8125b_2(struct rtl8125_private *tp)
{
        static const u16 mcu_patch_code_8125b_2[] = {
                0xE010, 0xE01B, 0xE026, 0xE037, 0xE03D, 0xE057, 0xE05B, 0xE060, 0xE062,
                0xE064, 0xE066, 0xE068, 0xE06A, 0xE06C, 0xE06E, 0xE070, 0x740A, 0x4846,
                0x4847, 0x9C0A, 0xC607, 0x74C0, 0x48C6, 0x9CC0, 0xC602, 0xBE00, 0x13F0,
                0xE054, 0x72CA, 0x4826, 0x4827, 0x9ACA, 0xC607, 0x72C0, 0x48A6, 0x9AC0,
                0xC602, 0xBE00, 0x081C, 0xE054, 0xC60F, 0x74C4, 0x49CC, 0xF109, 0xC60C,
                0x74CA, 0x48C7, 0x9CCA, 0xC609, 0x74C0, 0x4846, 0x9CC0, 0xC602, 0xBE00,
                0x2494, 0xE092, 0xE0C0, 0xE054, 0x7420, 0x48C0, 0x9C20, 0x7444, 0xC602,
                0xBE00, 0x12DC, 0x733A, 0x21B5, 0x25BC, 0x1304, 0xF111, 0x1B12, 0x1D2A,
                0x3168, 0x3ADA, 0x31AB, 0x1A00, 0x9AC0, 0x1300, 0xF1FB, 0x7620, 0x236E,
                0x276F, 0x1A3C, 0x22A1, 0x41B5, 0x9EE2, 0x76E4, 0x486F, 0x9EE4, 0xC602,
                0xBE00, 0x4A26, 0x733A, 0x49BB, 0xC602, 0xBE00, 0x47A2, 0x48C1, 0x48C2,
                0x9C46, 0xC402, 0xBC00, 0x0A52, 0xC602, 0xBE00, 0x0000, 0xC602, 0xBE00,
                0x0000, 0xC602, 0xBE00, 0x0000, 0xC602, 0xBE00, 0x0000, 0xC602, 0xBE00,
                0x0000, 0xC602, 0xBE00, 0x0000, 0xC602, 0xBE00, 0x0000, 0xC602, 0xBE00,
                0x0000, 0xC602, 0xBE00, 0x0000
        };

        rtl8125_hw_disable_mac_mcu_bps(tp);
        rtl8125_write_mac_mcu_ram_code(tp, mcu_patch_code_8125b_2, ARRAY_SIZE(mcu_patch_code_8125b_2));

        rtl8125_mac_ocp_write(tp, 0xFC26, 0x8000);
        rtl8125_mac_ocp_write(tp, 0xFC28, 0x13E6);
        rtl8125_mac_ocp_write(tp, 0xFC2A, 0x0812);
        rtl8125_mac_ocp_write(tp, 0xFC2C, 0x248C);
        rtl8125_mac_ocp_write(tp, 0xFC2E, 0x12DA);
        rtl8125_mac_ocp_write(tp, 0xFC30, 0x4A20);
        rtl8125_mac_ocp_write(tp, 0xFC32, 0x47A0);
        //rtl8125_mac_ocp_write(tp, 0xFC34, 0x0A46);
        rtl8125_mac_ocp_write(tp, 0xFC48, 0x003F);
}

static void
rtl8125_set_mac_mcu_8125bp_1(struct rtl8125_private *tp)
{
        static const u16 mcu_patch_code_8125bp_1[] = {
                0xE003, 0xE007, 0xE01A, 0x1BC8, 0x46EB, 0xC302, 0xBB00, 0x0F14, 0xC211,
                0x400A, 0xF00A, 0xC20F, 0x400A, 0xF007, 0x73A4, 0xC20C, 0x400A, 0xF102,
                0x48B0, 0x9B20, 0x1B00, 0x9BA0, 0xC602, 0xBE00, 0x4364, 0xE6E0, 0xE6E2,
                0xC01C, 0xB406, 0x1000, 0xF016, 0xC61F, 0x400E, 0xF012, 0x218E, 0x25BE,
                0x1300, 0xF007, 0x7340, 0xC618, 0x400E, 0xF102, 0x48B0, 0x8320, 0xB400,
                0x2402, 0x1000, 0xF003, 0x7342, 0x8322, 0xB000, 0xE007, 0x7322, 0x9B42,
                0x7320, 0x9B40, 0x0300, 0x0300, 0xB006, 0xC302, 0xBB00, 0x413E, 0xE6E0,
                0xC01C, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x1171, 0x0B17, 0x0816, 0x1108
        };

        rtl8125_hw_disable_mac_mcu_bps(tp);
        rtl8125_write_mac_mcu_ram_code(tp, mcu_patch_code_8125bp_1, ARRAY_SIZE(mcu_patch_code_8125bp_1));

        rtl8125_mac_ocp_write(tp, 0xFC26, 0x8000);
        rtl8125_mac_ocp_write(tp, 0xFC28, 0x0f10);
        rtl8125_mac_ocp_write(tp, 0xFC2A, 0x435c);
        rtl8125_mac_ocp_write(tp, 0xFC2C, 0x4112);
        rtl8125_mac_ocp_write(tp, 0xFC48, 0x0007);
}

static void
rtl8125_set_mac_mcu_8125bp_2(struct rtl8125_private *tp)
{
        static const u16 mcu_patch_code_8125bp_2[] = {
                0xE010, 0xE033, 0xE046, 0xE04A, 0xE04C, 0xE04E, 0xE050, 0xE052, 0xE054,
                0xE056, 0xE058, 0xE05A, 0xE05C, 0xE05E, 0xE060, 0xE062, 0xB406, 0x1000,
                0xF016, 0xC61F, 0x400E, 0xF012, 0x218E, 0x25BE, 0x1300, 0xF007, 0x7340,
                0xC618, 0x400E, 0xF102, 0x48B0, 0x8320, 0xB400, 0x2402, 0x1000, 0xF003,
                0x7342, 0x8322, 0xB000, 0xE007, 0x7322, 0x9B42, 0x7320, 0x9B40, 0x0300,
                0x0300, 0xB006, 0xC302, 0xBB00, 0x4168, 0xE6E0, 0xC01C, 0xC211, 0x400A,
                0xF00A, 0xC20F, 0x400A, 0xF007, 0x73A4, 0xC20C, 0x400A, 0xF102, 0x48B0,
                0x9B20, 0x1B00, 0x9BA0, 0xC602, 0xBE00, 0x4392, 0xE6E0, 0xE6E2, 0xC01C,
                0x4166, 0x9CF6, 0xC002, 0xB800, 0x143C, 0xC602, 0xBE00, 0x0000, 0xC602,
                0xBE00, 0x0000, 0xC602, 0xBE00, 0x0000, 0xC602, 0xBE00, 0x0000, 0xC102,
                0xB900, 0x0000, 0xC002, 0xB800, 0x0000, 0xC602, 0xBE00, 0x0000, 0xC602,
                0xBE00, 0x0000, 0xC602, 0xBE00, 0x0000, 0xC602, 0xBE00, 0x0000, 0xC602,
                0xBE00, 0x0000, 0xC602, 0xBE00, 0x0000, 0xC602, 0xBE00, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1171,
                0x0B18, 0x030D, 0x0A2A
        };

        rtl8125_hw_disable_mac_mcu_bps(tp);
        rtl8125_write_mac_mcu_ram_code(tp, mcu_patch_code_8125bp_2, ARRAY_SIZE(mcu_patch_code_8125bp_2));
        rtl8125_mac_ocp_write(tp, 0xFC26, 0x8000);
        rtl8125_mac_ocp_write(tp, 0xFC28, 0x413C);
        rtl8125_mac_ocp_write(tp, 0xFC2A, 0x438A);
        rtl8125_mac_ocp_write(tp, 0xFC2C, 0x143A);
        rtl8125_mac_ocp_write(tp, 0xFC48, 0x0007);
}

static void
rtl8125_set_mac_mcu_8125d_1(struct rtl8125_private *tp)
{
        static const u16 mcu_patch_code_8125d_1[] = {
                0xE002, 0xE006, 0x4166, 0x9CF6, 0xC002, 0xB800, 0x14A4, 0xC102, 0xB900,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x6938,
                0x0A18, 0x0217, 0x0D2A
        };

        rtl8125_hw_disable_mac_mcu_bps(tp);
        rtl8125_write_mac_mcu_ram_code(tp, mcu_patch_code_8125d_1, ARRAY_SIZE(mcu_patch_code_8125d_1));
        rtl8125_mac_ocp_write(tp, 0xFC26, 0x8000);
        rtl8125_mac_ocp_write(tp, 0xFC28, 0x14A2);
        rtl8125_mac_ocp_write(tp, 0xFC48, 0x0001);
}

static void
rtl8125_set_mac_mcu_8125d_2(struct rtl8125_private *tp)
{
        rtl8125_hw_disable_mac_mcu_bps(tp);
}

static void
rtl8125_hw_mac_mcu_config(struct rtl8125_private *tp)
{
        switch (tp->mcfg) {
        case CFG_METHOD_2:
                rtl8125_set_mac_mcu_8125a_1(tp);
                break;
        case CFG_METHOD_3:
                rtl8125_set_mac_mcu_8125a_2(tp);
                break;
        case CFG_METHOD_4:
                rtl8125_set_mac_mcu_8125b_1(tp);
                break;
        case CFG_METHOD_5:
                rtl8125_set_mac_mcu_8125b_2(tp);
                break;
        case CFG_METHOD_8:
                rtl8125_set_mac_mcu_8125bp_1(tp);
                break;
        case CFG_METHOD_9:
                rtl8125_set_mac_mcu_8125bp_2(tp);
                break;
        case CFG_METHOD_10:
                rtl8125_set_mac_mcu_8125d_1(tp);
                break;
        case CFG_METHOD_11:
                rtl8125_set_mac_mcu_8125d_2(tp);
                break;
        }
}
#else // ENABLE_USE_FIRMWARE_FILE
static void rtl8125_release_firmware(struct rtl8125_private *tp)
{
        if (tp->rtl_fw) {
                rtl8125_fw_release_firmware(tp->rtl_fw);
                kfree(tp->rtl_fw);
                tp->rtl_fw = NULL;
        }
}

static void rtl8125_apply_firmware(struct rtl8125_private *tp)
{
        /* TODO: release firmware if rtl_fw_write_firmware signals failure. */
        if (tp->rtl_fw) {
                rtl8125_fw_write_firmware(tp, tp->rtl_fw);
                /* At least one firmware doesn't reset tp->ocp_base. */
                tp->ocp_base = OCP_STD_PHY_BASE;

                /* PHY soft reset may still be in progress */
                //phy_read_poll_timeout(tp->phydev, MII_BMCR, val,
                //                      !(val & BMCR_RESET),
                //                      50000, 600000, true);
                rtl8125_wait_phy_reset_complete(tp);

                tp->hw_ram_code_ver = rtl8125_get_hw_phy_mcu_code_ver(tp);
                tp->sw_ram_code_ver = tp->hw_ram_code_ver;
                rtl8125_set_flag(tp, HwHasWrRamCodeToMicroP);
        }
}
#endif // ENABLE_USE_FIRMWARE_FILE

static void
rtl8125_hw_init(struct rtl8125_private *tp)
{
        u32 csi_tmp;

        rtl8125_enable_aspm_clkreq_lock(tp, 0);
        rtl8125_enable_force_clkreq(tp, 0);

        //Disable UPS
        rtl8125_mac_ocp_write(tp, 0xD40A, rtl8125_mac_ocp_read(tp, 0xD40A) & ~(BIT_4));

#ifndef ENABLE_USE_FIRMWARE_FILE
        if (!tp->rtl_fw)
                rtl8125_hw_mac_mcu_config(tp);
#endif

        /*disable ocp phy power saving*/
        //if (rtl8125_flag_is_set(tp, IsMcfg236))
        if (is_8125A(tp))
                rtl8125_disable_ocp_phy_power_saving(tp);

        //Set PCIE uncorrectable error status mask pcie 0x108
        csi_tmp = rtl8125_csi_read(tp, 0x108) | BIT_20;
        rtl8125_csi_write(tp, 0x108, csi_tmp);

        rtl8125_unlock_config_regs(tp);
        rtl8125_set_linkchg_wakeup(tp, false);
        rtl8125_lock_config_regs(tp);

#ifdef ENABLE_S0_MAGIC_PACKET
        rtl8125_set_magic_packet(tp, true);
#else
        rtl8125_set_magic_packet(tp, false);
#endif
        rtl8125_disable_d0_speedup(tp);
        rtl8125_set_pci_pme(tp, 0);

#ifdef ENABLE_USE_FIRMWARE_FILE
        if (tp->rtl_fw && !rtl8125_flag_is_set(tp, ResumeNoSpeedChange))
                rtl8125_apply_firmware(tp);
#endif
}

struct ephy_info {
        u8 offset;
        //u16 mask;
        u16 data;
};

static void __rtl_ephy_init(struct rtl8125_private *tp, const struct ephy_info *e, int len)
{
        while (len-- > 0) {
                //w = (rtl_ephy_read(tp, e->offset) & ~e->mask) | e->bits;
                rtl8125_ephy_write(tp, e->offset, e->data);
                e++;
        }
}

#define rtl_ephy_init(tp, a) __rtl_ephy_init(tp, a, ARRAY_SIZE(a))
static void
rtl8125_hw_ephy_config(struct rtl8125_private *tp)
{
        switch (tp->mcfg) {
        case CFG_METHOD_2:
                static const struct ephy_info e_info_2[] = {
                { 0x01, 0xA812 }, { 0x09, 0x520C }, { 0x04, 0xD000 }, { 0x0D, 0xF702 },
                { 0x0A, 0x8653 }, { 0x06, 0x001E }, { 0x08, 0x3595 }, { 0x20, 0x9455 },
                { 0x21, 0x99FF }, { 0x02, 0x6046 }, { 0x29, 0xFE00 }, { 0x23, 0xAB62 },
                { 0x41, 0xA80C }, { 0x49, 0x520C }, { 0x44, 0xD000 }, { 0x4D, 0xF702 },
                { 0x4A, 0x8653 }, { 0x46, 0x001E }, { 0x48, 0x3595 }, { 0x60, 0x9455 },
                { 0x61, 0x99FF }, { 0x42, 0x6046 }, { 0x69, 0xFE00 }, { 0x63, 0xAB62 } };
                rtl_ephy_init(tp, e_info_2);
                break;
        case CFG_METHOD_3:
                static const struct ephy_info e_info_3[] = {
                { 0x04, 0xD000 }, { 0x0A, 0x8653 }, { 0x23, 0xAB66 }, { 0x20, 0x9455 },
                { 0x21, 0x99FF }, { 0x29, 0xFE04 }, { 0x44, 0xD000 }, { 0x4A, 0x8653 },
                { 0x63, 0xAB66 }, { 0x60, 0x9455 }, { 0x61, 0x99FF }, { 0x69, 0xFE04 } };
                rtl_ephy_init(tp, e_info_3);

                ClearAndSetPCIePhyBit(tp, 0x2A, (BIT_14 | BIT_13 | BIT_12), (BIT_13 | BIT_12));
                ClearPCIePhyBit(tp, 0x19, BIT_6);
                SetPCIePhyBit(tp, 0x1B, (BIT_11 | BIT_10 | BIT_9));
                ClearPCIePhyBit(tp, 0x1B, (BIT_14 | BIT_13 | BIT_12));
                rtl8125_ephy_write(tp, 0x02, 0x6042);
                rtl8125_ephy_write(tp, 0x06, 0x0014);

                ClearAndSetPCIePhyBit(tp, 0x6A, (BIT_14 | BIT_13 | BIT_12), (BIT_13 | BIT_12));
                ClearPCIePhyBit(tp, 0x59, BIT_6);
                SetPCIePhyBit(tp, 0x5B, (BIT_11 | BIT_10 | BIT_9));
                ClearPCIePhyBit(tp, 0x5B, (BIT_14 | BIT_13 | BIT_12));
                rtl8125_ephy_write(tp, 0x42, 0x6042);
                rtl8125_ephy_write(tp, 0x46, 0x0014);
                break;
        case CFG_METHOD_4:
                static const struct ephy_info e_info_4[] = {
                { 0x06, 0x001F }, { 0x0A, 0xB66B }, { 0x01, 0xA852 }, { 0x24, 0x0008 },
                { 0x2F, 0x6052 }, { 0x0D, 0xF716 }, { 0x20, 0xD477 }, { 0x21, 0x4477 },
                { 0x22, 0x0013 }, { 0x23, 0xBB66 }, { 0x0B, 0xA909 }, { 0x29, 0xFF04 },
                { 0x1B, 0x1EA0 }, { 0x46, 0x001F }, { 0x4A, 0xB66B }, { 0x41, 0xA84A },
                { 0x64, 0x000C }, { 0x6F, 0x604A }, { 0x4D, 0xF716 }, { 0x60, 0xD477 },
                { 0x61, 0x4477 }, { 0x62, 0x0013 }, { 0x63, 0xBB66 }, { 0x4B, 0xA909 },
                { 0x69, 0xFF04 }, { 0x5B, 0x1EA0 } };
                rtl_ephy_init(tp, e_info_4);
                break;
        case CFG_METHOD_5:
                static const struct ephy_info e_info_5[] = {
                { 0x0B, 0xA908 }, { 0x1E, 0x20EB }, { 0x22, 0x0023 }, { 0x02, 0x60C2 },
                { 0x29, 0xFF00 }, { 0x4B, 0xA908 }, { 0x5E, 0x28EB }, { 0x62, 0x0023 },
                { 0x42, 0x60C2 }, { 0x69, 0xFF00 } };
                rtl_ephy_init(tp, e_info_5);
                break;
        case CFG_METHOD_8...CFG_METHOD_11:
                /* nothing to do */
                break;
        }
}

static u16
rtl8125_get_hw_phy_mcu_code_ver(const struct rtl8125_private * const tp)
{
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x801E);
        return rtl8125_mdio_direct_read_phy_ocp(tp, 0xA438);
}

static int
rtl8125_check_hw_phy_mcu_code_ver(struct rtl8125_private *tp)
{
        tp->hw_ram_code_ver = rtl8125_get_hw_phy_mcu_code_ver(tp);

        if (tp->hw_ram_code_ver == tp->sw_ram_code_ver) {
                rtl8125_set_flag(tp, HwHasWrRamCodeToMicroP);
                return 1;
        }
        rtl8125_clear_flag(tp, HwHasWrRamCodeToMicroP);
        return 0;
}

bool
rtl8125_set_phy_mcu_patch_request(struct rtl8125_private *tp)
{
        u16 gphy_val;
        u16 WaitCount;
        bool bSuccess = TRUE;

        rtl8125_set_eth_phy_ocp_bit(tp, 0xB820, BIT_4);

        WaitCount = 0;
        do {
                gphy_val = rtl8125_mdio_direct_read_phy_ocp(tp, 0xB800);
                udelay(100);
                WaitCount++;
        } while (!(gphy_val & BIT_6) && (WaitCount < 1000));

        if (!(gphy_val & BIT_6) && (WaitCount == 1000))
                bSuccess = FALSE;

        if (!bSuccess)
                dprintk("rtl8125_set_phy_mcu_patch_request fail.\n");

        return bSuccess;
}

bool
rtl8125_clear_phy_mcu_patch_request(struct rtl8125_private *tp)
{
        u16 gphy_val;
        u16 WaitCount;
        bool bSuccess = TRUE;

        rtl8125_clear_eth_phy_ocp_bit(tp, 0xB820, BIT_4);

        WaitCount = 0;
        do {
                gphy_val = rtl8125_mdio_direct_read_phy_ocp(tp, 0xB800);
                udelay(100);
                WaitCount++;
        } while ((gphy_val & BIT_6) && (WaitCount < 1000));

        if ((gphy_val & BIT_6) && (WaitCount == 1000))
                bSuccess = FALSE;

        if (!bSuccess)
                dprintk("rtl8125_clear_phy_mcu_patch_request fail.\n");

        return bSuccess;
}

#ifndef ENABLE_USE_FIRMWARE_FILE
static void
rtl8125_write_hw_phy_mcu_code_ver(struct rtl8125_private *tp)
{
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x801E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, tp->sw_ram_code_ver);
        tp->hw_ram_code_ver = tp->sw_ram_code_ver;
}

static void
rtl8125_acquire_phy_mcu_patch_key_lock(struct rtl8125_private *tp, u16 patchKey)
{
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8024);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, patchKey);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xB82E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0001);
}

static void
rtl8125_release_phy_mcu_patch_key_lock(struct rtl8125_private *tp)
{
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xB82E, BIT_0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8024);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
}

static void
rtl8125_set_phy_mcu_ram_code(struct rtl8125_private *tp, const u16 *ramcode, u16 codesize)
{
        u16 addr, val;

        if (unlikely(ramcode == NULL || codesize % 2))
                return;

        for (u16 i = 0; i < codesize; i += 2) {
                addr = ramcode[i];
                val = ramcode[i + 1];
                if (addr == 0xFFFF && val == 0xFFFF) {
                        return;
                }
                rtl8125_mdio_direct_write_phy_ocp(tp, addr, val);
        }
}

static void
rtl8125_enable_phy_disable_mode(struct rtl8125_private *tp)
{
        RTL_W8(tp, 0xF2, RTL_R8(tp, 0xF2) | BIT_5);
        dprintk("enable phy disable mode.\n");
}

static void
rtl8125_disable_phy_disable_mode(struct rtl8125_private *tp)
{
        RTL_W8(tp, 0xF2, RTL_R8(tp, 0xF2) & ~BIT_5);
        mdelay(1);
        dprintk("disable phy disable mode.\n");
}

static void
rtl8125_set_hw_phy_before_init_phy_mcu(struct rtl8125_private *tp)
{
        u16 PhyRegValue;

        if (tp->mcfg == CFG_METHOD_4) {
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xBF86, 0x9000);

                rtl8125_set_eth_phy_ocp_bit(tp, 0xC402, BIT_10);
                rtl8125_clear_eth_phy_ocp_bit(tp, 0xC402, BIT_10);

                PhyRegValue = rtl8125_mdio_direct_read_phy_ocp(tp, 0xBF86);
                PhyRegValue &= (BIT_1 | BIT_0);
                if (PhyRegValue != 0)
                        dprintk("PHY watch dog not clear, value = 0x%x \n", PhyRegValue);

                rtl8125_mdio_direct_write_phy_ocp(tp, 0xBD86, 0x1010);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xBD88, 0x1010);

                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBD4E, BIT_11 | BIT_10, BIT_11);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBF46,
                        BIT_11 | BIT_10 | BIT_9 | BIT_8, BIT_10 | BIT_9 | BIT_8);
        }
}

static void
rtl8125_real_set_phy_mcu_8125a_1(struct rtl8125_private *tp)
{
        rtl8125_acquire_phy_mcu_patch_key_lock(tp, 0x8600);

        rtl8125_set_eth_phy_ocp_bit(tp, 0xB820, BIT_7);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA016);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA012);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA014);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8013);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8021);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x802f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x803d);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8042);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8051);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8051);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa088);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0a50);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8008);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd014);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd1a3);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x401a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd707);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x40c2);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x60a6);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5f8b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0a86);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0a6c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8080);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd019);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd1a2);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x401a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd707);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x40c4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x60a6);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5f8b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0a86);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0a84);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd503);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8970);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c07);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0901);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce01);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xcf09);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd705);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xceff);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xaf0a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd504);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1213);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8401);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8580);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1253);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd064);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd181);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd704);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4018);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd504);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xc50f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd706);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x2c59);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x804d);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xc60f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf002);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xc605);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xae02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x10fd);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA026);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xffff);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA024);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xffff);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA022);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x10f4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA020);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1252);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA006);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1206);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA004);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0a78);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA002);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0a60);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0a4f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA008);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x3f00);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA016);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA012);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA014);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8066);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x807c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8089);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x808e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x80a0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x80b2);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x80c2);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd501);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce01);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x62db);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x655c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd73e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x60e9);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x614a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x61ab);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0501);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0304);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0503);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0304);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0505);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0304);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0509);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0304);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x653c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd73e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x60e9);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x614a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x61ab);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0503);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0304);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0502);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0304);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0506);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0304);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x050a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0304);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd73e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x60e9);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x614a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x61ab);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0505);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0304);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0506);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0304);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0504);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0304);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x050c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0304);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd73e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x60e9);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x614a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x61ab);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0509);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0304);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x050a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0304);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x050c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0304);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0508);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0304);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd501);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce01);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd73e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x60e9);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x614a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x61ab);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0501);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0321);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0502);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0321);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0504);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0321);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0508);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0321);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0346);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd501);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce01);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8208);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x609d);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa50f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x001a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0503);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x001a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x607d);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00ab);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00ab);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd501);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce01);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x60fd);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa50f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xaa0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x017b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0503);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0a05);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x017b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd501);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce01);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x60fd);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa50f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xaa0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x01e0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0503);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0a05);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x01e0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x60fd);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa50f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xaa0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0231);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0503);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0a05);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0231);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA08E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xffff);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA08C);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0221);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA08A);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x01ce);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA088);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0169);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA086);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00a6);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA084);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x000d);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA082);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0308);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA080);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x029f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA090);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x007f);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA016);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0020);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA012);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA014);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8017);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x801b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8029);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8054);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x805a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8064);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x80a7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x9430);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x9480);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb408);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd120);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd057);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x064b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xcb80);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x9906);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0567);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xcb94);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8190);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x82a0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x800a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8406);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa740);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8dff);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x07e4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa840);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0773);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xcb91);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4063);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd139);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf002);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd140);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd040);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb404);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x07dc);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa610);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa110);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa2a0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa404);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd704);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4045);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa180);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd704);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x405d);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa720);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0742);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x07ec);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5f74);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0742);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd702);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x7fb6);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8190);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x82a0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8404);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8610);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d01);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x07dc);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x064b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x07c0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5fa7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0481);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x94bc);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x870c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa190);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa00a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa280);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa404);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8220);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x078e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xcb92);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa840);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4063);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd140);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf002);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd150);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd040);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd703);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x60a0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x6121);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x61a2);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x6223);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf02f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0cf0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d10);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa740);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf00f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0cf0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d20);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa740);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf00a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0cf0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d30);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa740);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf005);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0cf0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d40);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa740);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x07e4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa610);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa008);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd704);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4046);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa002);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd704);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x405d);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa720);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0742);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x07f7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5f74);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0742);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd702);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x7fb5);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x800a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0cf0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x07e4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa740);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd701);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x3ad4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0537);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8610);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8840);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x064b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8301);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x800a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8190);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x82a0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8404);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa70c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x9402);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x890c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8840);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x064b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA10E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0642);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA10C);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0686);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA10A);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0788);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA108);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x047b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA106);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x065c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA104);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0769);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA102);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0565);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA100);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x06f9);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA110);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00ff);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xb87c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8530);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xb87e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xaf85);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x3caf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8593);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xaf85);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x9caf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x85a5);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf86);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd702);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5afb);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xe083);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xfb0c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x020d);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x021b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x10bf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x86d7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb7bf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x86da);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xfbe0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x83fc);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1b10);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf86);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xda02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf86);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xdd02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5afb);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xe083);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xfd0c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x020d);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x021b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x10bf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x86dd);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb7bf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x86e0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xfbe0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x83fe);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1b10);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf86);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xe002);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xaf2f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbd02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x2cac);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0286);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x65af);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x212b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x022c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x6002);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x86b6);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xaf21);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0cd1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x03bf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8710);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb7bf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x870d);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb7bf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8719);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb7bf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8716);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb7bf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x871f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb7bf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x871c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb7bf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8728);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb7bf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8725);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb7bf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8707);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xfbad);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x281c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd100);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0a02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1302);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x2202);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x2b02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xae1a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd101);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0a02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1302);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x2202);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x2b02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd101);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x3402);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x3102);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x3d02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x3a02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4302);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4002);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4c02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4902);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd100);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x2e02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x3702);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4602);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf87);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4f02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ab7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xaf35);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x7ff8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xfaef);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x69bf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x86e3);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xfbbf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x86fb);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb7bf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x86e6);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xfbbf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x86fe);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb7bf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x86e9);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xfbbf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8701);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb7bf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x86ec);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xfbbf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8704);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x025a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb7bf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x86ef);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0262);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x7cbf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x86f2);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0262);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x7cbf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x86f5);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0262);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x7cbf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x86f8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0262);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x7cef);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x96fe);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xfc04);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf8fa);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xef69);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf86);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xef02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x6273);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf86);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf202);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x6273);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf86);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf502);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x6273);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbf86);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf802);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x6273);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xef96);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xfefc);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0420);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb540);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x53b5);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4086);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb540);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb9b5);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x40c8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb03a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xc8b0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbac8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb13a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xc8b1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xba77);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbd26);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xffbd);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x2677);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbd28);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xffbd);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x2840);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbd26);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xc8bd);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x2640);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbd28);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xc8bd);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x28bb);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa430);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x98b0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1eba);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb01e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xdcb0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1e98);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb09e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbab0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x9edc);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb09e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x98b1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1eba);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb11e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xdcb1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1e98);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb19e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbab1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x9edc);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb19e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x11b0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1e22);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb01e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x33b0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1e11);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb09e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x22b0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x9e33);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb09e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x11b1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1e22);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb11e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x33b1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1e11);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb19e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x22b1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x9e33);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb19e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xb85e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x2f71);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xb860);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x20d9);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xb862);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x2109);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xb864);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x34e7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xb878);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x000f);

        rtl8125_clear_eth_phy_ocp_bit(tp, 0xB820, BIT_7);

        rtl8125_release_phy_mcu_patch_key_lock(tp);
}

static void
rtl8125_set_phy_mcu_8125a_1(struct rtl8125_private *tp)
{
        rtl8125_set_phy_mcu_patch_request(tp);
        rtl8125_real_set_phy_mcu_8125a_1(tp);
        rtl8125_clear_phy_mcu_patch_request(tp);
}

static void
rtl8125_real_set_phy_mcu_8125a_2(struct rtl8125_private *tp)
{
        rtl8125_acquire_phy_mcu_patch_key_lock(tp, 0x8601);

        rtl8125_set_eth_phy_ocp_bit(tp, 0xB820, BIT_7);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA016);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA012);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA014);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x808b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x808f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8093);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8097);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x809d);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x80a1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x80aa);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd718);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x607b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x40da);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf00e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x42da);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf01e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd718);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x615b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1456);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x14a4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x14bc);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd718);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5f2e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf01c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1456);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x14a4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x14bc);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd718);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5f2e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf024);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1456);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x14a4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x14bc);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd718);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5f2e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf02c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1456);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x14a4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x14bc);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd718);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5f2e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf034);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd719);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4118);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd504);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xac11);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd501);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce01);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa410);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4779);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd504);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xac0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xae01);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1444);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf034);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd719);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4118);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd504);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xac22);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd501);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce01);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa420);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4559);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd504);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xac0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xae01);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1444);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf023);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd719);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4118);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd504);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xac44);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd501);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce01);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa440);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4339);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd504);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xac0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xae01);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1444);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf012);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd719);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4118);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd504);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xac88);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd501);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce01);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa480);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xce00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4119);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd504);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xac0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xae01);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1444);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf001);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1456);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd718);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5fac);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xc48f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x141b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd504);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x121a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd0b4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd1bb);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0898);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd0b4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd1bb);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0a0e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd064);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd18a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0b7e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x401c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd501);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa804);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8804);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x053b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa301);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0648);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xc520);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa201);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd701);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x252d);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1646);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd708);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4006);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1646);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0308);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA026);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0307);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA024);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1645);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA022);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0647);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA020);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x053a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA006);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0b7c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA004);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0a0c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA002);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0896);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x11a1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA008);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xff00);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA016);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA012);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA014);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8015);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x801a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x801a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x801a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x801a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x801a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x801a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xad02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x02d7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00ed);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0509);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xc100);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x008f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA08E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xffff);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA08C);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xffff);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA08A);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xffff);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA088);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xffff);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA086);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xffff);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA084);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xffff);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA082);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x008d);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA080);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00eb);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA090);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0103);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA016);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0020);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA012);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA014);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8014);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8018);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8024);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8051);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8055);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8072);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x80dc);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xfffd);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xfffd);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8301);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x800a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8190);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x82a0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8404);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa70c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x9402);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x890c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8840);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa380);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x066e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xcb91);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4063);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd139);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf002);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd140);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd040);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb404);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x07e0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa610);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa110);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa2a0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa404);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd704);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4085);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa180);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa404);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8280);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd704);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x405d);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa720);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0743);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x07f0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5f74);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0743);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd702);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x7fb6);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8190);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x82a0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8404);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8610);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0c0f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d01);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x07e0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x066e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd158);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd04d);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x03d4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x94bc);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x870c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8380);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd10d);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd040);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x07c4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5fb4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa190);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa00a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa280);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa404);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa220);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd130);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd040);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x07c4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5fb4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xbb80);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd1c4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd074);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa301);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd704);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x604b);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa90c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0556);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xcb92);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4063);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd116);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf002);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd119);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd040);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd703);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x60a0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x6241);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x63e2);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x6583);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf054);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd701);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x611e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd701);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x40da);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0cf0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d10);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8740);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf02f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0cf0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d50);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa740);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf02a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd701);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x611e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd701);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x40da);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0cf0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d20);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8740);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf021);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0cf0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d60);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa740);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf01c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd701);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x611e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd701);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x40da);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0cf0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d30);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8740);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf013);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0cf0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d70);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa740);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf00e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd701);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x611e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd701);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x40da);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0cf0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d40);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8740);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf005);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0cf0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d80);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa740);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x07e8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa610);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd704);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x405d);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa720);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5ff4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa008);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd704);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4046);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa002);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0743);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x07fb);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd703);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x7f6f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x7f4e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x7f2d);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x7f0c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x800a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0cf0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0d00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x07e8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8010);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa740);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0743);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd702);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x7fb5);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd701);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x3ad4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0556);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8610);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x066e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd1f5);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xd049);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x01ec);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA10E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x01ea);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA10C);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x06a9);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA10A);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x078a);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA108);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x03d2);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA106);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x067f);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA104);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0665);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA102);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA100);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xA110);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00fc);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xb87c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8530);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xb87e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xaf85);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x3caf);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8545);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xaf85);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x45af);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8545);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xee82);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf900);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0103);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xaf03);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb7f8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xe0a6);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00e1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa601);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xef01);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x58f0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa080);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x37a1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8402);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xae16);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa185);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x02ae);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x11a1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8702);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xae0c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xa188);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x02ae);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x07a1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8902);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xae02);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xae1c);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xe0b4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x62e1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb463);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x6901);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xe4b4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x62e5);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb463);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xe0b4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x62e1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb463);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x6901);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xe4b4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x62e5);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xb463);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xfc04);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xb85e);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x03b3);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xb860);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xffff);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xb862);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xffff);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xb864);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xffff);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0xb878);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0001);

        rtl8125_clear_eth_phy_ocp_bit(tp, 0xB820, BIT_7);
        rtl8125_release_phy_mcu_patch_key_lock(tp);
}

static void
rtl8125_set_phy_mcu_8125a_2(struct rtl8125_private *tp)
{
        rtl8125_set_phy_mcu_patch_request(tp);
        rtl8125_real_set_phy_mcu_8125a_2(tp);
        rtl8125_clear_phy_mcu_patch_request(tp);
}

static const u16 phy_mcu_ram_code_8125b_1[] = {
        0xa436, 0x8024, 0xa438, 0x3700, 0xa436, 0xB82E, 0xa438, 0x0001,
        0xb820, 0x0090, 0xa436, 0xA016, 0xa438, 0x0000, 0xa436, 0xA012,
        0xa438, 0x0000, 0xa436, 0xA014, 0xa438, 0x1800, 0xa438, 0x8010,
        0xa438, 0x1800, 0xa438, 0x8025, 0xa438, 0x1800, 0xa438, 0x803a,
        0xa438, 0x1800, 0xa438, 0x8044, 0xa438, 0x1800, 0xa438, 0x8083,
        0xa438, 0x1800, 0xa438, 0x808d, 0xa438, 0x1800, 0xa438, 0x808d,
        0xa438, 0x1800, 0xa438, 0x808d, 0xa438, 0xd712, 0xa438, 0x4077,
        0xa438, 0xd71e, 0xa438, 0x4159, 0xa438, 0xd71e, 0xa438, 0x6099,
        0xa438, 0x7f44, 0xa438, 0x1800, 0xa438, 0x1a14, 0xa438, 0x9040,
        0xa438, 0x9201, 0xa438, 0x1800, 0xa438, 0x1b1a, 0xa438, 0xd71e,
        0xa438, 0x2425, 0xa438, 0x1a14, 0xa438, 0xd71f, 0xa438, 0x3ce5,
        0xa438, 0x1afb, 0xa438, 0x1800, 0xa438, 0x1b00, 0xa438, 0xd712,
        0xa438, 0x4077, 0xa438, 0xd71e, 0xa438, 0x4159, 0xa438, 0xd71e,
        0xa438, 0x60b9, 0xa438, 0x2421, 0xa438, 0x1c17, 0xa438, 0x1800,
        0xa438, 0x1a14, 0xa438, 0x9040, 0xa438, 0x1800, 0xa438, 0x1c2c,
        0xa438, 0xd71e, 0xa438, 0x2425, 0xa438, 0x1a14, 0xa438, 0xd71f,
        0xa438, 0x3ce5, 0xa438, 0x1c0f, 0xa438, 0x1800, 0xa438, 0x1c13,
        0xa438, 0xd702, 0xa438, 0xd501, 0xa438, 0x6072, 0xa438, 0x8401,
        0xa438, 0xf002, 0xa438, 0xa401, 0xa438, 0x1000, 0xa438, 0x146e,
        0xa438, 0x1800, 0xa438, 0x0b77, 0xa438, 0xd703, 0xa438, 0x665d,
        0xa438, 0x653e, 0xa438, 0x641f, 0xa438, 0xd700, 0xa438, 0x62c4,
        0xa438, 0x6185, 0xa438, 0x6066, 0xa438, 0x1800, 0xa438, 0x165a,
        0xa438, 0xc101, 0xa438, 0xcb00, 0xa438, 0x1000, 0xa438, 0x1945,
        0xa438, 0xd700, 0xa438, 0x7fa6, 0xa438, 0x1800, 0xa438, 0x807d,
        0xa438, 0xc102, 0xa438, 0xcb00, 0xa438, 0x1000, 0xa438, 0x1945,
        0xa438, 0xd700, 0xa438, 0x2569, 0xa438, 0x8058, 0xa438, 0x1800,
        0xa438, 0x807d, 0xa438, 0xc104, 0xa438, 0xcb00, 0xa438, 0x1000,
        0xa438, 0x1945, 0xa438, 0xd700, 0xa438, 0x7fa4, 0xa438, 0x1800,
        0xa438, 0x807d, 0xa438, 0xc120, 0xa438, 0xcb00, 0xa438, 0x1000,
        0xa438, 0x1945, 0xa438, 0xd703, 0xa438, 0x7fbf, 0xa438, 0x1800,
        0xa438, 0x807d, 0xa438, 0xc140, 0xa438, 0xcb00, 0xa438, 0x1000,
        0xa438, 0x1945, 0xa438, 0xd703, 0xa438, 0x7fbe, 0xa438, 0x1800,
        0xa438, 0x807d, 0xa438, 0xc180, 0xa438, 0xcb00, 0xa438, 0x1000,
        0xa438, 0x1945, 0xa438, 0xd703, 0xa438, 0x7fbd, 0xa438, 0xc100,
        0xa438, 0xcb00, 0xa438, 0xd708, 0xa438, 0x6018, 0xa438, 0x1800,
        0xa438, 0x165a, 0xa438, 0x1000, 0xa438, 0x14f6, 0xa438, 0xd014,
        0xa438, 0xd1e3, 0xa438, 0x1000, 0xa438, 0x1356, 0xa438, 0xd705,
        0xa438, 0x5fbe, 0xa438, 0x1800, 0xa438, 0x1559, 0xa436, 0xA026,
        0xa438, 0xffff, 0xa436, 0xA024, 0xa438, 0xffff, 0xa436, 0xA022,
        0xa438, 0xffff, 0xa436, 0xA020, 0xa438, 0x1557, 0xa436, 0xA006,
        0xa438, 0x1677, 0xa436, 0xA004, 0xa438, 0x0b75, 0xa436, 0xA002,
        0xa438, 0x1c17, 0xa436, 0xA000, 0xa438, 0x1b04, 0xa436, 0xA008,
        0xa438, 0x1f00, 0xa436, 0xA016, 0xa438, 0x0020, 0xa436, 0xA012,
        0xa438, 0x0000, 0xa436, 0xA014, 0xa438, 0x1800, 0xa438, 0x8010,
        0xa438, 0x1800, 0xa438, 0x817f, 0xa438, 0x1800, 0xa438, 0x82ab,
        0xa438, 0x1800, 0xa438, 0x83f8, 0xa438, 0x1800, 0xa438, 0x8444,
        0xa438, 0x1800, 0xa438, 0x8454, 0xa438, 0x1800, 0xa438, 0x8459,
        0xa438, 0x1800, 0xa438, 0x8465, 0xa438, 0xcb11, 0xa438, 0xa50c,
        0xa438, 0x8310, 0xa438, 0xd701, 0xa438, 0x4076, 0xa438, 0x0c03,
        0xa438, 0x0903, 0xa438, 0xd700, 0xa438, 0x6083, 0xa438, 0x0c1f,
        0xa438, 0x0d00, 0xa438, 0xf003, 0xa438, 0x0c1f, 0xa438, 0x0d00,
        0xa438, 0x1000, 0xa438, 0x0a7d, 0xa438, 0x1000, 0xa438, 0x0a4d,
        0xa438, 0xcb12, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f,
        0xa438, 0x5f84, 0xa438, 0xd102, 0xa438, 0xd040, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x5fb4, 0xa438, 0xd701,
        0xa438, 0x60f3, 0xa438, 0xd413, 0xa438, 0x1000, 0xa438, 0x0a37,
        0xa438, 0xd410, 0xa438, 0x1000, 0xa438, 0x0a37, 0xa438, 0xcb13,
        0xa438, 0xa108, 0xa438, 0x1000, 0xa438, 0x0a42, 0xa438, 0x8108,
        0xa438, 0xa00a, 0xa438, 0xa910, 0xa438, 0xa780, 0xa438, 0xd14a,
        0xa438, 0xd048, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd701,
        0xa438, 0x6255, 0xa438, 0xd700, 0xa438, 0x5f74, 0xa438, 0x6326,
        0xa438, 0xd702, 0xa438, 0x5f07, 0xa438, 0x800a, 0xa438, 0xa004,
        0xa438, 0x1000, 0xa438, 0x0a42, 0xa438, 0x8004, 0xa438, 0xa001,
        0xa438, 0x1000, 0xa438, 0x0a42, 0xa438, 0x8001, 0xa438, 0x0c03,
        0xa438, 0x0902, 0xa438, 0xffe2, 0xa438, 0x1000, 0xa438, 0x0a5e,
        0xa438, 0xd71f, 0xa438, 0x5fab, 0xa438, 0xba08, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x7f8b, 0xa438, 0x9a08,
        0xa438, 0x800a, 0xa438, 0xd702, 0xa438, 0x6535, 0xa438, 0xd40d,
        0xa438, 0x1000, 0xa438, 0x0a37, 0xa438, 0xcb14, 0xa438, 0xa004,
        0xa438, 0x1000, 0xa438, 0x0a42, 0xa438, 0x8004, 0xa438, 0xa001,
        0xa438, 0x1000, 0xa438, 0x0a42, 0xa438, 0x8001, 0xa438, 0xa00a,
        0xa438, 0xa780, 0xa438, 0xd14a, 0xa438, 0xd048, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x5fb4, 0xa438, 0x6206,
        0xa438, 0xd702, 0xa438, 0x5f47, 0xa438, 0x800a, 0xa438, 0xa004,
        0xa438, 0x1000, 0xa438, 0x0a42, 0xa438, 0x8004, 0xa438, 0xa001,
        0xa438, 0x1000, 0xa438, 0x0a42, 0xa438, 0x8001, 0xa438, 0x0c03,
        0xa438, 0x0902, 0xa438, 0x1800, 0xa438, 0x8064, 0xa438, 0x800a,
        0xa438, 0xd40e, 0xa438, 0x1000, 0xa438, 0x0a37, 0xa438, 0xb920,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x5fac,
        0xa438, 0x9920, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f,
        0xa438, 0x7f8c, 0xa438, 0xd701, 0xa438, 0x6073, 0xa438, 0xd701,
        0xa438, 0x4216, 0xa438, 0xa004, 0xa438, 0x1000, 0xa438, 0x0a42,
        0xa438, 0x8004, 0xa438, 0xa001, 0xa438, 0x1000, 0xa438, 0x0a42,
        0xa438, 0x8001, 0xa438, 0xd120, 0xa438, 0xd040, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x5fb4, 0xa438, 0x8504,
        0xa438, 0xcb21, 0xa438, 0xa301, 0xa438, 0x1000, 0xa438, 0x0a5e,
        0xa438, 0xd700, 0xa438, 0x5f9f, 0xa438, 0x8301, 0xa438, 0xd704,
        0xa438, 0x40e0, 0xa438, 0xd196, 0xa438, 0xd04d, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x5fb4, 0xa438, 0xcb22,
        0xa438, 0x1000, 0xa438, 0x0a6d, 0xa438, 0x0c03, 0xa438, 0x1502,
        0xa438, 0xa640, 0xa438, 0x9503, 0xa438, 0x8910, 0xa438, 0x8720,
        0xa438, 0xd700, 0xa438, 0x6083, 0xa438, 0x0c1f, 0xa438, 0x0d01,
        0xa438, 0xf003, 0xa438, 0x0c1f, 0xa438, 0x0d01, 0xa438, 0x1000,
        0xa438, 0x0a7d, 0xa438, 0x0c1f, 0xa438, 0x0f14, 0xa438, 0xcb23,
        0xa438, 0x8fc0, 0xa438, 0x1000, 0xa438, 0x0a25, 0xa438, 0xaf40,
        0xa438, 0x1000, 0xa438, 0x0a25, 0xa438, 0x0cc0, 0xa438, 0x0f80,
        0xa438, 0x1000, 0xa438, 0x0a25, 0xa438, 0xafc0, 0xa438, 0x1000,
        0xa438, 0x0a25, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd701,
        0xa438, 0x5dee, 0xa438, 0xcb24, 0xa438, 0x8f1f, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd701, 0xa438, 0x7f6e, 0xa438, 0xa111,
        0xa438, 0xa215, 0xa438, 0xa401, 0xa438, 0x8404, 0xa438, 0xa720,
        0xa438, 0xcb25, 0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0x8640,
        0xa438, 0x9503, 0xa438, 0x1000, 0xa438, 0x0b43, 0xa438, 0x1000,
        0xa438, 0x0b86, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xb920,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x5fac,
        0xa438, 0x9920, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f,
        0xa438, 0x7f8c, 0xa438, 0xcb26, 0xa438, 0x1000, 0xa438, 0x0a5e,
        0xa438, 0xd71f, 0xa438, 0x5f82, 0xa438, 0x8111, 0xa438, 0x8205,
        0xa438, 0x8404, 0xa438, 0xcb27, 0xa438, 0xd404, 0xa438, 0x1000,
        0xa438, 0x0a37, 0xa438, 0xd700, 0xa438, 0x6083, 0xa438, 0x0c1f,
        0xa438, 0x0d02, 0xa438, 0xf003, 0xa438, 0x0c1f, 0xa438, 0x0d02,
        0xa438, 0x1000, 0xa438, 0x0a7d, 0xa438, 0xa710, 0xa438, 0xa104,
        0xa438, 0x1000, 0xa438, 0x0a42, 0xa438, 0x8104, 0xa438, 0xa001,
        0xa438, 0x1000, 0xa438, 0x0a42, 0xa438, 0x8001, 0xa438, 0xa120,
        0xa438, 0xaa0f, 0xa438, 0x8110, 0xa438, 0xa284, 0xa438, 0xa404,
        0xa438, 0xa00a, 0xa438, 0xd193, 0xa438, 0xd046, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x5fb4, 0xa438, 0xcb28,
        0xa438, 0xa110, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd700,
        0xa438, 0x5fa8, 0xa438, 0x8110, 0xa438, 0x8284, 0xa438, 0xa404,
        0xa438, 0x800a, 0xa438, 0x8710, 0xa438, 0xb804, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x7f82, 0xa438, 0x9804,
        0xa438, 0xcb29, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f,
        0xa438, 0x5f85, 0xa438, 0xa710, 0xa438, 0xb820, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x7f65, 0xa438, 0x9820,
        0xa438, 0xcb2a, 0xa438, 0xa190, 0xa438, 0xa284, 0xa438, 0xa404,
        0xa438, 0xa00a, 0xa438, 0xd13d, 0xa438, 0xd04a, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x3444, 0xa438, 0x8149,
        0xa438, 0xa220, 0xa438, 0xd1a0, 0xa438, 0xd040, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x3444, 0xa438, 0x8151,
        0xa438, 0xd702, 0xa438, 0x5f51, 0xa438, 0xcb2f, 0xa438, 0xa302,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd708, 0xa438, 0x5f63,
        0xa438, 0xd411, 0xa438, 0x1000, 0xa438, 0x0a37, 0xa438, 0x8302,
        0xa438, 0xd409, 0xa438, 0x1000, 0xa438, 0x0a37, 0xa438, 0xb920,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x5fac,
        0xa438, 0x9920, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f,
        0xa438, 0x7f8c, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f,
        0xa438, 0x5fa3, 0xa438, 0x8190, 0xa438, 0x82a4, 0xa438, 0x8404,
        0xa438, 0x800a, 0xa438, 0xb808, 0xa438, 0x1000, 0xa438, 0x0a5e,
        0xa438, 0xd71f, 0xa438, 0x7fa3, 0xa438, 0x9808, 0xa438, 0x1800,
        0xa438, 0x0433, 0xa438, 0xcb15, 0xa438, 0xa508, 0xa438, 0xd700,
        0xa438, 0x6083, 0xa438, 0x0c1f, 0xa438, 0x0d01, 0xa438, 0xf003,
        0xa438, 0x0c1f, 0xa438, 0x0d01, 0xa438, 0x1000, 0xa438, 0x0a7d,
        0xa438, 0x1000, 0xa438, 0x0a4d, 0xa438, 0xa301, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x5f9f, 0xa438, 0x8301,
        0xa438, 0xd704, 0xa438, 0x40e0, 0xa438, 0xd115, 0xa438, 0xd04f,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x5fb4,
        0xa438, 0xd413, 0xa438, 0x1000, 0xa438, 0x0a37, 0xa438, 0xcb16,
        0xa438, 0x1000, 0xa438, 0x0a6d, 0xa438, 0x0c03, 0xa438, 0x1502,
        0xa438, 0xa640, 0xa438, 0x9503, 0xa438, 0x8720, 0xa438, 0xd17a,
        0xa438, 0xd04c, 0xa438, 0x0c1f, 0xa438, 0x0f14, 0xa438, 0xcb17,
        0xa438, 0x8fc0, 0xa438, 0x1000, 0xa438, 0x0a25, 0xa438, 0xaf40,
        0xa438, 0x1000, 0xa438, 0x0a25, 0xa438, 0x0cc0, 0xa438, 0x0f80,
        0xa438, 0x1000, 0xa438, 0x0a25, 0xa438, 0xafc0, 0xa438, 0x1000,
        0xa438, 0x0a25, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd701,
        0xa438, 0x61ce, 0xa438, 0xd700, 0xa438, 0x5db4, 0xa438, 0xcb18,
        0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0x8640, 0xa438, 0x9503,
        0xa438, 0xa720, 0xa438, 0x1000, 0xa438, 0x0b43, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xffd6, 0xa438, 0x8f1f, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd701, 0xa438, 0x7f8e, 0xa438, 0xa131,
        0xa438, 0xaa0f, 0xa438, 0xa2d5, 0xa438, 0xa407, 0xa438, 0xa720,
        0xa438, 0x8310, 0xa438, 0xa308, 0xa438, 0x8308, 0xa438, 0xcb19,
        0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0x8640, 0xa438, 0x9503,
        0xa438, 0x1000, 0xa438, 0x0b43, 0xa438, 0x1000, 0xa438, 0x0b86,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xb920, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x5fac, 0xa438, 0x9920,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x7f8c,
        0xa438, 0xcb1a, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f,
        0xa438, 0x5f82, 0xa438, 0x8111, 0xa438, 0x82c5, 0xa438, 0xa404,
        0xa438, 0x8402, 0xa438, 0xb804, 0xa438, 0x1000, 0xa438, 0x0a5e,
        0xa438, 0xd71f, 0xa438, 0x7f82, 0xa438, 0x9804, 0xa438, 0xcb1b,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x5f85,
        0xa438, 0xa710, 0xa438, 0xb820, 0xa438, 0x1000, 0xa438, 0x0a5e,
        0xa438, 0xd71f, 0xa438, 0x7f65, 0xa438, 0x9820, 0xa438, 0xcb1c,
        0xa438, 0xd700, 0xa438, 0x6083, 0xa438, 0x0c1f, 0xa438, 0x0d02,
        0xa438, 0xf003, 0xa438, 0x0c1f, 0xa438, 0x0d02, 0xa438, 0x1000,
        0xa438, 0x0a7d, 0xa438, 0xa110, 0xa438, 0xa284, 0xa438, 0xa404,
        0xa438, 0x8402, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd700,
        0xa438, 0x5fa8, 0xa438, 0xcb1d, 0xa438, 0xa180, 0xa438, 0xa402,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x5fa8,
        0xa438, 0xa220, 0xa438, 0xd1f5, 0xa438, 0xd049, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x3444, 0xa438, 0x8221,
        0xa438, 0xd702, 0xa438, 0x5f51, 0xa438, 0xb920, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x5fac, 0xa438, 0x9920,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x7f8c,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x5fa3,
        0xa438, 0xa504, 0xa438, 0xd700, 0xa438, 0x6083, 0xa438, 0x0c1f,
        0xa438, 0x0d00, 0xa438, 0xf003, 0xa438, 0x0c1f, 0xa438, 0x0d00,
        0xa438, 0x1000, 0xa438, 0x0a7d, 0xa438, 0xa00a, 0xa438, 0x8190,
        0xa438, 0x82a4, 0xa438, 0x8402, 0xa438, 0xa404, 0xa438, 0xb808,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x7fa3,
        0xa438, 0x9808, 0xa438, 0xcb2b, 0xa438, 0xcb2c, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x5f84, 0xa438, 0xd14a,
        0xa438, 0xd048, 0xa438, 0xa780, 0xa438, 0xcb2d, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x5f94, 0xa438, 0x6208,
        0xa438, 0xd702, 0xa438, 0x5f27, 0xa438, 0x800a, 0xa438, 0xa004,
        0xa438, 0x1000, 0xa438, 0x0a42, 0xa438, 0x8004, 0xa438, 0xa001,
        0xa438, 0x1000, 0xa438, 0x0a42, 0xa438, 0x8001, 0xa438, 0x0c03,
        0xa438, 0x0902, 0xa438, 0xa00a, 0xa438, 0xffe9, 0xa438, 0xcb2e,
        0xa438, 0xd700, 0xa438, 0x6083, 0xa438, 0x0c1f, 0xa438, 0x0d02,
        0xa438, 0xf003, 0xa438, 0x0c1f, 0xa438, 0x0d02, 0xa438, 0x1000,
        0xa438, 0x0a7d, 0xa438, 0xa190, 0xa438, 0xa284, 0xa438, 0xa406,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x5fa8,
        0xa438, 0xa220, 0xa438, 0xd1a0, 0xa438, 0xd040, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x3444, 0xa438, 0x827d,
        0xa438, 0xd702, 0xa438, 0x5f51, 0xa438, 0xcb2f, 0xa438, 0xa302,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd708, 0xa438, 0x5f63,
        0xa438, 0xd411, 0xa438, 0x1000, 0xa438, 0x0a37, 0xa438, 0x8302,
        0xa438, 0xd409, 0xa438, 0x1000, 0xa438, 0x0a37, 0xa438, 0xb920,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x5fac,
        0xa438, 0x9920, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f,
        0xa438, 0x7f8c, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f,
        0xa438, 0x5fa3, 0xa438, 0x8190, 0xa438, 0x82a4, 0xa438, 0x8406,
        0xa438, 0x800a, 0xa438, 0xb808, 0xa438, 0x1000, 0xa438, 0x0a5e,
        0xa438, 0xd71f, 0xa438, 0x7fa3, 0xa438, 0x9808, 0xa438, 0x1800,
        0xa438, 0x0433, 0xa438, 0xcb30, 0xa438, 0x8380, 0xa438, 0xcb31,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x5f86,
        0xa438, 0x9308, 0xa438, 0xb204, 0xa438, 0xb301, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd701, 0xa438, 0x5fa2, 0xa438, 0xb302,
        0xa438, 0x9204, 0xa438, 0xcb32, 0xa438, 0xd408, 0xa438, 0x1000,
        0xa438, 0x0a37, 0xa438, 0xd141, 0xa438, 0xd043, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x5fb4, 0xa438, 0xd704,
        0xa438, 0x4ccc, 0xa438, 0xd700, 0xa438, 0x4c81, 0xa438, 0xd702,
        0xa438, 0x609e, 0xa438, 0xd1e5, 0xa438, 0xd04d, 0xa438, 0xf003,
        0xa438, 0xd1e5, 0xa438, 0xd04d, 0xa438, 0x1000, 0xa438, 0x0a5e,
        0xa438, 0xd700, 0xa438, 0x5fb4, 0xa438, 0xd700, 0xa438, 0x6083,
        0xa438, 0x0c1f, 0xa438, 0x0d01, 0xa438, 0xf003, 0xa438, 0x0c1f,
        0xa438, 0x0d01, 0xa438, 0x1000, 0xa438, 0x0a7d, 0xa438, 0x8710,
        0xa438, 0xa108, 0xa438, 0x1000, 0xa438, 0x0a42, 0xa438, 0x8108,
        0xa438, 0xa203, 0xa438, 0x8120, 0xa438, 0x8a0f, 0xa438, 0xa111,
        0xa438, 0x8204, 0xa438, 0xa140, 0xa438, 0x1000, 0xa438, 0x0a42,
        0xa438, 0x8140, 0xa438, 0xd17a, 0xa438, 0xd04b, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x5fb4, 0xa438, 0xa204,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x5fa7,
        0xa438, 0xb920, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f,
        0xa438, 0x5fac, 0xa438, 0x9920, 0xa438, 0x1000, 0xa438, 0x0a5e,
        0xa438, 0xd71f, 0xa438, 0x7f8c, 0xa438, 0xd404, 0xa438, 0x1000,
        0xa438, 0x0a37, 0xa438, 0xd700, 0xa438, 0x6083, 0xa438, 0x0c1f,
        0xa438, 0x0d02, 0xa438, 0xf003, 0xa438, 0x0c1f, 0xa438, 0x0d02,
        0xa438, 0x1000, 0xa438, 0x0a7d, 0xa438, 0xa710, 0xa438, 0x8101,
        0xa438, 0x8201, 0xa438, 0xa104, 0xa438, 0x1000, 0xa438, 0x0a42,
        0xa438, 0x8104, 0xa438, 0xa120, 0xa438, 0xaa0f, 0xa438, 0x8110,
        0xa438, 0xa284, 0xa438, 0xa404, 0xa438, 0xa00a, 0xa438, 0xd193,
        0xa438, 0xd047, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd700,
        0xa438, 0x5fb4, 0xa438, 0xa110, 0xa438, 0x1000, 0xa438, 0x0a5e,
        0xa438, 0xd700, 0xa438, 0x5fa8, 0xa438, 0xa180, 0xa438, 0xd13d,
        0xa438, 0xd04a, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd700,
        0xa438, 0x5fb4, 0xa438, 0xf024, 0xa438, 0xa710, 0xa438, 0xa00a,
        0xa438, 0x8190, 0xa438, 0x8204, 0xa438, 0xa280, 0xa438, 0xa404,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x5fa7,
        0xa438, 0x8710, 0xa438, 0xb920, 0xa438, 0x1000, 0xa438, 0x0a5e,
        0xa438, 0xd71f, 0xa438, 0x5fac, 0xa438, 0x9920, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x7f8c, 0xa438, 0x800a,
        0xa438, 0x8190, 0xa438, 0x8284, 0xa438, 0x8406, 0xa438, 0xd700,
        0xa438, 0x4121, 0xa438, 0xd701, 0xa438, 0x60f3, 0xa438, 0xd1e5,
        0xa438, 0xd04d, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd700,
        0xa438, 0x5fb4, 0xa438, 0x8710, 0xa438, 0xa00a, 0xa438, 0x8190,
        0xa438, 0x8204, 0xa438, 0xa280, 0xa438, 0xa404, 0xa438, 0xb920,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x5fac,
        0xa438, 0x9920, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f,
        0xa438, 0x7f8c, 0xa438, 0xcb33, 0xa438, 0x1000, 0xa438, 0x0a5e,
        0xa438, 0xd71f, 0xa438, 0x5f85, 0xa438, 0xa710, 0xa438, 0xb820,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd71f, 0xa438, 0x7f65,
        0xa438, 0x9820, 0xa438, 0xcb34, 0xa438, 0xa00a, 0xa438, 0xa190,
        0xa438, 0xa284, 0xa438, 0xa404, 0xa438, 0x1000, 0xa438, 0x0a5e,
        0xa438, 0xd700, 0xa438, 0x5fa9, 0xa438, 0xd701, 0xa438, 0x6853,
        0xa438, 0xd700, 0xa438, 0x6083, 0xa438, 0x0c1f, 0xa438, 0x0d00,
        0xa438, 0xf003, 0xa438, 0x0c1f, 0xa438, 0x0d00, 0xa438, 0x1000,
        0xa438, 0x0a7d, 0xa438, 0x8190, 0xa438, 0x8284, 0xa438, 0xcb35,
        0xa438, 0xd407, 0xa438, 0x1000, 0xa438, 0x0a37, 0xa438, 0x8110,
        0xa438, 0x8204, 0xa438, 0xa280, 0xa438, 0xa00a, 0xa438, 0xd704,
        0xa438, 0x4215, 0xa438, 0xa304, 0xa438, 0x1000, 0xa438, 0x0a5e,
        0xa438, 0xd700, 0xa438, 0x5fb8, 0xa438, 0xd1c3, 0xa438, 0xd043,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x5fb4,
        0xa438, 0x8304, 0xa438, 0xd700, 0xa438, 0x4109, 0xa438, 0xf01e,
        0xa438, 0xcb36, 0xa438, 0xd412, 0xa438, 0x1000, 0xa438, 0x0a37,
        0xa438, 0xd700, 0xa438, 0x6309, 0xa438, 0xd702, 0xa438, 0x42c7,
        0xa438, 0x800a, 0xa438, 0x8180, 0xa438, 0x8280, 0xa438, 0x8404,
        0xa438, 0xa004, 0xa438, 0x1000, 0xa438, 0x0a42, 0xa438, 0x8004,
        0xa438, 0xa001, 0xa438, 0x1000, 0xa438, 0x0a42, 0xa438, 0x8001,
        0xa438, 0x0c03, 0xa438, 0x0902, 0xa438, 0xa00a, 0xa438, 0xd14a,
        0xa438, 0xd048, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd700,
        0xa438, 0x5fb4, 0xa438, 0xd700, 0xa438, 0x6083, 0xa438, 0x0c1f,
        0xa438, 0x0d02, 0xa438, 0xf003, 0xa438, 0x0c1f, 0xa438, 0x0d02,
        0xa438, 0x1000, 0xa438, 0x0a7d, 0xa438, 0xcc55, 0xa438, 0xcb37,
        0xa438, 0xa00a, 0xa438, 0xa190, 0xa438, 0xa2a4, 0xa438, 0xa404,
        0xa438, 0xd700, 0xa438, 0x6041, 0xa438, 0xa402, 0xa438, 0xd13d,
        0xa438, 0xd04a, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd700,
        0xa438, 0x5fb4, 0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd700,
        0xa438, 0x5fa9, 0xa438, 0xd702, 0xa438, 0x5f71, 0xa438, 0xcb38,
        0xa438, 0x8224, 0xa438, 0xa288, 0xa438, 0x8180, 0xa438, 0xa110,
        0xa438, 0xa404, 0xa438, 0x800a, 0xa438, 0xd700, 0xa438, 0x6041,
        0xa438, 0x8402, 0xa438, 0xd415, 0xa438, 0x1000, 0xa438, 0x0a37,
        0xa438, 0xd13d, 0xa438, 0xd04a, 0xa438, 0x1000, 0xa438, 0x0a5e,
        0xa438, 0xd700, 0xa438, 0x5fb4, 0xa438, 0xcb39, 0xa438, 0xa00a,
        0xa438, 0xa190, 0xa438, 0xa2a0, 0xa438, 0xa404, 0xa438, 0xd700,
        0xa438, 0x6041, 0xa438, 0xa402, 0xa438, 0xd17a, 0xa438, 0xd047,
        0xa438, 0x1000, 0xa438, 0x0a5e, 0xa438, 0xd700, 0xa438, 0x5fb4,
        0xa438, 0x1800, 0xa438, 0x0560, 0xa438, 0xa111, 0xa438, 0x0000,
        0xa438, 0x0000, 0xa438, 0x0000, 0xa438, 0x0000, 0xa438, 0xd3f5,
        0xa438, 0xd219, 0xa438, 0x1000, 0xa438, 0x0c31, 0xa438, 0xd708,
        0xa438, 0x5fa5, 0xa438, 0xa215, 0xa438, 0xd30e, 0xa438, 0xd21a,
        0xa438, 0x1000, 0xa438, 0x0c31, 0xa438, 0xd708, 0xa438, 0x63e9,
        0xa438, 0xd708, 0xa438, 0x5f65, 0xa438, 0xd708, 0xa438, 0x7f36,
        0xa438, 0xa004, 0xa438, 0x1000, 0xa438, 0x0c35, 0xa438, 0x8004,
        0xa438, 0xa001, 0xa438, 0x1000, 0xa438, 0x0c35, 0xa438, 0x8001,
        0xa438, 0xd708, 0xa438, 0x4098, 0xa438, 0xd102, 0xa438, 0x9401,
        0xa438, 0xf003, 0xa438, 0xd103, 0xa438, 0xb401, 0xa438, 0x1000,
        0xa438, 0x0c27, 0xa438, 0xa108, 0xa438, 0x1000, 0xa438, 0x0c35,
        0xa438, 0x8108, 0xa438, 0x8110, 0xa438, 0x8294, 0xa438, 0xa202,
        0xa438, 0x1800, 0xa438, 0x0bdb, 0xa438, 0xd39c, 0xa438, 0xd210,
        0xa438, 0x1000, 0xa438, 0x0c31, 0xa438, 0xd708, 0xa438, 0x5fa5,
        0xa438, 0xd39c, 0xa438, 0xd210, 0xa438, 0x1000, 0xa438, 0x0c31,
        0xa438, 0xd708, 0xa438, 0x5fa5, 0xa438, 0x1000, 0xa438, 0x0c31,
        0xa438, 0xd708, 0xa438, 0x29b5, 0xa438, 0x840e, 0xa438, 0xd708,
        0xa438, 0x5f4a, 0xa438, 0x0c1f, 0xa438, 0x1014, 0xa438, 0x1000,
        0xa438, 0x0c31, 0xa438, 0xd709, 0xa438, 0x7fa4, 0xa438, 0x901f,
        0xa438, 0x1800, 0xa438, 0x0c23, 0xa438, 0xcb43, 0xa438, 0xa508,
        0xa438, 0xd701, 0xa438, 0x3699, 0xa438, 0x844a, 0xa438, 0xa504,
        0xa438, 0xa190, 0xa438, 0xa2a0, 0xa438, 0xa404, 0xa438, 0xa00a,
        0xa438, 0xd700, 0xa438, 0x2109, 0xa438, 0x05ea, 0xa438, 0xa402,
        0xa438, 0x1800, 0xa438, 0x05ea, 0xa438, 0xcb90, 0xa438, 0x0cf0,
        0xa438, 0x0ca0, 0xa438, 0x1800, 0xa438, 0x06db, 0xa438, 0xd1ff,
        0xa438, 0xd052, 0xa438, 0xa508, 0xa438, 0x8718, 0xa438, 0xa00a,
        0xa438, 0xa190, 0xa438, 0xa2a0, 0xa438, 0xa404, 0xa438, 0x0cf0,
        0xa438, 0x0c50, 0xa438, 0x1800, 0xa438, 0x09ef, 0xa438, 0x1000,
        0xa438, 0x0a5e, 0xa438, 0xd704, 0xa438, 0x2e70, 0xa438, 0x06da,
        0xa438, 0xd700, 0xa438, 0x5f55, 0xa438, 0xa90c, 0xa438, 0x1800,
        0xa438, 0x0645, 0xa436, 0xA10E, 0xa438, 0x0644, 0xa436, 0xA10C,
        0xa438, 0x09e9, 0xa436, 0xA10A, 0xa438, 0x06da, 0xa436, 0xA108,
        0xa438, 0x05e1, 0xa436, 0xA106, 0xa438, 0x0be4, 0xa436, 0xA104,
        0xa438, 0x0435, 0xa436, 0xA102, 0xa438, 0x0141, 0xa436, 0xA100,
        0xa438, 0x026d, 0xa436, 0xA110, 0xa438, 0x00ff, 0xa436, 0xb87c,
        0xa438, 0x85fe, 0xa436, 0xb87e, 0xa438, 0xaf86, 0xa438, 0x16af,
        0xa438, 0x8699, 0xa438, 0xaf86, 0xa438, 0xe5af, 0xa438, 0x86f9,
        0xa438, 0xaf87, 0xa438, 0x7aaf, 0xa438, 0x883a, 0xa438, 0xaf88,
        0xa438, 0x58af, 0xa438, 0x8b6c, 0xa438, 0xd48b, 0xa438, 0x7c02,
        0xa438, 0x8644, 0xa438, 0x2c00, 0xa438, 0x503c, 0xa438, 0xffd6,
        0xa438, 0xac27, 0xa438, 0x18e1, 0xa438, 0x82fe, 0xa438, 0xad28,
        0xa438, 0x0cd4, 0xa438, 0x8b84, 0xa438, 0x0286, 0xa438, 0x442c,
        0xa438, 0x003c, 0xa438, 0xac27, 0xa438, 0x06ee, 0xa438, 0x8299,
        0xa438, 0x01ae, 0xa438, 0x04ee, 0xa438, 0x8299, 0xa438, 0x00af,
        0xa438, 0x23dc, 0xa438, 0xf9fa, 0xa438, 0xcefa, 0xa438, 0xfbef,
        0xa438, 0x79fb, 0xa438, 0xc4bf, 0xa438, 0x8b76, 0xa438, 0x026c,
        0xa438, 0x6dac, 0xa438, 0x2804, 0xa438, 0xd203, 0xa438, 0xae02,
        0xa438, 0xd201, 0xa438, 0xbdd8, 0xa438, 0x19d9, 0xa438, 0xef94,
        0xa438, 0x026c, 0xa438, 0x6d78, 0xa438, 0x03ef, 0xa438, 0x648a,
        0xa438, 0x0002, 0xa438, 0xbdd8, 0xa438, 0x19d9, 0xa438, 0xef94,
        0xa438, 0x026c, 0xa438, 0x6d78, 0xa438, 0x03ef, 0xa438, 0x7402,
        0xa438, 0x72cd, 0xa438, 0xac50, 0xa438, 0x02ef, 0xa438, 0x643a,
        0xa438, 0x019f, 0xa438, 0xe4ef, 0xa438, 0x4678, 0xa438, 0x03ac,
        0xa438, 0x2002, 0xa438, 0xae02, 0xa438, 0xd0ff, 0xa438, 0xffef,
        0xa438, 0x97ff, 0xa438, 0xfec6, 0xa438, 0xfefd, 0xa438, 0x041f,
        0xa438, 0x771f, 0xa438, 0x221c, 0xa438, 0x450d, 0xa438, 0x481f,
        0xa438, 0x00ac, 0xa438, 0x7f04, 0xa438, 0x1a94, 0xa438, 0xae08,
        0xa438, 0x1a94, 0xa438, 0xac7f, 0xa438, 0x03d7, 0xa438, 0x0100,
        0xa438, 0xef46, 0xa438, 0x0d48, 0xa438, 0x1f00, 0xa438, 0x1c45,
        0xa438, 0xef69, 0xa438, 0xef57, 0xa438, 0xef74, 0xa438, 0x0272,
        0xa438, 0xe8a7, 0xa438, 0xffff, 0xa438, 0x0d1a, 0xa438, 0x941b,
        0xa438, 0x979e, 0xa438, 0x072d, 0xa438, 0x0100, 0xa438, 0x1a64,
        0xa438, 0xef76, 0xa438, 0xef97, 0xa438, 0x0d98, 0xa438, 0xd400,
        0xa438, 0xff1d, 0xa438, 0x941a, 0xa438, 0x89cf, 0xa438, 0x1a75,
        0xa438, 0xaf74, 0xa438, 0xf9bf, 0xa438, 0x8b79, 0xa438, 0x026c,
        0xa438, 0x6da1, 0xa438, 0x0005, 0xa438, 0xe180, 0xa438, 0xa0ae,
        0xa438, 0x03e1, 0xa438, 0x80a1, 0xa438, 0xaf26, 0xa438, 0x9aac,
        0xa438, 0x284d, 0xa438, 0xe08f, 0xa438, 0xffef, 0xa438, 0x10c0,
        0xa438, 0xe08f, 0xa438, 0xfe10, 0xa438, 0x1b08, 0xa438, 0xa000,
        0xa438, 0x04c8, 0xa438, 0xaf40, 0xa438, 0x67c8, 0xa438, 0xbf8b,
        0xa438, 0x8c02, 0xa438, 0x6c4e, 0xa438, 0xc4bf, 0xa438, 0x8b8f,
        0xa438, 0x026c, 0xa438, 0x6def, 0xa438, 0x74e0, 0xa438, 0x830c,
        0xa438, 0xad20, 0xa438, 0x0302, 0xa438, 0x74ac, 0xa438, 0xccef,
        0xa438, 0x971b, 0xa438, 0x76ad, 0xa438, 0x5f02, 0xa438, 0xae13,
        0xa438, 0xef69, 0xa438, 0xef30, 0xa438, 0x1b32, 0xa438, 0xc4ef,
        0xa438, 0x46e4, 0xa438, 0x8ffb, 0xa438, 0xe58f, 0xa438, 0xfce7,
        0xa438, 0x8ffd, 0xa438, 0xcc10, 0xa438, 0x11ae, 0xa438, 0xb8d1,
        0xa438, 0x00a1, 0xa438, 0x1f03, 0xa438, 0xaf40, 0xa438, 0x4fbf,
        0xa438, 0x8b8c, 0xa438, 0x026c, 0xa438, 0x4ec4, 0xa438, 0xbf8b,
        0xa438, 0x8f02, 0xa438, 0x6c6d, 0xa438, 0xef74, 0xa438, 0xe083,
        0xa438, 0x0cad, 0xa438, 0x2003, 0xa438, 0x0274, 0xa438, 0xaccc,
        0xa438, 0xef97, 0xa438, 0x1b76, 0xa438, 0xad5f, 0xa438, 0x02ae,
        0xa438, 0x04ef, 0xa438, 0x69ef, 0xa438, 0x3111, 0xa438, 0xaed1,
        0xa438, 0x0287, 0xa438, 0x80af, 0xa438, 0x2293, 0xa438, 0xf8f9,
        0xa438, 0xfafb, 0xa438, 0xef59, 0xa438, 0xe080, 0xa438, 0x13ad,
        0xa438, 0x252f, 0xa438, 0xbf88, 0xa438, 0x2802, 0xa438, 0x6c6d,
        0xa438, 0xef64, 0xa438, 0x1f44, 0xa438, 0xe18f, 0xa438, 0xb91b,
        0xa438, 0x64ad, 0xa438, 0x4f1d, 0xa438, 0xd688, 0xa438, 0x2bd7,
        0xa438, 0x882e, 0xa438, 0x0274, 0xa438, 0x73ad, 0xa438, 0x5008,
        0xa438, 0xbf88, 0xa438, 0x3102, 0xa438, 0x737c, 0xa438, 0xae03,
        0xa438, 0x0287, 0xa438, 0xd0bf, 0xa438, 0x882b, 0xa438, 0x0273,
        0xa438, 0x73e0, 0xa438, 0x824c, 0xa438, 0xf621, 0xa438, 0xe482,
        0xa438, 0x4cbf, 0xa438, 0x8834, 0xa438, 0x0273, 0xa438, 0x7cef,
        0xa438, 0x95ff, 0xa438, 0xfefd, 0xa438, 0xfc04, 0xa438, 0xf8f9,
        0xa438, 0xfafb, 0xa438, 0xef79, 0xa438, 0xbf88, 0xa438, 0x1f02,
        0xa438, 0x737c, 0xa438, 0x1f22, 0xa438, 0xac32, 0xa438, 0x31ef,
        0xa438, 0x12bf, 0xa438, 0x8822, 0xa438, 0x026c, 0xa438, 0x4ed6,
        0xa438, 0x8fba, 0xa438, 0x1f33, 0xa438, 0xac3c, 0xa438, 0x1eef,
        0xa438, 0x13bf, 0xa438, 0x8837, 0xa438, 0x026c, 0xa438, 0x4eef,
        0xa438, 0x96d8, 0xa438, 0x19d9, 0xa438, 0xbf88, 0xa438, 0x2502,
        0xa438, 0x6c4e, 0xa438, 0xbf88, 0xa438, 0x2502, 0xa438, 0x6c4e,
        0xa438, 0x1616, 0xa438, 0x13ae, 0xa438, 0xdf12, 0xa438, 0xaecc,
        0xa438, 0xbf88, 0xa438, 0x1f02, 0xa438, 0x7373, 0xa438, 0xef97,
        0xa438, 0xfffe, 0xa438, 0xfdfc, 0xa438, 0x0466, 0xa438, 0xac88,
        0xa438, 0x54ac, 0xa438, 0x88f0, 0xa438, 0xac8a, 0xa438, 0x92ac,
        0xa438, 0xbadd, 0xa438, 0xac6c, 0xa438, 0xeeac, 0xa438, 0x6cff,
        0xa438, 0xad02, 0xa438, 0x99ac, 0xa438, 0x0030, 0xa438, 0xac88,
        0xa438, 0xd4c3, 0xa438, 0x5000, 0xa438, 0x0000, 0xa438, 0x0000,
        0xa438, 0x0000, 0xa438, 0x0000, 0xa438, 0x0000, 0xa438, 0x0000,
        0xa438, 0x0000, 0xa438, 0x0000, 0xa438, 0x00b4, 0xa438, 0xecee,
        0xa438, 0x8298, 0xa438, 0x00af, 0xa438, 0x1412, 0xa438, 0xf8bf,
        0xa438, 0x8b5d, 0xa438, 0x026c, 0xa438, 0x6d58, 0xa438, 0x03e1,
        0xa438, 0x8fb8, 0xa438, 0x2901, 0xa438, 0xe58f, 0xa438, 0xb8a0,
        0xa438, 0x0049, 0xa438, 0xef47, 0xa438, 0xe483, 0xa438, 0x02e5,
        0xa438, 0x8303, 0xa438, 0xbfc2, 0xa438, 0x5f1a, 0xa438, 0x95f7,
        0xa438, 0x05ee, 0xa438, 0xffd2, 0xa438, 0x00d8, 0xa438, 0xf605,
        0xa438, 0x1f11, 0xa438, 0xef60, 0xa438, 0xbf8b, 0xa438, 0x3002,
        0xa438, 0x6c4e, 0xa438, 0xbf8b, 0xa438, 0x3302, 0xa438, 0x6c6d,
        0xa438, 0xf728, 0xa438, 0xbf8b, 0xa438, 0x3302, 0xa438, 0x6c4e,
        0xa438, 0xf628, 0xa438, 0xbf8b, 0xa438, 0x3302, 0xa438, 0x6c4e,
        0xa438, 0x0c64, 0xa438, 0xef46, 0xa438, 0xbf8b, 0xa438, 0x6002,
        0xa438, 0x6c4e, 0xa438, 0x0289, 0xa438, 0x9902, 0xa438, 0x3920,
        0xa438, 0xaf89, 0xa438, 0x96a0, 0xa438, 0x0149, 0xa438, 0xef47,
        0xa438, 0xe483, 0xa438, 0x04e5, 0xa438, 0x8305, 0xa438, 0xbfc2,
        0xa438, 0x5f1a, 0xa438, 0x95f7, 0xa438, 0x05ee, 0xa438, 0xffd2,
        0xa438, 0x00d8, 0xa438, 0xf605, 0xa438, 0x1f11, 0xa438, 0xef60,
        0xa438, 0xbf8b, 0xa438, 0x3002, 0xa438, 0x6c4e, 0xa438, 0xbf8b,
        0xa438, 0x3302, 0xa438, 0x6c6d, 0xa438, 0xf729, 0xa438, 0xbf8b,
        0xa438, 0x3302, 0xa438, 0x6c4e, 0xa438, 0xf629, 0xa438, 0xbf8b,
        0xa438, 0x3302, 0xa438, 0x6c4e, 0xa438, 0x0c64, 0xa438, 0xef46,
        0xa438, 0xbf8b, 0xa438, 0x6302, 0xa438, 0x6c4e, 0xa438, 0x0289,
        0xa438, 0x9902, 0xa438, 0x3920, 0xa438, 0xaf89, 0xa438, 0x96a0,
        0xa438, 0x0249, 0xa438, 0xef47, 0xa438, 0xe483, 0xa438, 0x06e5,
        0xa438, 0x8307, 0xa438, 0xbfc2, 0xa438, 0x5f1a, 0xa438, 0x95f7,
        0xa438, 0x05ee, 0xa438, 0xffd2, 0xa438, 0x00d8, 0xa438, 0xf605,
        0xa438, 0x1f11, 0xa438, 0xef60, 0xa438, 0xbf8b, 0xa438, 0x3002,
        0xa438, 0x6c4e, 0xa438, 0xbf8b, 0xa438, 0x3302, 0xa438, 0x6c6d,
        0xa438, 0xf72a, 0xa438, 0xbf8b, 0xa438, 0x3302, 0xa438, 0x6c4e,
        0xa438, 0xf62a, 0xa438, 0xbf8b, 0xa438, 0x3302, 0xa438, 0x6c4e,
        0xa438, 0x0c64, 0xa438, 0xef46, 0xa438, 0xbf8b, 0xa438, 0x6602,
        0xa438, 0x6c4e, 0xa438, 0x0289, 0xa438, 0x9902, 0xa438, 0x3920,
        0xa438, 0xaf89, 0xa438, 0x96ef, 0xa438, 0x47e4, 0xa438, 0x8308,
        0xa438, 0xe583, 0xa438, 0x09bf, 0xa438, 0xc25f, 0xa438, 0x1a95,
        0xa438, 0xf705, 0xa438, 0xeeff, 0xa438, 0xd200, 0xa438, 0xd8f6,
        0xa438, 0x051f, 0xa438, 0x11ef, 0xa438, 0x60bf, 0xa438, 0x8b30,
        0xa438, 0x026c, 0xa438, 0x4ebf, 0xa438, 0x8b33, 0xa438, 0x026c,
        0xa438, 0x6df7, 0xa438, 0x2bbf, 0xa438, 0x8b33, 0xa438, 0x026c,
        0xa438, 0x4ef6, 0xa438, 0x2bbf, 0xa438, 0x8b33, 0xa438, 0x026c,
        0xa438, 0x4e0c, 0xa438, 0x64ef, 0xa438, 0x46bf, 0xa438, 0x8b69,
        0xa438, 0x026c, 0xa438, 0x4e02, 0xa438, 0x8999, 0xa438, 0x0239,
        0xa438, 0x20af, 0xa438, 0x8996, 0xa438, 0xaf39, 0xa438, 0x1ef8,
        0xa438, 0xf9fa, 0xa438, 0xe08f, 0xa438, 0xb838, 0xa438, 0x02ad,
        0xa438, 0x2702, 0xa438, 0xae03, 0xa438, 0xaf8b, 0xa438, 0x201f,
        0xa438, 0x66ef, 0xa438, 0x65bf, 0xa438, 0xc21f, 0xa438, 0x1a96,
        0xa438, 0xf705, 0xa438, 0xeeff, 0xa438, 0xd200, 0xa438, 0xdaf6,
        0xa438, 0x05bf, 0xa438, 0xc22f, 0xa438, 0x1a96, 0xa438, 0xf705,
        0xa438, 0xeeff, 0xa438, 0xd200, 0xa438, 0xdbf6, 0xa438, 0x05ef,
        0xa438, 0x021f, 0xa438, 0x110d, 0xa438, 0x42bf, 0xa438, 0x8b3c,
        0xa438, 0x026c, 0xa438, 0x4eef, 0xa438, 0x021b, 0xa438, 0x031f,
        0xa438, 0x110d, 0xa438, 0x42bf, 0xa438, 0x8b36, 0xa438, 0x026c,
        0xa438, 0x4eef, 0xa438, 0x021a, 0xa438, 0x031f, 0xa438, 0x110d,
        0xa438, 0x42bf, 0xa438, 0x8b39, 0xa438, 0x026c, 0xa438, 0x4ebf,
        0xa438, 0xc23f, 0xa438, 0x1a96, 0xa438, 0xf705, 0xa438, 0xeeff,
        0xa438, 0xd200, 0xa438, 0xdaf6, 0xa438, 0x05bf, 0xa438, 0xc24f,
        0xa438, 0x1a96, 0xa438, 0xf705, 0xa438, 0xeeff, 0xa438, 0xd200,
        0xa438, 0xdbf6, 0xa438, 0x05ef, 0xa438, 0x021f, 0xa438, 0x110d,
        0xa438, 0x42bf, 0xa438, 0x8b45, 0xa438, 0x026c, 0xa438, 0x4eef,
        0xa438, 0x021b, 0xa438, 0x031f, 0xa438, 0x110d, 0xa438, 0x42bf,
        0xa438, 0x8b3f, 0xa438, 0x026c, 0xa438, 0x4eef, 0xa438, 0x021a,
        0xa438, 0x031f, 0xa438, 0x110d, 0xa438, 0x42bf, 0xa438, 0x8b42,
        0xa438, 0x026c, 0xa438, 0x4eef, 0xa438, 0x56d0, 0xa438, 0x201f,
        0xa438, 0x11bf, 0xa438, 0x8b4e, 0xa438, 0x026c, 0xa438, 0x4ebf,
        0xa438, 0x8b48, 0xa438, 0x026c, 0xa438, 0x4ebf, 0xa438, 0x8b4b,
        0xa438, 0x026c, 0xa438, 0x4ee1, 0xa438, 0x8578, 0xa438, 0xef03,
        0xa438, 0x480a, 0xa438, 0x2805, 0xa438, 0xef20, 0xa438, 0x1b01,
        0xa438, 0xad27, 0xa438, 0x3f1f, 0xa438, 0x44e0, 0xa438, 0x8560,
        0xa438, 0xe185, 0xa438, 0x61bf, 0xa438, 0x8b51, 0xa438, 0x026c,
        0xa438, 0x4ee0, 0xa438, 0x8566, 0xa438, 0xe185, 0xa438, 0x67bf,
        0xa438, 0x8b54, 0xa438, 0x026c, 0xa438, 0x4ee0, 0xa438, 0x856c,
        0xa438, 0xe185, 0xa438, 0x6dbf, 0xa438, 0x8b57, 0xa438, 0x026c,
        0xa438, 0x4ee0, 0xa438, 0x8572, 0xa438, 0xe185, 0xa438, 0x73bf,
        0xa438, 0x8b5a, 0xa438, 0x026c, 0xa438, 0x4ee1, 0xa438, 0x8fb8,
        0xa438, 0x5900, 0xa438, 0xf728, 0xa438, 0xe58f, 0xa438, 0xb8af,
        0xa438, 0x8b2c, 0xa438, 0xe185, 0xa438, 0x791b, 0xa438, 0x21ad,
        0xa438, 0x373e, 0xa438, 0x1f44, 0xa438, 0xe085, 0xa438, 0x62e1,
        0xa438, 0x8563, 0xa438, 0xbf8b, 0xa438, 0x5102, 0xa438, 0x6c4e,
        0xa438, 0xe085, 0xa438, 0x68e1, 0xa438, 0x8569, 0xa438, 0xbf8b,
        0xa438, 0x5402, 0xa438, 0x6c4e, 0xa438, 0xe085, 0xa438, 0x6ee1,
        0xa438, 0x856f, 0xa438, 0xbf8b, 0xa438, 0x5702, 0xa438, 0x6c4e,
        0xa438, 0xe085, 0xa438, 0x74e1, 0xa438, 0x8575, 0xa438, 0xbf8b,
        0xa438, 0x5a02, 0xa438, 0x6c4e, 0xa438, 0xe18f, 0xa438, 0xb859,
        0xa438, 0x00f7, 0xa438, 0x28e5, 0xa438, 0x8fb8, 0xa438, 0xae4a,
        0xa438, 0x1f44, 0xa438, 0xe085, 0xa438, 0x64e1, 0xa438, 0x8565,
        0xa438, 0xbf8b, 0xa438, 0x5102, 0xa438, 0x6c4e, 0xa438, 0xe085,
        0xa438, 0x6ae1, 0xa438, 0x856b, 0xa438, 0xbf8b, 0xa438, 0x5402,
        0xa438, 0x6c4e, 0xa438, 0xe085, 0xa438, 0x70e1, 0xa438, 0x8571,
        0xa438, 0xbf8b, 0xa438, 0x5702, 0xa438, 0x6c4e, 0xa438, 0xe085,
        0xa438, 0x76e1, 0xa438, 0x8577, 0xa438, 0xbf8b, 0xa438, 0x5a02,
        0xa438, 0x6c4e, 0xa438, 0xe18f, 0xa438, 0xb859, 0xa438, 0x00f7,
        0xa438, 0x28e5, 0xa438, 0x8fb8, 0xa438, 0xae0c, 0xa438, 0xe18f,
        0xa438, 0xb839, 0xa438, 0x04ac, 0xa438, 0x2f04, 0xa438, 0xee8f,
        0xa438, 0xb800, 0xa438, 0xfefd, 0xa438, 0xfc04, 0xa438, 0xf0ac,
        0xa438, 0x8efc, 0xa438, 0xac8c, 0xa438, 0xf0ac, 0xa438, 0xfaf0,
        0xa438, 0xacf8, 0xa438, 0xf0ac, 0xa438, 0xf6f0, 0xa438, 0xad00,
        0xa438, 0xf0ac, 0xa438, 0xfef0, 0xa438, 0xacfc, 0xa438, 0xf0ac,
        0xa438, 0xf4f0, 0xa438, 0xacf2, 0xa438, 0xf0ac, 0xa438, 0xf0f0,
        0xa438, 0xacb0, 0xa438, 0xf0ac, 0xa438, 0xaef0, 0xa438, 0xacac,
        0xa438, 0xf0ac, 0xa438, 0xaaf0, 0xa438, 0xacee, 0xa438, 0xf0b0,
        0xa438, 0x24f0, 0xa438, 0xb0a4, 0xa438, 0xf0b1, 0xa438, 0x24f0,
        0xa438, 0xb1a4, 0xa438, 0xee8f, 0xa438, 0xb800, 0xa438, 0xd400,
        0xa438, 0x00af, 0xa438, 0x3976, 0xa438, 0x66ac, 0xa438, 0xeabb,
        0xa438, 0xa430, 0xa438, 0x6e50, 0xa438, 0x6e53, 0xa438, 0x6e56,
        0xa438, 0x6e59, 0xa438, 0x6e5c, 0xa438, 0x6e5f, 0xa438, 0x6e62,
        0xa438, 0x6e65, 0xa438, 0xd9ac, 0xa438, 0x70f0, 0xa438, 0xac6a,
        0xa436, 0xb85e, 0xa438, 0x23b7, 0xa436, 0xb860, 0xa438, 0x74db,
        0xa436, 0xb862, 0xa438, 0x268c, 0xa436, 0xb864, 0xa438, 0x3FE5,
        0xa436, 0xb886, 0xa438, 0x2250, 0xa436, 0xb888, 0xa438, 0x140e,
        0xa436, 0xb88a, 0xa438, 0x3696, 0xa436, 0xb88c, 0xa438, 0x3973,
        0xa436, 0xb838, 0xa438, 0x00ff, 0xb820, 0x0010, 0xa436, 0x8464,
        0xa438, 0xaf84, 0xa438, 0x7caf, 0xa438, 0x8485, 0xa438, 0xaf85,
        0xa438, 0x13af, 0xa438, 0x851e, 0xa438, 0xaf85, 0xa438, 0xb9af,
        0xa438, 0x8684, 0xa438, 0xaf87, 0xa438, 0x01af, 0xa438, 0x8701,
        0xa438, 0xac38, 0xa438, 0x03af, 0xa438, 0x38bb, 0xa438, 0xaf38,
        0xa438, 0xc302, 0xa438, 0x4618, 0xa438, 0xbf85, 0xa438, 0x0a02,
        0xa438, 0x54b7, 0xa438, 0xbf85, 0xa438, 0x1002, 0xa438, 0x54c0,
        0xa438, 0xd400, 0xa438, 0x0fbf, 0xa438, 0x8507, 0xa438, 0x024f,
        0xa438, 0x48bf, 0xa438, 0x8504, 0xa438, 0x024f, 0xa438, 0x6759,
        0xa438, 0xf0a1, 0xa438, 0x3008, 0xa438, 0xbf85, 0xa438, 0x0d02,
        0xa438, 0x54c0, 0xa438, 0xae06, 0xa438, 0xbf85, 0xa438, 0x0d02,
        0xa438, 0x54b7, 0xa438, 0xbf85, 0xa438, 0x0402, 0xa438, 0x4f67,
        0xa438, 0xa183, 0xa438, 0x02ae, 0xa438, 0x15a1, 0xa438, 0x8502,
        0xa438, 0xae10, 0xa438, 0x59f0, 0xa438, 0xa180, 0xa438, 0x16bf,
        0xa438, 0x8501, 0xa438, 0x024f, 0xa438, 0x67a1, 0xa438, 0x381b,
        0xa438, 0xae0b, 0xa438, 0xe18f, 0xa438, 0xffbf, 0xa438, 0x84fe,
        0xa438, 0x024f, 0xa438, 0x48ae, 0xa438, 0x17bf, 0xa438, 0x84fe,
        0xa438, 0x0254, 0xa438, 0xb7bf, 0xa438, 0x84fb, 0xa438, 0x0254,
        0xa438, 0xb7ae, 0xa438, 0x09a1, 0xa438, 0x5006, 0xa438, 0xbf84,
        0xa438, 0xfb02, 0xa438, 0x54c0, 0xa438, 0xaf04, 0xa438, 0x4700,
        0xa438, 0xad34, 0xa438, 0xfdad, 0xa438, 0x0670, 0xa438, 0xae14,
        0xa438, 0xf0a6, 0xa438, 0x00b8, 0xa438, 0xbd32, 0xa438, 0x30bd,
        0xa438, 0x30aa, 0xa438, 0xbd2c, 0xa438, 0xccbd, 0xa438, 0x2ca1,
        0xa438, 0x0705, 0xa438, 0xec80, 0xa438, 0xaf40, 0xa438, 0xf7af,
        0xa438, 0x40f5, 0xa438, 0xd101, 0xa438, 0xbf85, 0xa438, 0xa402,
        0xa438, 0x4f48, 0xa438, 0xbf85, 0xa438, 0xa702, 0xa438, 0x54c0,
        0xa438, 0xd10f, 0xa438, 0xbf85, 0xa438, 0xaa02, 0xa438, 0x4f48,
        0xa438, 0x024d, 0xa438, 0x6abf, 0xa438, 0x85ad, 0xa438, 0x024f,
        0xa438, 0x67bf, 0xa438, 0x8ff7, 0xa438, 0xddbf, 0xa438, 0x85b0,
        0xa438, 0x024f, 0xa438, 0x67bf, 0xa438, 0x8ff8, 0xa438, 0xddbf,
        0xa438, 0x85b3, 0xa438, 0x024f, 0xa438, 0x67bf, 0xa438, 0x8ff9,
        0xa438, 0xddbf, 0xa438, 0x85b6, 0xa438, 0x024f, 0xa438, 0x67bf,
        0xa438, 0x8ffa, 0xa438, 0xddd1, 0xa438, 0x00bf, 0xa438, 0x85aa,
        0xa438, 0x024f, 0xa438, 0x4802, 0xa438, 0x4d6a, 0xa438, 0xbf85,
        0xa438, 0xad02, 0xa438, 0x4f67, 0xa438, 0xbf8f, 0xa438, 0xfbdd,
        0xa438, 0xbf85, 0xa438, 0xb002, 0xa438, 0x4f67, 0xa438, 0xbf8f,
        0xa438, 0xfcdd, 0xa438, 0xbf85, 0xa438, 0xb302, 0xa438, 0x4f67,
        0xa438, 0xbf8f, 0xa438, 0xfddd, 0xa438, 0xbf85, 0xa438, 0xb602,
        0xa438, 0x4f67, 0xa438, 0xbf8f, 0xa438, 0xfedd, 0xa438, 0xbf85,
        0xa438, 0xa702, 0xa438, 0x54b7, 0xa438, 0xbf85, 0xa438, 0xa102,
        0xa438, 0x54b7, 0xa438, 0xaf3c, 0xa438, 0x2066, 0xa438, 0xb800,
        0xa438, 0xb8bd, 0xa438, 0x30ee, 0xa438, 0xbd2c, 0xa438, 0xb8bd,
        0xa438, 0x7040, 0xa438, 0xbd86, 0xa438, 0xc8bd, 0xa438, 0x8640,
        0xa438, 0xbd88, 0xa438, 0xc8bd, 0xa438, 0x8802, 0xa438, 0x1929,
        0xa438, 0xa202, 0xa438, 0x02ae, 0xa438, 0x03a2, 0xa438, 0x032e,
        0xa438, 0xd10f, 0xa438, 0xbf85, 0xa438, 0xaa02, 0xa438, 0x4f48,
        0xa438, 0xe18f, 0xa438, 0xf7bf, 0xa438, 0x85ad, 0xa438, 0x024f,
        0xa438, 0x48e1, 0xa438, 0x8ff8, 0xa438, 0xbf85, 0xa438, 0xb002,
        0xa438, 0x4f48, 0xa438, 0xe18f, 0xa438, 0xf9bf, 0xa438, 0x85b3,
        0xa438, 0x024f, 0xa438, 0x48e1, 0xa438, 0x8ffa, 0xa438, 0xbf85,
        0xa438, 0xb602, 0xa438, 0x4f48, 0xa438, 0xae2c, 0xa438, 0xd100,
        0xa438, 0xbf85, 0xa438, 0xaa02, 0xa438, 0x4f48, 0xa438, 0xe18f,
        0xa438, 0xfbbf, 0xa438, 0x85ad, 0xa438, 0x024f, 0xa438, 0x48e1,
        0xa438, 0x8ffc, 0xa438, 0xbf85, 0xa438, 0xb002, 0xa438, 0x4f48,
        0xa438, 0xe18f, 0xa438, 0xfdbf, 0xa438, 0x85b3, 0xa438, 0x024f,
        0xa438, 0x48e1, 0xa438, 0x8ffe, 0xa438, 0xbf85, 0xa438, 0xb602,
        0xa438, 0x4f48, 0xa438, 0xbf86, 0xa438, 0x7e02, 0xa438, 0x4f67,
        0xa438, 0xa100, 0xa438, 0x02ae, 0xa438, 0x25a1, 0xa438, 0x041d,
        0xa438, 0xe18f, 0xa438, 0xf1bf, 0xa438, 0x8675, 0xa438, 0x024f,
        0xa438, 0x48e1, 0xa438, 0x8ff2, 0xa438, 0xbf86, 0xa438, 0x7802,
        0xa438, 0x4f48, 0xa438, 0xe18f, 0xa438, 0xf3bf, 0xa438, 0x867b,
        0xa438, 0x024f, 0xa438, 0x48ae, 0xa438, 0x29a1, 0xa438, 0x070b,
        0xa438, 0xae24, 0xa438, 0xbf86, 0xa438, 0x8102, 0xa438, 0x4f67,
        0xa438, 0xad28, 0xa438, 0x1be1, 0xa438, 0x8ff4, 0xa438, 0xbf86,
        0xa438, 0x7502, 0xa438, 0x4f48, 0xa438, 0xe18f, 0xa438, 0xf5bf,
        0xa438, 0x8678, 0xa438, 0x024f, 0xa438, 0x48e1, 0xa438, 0x8ff6,
        0xa438, 0xbf86, 0xa438, 0x7b02, 0xa438, 0x4f48, 0xa438, 0xaf09,
        0xa438, 0x8420, 0xa438, 0xbc32, 0xa438, 0x20bc, 0xa438, 0x3e76,
        0xa438, 0xbc08, 0xa438, 0xfda6, 0xa438, 0x1a00, 0xa438, 0xb64e,
        0xa438, 0xd101, 0xa438, 0xbf85, 0xa438, 0xa402, 0xa438, 0x4f48,
        0xa438, 0xbf85, 0xa438, 0xa702, 0xa438, 0x54c0, 0xa438, 0xd10f,
        0xa438, 0xbf85, 0xa438, 0xaa02, 0xa438, 0x4f48, 0xa438, 0x024d,
        0xa438, 0x6abf, 0xa438, 0x85ad, 0xa438, 0x024f, 0xa438, 0x67bf,
        0xa438, 0x8ff7, 0xa438, 0xddbf, 0xa438, 0x85b0, 0xa438, 0x024f,
        0xa438, 0x67bf, 0xa438, 0x8ff8, 0xa438, 0xddbf, 0xa438, 0x85b3,
        0xa438, 0x024f, 0xa438, 0x67bf, 0xa438, 0x8ff9, 0xa438, 0xddbf,
        0xa438, 0x85b6, 0xa438, 0x024f, 0xa438, 0x67bf, 0xa438, 0x8ffa,
        0xa438, 0xddd1, 0xa438, 0x00bf, 0xa438, 0x85aa, 0xa438, 0x024f,
        0xa438, 0x4802, 0xa438, 0x4d6a, 0xa438, 0xbf85, 0xa438, 0xad02,
        0xa438, 0x4f67, 0xa438, 0xbf8f, 0xa438, 0xfbdd, 0xa438, 0xbf85,
        0xa438, 0xb002, 0xa438, 0x4f67, 0xa438, 0xbf8f, 0xa438, 0xfcdd,
        0xa438, 0xbf85, 0xa438, 0xb302, 0xa438, 0x4f67, 0xa438, 0xbf8f,
        0xa438, 0xfddd, 0xa438, 0xbf85, 0xa438, 0xb602, 0xa438, 0x4f67,
        0xa438, 0xbf8f, 0xa438, 0xfedd, 0xa438, 0xbf85, 0xa438, 0xa702,
        0xa438, 0x54b7, 0xa438, 0xaf00, 0xa438, 0x8800, 0xa436, 0xb818,
        0xa438, 0x38b8, 0xa436, 0xb81a, 0xa438, 0x0444, 0xa436, 0xb81c,
        0xa438, 0x40ee, 0xa436, 0xb81e, 0xa438, 0x3C1A, 0xa436, 0xb850,
        0xa438, 0x0981, 0xa436, 0xb852, 0xa438, 0x0085, 0xa436, 0xb878,
        0xa438, 0xffff, 0xa436, 0xb884, 0xa438, 0xffff, 0xa436, 0xb832,
        0xa438, 0x003f, 0xa436, 0x0000, 0xa438, 0x0000, 0xa436, 0xB82E,
        0xa438, 0x0000, 0xa436, 0x8024, 0xa438, 0x0000, 0xb820, 0x0000,
        0xa436, 0x801E, 0xa438, 0x0021, 0xFFFF, 0xFFFF
};

static const u16 phy_mcu_ram_code_8125b_2[] = {
        0xa436, 0x8024, 0xa438, 0x3701, 0xa436, 0xB82E, 0xa438, 0x0001,
        0xb820, 0x0090, 0xa436, 0xA016, 0xa438, 0x0000, 0xa436, 0xA012,
        0xa438, 0x0000, 0xa436, 0xA014, 0xa438, 0x1800, 0xa438, 0x8010,
        0xa438, 0x1800, 0xa438, 0x801a, 0xa438, 0x1800, 0xa438, 0x803f,
        0xa438, 0x1800, 0xa438, 0x8045, 0xa438, 0x1800, 0xa438, 0x8067,
        0xa438, 0x1800, 0xa438, 0x806d, 0xa438, 0x1800, 0xa438, 0x8071,
        0xa438, 0x1800, 0xa438, 0x80b1, 0xa438, 0xd093, 0xa438, 0xd1c4,
        0xa438, 0x1000, 0xa438, 0x135c, 0xa438, 0xd704, 0xa438, 0x5fbc,
        0xa438, 0xd504, 0xa438, 0xc9f1, 0xa438, 0x1800, 0xa438, 0x0fc9,
        0xa438, 0xbb50, 0xa438, 0xd505, 0xa438, 0xa202, 0xa438, 0xd504,
        0xa438, 0x8c0f, 0xa438, 0xd500, 0xa438, 0x1000, 0xa438, 0x1519,
        0xa438, 0x1000, 0xa438, 0x135c, 0xa438, 0xd75e, 0xa438, 0x5fae,
        0xa438, 0x9b50, 0xa438, 0x1000, 0xa438, 0x135c, 0xa438, 0xd75e,
        0xa438, 0x7fae, 0xa438, 0x1000, 0xa438, 0x135c, 0xa438, 0xd707,
        0xa438, 0x40a7, 0xa438, 0xd719, 0xa438, 0x4071, 0xa438, 0x1800,
        0xa438, 0x1557, 0xa438, 0xd719, 0xa438, 0x2f70, 0xa438, 0x803b,
        0xa438, 0x2f73, 0xa438, 0x156a, 0xa438, 0x5e70, 0xa438, 0x1800,
        0xa438, 0x155d, 0xa438, 0xd505, 0xa438, 0xa202, 0xa438, 0xd500,
        0xa438, 0xffed, 0xa438, 0xd709, 0xa438, 0x4054, 0xa438, 0xa788,
        0xa438, 0xd70b, 0xa438, 0x1800, 0xa438, 0x172a, 0xa438, 0xc0c1,
        0xa438, 0xc0c0, 0xa438, 0xd05a, 0xa438, 0xd1ba, 0xa438, 0xd701,
        0xa438, 0x2529, 0xa438, 0x022a, 0xa438, 0xd0a7, 0xa438, 0xd1b9,
        0xa438, 0xa208, 0xa438, 0x1000, 0xa438, 0x080e, 0xa438, 0xd701,
        0xa438, 0x408b, 0xa438, 0x1000, 0xa438, 0x0a65, 0xa438, 0xf003,
        0xa438, 0x1000, 0xa438, 0x0a6b, 0xa438, 0xd701, 0xa438, 0x1000,
        0xa438, 0x0920, 0xa438, 0x1000, 0xa438, 0x0915, 0xa438, 0x1000,
        0xa438, 0x0909, 0xa438, 0x228f, 0xa438, 0x804e, 0xa438, 0x9801,
        0xa438, 0xd71e, 0xa438, 0x5d61, 0xa438, 0xd701, 0xa438, 0x1800,
        0xa438, 0x022a, 0xa438, 0x2005, 0xa438, 0x091a, 0xa438, 0x3bd9,
        0xa438, 0x0919, 0xa438, 0x1800, 0xa438, 0x0916, 0xa438, 0xd090,
        0xa438, 0xd1c9, 0xa438, 0x1800, 0xa438, 0x1064, 0xa438, 0xd096,
        0xa438, 0xd1a9, 0xa438, 0xd503, 0xa438, 0xa104, 0xa438, 0x0c07,
        0xa438, 0x0902, 0xa438, 0xd500, 0xa438, 0xbc10, 0xa438, 0xd501,
        0xa438, 0xce01, 0xa438, 0xa201, 0xa438, 0x8201, 0xa438, 0xce00,
        0xa438, 0xd500, 0xa438, 0xc484, 0xa438, 0xd503, 0xa438, 0xcc02,
        0xa438, 0xcd0d, 0xa438, 0xaf01, 0xa438, 0xd500, 0xa438, 0xd703,
        0xa438, 0x4371, 0xa438, 0xbd08, 0xa438, 0x1000, 0xa438, 0x135c,
        0xa438, 0xd75e, 0xa438, 0x5fb3, 0xa438, 0xd503, 0xa438, 0xd0f5,
        0xa438, 0xd1c6, 0xa438, 0x0cf0, 0xa438, 0x0e50, 0xa438, 0xd704,
        0xa438, 0x401c, 0xa438, 0xd0f5, 0xa438, 0xd1c6, 0xa438, 0x0cf0,
        0xa438, 0x0ea0, 0xa438, 0x401c, 0xa438, 0xd07b, 0xa438, 0xd1c5,
        0xa438, 0x8ef0, 0xa438, 0x401c, 0xa438, 0x9d08, 0xa438, 0x1000,
        0xa438, 0x135c, 0xa438, 0xd75e, 0xa438, 0x7fb3, 0xa438, 0x1000,
        0xa438, 0x135c, 0xa438, 0xd75e, 0xa438, 0x5fad, 0xa438, 0x1000,
        0xa438, 0x14c5, 0xa438, 0xd703, 0xa438, 0x3181, 0xa438, 0x80af,
        0xa438, 0x60ad, 0xa438, 0x1000, 0xa438, 0x135c, 0xa438, 0xd703,
        0xa438, 0x5fba, 0xa438, 0x1800, 0xa438, 0x0cc7, 0xa438, 0xa802,
        0xa438, 0xa301, 0xa438, 0xa801, 0xa438, 0xc004, 0xa438, 0xd710,
        0xa438, 0x4000, 0xa438, 0x1800, 0xa438, 0x1e79, 0xa436, 0xA026,
        0xa438, 0x1e78, 0xa436, 0xA024, 0xa438, 0x0c93, 0xa436, 0xA022,
        0xa438, 0x1062, 0xa436, 0xA020, 0xa438, 0x0915, 0xa436, 0xA006,
        0xa438, 0x020a, 0xa436, 0xA004, 0xa438, 0x1726, 0xa436, 0xA002,
        0xa438, 0x1542, 0xa436, 0xA000, 0xa438, 0x0fc7, 0xa436, 0xA008,
        0xa438, 0xff00, 0xa436, 0xA016, 0xa438, 0x0010, 0xa436, 0xA012,
        0xa438, 0x0000, 0xa436, 0xA014, 0xa438, 0x1800, 0xa438, 0x8010,
        0xa438, 0x1800, 0xa438, 0x801d, 0xa438, 0x1800, 0xa438, 0x802c,
        0xa438, 0x1800, 0xa438, 0x802c, 0xa438, 0x1800, 0xa438, 0x802c,
        0xa438, 0x1800, 0xa438, 0x802c, 0xa438, 0x1800, 0xa438, 0x802c,
        0xa438, 0x1800, 0xa438, 0x802c, 0xa438, 0xd700, 0xa438, 0x6090,
        0xa438, 0x60d1, 0xa438, 0xc95c, 0xa438, 0xf007, 0xa438, 0x60b1,
        0xa438, 0xc95a, 0xa438, 0xf004, 0xa438, 0xc956, 0xa438, 0xf002,
        0xa438, 0xc94e, 0xa438, 0x1800, 0xa438, 0x00cd, 0xa438, 0xd700,
        0xa438, 0x6090, 0xa438, 0x60d1, 0xa438, 0xc95c, 0xa438, 0xf007,
        0xa438, 0x60b1, 0xa438, 0xc95a, 0xa438, 0xf004, 0xa438, 0xc956,
        0xa438, 0xf002, 0xa438, 0xc94e, 0xa438, 0x1000, 0xa438, 0x022a,
        0xa438, 0x1800, 0xa438, 0x0132, 0xa436, 0xA08E, 0xa438, 0xffff,
        0xa436, 0xA08C, 0xa438, 0xffff, 0xa436, 0xA08A, 0xa438, 0xffff,
        0xa436, 0xA088, 0xa438, 0xffff, 0xa436, 0xA086, 0xa438, 0xffff,
        0xa436, 0xA084, 0xa438, 0xffff, 0xa436, 0xA082, 0xa438, 0x012f,
        0xa436, 0xA080, 0xa438, 0x00cc, 0xa436, 0xA090, 0xa438, 0x0103,
        0xa436, 0xA016, 0xa438, 0x0020, 0xa436, 0xA012, 0xa438, 0x0000,
        0xa436, 0xA014, 0xa438, 0x1800, 0xa438, 0x8010, 0xa438, 0x1800,
        0xa438, 0x8020, 0xa438, 0x1800, 0xa438, 0x802a, 0xa438, 0x1800,
        0xa438, 0x8035, 0xa438, 0x1800, 0xa438, 0x803c, 0xa438, 0x1800,
        0xa438, 0x803c, 0xa438, 0x1800, 0xa438, 0x803c, 0xa438, 0x1800,
        0xa438, 0x803c, 0xa438, 0xd107, 0xa438, 0xd042, 0xa438, 0xa404,
        0xa438, 0x1000, 0xa438, 0x09df, 0xa438, 0xd700, 0xa438, 0x5fb4,
        0xa438, 0x8280, 0xa438, 0xd700, 0xa438, 0x6065, 0xa438, 0xd125,
        0xa438, 0xf002, 0xa438, 0xd12b, 0xa438, 0xd040, 0xa438, 0x1800,
        0xa438, 0x077f, 0xa438, 0x0cf0, 0xa438, 0x0c50, 0xa438, 0xd104,
        0xa438, 0xd040, 0xa438, 0x1000, 0xa438, 0x0aa8, 0xa438, 0xd700,
        0xa438, 0x5fb4, 0xa438, 0x1800, 0xa438, 0x0a2e, 0xa438, 0xcb9b,
        0xa438, 0xd110, 0xa438, 0xd040, 0xa438, 0x1000, 0xa438, 0x0b7b,
        0xa438, 0x1000, 0xa438, 0x09df, 0xa438, 0xd700, 0xa438, 0x5fb4,
        0xa438, 0x1800, 0xa438, 0x081b, 0xa438, 0x1000, 0xa438, 0x09df,
        0xa438, 0xd704, 0xa438, 0x7fb8, 0xa438, 0xa718, 0xa438, 0x1800,
        0xa438, 0x074e, 0xa436, 0xA10E, 0xa438, 0xffff, 0xa436, 0xA10C,
        0xa438, 0xffff, 0xa436, 0xA10A, 0xa438, 0xffff, 0xa436, 0xA108,
        0xa438, 0xffff, 0xa436, 0xA106, 0xa438, 0x074d, 0xa436, 0xA104,
        0xa438, 0x0818, 0xa436, 0xA102, 0xa438, 0x0a2c, 0xa436, 0xA100,
        0xa438, 0x077e, 0xa436, 0xA110, 0xa438, 0x000f, 0xa436, 0xb87c,
        0xa438, 0x8625, 0xa436, 0xb87e, 0xa438, 0xaf86, 0xa438, 0x3daf,
        0xa438, 0x8689, 0xa438, 0xaf88, 0xa438, 0x69af, 0xa438, 0x8887,
        0xa438, 0xaf88, 0xa438, 0x9caf, 0xa438, 0x88be, 0xa438, 0xaf88,
        0xa438, 0xbeaf, 0xa438, 0x88be, 0xa438, 0xbf86, 0xa438, 0x49d7,
        0xa438, 0x0040, 0xa438, 0x0277, 0xa438, 0x7daf, 0xa438, 0x2727,
        0xa438, 0x0000, 0xa438, 0x7205, 0xa438, 0x0000, 0xa438, 0x7208,
        0xa438, 0x0000, 0xa438, 0x71f3, 0xa438, 0x0000, 0xa438, 0x71f6,
        0xa438, 0x0000, 0xa438, 0x7229, 0xa438, 0x0000, 0xa438, 0x722c,
        0xa438, 0x0000, 0xa438, 0x7217, 0xa438, 0x0000, 0xa438, 0x721a,
        0xa438, 0x0000, 0xa438, 0x721d, 0xa438, 0x0000, 0xa438, 0x7211,
        0xa438, 0x0000, 0xa438, 0x7220, 0xa438, 0x0000, 0xa438, 0x7214,
        0xa438, 0x0000, 0xa438, 0x722f, 0xa438, 0x0000, 0xa438, 0x7223,
        0xa438, 0x0000, 0xa438, 0x7232, 0xa438, 0x0000, 0xa438, 0x7226,
        0xa438, 0xf8f9, 0xa438, 0xfae0, 0xa438, 0x85b3, 0xa438, 0x3802,
        0xa438, 0xad27, 0xa438, 0x02ae, 0xa438, 0x03af, 0xa438, 0x8830,
        0xa438, 0x1f66, 0xa438, 0xef65, 0xa438, 0xbfc2, 0xa438, 0x1f1a,
        0xa438, 0x96f7, 0xa438, 0x05ee, 0xa438, 0xffd2, 0xa438, 0x00da,
        0xa438, 0xf605, 0xa438, 0xbfc2, 0xa438, 0x2f1a, 0xa438, 0x96f7,
        0xa438, 0x05ee, 0xa438, 0xffd2, 0xa438, 0x00db, 0xa438, 0xf605,
        0xa438, 0xef02, 0xa438, 0x1f11, 0xa438, 0x0d42, 0xa438, 0xbf88,
        0xa438, 0x4202, 0xa438, 0x6e7d, 0xa438, 0xef02, 0xa438, 0x1b03,
        0xa438, 0x1f11, 0xa438, 0x0d42, 0xa438, 0xbf88, 0xa438, 0x4502,
        0xa438, 0x6e7d, 0xa438, 0xef02, 0xa438, 0x1a03, 0xa438, 0x1f11,
        0xa438, 0x0d42, 0xa438, 0xbf88, 0xa438, 0x4802, 0xa438, 0x6e7d,
        0xa438, 0xbfc2, 0xa438, 0x3f1a, 0xa438, 0x96f7, 0xa438, 0x05ee,
        0xa438, 0xffd2, 0xa438, 0x00da, 0xa438, 0xf605, 0xa438, 0xbfc2,
        0xa438, 0x4f1a, 0xa438, 0x96f7, 0xa438, 0x05ee, 0xa438, 0xffd2,
        0xa438, 0x00db, 0xa438, 0xf605, 0xa438, 0xef02, 0xa438, 0x1f11,
        0xa438, 0x0d42, 0xa438, 0xbf88, 0xa438, 0x4b02, 0xa438, 0x6e7d,
        0xa438, 0xef02, 0xa438, 0x1b03, 0xa438, 0x1f11, 0xa438, 0x0d42,
        0xa438, 0xbf88, 0xa438, 0x4e02, 0xa438, 0x6e7d, 0xa438, 0xef02,
        0xa438, 0x1a03, 0xa438, 0x1f11, 0xa438, 0x0d42, 0xa438, 0xbf88,
        0xa438, 0x5102, 0xa438, 0x6e7d, 0xa438, 0xef56, 0xa438, 0xd020,
        0xa438, 0x1f11, 0xa438, 0xbf88, 0xa438, 0x5402, 0xa438, 0x6e7d,
        0xa438, 0xbf88, 0xa438, 0x5702, 0xa438, 0x6e7d, 0xa438, 0xbf88,
        0xa438, 0x5a02, 0xa438, 0x6e7d, 0xa438, 0xe185, 0xa438, 0xa0ef,
        0xa438, 0x0348, 0xa438, 0x0a28, 0xa438, 0x05ef, 0xa438, 0x201b,
        0xa438, 0x01ad, 0xa438, 0x2735, 0xa438, 0x1f44, 0xa438, 0xe085,
        0xa438, 0x88e1, 0xa438, 0x8589, 0xa438, 0xbf88, 0xa438, 0x5d02,
        0xa438, 0x6e7d, 0xa438, 0xe085, 0xa438, 0x8ee1, 0xa438, 0x858f,
        0xa438, 0xbf88, 0xa438, 0x6002, 0xa438, 0x6e7d, 0xa438, 0xe085,
        0xa438, 0x94e1, 0xa438, 0x8595, 0xa438, 0xbf88, 0xa438, 0x6302,
        0xa438, 0x6e7d, 0xa438, 0xe085, 0xa438, 0x9ae1, 0xa438, 0x859b,
        0xa438, 0xbf88, 0xa438, 0x6602, 0xa438, 0x6e7d, 0xa438, 0xaf88,
        0xa438, 0x3cbf, 0xa438, 0x883f, 0xa438, 0x026e, 0xa438, 0x9cad,
        0xa438, 0x2835, 0xa438, 0x1f44, 0xa438, 0xe08f, 0xa438, 0xf8e1,
        0xa438, 0x8ff9, 0xa438, 0xbf88, 0xa438, 0x5d02, 0xa438, 0x6e7d,
        0xa438, 0xe08f, 0xa438, 0xfae1, 0xa438, 0x8ffb, 0xa438, 0xbf88,
        0xa438, 0x6002, 0xa438, 0x6e7d, 0xa438, 0xe08f, 0xa438, 0xfce1,
        0xa438, 0x8ffd, 0xa438, 0xbf88, 0xa438, 0x6302, 0xa438, 0x6e7d,
        0xa438, 0xe08f, 0xa438, 0xfee1, 0xa438, 0x8fff, 0xa438, 0xbf88,
        0xa438, 0x6602, 0xa438, 0x6e7d, 0xa438, 0xaf88, 0xa438, 0x3ce1,
        0xa438, 0x85a1, 0xa438, 0x1b21, 0xa438, 0xad37, 0xa438, 0x341f,
        0xa438, 0x44e0, 0xa438, 0x858a, 0xa438, 0xe185, 0xa438, 0x8bbf,
        0xa438, 0x885d, 0xa438, 0x026e, 0xa438, 0x7de0, 0xa438, 0x8590,
        0xa438, 0xe185, 0xa438, 0x91bf, 0xa438, 0x8860, 0xa438, 0x026e,
        0xa438, 0x7de0, 0xa438, 0x8596, 0xa438, 0xe185, 0xa438, 0x97bf,
        0xa438, 0x8863, 0xa438, 0x026e, 0xa438, 0x7de0, 0xa438, 0x859c,
        0xa438, 0xe185, 0xa438, 0x9dbf, 0xa438, 0x8866, 0xa438, 0x026e,
        0xa438, 0x7dae, 0xa438, 0x401f, 0xa438, 0x44e0, 0xa438, 0x858c,
        0xa438, 0xe185, 0xa438, 0x8dbf, 0xa438, 0x885d, 0xa438, 0x026e,
        0xa438, 0x7de0, 0xa438, 0x8592, 0xa438, 0xe185, 0xa438, 0x93bf,
        0xa438, 0x8860, 0xa438, 0x026e, 0xa438, 0x7de0, 0xa438, 0x8598,
        0xa438, 0xe185, 0xa438, 0x99bf, 0xa438, 0x8863, 0xa438, 0x026e,
        0xa438, 0x7de0, 0xa438, 0x859e, 0xa438, 0xe185, 0xa438, 0x9fbf,
        0xa438, 0x8866, 0xa438, 0x026e, 0xa438, 0x7dae, 0xa438, 0x0ce1,
        0xa438, 0x85b3, 0xa438, 0x3904, 0xa438, 0xac2f, 0xa438, 0x04ee,
        0xa438, 0x85b3, 0xa438, 0x00af, 0xa438, 0x39d9, 0xa438, 0x22ac,
        0xa438, 0xeaf0, 0xa438, 0xacf6, 0xa438, 0xf0ac, 0xa438, 0xfaf0,
        0xa438, 0xacf8, 0xa438, 0xf0ac, 0xa438, 0xfcf0, 0xa438, 0xad00,
        0xa438, 0xf0ac, 0xa438, 0xfef0, 0xa438, 0xacf0, 0xa438, 0xf0ac,
        0xa438, 0xf4f0, 0xa438, 0xacf2, 0xa438, 0xf0ac, 0xa438, 0xb0f0,
        0xa438, 0xacae, 0xa438, 0xf0ac, 0xa438, 0xacf0, 0xa438, 0xacaa,
        0xa438, 0xa100, 0xa438, 0x0ce1, 0xa438, 0x8ff7, 0xa438, 0xbf88,
        0xa438, 0x8402, 0xa438, 0x6e7d, 0xa438, 0xaf26, 0xa438, 0xe9e1,
        0xa438, 0x8ff6, 0xa438, 0xbf88, 0xa438, 0x8402, 0xa438, 0x6e7d,
        0xa438, 0xaf26, 0xa438, 0xf520, 0xa438, 0xac86, 0xa438, 0xbf88,
        0xa438, 0x3f02, 0xa438, 0x6e9c, 0xa438, 0xad28, 0xa438, 0x03af,
        0xa438, 0x3324, 0xa438, 0xad38, 0xa438, 0x03af, 0xa438, 0x32e6,
        0xa438, 0xaf32, 0xa438, 0xfbee, 0xa438, 0x826a, 0xa438, 0x0002,
        0xa438, 0x88a6, 0xa438, 0xaf04, 0xa438, 0x78f8, 0xa438, 0xfaef,
        0xa438, 0x69e0, 0xa438, 0x8015, 0xa438, 0xad20, 0xa438, 0x06bf,
        0xa438, 0x88bb, 0xa438, 0x0275, 0xa438, 0xb1ef, 0xa438, 0x96fe,
        0xa438, 0xfc04, 0xa438, 0x00b8, 0xa438, 0x7a00, 0xa436, 0xb87c,
        0xa438, 0x8ff6, 0xa436, 0xb87e, 0xa438, 0x0705, 0xa436, 0xb87c,
        0xa438, 0x8ff8, 0xa436, 0xb87e, 0xa438, 0x19cc, 0xa436, 0xb87c,
        0xa438, 0x8ffa, 0xa436, 0xb87e, 0xa438, 0x28e3, 0xa436, 0xb87c,
        0xa438, 0x8ffc, 0xa436, 0xb87e, 0xa438, 0x1047, 0xa436, 0xb87c,
        0xa438, 0x8ffe, 0xa436, 0xb87e, 0xa438, 0x0a45, 0xa436, 0xb85e,
        0xa438, 0x271E, 0xa436, 0xb860, 0xa438, 0x3846, 0xa436, 0xb862,
        0xa438, 0x26E6, 0xa436, 0xb864, 0xa438, 0x32E3, 0xa436, 0xb886,
        0xa438, 0x0474, 0xa436, 0xb888, 0xa438, 0xffff, 0xa436, 0xb88a,
        0xa438, 0xffff, 0xa436, 0xb88c, 0xa438, 0xffff, 0xa436, 0xb838,
        0xa438, 0x001f, 0xb820, 0x0010, 0xa436, 0x846e, 0xa438, 0xaf84,
        0xa438, 0x86af, 0xa438, 0x8690, 0xa438, 0xaf86, 0xa438, 0xa4af,
        0xa438, 0x8934, 0xa438, 0xaf89, 0xa438, 0x60af, 0xa438, 0x897e,
        0xa438, 0xaf89, 0xa438, 0xa9af, 0xa438, 0x89a9, 0xa438, 0xee82,
        0xa438, 0x5f00, 0xa438, 0x0284, 0xa438, 0x90af, 0xa438, 0x0441,
        0xa438, 0xf8e0, 0xa438, 0x8ff3, 0xa438, 0xa000, 0xa438, 0x0502,
        0xa438, 0x84a4, 0xa438, 0xae06, 0xa438, 0xa001, 0xa438, 0x0302,
        0xa438, 0x84c8, 0xa438, 0xfc04, 0xa438, 0xf8f9, 0xa438, 0xef59,
        0xa438, 0xe080, 0xa438, 0x15ad, 0xa438, 0x2702, 0xa438, 0xae03,
        0xa438, 0xaf84, 0xa438, 0xc3bf, 0xa438, 0x53ca, 0xa438, 0x0252,
        0xa438, 0xc8ad, 0xa438, 0x2807, 0xa438, 0x0285, 0xa438, 0x2cee,
        0xa438, 0x8ff3, 0xa438, 0x01ef, 0xa438, 0x95fd, 0xa438, 0xfc04,
        0xa438, 0xf8f9, 0xa438, 0xfaef, 0xa438, 0x69bf, 0xa438, 0x53ca,
        0xa438, 0x0252, 0xa438, 0xc8ac, 0xa438, 0x2822, 0xa438, 0xd480,
        0xa438, 0x00bf, 0xa438, 0x8684, 0xa438, 0x0252, 0xa438, 0xa9bf,
        0xa438, 0x8687, 0xa438, 0x0252, 0xa438, 0xa9bf, 0xa438, 0x868a,
        0xa438, 0x0252, 0xa438, 0xa9bf, 0xa438, 0x868d, 0xa438, 0x0252,
        0xa438, 0xa9ee, 0xa438, 0x8ff3, 0xa438, 0x00af, 0xa438, 0x8526,
        0xa438, 0xe08f, 0xa438, 0xf4e1, 0xa438, 0x8ff5, 0xa438, 0xe28f,
        0xa438, 0xf6e3, 0xa438, 0x8ff7, 0xa438, 0x1b45, 0xa438, 0xac27,
        0xa438, 0x0eee, 0xa438, 0x8ff4, 0xa438, 0x00ee, 0xa438, 0x8ff5,
        0xa438, 0x0002, 0xa438, 0x852c, 0xa438, 0xaf85, 0xa438, 0x26e0,
        0xa438, 0x8ff4, 0xa438, 0xe18f, 0xa438, 0xf52c, 0xa438, 0x0001,
        0xa438, 0xe48f, 0xa438, 0xf4e5, 0xa438, 0x8ff5, 0xa438, 0xef96,
        0xa438, 0xfefd, 0xa438, 0xfc04, 0xa438, 0xf8f9, 0xa438, 0xef59,
        0xa438, 0xbf53, 0xa438, 0x2202, 0xa438, 0x52c8, 0xa438, 0xa18b,
        0xa438, 0x02ae, 0xa438, 0x03af, 0xa438, 0x85da, 0xa438, 0xbf57,
        0xa438, 0x7202, 0xa438, 0x52c8, 0xa438, 0xe48f, 0xa438, 0xf8e5,
        0xa438, 0x8ff9, 0xa438, 0xbf57, 0xa438, 0x7502, 0xa438, 0x52c8,
        0xa438, 0xe48f, 0xa438, 0xfae5, 0xa438, 0x8ffb, 0xa438, 0xbf57,
        0xa438, 0x7802, 0xa438, 0x52c8, 0xa438, 0xe48f, 0xa438, 0xfce5,
        0xa438, 0x8ffd, 0xa438, 0xbf57, 0xa438, 0x7b02, 0xa438, 0x52c8,
        0xa438, 0xe48f, 0xa438, 0xfee5, 0xa438, 0x8fff, 0xa438, 0xbf57,
        0xa438, 0x6c02, 0xa438, 0x52c8, 0xa438, 0xa102, 0xa438, 0x13ee,
        0xa438, 0x8ffc, 0xa438, 0x80ee, 0xa438, 0x8ffd, 0xa438, 0x00ee,
        0xa438, 0x8ffe, 0xa438, 0x80ee, 0xa438, 0x8fff, 0xa438, 0x00af,
        0xa438, 0x8599, 0xa438, 0xa101, 0xa438, 0x0cbf, 0xa438, 0x534c,
        0xa438, 0x0252, 0xa438, 0xc8a1, 0xa438, 0x0303, 0xa438, 0xaf85,
        0xa438, 0x77bf, 0xa438, 0x5322, 0xa438, 0x0252, 0xa438, 0xc8a1,
        0xa438, 0x8b02, 0xa438, 0xae03, 0xa438, 0xaf86, 0xa438, 0x64e0,
        0xa438, 0x8ff8, 0xa438, 0xe18f, 0xa438, 0xf9bf, 0xa438, 0x8684,
        0xa438, 0x0252, 0xa438, 0xa9e0, 0xa438, 0x8ffa, 0xa438, 0xe18f,
        0xa438, 0xfbbf, 0xa438, 0x8687, 0xa438, 0x0252, 0xa438, 0xa9e0,
        0xa438, 0x8ffc, 0xa438, 0xe18f, 0xa438, 0xfdbf, 0xa438, 0x868a,
        0xa438, 0x0252, 0xa438, 0xa9e0, 0xa438, 0x8ffe, 0xa438, 0xe18f,
        0xa438, 0xffbf, 0xa438, 0x868d, 0xa438, 0x0252, 0xa438, 0xa9af,
        0xa438, 0x867f, 0xa438, 0xbf53, 0xa438, 0x2202, 0xa438, 0x52c8,
        0xa438, 0xa144, 0xa438, 0x3cbf, 0xa438, 0x547b, 0xa438, 0x0252,
        0xa438, 0xc8e4, 0xa438, 0x8ff8, 0xa438, 0xe58f, 0xa438, 0xf9bf,
        0xa438, 0x547e, 0xa438, 0x0252, 0xa438, 0xc8e4, 0xa438, 0x8ffa,
        0xa438, 0xe58f, 0xa438, 0xfbbf, 0xa438, 0x5481, 0xa438, 0x0252,
        0xa438, 0xc8e4, 0xa438, 0x8ffc, 0xa438, 0xe58f, 0xa438, 0xfdbf,
        0xa438, 0x5484, 0xa438, 0x0252, 0xa438, 0xc8e4, 0xa438, 0x8ffe,
        0xa438, 0xe58f, 0xa438, 0xffbf, 0xa438, 0x5322, 0xa438, 0x0252,
        0xa438, 0xc8a1, 0xa438, 0x4448, 0xa438, 0xaf85, 0xa438, 0xa7bf,
        0xa438, 0x5322, 0xa438, 0x0252, 0xa438, 0xc8a1, 0xa438, 0x313c,
        0xa438, 0xbf54, 0xa438, 0x7b02, 0xa438, 0x52c8, 0xa438, 0xe48f,
        0xa438, 0xf8e5, 0xa438, 0x8ff9, 0xa438, 0xbf54, 0xa438, 0x7e02,
        0xa438, 0x52c8, 0xa438, 0xe48f, 0xa438, 0xfae5, 0xa438, 0x8ffb,
        0xa438, 0xbf54, 0xa438, 0x8102, 0xa438, 0x52c8, 0xa438, 0xe48f,
        0xa438, 0xfce5, 0xa438, 0x8ffd, 0xa438, 0xbf54, 0xa438, 0x8402,
        0xa438, 0x52c8, 0xa438, 0xe48f, 0xa438, 0xfee5, 0xa438, 0x8fff,
        0xa438, 0xbf53, 0xa438, 0x2202, 0xa438, 0x52c8, 0xa438, 0xa131,
        0xa438, 0x03af, 0xa438, 0x85a7, 0xa438, 0xd480, 0xa438, 0x00bf,
        0xa438, 0x8684, 0xa438, 0x0252, 0xa438, 0xa9bf, 0xa438, 0x8687,
        0xa438, 0x0252, 0xa438, 0xa9bf, 0xa438, 0x868a, 0xa438, 0x0252,
        0xa438, 0xa9bf, 0xa438, 0x868d, 0xa438, 0x0252, 0xa438, 0xa9ef,
        0xa438, 0x95fd, 0xa438, 0xfc04, 0xa438, 0xf0d1, 0xa438, 0x2af0,
        0xa438, 0xd12c, 0xa438, 0xf0d1, 0xa438, 0x44f0, 0xa438, 0xd146,
        0xa438, 0xbf86, 0xa438, 0xa102, 0xa438, 0x52c8, 0xa438, 0xbf86,
        0xa438, 0xa102, 0xa438, 0x52c8, 0xa438, 0xd101, 0xa438, 0xaf06,
        0xa438, 0xa570, 0xa438, 0xce42, 0xa438, 0xee83, 0xa438, 0xc800,
        0xa438, 0x0286, 0xa438, 0xba02, 0xa438, 0x8728, 0xa438, 0x0287,
        0xa438, 0xbe02, 0xa438, 0x87f9, 0xa438, 0x0288, 0xa438, 0xc3af,
        0xa438, 0x4771, 0xa438, 0xf8f9, 0xa438, 0xfafb, 0xa438, 0xef69,
        0xa438, 0xfae0, 0xa438, 0x8015, 0xa438, 0xad25, 0xa438, 0x45d2,
        0xa438, 0x0002, 0xa438, 0x8714, 0xa438, 0xac4f, 0xa438, 0x02ae,
        0xa438, 0x0bef, 0xa438, 0x46f6, 0xa438, 0x273c, 0xa438, 0x0400,
        0xa438, 0xab26, 0xa438, 0xae30, 0xa438, 0xe08f, 0xa438, 0xe9e1,
        0xa438, 0x8fea, 0xa438, 0x1b46, 0xa438, 0xab26, 0xa438, 0xef32,
        0xa438, 0x0c31, 0xa438, 0xbf8f, 0xa438, 0xe91a, 0xa438, 0x93d8,
        0xa438, 0x19d9, 0xa438, 0x1b46, 0xa438, 0xab0a, 0xa438, 0x19d8,
        0xa438, 0x19d9, 0xa438, 0x1b46, 0xa438, 0xaa02, 0xa438, 0xae0c,
        0xa438, 0xbf57, 0xa438, 0x1202, 0xa438, 0x58b1, 0xa438, 0xbf57,
        0xa438, 0x1202, 0xa438, 0x58a8, 0xa438, 0xfeef, 0xa438, 0x96ff,
        0xa438, 0xfefd, 0xa438, 0xfc04, 0xa438, 0xf8fb, 0xa438, 0xef79,
        0xa438, 0xa200, 0xa438, 0x08bf, 0xa438, 0x892e, 0xa438, 0x0252,
        0xa438, 0xc8ef, 0xa438, 0x64ef, 0xa438, 0x97ff, 0xa438, 0xfc04,
        0xa438, 0xf8f9, 0xa438, 0xfafb, 0xa438, 0xef69, 0xa438, 0xfae0,
        0xa438, 0x8015, 0xa438, 0xad25, 0xa438, 0x50d2, 0xa438, 0x0002,
        0xa438, 0x878d, 0xa438, 0xac4f, 0xa438, 0x02ae, 0xa438, 0x0bef,
        0xa438, 0x46f6, 0xa438, 0x273c, 0xa438, 0x1000, 0xa438, 0xab31,
        0xa438, 0xae29, 0xa438, 0xe08f, 0xa438, 0xede1, 0xa438, 0x8fee,
        0xa438, 0x1b46, 0xa438, 0xab1f, 0xa438, 0xa200, 0xa438, 0x04ef,
        0xa438, 0x32ae, 0xa438, 0x02d3, 0xa438, 0x010c, 0xa438, 0x31bf,
        0xa438, 0x8fed, 0xa438, 0x1a93, 0xa438, 0xd819, 0xa438, 0xd91b,
        0xa438, 0x46ab, 0xa438, 0x0e19, 0xa438, 0xd819, 0xa438, 0xd91b,
        0xa438, 0x46aa, 0xa438, 0x0612, 0xa438, 0xa205, 0xa438, 0xc0ae,
        0xa438, 0x0cbf, 0xa438, 0x5712, 0xa438, 0x0258, 0xa438, 0xb1bf,
        0xa438, 0x5712, 0xa438, 0x0258, 0xa438, 0xa8fe, 0xa438, 0xef96,
        0xa438, 0xfffe, 0xa438, 0xfdfc, 0xa438, 0x04f8, 0xa438, 0xfbef,
        0xa438, 0x79a2, 0xa438, 0x0005, 0xa438, 0xbf89, 0xa438, 0x1fae,
        0xa438, 0x1ba2, 0xa438, 0x0105, 0xa438, 0xbf89, 0xa438, 0x22ae,
        0xa438, 0x13a2, 0xa438, 0x0205, 0xa438, 0xbf89, 0xa438, 0x25ae,
        0xa438, 0x0ba2, 0xa438, 0x0305, 0xa438, 0xbf89, 0xa438, 0x28ae,
        0xa438, 0x03bf, 0xa438, 0x892b, 0xa438, 0x0252, 0xa438, 0xc8ef,
        0xa438, 0x64ef, 0xa438, 0x97ff, 0xa438, 0xfc04, 0xa438, 0xf8f9,
        0xa438, 0xfaef, 0xa438, 0x69fa, 0xa438, 0xe080, 0xa438, 0x15ad,
        0xa438, 0x2628, 0xa438, 0xe081, 0xa438, 0xabe1, 0xa438, 0x81ac,
        0xa438, 0xef64, 0xa438, 0xbf57, 0xa438, 0x1802, 0xa438, 0x52c8,
        0xa438, 0x1b46, 0xa438, 0xaa0a, 0xa438, 0xbf57, 0xa438, 0x1b02,
        0xa438, 0x52c8, 0xa438, 0x1b46, 0xa438, 0xab0c, 0xa438, 0xbf57,
        0xa438, 0x1502, 0xa438, 0x58b1, 0xa438, 0xbf57, 0xa438, 0x1502,
        0xa438, 0x58a8, 0xa438, 0xfeef, 0xa438, 0x96fe, 0xa438, 0xfdfc,
        0xa438, 0x04f8, 0xa438, 0xf9ef, 0xa438, 0x59f9, 0xa438, 0xe080,
        0xa438, 0x15ad, 0xa438, 0x2622, 0xa438, 0xbf53, 0xa438, 0x2202,
        0xa438, 0x52c8, 0xa438, 0x3972, 0xa438, 0x9e10, 0xa438, 0xe083,
        0xa438, 0xc9ac, 0xa438, 0x2605, 0xa438, 0x0288, 0xa438, 0x2cae,
        0xa438, 0x0d02, 0xa438, 0x8870, 0xa438, 0xae08, 0xa438, 0xe283,
        0xa438, 0xc9f6, 0xa438, 0x36e6, 0xa438, 0x83c9, 0xa438, 0xfdef,
        0xa438, 0x95fd, 0xa438, 0xfc04, 0xa438, 0xf8f9, 0xa438, 0xfafb,
        0xa438, 0xef79, 0xa438, 0xfbbf, 0xa438, 0x5718, 0xa438, 0x0252,
        0xa438, 0xc8ef, 0xa438, 0x64e2, 0xa438, 0x8fe5, 0xa438, 0xe38f,
        0xa438, 0xe61b, 0xa438, 0x659e, 0xa438, 0x10e4, 0xa438, 0x8fe5,
        0xa438, 0xe58f, 0xa438, 0xe6e2, 0xa438, 0x83c9, 0xa438, 0xf636,
        0xa438, 0xe683, 0xa438, 0xc9ae, 0xa438, 0x13e2, 0xa438, 0x83c9,
        0xa438, 0xf736, 0xa438, 0xe683, 0xa438, 0xc902, 0xa438, 0x5820,
        0xa438, 0xef57, 0xa438, 0xe68f, 0xa438, 0xe7e7, 0xa438, 0x8fe8,
        0xa438, 0xffef, 0xa438, 0x97ff, 0xa438, 0xfefd, 0xa438, 0xfc04,
        0xa438, 0xf8f9, 0xa438, 0xfafb, 0xa438, 0xef79, 0xa438, 0xfbe2,
        0xa438, 0x8fe7, 0xa438, 0xe38f, 0xa438, 0xe8ef, 0xa438, 0x65e2,
        0xa438, 0x81b8, 0xa438, 0xe381, 0xa438, 0xb9ef, 0xa438, 0x7502,
        0xa438, 0x583b, 0xa438, 0xac50, 0xa438, 0x1abf, 0xa438, 0x5718,
        0xa438, 0x0252, 0xa438, 0xc8ef, 0xa438, 0x64e2, 0xa438, 0x8fe5,
        0xa438, 0xe38f, 0xa438, 0xe61b, 0xa438, 0x659e, 0xa438, 0x1ce4,
        0xa438, 0x8fe5, 0xa438, 0xe58f, 0xa438, 0xe6ae, 0xa438, 0x0cbf,
        0xa438, 0x5715, 0xa438, 0x0258, 0xa438, 0xb1bf, 0xa438, 0x5715,
        0xa438, 0x0258, 0xa438, 0xa8e2, 0xa438, 0x83c9, 0xa438, 0xf636,
        0xa438, 0xe683, 0xa438, 0xc9ff, 0xa438, 0xef97, 0xa438, 0xfffe,
        0xa438, 0xfdfc, 0xa438, 0x04f8, 0xa438, 0xf9fa, 0xa438, 0xef69,
        0xa438, 0xe080, 0xa438, 0x15ad, 0xa438, 0x264b, 0xa438, 0xbf53,
        0xa438, 0xca02, 0xa438, 0x52c8, 0xa438, 0xad28, 0xa438, 0x42bf,
        0xa438, 0x8931, 0xa438, 0x0252, 0xa438, 0xc8ef, 0xa438, 0x54bf,
        0xa438, 0x576c, 0xa438, 0x0252, 0xa438, 0xc8a1, 0xa438, 0x001b,
        0xa438, 0xbf53, 0xa438, 0x4c02, 0xa438, 0x52c8, 0xa438, 0xac29,
        0xa438, 0x0dac, 0xa438, 0x2805, 0xa438, 0xa302, 0xa438, 0x16ae,
        0xa438, 0x20a3, 0xa438, 0x0311, 0xa438, 0xae1b, 0xa438, 0xa304,
        0xa438, 0x0cae, 0xa438, 0x16a3, 0xa438, 0x0802, 0xa438, 0xae11,
        0xa438, 0xa309, 0xa438, 0x02ae, 0xa438, 0x0cbf, 0xa438, 0x5715,
        0xa438, 0x0258, 0xa438, 0xb1bf, 0xa438, 0x5715, 0xa438, 0x0258,
        0xa438, 0xa8ef, 0xa438, 0x96fe, 0xa438, 0xfdfc, 0xa438, 0x04f0,
        0xa438, 0xa300, 0xa438, 0xf0a3, 0xa438, 0x02f0, 0xa438, 0xa304,
        0xa438, 0xf0a3, 0xa438, 0x06f0, 0xa438, 0xa308, 0xa438, 0xf0a2,
        0xa438, 0x8074, 0xa438, 0xa600, 0xa438, 0xac4f, 0xa438, 0x02ae,
        0xa438, 0x0bef, 0xa438, 0x46f6, 0xa438, 0x273c, 0xa438, 0x1000,
        0xa438, 0xab1b, 0xa438, 0xae16, 0xa438, 0xe081, 0xa438, 0xabe1,
        0xa438, 0x81ac, 0xa438, 0x1b46, 0xa438, 0xab0c, 0xa438, 0xac32,
        0xa438, 0x04ef, 0xa438, 0x32ae, 0xa438, 0x02d3, 0xa438, 0x04af,
        0xa438, 0x486c, 0xa438, 0xaf48, 0xa438, 0x82af, 0xa438, 0x4888,
        0xa438, 0xe081, 0xa438, 0x9be1, 0xa438, 0x819c, 0xa438, 0xe28f,
        0xa438, 0xe3ad, 0xa438, 0x3009, 0xa438, 0x1f55, 0xa438, 0xe38f,
        0xa438, 0xe20c, 0xa438, 0x581a, 0xa438, 0x45e4, 0xa438, 0x83a6,
        0xa438, 0xe583, 0xa438, 0xa7af, 0xa438, 0x2a75, 0xa438, 0xe08f,
        0xa438, 0xe3ad, 0xa438, 0x201c, 0xa438, 0x1f44, 0xa438, 0xe18f,
        0xa438, 0xe10c, 0xa438, 0x44ef, 0xa438, 0x64e0, 0xa438, 0x8232,
        0xa438, 0xe182, 0xa438, 0x331b, 0xa438, 0x649f, 0xa438, 0x091f,
        0xa438, 0x44e1, 0xa438, 0x8fe2, 0xa438, 0x0c48, 0xa438, 0x1b54,
        0xa438, 0xe683, 0xa438, 0xa6e7, 0xa438, 0x83a7, 0xa438, 0xaf2b,
        0xa438, 0xd900, 0xa436, 0xb818, 0xa438, 0x043d, 0xa436, 0xb81a,
        0xa438, 0x06a3, 0xa436, 0xb81c, 0xa438, 0x476d, 0xa436, 0xb81e,
        0xa438, 0x4852, 0xa436, 0xb850, 0xa438, 0x2A69, 0xa436, 0xb852,
        0xa438, 0x2BD3, 0xa436, 0xb878, 0xa438, 0xffff, 0xa436, 0xb884,
        0xa438, 0xffff, 0xa436, 0xb832, 0xa438, 0x003f, 0xb844, 0xffff,
        0xa436, 0x8fe9, 0xa438, 0x0000, 0xa436, 0x8feb, 0xa438, 0x02fe,
        0xa436, 0x8fed, 0xa438, 0x0019, 0xa436, 0x8fef, 0xa438, 0x0bdb,
        0xa436, 0x8ff1, 0xa438, 0x0ca4, 0xa436, 0x0000, 0xa438, 0x0000,
        0xa436, 0xB82E, 0xa438, 0x0000, 0xa436, 0x8024, 0xa438, 0x0000,
        0xa436, 0x801E, 0xa438, 0x0024, 0xb820, 0x0000, 0xFFFF, 0xFFFF
};

static const u16  phy_mcu_ram_code_8125d_1_1[] = {
        0xa436, 0x8023, 0xa438, 0x3800, 0xa436, 0xB82E, 0xa438, 0x0001,
        0xb820, 0x0090, 0xa436, 0xA016, 0xa438, 0x0000, 0xa436, 0xA012,
        0xa438, 0x0000, 0xa436, 0xA014, 0xa438, 0x1800, 0xa438, 0x8010,
        0xa438, 0x1800, 0xa438, 0x8018, 0xa438, 0x1800, 0xa438, 0x8021,
        0xa438, 0x1800, 0xa438, 0x8029, 0xa438, 0x1800, 0xa438, 0x8031,
        0xa438, 0x1800, 0xa438, 0x8035, 0xa438, 0x1800, 0xa438, 0x819c,
        0xa438, 0x1800, 0xa438, 0x81e9, 0xa438, 0xd711, 0xa438, 0x6081,
        0xa438, 0x8904, 0xa438, 0x1800, 0xa438, 0x2021, 0xa438, 0xa904,
        0xa438, 0x1800, 0xa438, 0x2021, 0xa438, 0xd75f, 0xa438, 0x4083,
        0xa438, 0xd503, 0xa438, 0xa908, 0xa438, 0x87f0, 0xa438, 0x1000,
        0xa438, 0x17e0, 0xa438, 0x1800, 0xa438, 0x13c3, 0xa438, 0xd707,
        0xa438, 0x2005, 0xa438, 0x8027, 0xa438, 0xd75e, 0xa438, 0x1800,
        0xa438, 0x1434, 0xa438, 0x1800, 0xa438, 0x14a5, 0xa438, 0xc504,
        0xa438, 0xce20, 0xa438, 0xcf01, 0xa438, 0xd70a, 0xa438, 0x4005,
        0xa438, 0xcf02, 0xa438, 0x1800, 0xa438, 0x1c50, 0xa438, 0xa980,
        0xa438, 0xd500, 0xa438, 0x1800, 0xa438, 0x14f3, 0xa438, 0xd75e,
        0xa438, 0x67b1, 0xa438, 0xd504, 0xa438, 0xd71e, 0xa438, 0x65bb,
        0xa438, 0x63da, 0xa438, 0x61f9, 0xa438, 0x0cf0, 0xa438, 0x0c10,
        0xa438, 0xd505, 0xa438, 0x0c0f, 0xa438, 0x0808, 0xa438, 0xd501,
        0xa438, 0xce01, 0xa438, 0x0cf0, 0xa438, 0x0470, 0xa438, 0x0cf0,
        0xa438, 0x0430, 0xa438, 0x0cf0, 0xa438, 0x0410, 0xa438, 0xf02a,
        0xa438, 0x0cf0, 0xa438, 0x0c20, 0xa438, 0xd505, 0xa438, 0x0c0f,
        0xa438, 0x0804, 0xa438, 0xd501, 0xa438, 0xce01, 0xa438, 0x0cf0,
        0xa438, 0x0470, 0xa438, 0x0cf0, 0xa438, 0x0430, 0xa438, 0x0cf0,
        0xa438, 0x0420, 0xa438, 0xf01c, 0xa438, 0x0cf0, 0xa438, 0x0c40,
        0xa438, 0xd505, 0xa438, 0x0c0f, 0xa438, 0x0802, 0xa438, 0xd501,
        0xa438, 0xce01, 0xa438, 0x0cf0, 0xa438, 0x0470, 0xa438, 0x0cf0,
        0xa438, 0x0450, 0xa438, 0x0cf0, 0xa438, 0x0440, 0xa438, 0xf00e,
        0xa438, 0x0cf0, 0xa438, 0x0c80, 0xa438, 0xd505, 0xa438, 0x0c0f,
        0xa438, 0x0801, 0xa438, 0xd501, 0xa438, 0xce01, 0xa438, 0x0cf0,
        0xa438, 0x04b0, 0xa438, 0x0cf0, 0xa438, 0x0490, 0xa438, 0x0cf0,
        0xa438, 0x0480, 0xa438, 0xd501, 0xa438, 0xce00, 0xa438, 0xd500,
        0xa438, 0xc48e, 0xa438, 0x1000, 0xa438, 0x1a41, 0xa438, 0xd718,
        0xa438, 0x5faf, 0xa438, 0xd504, 0xa438, 0x8e01, 0xa438, 0x8c0f,
        0xa438, 0xd500, 0xa438, 0x1000, 0xa438, 0x17e0, 0xa438, 0xd504,
        0xa438, 0xd718, 0xa438, 0x4074, 0xa438, 0x6195, 0xa438, 0xf005,
        0xa438, 0x60f5, 0xa438, 0x0c03, 0xa438, 0x0d00, 0xa438, 0xf009,
        0xa438, 0x0c03, 0xa438, 0x0d01, 0xa438, 0xf006, 0xa438, 0x0c03,
        0xa438, 0x0d02, 0xa438, 0xf003, 0xa438, 0x0c03, 0xa438, 0x0d03,
        0xa438, 0xd500, 0xa438, 0xd706, 0xa438, 0x2529, 0xa438, 0x809c,
        0xa438, 0xd718, 0xa438, 0x607b, 0xa438, 0x40da, 0xa438, 0xf00f,
        0xa438, 0x431a, 0xa438, 0xf021, 0xa438, 0xd718, 0xa438, 0x617b,
        0xa438, 0x1000, 0xa438, 0x1a41, 0xa438, 0x1000, 0xa438, 0x1ad1,
        0xa438, 0xd718, 0xa438, 0x608e, 0xa438, 0xd73e, 0xa438, 0x5f34,
        0xa438, 0xf020, 0xa438, 0xf053, 0xa438, 0x1000, 0xa438, 0x1a41,
        0xa438, 0x1000, 0xa438, 0x1ad1, 0xa438, 0xd718, 0xa438, 0x608e,
        0xa438, 0xd73e, 0xa438, 0x5f34, 0xa438, 0xf023, 0xa438, 0xf067,
        0xa438, 0x1000, 0xa438, 0x1a41, 0xa438, 0x1000, 0xa438, 0x1ad1,
        0xa438, 0xd718, 0xa438, 0x608e, 0xa438, 0xd73e, 0xa438, 0x5f34,
        0xa438, 0xf026, 0xa438, 0xf07b, 0xa438, 0x1000, 0xa438, 0x1a41,
        0xa438, 0x1000, 0xa438, 0x1ad1, 0xa438, 0xd718, 0xa438, 0x608e,
        0xa438, 0xd73e, 0xa438, 0x5f34, 0xa438, 0xf029, 0xa438, 0xf08f,
        0xa438, 0x1000, 0xa438, 0x8173, 0xa438, 0x1000, 0xa438, 0x1a41,
        0xa438, 0xd73e, 0xa438, 0x7fb4, 0xa438, 0x1000, 0xa438, 0x8188,
        0xa438, 0x1000, 0xa438, 0x1a41, 0xa438, 0xd718, 0xa438, 0x5fae,
        0xa438, 0xf028, 0xa438, 0x1000, 0xa438, 0x8173, 0xa438, 0x1000,
        0xa438, 0x1a41, 0xa438, 0xd73e, 0xa438, 0x7fb4, 0xa438, 0x1000,
        0xa438, 0x8188, 0xa438, 0x1000, 0xa438, 0x1a41, 0xa438, 0xd718,
        0xa438, 0x5fae, 0xa438, 0xf039, 0xa438, 0x1000, 0xa438, 0x8173,
        0xa438, 0x1000, 0xa438, 0x1a41, 0xa438, 0xd73e, 0xa438, 0x7fb4,
        0xa438, 0x1000, 0xa438, 0x8188, 0xa438, 0x1000, 0xa438, 0x1a41,
        0xa438, 0xd718, 0xa438, 0x5fae, 0xa438, 0xf04a, 0xa438, 0x1000,
        0xa438, 0x8173, 0xa438, 0x1000, 0xa438, 0x1a41, 0xa438, 0xd73e,
        0xa438, 0x7fb4, 0xa438, 0x1000, 0xa438, 0x8188, 0xa438, 0x1000,
        0xa438, 0x1a41, 0xa438, 0xd718, 0xa438, 0x5fae, 0xa438, 0xf05b,
        0xa438, 0xd719, 0xa438, 0x4119, 0xa438, 0xd504, 0xa438, 0xac01,
        0xa438, 0xae01, 0xa438, 0xd500, 0xa438, 0x1000, 0xa438, 0x1a2f,
        0xa438, 0xf00a, 0xa438, 0xd719, 0xa438, 0x4118, 0xa438, 0xd504,
        0xa438, 0xac11, 0xa438, 0xd501, 0xa438, 0xce01, 0xa438, 0xa410,
        0xa438, 0xce00, 0xa438, 0xd500, 0xa438, 0x1000, 0xa438, 0x1a41,
        0xa438, 0xd718, 0xa438, 0x5fb0, 0xa438, 0xd505, 0xa438, 0xd719,
        0xa438, 0x4079, 0xa438, 0xa80f, 0xa438, 0xf05d, 0xa438, 0x4b98,
        0xa438, 0xa808, 0xa438, 0xf05a, 0xa438, 0xd719, 0xa438, 0x4119,
        0xa438, 0xd504, 0xa438, 0xac02, 0xa438, 0xae01, 0xa438, 0xd500,
        0xa438, 0x1000, 0xa438, 0x1a2f, 0xa438, 0xf00a, 0xa438, 0xd719,
        0xa438, 0x4118, 0xa438, 0xd504, 0xa438, 0xac22, 0xa438, 0xd501,
        0xa438, 0xce01, 0xa438, 0xa420, 0xa438, 0xce00, 0xa438, 0xd500,
        0xa438, 0x1000, 0xa438, 0x1a41, 0xa438, 0xd718, 0xa438, 0x5fb0,
        0xa438, 0xd505, 0xa438, 0xd719, 0xa438, 0x4079, 0xa438, 0xa80f,
        0xa438, 0xf03f, 0xa438, 0x47d8, 0xa438, 0xa804, 0xa438, 0xf03c,
        0xa438, 0xd719, 0xa438, 0x4119, 0xa438, 0xd504, 0xa438, 0xac04,
        0xa438, 0xae01, 0xa438, 0xd500, 0xa438, 0x1000, 0xa438, 0x1a2f,
        0xa438, 0xf00a, 0xa438, 0xd719, 0xa438, 0x4118, 0xa438, 0xd504,
        0xa438, 0xac44, 0xa438, 0xd501, 0xa438, 0xce01, 0xa438, 0xa440,
        0xa438, 0xce00, 0xa438, 0xd500, 0xa438, 0x1000, 0xa438, 0x1a41,
        0xa438, 0xd718, 0xa438, 0x5fb0, 0xa438, 0xd505, 0xa438, 0xd719,
        0xa438, 0x4079, 0xa438, 0xa80f, 0xa438, 0xf021, 0xa438, 0x4418,
        0xa438, 0xa802, 0xa438, 0xf01e, 0xa438, 0xd719, 0xa438, 0x4119,
        0xa438, 0xd504, 0xa438, 0xac08, 0xa438, 0xae01, 0xa438, 0xd500,
        0xa438, 0x1000, 0xa438, 0x1a2f, 0xa438, 0xf00a, 0xa438, 0xd719,
        0xa438, 0x4118, 0xa438, 0xd504, 0xa438, 0xac88, 0xa438, 0xd501,
        0xa438, 0xce01, 0xa438, 0xa480, 0xa438, 0xce00, 0xa438, 0xd500,
        0xa438, 0x1000, 0xa438, 0x1a41, 0xa438, 0xd718, 0xa438, 0x5fb0,
        0xa438, 0xd505, 0xa438, 0xd719, 0xa438, 0x4079, 0xa438, 0xa80f,
        0xa438, 0xf003, 0xa438, 0x4058, 0xa438, 0xa801, 0xa438, 0x1800,
        0xa438, 0x16ed, 0xa438, 0xd73e, 0xa438, 0xd505, 0xa438, 0x3088,
        0xa438, 0x817a, 0xa438, 0x6193, 0xa438, 0x6132, 0xa438, 0x60d1,
        0xa438, 0x3298, 0xa438, 0x8185, 0xa438, 0xf00a, 0xa438, 0xa808,
        0xa438, 0xf008, 0xa438, 0xa804, 0xa438, 0xf006, 0xa438, 0xa802,
        0xa438, 0xf004, 0xa438, 0xa801, 0xa438, 0xf002, 0xa438, 0xa80f,
        0xa438, 0xd500, 0xa438, 0x0800, 0xa438, 0xd505, 0xa438, 0xd75e,
        0xa438, 0x6211, 0xa438, 0xd71e, 0xa438, 0x619b, 0xa438, 0x611a,
        0xa438, 0x6099, 0xa438, 0x0c0f, 0xa438, 0x0808, 0xa438, 0xf009,
        0xa438, 0x0c0f, 0xa438, 0x0804, 0xa438, 0xf006, 0xa438, 0x0c0f,
        0xa438, 0x0802, 0xa438, 0xf003, 0xa438, 0x0c0f, 0xa438, 0x0801,
        0xa438, 0xd500, 0xa438, 0x0800, 0xa438, 0xd500, 0xa438, 0xc48d,
        0xa438, 0xd504, 0xa438, 0x8d03, 0xa438, 0xd701, 0xa438, 0x4045,
        0xa438, 0xad02, 0xa438, 0xd504, 0xa438, 0xd706, 0xa438, 0x2529,
        0xa438, 0x81ad, 0xa438, 0xd718, 0xa438, 0x607b, 0xa438, 0x40da,
        0xa438, 0xf013, 0xa438, 0x441a, 0xa438, 0xf02d, 0xa438, 0xd718,
        0xa438, 0x61fb, 0xa438, 0xbb01, 0xa438, 0xd75e, 0xa438, 0x6171,
        0xa438, 0x0cf0, 0xa438, 0x0c10, 0xa438, 0xd501, 0xa438, 0xce01,
        0xa438, 0x0cf0, 0xa438, 0x0410, 0xa438, 0xce00, 0xa438, 0xd505,
        0xa438, 0x0c0f, 0xa438, 0x0808, 0xa438, 0xf02a, 0xa438, 0xbb02,
        0xa438, 0xd75e, 0xa438, 0x6171, 0xa438, 0x0cf0, 0xa438, 0x0c20,
        0xa438, 0xd501, 0xa438, 0xce01, 0xa438, 0x0cf0, 0xa438, 0x0420,
        0xa438, 0xce00, 0xa438, 0xd505, 0xa438, 0x0c0f, 0xa438, 0x0804,
        0xa438, 0xf01c, 0xa438, 0xbb04, 0xa438, 0xd75e, 0xa438, 0x6171,
        0xa438, 0x0cf0, 0xa438, 0x0c40, 0xa438, 0xd501, 0xa438, 0xce01,
        0xa438, 0x0cf0, 0xa438, 0x0440, 0xa438, 0xce00, 0xa438, 0xd505,
        0xa438, 0x0c0f, 0xa438, 0x0802, 0xa438, 0xf00e, 0xa438, 0xbb08,
        0xa438, 0xd75e, 0xa438, 0x6171, 0xa438, 0x0cf0, 0xa438, 0x0c80,
        0xa438, 0xd501, 0xa438, 0xce01, 0xa438, 0x0cf0, 0xa438, 0x0480,
        0xa438, 0xce00, 0xa438, 0xd505, 0xa438, 0x0c0f, 0xa438, 0x0801,
        0xa438, 0xd500, 0xa438, 0x1800, 0xa438, 0x1616, 0xa436, 0xA026,
        0xa438, 0xffff, 0xa436, 0xA024, 0xa438, 0x15d8, 0xa436, 0xA022,
        0xa438, 0x161f, 0xa436, 0xA020, 0xa438, 0x14f2, 0xa436, 0xA006,
        0xa438, 0x1c4f, 0xa436, 0xA004, 0xa438, 0x1433, 0xa436, 0xA002,
        0xa438, 0x13c1, 0xa436, 0xA000, 0xa438, 0x2020, 0xa436, 0xA008,
        0xa438, 0x7f00, 0xa436, 0xA016, 0xa438, 0x0000, 0xa436, 0xA012,
        0xa438, 0x07f8, 0xa436, 0xA014, 0xa438, 0xd04d, 0xa438, 0x8904,
        0xa438, 0x813C, 0xa438, 0xA13D, 0xa438, 0xcc01, 0xa438, 0x0000,
        0xa438, 0x0000, 0xa438, 0x0000, 0xa436, 0xA152, 0xa438, 0x1384,
        0xa436, 0xA154, 0xa438, 0x1fa8, 0xa436, 0xA156, 0xa438, 0x218B,
        0xa436, 0xA158, 0xa438, 0x21B8, 0xa436, 0xA15A, 0xa438, 0x021c,
        0xa436, 0xA15C, 0xa438, 0x3fff, 0xa436, 0xA15E, 0xa438, 0x3fff,
        0xa436, 0xA160, 0xa438, 0x3fff, 0xa436, 0xA150, 0xa438, 0x001f,
        0xa436, 0xA016, 0xa438, 0x0010, 0xa436, 0xA012, 0xa438, 0x0000,
        0xa436, 0xA014, 0xa438, 0x1800, 0xa438, 0x8010, 0xa438, 0x1800,
        0xa438, 0x8013, 0xa438, 0x1800, 0xa438, 0x803a, 0xa438, 0x1800,
        0xa438, 0x8045, 0xa438, 0x1800, 0xa438, 0x8049, 0xa438, 0x1800,
        0xa438, 0x804d, 0xa438, 0x1800, 0xa438, 0x8059, 0xa438, 0x1800,
        0xa438, 0x805d, 0xa438, 0xc2ff, 0xa438, 0x1800, 0xa438, 0x0042,
        0xa438, 0x1000, 0xa438, 0x02e5, 0xa438, 0x1000, 0xa438, 0x02b4,
        0xa438, 0xd701, 0xa438, 0x40e3, 0xa438, 0xd700, 0xa438, 0x5f6c,
        0xa438, 0x1000, 0xa438, 0x8021, 0xa438, 0x1800, 0xa438, 0x0073,
        0xa438, 0x1800, 0xa438, 0x0084, 0xa438, 0xd701, 0xa438, 0x4061,
        0xa438, 0xba0f, 0xa438, 0xf004, 0xa438, 0x4060, 0xa438, 0x1000,
        0xa438, 0x802a, 0xa438, 0xba10, 0xa438, 0x0800, 0xa438, 0xd700,
        0xa438, 0x60bb, 0xa438, 0x611c, 0xa438, 0x0c0f, 0xa438, 0x1a01,
        0xa438, 0xf00a, 0xa438, 0x60fc, 0xa438, 0x0c0f, 0xa438, 0x1a02,
        0xa438, 0xf006, 0xa438, 0x0c0f, 0xa438, 0x1a04, 0xa438, 0xf003,
        0xa438, 0x0c0f, 0xa438, 0x1a08, 0xa438, 0x0800, 0xa438, 0x0c0f,
        0xa438, 0x0504, 0xa438, 0xad02, 0xa438, 0x1000, 0xa438, 0x02c0,
        0xa438, 0xd700, 0xa438, 0x5fac, 0xa438, 0x1000, 0xa438, 0x8021,
        0xa438, 0x1800, 0xa438, 0x0139, 0xa438, 0x9a1f, 0xa438, 0x8bf0,
        0xa438, 0x1800, 0xa438, 0x02df, 0xa438, 0x9a1f, 0xa438, 0x9910,
        0xa438, 0x1800, 0xa438, 0x02d7, 0xa438, 0xad02, 0xa438, 0x8d01,
        0xa438, 0x9a1f, 0xa438, 0x9910, 0xa438, 0x9860, 0xa438, 0xcb00,
        0xa438, 0xd501, 0xa438, 0xce01, 0xa438, 0x85f0, 0xa438, 0xd500,
        0xa438, 0x1800, 0xa438, 0x015c, 0xa438, 0x8580, 0xa438, 0x8d02,
        0xa438, 0x1800, 0xa438, 0x018f, 0xa438, 0x0c0f, 0xa438, 0x0503,
        0xa438, 0xad02, 0xa438, 0x1800, 0xa438, 0x00dd, 0xa436, 0xA08E,
        0xa438, 0x00db, 0xa436, 0xA08C, 0xa438, 0x018e, 0xa436, 0xA08A,
        0xa438, 0x015a, 0xa436, 0xA088, 0xa438, 0x02d6, 0xa436, 0xA086,
        0xa438, 0x02de, 0xa436, 0xA084, 0xa438, 0x0137, 0xa436, 0xA082,
        0xa438, 0x0071, 0xa436, 0xA080, 0xa438, 0x0041, 0xa436, 0xA090,
        0xa438, 0x00ff, 0xa436, 0xA016, 0xa438, 0x0020, 0xa436, 0xA012,
        0xa438, 0x1ff8, 0xa436, 0xA014, 0xa438, 0x001c, 0xa438, 0xce15,
        0xa438, 0xd105, 0xa438, 0xa410, 0xa438, 0x8320, 0xa438, 0xFFD7,
        0xa438, 0x0000, 0xa438, 0x0000, 0xa436, 0xA164, 0xa438, 0x0260,
        0xa436, 0xA166, 0xa438, 0x0add, 0xa436, 0xA168, 0xa438, 0x05CC,
        0xa436, 0xA16A, 0xa438, 0x05C5, 0xa436, 0xA16C, 0xa438, 0x0429,
        0xa436, 0xA16E, 0xa438, 0x07B6, 0xa436, 0xA170, 0xa438, 0x0259,
        0xa436, 0xA172, 0xa438, 0x3fff, 0xa436, 0xA162, 0xa438, 0x003f,
        0xa436, 0xA016, 0xa438, 0x0020, 0xa436, 0xA012, 0xa438, 0x0000,
        0xa436, 0xA014, 0xa438, 0x1800, 0xa438, 0x8010, 0xa438, 0x1800,
        0xa438, 0x8023, 0xa438, 0x1800, 0xa438, 0x814c, 0xa438, 0x1800,
        0xa438, 0x8156, 0xa438, 0x1800, 0xa438, 0x815e, 0xa438, 0x1800,
        0xa438, 0x8210, 0xa438, 0x1800, 0xa438, 0x8221, 0xa438, 0x1800,
        0xa438, 0x822f, 0xa438, 0xa801, 0xa438, 0x9308, 0xa438, 0xb201,
        0xa438, 0xb301, 0xa438, 0xd701, 0xa438, 0x4000, 0xa438, 0xd2ff,
        0xa438, 0xb302, 0xa438, 0xd200, 0xa438, 0xb201, 0xa438, 0xb309,
        0xa438, 0xd701, 0xa438, 0x4000, 0xa438, 0xd2ff, 0xa438, 0xb302,
        0xa438, 0xd200, 0xa438, 0xa800, 0xa438, 0x1800, 0xa438, 0x0031,
        0xa438, 0xd700, 0xa438, 0x4543, 0xa438, 0xd71f, 0xa438, 0x40fe,
        0xa438, 0xd1b7, 0xa438, 0xd049, 0xa438, 0x1000, 0xa438, 0x109e,
        0xa438, 0xd700, 0xa438, 0x5fbb, 0xa438, 0xa220, 0xa438, 0x8501,
        0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0x0c70, 0xa438, 0x0b00,
        0xa438, 0x0c07, 0xa438, 0x0604, 0xa438, 0x9503, 0xa438, 0xa510,
        0xa438, 0xce49, 0xa438, 0x1000, 0xa438, 0x10be, 0xa438, 0x8520,
        0xa438, 0xa520, 0xa438, 0xa501, 0xa438, 0xd105, 0xa438, 0xd047,
        0xa438, 0x1000, 0xa438, 0x109e, 0xa438, 0xd707, 0xa438, 0x6087,
        0xa438, 0xd700, 0xa438, 0x5f7b, 0xa438, 0xffe9, 0xa438, 0x1000,
        0xa438, 0x109e, 0xa438, 0x8501, 0xa438, 0xd707, 0xa438, 0x5e08,
        0xa438, 0x8530, 0xa438, 0xba20, 0xa438, 0xf00c, 0xa438, 0xd700,
        0xa438, 0x4098, 0xa438, 0xd1ef, 0xa438, 0xd047, 0xa438, 0xf003,
        0xa438, 0xd1db, 0xa438, 0xd040, 0xa438, 0x1000, 0xa438, 0x109e,
        0xa438, 0xd700, 0xa438, 0x5fbb, 0xa438, 0x8980, 0xa438, 0xd702,
        0xa438, 0x6126, 0xa438, 0xd704, 0xa438, 0x4063, 0xa438, 0xd702,
        0xa438, 0x6060, 0xa438, 0xd702, 0xa438, 0x6077, 0xa438, 0x8410,
        0xa438, 0xf002, 0xa438, 0xa410, 0xa438, 0xce02, 0xa438, 0x1000,
        0xa438, 0x10be, 0xa438, 0xcd81, 0xa438, 0xd412, 0xa438, 0x1000,
        0xa438, 0x1069, 0xa438, 0xcd82, 0xa438, 0xd40e, 0xa438, 0x1000,
        0xa438, 0x1069, 0xa438, 0xcd83, 0xa438, 0x1000, 0xa438, 0x109e,
        0xa438, 0xd71f, 0xa438, 0x5fb4, 0xa438, 0xd702, 0xa438, 0x6c26,
        0xa438, 0xd704, 0xa438, 0x4063, 0xa438, 0xd702, 0xa438, 0x6060,
        0xa438, 0xd702, 0xa438, 0x6b77, 0xa438, 0xa340, 0xa438, 0x0c06,
        0xa438, 0x0102, 0xa438, 0xce01, 0xa438, 0x1000, 0xa438, 0x10be,
        0xa438, 0xa240, 0xa438, 0xa902, 0xa438, 0xa204, 0xa438, 0xa280,
        0xa438, 0xa364, 0xa438, 0xab02, 0xa438, 0x8380, 0xa438, 0xa00a,
        0xa438, 0xcd8d, 0xa438, 0x1000, 0xa438, 0x109e, 0xa438, 0xd706,
        0xa438, 0x5fb5, 0xa438, 0xb920, 0xa438, 0x1000, 0xa438, 0x109e,
        0xa438, 0xd71f, 0xa438, 0x7fb4, 0xa438, 0x9920, 0xa438, 0x1000,
        0xa438, 0x109e, 0xa438, 0xd71f, 0xa438, 0x6065, 0xa438, 0x7c74,
        0xa438, 0xfffb, 0xa438, 0xb820, 0xa438, 0x1000, 0xa438, 0x109e,
        0xa438, 0xd71f, 0xa438, 0x7fa5, 0xa438, 0x9820, 0xa438, 0xa410,
        0xa438, 0x8902, 0xa438, 0xa120, 0xa438, 0xa380, 0xa438, 0xce02,
        0xa438, 0x1000, 0xa438, 0x10be, 0xa438, 0x8280, 0xa438, 0xa324,
        0xa438, 0xab02, 0xa438, 0xa00a, 0xa438, 0x8118, 0xa438, 0x863f,
        0xa438, 0x87fb, 0xa438, 0xcd8e, 0xa438, 0xd193, 0xa438, 0xd047,
        0xa438, 0x1000, 0xa438, 0x109e, 0xa438, 0x1000, 0xa438, 0x10a3,
        0xa438, 0xd700, 0xa438, 0x5f7b, 0xa438, 0xa280, 0xa438, 0x1000,
        0xa438, 0x109e, 0xa438, 0x1000, 0xa438, 0x10a3, 0xa438, 0xd706,
        0xa438, 0x5f78, 0xa438, 0xa210, 0xa438, 0xd700, 0xa438, 0x6083,
        0xa438, 0xd101, 0xa438, 0xd047, 0xa438, 0xf003, 0xa438, 0xd160,
        0xa438, 0xd04b, 0xa438, 0x1000, 0xa438, 0x109e, 0xa438, 0x1000,
        0xa438, 0x10a3, 0xa438, 0xd700, 0xa438, 0x5f7b, 0xa438, 0x1000,
        0xa438, 0x109e, 0xa438, 0x1000, 0xa438, 0x10a3, 0xa438, 0xd706,
        0xa438, 0x5f79, 0xa438, 0x8120, 0xa438, 0xbb20, 0xa438, 0xf04c,
        0xa438, 0xa00a, 0xa438, 0xa340, 0xa438, 0x0c06, 0xa438, 0x0102,
        0xa438, 0xa240, 0xa438, 0xa290, 0xa438, 0xa324, 0xa438, 0xab02,
        0xa438, 0xd13e, 0xa438, 0xd05a, 0xa438, 0xd13e, 0xa438, 0xd06b,
        0xa438, 0xcd84, 0xa438, 0x1000, 0xa438, 0x109e, 0xa438, 0xd706,
        0xa438, 0x6079, 0xa438, 0xd700, 0xa438, 0x5f5c, 0xa438, 0xcd8a,
        0xa438, 0x1000, 0xa438, 0x109e, 0xa438, 0xd706, 0xa438, 0x6079,
        0xa438, 0xd700, 0xa438, 0x5f5d, 0xa438, 0xcd8b, 0xa438, 0x1000,
        0xa438, 0x109e, 0xa438, 0xcd8c, 0xa438, 0xd700, 0xa438, 0x6050,
        0xa438, 0xab04, 0xa438, 0xd700, 0xa438, 0x4083, 0xa438, 0xd160,
        0xa438, 0xd04b, 0xa438, 0xf003, 0xa438, 0xd193, 0xa438, 0xd047,
        0xa438, 0x1000, 0xa438, 0x109e, 0xa438, 0xd700, 0xa438, 0x5fbb,
        0xa438, 0x1000, 0xa438, 0x109e, 0xa438, 0x8410, 0xa438, 0xd71f,
        0xa438, 0x5f94, 0xa438, 0xb920, 0xa438, 0x1000, 0xa438, 0x109e,
        0xa438, 0xd71f, 0xa438, 0x7fb4, 0xa438, 0x9920, 0xa438, 0x1000,
        0xa438, 0x109e, 0xa438, 0xd71f, 0xa438, 0x6105, 0xa438, 0x6054,
        0xa438, 0xfffb, 0xa438, 0x1000, 0xa438, 0x109e, 0xa438, 0xd706,
        0xa438, 0x5fb9, 0xa438, 0xfff0, 0xa438, 0xa410, 0xa438, 0xb820,
        0xa438, 0xcd85, 0xa438, 0x1000, 0xa438, 0x109e, 0xa438, 0xd71f,
        0xa438, 0x7fa5, 0xa438, 0x9820, 0xa438, 0xbb20, 0xa438, 0xd105,
        0xa438, 0xd042, 0xa438, 0x1000, 0xa438, 0x109e, 0xa438, 0xd706,
        0xa438, 0x5fbb, 0xa438, 0x5f85, 0xa438, 0xd700, 0xa438, 0x5f5b,
        0xa438, 0xd700, 0xa438, 0x6090, 0xa438, 0xd700, 0xa438, 0x4043,
        0xa438, 0xaa20, 0xa438, 0xcd86, 0xa438, 0xd700, 0xa438, 0x6083,
        0xa438, 0xd1c7, 0xa438, 0xd045, 0xa438, 0xf003, 0xa438, 0xd17a,
        0xa438, 0xd04b, 0xa438, 0x1000, 0xa438, 0x109e, 0xa438, 0xd700,
        0xa438, 0x5fbb, 0xa438, 0x0c18, 0xa438, 0x0108, 0xa438, 0x0c3f,
        0xa438, 0x0609, 0xa438, 0x0cfb, 0xa438, 0x0729, 0xa438, 0xa308,
        0xa438, 0x8320, 0xa438, 0xd105, 0xa438, 0xd042, 0xa438, 0x1000,
        0xa438, 0x109e, 0xa438, 0xd700, 0xa438, 0x5fbb, 0xa438, 0x1800,
        0xa438, 0x08f7, 0xa438, 0x1000, 0xa438, 0x109e, 0xa438, 0x1000,
        0xa438, 0x10a3, 0xa438, 0xd700, 0xa438, 0x607b, 0xa438, 0xd700,
        0xa438, 0x5f2b, 0xa438, 0x1800, 0xa438, 0x0a81, 0xa438, 0xd700,
        0xa438, 0x40bd, 0xa438, 0xd707, 0xa438, 0x4065, 0xa438, 0x1800,
        0xa438, 0x1121, 0xa438, 0x1800, 0xa438, 0x1124, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0x8f80, 0xa438, 0x9503, 0xa438, 0xd705,
        0xa438, 0x641d, 0xa438, 0xd704, 0xa438, 0x62b2, 0xa438, 0xd702,
        0xa438, 0x4116, 0xa438, 0xce15, 0xa438, 0x1000, 0xa438, 0x10be,
        0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0x8f40, 0xa438, 0x9503,
        0xa438, 0xa00a, 0xa438, 0xd704, 0xa438, 0x4247, 0xa438, 0xd700,
        0xa438, 0x3691, 0xa438, 0x8183, 0xa438, 0x0c03, 0xa438, 0x1502,
        0xa438, 0xa570, 0xa438, 0x9503, 0xa438, 0xf00a, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0xaf40, 0xa438, 0x9503, 0xa438, 0x800a,
        0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0x8570, 0xa438, 0x9503,
        0xa438, 0x1000, 0xa438, 0x109e, 0xa438, 0x1000, 0xa438, 0x1108,
        0xa438, 0xcd64, 0xa438, 0xd704, 0xa438, 0x3398, 0xa438, 0x8203,
        0xa438, 0xd71f, 0xa438, 0x620e, 0xa438, 0xd704, 0xa438, 0x6096,
        0xa438, 0xd705, 0xa438, 0x6051, 0xa438, 0xf004, 0xa438, 0xd705,
        0xa438, 0x605d, 0xa438, 0xf008, 0xa438, 0xd706, 0xa438, 0x609d,
        0xa438, 0xd705, 0xa438, 0x405f, 0xa438, 0xf003, 0xa438, 0xd700,
        0xa438, 0x58fb, 0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0xc7aa,
        0xa438, 0x9503, 0xa438, 0xd71f, 0xa438, 0x6d2e, 0xa438, 0xd704,
        0xa438, 0x6096, 0xa438, 0xd705, 0xa438, 0x6051, 0xa438, 0xf005,
        0xa438, 0xd705, 0xa438, 0x607d, 0xa438, 0x1800, 0xa438, 0x0cc7,
        0xa438, 0xd706, 0xa438, 0x60bd, 0xa438, 0xd705, 0xa438, 0x407f,
        0xa438, 0x1800, 0xa438, 0x0e42, 0xa438, 0xd702, 0xa438, 0x40a4,
        0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0x8e20, 0xa438, 0x9503,
        0xa438, 0xd702, 0xa438, 0x40a5, 0xa438, 0x0c03, 0xa438, 0x1502,
        0xa438, 0x8e40, 0xa438, 0x9503, 0xa438, 0xd705, 0xa438, 0x659d,
        0xa438, 0xd704, 0xa438, 0x62b2, 0xa438, 0xd702, 0xa438, 0x4116,
        0xa438, 0xce15, 0xa438, 0x1000, 0xa438, 0x10be, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0x8f40, 0xa438, 0x9503, 0xa438, 0xa00a,
        0xa438, 0xd704, 0xa438, 0x4247, 0xa438, 0xd700, 0xa438, 0x3691,
        0xa438, 0x81de, 0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0xa570,
        0xa438, 0x9503, 0xa438, 0xf00a, 0xa438, 0x0c03, 0xa438, 0x1502,
        0xa438, 0xaf40, 0xa438, 0x9503, 0xa438, 0x800a, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0x8570, 0xa438, 0x9503, 0xa438, 0xd706,
        0xa438, 0x60e4, 0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0x0cf0,
        0xa438, 0x07a0, 0xa438, 0x9503, 0xa438, 0xf005, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0x87f0, 0xa438, 0x9503, 0xa438, 0x1000,
        0xa438, 0x109e, 0xa438, 0x1000, 0xa438, 0x1108, 0xa438, 0xcd61,
        0xa438, 0xd704, 0xa438, 0x3398, 0xa438, 0x8203, 0xa438, 0xd704,
        0xa438, 0x6096, 0xa438, 0xd705, 0xa438, 0x6051, 0xa438, 0xf005,
        0xa438, 0xd705, 0xa438, 0x607d, 0xa438, 0x1800, 0xa438, 0x0cc7,
        0xa438, 0xd71f, 0xa438, 0x61ce, 0xa438, 0xd706, 0xa438, 0x767d,
        0xa438, 0xd705, 0xa438, 0x563f, 0xa438, 0x1800, 0xa438, 0x0e42,
        0xa438, 0x800a, 0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0xae40,
        0xa438, 0x9503, 0xa438, 0x1800, 0xa438, 0x0c47, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0xaf80, 0xa438, 0x9503, 0xa438, 0x1800,
        0xa438, 0x0b5f, 0xa438, 0x607c, 0xa438, 0x1800, 0xa438, 0x027a,
        0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0xae01, 0xa438, 0x9503,
        0xa438, 0x1000, 0xa438, 0x109e, 0xa438, 0xd702, 0xa438, 0x5fa3,
        0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0x8e01, 0xa438, 0x9503,
        0xa438, 0x1800, 0xa438, 0x027d, 0xa438, 0x1000, 0xa438, 0x10be,
        0xa438, 0xd702, 0xa438, 0x40a5, 0xa438, 0x0c03, 0xa438, 0x1502,
        0xa438, 0x8e40, 0xa438, 0x9503, 0xa438, 0xd73e, 0xa438, 0x6065,
        0xa438, 0x1800, 0xa438, 0x0cea, 0xa438, 0x1800, 0xa438, 0x0cf4,
        0xa438, 0xd701, 0xa438, 0x6fd1, 0xa438, 0xd71f, 0xa438, 0x6eee,
        0xa438, 0xd707, 0xa438, 0x4d0f, 0xa438, 0xd73e, 0xa438, 0x4cc5,
        0xa438, 0xd705, 0xa438, 0x4c99, 0xa438, 0xd704, 0xa438, 0x6c57,
        0xa438, 0xd702, 0xa438, 0x6c11, 0xa438, 0x0c03, 0xa438, 0x1502,
        0xa438, 0x8c20, 0xa438, 0xa608, 0xa438, 0x9503, 0xa438, 0xa201,
        0xa438, 0xa804, 0xa438, 0xd704, 0xa438, 0x40a7, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0xa620, 0xa438, 0x9503, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0xac40, 0xa438, 0x9503, 0xa438, 0x800a,
        0xa438, 0x8290, 0xa438, 0x8306, 0xa438, 0x8b02, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0x8570, 0xa438, 0x9503, 0xa438, 0xce00,
        0xa438, 0x1000, 0xa438, 0x10be, 0xa438, 0xcd99, 0xa438, 0x1000,
        0xa438, 0x109e, 0xa438, 0x1000, 0xa438, 0x10cc, 0xa438, 0xd701,
        0xa438, 0x69f1, 0xa438, 0xd71f, 0xa438, 0x690e, 0xa438, 0xd73e,
        0xa438, 0x5ee6, 0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0x87f0,
        0xa438, 0x9503, 0xa438, 0xce46, 0xa438, 0x1000, 0xa438, 0x10be,
        0xa438, 0xa00a, 0xa438, 0xd704, 0xa438, 0x40a7, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0xa570, 0xa438, 0x9503, 0xa438, 0xcd9a,
        0xa438, 0xd700, 0xa438, 0x6078, 0xa438, 0xd700, 0xa438, 0x609a,
        0xa438, 0xd109, 0xa438, 0xd074, 0xa438, 0xf003, 0xa438, 0xd109,
        0xa438, 0xd075, 0xa438, 0x1000, 0xa438, 0x109e, 0xa438, 0x1000,
        0xa438, 0x10cc, 0xa438, 0xd701, 0xa438, 0x65b1, 0xa438, 0xd71f,
        0xa438, 0x64ce, 0xa438, 0xd700, 0xa438, 0x5efe, 0xa438, 0xce00,
        0xa438, 0x1000, 0xa438, 0x10be, 0xa438, 0x0c03, 0xa438, 0x1502,
        0xa438, 0x8608, 0xa438, 0x8c40, 0xa438, 0x9503, 0xa438, 0x8201,
        0xa438, 0x800a, 0xa438, 0x8290, 0xa438, 0x8306, 0xa438, 0x8b02,
        0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0xc7aa, 0xa438, 0x8570,
        0xa438, 0x8d08, 0xa438, 0x9503, 0xa438, 0xcd9b, 0xa438, 0x1800,
        0xa438, 0x0c8b, 0xa438, 0x1000, 0xa438, 0x109e, 0xa438, 0xd705,
        0xa438, 0x61d9, 0xa438, 0xd704, 0xa438, 0x4193, 0xa438, 0x800a,
        0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0xae40, 0xa438, 0x9503,
        0xa438, 0x1800, 0xa438, 0x0c47, 0xa438, 0x1800, 0xa438, 0x0df8,
        0xa438, 0x1800, 0xa438, 0x8339, 0xa438, 0x0800, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0x8d08, 0xa438, 0x8f02, 0xa438, 0x8c40,
        0xa438, 0x9503, 0xa438, 0x8201, 0xa438, 0xa804, 0xa438, 0xd704,
        0xa438, 0x40a7, 0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0xa620,
        0xa438, 0x9503, 0xa438, 0x800a, 0xa438, 0x8290, 0xa438, 0x8306,
        0xa438, 0x8b02, 0xa438, 0x8010, 0xa438, 0x0c03, 0xa438, 0x1502,
        0xa438, 0x8570, 0xa438, 0x9503, 0xa438, 0xaa03, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0xac20, 0xa438, 0xa608, 0xa438, 0x9503,
        0xa438, 0xce00, 0xa438, 0x1000, 0xa438, 0x10be, 0xa438, 0xcd95,
        0xa438, 0x1000, 0xa438, 0x109e, 0xa438, 0xd701, 0xa438, 0x7b91,
        0xa438, 0xd71f, 0xa438, 0x7aae, 0xa438, 0xd701, 0xa438, 0x7ab0,
        0xa438, 0xd704, 0xa438, 0x7ef3, 0xa438, 0xd701, 0xa438, 0x5eb3,
        0xa438, 0x84b0, 0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0xa608,
        0xa438, 0xc700, 0xa438, 0x9503, 0xa438, 0xce54, 0xa438, 0x1000,
        0xa438, 0x10be, 0xa438, 0xa290, 0xa438, 0xa304, 0xa438, 0xab02,
        0xa438, 0xd700, 0xa438, 0x6050, 0xa438, 0xab04, 0xa438, 0x0c38,
        0xa438, 0x0608, 0xa438, 0xaa0b, 0xa438, 0x0c03, 0xa438, 0x1502,
        0xa438, 0x8d01, 0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0xae40,
        0xa438, 0x9503, 0xa438, 0xd702, 0xa438, 0x40a4, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0x8e20, 0xa438, 0x9503, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0x8c20, 0xa438, 0x9503, 0xa438, 0xd700,
        0xa438, 0x6078, 0xa438, 0xd700, 0xa438, 0x609a, 0xa438, 0xd109,
        0xa438, 0xd074, 0xa438, 0xf003, 0xa438, 0xd109, 0xa438, 0xd075,
        0xa438, 0xd704, 0xa438, 0x62b2, 0xa438, 0xd702, 0xa438, 0x4116,
        0xa438, 0xce54, 0xa438, 0x1000, 0xa438, 0x10be, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0x8f40, 0xa438, 0x9503, 0xa438, 0xa00a,
        0xa438, 0xd704, 0xa438, 0x4247, 0xa438, 0xd700, 0xa438, 0x3691,
        0xa438, 0x8326, 0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0xa570,
        0xa438, 0x9503, 0xa438, 0xf00a, 0xa438, 0x0c03, 0xa438, 0x1502,
        0xa438, 0xaf40, 0xa438, 0x9503, 0xa438, 0x800a, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0x8570, 0xa438, 0x9503, 0xa438, 0x1000,
        0xa438, 0x109e, 0xa438, 0xd704, 0xa438, 0x60f3, 0xa438, 0xd71f,
        0xa438, 0x618e, 0xa438, 0xd700, 0xa438, 0x5b5e, 0xa438, 0x1800,
        0xa438, 0x0deb, 0xa438, 0x800a, 0xa438, 0x0c03, 0xa438, 0x1502,
        0xa438, 0xae40, 0xa438, 0x9503, 0xa438, 0x1800, 0xa438, 0x0c47,
        0xa438, 0x1800, 0xa438, 0x0df8, 0xa438, 0x0c03, 0xa438, 0x1502,
        0xa438, 0x8608, 0xa438, 0x9503, 0xa438, 0x1800, 0xa438, 0x0e2b,
        0xa436, 0xA10E, 0xa438, 0x0d14, 0xa436, 0xA10C, 0xa438, 0x0ce8,
        0xa436, 0xA10A, 0xa438, 0x0279, 0xa436, 0xA108, 0xa438, 0x0b19,
        0xa436, 0xA106, 0xa438, 0x111f, 0xa436, 0xA104, 0xa438, 0x0a7b,
        0xa436, 0xA102, 0xa438, 0x0ba3, 0xa436, 0xA100, 0xa438, 0x0022,
        0xa436, 0xA110, 0xa438, 0x00ff, 0xa436, 0xb87c, 0xa438, 0x859b,
        0xa436, 0xb87e, 0xa438, 0xaf85, 0xa438, 0xb3af, 0xa438, 0x863b,
        0xa438, 0xaf86, 0xa438, 0x4caf, 0xa438, 0x8688, 0xa438, 0xaf86,
        0xa438, 0xceaf, 0xa438, 0x8744, 0xa438, 0xaf87, 0xa438, 0x68af,
        0xa438, 0x8781, 0xa438, 0xbf5e, 0xa438, 0x7202, 0xa438, 0x5f7e,
        0xa438, 0xac28, 0xa438, 0x68e1, 0xa438, 0x84e6, 0xa438, 0xad28,
        0xa438, 0x09bf, 0xa438, 0x5e75, 0xa438, 0x025f, 0xa438, 0x7eac,
        0xa438, 0x2d59, 0xa438, 0xe18f, 0xa438, 0xebad, 0xa438, 0x2809,
        0xa438, 0xbf5e, 0xa438, 0x7502, 0xa438, 0x5f7e, 0xa438, 0xac2e,
        0xa438, 0x50e1, 0xa438, 0x84e6, 0xa438, 0xac28, 0xa438, 0x08bf,
        0xa438, 0x873e, 0xa438, 0x025f, 0xa438, 0x3cae, 0xa438, 0x06bf,
        0xa438, 0x873e, 0xa438, 0x025f, 0xa438, 0x33bf, 0xa438, 0x8741,
        0xa438, 0x025f, 0xa438, 0x33ee, 0xa438, 0x8fea, 0xa438, 0x02e1,
        0xa438, 0x84e4, 0xa438, 0xad28, 0xa438, 0x14e1, 0xa438, 0x8fe8,
        0xa438, 0xad28, 0xa438, 0x17e1, 0xa438, 0x84e5, 0xa438, 0x11e5,
        0xa438, 0x84e5, 0xa438, 0xa10c, 0xa438, 0x04ee, 0xa438, 0x84e5,
        0xa438, 0x0002, 0xa438, 0x4977, 0xa438, 0xee84, 0xa438, 0xdc03,
        0xa438, 0xae1d, 0xa438, 0xe18f, 0xa438, 0xe811, 0xa438, 0xe58f,
        0xa438, 0xe8ae, 0xa438, 0x14bf, 0xa438, 0x873e, 0xa438, 0x025f,
        0xa438, 0x3cbf, 0xa438, 0x8741, 0xa438, 0x025f, 0xa438, 0x3cee,
        0xa438, 0x8fea, 0xa438, 0x01ee, 0xa438, 0x84e4, 0xa438, 0x00af,
        0xa438, 0x50c1, 0xa438, 0x1f00, 0xa438, 0xbf5a, 0xa438, 0x6102,
        0xa438, 0x5f5f, 0xa438, 0xbf5a, 0xa438, 0x5e02, 0xa438, 0x5f3c,
        0xa438, 0xaf45, 0xa438, 0x7be0, 0xa438, 0x8012, 0xa438, 0xad23,
        0xa438, 0x141f, 0xa438, 0x001f, 0xa438, 0x22d1, 0xa438, 0x00bf,
        0xa438, 0x3fcf, 0xa438, 0x0261, 0xa438, 0x3412, 0xa438, 0xa204,
        0xa438, 0xf6ee, 0xa438, 0x8317, 0xa438, 0x00e0, 0xa438, 0x8012,
        0xa438, 0xad24, 0xa438, 0x141f, 0xa438, 0x001f, 0xa438, 0x22d1,
        0xa438, 0x00bf, 0xa438, 0x3fd7, 0xa438, 0x0261, 0xa438, 0x3412,
        0xa438, 0xa204, 0xa438, 0xf6ee, 0xa438, 0x8317, 0xa438, 0x00ef,
        0xa438, 0x96fe, 0xa438, 0xfdfc, 0xa438, 0xaf42, 0xa438, 0x9802,
        0xa438, 0x56ec, 0xa438, 0xf70b, 0xa438, 0xac13, 0xa438, 0x0fbf,
        0xa438, 0x5e75, 0xa438, 0x025f, 0xa438, 0x7eac, 0xa438, 0x280c,
        0xa438, 0xe2ff, 0xa438, 0xcfad, 0xa438, 0x32ee, 0xa438, 0x0257,
        0xa438, 0x05af, 0xa438, 0x00a4, 0xa438, 0x0286, 0xa438, 0xaaae,
        0xa438, 0xeff8, 0xa438, 0xf9ef, 0xa438, 0x5902, 0xa438, 0x1fe1,
        0xa438, 0xbf59, 0xa438, 0x4d02, 0xa438, 0x5f3c, 0xa438, 0xac13,
        0xa438, 0x09bf, 0xa438, 0x5e75, 0xa438, 0x025f, 0xa438, 0x7ea1,
        0xa438, 0x00f4, 0xa438, 0xbf59, 0xa438, 0x4d02, 0xa438, 0x5f33,
        0xa438, 0xef95, 0xa438, 0xfdfc, 0xa438, 0x04bf, 0xa438, 0x5e72,
        0xa438, 0x025f, 0xa438, 0x7eac, 0xa438, 0x284a, 0xa438, 0xe184,
        0xa438, 0xe6ad, 0xa438, 0x2809, 0xa438, 0xbf5e, 0xa438, 0x7502,
        0xa438, 0x5f7e, 0xa438, 0xac2d, 0xa438, 0x3be1, 0xa438, 0x8feb,
        0xa438, 0xad28, 0xa438, 0x09bf, 0xa438, 0x5e75, 0xa438, 0x025f,
        0xa438, 0x7eac, 0xa438, 0x2e32, 0xa438, 0xe184, 0xa438, 0xe6ac,
        0xa438, 0x2808, 0xa438, 0xbf87, 0xa438, 0x3e02, 0xa438, 0x5f3c,
        0xa438, 0xae06, 0xa438, 0xbf87, 0xa438, 0x3e02, 0xa438, 0x5f33,
        0xa438, 0xbf87, 0xa438, 0x4102, 0xa438, 0x5f33, 0xa438, 0xee8f,
        0xa438, 0xea04, 0xa438, 0xbf5e, 0xa438, 0x4e02, 0xa438, 0x5f7e,
        0xa438, 0xad28, 0xa438, 0x1f02, 0xa438, 0x4b12, 0xa438, 0xae1a,
        0xa438, 0xbf87, 0xa438, 0x3e02, 0xa438, 0x5f3c, 0xa438, 0xbf87,
        0xa438, 0x4102, 0xa438, 0x5f3c, 0xa438, 0xee8f, 0xa438, 0xea03,
        0xa438, 0xbf5e, 0xa438, 0x2a02, 0xa438, 0x5f33, 0xa438, 0xee84,
        0xa438, 0xe701, 0xa438, 0xaf4a, 0xa438, 0x7444, 0xa438, 0xac0e,
        0xa438, 0x55ac, 0xa438, 0x0ebf, 0xa438, 0x5e75, 0xa438, 0x025f,
        0xa438, 0x7ead, 0xa438, 0x2d0b, 0xa438, 0xbf5e, 0xa438, 0x36e1,
        0xa438, 0x8fe9, 0xa438, 0x025f, 0xa438, 0x5fae, 0xa438, 0x09bf,
        0xa438, 0x5e36, 0xa438, 0xe184, 0xa438, 0xe102, 0xa438, 0x5f5f,
        0xa438, 0xee8f, 0xa438, 0xe800, 0xa438, 0xaf49, 0xa438, 0xcdbf,
        0xa438, 0x595c, 0xa438, 0x025f, 0xa438, 0x7ea1, 0xa438, 0x0203,
        0xa438, 0xaf87, 0xa438, 0x79d1, 0xa438, 0x00af, 0xa438, 0x877c,
        0xa438, 0xe181, 0xa438, 0x941f, 0xa438, 0x00af, 0xa438, 0x3ff7,
        0xa438, 0xac4e, 0xa438, 0x06ac, 0xa438, 0x4003, 0xa438, 0xaf24,
        0xa438, 0x97af, 0xa438, 0x2467, 0xa436, 0xb85e, 0xa438, 0x5082,
        0xa436, 0xb860, 0xa438, 0x4575, 0xa436, 0xb862, 0xa438, 0x425F,
        0xa436, 0xb864, 0xa438, 0x0096, 0xa436, 0xb886, 0xa438, 0x4A44,
        0xa436, 0xb888, 0xa438, 0x49c4, 0xa436, 0xb88a, 0xa438, 0x3FF2,
        0xa436, 0xb88c, 0xa438, 0x245C, 0xa436, 0xb838, 0xa438, 0x00ff,
        0xb820, 0x0010, 0xa436, 0x843d, 0xa438, 0xaf84, 0xa438, 0xa6af,
        0xa438, 0x8540, 0xa438, 0xaf85, 0xa438, 0xaeaf, 0xa438, 0x85b5,
        0xa438, 0xaf87, 0xa438, 0x7daf, 0xa438, 0x8784, 0xa438, 0xaf87,
        0xa438, 0x87af, 0xa438, 0x87e5, 0xa438, 0x0066, 0xa438, 0x0a03,
        0xa438, 0x6607, 0xa438, 0x2666, 0xa438, 0x1c00, 0xa438, 0x660d,
        0xa438, 0x0166, 0xa438, 0x1004, 0xa438, 0x6616, 0xa438, 0x0566,
        0xa438, 0x1f06, 0xa438, 0x6a5d, 0xa438, 0x2766, 0xa438, 0x1900,
        0xa438, 0x6625, 0xa438, 0x2466, 0xa438, 0x2820, 0xa438, 0x662b,
        0xa438, 0x2466, 0xa438, 0x4600, 0xa438, 0x664c, 0xa438, 0x0166,
        0xa438, 0x4902, 0xa438, 0x8861, 0xa438, 0x0388, 0xa438, 0x5e05,
        0xa438, 0x886d, 0xa438, 0x0588, 0xa438, 0x7005, 0xa438, 0x8873,
        0xa438, 0x0588, 0xa438, 0x7605, 0xa438, 0x8879, 0xa438, 0x0588,
        0xa438, 0x7c05, 0xa438, 0x887f, 0xa438, 0x0588, 0xa438, 0x8205,
        0xa438, 0x8885, 0xa438, 0x0588, 0xa438, 0x881e, 0xa438, 0x13ad,
        0xa438, 0x2841, 0xa438, 0xbf64, 0xa438, 0xf102, 0xa438, 0x6b9d,
        0xa438, 0xad28, 0xa438, 0x03af, 0xa438, 0x15fc, 0xa438, 0xbf65,
        0xa438, 0xcb02, 0xa438, 0x6b9d, 0xa438, 0x0d11, 0xa438, 0xf62f,
        0xa438, 0xef31, 0xa438, 0xd202, 0xa438, 0xbf88, 0xa438, 0x6402,
        0xa438, 0x6b52, 0xa438, 0xe082, 0xa438, 0x020d, 0xa438, 0x01f6,
        0xa438, 0x271b, 0xa438, 0x03aa, 0xa438, 0x0182, 0xa438, 0xe082,
        0xa438, 0x010d, 0xa438, 0x01f6, 0xa438, 0x271b, 0xa438, 0x03aa,
        0xa438, 0x0782, 0xa438, 0xbf88, 0xa438, 0x6402, 0xa438, 0x6b5b,
        0xa438, 0xaf15, 0xa438, 0xf9bf, 0xa438, 0x65cb, 0xa438, 0x026b,
        0xa438, 0x9d0d, 0xa438, 0x11f6, 0xa438, 0x2fef, 0xa438, 0x31e0,
        0xa438, 0x8ff7, 0xa438, 0x0d01, 0xa438, 0xf627, 0xa438, 0x1b03,
        0xa438, 0xaa20, 0xa438, 0xe18f, 0xa438, 0xf4d0, 0xa438, 0x00bf,
        0xa438, 0x6587, 0xa438, 0x026b, 0xa438, 0x7ee1, 0xa438, 0x8ff5,
        0xa438, 0xbf65, 0xa438, 0x8a02, 0xa438, 0x6b7e, 0xa438, 0xe18f,
        0xa438, 0xf6bf, 0xa438, 0x6584, 0xa438, 0x026b, 0xa438, 0x7eaf,
        0xa438, 0x15fc, 0xa438, 0xe18f, 0xa438, 0xf1d0, 0xa438, 0x00bf,
        0xa438, 0x6587, 0xa438, 0x026b, 0xa438, 0x7ee1, 0xa438, 0x8ff2,
        0xa438, 0xbf65, 0xa438, 0x8a02, 0xa438, 0x6b7e, 0xa438, 0xe18f,
        0xa438, 0xf3bf, 0xa438, 0x6584, 0xa438, 0xaf15, 0xa438, 0xfcd1,
        0xa438, 0x07bf, 0xa438, 0x65ce, 0xa438, 0x026b, 0xa438, 0x7ed1,
        0xa438, 0x0cbf, 0xa438, 0x65d1, 0xa438, 0x026b, 0xa438, 0x7ed1,
        0xa438, 0x03bf, 0xa438, 0x885e, 0xa438, 0x026b, 0xa438, 0x7ed1,
        0xa438, 0x05bf, 0xa438, 0x8867, 0xa438, 0x026b, 0xa438, 0x7ed1,
        0xa438, 0x07bf, 0xa438, 0x886a, 0xa438, 0x026b, 0xa438, 0x7ebf,
        0xa438, 0x6a6c, 0xa438, 0x026b, 0xa438, 0x5b02, 0xa438, 0x62b5,
        0xa438, 0xbf6a, 0xa438, 0x0002, 0xa438, 0x6b5b, 0xa438, 0xbf64,
        0xa438, 0x4e02, 0xa438, 0x6b9d, 0xa438, 0xac28, 0xa438, 0x0bbf,
        0xa438, 0x6412, 0xa438, 0x026b, 0xa438, 0x9da1, 0xa438, 0x0502,
        0xa438, 0xaeec, 0xa438, 0xd104, 0xa438, 0xbf65, 0xa438, 0xce02,
        0xa438, 0x6b7e, 0xa438, 0xd104, 0xa438, 0xbf65, 0xa438, 0xd102,
        0xa438, 0x6b7e, 0xa438, 0xd102, 0xa438, 0xbf88, 0xa438, 0x6702,
        0xa438, 0x6b7e, 0xa438, 0xd104, 0xa438, 0xbf88, 0xa438, 0x6a02,
        0xa438, 0x6b7e, 0xa438, 0xaf62, 0xa438, 0x72f6, 0xa438, 0x0af6,
        0xa438, 0x09af, 0xa438, 0x34e3, 0xa438, 0x0285, 0xa438, 0xbe02,
        0xa438, 0x106c, 0xa438, 0xaf10, 0xa438, 0x6bf8, 0xa438, 0xfaef,
        0xa438, 0x69e0, 0xa438, 0x804c, 0xa438, 0xac25, 0xa438, 0x17e0,
        0xa438, 0x8040, 0xa438, 0xad25, 0xa438, 0x1a02, 0xa438, 0x85ed,
        0xa438, 0xe080, 0xa438, 0x40ac, 0xa438, 0x2511, 0xa438, 0xbf87,
        0xa438, 0x6502, 0xa438, 0x6b5b, 0xa438, 0xae09, 0xa438, 0x0287,
        0xa438, 0x2402, 0xa438, 0x875a, 0xa438, 0x0287, 0xa438, 0x4fef,
        0xa438, 0x96fe, 0xa438, 0xfc04, 0xa438, 0xf8e0, 0xa438, 0x8019,
        0xa438, 0xad20, 0xa438, 0x11e0, 0xa438, 0x8fe3, 0xa438, 0xac20,
        0xa438, 0x0502, 0xa438, 0x860a, 0xa438, 0xae03, 0xa438, 0x0286,
        0xa438, 0x7802, 0xa438, 0x86c1, 0xa438, 0x0287, 0xa438, 0x4ffc,
        0xa438, 0x04f8, 0xa438, 0xf9ef, 0xa438, 0x79fb, 0xa438, 0xbf87,
        0xa438, 0x6802, 0xa438, 0x6b9d, 0xa438, 0x5c20, 0xa438, 0x000d,
        0xa438, 0x4da1, 0xa438, 0x0151, 0xa438, 0xbf87, 0xa438, 0x6802,
        0xa438, 0x6b9d, 0xa438, 0x5c07, 0xa438, 0xffe3, 0xa438, 0x8fe4,
        0xa438, 0x1b31, 0xa438, 0x9f41, 0xa438, 0x0d48, 0xa438, 0xe38f,
        0xa438, 0xe51b, 0xa438, 0x319f, 0xa438, 0x38bf, 0xa438, 0x876b,
        0xa438, 0x026b, 0xa438, 0x9d5c, 0xa438, 0x07ff, 0xa438, 0xe38f,
        0xa438, 0xe61b, 0xa438, 0x319f, 0xa438, 0x280d, 0xa438, 0x48e3,
        0xa438, 0x8fe7, 0xa438, 0x1b31, 0xa438, 0x9f1f, 0xa438, 0xbf87,
        0xa438, 0x6e02, 0xa438, 0x6b9d, 0xa438, 0x5c07, 0xa438, 0xffe3,
        0xa438, 0x8fe8, 0xa438, 0x1b31, 0xa438, 0x9f0f, 0xa438, 0x0d48,
        0xa438, 0xe38f, 0xa438, 0xe91b, 0xa438, 0x319f, 0xa438, 0x06ee,
        0xa438, 0x8fe3, 0xa438, 0x01ae, 0xa438, 0x04ee, 0xa438, 0x8fe3,
        0xa438, 0x00ff, 0xa438, 0xef97, 0xa438, 0xfdfc, 0xa438, 0x04f8,
        0xa438, 0xf9ef, 0xa438, 0x79fb, 0xa438, 0xbf87, 0xa438, 0x6802,
        0xa438, 0x6b9d, 0xa438, 0x5c20, 0xa438, 0x000d, 0xa438, 0x4da1,
        0xa438, 0x0020, 0xa438, 0xbf87, 0xa438, 0x6802, 0xa438, 0x6b9d,
        0xa438, 0x5c06, 0xa438, 0x000d, 0xa438, 0x49e3, 0xa438, 0x8fea,
        0xa438, 0x1b31, 0xa438, 0x9f0e, 0xa438, 0xbf87, 0xa438, 0x7102,
        0xa438, 0x6b5b, 0xa438, 0xbf87, 0xa438, 0x7702, 0xa438, 0x6b5b,
        0xa438, 0xae0c, 0xa438, 0xbf87, 0xa438, 0x7102, 0xa438, 0x6b52,
        0xa438, 0xbf87, 0xa438, 0x7702, 0xa438, 0x6b52, 0xa438, 0xee8f,
        0xa438, 0xe300, 0xa438, 0xffef, 0xa438, 0x97fd, 0xa438, 0xfc04,
        0xa438, 0xf8f9, 0xa438, 0xef79, 0xa438, 0xfbbf, 0xa438, 0x8768,
        0xa438, 0x026b, 0xa438, 0x9d5c, 0xa438, 0x2000, 0xa438, 0x0d4d,
        0xa438, 0xa101, 0xa438, 0x4abf, 0xa438, 0x8768, 0xa438, 0x026b,
        0xa438, 0x9d5c, 0xa438, 0x07ff, 0xa438, 0xe38f, 0xa438, 0xeb1b,
        0xa438, 0x319f, 0xa438, 0x3a0d, 0xa438, 0x48e3, 0xa438, 0x8fec,
        0xa438, 0x1b31, 0xa438, 0x9f31, 0xa438, 0xbf87, 0xa438, 0x6b02,
        0xa438, 0x6b9d, 0xa438, 0xe38f, 0xa438, 0xed1b, 0xa438, 0x319f,
        0xa438, 0x240d, 0xa438, 0x48e3, 0xa438, 0x8fee, 0xa438, 0x1b31,
        0xa438, 0x9f1b, 0xa438, 0xbf87, 0xa438, 0x6e02, 0xa438, 0x6b9d,
        0xa438, 0xe38f, 0xa438, 0xef1b, 0xa438, 0x319f, 0xa438, 0x0ebf,
        0xa438, 0x8774, 0xa438, 0x026b, 0xa438, 0x5bbf, 0xa438, 0x877a,
        0xa438, 0x026b, 0xa438, 0x5bae, 0xa438, 0x00ff, 0xa438, 0xef97,
        0xa438, 0xfdfc, 0xa438, 0x04f8, 0xa438, 0xef79, 0xa438, 0xfbe0,
        0xa438, 0x8019, 0xa438, 0xad20, 0xa438, 0x1cee, 0xa438, 0x8fe3,
        0xa438, 0x00bf, 0xa438, 0x8771, 0xa438, 0x026b, 0xa438, 0x52bf,
        0xa438, 0x8777, 0xa438, 0x026b, 0xa438, 0x52bf, 0xa438, 0x8774,
        0xa438, 0x026b, 0xa438, 0x52bf, 0xa438, 0x877a, 0xa438, 0x026b,
        0xa438, 0x52ff, 0xa438, 0xef97, 0xa438, 0xfc04, 0xa438, 0xf8e0,
        0xa438, 0x8040, 0xa438, 0xf625, 0xa438, 0xe480, 0xa438, 0x40fc,
        0xa438, 0x04f8, 0xa438, 0xe080, 0xa438, 0x4cf6, 0xa438, 0x25e4,
        0xa438, 0x804c, 0xa438, 0xfc04, 0xa438, 0x55a4, 0xa438, 0xbaf0,
        0xa438, 0xa64a, 0xa438, 0xf0a6, 0xa438, 0x4cf0, 0xa438, 0xa64e,
        0xa438, 0x66a4, 0xa438, 0xb655, 0xa438, 0xa4b6, 0xa438, 0x00ac,
        0xa438, 0x0e66, 0xa438, 0xac0e, 0xa438, 0xee80, 0xa438, 0x4c3a,
        0xa438, 0xaf07, 0xa438, 0xd0af, 0xa438, 0x26d0, 0xa438, 0xa201,
        0xa438, 0x0ebf, 0xa438, 0x663d, 0xa438, 0x026b, 0xa438, 0x52bf,
        0xa438, 0x6643, 0xa438, 0x026b, 0xa438, 0x52ae, 0xa438, 0x11bf,
        0xa438, 0x6643, 0xa438, 0x026b, 0xa438, 0x5bd4, 0xa438, 0x0054,
        0xa438, 0xb4fe, 0xa438, 0xbf66, 0xa438, 0x3d02, 0xa438, 0x6b5b,
        0xa438, 0xd300, 0xa438, 0x020d, 0xa438, 0xf6a2, 0xa438, 0x0405,
        0xa438, 0xe081, 0xa438, 0x47ae, 0xa438, 0x03e0, 0xa438, 0x8148,
        0xa438, 0xac23, 0xa438, 0x02ae, 0xa438, 0x0268, 0xa438, 0xf01a,
        0xa438, 0x10ad, 0xa438, 0x2f04, 0xa438, 0xd100, 0xa438, 0xae05,
        0xa438, 0xad2c, 0xa438, 0x02d1, 0xa438, 0x0f1f, 0xa438, 0x00a2,
        0xa438, 0x0407, 0xa438, 0x3908, 0xa438, 0xad2f, 0xa438, 0x02d1,
        0xa438, 0x0002, 0xa438, 0x0e1c, 0xa438, 0x2b01, 0xa438, 0xad3a,
        0xa438, 0xc9af, 0xa438, 0x0dee, 0xa438, 0xa000, 0xa438, 0x2702,
        0xa438, 0x1beb, 0xa438, 0xe18f, 0xa438, 0xe1ac, 0xa438, 0x2819,
        0xa438, 0xee8f, 0xa438, 0xe101, 0xa438, 0x1f44, 0xa438, 0xbf65,
        0xa438, 0x9302, 0xa438, 0x6b9d, 0xa438, 0xe58f, 0xa438, 0xe21f,
        0xa438, 0x44d1, 0xa438, 0x02bf, 0xa438, 0x6593, 0xa438, 0x026b,
        0xa438, 0x7ee0, 0xa438, 0x82b1, 0xa438, 0xae49, 0xa438, 0xa001,
        0xa438, 0x0502, 0xa438, 0x1c4d, 0xa438, 0xae41, 0xa438, 0xa002,
        0xa438, 0x0502, 0xa438, 0x1c90, 0xa438, 0xae39, 0xa438, 0xa003,
        0xa438, 0x0502, 0xa438, 0x1c9d, 0xa438, 0xae31, 0xa438, 0xa004,
        0xa438, 0x0502, 0xa438, 0x1cbc, 0xa438, 0xae29, 0xa438, 0xa005,
        0xa438, 0x1e02, 0xa438, 0x1cc9, 0xa438, 0xe080, 0xa438, 0xdfac,
        0xa438, 0x2013, 0xa438, 0xac21, 0xa438, 0x10ac, 0xa438, 0x220d,
        0xa438, 0xe18f, 0xa438, 0xe2bf, 0xa438, 0x6593, 0xa438, 0x026b,
        0xa438, 0x7eee, 0xa438, 0x8fe1, 0xa438, 0x00ae, 0xa438, 0x08a0,
        0xa438, 0x0605, 0xa438, 0x021d, 0xa438, 0x07ae, 0xa438, 0x00e0,
        0xa438, 0x82b1, 0xa438, 0xaf1b, 0xa438, 0xe910, 0xa438, 0xbf4a,
        0xa438, 0x99bf, 0xa438, 0x4a00, 0xa438, 0xa86a, 0xa438, 0xfdad,
        0xa438, 0x5eca, 0xa438, 0xad5e, 0xa438, 0x88bd, 0xa438, 0x2c99,
        0xa438, 0xbd2c, 0xa438, 0x33bd, 0xa438, 0x3222, 0xa438, 0xbd32,
        0xa438, 0x11bd, 0xa438, 0x3200, 0xa438, 0xbd32, 0xa438, 0x77bd,
        0xa438, 0x3266, 0xa438, 0xbd32, 0xa438, 0x55bd, 0xa438, 0x3244,
        0xa438, 0xbd32, 0xa436, 0xb818, 0xa438, 0x15c5, 0xa436, 0xb81a,
        0xa438, 0x6255, 0xa436, 0xb81c, 0xa438, 0x34e1, 0xa436, 0xb81e,
        0xa438, 0x1068, 0xa436, 0xb850, 0xa438, 0x07cc, 0xa436, 0xb852,
        0xa438, 0x26ca, 0xa436, 0xb878, 0xa438, 0x0dbf, 0xa436, 0xb884,
        0xa438, 0x1BB1, 0xa436, 0xb832, 0xa438, 0x00ff, 0xa436, 0x0000,
        0xa438, 0x0000, 0xB82E, 0x0000, 0xa436, 0x8023, 0xa438, 0x0000,
        0xa436, 0x801E, 0xa438, 0x0031, 0xB820, 0x0000, 0xFFFF, 0xFFFF
};

static const u16  phy_mcu_ram_code_8125d_1_2[] = {
        0xb892, 0x0000, 0xB88E, 0xC28F, 0xB890, 0x252D, 0xB88E, 0xC290,
        0xB890, 0xC924, 0xB88E, 0xC291, 0xB890, 0xC92E, 0xB88E, 0xC292,
        0xB890, 0xF626, 0xB88E, 0xC293, 0xB890, 0xF630, 0xB88E, 0xC294,
        0xB890, 0xA328, 0xB88E, 0xC295, 0xB890, 0xA332, 0xB88E, 0xC296,
        0xB890, 0xD72B, 0xB88E, 0xC297, 0xB890, 0xD735, 0xB88E, 0xC298,
        0xB890, 0x8A2E, 0xB88E, 0xC299, 0xB890, 0x8A38, 0xB88E, 0xC29A,
        0xB890, 0xBE32, 0xB88E, 0xC29B, 0xB890, 0xBE3C, 0xB88E, 0xC29C,
        0xB890, 0x7436, 0xB88E, 0xC29D, 0xB890, 0x7440, 0xB88E, 0xC29E,
        0xB890, 0xAD3B, 0xB88E, 0xC29F, 0xB890, 0xAD45, 0xB88E, 0xC2A0,
        0xB890, 0x6640, 0xB88E, 0xC2A1, 0xB890, 0x664A, 0xB88E, 0xC2A2,
        0xB890, 0xA646, 0xB88E, 0xC2A3, 0xB890, 0xA650, 0xB88E, 0xC2A4,
        0xB890, 0x624C, 0xB88E, 0xC2A5, 0xB890, 0x6256, 0xB88E, 0xC2A6,
        0xB890, 0xA453, 0xB88E, 0xC2A7, 0xB890, 0xA45D, 0xB88E, 0xC2A8,
        0xB890, 0x665A, 0xB88E, 0xC2A9, 0xB890, 0x6664, 0xB88E, 0xC2AA,
        0xB890, 0xAC62, 0xB88E, 0xC2AB, 0xB890, 0xAC6C, 0xB88E, 0xC2AC,
        0xB890, 0x746A, 0xB88E, 0xC2AD, 0xB890, 0x7474, 0xB88E, 0xC2AE,
        0xB890, 0xBCFA, 0xB88E, 0xC2AF, 0xB890, 0xBCFD, 0xB88E, 0xC2B0,
        0xB890, 0x79FF, 0xB88E, 0xC2B1, 0xB890, 0x7901, 0xB88E, 0xC2B2,
        0xB890, 0xF703, 0xB88E, 0xC2B3, 0xB890, 0xF706, 0xB88E, 0xC2B4,
        0xB890, 0x7408, 0xB88E, 0xC2B5, 0xB890, 0x740A, 0xB88E, 0xC2B6,
        0xB890, 0xF10C, 0xB88E, 0xC2B7, 0xB890, 0xF10F, 0xB88E, 0xC2B8,
        0xB890, 0x6F10, 0xB88E, 0xC2B9, 0xB890, 0x6F13, 0xB88E, 0xC2BA,
        0xB890, 0xEC15, 0xB88E, 0xC2BB, 0xB890, 0xEC18, 0xB88E, 0xC2BC,
        0xB890, 0x6A1A, 0xB88E, 0xC2BD, 0xB890, 0x6A1C, 0xB88E, 0xC2BE,
        0xB890, 0xE71E, 0xB88E, 0xC2BF, 0xB890, 0xE721, 0xB88E, 0xC2C0,
        0xB890, 0x6424, 0xB88E, 0xC2C1, 0xB890, 0x6425, 0xB88E, 0xC2C2,
        0xB890, 0xE228, 0xB88E, 0xC2C3, 0xB890, 0xE22A, 0xB88E, 0xC2C4,
        0xB890, 0x5F2B, 0xB88E, 0xC2C5, 0xB890, 0x5F2E, 0xB88E, 0xC2C6,
        0xB890, 0xDC31, 0xB88E, 0xC2C7, 0xB890, 0xDC33, 0xB88E, 0xC2C8,
        0xB890, 0x2035, 0xB88E, 0xC2C9, 0xB890, 0x2036, 0xB88E, 0xC2CA,
        0xB890, 0x9F3A, 0xB88E, 0xC2CB, 0xB890, 0x9F3A, 0xB88E, 0xC2CC,
        0xB890, 0x4430, 0xFFFF, 0xFFFF
};

static const u16  phy_mcu_ram_code_8125d_1_3[] = {
        0xa436, 0xacca, 0xa438, 0x0104, 0xa436, 0xaccc, 0xa438, 0x8000,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x0fff,
        0xa436, 0xacce, 0xa438, 0xfd47, 0xa436, 0xacd0, 0xa438, 0x0fff,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xe56f, 0xa436, 0xacd0, 0xa438, 0x01c0,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xed97, 0xa436, 0xacd0, 0xa438, 0x01c8,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xf5bf, 0xa436, 0xacd0, 0xa438, 0x01d0,
        0xa436, 0xacce, 0xa438, 0xfb07, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb0f, 0xa436, 0xacd0, 0xa438, 0x01d8,
        0xa436, 0xacce, 0xa438, 0xa087, 0xa436, 0xacd0, 0xa438, 0x0180,
        0xa436, 0xacce, 0xa438, 0xa00f, 0xa436, 0xacd0, 0xa438, 0x0108,
        0xa436, 0xacce, 0xa438, 0xa807, 0xa436, 0xacd0, 0xa438, 0x0100,
        0xa436, 0xacce, 0xa438, 0xa88f, 0xa436, 0xacd0, 0xa438, 0x0188,
        0xa436, 0xacce, 0xa438, 0xb027, 0xa436, 0xacd0, 0xa438, 0x0120,
        0xa436, 0xacce, 0xa438, 0xb02f, 0xa436, 0xacd0, 0xa438, 0x0128,
        0xa436, 0xacce, 0xa438, 0xb847, 0xa436, 0xacd0, 0xa438, 0x0140,
        0xa436, 0xacce, 0xa438, 0xb84f, 0xa436, 0xacd0, 0xa438, 0x0148,
        0xa436, 0xacce, 0xa438, 0xfb17, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb1f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xa017, 0xa436, 0xacd0, 0xa438, 0x0110,
        0xa436, 0xacce, 0xa438, 0xa01f, 0xa436, 0xacd0, 0xa438, 0x0118,
        0xa436, 0xacce, 0xa438, 0xa837, 0xa436, 0xacd0, 0xa438, 0x0130,
        0xa436, 0xacce, 0xa438, 0xa83f, 0xa436, 0xacd0, 0xa438, 0x0138,
        0xa436, 0xacce, 0xa438, 0xb097, 0xa436, 0xacd0, 0xa438, 0x0190,
        0xa436, 0xacce, 0xa438, 0xb05f, 0xa436, 0xacd0, 0xa438, 0x0158,
        0xa436, 0xacce, 0xa438, 0xb857, 0xa436, 0xacd0, 0xa438, 0x0150,
        0xa436, 0xacce, 0xa438, 0xb89f, 0xa436, 0xacd0, 0xa438, 0x0198,
        0xa436, 0xacce, 0xa438, 0xfb27, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb2f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x8087, 0xa436, 0xacd0, 0xa438, 0x0180,
        0xa436, 0xacce, 0xa438, 0x800f, 0xa436, 0xacd0, 0xa438, 0x0108,
        0xa436, 0xacce, 0xa438, 0x8807, 0xa436, 0xacd0, 0xa438, 0x0100,
        0xa436, 0xacce, 0xa438, 0x888f, 0xa436, 0xacd0, 0xa438, 0x0188,
        0xa436, 0xacce, 0xa438, 0x9027, 0xa436, 0xacd0, 0xa438, 0x0120,
        0xa436, 0xacce, 0xa438, 0x902f, 0xa436, 0xacd0, 0xa438, 0x0128,
        0xa436, 0xacce, 0xa438, 0x9847, 0xa436, 0xacd0, 0xa438, 0x0140,
        0xa436, 0xacce, 0xa438, 0x984f, 0xa436, 0xacd0, 0xa438, 0x0148,
        0xa436, 0xacce, 0xa438, 0xa0a7, 0xa436, 0xacd0, 0xa438, 0x01a0,
        0xa436, 0xacce, 0xa438, 0xa8af, 0xa436, 0xacd0, 0xa438, 0x01a8,
        0xa436, 0xacce, 0xa438, 0xa067, 0xa436, 0xacd0, 0xa438, 0x0161,
        0xa436, 0xacce, 0xa438, 0xa86f, 0xa436, 0xacd0, 0xa438, 0x0169,
        0xa436, 0xacce, 0xa438, 0xfb37, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb3f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x8017, 0xa436, 0xacd0, 0xa438, 0x0110,
        0xa436, 0xacce, 0xa438, 0x801f, 0xa436, 0xacd0, 0xa438, 0x0118,
        0xa436, 0xacce, 0xa438, 0x8837, 0xa436, 0xacd0, 0xa438, 0x0130,
        0xa436, 0xacce, 0xa438, 0x883f, 0xa436, 0xacd0, 0xa438, 0x0138,
        0xa436, 0xacce, 0xa438, 0x9097, 0xa436, 0xacd0, 0xa438, 0x0190,
        0xa436, 0xacce, 0xa438, 0x905f, 0xa436, 0xacd0, 0xa438, 0x0158,
        0xa436, 0xacce, 0xa438, 0x9857, 0xa436, 0xacd0, 0xa438, 0x0150,
        0xa436, 0xacce, 0xa438, 0x989f, 0xa436, 0xacd0, 0xa438, 0x0198,
        0xa436, 0xacce, 0xa438, 0xb0b7, 0xa436, 0xacd0, 0xa438, 0x01b0,
        0xa436, 0xacce, 0xa438, 0xb8bf, 0xa436, 0xacd0, 0xa438, 0x01b8,
        0xa436, 0xacce, 0xa438, 0xb077, 0xa436, 0xacd0, 0xa438, 0x0171,
        0xa436, 0xacce, 0xa438, 0xb87f, 0xa436, 0xacd0, 0xa438, 0x0179,
        0xa436, 0xacce, 0xa438, 0xfb47, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb4f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x6087, 0xa436, 0xacd0, 0xa438, 0x0180,
        0xa436, 0xacce, 0xa438, 0x600f, 0xa436, 0xacd0, 0xa438, 0x0108,
        0xa436, 0xacce, 0xa438, 0x6807, 0xa436, 0xacd0, 0xa438, 0x0100,
        0xa436, 0xacce, 0xa438, 0x688f, 0xa436, 0xacd0, 0xa438, 0x0188,
        0xa436, 0xacce, 0xa438, 0x7027, 0xa436, 0xacd0, 0xa438, 0x0120,
        0xa436, 0xacce, 0xa438, 0x702f, 0xa436, 0xacd0, 0xa438, 0x0128,
        0xa436, 0xacce, 0xa438, 0x7847, 0xa436, 0xacd0, 0xa438, 0x0140,
        0xa436, 0xacce, 0xa438, 0x784f, 0xa436, 0xacd0, 0xa438, 0x0148,
        0xa436, 0xacce, 0xa438, 0x80a7, 0xa436, 0xacd0, 0xa438, 0x01a0,
        0xa436, 0xacce, 0xa438, 0x88af, 0xa436, 0xacd0, 0xa438, 0x01a8,
        0xa436, 0xacce, 0xa438, 0x8067, 0xa436, 0xacd0, 0xa438, 0x0161,
        0xa436, 0xacce, 0xa438, 0x886f, 0xa436, 0xacd0, 0xa438, 0x0169,
        0xa436, 0xacce, 0xa438, 0xfb57, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb5f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x6017, 0xa436, 0xacd0, 0xa438, 0x0110,
        0xa436, 0xacce, 0xa438, 0x601f, 0xa436, 0xacd0, 0xa438, 0x0118,
        0xa436, 0xacce, 0xa438, 0x6837, 0xa436, 0xacd0, 0xa438, 0x0130,
        0xa436, 0xacce, 0xa438, 0x683f, 0xa436, 0xacd0, 0xa438, 0x0138,
        0xa436, 0xacce, 0xa438, 0x7097, 0xa436, 0xacd0, 0xa438, 0x0190,
        0xa436, 0xacce, 0xa438, 0x705f, 0xa436, 0xacd0, 0xa438, 0x0158,
        0xa436, 0xacce, 0xa438, 0x7857, 0xa436, 0xacd0, 0xa438, 0x0150,
        0xa436, 0xacce, 0xa438, 0x789f, 0xa436, 0xacd0, 0xa438, 0x0198,
        0xa436, 0xacce, 0xa438, 0x90b7, 0xa436, 0xacd0, 0xa438, 0x01b0,
        0xa436, 0xacce, 0xa438, 0x98bf, 0xa436, 0xacd0, 0xa438, 0x01b8,
        0xa436, 0xacce, 0xa438, 0x9077, 0xa436, 0xacd0, 0xa438, 0x0171,
        0xa436, 0xacce, 0xa438, 0x987f, 0xa436, 0xacd0, 0xa438, 0x0179,
        0xa436, 0xacce, 0xa438, 0xfb67, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb6f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x4087, 0xa436, 0xacd0, 0xa438, 0x0180,
        0xa436, 0xacce, 0xa438, 0x400f, 0xa436, 0xacd0, 0xa438, 0x0108,
        0xa436, 0xacce, 0xa438, 0x4807, 0xa436, 0xacd0, 0xa438, 0x0100,
        0xa436, 0xacce, 0xa438, 0x488f, 0xa436, 0xacd0, 0xa438, 0x0188,
        0xa436, 0xacce, 0xa438, 0x5027, 0xa436, 0xacd0, 0xa438, 0x0120,
        0xa436, 0xacce, 0xa438, 0x502f, 0xa436, 0xacd0, 0xa438, 0x0128,
        0xa436, 0xacce, 0xa438, 0x5847, 0xa436, 0xacd0, 0xa438, 0x0140,
        0xa436, 0xacce, 0xa438, 0x584f, 0xa436, 0xacd0, 0xa438, 0x0148,
        0xa436, 0xacce, 0xa438, 0x60a7, 0xa436, 0xacd0, 0xa438, 0x01a0,
        0xa436, 0xacce, 0xa438, 0x68af, 0xa436, 0xacd0, 0xa438, 0x01a8,
        0xa436, 0xacce, 0xa438, 0x6067, 0xa436, 0xacd0, 0xa438, 0x0161,
        0xa436, 0xacce, 0xa438, 0x686f, 0xa436, 0xacd0, 0xa438, 0x0169,
        0xa436, 0xacce, 0xa438, 0xfb77, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb7f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x4017, 0xa436, 0xacd0, 0xa438, 0x0110,
        0xa436, 0xacce, 0xa438, 0x401f, 0xa436, 0xacd0, 0xa438, 0x0118,
        0xa436, 0xacce, 0xa438, 0x4837, 0xa436, 0xacd0, 0xa438, 0x0130,
        0xa436, 0xacce, 0xa438, 0x483f, 0xa436, 0xacd0, 0xa438, 0x0138,
        0xa436, 0xacce, 0xa438, 0x5097, 0xa436, 0xacd0, 0xa438, 0x0190,
        0xa436, 0xacce, 0xa438, 0x505f, 0xa436, 0xacd0, 0xa438, 0x0158,
        0xa436, 0xacce, 0xa438, 0x5857, 0xa436, 0xacd0, 0xa438, 0x0150,
        0xa436, 0xacce, 0xa438, 0x589f, 0xa436, 0xacd0, 0xa438, 0x0198,
        0xa436, 0xacce, 0xa438, 0x70b7, 0xa436, 0xacd0, 0xa438, 0x01b0,
        0xa436, 0xacce, 0xa438, 0x78bf, 0xa436, 0xacd0, 0xa438, 0x01b8,
        0xa436, 0xacce, 0xa438, 0x7077, 0xa436, 0xacd0, 0xa438, 0x0171,
        0xa436, 0xacce, 0xa438, 0x787f, 0xa436, 0xacd0, 0xa438, 0x0179,
        0xa436, 0xacce, 0xa438, 0xfb87, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb8f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x40a7, 0xa436, 0xacd0, 0xa438, 0x01a0,
        0xa436, 0xacce, 0xa438, 0x48af, 0xa436, 0xacd0, 0xa438, 0x01a8,
        0xa436, 0xacce, 0xa438, 0x4067, 0xa436, 0xacd0, 0xa438, 0x0161,
        0xa436, 0xacce, 0xa438, 0x486f, 0xa436, 0xacd0, 0xa438, 0x0169,
        0xa436, 0xacce, 0xa438, 0xfb97, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb9f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x50b7, 0xa436, 0xacd0, 0xa438, 0x01b0,
        0xa436, 0xacce, 0xa438, 0x58bf, 0xa436, 0xacd0, 0xa438, 0x01b8,
        0xa436, 0xacce, 0xa438, 0x5077, 0xa436, 0xacd0, 0xa438, 0x0171,
        0xa436, 0xacce, 0xa438, 0x587f, 0xa436, 0xacd0, 0xa438, 0x0179,
        0xa436, 0xacce, 0xa438, 0xfba7, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfbaf, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x2067, 0xa436, 0xacd0, 0xa438, 0x0161,
        0xa436, 0xacce, 0xa438, 0x286f, 0xa436, 0xacd0, 0xa438, 0x0169,
        0xa436, 0xacce, 0xa438, 0xfbb7, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfbbf, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x3077, 0xa436, 0xacd0, 0xa438, 0x0171,
        0xa436, 0xacce, 0xa438, 0x387f, 0xa436, 0xacd0, 0xa438, 0x0179,
        0xa436, 0xacce, 0xa438, 0xfff9, 0xa436, 0xacd0, 0xa438, 0x17ff,
        0xa436, 0xacce, 0xa438, 0xfff9, 0xa436, 0xacd0, 0xa438, 0x17ff,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x0fff,
        0xa436, 0xacce, 0xa438, 0xfff8, 0xa436, 0xacd0, 0xa438, 0x0fff,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb47, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb4f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x6087, 0xa436, 0xacd0, 0xa438, 0x0180,
        0xa436, 0xacce, 0xa438, 0x600f, 0xa436, 0xacd0, 0xa438, 0x0108,
        0xa436, 0xacce, 0xa438, 0x6807, 0xa436, 0xacd0, 0xa438, 0x0100,
        0xa436, 0xacce, 0xa438, 0x688f, 0xa436, 0xacd0, 0xa438, 0x0188,
        0xa436, 0xacce, 0xa438, 0x7027, 0xa436, 0xacd0, 0xa438, 0x0120,
        0xa436, 0xacce, 0xa438, 0x702f, 0xa436, 0xacd0, 0xa438, 0x0128,
        0xa436, 0xacce, 0xa438, 0x7847, 0xa436, 0xacd0, 0xa438, 0x0140,
        0xa436, 0xacce, 0xa438, 0x784f, 0xa436, 0xacd0, 0xa438, 0x0148,
        0xa436, 0xacce, 0xa438, 0x80a7, 0xa436, 0xacd0, 0xa438, 0x01a0,
        0xa436, 0xacce, 0xa438, 0x88af, 0xa436, 0xacd0, 0xa438, 0x01a8,
        0xa436, 0xacce, 0xa438, 0x8067, 0xa436, 0xacd0, 0xa438, 0x0161,
        0xa436, 0xacce, 0xa438, 0x886f, 0xa436, 0xacd0, 0xa438, 0x0169,
        0xa436, 0xacce, 0xa438, 0xfb57, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb5f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x6017, 0xa436, 0xacd0, 0xa438, 0x0110,
        0xa436, 0xacce, 0xa438, 0x601f, 0xa436, 0xacd0, 0xa438, 0x0118,
        0xa436, 0xacce, 0xa438, 0x6837, 0xa436, 0xacd0, 0xa438, 0x0130,
        0xa436, 0xacce, 0xa438, 0x683f, 0xa436, 0xacd0, 0xa438, 0x0138,
        0xa436, 0xacce, 0xa438, 0x7097, 0xa436, 0xacd0, 0xa438, 0x0190,
        0xa436, 0xacce, 0xa438, 0x705f, 0xa436, 0xacd0, 0xa438, 0x0158,
        0xa436, 0xacce, 0xa438, 0x7857, 0xa436, 0xacd0, 0xa438, 0x0150,
        0xa436, 0xacce, 0xa438, 0x789f, 0xa436, 0xacd0, 0xa438, 0x0198,
        0xa436, 0xacce, 0xa438, 0x90b7, 0xa436, 0xacd0, 0xa438, 0x01b0,
        0xa436, 0xacce, 0xa438, 0x98bf, 0xa436, 0xacd0, 0xa438, 0x01b8,
        0xa436, 0xacce, 0xa438, 0x9077, 0xa436, 0xacd0, 0xa438, 0x1171,
        0xa436, 0xacce, 0xa438, 0x987f, 0xa436, 0xacd0, 0xa438, 0x1179,
        0xa436, 0xacca, 0xa438, 0x0004, 0xa436, 0xacc6, 0xa438, 0x0008,
        0xa436, 0xacc8, 0xa438, 0xc000, 0xa436, 0xacc6, 0xa438, 0x0015,
        0xa436, 0xacc8, 0xa438, 0xc043, 0xa436, 0xacc8, 0xa438, 0x0000,
        0xB820, 0x0000, 0xFFFF, 0xFFFF
};

static const u16  phy_mcu_ram_code_8125d_2_1[] = {
        0xa436, 0x8023, 0xa438, 0x3801, 0xa436, 0xB82E, 0xa438, 0x0001,
        0xb820, 0x0090, 0xa436, 0xA016, 0xa438, 0x0000, 0xa436, 0xA012,
        0xa438, 0x0000, 0xa436, 0xA014, 0xa438, 0x1800, 0xa438, 0x8010,
        0xa438, 0x1800, 0xa438, 0x807e, 0xa438, 0x1800, 0xa438, 0x80be,
        0xa438, 0x1800, 0xa438, 0x81c8, 0xa438, 0x1800, 0xa438, 0x81c8,
        0xa438, 0x1800, 0xa438, 0x81c8, 0xa438, 0x1800, 0xa438, 0x81c8,
        0xa438, 0x1800, 0xa438, 0x81c8, 0xa438, 0xd500, 0xa438, 0xc48d,
        0xa438, 0xd504, 0xa438, 0x8d03, 0xa438, 0xd701, 0xa438, 0x4045,
        0xa438, 0xad02, 0xa438, 0xd504, 0xa438, 0xd706, 0xa438, 0x2529,
        0xa438, 0x8021, 0xa438, 0xd718, 0xa438, 0x607b, 0xa438, 0x40da,
        0xa438, 0xf01b, 0xa438, 0x461a, 0xa438, 0xf045, 0xa438, 0xd718,
        0xa438, 0x62fb, 0xa438, 0xbb01, 0xa438, 0xd75e, 0xa438, 0x6271,
        0xa438, 0x0cf0, 0xa438, 0x0c10, 0xa438, 0xd501, 0xa438, 0xce01,
        0xa438, 0xd70c, 0xa438, 0x6187, 0xa438, 0x0cf0, 0xa438, 0x0470,
        0xa438, 0x0cf0, 0xa438, 0x0430, 0xa438, 0x0cf0, 0xa438, 0x0410,
        0xa438, 0xce00, 0xa438, 0xd505, 0xa438, 0x0c0f, 0xa438, 0x0808,
        0xa438, 0xf002, 0xa438, 0xa4f0, 0xa438, 0xf042, 0xa438, 0xbb02,
        0xa438, 0xd75e, 0xa438, 0x6271, 0xa438, 0x0cf0, 0xa438, 0x0c20,
        0xa438, 0xd501, 0xa438, 0xce01, 0xa438, 0xd70c, 0xa438, 0x6187,
        0xa438, 0x0cf0, 0xa438, 0x0470, 0xa438, 0x0cf0, 0xa438, 0x0430,
        0xa438, 0x0cf0, 0xa438, 0x0420, 0xa438, 0xce00, 0xa438, 0xd505,
        0xa438, 0x0c0f, 0xa438, 0x0804, 0xa438, 0xf002, 0xa438, 0xa4f0,
        0xa438, 0xf02c, 0xa438, 0xbb04, 0xa438, 0xd75e, 0xa438, 0x6271,
        0xa438, 0x0cf0, 0xa438, 0x0c40, 0xa438, 0xd501, 0xa438, 0xce01,
        0xa438, 0xd70c, 0xa438, 0x6187, 0xa438, 0x0cf0, 0xa438, 0x0470,
        0xa438, 0x0cf0, 0xa438, 0x0450, 0xa438, 0x0cf0, 0xa438, 0x0440,
        0xa438, 0xce00, 0xa438, 0xd505, 0xa438, 0x0c0f, 0xa438, 0x0802,
        0xa438, 0xf002, 0xa438, 0xa4f0, 0xa438, 0xf016, 0xa438, 0xbb08,
        0xa438, 0xd75e, 0xa438, 0x6271, 0xa438, 0x0cf0, 0xa438, 0x0c80,
        0xa438, 0xd501, 0xa438, 0xce01, 0xa438, 0xd70c, 0xa438, 0x6187,
        0xa438, 0x0cf0, 0xa438, 0x04b0, 0xa438, 0x0cf0, 0xa438, 0x0490,
        0xa438, 0x0cf0, 0xa438, 0x0480, 0xa438, 0xce00, 0xa438, 0xd505,
        0xa438, 0x0c0f, 0xa438, 0x0801, 0xa438, 0xf002, 0xa438, 0xa4f0,
        0xa438, 0xce00, 0xa438, 0xd500, 0xa438, 0x1800, 0xa438, 0x165a,
        0xa438, 0xd75e, 0xa438, 0x67b1, 0xa438, 0xd504, 0xa438, 0xd71e,
        0xa438, 0x65bb, 0xa438, 0x63da, 0xa438, 0x61f9, 0xa438, 0x0cf0,
        0xa438, 0x0c10, 0xa438, 0xd505, 0xa438, 0x0c0f, 0xa438, 0x0808,
        0xa438, 0xd501, 0xa438, 0xce01, 0xa438, 0xd70c, 0xa438, 0x6087,
        0xa438, 0x0cf0, 0xa438, 0x0410, 0xa438, 0xf02c, 0xa438, 0xa4f0,
        0xa438, 0xf02a, 0xa438, 0x0cf0, 0xa438, 0x0c20, 0xa438, 0xd505,
        0xa438, 0x0c0f, 0xa438, 0x0804, 0xa438, 0xd501, 0xa438, 0xce01,
        0xa438, 0xd70c, 0xa438, 0x6087, 0xa438, 0x0cf0, 0xa438, 0x0420,
        0xa438, 0xf01e, 0xa438, 0xa4f0, 0xa438, 0xf01c, 0xa438, 0x0cf0,
        0xa438, 0x0c40, 0xa438, 0xd505, 0xa438, 0x0c0f, 0xa438, 0x0802,
        0xa438, 0xd501, 0xa438, 0xce01, 0xa438, 0xd70c, 0xa438, 0x6087,
        0xa438, 0x0cf0, 0xa438, 0x0440, 0xa438, 0xf010, 0xa438, 0xa4f0,
        0xa438, 0xf00e, 0xa438, 0x0cf0, 0xa438, 0x0c80, 0xa438, 0xd505,
        0xa438, 0x0c0f, 0xa438, 0x0801, 0xa438, 0xd501, 0xa438, 0xce01,
        0xa438, 0xd70c, 0xa438, 0x6087, 0xa438, 0x0cf0, 0xa438, 0x0480,
        0xa438, 0xf002, 0xa438, 0xa4f0, 0xa438, 0x1800, 0xa438, 0x168c,
        0xa438, 0xd500, 0xa438, 0xd706, 0xa438, 0x2529, 0xa438, 0x80c8,
        0xa438, 0xd718, 0xa438, 0x607b, 0xa438, 0x40da, 0xa438, 0xf00f,
        0xa438, 0x431a, 0xa438, 0xf021, 0xa438, 0xd718, 0xa438, 0x617b,
        0xa438, 0x1000, 0xa438, 0x1a8a, 0xa438, 0x1000, 0xa438, 0x1b1a,
        0xa438, 0xd718, 0xa438, 0x608e, 0xa438, 0xd73e, 0xa438, 0x5f34,
        0xa438, 0xf020, 0xa438, 0xf053, 0xa438, 0x1000, 0xa438, 0x1a8a,
        0xa438, 0x1000, 0xa438, 0x1b1a, 0xa438, 0xd718, 0xa438, 0x608e,
        0xa438, 0xd73e, 0xa438, 0x5f34, 0xa438, 0xf023, 0xa438, 0xf067,
        0xa438, 0x1000, 0xa438, 0x1a8a, 0xa438, 0x1000, 0xa438, 0x1b1a,
        0xa438, 0xd718, 0xa438, 0x608e, 0xa438, 0xd73e, 0xa438, 0x5f34,
        0xa438, 0xf026, 0xa438, 0xf07b, 0xa438, 0x1000, 0xa438, 0x1a8a,
        0xa438, 0x1000, 0xa438, 0x1b1a, 0xa438, 0xd718, 0xa438, 0x608e,
        0xa438, 0xd73e, 0xa438, 0x5f34, 0xa438, 0xf029, 0xa438, 0xf08f,
        0xa438, 0x1000, 0xa438, 0x819f, 0xa438, 0x1000, 0xa438, 0x1a8a,
        0xa438, 0xd73e, 0xa438, 0x7fb4, 0xa438, 0x1000, 0xa438, 0x81b4,
        0xa438, 0x1000, 0xa438, 0x1a8a, 0xa438, 0xd718, 0xa438, 0x5fae,
        0xa438, 0xf028, 0xa438, 0x1000, 0xa438, 0x819f, 0xa438, 0x1000,
        0xa438, 0x1a8a, 0xa438, 0xd73e, 0xa438, 0x7fb4, 0xa438, 0x1000,
        0xa438, 0x81b4, 0xa438, 0x1000, 0xa438, 0x1a8a, 0xa438, 0xd718,
        0xa438, 0x5fae, 0xa438, 0xf039, 0xa438, 0x1000, 0xa438, 0x819f,
        0xa438, 0x1000, 0xa438, 0x1a8a, 0xa438, 0xd73e, 0xa438, 0x7fb4,
        0xa438, 0x1000, 0xa438, 0x81b4, 0xa438, 0x1000, 0xa438, 0x1a8a,
        0xa438, 0xd718, 0xa438, 0x5fae, 0xa438, 0xf04a, 0xa438, 0x1000,
        0xa438, 0x819f, 0xa438, 0x1000, 0xa438, 0x1a8a, 0xa438, 0xd73e,
        0xa438, 0x7fb4, 0xa438, 0x1000, 0xa438, 0x81b4, 0xa438, 0x1000,
        0xa438, 0x1a8a, 0xa438, 0xd718, 0xa438, 0x5fae, 0xa438, 0xf05b,
        0xa438, 0xd719, 0xa438, 0x4119, 0xa438, 0xd504, 0xa438, 0xac01,
        0xa438, 0xae01, 0xa438, 0xd500, 0xa438, 0x1000, 0xa438, 0x1a78,
        0xa438, 0xf00a, 0xa438, 0xd719, 0xa438, 0x4118, 0xa438, 0xd504,
        0xa438, 0xac11, 0xa438, 0xd501, 0xa438, 0xce01, 0xa438, 0xa410,
        0xa438, 0xce00, 0xa438, 0xd500, 0xa438, 0x1000, 0xa438, 0x1a8a,
        0xa438, 0xd718, 0xa438, 0x5fb0, 0xa438, 0xd505, 0xa438, 0xd719,
        0xa438, 0x4079, 0xa438, 0xa80f, 0xa438, 0xf05d, 0xa438, 0x4b98,
        0xa438, 0xa808, 0xa438, 0xf05a, 0xa438, 0xd719, 0xa438, 0x4119,
        0xa438, 0xd504, 0xa438, 0xac02, 0xa438, 0xae01, 0xa438, 0xd500,
        0xa438, 0x1000, 0xa438, 0x1a78, 0xa438, 0xf00a, 0xa438, 0xd719,
        0xa438, 0x4118, 0xa438, 0xd504, 0xa438, 0xac22, 0xa438, 0xd501,
        0xa438, 0xce01, 0xa438, 0xa420, 0xa438, 0xce00, 0xa438, 0xd500,
        0xa438, 0x1000, 0xa438, 0x1a8a, 0xa438, 0xd718, 0xa438, 0x5fb0,
        0xa438, 0xd505, 0xa438, 0xd719, 0xa438, 0x4079, 0xa438, 0xa80f,
        0xa438, 0xf03f, 0xa438, 0x47d8, 0xa438, 0xa804, 0xa438, 0xf03c,
        0xa438, 0xd719, 0xa438, 0x4119, 0xa438, 0xd504, 0xa438, 0xac04,
        0xa438, 0xae01, 0xa438, 0xd500, 0xa438, 0x1000, 0xa438, 0x1a78,
        0xa438, 0xf00a, 0xa438, 0xd719, 0xa438, 0x4118, 0xa438, 0xd504,
        0xa438, 0xac44, 0xa438, 0xd501, 0xa438, 0xce01, 0xa438, 0xa440,
        0xa438, 0xce00, 0xa438, 0xd500, 0xa438, 0x1000, 0xa438, 0x1a8a,
        0xa438, 0xd718, 0xa438, 0x5fb0, 0xa438, 0xd505, 0xa438, 0xd719,
        0xa438, 0x4079, 0xa438, 0xa80f, 0xa438, 0xf021, 0xa438, 0x4418,
        0xa438, 0xa802, 0xa438, 0xf01e, 0xa438, 0xd719, 0xa438, 0x4119,
        0xa438, 0xd504, 0xa438, 0xac08, 0xa438, 0xae01, 0xa438, 0xd500,
        0xa438, 0x1000, 0xa438, 0x1a78, 0xa438, 0xf00a, 0xa438, 0xd719,
        0xa438, 0x4118, 0xa438, 0xd504, 0xa438, 0xac88, 0xa438, 0xd501,
        0xa438, 0xce01, 0xa438, 0xa480, 0xa438, 0xce00, 0xa438, 0xd500,
        0xa438, 0x1000, 0xa438, 0x1a8a, 0xa438, 0xd718, 0xa438, 0x5fb0,
        0xa438, 0xd505, 0xa438, 0xd719, 0xa438, 0x4079, 0xa438, 0xa80f,
        0xa438, 0xf003, 0xa438, 0x4058, 0xa438, 0xa801, 0xa438, 0x1800,
        0xa438, 0x1736, 0xa438, 0xd73e, 0xa438, 0xd505, 0xa438, 0x3088,
        0xa438, 0x81a6, 0xa438, 0x6193, 0xa438, 0x6132, 0xa438, 0x60d1,
        0xa438, 0x3298, 0xa438, 0x81b1, 0xa438, 0xf00a, 0xa438, 0xa808,
        0xa438, 0xf008, 0xa438, 0xa804, 0xa438, 0xf006, 0xa438, 0xa802,
        0xa438, 0xf004, 0xa438, 0xa801, 0xa438, 0xf002, 0xa438, 0xa80f,
        0xa438, 0xd500, 0xa438, 0x0800, 0xa438, 0xd505, 0xa438, 0xd75e,
        0xa438, 0x6211, 0xa438, 0xd71e, 0xa438, 0x619b, 0xa438, 0x611a,
        0xa438, 0x6099, 0xa438, 0x0c0f, 0xa438, 0x0808, 0xa438, 0xf009,
        0xa438, 0x0c0f, 0xa438, 0x0804, 0xa438, 0xf006, 0xa438, 0x0c0f,
        0xa438, 0x0802, 0xa438, 0xf003, 0xa438, 0x0c0f, 0xa438, 0x0801,
        0xa438, 0xd500, 0xa438, 0x0800, 0xa436, 0xA026, 0xa438, 0xffff,
        0xa436, 0xA024, 0xa438, 0xffff, 0xa436, 0xA022, 0xa438, 0xffff,
        0xa436, 0xA020, 0xa438, 0xffff, 0xa436, 0xA006, 0xa438, 0xffff,
        0xa436, 0xA004, 0xa438, 0x16ab, 0xa436, 0xA002, 0xa438, 0x1663,
        0xa436, 0xA000, 0xa438, 0x1608, 0xa436, 0xA008, 0xa438, 0x0700,
        0xa436, 0xA016, 0xa438, 0x0000, 0xa436, 0xA012, 0xa438, 0x07f8,
        0xa436, 0xA014, 0xa438, 0xcc01, 0xa438, 0x0000, 0xa438, 0x0000,
        0xa438, 0x0000, 0xa438, 0x0000, 0xa438, 0x0000, 0xa438, 0x0000,
        0xa438, 0x0000, 0xa436, 0xA152, 0xa438, 0x021c, 0xa436, 0xA154,
        0xa438, 0x3fff, 0xa436, 0xA156, 0xa438, 0x3fff, 0xa436, 0xA158,
        0xa438, 0x3fff, 0xa436, 0xA15A, 0xa438, 0x3fff, 0xa436, 0xA15C,
        0xa438, 0x3fff, 0xa436, 0xA15E, 0xa438, 0x3fff, 0xa436, 0xA160,
        0xa438, 0x3fff, 0xa436, 0xA150, 0xa438, 0x0001, 0xa436, 0xA016,
        0xa438, 0x0010, 0xa436, 0xA012, 0xa438, 0x0000, 0xa436, 0xA014,
        0xa438, 0x1800, 0xa438, 0x8010, 0xa438, 0x1800, 0xa438, 0x8013,
        0xa438, 0x1800, 0xa438, 0x803a, 0xa438, 0x1800, 0xa438, 0x8045,
        0xa438, 0x1800, 0xa438, 0x8049, 0xa438, 0x1800, 0xa438, 0x804d,
        0xa438, 0x1800, 0xa438, 0x8059, 0xa438, 0x1800, 0xa438, 0x805d,
        0xa438, 0xc2ff, 0xa438, 0x1800, 0xa438, 0x0042, 0xa438, 0x1000,
        0xa438, 0x02e5, 0xa438, 0x1000, 0xa438, 0x02b4, 0xa438, 0xd701,
        0xa438, 0x40e3, 0xa438, 0xd700, 0xa438, 0x5f6c, 0xa438, 0x1000,
        0xa438, 0x8021, 0xa438, 0x1800, 0xa438, 0x0073, 0xa438, 0x1800,
        0xa438, 0x0084, 0xa438, 0xd701, 0xa438, 0x4061, 0xa438, 0xba0f,
        0xa438, 0xf004, 0xa438, 0x4060, 0xa438, 0x1000, 0xa438, 0x802a,
        0xa438, 0xba10, 0xa438, 0x0800, 0xa438, 0xd700, 0xa438, 0x60bb,
        0xa438, 0x611c, 0xa438, 0x0c0f, 0xa438, 0x1a01, 0xa438, 0xf00a,
        0xa438, 0x60fc, 0xa438, 0x0c0f, 0xa438, 0x1a02, 0xa438, 0xf006,
        0xa438, 0x0c0f, 0xa438, 0x1a04, 0xa438, 0xf003, 0xa438, 0x0c0f,
        0xa438, 0x1a08, 0xa438, 0x0800, 0xa438, 0x0c0f, 0xa438, 0x0504,
        0xa438, 0xad02, 0xa438, 0x1000, 0xa438, 0x02c0, 0xa438, 0xd700,
        0xa438, 0x5fac, 0xa438, 0x1000, 0xa438, 0x8021, 0xa438, 0x1800,
        0xa438, 0x0139, 0xa438, 0x9a1f, 0xa438, 0x8bf0, 0xa438, 0x1800,
        0xa438, 0x02df, 0xa438, 0x9a1f, 0xa438, 0x9910, 0xa438, 0x1800,
        0xa438, 0x02d7, 0xa438, 0xad02, 0xa438, 0x8d01, 0xa438, 0x9a1f,
        0xa438, 0x9910, 0xa438, 0x9860, 0xa438, 0xcb00, 0xa438, 0xd501,
        0xa438, 0xce01, 0xa438, 0x85f0, 0xa438, 0xd500, 0xa438, 0x1800,
        0xa438, 0x015c, 0xa438, 0x8580, 0xa438, 0x8d02, 0xa438, 0x1800,
        0xa438, 0x018f, 0xa438, 0x0c0f, 0xa438, 0x0503, 0xa438, 0xad02,
        0xa438, 0x1800, 0xa438, 0x00dd, 0xa436, 0xA08E, 0xa438, 0x00db,
        0xa436, 0xA08C, 0xa438, 0x018e, 0xa436, 0xA08A, 0xa438, 0x015a,
        0xa436, 0xA088, 0xa438, 0x02d6, 0xa436, 0xA086, 0xa438, 0x02de,
        0xa436, 0xA084, 0xa438, 0x0137, 0xa436, 0xA082, 0xa438, 0x0071,
        0xa436, 0xA080, 0xa438, 0x0041, 0xa436, 0xA090, 0xa438, 0x00ff,
        0xa436, 0xA016, 0xa438, 0x0020, 0xa436, 0xA012, 0xa438, 0x0000,
        0xa436, 0xA014, 0xa438, 0x1800, 0xa438, 0x8010, 0xa438, 0x1800,
        0xa438, 0x801d, 0xa438, 0x1800, 0xa438, 0x808a, 0xa438, 0x1800,
        0xa438, 0x80a1, 0xa438, 0x1800, 0xa438, 0x80b4, 0xa438, 0x1800,
        0xa438, 0x8104, 0xa438, 0x1800, 0xa438, 0x810b, 0xa438, 0x1800,
        0xa438, 0x810f, 0xa438, 0x8980, 0xa438, 0xd702, 0xa438, 0x6126,
        0xa438, 0xd704, 0xa438, 0x4063, 0xa438, 0xd702, 0xa438, 0x6060,
        0xa438, 0xd702, 0xa438, 0x6077, 0xa438, 0x1800, 0xa438, 0x0c29,
        0xa438, 0x1800, 0xa438, 0x0c2b, 0xa438, 0x1000, 0xa438, 0x115a,
        0xa438, 0xd71f, 0xa438, 0x5fb4, 0xa438, 0xd702, 0xa438, 0x6c46,
        0xa438, 0xd704, 0xa438, 0x4063, 0xa438, 0xd702, 0xa438, 0x6060,
        0xa438, 0xd702, 0xa438, 0x6b97, 0xa438, 0xa340, 0xa438, 0x0c06,
        0xa438, 0x0102, 0xa438, 0xce01, 0xa438, 0x1000, 0xa438, 0x117a,
        0xa438, 0xa240, 0xa438, 0xa902, 0xa438, 0xa204, 0xa438, 0xa280,
        0xa438, 0xa364, 0xa438, 0xab02, 0xa438, 0x8380, 0xa438, 0xa00a,
        0xa438, 0xcd8d, 0xa438, 0x1000, 0xa438, 0x115a, 0xa438, 0xd706,
        0xa438, 0x5fb5, 0xa438, 0xb920, 0xa438, 0x1000, 0xa438, 0x115a,
        0xa438, 0xd71f, 0xa438, 0x7fb4, 0xa438, 0x9920, 0xa438, 0x1000,
        0xa438, 0x115a, 0xa438, 0xd71f, 0xa438, 0x6065, 0xa438, 0x7c74,
        0xa438, 0xfffb, 0xa438, 0xb820, 0xa438, 0x1000, 0xa438, 0x115a,
        0xa438, 0xd71f, 0xa438, 0x7fa5, 0xa438, 0x9820, 0xa438, 0xa410,
        0xa438, 0x8902, 0xa438, 0xa120, 0xa438, 0xa380, 0xa438, 0xce02,
        0xa438, 0x1000, 0xa438, 0x117a, 0xa438, 0x8280, 0xa438, 0xa324,
        0xa438, 0xab02, 0xa438, 0xa00a, 0xa438, 0x8118, 0xa438, 0x863f,
        0xa438, 0x87fb, 0xa438, 0xcd8e, 0xa438, 0xd193, 0xa438, 0xd047,
        0xa438, 0x1000, 0xa438, 0x115a, 0xa438, 0x1000, 0xa438, 0x115f,
        0xa438, 0xd700, 0xa438, 0x5f7b, 0xa438, 0xa280, 0xa438, 0x1000,
        0xa438, 0x115a, 0xa438, 0x1000, 0xa438, 0x115f, 0xa438, 0xd706,
        0xa438, 0x5f78, 0xa438, 0xa210, 0xa438, 0xd700, 0xa438, 0x6083,
        0xa438, 0xd101, 0xa438, 0xd047, 0xa438, 0xf003, 0xa438, 0xd160,
        0xa438, 0xd04b, 0xa438, 0x1000, 0xa438, 0x115a, 0xa438, 0x1000,
        0xa438, 0x115f, 0xa438, 0xd700, 0xa438, 0x5f7b, 0xa438, 0x1000,
        0xa438, 0x115a, 0xa438, 0x1000, 0xa438, 0x115f, 0xa438, 0xd706,
        0xa438, 0x5f79, 0xa438, 0x8120, 0xa438, 0xbb20, 0xa438, 0x1800,
        0xa438, 0x0c8b, 0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0x8f80,
        0xa438, 0x9503, 0xa438, 0x1800, 0xa438, 0x0c3c, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0x8f80, 0xa438, 0x9503, 0xa438, 0xd704,
        0xa438, 0x6192, 0xa438, 0xd702, 0xa438, 0x4116, 0xa438, 0xce04,
        0xa438, 0x1000, 0xa438, 0x117a, 0xa438, 0x0c03, 0xa438, 0x1502,
        0xa438, 0x8f40, 0xa438, 0x9503, 0xa438, 0x1800, 0xa438, 0x0b3d,
        0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0xaf40, 0xa438, 0x9503,
        0xa438, 0x1800, 0xa438, 0x0b48, 0xa438, 0xd704, 0xa438, 0x6192,
        0xa438, 0xd702, 0xa438, 0x4116, 0xa438, 0xce04, 0xa438, 0x1000,
        0xa438, 0x117a, 0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0x8f40,
        0xa438, 0x9503, 0xa438, 0x1800, 0xa438, 0x1269, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0xaf40, 0xa438, 0x9503, 0xa438, 0x1800,
        0xa438, 0x1274, 0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0xa608,
        0xa438, 0xc700, 0xa438, 0x9503, 0xa438, 0xce54, 0xa438, 0x1000,
        0xa438, 0x117a, 0xa438, 0xa290, 0xa438, 0xa304, 0xa438, 0xab02,
        0xa438, 0xd700, 0xa438, 0x6050, 0xa438, 0xab04, 0xa438, 0x0c38,
        0xa438, 0x0608, 0xa438, 0xaa0b, 0xa438, 0xd702, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0x8d01, 0xa438, 0xae40, 0xa438, 0x4044,
        0xa438, 0x8e20, 0xa438, 0x9503, 0xa438, 0x0c03, 0xa438, 0x1502,
        0xa438, 0x8c20, 0xa438, 0x9503, 0xa438, 0xd700, 0xa438, 0x6078,
        0xa438, 0xd700, 0xa438, 0x609a, 0xa438, 0xd109, 0xa438, 0xd074,
        0xa438, 0xf003, 0xa438, 0xd109, 0xa438, 0xd075, 0xa438, 0x1000,
        0xa438, 0x115a, 0xa438, 0xd704, 0xa438, 0x6252, 0xa438, 0xd702,
        0xa438, 0x4116, 0xa438, 0xce54, 0xa438, 0x1000, 0xa438, 0x117a,
        0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0x8f40, 0xa438, 0x9503,
        0xa438, 0xa00a, 0xa438, 0xd704, 0xa438, 0x41e7, 0xa438, 0x0c03,
        0xa438, 0x1502, 0xa438, 0xa570, 0xa438, 0x9503, 0xa438, 0xf00a,
        0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0xaf40, 0xa438, 0x9503,
        0xa438, 0x800a, 0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0x8570,
        0xa438, 0x9503, 0xa438, 0xd704, 0xa438, 0x60f3, 0xa438, 0xd71f,
        0xa438, 0x60ee, 0xa438, 0xd700, 0xa438, 0x5bbe, 0xa438, 0x1800,
        0xa438, 0x0e71, 0xa438, 0x1800, 0xa438, 0x0e7c, 0xa438, 0x1800,
        0xa438, 0x0e7e, 0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0xaf80,
        0xa438, 0x9503, 0xa438, 0xcd62, 0xa438, 0x1800, 0xa438, 0x0bd2,
        0xa438, 0x800a, 0xa438, 0x8306, 0xa438, 0x1800, 0xa438, 0x0cb6,
        0xa438, 0x0c03, 0xa438, 0x1502, 0xa438, 0x8608, 0xa438, 0x8c20,
        0xa438, 0x9503, 0xa438, 0x1800, 0xa438, 0x0eb9, 0xa436, 0xA10E,
        0xa438, 0x0eb5, 0xa436, 0xA10C, 0xa438, 0x0cb5, 0xa436, 0xA10A,
        0xa438, 0x0bd1, 0xa436, 0xA108, 0xa438, 0x0e37, 0xa436, 0xA106,
        0xa438, 0x1267, 0xa436, 0xA104, 0xa438, 0x0b3b, 0xa436, 0xA102,
        0xa438, 0x0c38, 0xa436, 0xA100, 0xa438, 0x0c24, 0xa436, 0xA110,
        0xa438, 0x00ff, 0xa436, 0xb87c, 0xa438, 0x85bf, 0xa436, 0xb87e,
        0xa438, 0xaf85, 0xa438, 0xd7af, 0xa438, 0x85fb, 0xa438, 0xaf86,
        0xa438, 0x10af, 0xa438, 0x8638, 0xa438, 0xaf86, 0xa438, 0x47af,
        0xa438, 0x8647, 0xa438, 0xaf86, 0xa438, 0x47af, 0xa438, 0x8647,
        0xa438, 0xbf85, 0xa438, 0xf802, 0xa438, 0x627f, 0xa438, 0xbf61,
        0xa438, 0xc702, 0xa438, 0x627f, 0xa438, 0xae0c, 0xa438, 0xbf85,
        0xa438, 0xf802, 0xa438, 0x6276, 0xa438, 0xbf61, 0xa438, 0xc702,
        0xa438, 0x6276, 0xa438, 0xee85, 0xa438, 0x4200, 0xa438, 0xaf1b,
        0xa438, 0x2333, 0xa438, 0xa484, 0xa438, 0xbf86, 0xa438, 0x0a02,
        0xa438, 0x627f, 0xa438, 0xbf86, 0xa438, 0x0d02, 0xa438, 0x627f,
        0xa438, 0xaf1b, 0xa438, 0x8422, 0xa438, 0xa484, 0xa438, 0x66ac,
        0xa438, 0x0ef8, 0xa438, 0xfbef, 0xa438, 0x79fb, 0xa438, 0xe080,
        0xa438, 0x16ad, 0xa438, 0x230f, 0xa438, 0xee85, 0xa438, 0x4200,
        0xa438, 0x1f44, 0xa438, 0xbf86, 0xa438, 0x30d7, 0xa438, 0x0008,
        0xa438, 0x0264, 0xa438, 0xa3ff, 0xa438, 0xef97, 0xa438, 0xfffc,
        0xa438, 0x0485, 0xa438, 0xf861, 0xa438, 0xc786, 0xa438, 0x0a86,
        0xa438, 0x0de1, 0xa438, 0x8feb, 0xa438, 0xe583, 0xa438, 0x20e1,
        0xa438, 0x8fea, 0xa438, 0xe583, 0xa438, 0x21af, 0xa438, 0x41a7,
        0xa436, 0xb85e, 0xa438, 0x1b05, 0xa436, 0xb860, 0xa438, 0x1b78,
        0xa436, 0xb862, 0xa438, 0x1a08, 0xa436, 0xb864, 0xa438, 0x419F,
        0xa436, 0xb886, 0xa438, 0xffff, 0xa436, 0xb888, 0xa438, 0xffff,
        0xa436, 0xb88a, 0xa438, 0xffff, 0xa436, 0xb88c, 0xa438, 0xffff,
        0xa436, 0xb838, 0xa438, 0x000f, 0xb820, 0x0010, 0xa436, 0x0000,
        0xa438, 0x0000, 0xB82E, 0x0000, 0xa436, 0x8023, 0xa438, 0x0000,
        0xa436, 0x801E, 0xa438, 0x0008, 0xB820, 0x0000, 0xFFFF, 0xFFFF
};

static const u16  phy_mcu_ram_code_8125d_2_2[] = {
        0xa436, 0xacca, 0xa438, 0x0104, 0xa436, 0xaccc, 0xa438, 0x8000,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x0fff,
        0xa436, 0xacce, 0xa438, 0xfd47, 0xa436, 0xacd0, 0xa438, 0x0fff,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xe56f, 0xa436, 0xacd0, 0xa438, 0x01c0,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xed97, 0xa436, 0xacd0, 0xa438, 0x01c8,
        0xa436, 0xacce, 0xa438, 0xffff, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xf5bf, 0xa436, 0xacd0, 0xa438, 0x01d0,
        0xa436, 0xacce, 0xa438, 0xfb07, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb0f, 0xa436, 0xacd0, 0xa438, 0x01d8,
        0xa436, 0xacce, 0xa438, 0xa087, 0xa436, 0xacd0, 0xa438, 0x0180,
        0xa436, 0xacce, 0xa438, 0xa00f, 0xa436, 0xacd0, 0xa438, 0x0108,
        0xa436, 0xacce, 0xa438, 0xa807, 0xa436, 0xacd0, 0xa438, 0x0100,
        0xa436, 0xacce, 0xa438, 0xa88f, 0xa436, 0xacd0, 0xa438, 0x0188,
        0xa436, 0xacce, 0xa438, 0xb027, 0xa436, 0xacd0, 0xa438, 0x0120,
        0xa436, 0xacce, 0xa438, 0xb02f, 0xa436, 0xacd0, 0xa438, 0x0128,
        0xa436, 0xacce, 0xa438, 0xb847, 0xa436, 0xacd0, 0xa438, 0x0140,
        0xa436, 0xacce, 0xa438, 0xb84f, 0xa436, 0xacd0, 0xa438, 0x0148,
        0xa436, 0xacce, 0xa438, 0xfb17, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb1f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xa017, 0xa436, 0xacd0, 0xa438, 0x0110,
        0xa436, 0xacce, 0xa438, 0xa01f, 0xa436, 0xacd0, 0xa438, 0x0118,
        0xa436, 0xacce, 0xa438, 0xa837, 0xa436, 0xacd0, 0xa438, 0x0130,
        0xa436, 0xacce, 0xa438, 0xa83f, 0xa436, 0xacd0, 0xa438, 0x0138,
        0xa436, 0xacce, 0xa438, 0xb097, 0xa436, 0xacd0, 0xa438, 0x0190,
        0xa436, 0xacce, 0xa438, 0xb05f, 0xa436, 0xacd0, 0xa438, 0x0158,
        0xa436, 0xacce, 0xa438, 0xb857, 0xa436, 0xacd0, 0xa438, 0x0150,
        0xa436, 0xacce, 0xa438, 0xb89f, 0xa436, 0xacd0, 0xa438, 0x0198,
        0xa436, 0xacce, 0xa438, 0xfb27, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb2f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x8087, 0xa436, 0xacd0, 0xa438, 0x0180,
        0xa436, 0xacce, 0xa438, 0x800f, 0xa436, 0xacd0, 0xa438, 0x0108,
        0xa436, 0xacce, 0xa438, 0x8807, 0xa436, 0xacd0, 0xa438, 0x0100,
        0xa436, 0xacce, 0xa438, 0x888f, 0xa436, 0xacd0, 0xa438, 0x0188,
        0xa436, 0xacce, 0xa438, 0x9027, 0xa436, 0xacd0, 0xa438, 0x0120,
        0xa436, 0xacce, 0xa438, 0x902f, 0xa436, 0xacd0, 0xa438, 0x0128,
        0xa436, 0xacce, 0xa438, 0x9847, 0xa436, 0xacd0, 0xa438, 0x0140,
        0xa436, 0xacce, 0xa438, 0x984f, 0xa436, 0xacd0, 0xa438, 0x0148,
        0xa436, 0xacce, 0xa438, 0xa0a7, 0xa436, 0xacd0, 0xa438, 0x01a0,
        0xa436, 0xacce, 0xa438, 0xa8af, 0xa436, 0xacd0, 0xa438, 0x01a8,
        0xa436, 0xacce, 0xa438, 0xa067, 0xa436, 0xacd0, 0xa438, 0x0161,
        0xa436, 0xacce, 0xa438, 0xa86f, 0xa436, 0xacd0, 0xa438, 0x0169,
        0xa436, 0xacce, 0xa438, 0xfb37, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb3f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x8017, 0xa436, 0xacd0, 0xa438, 0x0110,
        0xa436, 0xacce, 0xa438, 0x801f, 0xa436, 0xacd0, 0xa438, 0x0118,
        0xa436, 0xacce, 0xa438, 0x8837, 0xa436, 0xacd0, 0xa438, 0x0130,
        0xa436, 0xacce, 0xa438, 0x883f, 0xa436, 0xacd0, 0xa438, 0x0138,
        0xa436, 0xacce, 0xa438, 0x9097, 0xa436, 0xacd0, 0xa438, 0x0190,
        0xa436, 0xacce, 0xa438, 0x905f, 0xa436, 0xacd0, 0xa438, 0x0158,
        0xa436, 0xacce, 0xa438, 0x9857, 0xa436, 0xacd0, 0xa438, 0x0150,
        0xa436, 0xacce, 0xa438, 0x989f, 0xa436, 0xacd0, 0xa438, 0x0198,
        0xa436, 0xacce, 0xa438, 0xb0b7, 0xa436, 0xacd0, 0xa438, 0x01b0,
        0xa436, 0xacce, 0xa438, 0xb8bf, 0xa436, 0xacd0, 0xa438, 0x01b8,
        0xa436, 0xacce, 0xa438, 0xb077, 0xa436, 0xacd0, 0xa438, 0x0171,
        0xa436, 0xacce, 0xa438, 0xb87f, 0xa436, 0xacd0, 0xa438, 0x0179,
        0xa436, 0xacce, 0xa438, 0xfb47, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb4f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x6087, 0xa436, 0xacd0, 0xa438, 0x0180,
        0xa436, 0xacce, 0xa438, 0x600f, 0xa436, 0xacd0, 0xa438, 0x0108,
        0xa436, 0xacce, 0xa438, 0x6807, 0xa436, 0xacd0, 0xa438, 0x0100,
        0xa436, 0xacce, 0xa438, 0x688f, 0xa436, 0xacd0, 0xa438, 0x0188,
        0xa436, 0xacce, 0xa438, 0x7027, 0xa436, 0xacd0, 0xa438, 0x0120,
        0xa436, 0xacce, 0xa438, 0x702f, 0xa436, 0xacd0, 0xa438, 0x0128,
        0xa436, 0xacce, 0xa438, 0x7847, 0xa436, 0xacd0, 0xa438, 0x0140,
        0xa436, 0xacce, 0xa438, 0x784f, 0xa436, 0xacd0, 0xa438, 0x0148,
        0xa436, 0xacce, 0xa438, 0x80a7, 0xa436, 0xacd0, 0xa438, 0x01a0,
        0xa436, 0xacce, 0xa438, 0x88af, 0xa436, 0xacd0, 0xa438, 0x01a8,
        0xa436, 0xacce, 0xa438, 0x8067, 0xa436, 0xacd0, 0xa438, 0x0161,
        0xa436, 0xacce, 0xa438, 0x886f, 0xa436, 0xacd0, 0xa438, 0x0169,
        0xa436, 0xacce, 0xa438, 0xfb57, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb5f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x6017, 0xa436, 0xacd0, 0xa438, 0x0110,
        0xa436, 0xacce, 0xa438, 0x601f, 0xa436, 0xacd0, 0xa438, 0x0118,
        0xa436, 0xacce, 0xa438, 0x6837, 0xa436, 0xacd0, 0xa438, 0x0130,
        0xa436, 0xacce, 0xa438, 0x683f, 0xa436, 0xacd0, 0xa438, 0x0138,
        0xa436, 0xacce, 0xa438, 0x7097, 0xa436, 0xacd0, 0xa438, 0x0190,
        0xa436, 0xacce, 0xa438, 0x705f, 0xa436, 0xacd0, 0xa438, 0x0158,
        0xa436, 0xacce, 0xa438, 0x7857, 0xa436, 0xacd0, 0xa438, 0x0150,
        0xa436, 0xacce, 0xa438, 0x789f, 0xa436, 0xacd0, 0xa438, 0x0198,
        0xa436, 0xacce, 0xa438, 0x90b7, 0xa436, 0xacd0, 0xa438, 0x01b0,
        0xa436, 0xacce, 0xa438, 0x98bf, 0xa436, 0xacd0, 0xa438, 0x01b8,
        0xa436, 0xacce, 0xa438, 0x9077, 0xa436, 0xacd0, 0xa438, 0x0171,
        0xa436, 0xacce, 0xa438, 0x987f, 0xa436, 0xacd0, 0xa438, 0x0179,
        0xa436, 0xacce, 0xa438, 0xfb67, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb6f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x4087, 0xa436, 0xacd0, 0xa438, 0x0180,
        0xa436, 0xacce, 0xa438, 0x400f, 0xa436, 0xacd0, 0xa438, 0x0108,
        0xa436, 0xacce, 0xa438, 0x4807, 0xa436, 0xacd0, 0xa438, 0x0100,
        0xa436, 0xacce, 0xa438, 0x488f, 0xa436, 0xacd0, 0xa438, 0x0188,
        0xa436, 0xacce, 0xa438, 0x5027, 0xa436, 0xacd0, 0xa438, 0x0120,
        0xa436, 0xacce, 0xa438, 0x502f, 0xa436, 0xacd0, 0xa438, 0x0128,
        0xa436, 0xacce, 0xa438, 0x5847, 0xa436, 0xacd0, 0xa438, 0x0140,
        0xa436, 0xacce, 0xa438, 0x584f, 0xa436, 0xacd0, 0xa438, 0x0148,
        0xa436, 0xacce, 0xa438, 0x60a7, 0xa436, 0xacd0, 0xa438, 0x01a0,
        0xa436, 0xacce, 0xa438, 0x68af, 0xa436, 0xacd0, 0xa438, 0x01a8,
        0xa436, 0xacce, 0xa438, 0x6067, 0xa436, 0xacd0, 0xa438, 0x0161,
        0xa436, 0xacce, 0xa438, 0x686f, 0xa436, 0xacd0, 0xa438, 0x0169,
        0xa436, 0xacce, 0xa438, 0xfb77, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb7f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x4017, 0xa436, 0xacd0, 0xa438, 0x0110,
        0xa436, 0xacce, 0xa438, 0x401f, 0xa436, 0xacd0, 0xa438, 0x0118,
        0xa436, 0xacce, 0xa438, 0x4837, 0xa436, 0xacd0, 0xa438, 0x0130,
        0xa436, 0xacce, 0xa438, 0x483f, 0xa436, 0xacd0, 0xa438, 0x0138,
        0xa436, 0xacce, 0xa438, 0x5097, 0xa436, 0xacd0, 0xa438, 0x0190,
        0xa436, 0xacce, 0xa438, 0x505f, 0xa436, 0xacd0, 0xa438, 0x0158,
        0xa436, 0xacce, 0xa438, 0x5857, 0xa436, 0xacd0, 0xa438, 0x0150,
        0xa436, 0xacce, 0xa438, 0x589f, 0xa436, 0xacd0, 0xa438, 0x0198,
        0xa436, 0xacce, 0xa438, 0x70b7, 0xa436, 0xacd0, 0xa438, 0x01b0,
        0xa436, 0xacce, 0xa438, 0x78bf, 0xa436, 0xacd0, 0xa438, 0x01b8,
        0xa436, 0xacce, 0xa438, 0x7077, 0xa436, 0xacd0, 0xa438, 0x0171,
        0xa436, 0xacce, 0xa438, 0x787f, 0xa436, 0xacd0, 0xa438, 0x0179,
        0xa436, 0xacce, 0xa438, 0xfb87, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb8f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x40a7, 0xa436, 0xacd0, 0xa438, 0x01a0,
        0xa436, 0xacce, 0xa438, 0x48af, 0xa436, 0xacd0, 0xa438, 0x01a8,
        0xa436, 0xacce, 0xa438, 0x4067, 0xa436, 0xacd0, 0xa438, 0x0161,
        0xa436, 0xacce, 0xa438, 0x486f, 0xa436, 0xacd0, 0xa438, 0x0169,
        0xa436, 0xacce, 0xa438, 0xfb97, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfb9f, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x50b7, 0xa436, 0xacd0, 0xa438, 0x01b0,
        0xa436, 0xacce, 0xa438, 0x58bf, 0xa436, 0xacd0, 0xa438, 0x01b8,
        0xa436, 0xacce, 0xa438, 0x5077, 0xa436, 0xacd0, 0xa438, 0x0171,
        0xa436, 0xacce, 0xa438, 0x587f, 0xa436, 0xacd0, 0xa438, 0x0179,
        0xa436, 0xacce, 0xa438, 0xfba7, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfbaf, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x2067, 0xa436, 0xacd0, 0xa438, 0x0161,
        0xa436, 0xacce, 0xa438, 0x286f, 0xa436, 0xacd0, 0xa438, 0x0169,
        0xa436, 0xacce, 0xa438, 0xfbb7, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0xfbbf, 0xa436, 0xacd0, 0xa438, 0x07ff,
        0xa436, 0xacce, 0xa438, 0x3077, 0xa436, 0xacd0, 0xa438, 0x0171,
        0xa436, 0xacce, 0xa438, 0x387f, 0xa436, 0xacd0, 0xa438, 0x0179,
        0xa436, 0xacce, 0xa438, 0xfff9, 0xa436, 0xacd0, 0xa438, 0x17ff,
        0xa436, 0xacce, 0xa438, 0xfff9, 0xa436, 0xacd0, 0xa438, 0x17ff,
        0xa436, 0xacca, 0xa438, 0x0004, 0xa436, 0xacc6, 0xa438, 0x0008,
        0xa436, 0xacc8, 0xa438, 0xc000, 0xa436, 0xacc8, 0xa438, 0x0000,
        0xB820, 0x0000, 0xFFFF, 0xFFFF
};

static const u16 phy_mcu_ram_code_8125bp_1_1[] = {
        0xa436, 0x8024, 0xa438, 0x3600, 0xa436, 0xB82E, 0xa438, 0x0001,
        0xb820, 0x0090, 0xa436, 0xA016, 0xa438, 0x0000, 0xa436, 0xA012,
        0xa438, 0x0000, 0xa436, 0xA014, 0xa438, 0x1800, 0xa438, 0x8010,
        0xa438, 0x1800, 0xa438, 0x8014, 0xa438, 0x1800, 0xa438, 0x8018,
        0xa438, 0x1800, 0xa438, 0x801c, 0xa438, 0x1800, 0xa438, 0x8020,
        0xa438, 0x1800, 0xa438, 0x8024, 0xa438, 0x1800, 0xa438, 0x8028,
        0xa438, 0x1800, 0xa438, 0x8028, 0xa438, 0xdb20, 0xa438, 0xd501,
        0xa438, 0x1800, 0xa438, 0x034c, 0xa438, 0xdb10, 0xa438, 0xd501,
        0xa438, 0x1800, 0xa438, 0x032c, 0xa438, 0x8620, 0xa438, 0xa480,
        0xa438, 0x1800, 0xa438, 0x1cfe, 0xa438, 0xbf40, 0xa438, 0xd703,
        0xa438, 0x1800, 0xa438, 0x0ce9, 0xa438, 0x9c10, 0xa438, 0x9f40,
        0xa438, 0x1800, 0xa438, 0x137a, 0xa438, 0x9f20, 0xa438, 0x9f40,
        0xa438, 0x1800, 0xa438, 0x16c4, 0xa436, 0xA026, 0xa438, 0xffff,
        0xa436, 0xA024, 0xa438, 0xffff, 0xa436, 0xA022, 0xa438, 0x16c3,
        0xa436, 0xA020, 0xa438, 0x1379, 0xa436, 0xA006, 0xa438, 0x0ce8,
        0xa436, 0xA004, 0xa438, 0x1cfd, 0xa436, 0xA002, 0xa438, 0x032b,
        0xa436, 0xA000, 0xa438, 0x034b, 0xa436, 0xA008, 0xa438, 0x3f00,
        0xa436, 0xA016, 0xa438, 0x0020, 0xa436, 0xA012, 0xa438, 0x0000,
        0xa436, 0xA014, 0xa438, 0x1800, 0xa438, 0x8010, 0xa438, 0x1800,
        0xa438, 0x8018, 0xa438, 0x1800, 0xa438, 0x8021, 0xa438, 0x1800,
        0xa438, 0x802b, 0xa438, 0x1800, 0xa438, 0x8055, 0xa438, 0x1800,
        0xa438, 0x805a, 0xa438, 0x1800, 0xa438, 0x805e, 0xa438, 0x1800,
        0xa438, 0x8062, 0xa438, 0x0000, 0xa438, 0x0000, 0xa438, 0xcb11,
        0xa438, 0xd1b9, 0xa438, 0xd05b, 0xa438, 0x0000, 0xa438, 0x1800,
        0xa438, 0x0284, 0xa438, 0x0000, 0xa438, 0x0000, 0xa438, 0xd700,
        0xa438, 0x5fb4, 0xa438, 0x5f95, 0xa438, 0x0000, 0xa438, 0x0000,
        0xa438, 0x1800, 0xa438, 0x02b7, 0xa438, 0x0000, 0xa438, 0x0000,
        0xa438, 0xcb21, 0xa438, 0x1000, 0xa438, 0x0b34, 0xa438, 0xd71f,
        0xa438, 0x5f5e, 0xa438, 0x0000, 0xa438, 0x1800, 0xa438, 0x0322,
        0xa438, 0xd700, 0xa438, 0xd113, 0xa438, 0xd040, 0xa438, 0x1000,
        0xa438, 0x0a57, 0xa438, 0xd700, 0xa438, 0x5fb4, 0xa438, 0xd700,
        0xa438, 0x6065, 0xa438, 0xd122, 0xa438, 0xf002, 0xa438, 0xd122,
        0xa438, 0xd040, 0xa438, 0x1000, 0xa438, 0x0b53, 0xa438, 0xa008,
        0xa438, 0xd704, 0xa438, 0x4052, 0xa438, 0xa002, 0xa438, 0xd704,
        0xa438, 0x4054, 0xa438, 0xa740, 0xa438, 0x1000, 0xa438, 0x0a57,
        0xa438, 0xd700, 0xa438, 0x5fb4, 0xa438, 0xcb9b, 0xa438, 0xd110,
        0xa438, 0xd040, 0xa438, 0x1000, 0xa438, 0x0c01, 0xa438, 0x1000,
        0xa438, 0x0a57, 0xa438, 0xd700, 0xa438, 0x5fb4, 0xa438, 0x801a,
        0xa438, 0x1000, 0xa438, 0x0a57, 0xa438, 0xd704, 0xa438, 0x7fb9,
        0xa438, 0x1800, 0xa438, 0x088d, 0xa438, 0xcb62, 0xa438, 0xd700,
        0xa438, 0x8880, 0xa438, 0x1800, 0xa438, 0x06cb, 0xa438, 0xbe02,
        0xa438, 0x0000, 0xa438, 0x1800, 0xa438, 0x002c, 0xa438, 0xbe04,
        0xa438, 0x0000, 0xa438, 0x1800, 0xa438, 0x002c, 0xa438, 0xbe08,
        0xa438, 0x0000, 0xa438, 0x1800, 0xa438, 0x002c, 0xa436, 0xA10E,
        0xa438, 0x802a, 0xa436, 0xA10C, 0xa438, 0x8026, 0xa436, 0xA10A,
        0xa438, 0x8022, 0xa436, 0xA108, 0xa438, 0x06ca, 0xa436, 0xA106,
        0xa438, 0x086f, 0xa436, 0xA104, 0xa438, 0x0321, 0xa436, 0xA102,
        0xa438, 0x02b5, 0xa436, 0xA100, 0xa438, 0x0283, 0xa436, 0xA110,
        0xa438, 0x001f, 0xb820, 0x0010, 0xb82e, 0x0000, 0xa436, 0x8024,
        0xa438, 0x0000, 0xB820, 0x0000, 0xFFFF, 0xFFFF
};

static const u16 phy_mcu_ram_code_8125bp_1_2[] = {
        0xb892, 0x0000, 0xb88e, 0xC201, 0xb890, 0x2C01, 0xb890, 0xCD02,
        0xb890, 0x0602, 0xb890, 0x5502, 0xb890, 0xB903, 0xb890, 0x3303,
        0xb890, 0xC204, 0xb890, 0x6605, 0xb890, 0x1F05, 0xb890, 0xEE06,
        0xb890, 0xD207, 0xb890, 0xCC08, 0xb890, 0xDA09, 0xb890, 0xFF0B,
        0xb890, 0x380C, 0xb890, 0x87F3, 0xb88e, 0xC27F, 0xb890, 0x2B66,
        0xb890, 0x6666, 0xb890, 0x6666, 0xb890, 0x6666, 0xb890, 0x6666,
        0xb890, 0x6666, 0xb890, 0x6666, 0xb890, 0x6666, 0xb890, 0x66C2,
        0xb88e, 0xC26F, 0xb890, 0x751D, 0xb890, 0x1D1F, 0xb890, 0x2022,
        0xb890, 0x2325, 0xb890, 0x2627, 0xb890, 0x2829, 0xb890, 0x2929,
        0xb890, 0x2A2A, 0xb890, 0x2B66, 0xB820, 0x0000, 0xFFFF, 0xFFFF
};

static void
rtl8125_real_set_phy_mcu_8125b_1(struct rtl8125_private *tp)
{
        rtl8125_set_phy_mcu_ram_code(tp, phy_mcu_ram_code_8125b_1,
                                     ARRAY_SIZE(phy_mcu_ram_code_8125b_1));
}

static void
rtl8125_set_phy_mcu_8125b_1(struct rtl8125_private *tp)
{
        rtl8125_set_phy_mcu_patch_request(tp);
        rtl8125_real_set_phy_mcu_8125b_1(tp);
        rtl8125_clear_phy_mcu_patch_request(tp);
}

static void
rtl8125_real_set_phy_mcu_8125b_2(struct rtl8125_private *tp)
{
        rtl8125_set_phy_mcu_ram_code(tp, phy_mcu_ram_code_8125b_2,
                                     ARRAY_SIZE(phy_mcu_ram_code_8125b_2));
}

static void
rtl8125_set_phy_mcu_8125b_2(struct rtl8125_private *tp)
{
        rtl8125_set_phy_mcu_patch_request(tp);
        rtl8125_real_set_phy_mcu_8125b_2(tp);
        rtl8125_clear_phy_mcu_patch_request(tp);
}

static void
rtl8125_real_set_phy_mcu_8125d_1_1(struct rtl8125_private *tp)
{
        rtl8125_set_phy_mcu_ram_code(tp, phy_mcu_ram_code_8125d_1_1,
                                     ARRAY_SIZE(phy_mcu_ram_code_8125d_1_1));
}

static void
rtl8125_real_set_phy_mcu_8125d_1_2(struct rtl8125_private *tp)
{
        rtl8125_set_phy_mcu_ram_code(tp, phy_mcu_ram_code_8125d_1_2,
                                     ARRAY_SIZE(phy_mcu_ram_code_8125d_1_2));
}

static void
rtl8125_real_set_phy_mcu_8125d_1_3(struct rtl8125_private *tp)
{
        rtl8125_set_phy_mcu_ram_code(tp, phy_mcu_ram_code_8125d_1_3,
                                     ARRAY_SIZE(phy_mcu_ram_code_8125d_1_3));
}

static void
rtl8125_set_phy_mcu_8125d_1(struct rtl8125_private *tp)
{
        rtl8125_set_phy_mcu_patch_request(tp);
        rtl8125_real_set_phy_mcu_8125d_1_1(tp);
        rtl8125_clear_phy_mcu_patch_request(tp);
        rtl8125_set_phy_mcu_patch_request(tp);
        rtl8125_real_set_phy_mcu_8125d_1_2(tp);
        rtl8125_clear_phy_mcu_patch_request(tp);
        rtl8125_set_phy_mcu_patch_request(tp);
        rtl8125_real_set_phy_mcu_8125d_1_3(tp);
        rtl8125_clear_phy_mcu_patch_request(tp);
}

static void
rtl8125_real_set_phy_mcu_8125bp_1_1(struct rtl8125_private *tp)
{
        rtl8125_set_phy_mcu_ram_code(tp, phy_mcu_ram_code_8125bp_1_1,
                                     ARRAY_SIZE(phy_mcu_ram_code_8125bp_1_1));
}

static void
rtl8125_real_set_phy_mcu_8125bp_1_2(struct rtl8125_private *tp)
{
        rtl8125_set_phy_mcu_ram_code(tp, phy_mcu_ram_code_8125bp_1_2,
                                     ARRAY_SIZE(phy_mcu_ram_code_8125bp_1_2));
}

static void
rtl8125_set_phy_mcu_8125bp_1(struct rtl8125_private *tp)
{
        rtl8125_set_phy_mcu_patch_request(tp);
        rtl8125_real_set_phy_mcu_8125bp_1_1(tp);
        rtl8125_clear_phy_mcu_patch_request(tp);
        rtl8125_set_phy_mcu_patch_request(tp);
        rtl8125_real_set_phy_mcu_8125bp_1_2(tp);
        rtl8125_clear_phy_mcu_patch_request(tp);
}

static void
rtl8125_init_hw_phy_mcu(struct rtl8125_private *tp)
{
        u8 require_disable_phy_disable_mode = FALSE;

        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
                return;

        if (rtl8125_is_in_phy_disable_mode(tp))
                require_disable_phy_disable_mode = TRUE;

        if (require_disable_phy_disable_mode)
                rtl8125_disable_phy_disable_mode(tp);

        switch (tp->mcfg) {
        case CFG_METHOD_2:
                rtl8125_set_phy_mcu_8125a_1(tp);
                break;
        case CFG_METHOD_3:
                rtl8125_set_phy_mcu_8125a_2(tp);
                break;
        case CFG_METHOD_4:
                rtl8125_set_phy_mcu_8125b_1(tp);
                break;
        case CFG_METHOD_5:
                rtl8125_set_phy_mcu_8125b_2(tp);
                break;
        case CFG_METHOD_8:
                rtl8125_set_phy_mcu_8125bp_1(tp);
                break;
        case CFG_METHOD_9:
                /* nothing to do */
                break;
        case CFG_METHOD_10:
                rtl8125_set_phy_mcu_8125d_1(tp);
                break;
        case CFG_METHOD_11:
                rtl8125_set_phy_mcu_8125d_2(dev);
                break;
        }

        if (require_disable_phy_disable_mode)
                rtl8125_enable_phy_disable_mode(tp);

        rtl8125_write_hw_phy_mcu_code_ver(tp);
        rtl8125_mdio_set_page(tp, 0x0000);
        rtl8125_set_flag(tp, HwHasWrRamCodeToMicroP);
}
#endif

static void
rtl8125_enable_phy_aldps(struct rtl8125_private *tp)
{
        //enable aldps
        //GPHY OCP 0xA430 bit[2] = 0x1 (en_aldps)
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA430, BIT_2);
}

static void
rtl8125_hw_phy_config_8125a_1(struct rtl8125_private *tp)
{
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAD40, 0x03FF, 0x84);

        rtl8125_set_eth_phy_ocp_bit(tp, 0xAD4E, BIT_4);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAD16, 0x03FF, 0x0006);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAD32, 0x003F, 0x0006);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xAC08, BIT_12);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xAC08, BIT_8);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAC8A,
                BIT_15|BIT_14|BIT_13|BIT_12, BIT_14|BIT_13|BIT_12);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xAD18, BIT_10);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xAD1A, 0x3FF);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xAD1C, 0x3FF);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80EA);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0xC400);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80EB);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0x0700, 0x0300);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80F8);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x1C00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80F1);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x3000);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80FE);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0xA500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8102);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x5000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8105);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x3300);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8100);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x7000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8104);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0xF000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8106);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x6500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80DC);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0xED00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80DF);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA438, BIT_8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80E1);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA438, BIT_8);

        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBF06, 0x003F, 0x38);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x819F);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xD0B6);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xBC34, 0x5555);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBF0A, BIT_11|BIT_10|BIT_9, BIT_11|BIT_9);

        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA5C0, BIT_10);

        rtl8125_set_eth_phy_ocp_bit(tp, 0xA442, BIT_11);

        //enable aldps
        //GPHY OCP 0xA430 bit[2] = 0x1 (en_aldps)
        if (aspm && HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
                rtl8125_enable_phy_aldps(tp);
}

static void
rtl8125_hw_phy_config_8125a_2(struct rtl8125_private *tp)
{
        rtl8125_set_eth_phy_ocp_bit(tp, 0xAD4E, BIT_4);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAD16, 0x03FF, 0x03FF);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAD32, 0x003F, 0x0006);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xAC08, BIT_12);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xAC08, BIT_8);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xACC0, BIT_1|BIT_0, BIT_1);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAD40, BIT_7|BIT_6|BIT_5, BIT_6);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAD40, BIT_2|BIT_1|BIT_0, BIT_2);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xAC14, BIT_7);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xAC80, BIT_9|BIT_8);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAC5E, BIT_2|BIT_1|BIT_0, BIT_1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xAD4C, 0x00A8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xAC5C, 0x01FF);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAC8A, BIT_7|BIT_6|BIT_5|BIT_4, BIT_5|BIT_4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8157);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x0500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8159);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x0700);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80A2);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0153);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x809C);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0153);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x81B3);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0043);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00A7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00D6);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00EC);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00F6);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00FB);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00FD);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00FF);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00BB);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0058);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0029);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0013);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0009);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0004);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0002);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8257);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x020F);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80EA);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x7843);

        rtl8125_set_phy_mcu_patch_request(tp);

        rtl8125_clear_eth_phy_ocp_bit(tp, 0xB896, BIT_0);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xB892, 0xFF00);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC091);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x6E12);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC092);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x1214);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC094);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x1516);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC096);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x171B);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC098);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x1B1C);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC09A);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x1F1F);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC09C);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x2021);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC09E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x2224);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC0A0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x2424);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC0A2);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x2424);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC0A4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x2424);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC018);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x0AF2);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC01A);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x0D4A);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC01C);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x0F26);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC01E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x118D);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC020);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x14F3);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC022);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x175A);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC024);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x19C0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC026);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x1C26);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC089);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x6050);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC08A);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x5F6E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC08C);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x6E6E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC08E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x6E6E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC090);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x6E12);

        rtl8125_set_eth_phy_ocp_bit(tp, 0xB896, BIT_0);
        rtl8125_clear_phy_mcu_patch_request(tp);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xD068, BIT_13);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x81A2);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA438, BIT_8);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB54C, 0xFF00, 0xDB00);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA454, BIT_0);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA5D4, BIT_5);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xAD4E, BIT_4);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA86A, BIT_0);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA442, BIT_11);

        /* Test for PhyMidiSwap */
        if ((rtl8125_mac_ocp_read(tp, 0xD442) & BIT_5) &&
                    (rtl8125_mdio_direct_read_phy_ocp(tp, 0xD068) & BIT_1)) { //  CFG_METHOD_3 or CFG_METHOD_6
                u16 adccal_offset_p0, adccal_offset_p1, adccal_offset_p2, adccal_offset_p3;
                u16 rg_lpf_cap_xg_p0, rg_lpf_cap_xg_p1, rg_lpf_cap_xg_p2, rg_lpf_cap_xg_p3;
                u16 rg_lpf_cap_p0, rg_lpf_cap_p1, rg_lpf_cap_p2, rg_lpf_cap_p3;

                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xD068, 0x0007, 0x0001);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xD068, 0x0018, 0x0000);
                adccal_offset_p0 = rtl8125_mdio_direct_read_phy_ocp(tp, 0xD06A) & 0x07FF;
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xD068, 0x0018, 0x0008);
                adccal_offset_p1 = rtl8125_mdio_direct_read_phy_ocp(tp, 0xD06A) & 0x07FF;
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xD068, 0x0018, 0x0010);
                adccal_offset_p2 = rtl8125_mdio_direct_read_phy_ocp(tp, 0xD06A) & 0x07FF;
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xD068, 0x0018, 0x0018);
                adccal_offset_p3 = rtl8125_mdio_direct_read_phy_ocp(tp, 0xD06A) & 0x07FF;

                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xD068, 0x0018, 0x0000);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xD06A, 0x07FF, adccal_offset_p3);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xD068, 0x0018, 0x0008);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xD06A, 0x07FF, adccal_offset_p2);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xD068, 0x0018, 0x0010);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xD06A, 0x07FF, adccal_offset_p1);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xD068, 0x0018, 0x0018);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xD06A, 0x07FF, adccal_offset_p0);

                rg_lpf_cap_xg_p0 = rtl8125_mdio_direct_read_phy_ocp(tp, 0xBD5A) & 0x001F;
                rg_lpf_cap_xg_p1 = rtl8125_mdio_direct_read_phy_ocp(tp, 0xBD5A) & 0x1F00;
                rg_lpf_cap_xg_p2 = rtl8125_mdio_direct_read_phy_ocp(tp, 0xBD5C) & 0x001F;
                rg_lpf_cap_xg_p3 = rtl8125_mdio_direct_read_phy_ocp(tp, 0xBD5C) & 0x1F00;
                rg_lpf_cap_p0 = rtl8125_mdio_direct_read_phy_ocp(tp, 0xBC18) & 0x001F;
                rg_lpf_cap_p1 = rtl8125_mdio_direct_read_phy_ocp(tp, 0xBC18) & 0x1F00;
                rg_lpf_cap_p2 = rtl8125_mdio_direct_read_phy_ocp(tp, 0xBC1A) & 0x001F;
                rg_lpf_cap_p3 = rtl8125_mdio_direct_read_phy_ocp(tp, 0xBC1A) & 0x1F00;

                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBD5A, 0x001F, rg_lpf_cap_xg_p3 >> 8);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBD5A, 0x1F00, rg_lpf_cap_xg_p2 << 8);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBD5C, 0x001F, rg_lpf_cap_xg_p1 >> 8);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBD5C, 0x1F00, rg_lpf_cap_xg_p0 << 8);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBC18, 0x001F, rg_lpf_cap_p3 >> 8);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBC18, 0x1F00, rg_lpf_cap_p2 << 8);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBC1A, 0x001F, rg_lpf_cap_p1 >> 8);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBC1A, 0x1F00, rg_lpf_cap_p0 << 8);
        }

        rtl8125_set_eth_phy_ocp_bit(tp, 0xA424, BIT_3);

        if (aspm && HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
                rtl8125_enable_phy_aldps(tp);
}

static void
rtl8125_hw_phy_config_8125b_1(struct rtl8125_private *tp)
{
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA442, BIT_11);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xBC08, (BIT_3 | BIT_2));

        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8FFF);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x0400);
        }
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8560);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x19CC);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8562);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x19CC);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8564);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x19CC);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8566);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x147D);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8568);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x147D);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x856A);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x147D);
        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FFE);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0907);
        }
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xACDA, 0xFF00, 0xFF00);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xACDE, 0xF000, 0xF000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80D6);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x2801);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80F2);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x2801);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80F4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x6077);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB506, 0x01E7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xAC8C, 0x0FFC);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xAC46, 0xB7B4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xAC50, 0x0FBC);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xAC3C, 0x9240);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xAC4E, 0x0DB4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xACC6, 0x0707);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xACC8, 0xA0D3);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xAD08, 0x0007);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8013);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0700);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FB9);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x2801);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FBA);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0100);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FBC);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x1900);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FBE);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xE100);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FC0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0800);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FC2);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xE500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FC4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0F00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FC6);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xF100);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FC8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0400);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FCa);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xF300);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FCc);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xFD00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FCe);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xFF00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FD0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xFB00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FD2);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0100);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FD4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xF400);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FD6);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xFF00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FD8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xF600);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x813D);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x390E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x814F);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x790E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80B0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0F31);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xBF4C, BIT_1);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xBCCA, (BIT_9 | BIT_8));
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8141);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x320E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8153);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x720E);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA432, BIT_6);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8529);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x050E);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x816C);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xC4A0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8170);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xC4A0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8174);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x04A0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8178);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x04A0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x817C);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0719);
        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8FF4);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0400);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8FF1);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0404);
        }
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xBF4A, 0x001B);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8033);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x7C13);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8037);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x7C13);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x803B);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xFC32);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x803F);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x7C13);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8043);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x7C13);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8047);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x7C13);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8145);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x370E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8157);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x770E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8169);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0D0A);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x817B);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x1D0A);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8217);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x5000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x821A);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x5000);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80DA);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0403);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80DC);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x1000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80B3);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0384);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80B7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x2007);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80BA);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x6C00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80B5);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xF009);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80BD);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x9F00);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80C7);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xf083);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80DD);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x03f0);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80DF);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x1000);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80CB);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x2007);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80CE);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x6C00);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80C9);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8009);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80D1);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x8000);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80A3);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x200A);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80A5);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xF0AD);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x809F);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x6073);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80A1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x000B);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80A9);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0xC000);

        rtl8125_set_phy_mcu_patch_request(tp);

        rtl8125_clear_eth_phy_ocp_bit(tp, 0xB896, BIT_0);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xB892, 0xFF00);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC23E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC240);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x0103);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC242);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x0507);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC244);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x090B);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC246);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x0C0E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC248);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x1012);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB88E, 0xC24A);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB890, 0x1416);

        rtl8125_set_eth_phy_ocp_bit(tp, 0xB896, BIT_0);
        rtl8125_clear_phy_mcu_patch_request(tp);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA86A, BIT_0);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA6F0, BIT_0);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xBFA0, 0xD70D);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xBFA2, 0x4100);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xBFA4, 0xE868);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xBFA6, 0xDC59);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB54C, 0x3C18);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xBFA4, BIT_5);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x817D);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA438, BIT_12);

        if (aspm && HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
                rtl8125_enable_phy_aldps(tp);
}

static void
rtl8125_hw_phy_config_8125b_2(struct rtl8125_private *tp)
{
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA442, BIT_11);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAC46, 0x00F0, 0x0090);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAD30, 0x0003, 0x0001);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80F5);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x760E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8107);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x360E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8551);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E,
                BIT_15 | BIT_14 | BIT_13 | BIT_12 | BIT_11 | BIT_10 | BIT_9 | BIT_8, BIT_11);

        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xbf00, 0xE000, 0xA000);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xbf46, 0x0F00, 0x0300);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa436, 0x8044);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa438, 0x2417);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa436, 0x804A);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa438, 0x2417);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa436, 0x8050);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa438, 0x2417);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa436, 0x8056);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa438, 0x2417);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa436, 0x805C);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa438, 0x2417);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa436, 0x8062);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa438, 0x2417);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa436, 0x8068);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa438, 0x2417);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa436, 0x806E);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa438, 0x2417);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa436, 0x8074);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa438, 0x2417);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa436, 0x807A);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xa438, 0x2417);

        rtl8125_set_eth_phy_ocp_bit(tp, 0xA4CA, BIT_6);

        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBF84,
                BIT_15 | BIT_14 | BIT_13, BIT_15 | BIT_13);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8170);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438,
                BIT_13 | BIT_10 | BIT_9 | BIT_8, BIT_15 | BIT_14 | BIT_12 | BIT_11);

        rtl8125_set_eth_phy_ocp_bit(tp, 0xA424, BIT_3);

        /*
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xBFA0, 0xD70D);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xBFA2, 0x4100);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xBFA4, 0xE868);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xBFA6, 0xDC59);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB54C, 0x3C18);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xBFA4, BIT_5);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x817D);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA438, BIT_12);
        */

        if (aspm && HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
                rtl8125_enable_phy_aldps(tp);
}

static void
rtl8125_hw_phy_config_8125bp_1(struct rtl8125_private *tp)
{
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA442, BIT_11);

        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA80C,
                BIT_14, BIT_15 | BIT_11 | BIT_10);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8010);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA438, BIT_11);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8088);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x9000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x808F);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x9000);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8174);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, BIT_13, BIT_12 | BIT_11);

        if (aspm && HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
                rtl8125_enable_phy_aldps(tp);
}

static void
rtl8125_hw_phy_config_8125bp_2(struct rtl8125_private *tp )
{
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA442, BIT_11);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8010);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA438, BIT_11);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8088);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x9000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x808F);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x9000);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8174);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, BIT_13, BIT_12 | BIT_11);

        if (aspm && HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
                rtl8125_enable_phy_aldps(tp);
}

static void
rtl8125_hw_phy_config_8125d_1(struct rtl8125_private *tp)
{
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA442, BIT_11);
        rtl8125_set_phy_mcu_patch_request(tp);

        rtl8125_set_eth_phy_ocp_bit(tp, 0xBF96, BIT_15);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBF94, 0x0007, 0x0005);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBF8E, 0x3C00, 0x2800);

        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBCD8, 0xC000, 0x4000);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xBCD8, BIT_15 | BIT_14);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBCD8, 0xC000, 0x4000);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBC80, 0x001F, 0x0004);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xBC82, BIT_15 | BIT_14 | BIT_13);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xBC82, BIT_12 | BIT_11 | BIT_10);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBC80, 0x001F, 0x0005);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBC82, 0x00E0, 0x0040);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xBC82, BIT_4 | BIT_3 | BIT_2);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xBCD8, BIT_15 | BIT_14);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xBD70, BIT_8);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA466, BIT_1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x836a);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA438, 0xFF00);

        rtl8125_clear_phy_mcu_patch_request(tp);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x832C);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x0500);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB106, 0x0700, 0x0100);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB206, 0x0700, 0x0200);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB306, 0x0700, 0x0300);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80CB);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x0300);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xBCF4, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xBCF6, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xBC12, 0x0000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x844d);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x0200);
        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8feb);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x0100);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8fe9);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x0600);
        }
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAC7E, 0x01FC, 0x00B4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8105);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x7A00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8117);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x3A00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8103);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x7400);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8115);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x3400);

        rtl8125_clear_eth_phy_ocp_bit(tp, 0xAD40, BIT_5 | BIT_4);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAD66, 0x000F, 0x0007);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAD68, 0xF000, 0x8000);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAD68, 0x0F00, 0x0500);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAD68, 0x000F, 0x0002);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAD6A, 0xF000, 0x7000);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xAC50, 0x01E8);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x81FA);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x5400);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA864, 0x00F0, 0x00C0);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA42C, 0x00FF, 0x0002);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80E1);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x0F00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80DE);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xF000, 0x0700);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA846, BIT_7);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80BA);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8A04);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80BD);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0xCA00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80B7);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0xB300);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80CE);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8A04);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80D1);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0xCA00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80CB);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0xBB00);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80A6);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4909);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80A8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x05B8);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8200);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x5800);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8FF1);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x7078);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8FF3);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x5D78);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8FF5);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x7862);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8FF7);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x1400);

        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x814C);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8455);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x814E);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x84AF);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8163);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x0600);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x816A);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x0500);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8171);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x1f00);
        }

        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBC3A, 0x000F, 0x0006);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8064);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA438, BIT_10 | BIT_9 | BIT_8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8067);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA438, BIT_10 | BIT_9 | BIT_8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x806A);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA438, BIT_10 | BIT_9 | BIT_8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x806D);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA438, BIT_10 | BIT_9 | BIT_8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8070);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA438, BIT_10 | BIT_9 | BIT_8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8073);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA438, BIT_10 | BIT_9 | BIT_8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8076);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA438, BIT_10 | BIT_9 | BIT_8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8079);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA438, BIT_10 | BIT_9 | BIT_8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x807C);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA438, BIT_10 | BIT_9 | BIT_8);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x807F);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA438, BIT_10 | BIT_9 | BIT_8);

        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBFA0, 0xFF70, 0x5500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xBFA2, 0x9D00);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8165);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0x0700, 0x0200);

        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8019);
                rtl8125_set_eth_phy_ocp_bit(tp, 0xA438, BIT_8);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8FE3);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0005);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00ED);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0502);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0B00);
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA438, 0xD401);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x2900);
        }

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8018);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x1700);

        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                rtl8125_mdio_direct_write_phy_ocp(tp, 0xA436, 0x815B);
                rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xA438, 0xFF00, 0x1700);
        }

        rtl8125_set_eth_phy_ocp_bit(tp, 0xA430, BIT_12 | BIT_0);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA442, BIT_7);

        if (aspm && HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
                rtl8125_enable_phy_aldps(tp);
}

static void
rtl8125_hw_phy_config_8125d_2(struct rtl8125_private *tp)
{
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA442, BIT_11);
        rtl8125_set_phy_mcu_patch_request(tp);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBCD8, 0xC000, 0x4000);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xBCD8, BIT_15 | BIT_14);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBCD8, 0xC000, 0x4000);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBC80, 0x001F, 0x0004);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xBC82, BIT_15 | BIT_14 | BIT_13);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xBC82, BIT_12 | BIT_11 | BIT_10);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBC80, 0x001F, 0x0005);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBC82, 0x00E0, 0x0040);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xBC82, BIT_4 | BIT_3 | BIT_2);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xBCD8, BIT_15 | BIT_14);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xBCD8, 0xC000, 0x8000);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xBCD8, BIT_15 | BIT_14);

        rtl8125_clear_phy_mcu_patch_request(tp);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xAC7E, 0x01FC, 0x00B4);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8105);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x7A00);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8117);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x3A00);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8103);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x7400);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8115);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x3400);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FEB);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x0500);
        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FEA);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0x0700);

        rtl8125_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80D6);
        rtl8125_clear_and_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00, 0xEF00);

        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA5D4, BIT_5);
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA654, BIT_11);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA430, BIT_12 | BIT_0);
        rtl8125_set_eth_phy_ocp_bit(tp, 0xA442, BIT_7);

        if (aspm && HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
                rtl8125_enable_phy_aldps(tp);
}

static void
rtl8125_hw_phy_config(struct rtl8125_private *tp)
{
        unsigned long flags;
        if (rtl8125_flag_is_set(tp, ResumeNoSpeedChange))
                return;

        spin_lock_irqsave(&tp->phy_lock, flags);
        //tp->phy_reset_enable(tp);
        rtl8125_xmii_reset_enable(tp);

#ifndef ENABLE_USE_FIRMWARE_FILE
        if (!tp->rtl_fw) {
                printk(KERN_INFO "r8125 Writing Firmware for %s\n", rtl_chip_info[chipset(tp)].name);
                rtl8125_set_hw_phy_before_init_phy_mcu(tp);
                rtl8125_init_hw_phy_mcu(tp);
        }
#endif

        // Invoke method specific firmware
        rtl_chip_info[chipset(tp)].rtl8125_hw_phy_config(tp);

        //legacy force mode(Chap 22)
        rtl8125_clear_eth_phy_ocp_bit(tp, 0xA5B4, BIT_15);

        /*ocp phy power saving*/
        /*
        if (aspm) {
                if (rtl8125_flag_is_set(tp, IsMcfg236)) {
                rtl8125_enable_ocp_phy_power_saving(dev);
        }
        */

        rtl8125_mdio_set_page(tp, 0x0000);
        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                if (tp->eee.eee_enabled)
                        rtl8125_enable_eee(tp);
                else
                        rtl8125_disable_eee(tp);
        }
        spin_unlock_irqrestore(&tp->phy_lock, flags);
}

static void
rtl8125_up(struct net_device *dev)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        rtl8125_hw_init(tp);
        rtl8125_hw_reset(tp);
        rtl8125_powerup_pll(tp);
        rtl8125_hw_ephy_config(tp);
        rtl8125_hw_phy_config(tp);
        rtl8125_hw_config(tp, dev);
}

/*
static inline void rtl8125_delete_esd_timer(struct net_device *dev, struct timer_list *timer)
{
        del_timer_sync(timer);
}

static inline void rtl8125_request_esd_timer(struct net_device *dev)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        struct timer_list *timer = &tp->esd_timer;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
        setup_timer(timer, rtl8125_esd_timer, (unsigned long)dev);
#else
        timer_setup(timer, rtl8125_esd_timer, 0);
#endif
        mod_timer(timer, jiffies + RTL8125_ESD_TIMEOUT);
}
*/

/*
static inline void rtl8125_delete_link_timer(struct net_device *dev, struct timer_list *timer)
{
        del_timer_sync(timer);
}

static inline void rtl8125_request_link_timer(struct net_device *dev)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        struct timer_list *timer = &tp->link_timer;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
        setup_timer(timer, rtl8125_link_timer, (unsigned long)dev);
#else
        timer_setup(timer, rtl8125_link_timer, 0);
#endif
        mod_timer(timer, jiffies + RTL8125_LINK_TIMEOUT);
}
*/

#ifdef CONFIG_NET_POLL_CONTROLLER
/*
 * Polling 'interrupt' - used by things like netconsole to send skbs
 * without having to re-enable interrupts. It's not called while
 * the interrupt routine is executing.
 */
static void
rtl8125_netpoll(struct net_device *dev)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        irq_handler_t handler = (tp->irq_nvecs > 1) ? rtl8125_interrupt_msix : rtl8125_interrupt;

        for (u8 i = 0; i < tp->irq_nvecs; i++) {
                int irq = tp->irq[i];
                disable_irq(irq); //->vector);
                handler(irq, &tp->napi[i]); //->vector, napi);
                enable_irq(irq); //->vector);
        }
}
#endif //CONFIG_NET_POLL_CONTROLLER

static void
rtl8125_setup_interrupt_mask(struct rtl8125_private *tp)
{
        int i;
        switch (tp->HwCurrIsrVer) {
        case 5:
                tp->intr_mask = ISRIMR_V5_LINKCHG | ISRIMR_V5_TOK_Q0;
                if (tp->num_tx_rings > 1)
                        tp->intr_mask |= ISRIMR_V5_TOK_Q1;
                for (i = 0; i < tp->num_rx_rings; i++)
                        tp->intr_mask |= ISRIMR_V5_ROK_Q0 << i;
                break;
        case 4:
                tp->intr_mask = ISRIMR_V4_LINKCHG;
                for (i = 0; i < max(tp->num_tx_rings, tp->num_rx_rings); i++)
                        tp->intr_mask |= ISRIMR_V4_ROK_Q0 << i;
                break;
        case 3:
                tp->intr_mask = ISRIMR_V2_LINKCHG;
                for (i = 0; i < max(tp->num_tx_rings, tp->num_rx_rings); i++)
                        tp->intr_mask |= ISRIMR_V2_ROK_Q0 << i;
                break;
        case 2:
                tp->intr_mask = ISRIMR_V2_LINKCHG | ISRIMR_TOK_Q0;
                if (tp->num_tx_rings > 1)
                        tp->intr_mask |= ISRIMR_TOK_Q1;

                for (i = 0; i < tp->num_rx_rings; i++)
                        tp->intr_mask |= ISRIMR_V2_ROK_Q0 << i;
                break;
        default:
                tp->intr_mask = LinkChg | RxDescUnavail | TxOK | RxOK | RxErr | TxErr | SWInt;
                tp->timer_intr_mask = LinkChg | PCSTimeout;
        }
}

static void
rtl8125_init_software_variable(struct net_device *dev)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        struct ethtool_keee *eee = &tp->eee;
        int chipset = chipset(tp);

#ifdef ENABLE_REALWOW_SUPPORT
        rtl8125_get_realwow_hw_version(dev);
#endif //ENABLE_REALWOW_SUPPORT

        if (aspm) {
                tp->org_pci_offset_99 = rtl8125_csi_fun0_read_byte(tp, 0x99);
                tp->org_pci_offset_99 &= ~(BIT_5|BIT_6);
                tp->org_pci_offset_180 = rtl8125_csi_fun0_read_byte(tp,
                        rtl_chip_info[chipset].org_pci_offset_180);
        }

        //pci_read_config_byte(pdev, 0x80, &tp->org_pci_offset_80);
        //pci_read_config_byte(pdev, 0x81, &tp->org_pci_offset_81);

        if (timer_count)
                rtl8125_set_flag(tp, UseIntrTimer);

        //if (tp->mcfg != CFG_METHOD_8 && tp->mcfg != CFG_METHOD_9)
        if (!is_8125BP(tp))
                rtl8125_set_flag(tp, WAKEUP_MAGIC_PACKET_V3);

        tp->sw_ram_code_ver = rtl_chip_info[chipset].sw_ram_code_ver;
        rtl8125_check_hw_phy_mcu_code_ver(tp);

#ifdef ENABLE_PTP_SUPPORT
        if (is_8125B(tp))
                rtl8125_set_flag(tp, EnablePtp);

        if (enable_ptp_master_mode)
                rtl8125_set_flag(tp, PtpMasterMode);
        //else
        //        rtl8125_clear_flag(tp, PtpMasterMode);
        //tp->ptp_master_mode = enable_ptp_master_mode;
#endif

        //init interrupt
        //tp->HwSuppIsrVer = rtl_chip_info[chipset].HwSuppIsrVer;
        tp->HwCurrIsrVer = rtl_chip_info[chipset].HwSuppIsrVer;
        if (tp->HwCurrIsrVer > 1) {
                if ((tp->irq_nvecs <= 1) ||  // implies !RTL_MSIX
                    tp->irq_nvecs < rtl_chip_info[chipset].min_irq_nvecs)
                        tp->HwCurrIsrVer = 1;
        }
        tp->LinkChgShift = rtl_chip_info[chipset].linkChgShift;
/*
        switch (tp->HwCurrIsrVer) {
        case 1:
                tp->LinkChgShift = 5; // LinkChg
                break;
        case 2: // ... 3: does not exist
                tp->LinkChgShift = 21; // ISRIMR_V2_LINKCHG
                break;
        case 4:
                tp->LinkChgShift = 29; // ISRIMR_V4_LINKCHG
                break;
        case 5:
                tp->LinkChgShift = 18; // ISRIMR_V5_LINKCHG
                break;
        }
*/
        /* set number of tx rings */
        tp->num_tx_rings = 1;
#if defined(ENABLE_MULTIPLE_TX_QUEUE)
        tp->num_tx_rings = HW_SUPP_NUM_TX_QUEUES(tp);
#endif
        if (tp->HwCurrIsrVer < 2 || (tp->HwCurrIsrVer == 2 && tp->irq_nvecs < 19))
                tp->num_tx_rings = 1;

        /* set number of rx rings */
        tp->num_rx_rings = 1;
        rtl8125_clear_flag(tp, EnableRss);
#ifdef ENABLE_RSS_SUPPORT
        if (tp->HwCurrIsrVer > 1) {  // implies !R8125A
                u8 rss_queue_num = netif_get_num_default_rss_queues();
                tp->num_rx_rings = min(HW_SUPP_NUM_RX_QUEUES(tp), rss_queue_num);
                //(tp->HwSuppNumRxQueues > rss_queue_num)? rss_queue_num : tp->HwSuppNumRxQueues;
                if (tp->num_rx_rings >= 2)
                        rtl8125_set_flag(tp, EnableRss);
        }
#endif
        // Set Interrupt Mask
        rtl8125_setup_interrupt_mask(tp);
        //rtl8125_setup_mqs_reg(tp);
        //rtl8125_set_ring_size(tp, NUM_RX_DESC, NUM_TX_DESC);

        //tp->HwSuppIntMitiVer = rtl_chip_info[chipset].HwSuppIntMitiVer;
        timer_count_v2 = (timer_count / 0x100);
        /* timer unit is double */
        if (is_8125BPD(tp)) //(tp->mcfg >= CFG_METHOD_8)
                timer_count_v2 /= 2;

        tp->RxDescType = RX_DESC_RING_TYPE_1;
        tp->rx_config = rtl_chip_info[chipset].RxConfig;

        if (is_8125AB(tp)) { //(tp->mcfg <= CFG_METHOD_5) {
                //tp->HwSuppRxDescType = RX_DESC_RING_TYPE_3;
#if defined(ENABLE_RSS_SUPPORT) || defined(ENABLE_PTP_SUPPORT)
#if defined(ENABLE_RSS_SUPPORT) && defined(ENABLE_PTP_SUPPORT)
                if (rtl8125_flag_is_set(tp, EnableRss | EnablePtp)) {
#elif defined(ENABLE_RSS_SUPPORT)
                if (rtl8125_flag_is_set(tp, EnableRss)) {
#else
                if (rtl8125_flag_is_set(tp, EnablePtp)) {
#endif
                        tp->RxDescType = RX_DESC_RING_TYPE_3;
                        tp->rx_config |= EnableRxDescV3;
                }
#endif

        } else {
                //tp->HwSuppRxDescType = RX_DESC_RING_TYPE_4;
#if defined(ENABLE_RSS_SUPPORT)
                if (rtl8125_flag_is_set(tp, EnableRss)) {
                        tp->RxDescType = RX_DESC_RING_TYPE_4;
                        tp->rx_config &= ~EnableRxDescV3;
                }
#endif
        }

        //tp->NicCustLedValue = RTL_R16(tp, CustomLED);

        tp->wol_opts = rtl8125_get_hw_wol(tp);
        tp->wol_enabled = (tp->wol_opts) ? WOL_ENABLED : WOL_DISABLED;

        rtl8125_set_link_option(tp, autoneg_mode, speed_mode, duplex_mode,
                                rtl8125_fc_full);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0)
        /* MTU range: 60 - hw-specific max */
        dev->min_mtu = ETH_MIN_MTU;
        dev->max_mtu = MAX_JUMBO_FRAME_SIZE;
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0)

        eee->eee_enabled = eee_enable;
#if LINUX_VERSION_CODE < KERNEL_VERSION(6,9,0)
        eee->supported  = SUPPORTED_100baseT_Full | SUPPORTED_1000baseT_Full;
        eee->advertised = mmd_eee_adv_to_ethtool_adv_t(MDIO_EEE_1000T | MDIO_EEE_100TX);
        if (!is_8125A(tp) && HW_SUPP_PHY_LINK_SPEED_2500M(tp)) {
                eee->supported |= SUPPORTED_2500baseX_Full;
                eee->advertised |= SUPPORTED_2500baseX_Full;
        }
#else
        linkmode_set_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT, eee->supported);
        linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT, eee->supported);
        linkmode_set_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT, eee->advertised);
        linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT, eee->advertised);
        if (!is_8125A(tp) && HW_SUPP_PHY_LINK_SPEED_2500M(tp)) {
                linkmode_set_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT, eee->supported);
                linkmode_set_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT, eee->advertised);
        }
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(6,9,0) */

        eee->tx_lpi_enabled = eee_enable;
        eee->tx_lpi_timer = dev->mtu + ETH_HLEN + 0x20;

#ifdef ENABLE_RSS_SUPPORT
        if (rtl8125_flag_is_set(tp, EnableRss))
                rtl8125_init_rss(tp);
#endif
}

static void
rtl8125_release_board(struct pci_dev *pdev, struct net_device *dev)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        void __iomem *ioaddr = tp->mmio_addr;

        rtl8125_rar_set(tp, tp->org_mac_addr);
        tp->wol_enabled = WOL_DISABLED;

        rtl8125_phy_power_down(tp);

        iounmap(ioaddr);
        pci_release_regions(pdev);
        pci_clear_mwi(pdev);
        pci_disable_device(pdev);
        free_netdev(dev);
}

static int
rtl8125_get_mac_address(struct net_device *dev)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        int i;
        u8 mac_addr[MAC_ADDR_LEN];
        const unsigned char *dev_addr = dev->dev_addr;

        for (i = 0; i < MAC_ADDR_LEN; i++)
                mac_addr[i] = RTL_R8(tp, MAC0 + i);

        *(u32*)&mac_addr[0] = RTL_R32(tp, BACKUP_ADDR0_8125);
        *(u16*)&mac_addr[4] = RTL_R16(tp, BACKUP_ADDR1_8125);

        if (!is_valid_ether_addr(mac_addr)) {
                netif_err(tp, probe, dev, "Invalid ether addr %pM\n", mac_addr);
                eth_random_addr(mac_addr);
                dev->addr_assign_type = NET_ADDR_RANDOM;
                netif_info(tp, probe, dev, "Random ether addr %pM\n", mac_addr);
                rtl8125_set_flag(tp, RandomMac);
        }

        //rtl8125_hw_address_set(dev, mac_addr);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,17,0)
        eth_hw_addr_set(dev, mac_addr);
#else
        memcpy(dev_addr, mac_addr, MAC_ADDR_LEN);
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(5,17,0)
        rtl8125_rar_set(tp, mac_addr);

        /* keep the original MAC address */
        memcpy(tp->org_mac_addr, dev_addr, MAC_ADDR_LEN);
        memcpy(dev->perm_addr, dev_addr, MAC_ADDR_LEN);
        return 0;
}

/**
 * rtl8125_set_mac_address - Change the Ethernet Address of the NIC
 * @dev: network interface device structure
 * @p:   pointer to an address structure
 *
 * Return 0 on success, negative on failure
 **/
static int
rtl8125_set_mac_address(struct net_device *dev, void *p)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        struct sockaddr *addr = p;

        if (!is_valid_ether_addr(addr->sa_data))
                return -EADDRNOTAVAIL;

        //rtl8125_hw_address_set(dev_addr, addr->sa_data);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,17,0)
        eth_hw_addr_set(dev, addr->sa_data);
#else
        memcpy(dev_addr, addr->sa_data, MAC_ADDR_LEN);
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(5,17,0)

        rtl8125_rar_set(tp, dev->dev_addr);
        return 0;
}

/******************************************************************************
 * rtl8125_rar_set - Puts an ethernet address into a receive address register.
 *
 * tp - The private data structure for driver
 * addr - Address to put into receive address register
 *****************************************************************************/
void
rtl8125_rar_set(struct rtl8125_private *tp, const u8 *addr)
{
        uint32_t rar_low = ((uint32_t) addr[0] | ((uint32_t) addr[1] << 8) |
                ((uint32_t) addr[2] << 16) | ((uint32_t) addr[3] << 24));

        uint32_t rar_high = ((uint32_t) addr[4] | ((uint32_t) addr[5] << 8));

        rtl8125_unlock_config_regs(tp);
        RTL_W32(tp, MAC0, rar_low);
        RTL_W32(tp, MAC4, rar_high);

        rtl8125_lock_config_regs(tp);
}

#ifdef ETHTOOL_OPS_COMPAT
static int ethtool_get_settings(struct net_device *dev, void *useraddr)
{
        struct ethtool_cmd cmd = { ETHTOOL_GSET };
        int err;

        if (!ethtool_ops->get_settings)
                return -EOPNOTSUPP;

        err = ethtool_ops->get_settings(dev, &cmd);
        if (err < 0)
                return err;

        if (copy_to_user(useraddr, &cmd, sizeof(cmd)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_settings(struct net_device *dev, void *useraddr)
{
        struct ethtool_cmd cmd;

        if (!ethtool_ops->set_settings)
                return -EOPNOTSUPP;

        if (copy_from_user(&cmd, useraddr, sizeof(cmd)))
                return -EFAULT;

        return ethtool_ops->set_settings(dev, &cmd);
}

static int ethtool_get_drvinfo(struct net_device *dev, void *useraddr)
{
        struct ethtool_drvinfo info;
        struct ethtool_ops *ops = ethtool_ops;

        if (!ops->get_drvinfo)
                return -EOPNOTSUPP;

        memset(&info, 0, sizeof(info));
        info.cmd = ETHTOOL_GDRVINFO;
        ops->get_drvinfo(dev, &info);

        if (ops->self_test_count)
                info.testinfo_len = ops->self_test_count(dev);
        if (ops->get_stats_count)
                info.n_stats = ops->get_stats_count(dev);
        if (ops->get_regs_len)
                info.regdump_len = ops->get_regs_len(dev);
        if (ops->get_eeprom_len)
                info.eedump_len = ops->get_eeprom_len(dev);

        if (copy_to_user(useraddr, &info, sizeof(info)))
                return -EFAULT;
        return 0;
}

static int ethtool_get_regs(struct net_device *dev, char *useraddr)
{
        struct ethtool_regs regs;
        struct ethtool_ops *ops = ethtool_ops;
        void *regbuf;
        int reglen, ret;

        if (!ops->get_regs || !ops->get_regs_len)
                return -EOPNOTSUPP;

        if (copy_from_user(&regs, useraddr, sizeof(regs)))
                return -EFAULT;

        reglen = ops->get_regs_len(dev);
        if (regs.len > reglen)
                regs.len = reglen;

        regbuf = kmalloc(reglen, GFP_USER);
        if (!regbuf)
                return -ENOMEM;

        ops->get_regs(dev, &regs, regbuf);

        ret = -EFAULT;
        if (copy_to_user(useraddr, &regs, sizeof(regs)))
                goto out;
        useraddr += offsetof(struct ethtool_regs, data);
        if (copy_to_user(useraddr, regbuf, reglen))
                goto out;
        ret = 0;

out:
        kfree(regbuf);
        return ret;
}

static int ethtool_get_wol(struct net_device *dev, char *useraddr)
{
        struct ethtool_wolinfo wol = { ETHTOOL_GWOL };

        if (!ethtool_ops->get_wol)
                return -EOPNOTSUPP;

        ethtool_ops->get_wol(dev, &wol);

        if (copy_to_user(useraddr, &wol, sizeof(wol)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_wol(struct net_device *dev, char *useraddr)
{
        struct ethtool_wolinfo wol;

        if (!ethtool_ops->set_wol)
                return -EOPNOTSUPP;

        if (copy_from_user(&wol, useraddr, sizeof(wol)))
                return -EFAULT;

        return ethtool_ops->set_wol(dev, &wol);
}

static int ethtool_get_msglevel(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata = { ETHTOOL_GMSGLVL };

        if (!ethtool_ops->get_msglevel)
                return -EOPNOTSUPP;

        edata.data = ethtool_ops->get_msglevel(dev);

        if (copy_to_user(useraddr, &edata, sizeof(edata)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_msglevel(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata;

        if (!ethtool_ops->set_msglevel)
                return -EOPNOTSUPP;

        if (copy_from_user(&edata, useraddr, sizeof(edata)))
                return -EFAULT;

        ethtool_ops->set_msglevel(dev, edata.data);
        return 0;
}

static int ethtool_nway_reset(struct net_device *dev)
{
        if (!ethtool_ops->nway_reset)
                return -EOPNOTSUPP;

        return ethtool_ops->nway_reset(dev);
}

static int ethtool_get_link(struct net_device *dev, void *useraddr)
{
        struct ethtool_value edata = { ETHTOOL_GLINK };

        if (!ethtool_ops->get_link)
                return -EOPNOTSUPP;

        edata.data = ethtool_ops->get_link(dev);

        if (copy_to_user(useraddr, &edata, sizeof(edata)))
                return -EFAULT;
        return 0;
}

static int ethtool_get_eeprom(struct net_device *dev, void *useraddr)
{
        struct ethtool_eeprom eeprom;
        struct ethtool_ops *ops = ethtool_ops;
        u8 *data;
        int ret;

        if (!ops->get_eeprom || !ops->get_eeprom_len)
                return -EOPNOTSUPP;

        if (copy_from_user(&eeprom, useraddr, sizeof(eeprom)))
                return -EFAULT;

        /* Check for wrap and zero */
        if (eeprom.offset + eeprom.len <= eeprom.offset)
                return -EINVAL;

        /* Check for exceeding total eeprom len */
        if (eeprom.offset + eeprom.len > ops->get_eeprom_len(dev))
                return -EINVAL;

        data = kmalloc(eeprom.len, GFP_USER);
        if (!data)
                return -ENOMEM;

        ret = -EFAULT;
        if (copy_from_user(data, useraddr + sizeof(eeprom), eeprom.len))
                goto out;

        ret = ops->get_eeprom(dev, &eeprom, data);
        if (ret)
                goto out;

        ret = -EFAULT;
        if (copy_to_user(useraddr, &eeprom, sizeof(eeprom)))
                goto out;
        if (copy_to_user(useraddr + sizeof(eeprom), data, eeprom.len))
                goto out;
        ret = 0;

out:
        kfree(data);
        return ret;
}

static int ethtool_set_eeprom(struct net_device *dev, void *useraddr)
{
        struct ethtool_eeprom eeprom;
        struct ethtool_ops *ops = ethtool_ops;
        u8 *data;
        int ret;

        if (!ops->set_eeprom || !ops->get_eeprom_len)
                return -EOPNOTSUPP;

        if (copy_from_user(&eeprom, useraddr, sizeof(eeprom)))
                return -EFAULT;

        /* Check for wrap and zero */
        if (eeprom.offset + eeprom.len <= eeprom.offset)
                return -EINVAL;

        /* Check for exceeding total eeprom len */
        if (eeprom.offset + eeprom.len > ops->get_eeprom_len(dev))
                return -EINVAL;

        data = kmalloc(eeprom.len, GFP_USER);
        if (!data)
                return -ENOMEM;

        ret = -EFAULT;
        if (copy_from_user(data, useraddr + sizeof(eeprom), eeprom.len))
                goto out;

        ret = ops->set_eeprom(dev, &eeprom, data);
        if (ret)
                goto out;

        if (copy_to_user(useraddr + sizeof(eeprom), data, eeprom.len))
                ret = -EFAULT;

out:
        kfree(data);
        return ret;
}

static int ethtool_get_coalesce(struct net_device *dev, void *useraddr)
{
        struct ethtool_coalesce coalesce = { ETHTOOL_GCOALESCE };

        if (!ethtool_ops->get_coalesce)
                return -EOPNOTSUPP;

        ethtool_ops->get_coalesce(dev, &coalesce);

        if (copy_to_user(useraddr, &coalesce, sizeof(coalesce)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_coalesce(struct net_device *dev, void *useraddr)
{
        struct ethtool_coalesce coalesce;

        if (!ethtool_ops->get_coalesce)
                return -EOPNOTSUPP;

        if (copy_from_user(&coalesce, useraddr, sizeof(coalesce)))
                return -EFAULT;

        return ethtool_ops->set_coalesce(dev, &coalesce);
}

static int ethtool_get_ringparam(struct net_device *dev, void *useraddr)
{
        struct ethtool_ringparam ringparam = { ETHTOOL_GRINGPARAM };

        if (!ethtool_ops->get_ringparam)
                return -EOPNOTSUPP;

        ethtool_ops->get_ringparam(dev, &ringparam);

        if (copy_to_user(useraddr, &ringparam, sizeof(ringparam)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_ringparam(struct net_device *dev, void *useraddr)
{
        struct ethtool_ringparam ringparam;

        if (!ethtool_ops->get_ringparam)
                return -EOPNOTSUPP;

        if (copy_from_user(&ringparam, useraddr, sizeof(ringparam)))
                return -EFAULT;

        return ethtool_ops->set_ringparam(dev, &ringparam);
}

static int ethtool_get_pauseparam(struct net_device *dev, void *useraddr)
{
        struct ethtool_pauseparam pauseparam = { ETHTOOL_GPAUSEPARAM };

        if (!ethtool_ops->get_pauseparam)
                return -EOPNOTSUPP;

        ethtool_ops->get_pauseparam(dev, &pauseparam);

        if (copy_to_user(useraddr, &pauseparam, sizeof(pauseparam)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_pauseparam(struct net_device *dev, void *useraddr)
{
        struct ethtool_pauseparam pauseparam;

        if (!ethtool_ops->get_pauseparam)
                return -EOPNOTSUPP;

        if (copy_from_user(&pauseparam, useraddr, sizeof(pauseparam)))
                return -EFAULT;

        return ethtool_ops->set_pauseparam(dev, &pauseparam);
}

static int ethtool_get_rx_csum(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata = { ETHTOOL_GRXCSUM };

        if (!ethtool_ops->get_rx_csum)
                return -EOPNOTSUPP;

        edata.data = ethtool_ops->get_rx_csum(dev);

        if (copy_to_user(useraddr, &edata, sizeof(edata)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_rx_csum(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata;

        if (!ethtool_ops->set_rx_csum)
                return -EOPNOTSUPP;

        if (copy_from_user(&edata, useraddr, sizeof(edata)))
                return -EFAULT;

        ethtool_ops->set_rx_csum(dev, edata.data);
        return 0;
}

static int ethtool_get_tx_csum(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata = { ETHTOOL_GTXCSUM };

        if (!ethtool_ops->get_tx_csum)
                return -EOPNOTSUPP;

        edata.data = ethtool_ops->get_tx_csum(dev);

        if (copy_to_user(useraddr, &edata, sizeof(edata)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_tx_csum(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata;

        if (!ethtool_ops->set_tx_csum)
                return -EOPNOTSUPP;

        if (copy_from_user(&edata, useraddr, sizeof(edata)))
                return -EFAULT;

        return ethtool_ops->set_tx_csum(dev, edata.data);
}

static int ethtool_get_sg(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata = { ETHTOOL_GSG };

        if (!ethtool_ops->get_sg)
                return -EOPNOTSUPP;

        edata.data = ethtool_ops->get_sg(dev);

        if (copy_to_user(useraddr, &edata, sizeof(edata)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_sg(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata;

        if (!ethtool_ops->set_sg)
                return -EOPNOTSUPP;

        if (copy_from_user(&edata, useraddr, sizeof(edata)))
                return -EFAULT;

        return ethtool_ops->set_sg(dev, edata.data);
}

static int ethtool_get_tso(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata = { ETHTOOL_GTSO };

        if (!ethtool_ops->get_tso)
                return -EOPNOTSUPP;

        edata.data = ethtool_ops->get_tso(dev);

        if (copy_to_user(useraddr, &edata, sizeof(edata)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_tso(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata;

        if (!ethtool_ops->set_tso)
                return -EOPNOTSUPP;

        if (copy_from_user(&edata, useraddr, sizeof(edata)))
                return -EFAULT;

        return ethtool_ops->set_tso(dev, edata.data);
}

static int ethtool_self_test(struct net_device *dev, char *useraddr)
{
        struct ethtool_test test;
        struct ethtool_ops *ops = ethtool_ops;
        u64 *data;
        int ret;

        if (!ops->self_test || !ops->self_test_count)
                return -EOPNOTSUPP;

        if (copy_from_user(&test, useraddr, sizeof(test)))
                return -EFAULT;

        test.len = ops->self_test_count(dev);
        data = kmalloc(test.len * sizeof(u64), GFP_USER);
        if (!data)
                return -ENOMEM;

        ops->self_test(dev, &test, data);

        ret = -EFAULT;
        if (copy_to_user(useraddr, &test, sizeof(test)))
                goto out;
        useraddr += sizeof(test);
        if (copy_to_user(useraddr, data, test.len * sizeof(u64)))
                goto out;
        ret = 0;

out:
        kfree(data);
        return ret;
}

static int ethtool_get_strings(struct net_device *dev, void *useraddr)
{
        struct ethtool_gstrings gstrings;
        struct ethtool_ops *ops = ethtool_ops;
        u8 *data;
        int ret;

        if (!ops->get_strings)
                return -EOPNOTSUPP;

        if (copy_from_user(&gstrings, useraddr, sizeof(gstrings)))
                return -EFAULT;

        switch (gstrings.string_set) {
        case ETH_SS_TEST:
                if (!ops->self_test_count)
                        return -EOPNOTSUPP;
                gstrings.len = ops->self_test_count(dev);
                break;
        case ETH_SS_STATS:
                if (!ops->get_stats_count)
                        return -EOPNOTSUPP;
                gstrings.len = ops->get_stats_count(dev);
                break;
        default:
                return -EINVAL;
        }

        data = kmalloc(gstrings.len * ETH_GSTRING_LEN, GFP_USER);
        if (!data)
                return -ENOMEM;

        ops->get_strings(dev, gstrings.string_set, data);

        ret = -EFAULT;
        if (copy_to_user(useraddr, &gstrings, sizeof(gstrings)))
                goto out;
        useraddr += sizeof(gstrings);
        if (copy_to_user(useraddr, data, gstrings.len * ETH_GSTRING_LEN))
                goto out;
        ret = 0;

out:
        kfree(data);
        return ret;
}

static int ethtool_phys_id(struct net_device *dev, void *useraddr)
{
        struct ethtool_value id;

        if (!ethtool_ops->phys_id)
                return -EOPNOTSUPP;

        if (copy_from_user(&id, useraddr, sizeof(id)))
                return -EFAULT;

        return ethtool_ops->phys_id(dev, id.data);
}

static int ethtool_get_stats(struct net_device *dev, void *useraddr)
{
        struct ethtool_stats stats;
        struct ethtool_ops *ops = ethtool_ops;
        u64 *data;
        int ret;

        if (!ops->get_ethtool_stats || !ops->get_stats_count)
                return -EOPNOTSUPP;

        if (copy_from_user(&stats, useraddr, sizeof(stats)))
                return -EFAULT;

        stats.n_stats = ops->get_stats_count(dev);
        data = kmalloc(stats.n_stats * sizeof(u64), GFP_USER);
        if (!data)
                return -ENOMEM;

        ops->get_ethtool_stats(dev, &stats, data);

        ret = -EFAULT;
        if (copy_to_user(useraddr, &stats, sizeof(stats)))
                goto out;
        useraddr += sizeof(stats);
        if (copy_to_user(useraddr, data, stats.n_stats * sizeof(u64)))
                goto out;
        ret = 0;

out:
        kfree(data);
        return ret;
}

static int ethtool_ioctl(struct ifreq *ifr)
{
        struct net_device *dev = __dev_get_by_name(ifr->ifr_name);
        void *useraddr = (void *) ifr->ifr_data;
        u32 ethcmd;

        /*
         * XXX: This can be pushed down into the ethtool_* handlers that
         * need it.  Keep existing behaviour for the moment.
         */
        if (!capable(CAP_NET_ADMIN))
                return -EPERM;

        if (!dev || !netif_device_present(dev))
                return -ENODEV;

        if (copy_from_user(&ethcmd, useraddr, sizeof (ethcmd)))
                return -EFAULT;

        switch (ethcmd) {
        case ETHTOOL_GSET:
                return ethtool_get_settings(dev, useraddr);
        case ETHTOOL_SSET:
                return ethtool_set_settings(dev, useraddr);
        case ETHTOOL_GDRVINFO:
                return ethtool_get_drvinfo(dev, useraddr);
        case ETHTOOL_GREGS:
                return ethtool_get_regs(dev, useraddr);
        case ETHTOOL_GWOL:
                return ethtool_get_wol(dev, useraddr);
        case ETHTOOL_SWOL:
                return ethtool_set_wol(dev, useraddr);
        case ETHTOOL_GMSGLVL:
                return ethtool_get_msglevel(dev, useraddr);
        case ETHTOOL_SMSGLVL:
                return ethtool_set_msglevel(dev, useraddr);
        case ETHTOOL_NWAY_RST:
                return ethtool_nway_reset(dev);
        case ETHTOOL_GLINK:
                return ethtool_get_link(dev, useraddr);
        case ETHTOOL_GEEPROM:
                return ethtool_get_eeprom(dev, useraddr);
        case ETHTOOL_SEEPROM:
                return ethtool_set_eeprom(dev, useraddr);
        case ETHTOOL_GCOALESCE:
                return ethtool_get_coalesce(dev, useraddr);
        case ETHTOOL_SCOALESCE:
                return ethtool_set_coalesce(dev, useraddr);
        case ETHTOOL_GRINGPARAM:
                return ethtool_get_ringparam(dev, useraddr);
        case ETHTOOL_SRINGPARAM:
                return ethtool_set_ringparam(dev, useraddr);
        case ETHTOOL_GPAUSEPARAM:
                return ethtool_get_pauseparam(dev, useraddr);
        case ETHTOOL_SPAUSEPARAM:
                return ethtool_set_pauseparam(dev, useraddr);
        case ETHTOOL_GRXCSUM:
                return ethtool_get_rx_csum(dev, useraddr);
        case ETHTOOL_SRXCSUM:
                return ethtool_set_rx_csum(dev, useraddr);
        case ETHTOOL_GTXCSUM:
                return ethtool_get_tx_csum(dev, useraddr);
        case ETHTOOL_STXCSUM:
                return ethtool_set_tx_csum(dev, useraddr);
        case ETHTOOL_GSG:
                return ethtool_get_sg(dev, useraddr);
        case ETHTOOL_SSG:
                return ethtool_set_sg(dev, useraddr);
        case ETHTOOL_GTSO:
                return ethtool_get_tso(dev, useraddr);
        case ETHTOOL_STSO:
                return ethtool_set_tso(dev, useraddr);
        case ETHTOOL_TEST:
                return ethtool_self_test(dev, useraddr);
        case ETHTOOL_GSTRINGS:
                return ethtool_get_strings(dev, useraddr);
        case ETHTOOL_PHYS_ID:
                return ethtool_phys_id(dev, useraddr);
        case ETHTOOL_GSTATS:
                return ethtool_get_stats(dev, useraddr);
        default:
                printk(KERN_ERR "r8125: ethtool invalid cmd %x\n", ethcmd);
                return -EOPNOTSUPP;
        }

        return -EOPNOTSUPP;
}
#endif //ETHTOOL_OPS_COMPAT

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,15,0)
static int rtl8125_siocdevprivate(struct net_device *dev, struct ifreq *ifr,
                                  void __user *data, int cmd)
{
        switch (cmd) {
#ifdef ENABLE_REALWOW_SUPPORT
        case SIOCDEVPRIVATE_RTLREALWOW:
                if (!netif_running(dev))
                        return -ENODEV;
                return rtl8125_realwow_ioctl(dev, ifr);
#endif

#ifdef ENABLE_RTL_TOOL
        case SIOCRTLTOOL:
                struct rtl8125_private *tp = netdev_priv(dev);
                if (!capable(CAP_NET_ADMIN))
                        return -EPERM;
                return rtl8125_tool_ioctl(tp, ifr);
#endif

        default:
                return -EOPNOTSUPP;
        }
        return 0;
}
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(5,15,0)

static int
rtl8125_do_ioctl(struct net_device * const dev, struct ifreq * const ifr, const int cmd)
{
        struct rtl8125_private * const tp = netdev_priv(dev);
        struct mii_ioctl_data *const data = if_mii(ifr);

        switch (cmd) {
        case SIOCGMIIPHY:
                data->phy_id = 32; /* Internal PHY */
                break;

        case SIOCGMIIREG:
                rtl8125_mdio_set_page(tp, 0x0000);
                data->val_out = rtl8125_mdio_read(tp, data->reg_num);
                break;

        case SIOCSMIIREG:
                if (!capable(CAP_NET_ADMIN))
                        return -EPERM;
                rtl8125_mdio_set_page(tp, 0x0000);
                rtl8125_mdio_write(tp, data->reg_num, data->val_in);
                break;

#ifdef ETHTOOL_OPS_COMPAT
        case SIOCETHTOOL:
                return ethtool_ioctl(ifr);
#endif

#ifdef ENABLE_PTP_SUPPORT
        case SIOCSHWTSTAMP:
                printk(KERN_INFO "r8125: SIOCSHWTSTAMP\n");
                return (rtl8125_flag_is_set(tp, EnablePtp)) ? rtl8125_set_tstamp(tp, ifr): -EOPNOTSUPP;
        case SIOCGHWTSTAMP:
                printk(KERN_INFO "r8125: SIOCGHWTSTAMP\n");
                return (rtl8125_flag_is_set(tp, EnablePtp)) ? rtl8125_get_tstamp(tp, ifr): -EOPNOTSUPP;
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0)

#ifdef ENABLE_REALWOW_SUPPORT
        case SIOCDEVPRIVATE_RTLREALWOW:
                if (!netif_running(dev))
                        return -ENODEV;
                if (!capable(CAP_NET_ADMIN))
                        return -EPERM;

                return rtl8125_realwow_ioctl(dev, ifr);
#endif

        case SIOCRTLTOOL:
                if (!capable(CAP_NET_ADMIN))
                        return -EPERM;
                return rtl8125_tool_ioctl(tp, ifr);

#endif //LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0)

        default:
                printk(KERN_ERR "r8125: ioctl invalid cmd %x\n", cmd);
                return EOPNOTSUPP;
        }
        return 0;
}

static void
rtl8125_phy_power_up(struct rtl8125_private * const tp)
{
        unsigned long flags;
        if (rtl8125_is_in_phy_disable_mode(tp))
                return;

        spin_lock_irqsave(&tp->phy_lock, flags);
        rtl8125_mdio_set_page(tp, 0x0000);
        rtl8125_mdio_write(tp, MII_BMCR, BMCR_ANENABLE);

        //wait ups resume (phy state 3)
        rtl8125_wait_phy_ups_resume(tp, 3);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
}

static void
rtl8125_phy_power_down(struct rtl8125_private *const tp)
{
        unsigned long flags;
        spin_lock_irqsave(&tp->phy_lock, flags);
        rtl8125_mdio_set_page(tp, 0x0000);
        rtl8125_mdio_write(tp, MII_BMCR, BMCR_ANENABLE | BMCR_PDOWN);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
}

static int __devinit
rtl8125_init_board(struct pci_dev *pdev, struct net_device **dev_out, void __iomem **ioaddr_out)
{
        void __iomem *ioaddr;
        struct net_device *dev;
        struct device* device = &pdev->dev;
        struct rtl8125_private *tp;
        int rc = -ENOMEM, i, pm_cap;

        assert(ioaddr_out != NULL);

        /* dev zeroed in alloc_etherdev */
        dev = alloc_etherdev_mq(sizeof (*tp), R8125_MAX_QUEUES);
        if (dev == NULL) {
                if (netif_msg_drv(&debug))
                        dev_err(&pdev->dev, "unable to alloc new ethernet\n");
                goto err_out;
        }

        SET_MODULE_OWNER(dev);
        SET_NETDEV_DEV(dev, &pdev->dev);
        tp = netdev_priv(dev);
        tp->dev = dev;
        tp->pci_dev = pdev;
        tp->device = device;
        tp->flags = 0;
        // Only enable the first 8 bits for netif messages
        tp->msg_enable = (u8)(netif_msg_init(debug.msg_enable, R8125_MSG_DEFAULT));

        if (!aspm)
                pci_disable_link_state(pdev, PCIE_LINK_STATE_L0S | PCIE_LINK_STATE_L1 |
                                       PCIE_LINK_STATE_CLKPM);

        /* enable device (incl. PCI PM wakeup and hotplug setup) */
        rc = pci_enable_device(pdev);
        if (rc < 0) {
                if (netif_msg_probe(tp))
                        dev_err(device, "enable failure\n");
                goto err_out_free_dev;
        }

        if (pci_set_mwi(pdev) < 0) {
                if (netif_msg_drv(&debug))
                        dev_info(device, "Mem-Wr-Inval unavailable.\n");
        }

        /* save power state before pci_enable_device overwrites it */
        pm_cap = pci_find_capability(pdev, PCI_CAP_ID_PM);
        if (pm_cap) {
                u16 pwr_command;

                pci_read_config_word(pdev, pm_cap + PCI_PM_CTRL, &pwr_command);
        } else {
                if (netif_msg_probe(tp)) {
                        dev_err(device, "PowerManagement capability not found.\n");
                }
        }

        /* make sure PCI base addr 1 is MMIO */
        if (!(pci_resource_flags(pdev, 2) & IORESOURCE_MEM)) {
                if (netif_msg_probe(tp))
                        dev_err(device, "region #1 not an MMIO resource, aborting\n");
                rc = -ENODEV;
                goto err_out_mwi;
        }
        /* check for weird/broken PCI region reporting */
        if (pci_resource_len(pdev, 2) < R8125_REGS_SIZE) {
                if (netif_msg_probe(tp))
                        dev_err(device, "Invalid PCI region size(s), aborting\n");
                rc = -ENODEV;
                goto err_out_mwi;
        }

        rc = pci_request_regions(pdev, MODULENAME);
        if (rc < 0) {
                if (netif_msg_probe(tp))
                        dev_err(device, "could not request regions.\n");
                goto err_out_mwi;
        }

        if ((sizeof(dma_addr_t) > 4) && use_dac &&
            !dma_set_mask(device, DMA_BIT_MASK(64)) &&
            !dma_set_coherent_mask(device, DMA_BIT_MASK(64))) {
                dev->features |= NETIF_F_HIGHDMA;
        } else {
                rc = dma_set_mask(device, DMA_BIT_MASK(32));
                if (rc < 0) {
                        if (netif_msg_probe(tp))
                                dev_err(device, "DMA configuration failed.\n");
                        goto err_out_free_res;
                }
        }

        /* ioremap MMIO region */
        ioaddr = ioremap(pci_resource_start(pdev, 2), pci_resource_len(pdev, 2));
        if (ioaddr == NULL) {
                if (netif_msg_probe(tp))
                        dev_err(device, "cannot remap MMIO, aborting\n");
                rc = -EIO;
                goto err_out_free_res;
        }

        tp->mmio_addr = ioaddr;

        /* Identify chip attached to board */
        rtl8125_get_mac_version(tp);

        for (i = ARRAY_SIZE(rtl_chip_info) - 1; i >= 0; i--)
                if (tp->mcfg == rtl_chip_info[i].mcfg)
                        break;

        /* By rejecting unknown chips, we can eliminate countless tests for valid tp->mcfg */
        if (i < 0 || tp->mcfg == CFG_METHOD_UNKNOWN) {
                /* Unknown chip */
                if (netif_msg_probe(tp))
                        dev_printk(KERN_DEBUG, device, ": Unknown chip version, exiting driver\n");
                goto err_out_free_res; /* Do not bind driver, release resources */
        }

        rtl8125_print_mac_version(tp);
        *ioaddr_out = ioaddr;
        *dev_out = dev;
out:
        return rc;

err_out_free_res:
        pci_release_regions(pdev);
err_out_mwi:
        pci_clear_mwi(pdev);
        pci_disable_device(pdev);
err_out_free_dev:
        free_netdev(dev);
err_out:
        *ioaddr_out = NULL;
        *dev_out = NULL;
        goto out;
}

#ifdef ENABLE_ESD
static inline void
rtl8125_esd_checker(struct rtl8125_private *tp, struct net_device *dev)
{
        struct pci_dev *pdev = tp->pci_dev;
        u8 cmd, ilr;
        u16 io_base_l, mem_base_l, mem_base_h;
        u16 resv_0x1c_h, resv_0x1c_l, resv_0x20_l, resv_0x20_h;
        u16 resv_0x24_l, resv_0x24_h, resv_0x2c_h, resv_0x2c_l;
        u32 pci_sn_l, pci_sn_h, hwPcieSNOffset;

#ifdef ENABLE_RTL_TOOL
        if (unlikely(tp->rtk_enable_diag))
                goto exit;
#endif

        tp->esd_flag = 0;

        pci_read_config_byte(pdev, PCI_COMMAND, &cmd);
        if (cmd != tp->pci_cfg_space.cmd) {
                printk(KERN_ERR "%s: cmd = 0x%02x, should be 0x%02x \n.", dev->name, cmd, tp->pci_cfg_space.cmd);
                pci_write_config_byte(pdev, PCI_COMMAND, tp->pci_cfg_space.cmd);
                tp->esd_flag |= BIT_0;

                pci_read_config_byte(pdev, PCI_COMMAND, &cmd);
                if (cmd == 0xff) {
                        printk(KERN_ERR "%s: pci link is down \n.", dev->name);
                        goto exit;
                }
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_0, &io_base_l);
        if (io_base_l != tp->pci_cfg_space.io_base_l) {
                printk(KERN_ERR "%s: io_base_l = 0x%04x, should be 0x%04x \n.", dev->name, io_base_l, tp->pci_cfg_space.io_base_l);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_0, tp->pci_cfg_space.io_base_l);
                tp->esd_flag |= BIT_1;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_2, &mem_base_l);
        if (mem_base_l != tp->pci_cfg_space.mem_base_l) {
                printk(KERN_ERR "%s: mem_base_l = 0x%04x, should be 0x%04x \n.", dev->name, mem_base_l, tp->pci_cfg_space.mem_base_l);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_2, tp->pci_cfg_space.mem_base_l);
                tp->esd_flag |= BIT_2;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_2 + 2, &mem_base_h);
        if (mem_base_h!= tp->pci_cfg_space.mem_base_h) {
                printk(KERN_ERR "%s: mem_base_h = 0x%04x, should be 0x%04x \n.", dev->name, mem_base_h, tp->pci_cfg_space.mem_base_h);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_2 + 2, tp->pci_cfg_space.mem_base_h);
                tp->esd_flag |= BIT_3;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_3, &resv_0x1c_l);
        if (resv_0x1c_l != tp->pci_cfg_space.resv_0x1c_l) {
                printk(KERN_ERR "%s: resv_0x1c_l = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x1c_l, tp->pci_cfg_space.resv_0x1c_l);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_3, tp->pci_cfg_space.resv_0x1c_l);
                tp->esd_flag |= BIT_4;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_3 + 2, &resv_0x1c_h);
        if (resv_0x1c_h != tp->pci_cfg_space.resv_0x1c_h) {
                printk(KERN_ERR "%s: resv_0x1c_h = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x1c_h, tp->pci_cfg_space.resv_0x1c_h);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_3 + 2, tp->pci_cfg_space.resv_0x1c_h);
                tp->esd_flag |= BIT_5;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_4, &resv_0x20_l);
        if (resv_0x20_l != tp->pci_cfg_space.resv_0x20_l) {
                printk(KERN_ERR "%s: resv_0x20_l = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x20_l, tp->pci_cfg_space.resv_0x20_l);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_4, tp->pci_cfg_space.resv_0x20_l);
                tp->esd_flag |= BIT_6;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_4 + 2, &resv_0x20_h);
        if (resv_0x20_h != tp->pci_cfg_space.resv_0x20_h) {
                printk(KERN_ERR "%s: resv_0x20_h = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x20_h, tp->pci_cfg_space.resv_0x20_h);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_4 + 2, tp->pci_cfg_space.resv_0x20_h);
                tp->esd_flag |= BIT_7;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_5, &resv_0x24_l);
        if (resv_0x24_l != tp->pci_cfg_space.resv_0x24_l) {
                printk(KERN_ERR "%s: resv_0x24_l = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x24_l, tp->pci_cfg_space.resv_0x24_l);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_5, tp->pci_cfg_space.resv_0x24_l);
                tp->esd_flag |= BIT_8;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_5 + 2, &resv_0x24_h);
        if (resv_0x24_h != tp->pci_cfg_space.resv_0x24_h) {
                printk(KERN_ERR "%s: resv_0x24_h = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x24_h, tp->pci_cfg_space.resv_0x24_h);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_5 + 2, tp->pci_cfg_space.resv_0x24_h);
                tp->esd_flag |= BIT_9;
        }

        pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &ilr);
        if (ilr != tp->pci_cfg_space.ilr) {
                printk(KERN_ERR "%s: ilr = 0x%02x, should be 0x%02x \n.", dev->name, ilr, tp->pci_cfg_space.ilr);
                pci_write_config_byte(pdev, PCI_INTERRUPT_LINE, tp->pci_cfg_space.ilr);
                tp->esd_flag |= BIT_10;
        }

        pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, &resv_0x2c_l);
        if (resv_0x2c_l != tp->pci_cfg_space.resv_0x2c_l) {
                printk(KERN_ERR "%s: resv_0x2c_l = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x2c_l, tp->pci_cfg_space.resv_0x2c_l);
                pci_write_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, tp->pci_cfg_space.resv_0x2c_l);
                tp->esd_flag |= BIT_11;
        }

        pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID + 2, &resv_0x2c_h);
        if (resv_0x2c_h != tp->pci_cfg_space.resv_0x2c_h) {
                printk(KERN_ERR "%s: resv_0x2c_h = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x2c_h, tp->pci_cfg_space.resv_0x2c_h);
                pci_write_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID + 2, tp->pci_cfg_space.resv_0x2c_h);
                tp->esd_flag |= BIT_12;
        }

        // Always true: (tp->HwPcieSNOffset > 0) {
        //hwPcieSNOffset = (tp->mcfg <= CFG_METHOD_7) ? 0x16C: 0x168;
        hwPcieSNOffset = is_8125AB(tp) ? 0x16C: 0x168;

        pci_sn_l = rtl8125_csi_read(tp, hwPcieSNOffset);
        if (pci_sn_l != tp->pci_cfg_space.pci_sn_l) {
                printk(KERN_ERR "%s: pci_sn_l = 0x%08x, should be 0x%08x \n.", dev->name, pci_sn_l, tp->pci_cfg_space.pci_sn_l);
                rtl8125_csi_write(tp, hwPcieSNOffset, tp->pci_cfg_space.pci_sn_l);
                tp->esd_flag |= BIT_13;
        }

        pci_sn_h = rtl8125_csi_read(tp, hwPcieSNOffset + 4);
        if (pci_sn_h != tp->pci_cfg_space.pci_sn_h) {
                printk(KERN_ERR "%s: pci_sn_h = 0x%08x, should be 0x%08x \n.", dev->name, pci_sn_h, tp->pci_cfg_space.pci_sn_h);
                rtl8125_csi_write(tp, hwPcieSNOffset+ 4, tp->pci_cfg_space.pci_sn_h);
                tp->esd_flag |= BIT_14;
        }

        if (tp->esd_flag != 0) {
                printk(KERN_ERR "%s: esd_flag = 0x%04x\n.\n", dev->name, tp->esd_flag);
                netif_carrier_off(dev);
                netif_tx_disable(dev);
                rtl8125_hw_reset(tp);
                rtl8125_tx_clear_rings(tp);
                rtl8125_rx_clear_rings(tp);
                rtl8125_init_rings(tp);
                rtl8125_up(dev);
                rtl8125_enable_hw_linkchg_interrupt(tp);
                rtl8125_set_speed(tp, rtl8125_flag_to_bool(tp, AutoNegMode), tp->speed, tp->duplex, tp->advertising);
                tp->esd_flag = 0;
        }
exit:
        return;
}
#endif

/* Cfg9346_Unlock assumed. */
static int rtl8125_msi_allocate_interrupts(struct rtl8125_private *tp)
{
        struct pci_dev *pdev = tp->pci_dev;
        u32 msi = 0; //, hw_supp_irq_nvecs;
        int nvecs, chipset = chipset(tp);

#if defined(RTL_USE_NEW_INTR_API)
        if ((nvecs = pci_alloc_irq_vectors(pdev, rtl_chip_info[chipset].min_irq_nvecs,
                rtl_chip_info[chipset].max_irq_nvecs, PCI_IRQ_MSIX)) > 0)
                msi = RTL_MSIX;
        else if ((nvecs = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_ALL_TYPES)) > 0 &&
                        pci_dev_msi_enabled(pdev))
                msi = RTL_MSI;
#else
        struct msix_entry msix_ent[R8125_MAX_MSIX_VEC];

        for (int i = 0; i < R8125_MAX_MSIX_VEC; i++) {
                msix_ent[i].entry = i;
                msix_ent[i].vector = 0;
        }
        nvecs = pci_enable_msix_range(tp->pci_dev, msix_ent,
                rtl_chip_info[chipset].min_irq_nvecs, rtl_chip_info[chipset].max_irq_nvecs);
        for (int i = 0; i < nvecs; i++)
                tp->irq = msix_ent[i].vector;
        msi = (nvecs > 0) ? RTL_MSIX : RTL_MSI;
#endif
        if (!msi)
                dev_info(&pdev->dev, "no MSI/MSI-X. Back to INTx.\n");

        tp->irq_nvecs = nvecs; // nvec > 0 implies MSI(X)
        rtl8125_set_flag(tp, msi);
        return nvecs;
}

static void rtl8125_disable_msi(struct pci_dev *pdev, struct rtl8125_private *tp)
{
#if defined(RTL_USE_NEW_INTR_API)
        if (likely(tp->irq_nvecs > 0)) // implies (RTL_MSI | RTL_MSIX)
                pci_free_irq_vectors(pdev);
#else
        if (tp->irq_nvecs > 1) // RTL_MSIX
                pci_disable_msix(pdev);
        else if (rtl8125_flag_is_set(tp, RTL_MSI))
                pci_disable_msi(pdev);
#endif
        rtl8125_clear_flag(tp, RTL_MSI | RTL_MSIX);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,11,0)
static void
rtl8125_get_stats64(struct net_device *const dev, struct rtnl_link_stats64 * const stats)
{
        struct rtl8125_private * const tp = netdev_priv(dev);
        const struct rtl8125_counters * const counters = tp->tally_vaddr;
        const dma_addr_t paddr = tp->tally_paddr;

        if (unlikely(!counters))
                return;
        pm_runtime_get_noresume(tp->device);
        netdev_stats_to_stats64(stats, &dev->stats);
        dev_fetch_sw_netstats(stats, dev->tstats);

        /*
         * Fetch additional counter values missing in stats collected by driver
         * from tally counters.
         */
        rtl8125_dump_tally_counter(tp, paddr);

        stats->tx_errors = le64_to_cpu(counters->tx_errors);
        stats->collisions = le32_to_cpu(counters->tx_multi_collision);
        stats->tx_aborted_errors = le16_to_cpu(counters->tx_aborted);
        stats->rx_missed_errors = le16_to_cpu(counters->rx_missed);
        pm_runtime_put_noidle(tp->device);
}
#else
/**
 *  rtl8125_get_stats - Get rtl8125 read/write statistics
 *  @dev: The Ethernet Device to get statistics for
 *
 *  Get TX/RX statistics for rtl8125
 */
static struct
net_device_stats *rtl8125_get_stats(struct net_device *dev)
{
        return &dev->stats;
}
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(5,11,0)

static const struct net_device_ops rtl8125_netdev_ops = {
        .ndo_open               = rtl8125_open,
        .ndo_stop               = rtl8125_close,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,11,0)
        .ndo_get_stats64        = rtl8125_get_stats64,
#else
        .ndo_get_stats          = rtl8125_get_stats,
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(5,11,0)
        .ndo_start_xmit         = rtl8125_start_xmit,
        .ndo_tx_timeout         = rtl8125_tx_timeout,
        .ndo_change_mtu         = rtl8125_change_mtu,
        .ndo_set_mac_address    = rtl8125_set_mac_address,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0)
        .ndo_do_ioctl           = rtl8125_do_ioctl,
#else
        .ndo_siocdevprivate     = rtl8125_siocdevprivate,
        .ndo_eth_ioctl          = rtl8125_do_ioctl,
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0)
        .ndo_set_rx_mode        = rtl8125_set_rx_mode,
        .ndo_fix_features       = rtl8125_fix_features,
        .ndo_set_features       = rtl8125_set_features,
#ifdef CONFIG_NET_POLL_CONTROLLER
        .ndo_poll_controller    = rtl8125_netpoll,
#endif
};

static int rtl8125_poll(struct napi_struct * const napi, int budget)
{
        struct rtl8125_private * const tp = netdev_priv(napi->dev);
        u8 i, work_done = 0;

        for (i = 0; i < tp->num_tx_rings; i++)
                rtl8125_tx_interrupt(tp, i, budget);
        for (i = 0; i < tp->num_rx_rings; i++)
                work_done += rtl8125_rx_interrupt(tp, i, budget);

        /* re-enable interrupts only when done */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0)
        if (work_done < budget && napi_complete_done(napi, work_done)) {
#else
        if (work_done < budget) {
                napi_complete_done(napi, work_done);
#endif
                rtl8125_switch_to_timer_interrupt(tp);
        }
        return work_done;
}

static int rtl8125_poll_msix_rx(struct napi_struct *const napi, const int budget)
{
        struct rtl8125_private * const tp = netdev_priv(napi->dev);
        const u8 message_id = napi - tp->napi;
        const int work_done = rtl8125_rx_interrupt(tp, message_id, budget);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0)
        if (work_done < budget && napi_complete_done(napi, work_done)) { // work_to_do
#else
        if (work_done < budget) { // work_to_do
                napi_complete_done(napi, work_done);
#endif
                rtl8125_enable_hw_interrupt_v2(tp, message_id);
        }
        return work_done;
}

static int rtl8125_poll_msix_tx(struct napi_struct *const napi, const int budget)
{
        struct rtl8125_private * const tp = netdev_priv(napi->dev);
        const u8 message_id = napi - tp->napi;

#ifdef ENABLE_MULTIPLE_TX_QUEUE
        rtl8125_tx_interrupt(tp, (message_id == 16) ? 0 : 1, budget);
#else
        rtl8125_tx_interrupt(tp, 0, budget);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0)
        if (likely(napi_complete_done(napi, 0))) {  // budget > 0
#else
        //if (budget > 0)
        {
                napi_complete_done(napi, 0);
#endif
                rtl8125_enable_hw_interrupt_v2(tp, message_id);
        }
        return 0;
}

static int rtl8125_poll_msix_ring(struct napi_struct *const napi, const int budget)
{
        struct rtl8125_private * const tp = netdev_priv(napi->dev);
        const u8 message_id = napi - tp->napi;
        int work_done;

        if (likely(message_id < tp->num_tx_rings))
                rtl8125_tx_interrupt((struct rtl8125_private * const) tp, message_id, budget);

        work_done = rtl8125_rx_interrupt(tp, message_id, budget);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0)
        if (work_done < budget && napi_complete_done(napi, work_done)) {
#else
        if (work_done < budget) {
                napi_complete_done(napi, work_done);
#endif
                rtl8125_enable_hw_interrupt_v2(tp, message_id);
        }
        return work_done;
}

static int rtl8125_poll_msix_other(struct napi_struct *napi, int budget)
{
        const struct rtl8125_private *tp = netdev_priv(napi->dev);
        const u8 message_id = napi - tp->napi;

        napi_complete_done(napi, budget);
        rtl8125_enable_hw_interrupt_v2(tp, message_id);
        return 1;
}

static void rtl8125_enable_napi(struct rtl8125_private * const tp)
{
        for (u8 i = 0; i < tp->irq_nvecs; i++)
                napi_enable(tp->napi + i);
}

static void rtl8125_disable_napi(struct rtl8125_private *const tp)
{
        for (int i = 0; i < tp->irq_nvecs; i++)
                napi_disable(tp->napi + i);
}

static void rtl8125_del_napi(struct rtl8125_private * const tp)
{
        for (int i = 0; i < tp->irq_nvecs; i++)
                netif_napi_del(tp->napi + i);
}

static void rtl8125_init_napi(struct rtl8125_private *const tp)
{
        for (int i = 0; i < tp->irq_nvecs; i++) {
                struct napi_struct *const napi = tp->napi + i;
                int (*poll)(struct napi_struct *, int) = rtl8125_poll;
                if (tp->irq_nvecs > 1) { // implies RTL_MSIX && tp->HwCurrIsrVer > 1
                        if (i < R8125_MAX_RX_QUEUES_VEC_V3)
                                poll = (tp->HwCurrIsrVer == 5 || tp->HwCurrIsrVer == 2) ?
                                        rtl8125_poll_msix_rx: rtl8125_poll_msix_ring;
                        else if (i > 18)
                                poll = rtl8125_poll_msix_other;
                        else if ((tp->HwCurrIsrVer == 5 && (i == 16 || i == 17)) ||
                                (tp->HwCurrIsrVer == 2 && (i == 16 || i == 18))) {
                                //poll = rtl8125_poll_msix_tx;
                                netif_napi_add_tx_weight(tp->dev, napi, rtl8125_poll_msix_tx, R8125_NAPI_WEIGHT);
                                continue;
                        }
                }
                netif_napi_add_weight(tp->dev, napi, poll, R8125_NAPI_WEIGHT);
        }
}

static int
rtl8125_set_real_num_queue(struct rtl8125_private *const tp)
{
        int retval = netif_set_real_num_tx_queues(tp->dev, tp->num_tx_rings);
        return (retval < 0) ? retval :
                netif_set_real_num_rx_queues(tp->dev, tp->num_rx_rings);
}

//#if LINUX_VERSION_CODE < KERNEL_VERSION(6,2,0)
void netdev_sw_irq_coalesce_default_on(struct net_device *const dev)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,8,0)
        WARN_ON(dev->reg_state == NETREG_REGISTERED);
        dev->gro_flush_timeout = SW_IRQ_COALESCING_TIMEOUT;
        dev->napi_defer_hard_irqs = 64;
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(5,8,0)
}
//#endif //LINUX_VERSION_CODE < KERNEL_VERSION(6,2,0)

static int __devinit
rtl8125_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
        struct net_device *dev = NULL;
        struct rtl8125_private *tp;
        void __iomem *ioaddr = NULL;
        int rc;

        assert(pdev != NULL);
        assert(ent != NULL);

        /*
        if (netif_msg_drv(&debug))
                printk(KERN_INFO "%s Ethernet controller driver %s loaded\n", MODULENAME, RTL8125_VERSION);
        */
        rc = rtl8125_init_board(pdev, &dev, &ioaddr);
        if (rc)
                goto out;

        tp = netdev_priv(dev);
        assert(ioaddr != NULL);

        spin_lock_init(&tp->phy_lock);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,11,0)
        dev->tstats = devm_netdev_alloc_pcpu_stats(&pdev->dev, struct pcpu_sw_netstats);
        if (!dev->tstats)
                goto err_out_1;
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(5,11,0)

        rc = rtl8125_msi_allocate_interrupts(tp);
        if (rc < 0) {
                dev_err(&pdev->dev, "Can't allocate interrupt\n");
                goto err_out_1;
        }

        rtl8125_init_software_variable(dev);
        dev->netdev_ops = &rtl8125_netdev_ops;
        dev->ethtool_ops = &rtl8125_ethtool_ops;
        dev->watchdog_timeo = RTL8125_TX_TIMEOUT;
#if defined(RTL_USE_NEW_INTR_API)
        pdev->irq = pci_irq_vector(pdev, 0);
#else
        dev->irq = pdev->irq;
#endif
        //dev->irq = rtl8125_get_irq(pdev);
        dev->base_addr = (unsigned long) ioaddr;
        rtl8125_init_napi(tp);

#ifdef CONFIG_R8125_VLAN
        dev->features |= NETIF_F_HW_VLAN_CTAG_TX | NETIF_F_HW_VLAN_CTAG_RX;
#endif

        /* There has been a number of reports that using SG/TSO results in
         * tx timeouts. However for a lot of people SG/TSO works fine.
         * Therefore disable both features by default, but allow users to
         * enable them. OK for RK3588 series to enable */
        tp->cp_cmd |= RTL_R16(tp, CPlusCmd);

        dev->features |= NETIF_F_IP_CSUM | NETIF_F_RXCSUM;
        dev->hw_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO |
                                NETIF_F_RXCSUM | NETIF_F_HW_VLAN_CTAG_TX | NETIF_F_HW_VLAN_CTAG_RX;
        dev->vlan_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO | NETIF_F_HIGHDMA;
        dev->priv_flags |= IFF_LIVE_ADDR_CHANGE;
        dev->hw_features |= NETIF_F_RXALL | NETIF_F_RXFCS | NETIF_F_IPV6_CSUM | NETIF_F_TSO6;
        dev->features |= NETIF_F_IPV6_CSUM;
        //if (! rtl8125_flag_is_set(tp, IsMcfg236))
        if (!is_8125A(tp))
                dev->features |= NETIF_F_SG | NETIF_F_TSO | NETIF_F_TSO6;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,19,0)
        netif_set_tso_max_size(dev, LSO_64K);
        netif_set_tso_max_segs(dev, NIC_MAX_PHYS_BUF_COUNT_LSO2);
#else //LINUX_VERSION_CODE >= KERNEL_VERSION(5,19,0)
        netif_set_gso_max_size(dev, LSO_64K);
        dev->gso_max_segs = NIC_MAX_PHYS_BUF_COUNT_LSO2;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,7,0)
        dev->gso_min_segs = NIC_MIN_PHYS_BUF_COUNT;
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(4,7,0)
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(5,19,0)

#ifdef ENABLE_RSS_SUPPORT
        if (rtl8125_flag_is_set(tp, EnableRss)) {
                dev->hw_features |= NETIF_F_RXHASH;
                dev->features |= NETIF_F_RXHASH;
        }
#endif
        netdev_sw_irq_coalesce_default_on(dev);
        rtl8125_init_all_schedule_work(tp);
        rc = rtl8125_set_real_num_queue(tp);
        if (rc < 0)
                goto err_out;

        rtl8125_exit_oob(tp);
        rtl8125_powerup_pll(tp);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,11,0)
        dev->ethtool->wol_enabled = 1;
#else
        dev->wol_enabled = 1;
#endif
        rtl8125_hw_init(tp);
        rtl8125_hw_reset(tp);
        printk(KERN_INFO "%s: This product is covered by one or more of the following patents: US6,570,884, US6,115,776, and US6,327,625.\n", MODULENAME);

#ifdef ENABLE_R8125_EEPROM
        /* Get production type from EEPROM */
        if (rtl8125_eeprom_type(tp))
                rtl8125_set_eeprom_sel_low(tp);
        else
                printk(KERN_INFO "r8125: No (supported) EEPROM found\n");
#endif

        rtl8125_get_mac_address(dev);
        //tp->fw_name = rtl_chip_info[chipset(tp)].fw_name;
        tp->tally_vaddr = dma_alloc_coherent(&pdev->dev, sizeof(*tp->tally_vaddr),
                                             &tp->tally_paddr, GFP_KERNEL);
        if (!tp->tally_vaddr) {
                rc = -ENOMEM;
                goto err_out;
        }

        rtl8125_tally_counter_clear(tp);
        pci_set_drvdata(pdev, dev);

        rc = register_netdev(dev);
        if (rc)
                goto err_out;

        rtl8125_disable_rxdvgate(tp);
        device_set_wakeup_enable(&pdev->dev, tp->wol_opts);  // implies WOL_ENABLED if not zero
        netif_carrier_off(dev);
#ifdef ENABLE_R8125_SYSFS
        rtl8125_sysfs_init(tp, dev);
#endif
        printk("%s", GPL_CLAIM);

out:
        return rc;

err_out:
        if (tp->tally_vaddr != NULL) {
                dma_free_coherent(&pdev->dev, sizeof(*tp->tally_vaddr), tp->tally_vaddr, tp->tally_paddr);
                tp->tally_vaddr = NULL;
        }
        rtl8125_del_napi(tp);
        rtl8125_disable_msi(pdev, tp);

err_out_1:
        rtl8125_release_board(pdev, dev);
        goto out;
}

static void __devexit
rtl8125_remove(struct pci_dev *pdev)
{
        struct net_device *dev = pci_get_drvdata(pdev);
        struct rtl8125_private *tp = netdev_priv(dev);

        assert(dev != NULL);
        assert(tp != NULL);

        set_bit(R8125_FLAG_DOWN, tp->task_flags);
        rtl8125_cancel_all_schedule_work(tp);

        rtl8125_del_napi(tp);

#ifdef ENABLE_R8125_SYSFS
        rtl8125_sysfs_remove(dev);
#endif //ENABLE_R8125_SYSFS

        unregister_netdev(dev);
        rtl8125_disable_msi(pdev, tp);
#ifdef ENABLE_R8125_PROCFS
        rtl8125_proc_remove(dev);
#endif
        if (tp->tally_vaddr != NULL) {
                dma_free_coherent(&pdev->dev, sizeof(*tp->tally_vaddr), tp->tally_vaddr, tp->tally_paddr);
                tp->tally_vaddr = NULL;
        }

        rtl8125_release_board(pdev, dev);

#ifdef ENABLE_USE_FIRMWARE_FILE
        rtl8125_release_firmware(tp);
#endif

        pci_set_drvdata(pdev, NULL);
}

static void rtl8125_free_irq(struct rtl8125_private *tp)
{
        for (u8 i = 0; i < tp->irq_nvecs; i++) {
                if (tp->irq[i]) //irq->requested) {
#if defined(RTL_USE_NEW_INTR_API)
                        pci_free_irq(tp->pci_dev, i, &tp->napi[i]);
#else
                        free_irq(irq->vector, &tp->napi[i]);
#endif
        }
}

static int rtl8125_alloc_irq(struct rtl8125_private *tp, struct net_device *dev)
{
        u8 i, rc = 0, irq_nvecs = tp->irq_nvecs;
        irq_handler_t handler = (irq_nvecs > 1) ? rtl8125_interrupt_msix : rtl8125_interrupt;

#if defined(RTL_USE_NEW_INTR_API)
        for (i = 0; i < irq_nvecs; i++) {
                rc = pci_request_irq(tp->pci_dev, i, handler, NULL, tp->napi + i, "%s-%d", dev->name, i);
                if (rc)
                        break;
                tp->irq[i] = pci_irq_vector(tp->pci_dev, i);
        }
#else
        unsigned long irq_flags = 0;
        if (irq_nvecs > 1) { // implies RTL_MSIX
                static char irq_name[IFNAMSIZ + 8];
                for (i = 0; i < irq_nvecs; i++) {
                        snprintf(irq_name, IFNAMSIZ + 8, "%s-%d", dev->name, i);
                        rc = request_irq(tp->irq[i], handler, irq_flags, irq_name, tp->napi + i);
                        if (rc)
                                break;
                }
        } else {  //RTL_MSI or traditional IRQ
                tp->irq[0] = dev->irq;
                if (!pci_dev_msi_enabled(tp->pci_dev))  // implies !RTL_MSI
                        irq_flags |= IRQF_SHARED;
                rc = request_irq(dev->irq, handler, irq_flags, dev->name, napi);
        }
#endif
        if (rc)
                rtl8125_free_irq(tp);
        return rc;
}

#define DESC_ALLOC_SIZE(tp) TX_DESC_ALLOC_SIZE * tp->num_tx_rings + RX_DESC_ALLOC_SIZE(tp) * tp->num_rx_rings
static int rtl8125_alloc_desc(struct rtl8125_private *tp)
{
        struct rtl8125_tx_ring *tx_ring;
        struct rtl8125_rx_ring *rx_ring;
        struct device *device = tp->device;
        unsigned int RxDescAllocSize = RX_DESC_ALLOC_SIZE(tp);
        dma_addr_t mapping = (dma_addr_t) dma_alloc_coherent(device, DESC_ALLOC_SIZE(tp),
                &tp->DescArrayPhyAddr, GFP_KERNEL | DMA_ATTR_FORCE_CONTIGUOUS);
        if (!mapping) {
                printk(KERN_ERR "r8125 failed to allocate descriptor DMA memory\n");
                return -1;
        }

        for (int i = 0; i < tp->num_tx_rings; i++, mapping += TX_DESC_ALLOC_SIZE) {
                tx_ring = &tp->tx_ring[i];
                tx_ring->TxDescArray = (struct TxDesc *) mapping;
        }
        //mapping = (mapping + 0xFF) & ~(0xFF); // Align to 256 bytes

        for (int i = 0; i < tp->num_rx_rings; i++, mapping += RxDescAllocSize) { //, mem += RxDescAllocSize
                rx_ring = &tp->rx_ring[i];
                rx_ring->RxDescArray = (struct RxDesc *) mapping;
        }
        return 0;
}

static void rtl8125_free_desc(struct rtl8125_private *tp)
{
        //printk(KERN_INFO "r8125 rtl8125_free_desc 0x%px, mapping 0x%llx\n", tp->tx_ring[0].TxDescArray, tp->DescArrayPhyAddr);
        if (tp->tx_ring[0].TxDescArray)
                dma_free_coherent(tp->device, DESC_ALLOC_SIZE(tp), tp->tx_ring[0].TxDescArray, tp->DescArrayPhyAddr);
        tp->DescArrayPhyAddr = 0;
        for (int i = 0; i < tp->num_tx_rings; i++)
                tp->tx_ring[i].TxDescArray = NULL;

        for (int i = 0; i < tp->num_rx_rings; i++)
                tp->rx_ring[i].RxDescArray = NULL;
}

#ifdef ENABLE_LARGE_PAGES
#define RX_BUFFER_ALLOC_SIZE(tp) (RX_BUF_SIZE + 1) * NUM_RX_DESC
static int
rtl8125_alloc_rx_buffer(struct rtl8125_private * const tp, struct rtl8125_rx_ring *const ring)
{
        struct device * const device = tp->device;
        dma_addr_t mapping;
        const int node = dev_to_node(device);
        const int size = RX_BUFFER_ALLOC_SIZE(tp);
        const int order = get_order(size);

        //get free page
        struct page * const page = alloc_pages_node(node, GFP_KERNEL| __GFP_COMP, order);
        //page = dev_alloc_pages(order); //| __GFP_COMP,
        if (unlikely(!page)) {
                netdev_err(tp->dev, "Failed to map RX buffer pages!\n");
                return -ENOMEM;
        }
        mapping = dma_map_page_attrs(device, page, 0, size, DMA_FROM_DEVICE,
                (DMA_ATTR_SKIP_CPU_SYNC | DMA_ATTR_WEAK_ORDERING));

        if (unlikely(dma_mapping_error(device, mapping))) {
                netdev_err(tp->dev, "Failed to map RX buffer DMA!\n");
                __free_pages(page, order);
                return -ENOMEM;
        }
        netdev_info(tp->dev, "RX buffer alloc (Large Pages, order %d, %d bytes)\n", order, size);
        //ring->RxBufferPageAddr = page;
        ring->RxBufferPhyAddr = mapping;
        return 0;
}
#endif

#ifdef ENABLE_USE_FIRMWARE_FILE
static void rtl8125_request_firmware(struct rtl8125_private *const tp)
{
        struct rtl8125_fw *rtl_fw;

        /* firmware loaded already or no firmware available */
        if (tp->rtl_fw || !rtl_chip_info[chipset(tp)].fw_name)
                return;

        rtl_fw = kzalloc(sizeof(*rtl_fw), GFP_KERNEL);
        if (!rtl_fw)
                return;

        rtl_fw->phy_write = rtl8125_mdio_write;
        rtl_fw->phy_read = rtl8125_mdio_read;
        rtl_fw->mac_mcu_write = mac_mcu_write;
        rtl_fw->mac_mcu_read = mac_mcu_read;
        rtl_fw->fw_name = rtl_chip_info[chipset(tp)].fw_name;
        rtl_fw->dev = tp->device;

        if (rtl8125_fw_request_firmware(rtl_fw))
                kfree(rtl_fw);
        else
                tp->rtl_fw = rtl_fw;
}
#endif

static int rtl8125_open(struct net_device *dev)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        int retval = -ENOMEM;

#ifdef ENABLE_R8125_PROCFS
        rtl8125_proc_init(dev);
#endif
        /*
         * Rx and Tx descriptors needs 256 bytes alignment.
         * pci_alloc_consistent provides more.
         */
        if (rtl8125_alloc_desc(tp) < 0)
                goto err_free_all_allocated_mem;

        retval = rtl8125_init_rings(tp);
        if (retval < 0)
                goto err_free_all_allocated_mem;

        retval = rtl8125_alloc_irq(tp, dev);
        if (retval < 0)
                goto err_free_all_allocated_mem;

        if (netif_msg_probe(tp)) {
                printk(KERN_INFO "%s: 0x%lx, "
                       "%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x, "
                       "IRQ %d\n", dev->name, dev->base_addr,
                       dev->dev_addr[0], dev->dev_addr[1],
                       dev->dev_addr[2], dev->dev_addr[3],
                       dev->dev_addr[4], dev->dev_addr[5], dev->irq);
        }

#ifdef ENABLE_USE_FIRMWARE_FILE
        rtl8125_request_firmware(tp);
#endif
        pci_set_master(tp->pci_dev);
        rtl8125_enable_napi(tp);
        printk(KERN_INFO PFX "%s: %d Transmit Queue(s)\n", dev->name, tp->num_tx_rings);
        printk(KERN_INFO PFX "%s: %d Receive Queue(s) (RSS %s)\n", dev->name, tp->num_rx_rings,
                rtl8125_flag_to_bool(tp, EnableRss) ? "enabled" : "disabled");
        rtl8125_exit_oob(tp);
        rtl8125_up(dev);

#ifdef ENABLE_PTP_SUPPORT
        if (rtl8125_flag_is_set(tp, EnablePtp))
                rtl8125_ptp_init(tp, dev);
#endif
        clear_bit(R8125_FLAG_DOWN, tp->task_flags);
        if (rtl8125_flag_is_set(tp, ResumeNoSpeedChange))
                rtl8125_check_link_status(dev);
        else
                rtl8125_set_speed(tp, rtl8125_flag_to_bool(tp, AutoNegMode), tp->speed, tp->duplex, tp->advertising);

#ifdef ENABLE_ESD
        if (tp->esd_flag == 0) {
                //rtl8125_request_esd_timer(dev);
                rtl8125_schedule_esd_work(tp);
        }
#endif
        //rtl8125_request_link_timer(dev);
        rtl8125_enable_hw_linkchg_interrupt(tp);

out:
        return retval;

err_free_all_allocated_mem:
        rtl8125_free_desc(tp);

        goto out;
}

static void
set_offset70F(struct rtl8125_private *tp, u8 setting)
{
        u32 csi_tmp = rtl8125_csi_read(tp, 0x70c) & 0xc0ffffff;
        /*set PCI configuration space offset 0x70F to setting*/
        /*When the register offset of PCI configuration space larger than 0xff, use CSI to access it.*/
        rtl8125_csi_write(tp, 0x70c, csi_tmp | (u32)(setting & 0x3f) << 24);
}

#ifndef CONFIG_SOC_LAN
static void
set_offset79(struct rtl8125_private *tp, u8 setting)
{
        //Set PCI configuration space offset 0x79 to setting
        struct pci_dev *pdev = tp->pci_dev;
        u8 device_control;

        pci_read_config_byte(pdev, 0x79, &device_control);
        device_control &= ~0x70 | setting;
        pci_write_config_byte(pdev, 0x79, device_control);
}
#endif

static void
rtl8125_set_rx_mode(struct net_device *dev)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        u32 mc_filter[2] = { 0xffffffff, 0xffffffff };  // Multicast hash filter
        int rx_mode = AcceptBroadcast | AcceptMyPhys;
        u32 tmp;

        if (dev->flags & IFF_PROMISC) {
                /* Unconditionally log net taps. */
                if (netif_msg_link(tp))
                        printk(KERN_NOTICE "%s: Promiscuous mode enabled.\n", dev->name);

                rx_mode |= AcceptAllPhys;
         } else if (!(dev->flags & IFF_MULTICAST)) {
                rx_mode &= ~AcceptMulticast;
        } else if (dev->flags & IFF_ALLMULTI) {
                /* accept all multicasts. */
                rx_mode |= AcceptMulticast ;
        } else {
                struct netdev_hw_addr *ha;

                rx_mode = AcceptBroadcast | AcceptMyPhys;
                mc_filter[1] = mc_filter[0] = 0;
                netdev_for_each_mc_addr(ha, dev) {
                        u32 bit_nr = eth_hw_addr_crc(ha) >> 26;
                        mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
                        rx_mode |= AcceptMulticast;
                }
        }

        if (dev->features & NETIF_F_RXALL)
                rx_mode |= (AcceptErr | AcceptRunt);

        tmp = mc_filter[0];
        mc_filter[0] = swab32(mc_filter[1]);
        mc_filter[1] = swab32(tmp);

        tmp = tp->rx_config | rx_mode | (RTL_R32(tp, RxConfig) & rtl_chip_info[chipset(tp)].RxConfigMask);

        RTL_W32(tp, RxConfig, tmp);
        RTL_W32(tp, MAR0 + 0, mc_filter[0]);
        RTL_W32(tp, MAR0 + 4, mc_filter[1]);
}

static void
rtl8125_set_rx_q_num(struct rtl8125_private *tp)
{
        u16 rx_q_num = ((u16)ilog2(tp->num_rx_rings) & (BIT_0 | BIT_1 | BIT_2)) << 2;
        u16 q_ctrl = (RTL_R16(tp, Q_NUM_CTRL_8125) & ~(BIT_2 | BIT_3 | BIT_4)) | rx_q_num;
        RTL_W16(tp, Q_NUM_CTRL_8125, q_ctrl);
}

static void
rtl8125_set_tx_q_num(struct rtl8125_private *tp)
{
        rtl8125_clear_set_mac_ocp_bit(tp, 0xE63E, (u16)(BIT_11 | BIT_10),
                (u16)((ilog2(tp->num_tx_rings) & 0x03) << 10));
}

static void
rtl8125_enable_mcu(struct rtl8125_private *tp, bool enable)
{
        if (enable)
                rtl8125_set_mac_ocp_bit(tp, 0xC0B4, BIT_0);
        else
                rtl8125_clear_mac_ocp_bit(tp, 0xC0B4, BIT_0);
}

static void
rtl8125_clear_tcam_entries(struct rtl8125_private *tp)
{
        rtl8125_set_mac_ocp_bit(tp, 0xEB54, BIT_0);
        udelay(1);
        rtl8125_clear_mac_ocp_bit(tp, 0xEB54, BIT_0);
}

static u8
rtl8125_get_l1off_cap_bits(struct rtl8125_private *tp)
{
        //return rtl8125_flag_is_set(tp, IsMcfg236) ?
        return is_8125A(tp) ?
                (BIT_0 | BIT_1) : (BIT_0 | BIT_1 | BIT_2 | BIT_3);
}

static void
rtl8125_hw_config(struct rtl8125_private *tp, struct net_device *dev)
{
        u16 mac_ocp_data;

        rtl8125_disable_rx_packet_filter(tp);
        rtl8125_hw_reset(tp);
        rtl8125_unlock_config_regs(tp);
        /* Disable aspm clkreq internal */
        rtl8125_enable_force_clkreq(tp, 0);
        rtl8125_enable_aspm_clkreq_lock(tp, 0);
        rtl8125_set_eee_lpi_timer(tp);

        // Keep magic packet only
        rtl8125_clear_mac_ocp_bit(tp, 0xC0B6, BIT_0);
        rtl8125_tally_counter_addr_fill(tp);
        rtl8125_enable_extend_tally_couter(tp);
        rtl8125_desc_addr_fill(tp);

        /* TX: Set DMA burst size and Interframe Gap Time */
        RTL_W32(tp, TxConfig, (TX_DMA_BURST_unlimited << TxDMAShift) |
                (InterFrameGap << TxInterFrameGapShift));

#ifdef ENABLE_TX_NO_CLOSE
        RTL_W32(tp, TxConfig, (RTL_R32(tp, TxConfig) | BIT_6));
#endif

#ifdef ENABLE_DOUBLE_VLAN
        rtl8125_enable_double_vlan(tp);
#else
        rtl8125_disable_double_vlan(tp);
#endif
        /* TCAM */
        if (is_8125AB(tp))
                RTL_W16(tp, 0x382, 0x221B);

        set_offset70F(tp, 0x27);
#ifndef CONFIG_SOC_LAN
        set_offset79(tp, 0x40);
#endif

#ifdef ENABLE_RSS_SUPPORT
        rtl8125_config_rss(tp);
#else
        RTL_W32(tp, RSS_CTRL_8125, 0x00);
#endif
        /* VMQ_control */
        rtl8125_set_rx_q_num(tp);

        /* Disable speed down */
        RTL_W8(tp, Config1, RTL_R8(tp, Config1) & ~Speed_down);

        /* CRC disable set */
        rtl8125_mac_ocp_write(tp, 0xC140, 0xFFFF);
        rtl8125_mac_ocp_write(tp, 0xC142, 0xFFFF);

        /* New TX desc format */
        mac_ocp_data = rtl8125_mac_ocp_read(tp, 0xEB58);
        rtl8125_mac_ocp_write(tp, 0xEB58, mac_ocp_data | (BIT_0));

        /* MTPS 15-8 maximum tx use credit number
                 7-0 reserved for pcie product line */
        mac_ocp_data = rtl8125_mac_ocp_read(tp, 0xE614);
        mac_ocp_data &= ~(BIT_10 | BIT_9 | BIT_8);
        if (is_8125B(tp))
                mac_ocp_data |= ((2 & 0x07) << 8);
        else
                mac_ocp_data |= ((3 & 0x07) << 8);
        rtl8125_mac_ocp_write(tp, 0xE614, mac_ocp_data);
        rtl8125_set_tx_q_num(tp);

        mac_ocp_data = rtl8125_mac_ocp_read(tp, 0xE63E);
        mac_ocp_data &= ~(BIT_5 | BIT_4);
        mac_ocp_data |= ((0x02 & 0x03) << 4);
        rtl8125_mac_ocp_write(tp, 0xE63E, mac_ocp_data);
        rtl8125_enable_mcu(tp, 0);
        rtl8125_enable_mcu(tp, 1);

        /* FTR_MCU_CTRL: 3-2 txpla packet valid start */
        mac_ocp_data = rtl8125_mac_ocp_read(tp, 0xC0B4) | (BIT_3 | BIT_2);
        rtl8125_mac_ocp_write(tp, 0xC0B4, mac_ocp_data);

        mac_ocp_data = rtl8125_mac_ocp_read(tp, 0xEB6A);
        mac_ocp_data &= ~(BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
        mac_ocp_data |= (BIT_5 | BIT_4 | BIT_1 | BIT_0);
        rtl8125_mac_ocp_write(tp, 0xEB6A, mac_ocp_data);

        mac_ocp_data = rtl8125_mac_ocp_read(tp, 0xEB50);
        mac_ocp_data &= ~(BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5);
        rtl8125_mac_ocp_write(tp, 0xEB50, mac_ocp_data | (BIT_6));

        mac_ocp_data = rtl8125_mac_ocp_read(tp, 0xE056);
        mac_ocp_data &= ~(BIT_7 | BIT_6 | BIT_5 | BIT_4);
        //mac_ocp_data |= (BIT_4 | BIT_5);
        rtl8125_mac_ocp_write(tp, 0xE056, mac_ocp_data);

        RTL_W8(tp, TDFNR, 0x10);

        /* EEE_CR */
        mac_ocp_data = rtl8125_mac_ocp_read(tp, 0xE040);
        mac_ocp_data &= ~(BIT_12);
        rtl8125_mac_ocp_write(tp, 0xE040, mac_ocp_data);

        mac_ocp_data = rtl8125_mac_ocp_read(tp, 0xEA1C);
        mac_ocp_data &= ~(BIT_1 | BIT_0);
        mac_ocp_data |= (BIT_0);
        rtl8125_mac_ocp_write(tp, 0xEA1C, mac_ocp_data);

        /* MAC_PWRDWN_CR0 */
        if (is_8125D(tp))  // (tp->mcfg >= CFG_METHOD_10)
                rtl8125_mac_ocp_write(tp, 0xE0C0, 0x4403);
        else
                rtl8125_mac_ocp_write(tp, 0xE0C0, 0x4000);
        rtl8125_set_mac_ocp_bit(tp, 0xE052, (BIT_6 | BIT_5));
        rtl8125_clear_mac_ocp_bit(tp, 0xE052, BIT_3 | BIT_7);

        mac_ocp_data = rtl8125_mac_ocp_read(tp, 0xD430);
        mac_ocp_data &= ~(BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
        rtl8125_mac_ocp_write(tp, 0xD430, mac_ocp_data | 0x45F);

        //rtl8125_mac_ocp_write(tp, 0xE0C0, 0x4F87);
        RTL_W8(tp, RDSAR1, RTL_R8(tp, RDSAR1) | BIT_6 | BIT_7);

        //if (rtl8125_flag_is_set(tp, IsMcfg236))
        if (is_8125A(tp))
                RTL_W8(tp, MCUCmd_reg, RTL_R8(tp, MCUCmd_reg) | BIT_0);

        //if (tp->mcfg <= CFG_METHOD_9)
        if (!is_8125D(tp))
                rtl8125_disable_eee_plus(tp);

        mac_ocp_data = rtl8125_mac_ocp_read(tp, 0xEA1C);
        mac_ocp_data &= ~(BIT_2);
        rtl8125_mac_ocp_write(tp, 0xEA1C, mac_ocp_data);

        /* Clear TCAM entries */
        rtl8125_clear_tcam_entries(tp);

        RTL_W16(tp, 0x1880, RTL_R16(tp, 0x1880) & ~(BIT_4 | BIT_5));

        //if (tp->HwSuppRxDescType == RX_DESC_RING_TYPE_4)
        //if (tp->RxDescType == RX_DESC_RING_TYPE_4)
        if (is_8125BPD(tp))
                RTL_W8(tp, 0xd8, RTL_R8(tp, 0xd8) | EnableRxDescV4_0);
        else
                RTL_W8(tp, 0xd8, RTL_R8(tp, 0xd8) & ~EnableRxDescV4_0);

        /* config interrupt type for RTL8125B */
        if (rtl_chip_info[chipset(tp)].HwSuppIsrVer)
                rtl8125_hw_set_interrupt_type(tp);

        //other hw parameters
        rtl8125_hw_clear_int_timer(tp);
        rtl8125_hw_clear_int_miti_timer(tp);

///*
        if (rtl8125_flag_is_set(tp, UseIntrTimer) && tp->irq_nvecs > 1) {  // implies RTL_MSIX and HwSuppIntMitiVer > 3
                for (int i = 0; i < tp->irq_nvecs; i++)
                        rtl8125_hw_set_int_miti_timer(tp, i, timer_count_v2);
        }
//*/
        //DMY_PWR_REG_0 - (1)ERI(0xD4)(OCP 0xC0AC).bit[7:12]=6'b111111, L1 Mask
        rtl8125_set_mac_ocp_bit(tp, 0xC0AC, (BIT_7 | BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12));

        rtl8125_mac_ocp_write(tp, 0xE098, 0xC302);
        // test r8168_mac_ocp_read(tp, 0xe00e) & BIT(13);  ?

        rtl8125_disable_pci_offset_99(tp);
        if (aspm && (tp->org_pci_offset_99 & (BIT_2 | BIT_5 | BIT_6)))
                rtl8125_init_pci_offset_99(tp);
        
        rtl8125_disable_pci_offset_180(tp);
        if (aspm && (tp->org_pci_offset_180 & rtl8125_get_l1off_cap_bits(tp)))
                rtl8125_init_pci_offset_180(tp);

        if (is_8125D(tp)) //(tp->mcfg >= CFG_METHOD_10)
                rtl8125_set_pfm_patch(tp, 0);

        tp->cp_cmd &= ~(EnableBist | Macdbgo_oe | Force_halfdup | Force_rxflow_en | Force_txflow_en |
                        Cxpl_dbg_sel | ASF | Macdbgo_sel);

        rtl8125_hw_set_features(tp, dev->features);
        RTL_W16(tp, RxMaxSize, RX_BUF_SIZE + 1);

        rtl8125_disable_rxdvgate(tp);

#ifdef ENABLE_ESD
        if (!tp->pci_cfg_is_read) {
                struct pci_dev *pdev = tp->pci_dev;
                //u32 hwPcieSNOffset = (tp->mcfg <= CFG_METHOD_7) ? 0x16C: 0x168;
                u32 hwPcieSNOffset = is_8125AB(tp) ? 0x16C: 0x168;
                pci_read_config_byte(pdev, PCI_COMMAND, &tp->pci_cfg_space.cmd);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_0, &tp->pci_cfg_space.io_base_l);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_0 + 2, &tp->pci_cfg_space.io_base_h);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_2, &tp->pci_cfg_space.mem_base_l);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_2 + 2, &tp->pci_cfg_space.mem_base_h);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_3, &tp->pci_cfg_space.resv_0x1c_l);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_3 + 2, &tp->pci_cfg_space.resv_0x1c_h);
                pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &tp->pci_cfg_space.ilr);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_4, &tp->pci_cfg_space.resv_0x20_l);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_4 + 2, &tp->pci_cfg_space.resv_0x20_h);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_5, &tp->pci_cfg_space.resv_0x24_l);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_5 + 2, &tp->pci_cfg_space.resv_0x24_h);
                pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, &tp->pci_cfg_space.resv_0x2c_l);
                pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID + 2, &tp->pci_cfg_space.resv_0x2c_h);
                // Always true: if (tp->HwPcieSNOffset > 0) {
                tp->pci_cfg_space.pci_sn_l = rtl8125_csi_read(tp, hwPcieSNOffset);
                tp->pci_cfg_space.pci_sn_h = rtl8125_csi_read(tp, hwPcieSNOffset + 4);

                tp->pci_cfg_is_read = 1;
        }
#endif

        /* Set Rx packet filter */
        rtl8125_set_rx_mode(dev);

        rtl8125_enable_aspm_clkreq_lock(tp, aspm ? 1 : 0);
        rtl8125_lock_config_regs(tp);
        udelay(10);
}

static void rtl8125_hw_start(struct rtl8125_private *tp)
{
        RTL_W8(tp, ChipCmd, CmdTxEnb | CmdRxEnb);
        rtl8125_enable_hw_interrupt(tp);
        //rtl8125_lib_reset_complete(tp);
}

static int
rtl8125_change_mtu(struct net_device *dev, int new_mtu)
{
        struct rtl8125_private *tp = netdev_priv(dev);
        int ret = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,10,0)
        if (new_mtu < ETH_MIN_MTU)
                return -EINVAL;
        else if (new_mtu > MAX_JUMBO_FRAME_SIZE)
                new_mtu = MAX_JUMBO_FRAME_SIZE;
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(4,10,0)

        dev->mtu = new_mtu;
        tp->eee.tx_lpi_timer = dev->mtu + ETH_HLEN + 0x20;

        if (!netif_running(dev))
                goto out;

        rtl8125_down(tp, dev);

        ret = rtl8125_init_rings(tp);
        if (ret < 0)
                goto err_out;

        rtl8125_enable_napi(tp);

        if (rtl8125_xmii_link_ok(tp))
                rtl8125_link_up_patch(tp, dev);
        else
                rtl8125_link_down_patch(tp, dev);

        //mod_timer(&tp->esd_timer, jiffies + RTL8125_ESD_TIMEOUT);
        //mod_timer(&tp->link_timer, jiffies + RTL8125_LINK_TIMEOUT);
out:
        netdev_update_features(dev);

err_out:
        return ret;
}

static __always_inline void
rtl8125_set_rx_desc_addr(struct rtl8125_private *tp, struct RxDesc *desc, dma_addr_t mapping)
{
#if defined(ENABLE_RSS_SUPPORT) || defined(ENABLE_PTP_SUPPORT)
        if (tp->RxDescType == RX_DESC_RING_TYPE_3)
               ((struct RxDescV3 *)desc)->addr = cpu_to_le64(mapping);
        else
#endif
#ifdef ENABLE_RSS_SUPPORT
        if (likely(tp->RxDescType == RX_DESC_RING_TYPE_4))
               ((struct RxDescV4 *)desc)->addr = cpu_to_le64(mapping);
        else
#endif
        desc->addr = cpu_to_le64(mapping);
}

static __always_inline void
rtl8125_mark_to_asic_v1(struct RxDesc * const desc)
{
        const u32 eor = le32_to_cpu(desc->opts1) & RingEnd;
        //desc->opt2 = 0;
        WRITE_ONCE(desc->opts1, cpu_to_le32(DescOwn | eor | RX_BUF_SIZE));
}

static __always_inline void
rtl8125_mark_to_asic_v3(struct RxDescV3 * const descv3)
{
        const u32 eor = le32_to_cpu(descv3->RxDescNormalDDWord4.opts1) & RingEnd;
        //descv3->RxDescNormalDDWord4.opts2 = 0;
        WRITE_ONCE(descv3->RxDescNormalDDWord4.opts1, cpu_to_le32(DescOwn | eor | RX_BUF_SIZE));
}

static __always_inline void
rtl8125_mark_to_asic_v4(struct RxDescV4 * const descv4)
{
        const u32 eor = le32_to_cpu(descv4->RxDescNormalDDWord2.opts1) & RingEnd;
        //descv4->RxDescNormalDDWord2.opts2 = 0;
        WRITE_ONCE(descv4->RxDescNormalDDWord2.opts1, cpu_to_le32(DescOwn | eor | RX_BUF_SIZE));
}

void
rtl8125_mark_to_asic(const struct rtl8125_private * const tp, struct RxDesc * const desc)
{
#if defined(ENABLE_RSS_SUPPORT) || defined(ENABLE_PTP_SUPPORT)
        if (tp->RxDescType == RX_DESC_RING_TYPE_3)
               rtl8125_mark_to_asic_v3((struct RxDescV3 *)desc);
        else
#endif
#ifdef ENABLE_RSS_SUPPORT
        if (likely(tp->RxDescType == RX_DESC_RING_TYPE_4))
               rtl8125_mark_to_asic_v4((struct RxDescV4 *)desc);
        else
#endif
        rtl8125_mark_to_asic_v1(desc);
}

__always_inline void
rtl8125_map_and_mark_to_asic_v3(struct RxDescV3 * const desc, const dma_addr_t mapping)
{
        desc->addr = cpu_to_le64(mapping);
        /* Force memory writes to complete before releasing descriptor */
        dma_wmb();
        rtl8125_mark_to_asic_v3(desc);
}

__always_inline void
rtl8125_map_and_mark_to_asic_v4(struct RxDescV4 * const desc, const dma_addr_t mapping)
{
        desc->addr = cpu_to_le64(mapping);
        /* Force memory writes to complete before releasing descriptor */
        dma_wmb();
        rtl8125_mark_to_asic_v4(desc);
}

__always_inline void
rtl8125_map_and_mark_to_asic(const struct rtl8125_private *const tp,
        struct RxDesc *const desc, const dma_addr_t mapping)
{
#if defined(ENABLE_RSS_SUPPORT) || defined(ENABLE_PTP_SUPPORT)
        if (tp->RxDescType == RX_DESC_RING_TYPE_3)
                rtl8125_map_and_mark_to_asic_v3((struct RxDescV3 *)desc, mapping);
        else
#endif
#ifdef ENABLE_RSS_SUPPORT
        if (likely(tp->RxDescType == RX_DESC_RING_TYPE_4)) {
                rtl8125_map_and_mark_to_asic_v4((struct RxDescV4 *)desc, mapping);
        } else
#endif
        desc->addr = cpu_to_le64(mapping);
        /* Force memory writes to complete before releasing descriptor */
        dma_wmb();
        rtl8125_mark_to_asic_v1(desc);
}

static inline void
rtl8125_make_unusable_by_asic(const struct rtl8125_private *const tp, struct RxDesc *const desc)
{
#if defined(ENABLE_RSS_SUPPORT) || defined(ENABLE_PTP_SUPPORT)
        if (tp->RxDescType == RX_DESC_RING_TYPE_3) {
                ((struct RxDescV3 *)desc)->addr = RTL8125_MAGIC_NUMBER;
                ((struct RxDescV3 *)desc)->RxDescNormalDDWord4.opts1 &= ~cpu_to_le32(DescOwn | RsvdMaskV3);
                return;
        }
#endif
#ifdef ENABLE_RSS_SUPPORT
        if (likely(tp->RxDescType == RX_DESC_RING_TYPE_4)) {
                ((struct RxDescV4 *)desc)->addr = RTL8125_MAGIC_NUMBER;
                ((struct RxDescV4 *)desc)->RxDescNormalDDWord2.opts1 &= ~cpu_to_le32(DescOwn | RsvdMaskV4);
                return;
        }
#endif
        desc->addr = RTL8125_MAGIC_NUMBER;
        desc->opts1 &= ~cpu_to_le32(DescOwn | RsvdMask);
}

static inline int
rtl8125_alloc_rx_buffer_page(struct rtl8125_private *const tp, const u8 ring_index, const unsigned int entry)
{
        struct rtl8125_rx_ring * const ring = tp->rx_ring + ring_index;
        struct RxDesc *const desc = rtl8125_get_rxdesc(tp, ring->RxDescArray, entry);
        //int node = dev_to_node(tp->device);

#ifdef ENABLE_LARGE_PAGES
        dma_addr_t mapping = GetRxBufferDmaPhyAddr(ring, entry);
#else
        dma_addr_t mapping;
        struct device *const device = tp->device;
        struct page * const page = dev_alloc_pages(get_order(RX_BUF_SIZE));
        //page = alloc_pages_node(node, GFP_KERNEL, order);

        if (unlikely(!page))
                return -ENOMEM;

        //mapping = dma_map_page(device, page, 0, RX_BUF_SIZE, DMA_FROM_DEVICE);
        mapping = dma_map_page_attrs(device, page, 0, RX_BUF_SIZE, DMA_FROM_DEVICE,
                (DMA_ATTR_SKIP_CPU_SYNC | DMA_ATTR_WEAK_ORDERING));

        if (unlikely(dma_mapping_error(device, mapping))) {
                netdev_err(tp->dev, "Failed to map RX DMA!\n");
                __free_pages(page, get_order(RX_BUF_SIZE));
                return -ENOMEM;
        }
        ring->RxBufferDmaPhyAddr[entry] = mapping;
        // No longer required to store the page, using phys_to_virt
        //ring->RxBufferPageAddr[entry] = page;
#endif
        /*
        if (entry < 10)
                printk(KERN_INFO "r8125 0x%px page alloc_rx_page 0x%px, mapping 0x%llx page_to_phys 0x%llx phys_to_page 0x%llx phys_to_virt 0x%llx\n",
                page, page_address(page), mapping, page_to_phys(page), phys_to_page(mapping), phys_to_virt(mapping));  // virt_to_page
        */
        rtl8125_map_and_mark_to_asic(tp, desc, mapping);
        return 0;
}

static void
rtl8125_rx_clear_ring(const struct rtl8125_private *const tp, struct rtl8125_rx_ring * const ring)
{
        struct page *page;

        printk(KERN_INFO "r8125 rtl8125_rx_clear_ring %d\n", (u8)(ring - tp->rx_ring));
        for (unsigned int i = 0; i < NUM_RX_DESC; i++) {
                struct RxDesc* desc = rtl8125_get_rxdesc(tp, ring->RxDescArray, i);
                //struct RxDesc* desc = rtl8125_get_rxdesc2(tp, ring_index, i);

#ifndef ENABLE_LARGE_PAGES
                if (!ring->RxBufferDmaPhyAddr[i])
                        continue;
                page = phys_to_page(ring->RxBufferDmaPhyAddr[i]);
                dma_unmap_page_attrs(tp->device, le64_to_cpu(page), RX_BUF_SIZE,
                        DMA_FROM_DEVICE, (DMA_ATTR_SKIP_CPU_SYNC | DMA_ATTR_WEAK_ORDERING));
                __free_pages(page, get_order(RX_BUF_SIZE));
                ring->RxBufferDmaPhyAddr[i] = 0;
#endif
                rtl8125_make_unusable_by_asic(tp, desc);
        }
#ifdef ENABLE_LARGE_PAGES
        page = phys_to_page(ring->RxBufferPhyAddr);
        dma_unmap_page_attrs(tp->device, le64_to_cpu(page), RX_BUFFER_ALLOC_SIZE(tp),
                        DMA_FROM_DEVICE, (DMA_ATTR_SKIP_CPU_SYNC | DMA_ATTR_WEAK_ORDERING));
        __free_pages(page, get_order(RX_BUFFER_ALLOC_SIZE(tp)));
#endif
}

static u32
rtl8125_rx_fill(struct rtl8125_private * const tp, const u8 ring_index)
{
        struct rtl8125_rx_ring * const ring = tp->rx_ring + ring_index;
#ifdef ENABLE_LARGE_PAGES
        rtl8125_alloc_rx_buffer(tp, ring);
#endif

        for (u32 i = 0; i < NUM_RX_DESC; i++) {
                if (rtl8125_alloc_rx_buffer_page(tp, ring_index, i)) {
                        rtl8125_rx_clear_ring(tp, ring);
                        return -ENOMEM;
                }
        }
        return 0;
}

static void
rtl8125_rx_clear_rings(struct rtl8125_private * const tp)
{
        for (u8 i = 0; i < tp->num_rx_rings; i++) {
                struct rtl8125_rx_ring *ring = tp->rx_ring + i;
                rtl8125_rx_clear_ring(tp, ring);
        }
}

static __always_inline void
rtl8125_mark_as_last_desc(const struct rtl8125_private *const tp, struct RxDesc *const desc)
{
#if defined(ENABLE_RSS_SUPPORT) || defined(ENABLE_PTP_SUPPORT)
        if (tp->RxDescType == RX_DESC_RING_TYPE_3)
                ((struct RxDescV3 *)desc)->RxDescNormalDDWord4.opts1 |= cpu_to_le32(RingEnd);
        else
#endif
#ifdef ENABLE_RSS_SUPPORT
        if (likely(tp->RxDescType == RX_DESC_RING_TYPE_4))
                ((struct RxDescV4 *)desc)->RxDescNormalDDWord2.opts1 |= cpu_to_le32(RingEnd);
        else
#endif
        desc->opts1 |= cpu_to_le32(RingEnd);
}

static void
rtl8125_desc_addr_fill(const struct rtl8125_private *const tp)
{
        int i;
        dma_addr_t phy_addr = tp->DescArrayPhyAddr;
        /* Transmit Descriptor Start Address */
        static const u16 tdsar_reg[] = { TDSAR_Q0_LOW_8125, TDSAR_Q1_LOW_8125 };
        static const u16 rdsar_reg[] = { RDSAR_Q0_LOW_8125, RDSAR_Q1_LOW_8125,
                                       RDSAR_Q2_LOW_8125, RDSAR_Q3_LOW_8125 };
        // tx
        for (i = 0; i < tp->num_tx_rings; i++, phy_addr += TX_DESC_ALLOC_SIZE) {
                RTL_W32(tp, tdsar_reg[i], phy_addr & DMA_BIT_MASK(32)); //((u64)ring->TxPhyAddr & DMA_BIT_MASK(32)));
                RTL_W32(tp, tdsar_reg[i] + 4, phy_addr >> 32); //((u64)ring->TxPhyAddr >> 32));
        }
        // rx
        for (i = 0; i < tp->num_rx_rings; i++, phy_addr += RX_DESC_ALLOC_SIZE(tp)) {
                RTL_W32(tp, rdsar_reg[i], phy_addr & DMA_BIT_MASK(32));
                RTL_W32(tp, rdsar_reg[i] + 4, phy_addr >> 32);
        }
}

static void
rtl8125_tx_desc_init(struct rtl8125_private *const tp)
{
        for (int i = 0; i < tp->num_tx_rings; i++) {
                struct rtl8125_tx_ring *ring = tp->tx_ring + i;
                memset(ring->TxDescArray, 0x0, TX_DESC_ALLOC_SIZE); //ring->TxDescAllocSize);
                ring->TxDescArray[NUM_TX_DESC - 1].opts1 = cpu_to_le32(RingEnd);
        }
}

static void
rtl8125_rx_desc_init(struct rtl8125_private *const tp)
{
        unsigned int RxDescAllocSize = RX_DESC_ALLOC_SIZE(tp);
        for (int i = 0; i < tp->num_rx_rings; i++) {
                struct rtl8125_rx_ring *ring = tp->rx_ring + i;
                memset(ring->RxDescArray, 0x0, RxDescAllocSize); //ring->RxDescAllocSize
        }
}

static int rtl8125_init_rings(struct rtl8125_private *const tp)
{
        int i;
        rtl8125_init_ring_indexes(tp);
        rtl8125_tx_desc_init(tp);
        rtl8125_rx_desc_init(tp);

        for (i = 0; i < tp->num_tx_rings; i++) {
                struct rtl8125_tx_ring *const ring = tp->tx_ring + i;
                memset(ring->tx_skb, 0x0, sizeof(ring->tx_skb));
        }

        for (i = 0; i < tp->num_rx_rings; i++) {
                struct rtl8125_rx_ring *ring = tp->rx_ring + i;
#ifndef ENABLE_LARGE_PAGES
                      memset(ring->RxBufferDmaPhyAddr, 0, sizeof(ring->RxBufferDmaPhyAddr));
#endif
                if (rtl8125_rx_fill(tp, i))
                        return -ENOMEM;
                /* mark as last descriptor in the ring */
                rtl8125_mark_as_last_desc(tp, rtl8125_get_rxdesc(tp, ring->RxDescArray, NUM_RX_DESC-1));
        }
        return 0;
}

static __always_inline void
rtl8125_unmap_tx_skb(struct device *const device, struct ring_info *const tx_skb, struct TxDesc * const desc)
{
        dma_unmap_single(device, le64_to_cpu(desc->addr), tx_skb->len, DMA_TO_DEVICE);
        memset(desc, 0, sizeof(*desc));
        tx_skb->len = 0;
}

static void
rtl8125_tx_clear_range(struct rtl8125_private *const tp,
        struct rtl8125_tx_ring * const ring, const u32 start, const unsigned int n)
{
        struct net_device *const dev = tp->dev;

        for (u32 i = 0; i < n; i++) {
                u16 entry = (i + start) % NUM_TX_DESC;
                struct TxDesc *desc = ring->TxDescArray + entry;
                struct ring_info *tx_skb = ring->tx_skb + entry;

                if (likely(tx_skb->len)) {
                        struct sk_buff *skb = tx_skb->skb;
                        rtl8125_unmap_tx_skb(tp->device, tx_skb, desc);
                        if (skb) {
                                dev->stats.tx_dropped++;
                                dev_kfree_skb_any(skb);
                                tx_skb->skb = NULL;
                        }
                }
        }
}

static void
rtl8125_tx_clear_rings(struct rtl8125_private * const tp)
{
        for (int i = 0; i < tp->num_tx_rings; i++) {
                struct rtl8125_tx_ring * const ring = tp->tx_ring + i;
                rtl8125_tx_clear_range(tp, ring, ring->dirty_tx, NUM_TX_DESC);
                netdev_tx_reset_queue(netdev_get_tx_queue(tp->dev, i));
                ring->cur_tx = ring->dirty_tx = 0;
        }
}

static void rtl8125_schedule_reset_work(struct rtl8125_private * const tp)
{
        set_bit(R8125_FLAG_TASK_RESET_PENDING, tp->task_flags);
        schedule_delayed_work(&tp->reset_task, 4);
}

static void rtl8125_cancel_schedule_reset_work(struct rtl8125_private * const tp)
{
        struct work_struct *work = &tp->reset_task.work;

        if (!work->func)
                return;
        cancel_delayed_work_sync(&tp->reset_task);
}

#ifdef ENABLE_ESD
static void rtl8125_schedule_esd_work(struct rtl8125_private *const tp)
{
        set_bit(R8125_FLAG_TASK_ESD_CHECK_PENDING, tp->task_flags);
        schedule_delayed_work(&tp->esd_task, RTL8125_ESD_TIMEOUT);
}

static void rtl8125_cancel_schedule_esd_work(struct rtl8125_private *tp)
{
        struct work_struct *work = &tp->esd_task.work;

        if (!work->func)
                return;

        cancel_delayed_work_sync(&tp->esd_task);
}
#endif

static void rtl8125_cancel_schedule_linkchg_work(struct rtl8125_private *const tp)
{
        struct work_struct *work = &tp->linkchg_task.work;

        if (!work->func)
                return;

        cancel_delayed_work_sync(&tp->linkchg_task);
}

static inline void rtl8125_schedule_linkchg_work(struct rtl8125_private *const tp)
{
        set_bit(R8125_FLAG_TASK_LINKCHG_CHECK_PENDING, tp->task_flags);
        schedule_delayed_work(&tp->linkchg_task, 4);
}

static void rtl8125_cancel_all_schedule_work(struct rtl8125_private * const tp)
{
        rtl8125_cancel_schedule_reset_work(tp);
#ifdef ENABLE_ESD
        rtl8125_cancel_schedule_esd_work(tp);
#endif
        rtl8125_cancel_schedule_linkchg_work(tp);
}

#ifdef ENABLE_ESD
static void rtl8125_esd_task(struct work_struct *work)
{
        struct rtl8125_private *tp =
                container_of(work, struct rtl8125_private, esd_task.work);
        struct net_device *dev = tp->dev;
        unsigned long flags;
        rtnl_lock();

        if (!netif_running(dev) ||
            test_bit(R8125_FLAG_DOWN, tp->task_flags) ||
            !test_and_clear_bit(R8125_FLAG_TASK_ESD_CHECK_PENDING, tp->task_flags))
                goto out_unlock;

        rtl8125_esd_checker(tp, dev);
        rtl8125_schedule_esd_work(tp);

out_unlock:
        rtnl_unlock();
}
#endif

static void rtl8125_linkchg_task(struct work_struct *const work)
{
        struct rtl8125_private *const tp =
                container_of(work, struct rtl8125_private, linkchg_task.work);
        struct net_device *const dev = tp->dev;
        rtnl_lock();

        if (!netif_running(dev) ||
            test_bit(R8125_FLAG_DOWN, tp->task_flags) ||
            !test_and_clear_bit(R8125_FLAG_TASK_LINKCHG_CHECK_PENDING, tp->task_flags))
                goto out_unlock;

        _rtl8125_check_link_status(tp, dev);

out_unlock:
        rtnl_unlock();
}

static void rtl8125_init_all_schedule_work(struct rtl8125_private *const tp)
{
        INIT_DELAYED_WORK(&tp->reset_task, rtl8125_reset_task);
#ifdef ENABLE_ESD
        INIT_DELAYED_WORK(&tp->esd_task, rtl8125_esd_task);
#endif
        INIT_DELAYED_WORK(&tp->linkchg_task, rtl8125_linkchg_task);
}

static void
rtl8125_wait_for_irq_complete(struct rtl8125_private *const tp)
{
        if (tp->irq_nvecs > 1)  // implies RTL_MSIX
                for (int i = 0; i < tp->irq_nvecs; i++)
                        synchronize_irq(tp->irq[i]);
                        //synchronize_irq(tp->irq_tbl[i].vector);
        else
                synchronize_irq(tp->dev->irq);
}

static void rtl8125_wait_for_quiescence(struct rtl8125_private *const tp)
{
        /* Wait for any pending NAPI task to complete */
        rtl8125_disable_napi(tp);

        /* Give a racing hard_start_xmit a few cycles to complete. */
        synchronize_net();
        rtl8125_irq_mask_and_ack(tp);
        rtl8125_wait_for_irq_complete(tp);
}

static void rtl8125_reset_task(struct work_struct *const work)
{
        struct rtl8125_private *tp = container_of(work, struct rtl8125_private, reset_task.work);
        struct net_device *dev = tp->dev;

        rtnl_lock();
        if (!netif_running(dev) ||
            test_bit(R8125_FLAG_DOWN, tp->task_flags) ||
            !test_and_clear_bit(R8125_FLAG_TASK_RESET_PENDING, tp->task_flags))
                goto out_unlock;

        netdev_err(dev, "Device resetting!\n");

        netif_carrier_off(dev);
        netif_tx_disable(dev);
        rtl8125_wait_for_quiescence(tp);
        rtl8125_hw_reset(tp);
        rtl8125_tx_clear_rings(tp);
        rtl8125_init_ring_indexes(tp);
        rtl8125_tx_desc_init(tp);
        for (int i = 0; i < tp->num_rx_rings; i++) {
                struct rtl8125_rx_ring *ring = &tp->rx_ring[i];
                for (u16 entry = 0; entry < NUM_RX_DESC; entry++) {
                        struct RxDesc *desc = rtl8125_get_rxdesc(tp, ring->RxDescArray, entry);
                        rtl8125_mark_to_asic(tp, desc);
                }
        }

#ifdef ENABLE_PTP_SUPPORT
        rtl8125_ptp_reset(tp, dev);
#endif

        rtl8125_enable_napi(tp);

        if (rtl8125_flag_is_set(tp, ResumeNoSpeedChange))
                rtl8125_check_link_status(dev);
        else {
                rtl8125_enable_hw_linkchg_interrupt(tp);
                rtl8125_set_speed(tp, rtl8125_flag_to_bool(tp, AutoNegMode),
                        tp->speed, tp->duplex, tp->advertising);
        }

out_unlock:
        rtnl_unlock();
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
static void
rtl8125_tx_timeout(struct net_device *dev, unsigned int txqueue)
#else
static void
rtl8125_tx_timeout(struct net_device *dev)
#endif
{
        struct rtl8125_private *tp = netdev_priv(dev);

        netdev_err(dev, "Transmit timeout reset Device!\n");

        /* Let's wait a bit while any (async) irq lands on */
        rtl8125_schedule_reset_work(tp);
}

/*
static inline u32 rtl8125_get_txd_opts1(struct rtl8125_tx_ring *ring, u32 opts1, u32 len, u16 entry)
{
        u32 status = opts1 | len;
        if (entry == NUM_TX_DESC - 1)
                status |= RingEnd;
        return status;
}
*/

static __always_inline int
rtl8125_xmit_frags(const struct rtl8125_private *tp, struct rtl8125_tx_ring * const ring, const u8 frags,
        const skb_frag_t *skb_frags, const u32 *opts, u16 entry)
{
        struct device *device = tp->device;
        struct TxDesc * const txdBase = ring->TxDescArray;
        struct TxDesc *txd;
        //bool lastFrag = false, lsoPatch = false;
        u8 frag = 0;

        //for (; frag < frags; frag++) {
        do {
                const skb_frag_t *skb_frag = skb_frags + frag;
                u32 len = skb_frag_size(skb_frag);
                void *addr = skb_frag_address(skb_frag);
                dma_addr_t mapping = dma_map_single(device, addr, len, DMA_TO_DEVICE);
                if (unlikely(dma_mapping_error(device, mapping)))
                        return frag;

                /* LSO Patch
                bool lastFrag = (frag == (frags - 1));

                // Is LSO (Large Send Offload) patch required?
                if (unlikely(is_8125A(tp) && lastFrag &&
                    (opts[0] & (GiantSendv4|GiantSendv6)) && pktLen < ETH_FRAME_LEN && len > 1)) {
                        len--;
                        lastFrag = false;
                        lsoPatch = true;
                }
map_fragment:
                */

                entry = (entry + 1) % NUM_TX_DESC;

                /* TX map */
                txd = txdBase + entry;
                txd->addr = cpu_to_le64(mapping);
                txd->opts2 = cpu_to_le32(opts[1]);

                ring->tx_skb[entry].len = len;
                //dma_wmb(); Only required for last frag ??
                txd->opts1 = cpu_to_le32(opts[0] | len | (unlikely (entry == NUM_TX_DESC - 1) ? RingEnd : 0));
                /* end of TX map */
                /*
                if (unlikely(lsoPatch)) { // Second txd which will now be the last fragment
                        addr += len;
                        len = 1;
                        frag++;
                        lsoPatch = false;
                        goto map_fragment;
                }
                pktLen += len;
                */
        } while (++frag < frags);
        //}
        txd->opts1 |= cpu_to_le32(LastFrag);
        return 0;
/*
err_out:
        rtl8125_tx_clear_range(tp, ring, ring->cur_tx + 1, frag);
        return -EIO;
*/
}

/*
static __always_inline bool rtl8125_require_pad_ptp_pkt(struct rtl8125_private *tp)
{
        //return (tp->mcfg <= CFG_METHOD_7) ? true : false;
        return is_8125AB(tp);
}
*/

static inline bool rtl8125_skb_is_udp(struct sk_buff * const skb)
{
        int no = skb_network_offset(skb);
        struct ipv6hdr *i6h, _i6h;
        struct iphdr *ih, _ih;

        switch (vlan_get_protocol(skb)) {
        case htons(ETH_P_IP):
                ih = skb_header_pointer(skb, no, sizeof(_ih), &_ih);
                return ih && ih->protocol == IPPROTO_UDP;
        case htons(ETH_P_IPV6):
                i6h = skb_header_pointer(skb, no, sizeof(_i6h), &_i6h);
                return i6h && i6h->nexthdr == IPPROTO_UDP;
        default:
                return false;
        }
}

/* Patch to work around HW defect on (certain) 8125 chips: break under heavy UDP laod */
#define MIN_PATCH_LEN (47)
static unsigned int
rtl8125_udp_padto(struct sk_buff * const skb, const u32 len)
{
        unsigned int padto = 0; //, len = skb->len;
        const unsigned int trans_data_len = skb_tail_pointer(skb) - skb_transport_header(skb);

        if (trans_data_len >= offsetof(struct udphdr, len) &&
                trans_data_len < MIN_PATCH_LEN) {
                u16 dest = ntohs(udp_hdr(skb)->dest);

                /* dest is a standard PTP port */
                if (dest == 319 || dest == 320)
                        padto = len + MIN_PATCH_LEN - trans_data_len;
        }

        if (trans_data_len < sizeof(struct udphdr))
                padto = max_t(unsigned int, padto, (len + sizeof(struct udphdr) - trans_data_len));
        return max_t(unsigned int, padto, ETH_ZLEN);
}

#define skb_transport_offset(skb) (skb->head + skb->transport_header - skb->data)

static __always_inline bool
rtl8125_tso_csum_v2(const struct rtl8125_private *tp, struct sk_buff * const skb,
        const struct skb_shared_info *shinfo, u32 * const opts)
{
        u32 mss = shinfo->gso_size, len;

        if (mss) {
                if (shinfo->gso_type & SKB_GSO_TCPV4)
                        opts[0] |= GiantSendv4;
                else if (shinfo->gso_type & SKB_GSO_TCPV6) {
                        if (skb_cow_head(skb, 0))
                                return false;

                        tcp_v6_gso_csum_prep(skb);
                        opts[0] |= GiantSendv6;
                } else
                        WARN_ON_ONCE(1);

                opts[0] |= (skb_transport_offset(skb) << GTTCPHO_SHIFT);
                opts[1] |= (mss & MSS_MAX) << TD1_MSS_SHIFT; //min(mss, MSS_MAX) << 18;
                return 0;

        }
        if (skb->ip_summed == CHECKSUM_PARTIAL) {
                u8 ip_protocol;

                switch (vlan_get_protocol(skb)) {
                case htons(ETH_P_IP):
                        opts[1] |= TxIPCS_C;
                        ip_protocol = ip_hdr(skb)->protocol;
                        break;

                case htons(ETH_P_IPV6):
                        opts[1] |= TxIPV6F_C;
                        ip_protocol = ipv6_hdr(skb)->nexthdr;
                        break;

                default:
                        ip_protocol = IPPROTO_RAW;
                        break;
                }

                if (likely(ip_protocol == IPPROTO_TCP))
                        opts[1] |= TxTCPCS_C;
                else if (ip_protocol == IPPROTO_UDP)
                        opts[1] |= TxUDPCS_C;
                else
                        WARN_ON_ONCE(1);

                opts[1] |= (skb_transport_offset(skb) << TCPHO_SHIFT);
                return 0;
        }
        /* skb_padto would free the skb on error */
        len = skb->len;
        if (unlikely(len < 128 + MIN_PATCH_LEN && //is_8125AB(tp) &&
                rtl8125_skb_is_udp(skb) && skb_transport_header_was_set(skb)))
                return __skb_put_padto(skb, rtl8125_udp_padto(skb, len), true);
        return 0;
}

#define rtl8125_free_slots(dtx,ctx) NUM_TX_DESC + dtx - ctx
#define rtl8125_out_of_tx_slots(dtx,ctx) (rtl8125_free_slots(dtx,ctx) <= MAX_SKB_FRAGS)
#define rtl8125_tx_slots_avail(dtx,ctx) (rtl8125_free_slots(dtx,ctx) > MAX_SKB_FRAGS)
/*
static __always_inline bool rtl8125_tx_slots_avail(struct rtl8125_tx_ring const *ring)
{
        // A skbuff with nr_frags needs nr_frags+1 entries in the tx queue
        return (READ_ONCE(ring->dirty_tx) + NUM_TX_DESC - READ_ONCE(ring->cur_tx) > MAX_SKB_FRAGS);
}
*/

static __always_inline void
#ifdef ENABLE_TX_NO_CLOSE
rtl8125_doorbell(struct rtl8125_private const *tp, const u8 ring_index, const struct rtl8125_tx_ring *ring)
{
        //if (tp->mcfg <= CFG_METHOD_7) // tp->HwSuppTxNoCloseVer == 3
        if (is_8125AB(tp)) // tp->HwSuppTxNoCloseVer == 3
                RTL_W16(tp, sw_tail_ptr_reg_v3(ring_index), READ_ONCE(ring->cur_tx));
        else
                RTL_W32(tp, sw_tail_ptr_reg_v6(ring_index), READ_ONCE(ring->cur_tx));
#else
rtl8125_doorbell(struct rtl8125_private *tp, const u8 ring_index)
        RTL_W16(tp, TPPOLL_8125, BIT(ring_index));    /* set polling bit */
#endif
}

static netdev_tx_t
rtl8125_start_xmit(struct sk_buff * const skb, struct net_device * const dev)
{
        struct rtl8125_private * const tp = netdev_priv(dev);
        struct device* const device = tp->device;
        const u8 ring_index = skb->queue_mapping;
        struct rtl8125_tx_ring * const ring = tp->tx_ring + ring_index;
        struct netdev_queue * const txq = netdev_get_tx_queue(dev, ring_index);
        struct skb_shared_info * const info = skb_shinfo(skb);
        const u32 len = skb_headlen(skb);
        u8 frags;
        dma_addr_t mapping;
        bool stop_queue;
        u32 cur_tx = ring->cur_tx;
        u16 entry = cur_tx % NUM_TX_DESC;
        struct TxDesc * const txd = ring->TxDescArray + entry; // First tx descriptor
        u32 opts[2] = { DescOwn, rtl8125_tx_vlan_tag(tp, skb) };
        u32 opts1 = FirstFrag | len | (unlikely(entry == NUM_TX_DESC - 1) ? RingEnd : 0);
        //static int count = 0;
        //if (entry == NUM_TX_DESC - 1) opts0 |= RingEnd;

        //if (unlikely(!rtl8125_tx_slots_avail(ring))) {
        if (unlikely(rtl8125_out_of_tx_slots(READ_ONCE(ring->dirty_tx), cur_tx))) {
                if (netif_msg_drv(tp)) {
                        printk(KERN_ERR "%s: BUG! Tx Ring[%d] full when queue awake!\n",
                               dev->name, ring_index);
                }
                goto err_stop;
        }

#ifndef ENABLE_TX_NO_CLOSE
        if (unlikely(le32_to_cpu(txd->opts1) & DescOwn)) {
                if (netif_msg_drv(tp))
                        printk(KERN_ERR "%s: BUG! Tx Desc owned by HW!\n", dev->name);
                goto err_stop;
        }
#endif

        if (unlikely(rtl8125_tso_csum_v2(tp, skb, info, opts))) // csum modifies opts !
                goto err_dma;

        // Map tx
        mapping = dma_map_single(device, skb->data, len, DMA_TO_DEVICE);
        if (unlikely(dma_mapping_error(device, mapping))) {
                if (unlikely(net_ratelimit()))
                        netif_err(tp, drv, dev, "Failed to map TX DMA\n");
                goto err_dma;
        }
        ring->tx_skb[entry].len = len;  // set first fragment's length

        // delay reading of frags as it may have been changed by rtl8125_tso_csum_v2
        frags = info->nr_frags;
        if (frags) {
                const u8 error_frags =
                        rtl8125_xmit_frags(tp, ring, frags, info->frags, opts, entry);
                if (unlikely(error_frags)) {
                        if (unlikely(net_ratelimit()))
                                netif_err(tp, drv, tp->dev, "DMA map of TX fragments failed\n");
                        rtl8125_tx_clear_range(tp, ring, ring->cur_tx + 1, error_frags);
                        goto err_dma;
                }
                entry = (entry + frags) % NUM_TX_DESC;
        } else { // First fragment is also last
                opts1 |= LastFrag;
        }

#ifdef ENABLE_PTP_SUPPORT
        // SKBTX_HW_TSTAMP if driver is expected to do HW time stamping
        if (unlikely(info->tx_flags & SKBTX_HW_TSTAMP)) {
        //if (unlikely(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) {
		//unsigned long flags;
                netif_info(tp, drv, tp->dev, "PTP: start TX:%d\n", ring_index);
		//spin_lock_irqsave(&tp->ptp_tx_lock, flags);
                if (!test_and_set_bit_lock(__RTL8125_PTP_TX_IN_PROGRESS, &tp->ptp_state)) {
                        if (likely(tp->hwtstamp_config.tx_type == HWTSTAMP_TX_ON && !tp->ptp_tx_skb)) {
                                tp->ptp_tx_skb = skb_get(skb);
                                tp->ptp_tx_start = jiffies;
                                info->tx_flags |= SKBTX_IN_PROGRESS;
                                schedule_work(&tp->ptp_tx_work);
                        } else
                                tp->tx_hwtstamp_skipped++;
                } else
                        netif_info(tp, drv, tp->dev, "PTP: start xmit -> unable to set lock\n");
        	//spin_unlock_irqrestore(&tp->ptp_tx_lock, flags);
        }
#endif
        //skb_tx_timestamp(skb);

        txd->addr = cpu_to_le64(mapping);
        txd->opts2 = cpu_to_le32(opts[1]);
        // set skb to last fragment
        ring->tx_skb[entry].skb = skb;

        dma_wmb();
        txd->opts1 = cpu_to_le32(opts[0] | opts1);  // Finally map first desc

        //door_bell = __netdev_tx_sent_queue(txq, skb->len, netdev_xmit_more());
        netdev_tx_sent_queue(txq, skb->len);
        skb_tx_timestamp(skb);

        /* rtl8125_tx_interrupt needs to see descriptor changes before updating ring->cur_tx */
        smp_wmb();
        cur_tx += frags + 1;
        //WRITE_ONCE(ring->cur_tx, ring->cur_tx + frags + 1);
        WRITE_ONCE(ring->cur_tx, cur_tx);

        //stop_queue = !rtl8125_tx_slots_avail(ring);
        stop_queue = rtl8125_out_of_tx_slots(READ_ONCE(ring->dirty_tx), cur_tx);

        if (unlikely(stop_queue)) {
                /* Avoid wrongly optimistic queue wake-up: rtl_tx thread must
                 * not miss a ring update when it notices a stopped queue. */
                //printk(KERN_INFO "r8125 TXQ stop 1\n");
                smp_wmb();
                netif_tx_stop_queue(txq);
        }

        if (netif_xmit_stopped(txq) || !netdev_xmit_more())
#ifdef ENABLE_TX_NO_CLOSE
                rtl8125_doorbell(tp, ring_index, ring);
#else
                rtl8125_doorbell(tp, ring_index);
#endif
        if (unlikely(stop_queue)) {
                // Sync with rtl8125_tx_interrupt:
                // - publish queue status and cur_tx ring index (write barrier)
                // - refresh dirty_tx ring index (read barrier).
                // May the current thread have a pessimistic view of the ring
                // status and forget to wake up queue, a racing rtl_tx thread can't.
                //printk(KERN_INFO "r8125 TXQ stop 2\n");
                smp_mb();
                if (rtl8125_tx_slots_avail(READ_ONCE(ring->dirty_tx), cur_tx))
                        //netif_tx_start_queue(txq);
                        netif_tx_wake_queue(txq);
        }
out:
        return NETDEV_TX_OK;
err_dma:
        dev->stats.tx_dropped++;
        dev_kfree_skb_any(skb);
        goto out;
err_stop:
        netif_tx_stop_queue(txq);
        dev->stats.tx_dropped++;
        return NETDEV_TX_BUSY;
}

static __always_inline u32
rtl8125_update_hw_clo_ptr(const struct rtl8125_private *tp, struct rtl8125_tx_ring * const ring, const u8 ring_index)
{
        u32 tx_count, hwDescCloPtr;
        //if (tp->mcfg <= CFG_METHOD_7) { // tp->HwSuppTxNoCloseVer == 3
        if (is_8125AB(tp)) { // tp->HwSuppTxNoCloseVer == 3
                hwDescCloPtr = RTL_R16(tp, hw_clo_ptr_reg_v3(ring_index));
                tx_count = (hwDescCloPtr - ring->BeginHwDesCloPtr) & MAX_TX_NO_CLOSE_DESC_PTR_MASK_V2;
        } else {
                hwDescCloPtr = RTL_R32(tp, hw_clo_ptr_reg_v6(ring_index));
                tx_count = (hwDescCloPtr - ring->BeginHwDesCloPtr) & MAX_TX_NO_CLOSE_DESC_PTR_MASK_V4;
        }
        ring->BeginHwDesCloPtr = hwDescCloPtr;
        return tx_count;
}

static void
rtl8125_tx_interrupt(struct rtl8125_private * const tp, const u8 ring_index, const int budget)
{
        struct net_device * const dev = tp->dev;
        struct rtl8125_tx_ring * const ring = tp->tx_ring + ring_index;
        struct netdev_queue * const txq = netdev_get_tx_queue(dev, ring_index);
        struct sk_buff *skb;
        struct ring_info *tx_skb;
        unsigned int tx_bytes = 0, tx_packets = 0;
        u32 dirty_tx;
        const u32 begin_dirty_tx = dirty_tx = ring->dirty_tx;


#ifdef ENABLE_TX_NO_CLOSE
        /* recycle tx no close desc*/
        u32 tx_count = rtl8125_update_hw_clo_ptr(tp, ring, ring_index);
        //static int count0=0, count1=0, count2=0;
        //count0++;

        while (tx_count-- > 0) {
                u16 entry = dirty_tx++ % NUM_TX_DESC;
                tx_skb = ring->tx_skb + entry;
                rtl8125_unmap_tx_skb(tp->device, tx_skb, ring->TxDescArray + entry);

#else // not ENABLE_TX_NO_CLOSE - recycle tx close desc
        while (READ_ONCE(ring->cur_tx) != dirty_tx) {
                u16 entry = dirty_tx % NUM_TX_DESC;
                struct TxDesc *desc = ring->TxDescArray + entry;
                tx_skb = ring->tx_skb + entry;
                if (le32_to_cpu(READ_ONCE(desc->opts1)) & DescOwn)
                        break;
                rtl8125_unmap_tx_skb(tp->device, tx_skb, desc);
                dirty_tx++;
#endif
                skb = tx_skb->skb;
                if (skb) {      // update the statistics for this packet
                        tx_bytes += skb->len;
                        tx_packets++;
                        napi_consume_skb(skb, budget);
                        tx_skb->skb = NULL;
                }
        }

        if (likely(begin_dirty_tx != dirty_tx)) {
                netdev_tx_completed_queue(txq, tx_packets, tx_bytes);
                dev->stats.tx_bytes += tx_bytes;
                dev->stats.tx_packets+= tx_packets;
                WRITE_ONCE(ring->dirty_tx, dirty_tx);
                smp_wmb();
                if (unlikely(netif_tx_queue_stopped(txq) && rtl8125_tx_slots_avail(dirty_tx, READ_ONCE(ring->cur_tx))))
                        netif_tx_start_queue(txq);
#ifndef ENABLE_TX_NO_CLOSE
                if (ring->cur_tx != dirty_tx && skb)
                        rtl8125_doorbell(tp, ring_index);
#endif
        }
}

static __always_inline int
rtl8125_fragmented_frame(const struct rtl8125_private *tp, const u32 status)
{
#if defined(ENABLE_RSS_SUPPORT) || defined(ENABLE_PTP_SUPPORT)
        if (tp->RxDescType == RX_DESC_RING_TYPE_3)
                return (status & (FirstFrag_V3 | LastFrag_V3)) != (FirstFrag_V3 | LastFrag_V3);
#endif
        // Last/FirstFlag_V4 are equal to First/LastFrag
        return (status & (FirstFrag | LastFrag)) != (FirstFrag | LastFrag);
}

/* Check for End Of Packet */
static __always_inline int
rtl8125_is_eop(const struct rtl8125_private *tp, const u32 status)
{
#if defined(ENABLE_RSS_SUPPORT) || defined(ENABLE_PTP_SUPPORT)
        if (tp->RxDescType == RX_DESC_RING_TYPE_3)
                return (status & LastFrag_V3);
#endif
        return (status & LastFrag);
}

static __always_inline int
rtl8125_rx_desc_type(u32 status)
{
        return ((status >> 26) & 0x0F);
}

static __always_inline bool rtl8125_rx_csum(const u8 rxDescType, u32 status)
{
#if defined(ENABLE_RSS_SUPPORT) || defined(ENABLE_PTP_SUPPORT)
        if (likely(rxDescType == RX_DESC_RING_TYPE_3)) {
                status &= (RxProtoMask_v3 | RxCSFailMask_v3);
                return (status == RxProtoTCP_v3 || status == RxProtoUDP_v3);
        }
#endif
#ifdef ENABLE_RSS_SUPPORT
        if (likely(rxDescType == RX_DESC_RING_TYPE_4)) {
                status &= (RxProtoMask_v4 | RxCSFailMask_v4);
                return (status == RxProtoTCP_v4 || status == RxProtoUDP_v4);
        }
#endif
        status &= (RxProtoMask | RxCSFailMask);
        return (status == RxProtoTCP || status == RxProtoUDP);
}

static __always_inline int
rtl8125_check_rx_desc_error(struct net_device * const dev, const struct rtl8125_private *tp, const u32 status)
{
#if defined(ENABLE_RSS_SUPPORT) || defined(ENABLE_PTP_SUPPORT)
        if (tp->RxDescType == RX_DESC_RING_TYPE_3) {
                if (unlikely(status & RxRES_V3)) {
                        if (status & (RxRWT_V3 | RxRUNT_V3))
                                dev->stats.rx_length_errors++;
                        if (status & RxCRC_V3)
                                dev->stats.rx_crc_errors++;
                        return -1;
                }
                return 0;
        }
#endif
#ifdef ENABLE_RSS_SUPPORT
        if (likely(tp->RxDescType == RX_DESC_RING_TYPE_4)) {
                if (unlikely(status & RxRES_V4)) {
                        if (status & RxRUNT_V4)
                                dev->stats.rx_length_errors++;
                        if (status & RxCRC_V4)
                                dev->stats.rx_crc_errors++;
                        return -1;
                }
                return 0;
        }
#endif
        if (unlikely(status & RxRES)) {
                if (status & (RxRWT | RxRUNT))
                        dev->stats.rx_length_errors++;
                if (status & RxCRC)
                        dev->stats.rx_crc_errors++;
                //return (status & RxRWT || !(status & (RxRUNT | RxCRC))) ? -2 : -1;
                return -1;
        }
        return 0;
}

static int
rtl8125_rx_interrupt(struct rtl8125_private * const tp, const u8 ring_index, int budget)
{
        struct rtl8125_rx_ring * const ring = &tp->rx_ring[ring_index];
        struct net_device * const dev = tp->dev;
        struct device * const device = tp->device;
        u64 rx_buf_phy_addr;
        u32 rx_bytes = 0, rx_packets = 0, pkt_size;
        u16 entry = ring->cur_rx % NUM_RX_DESC;

        do {
#ifdef ENABLE_PTP_SUPPORT
                u8 desc_type = RXDESC_TYPE_NORMAL;
                struct RxDescV3 ptp_desc, *desc_next;
#endif //ENABLE_PTP_SUPPORT
                const void *rx_buf;
                struct RxDesc *desc = rtl8125_get_rxdesc(tp, ring->RxDescArray, entry);
                struct sk_buff *skb;
                u32 i = 0, status;
                do { // This loop reduces system CPU time and can improve performance for busy networks
                        status = le32_to_cpu(rtl8125_rx_desc_opts1(tp, desc));
                        if (likely(!(status & DescOwn)))
                                goto desc_released;
                        RTL_R8(tp, IMR0_8125);
                        //udelay(i & 1);
                } while (i++ < 2); // Longer waits don't make sense
                if (unlikely(rx_packets == 0))
                        return 0;
                break;
desc_released:
                /* Barrier to prevent reading any other fields out of the Rx descriptor until
                 * the status of DescOwn is known */
                dma_rmb();

                // define early (required for release descriptor on V3/V4
/* For Desc V1 there is no need to save/reload the DMA address
   With Desc V3/v4 the descriptor address is zeroed after use  */
#if defined(ENABLE_RSS_SUPPORT) || defined(ENABLE_PTP_SUPPORT)
                rx_buf_phy_addr = GetRxBufferDmaPhyAddr(ring, entry);
#else
                rx_buf_phy_addr = le64_to_cpu(desc->addr);
#endif

                if (unlikely(rtl8125_check_rx_desc_error(dev, tp, status) < 0)) {
                        if (netif_msg_rx_err(tp))
                                netdev_warn(dev, "Rx ERROR. status = %08x\n", status);
                        dev->stats.rx_errors++;

                        if (!(dev->features & NETIF_F_RXALL))
                                goto release_descriptor;
                }
                pkt_size = status & 0x00003fff;
                if (likely(!(dev->features & NETIF_F_RXFCS))) {
#ifdef ENABLE_RX_PACKET_FRAGMENT
                        if (!rtl8125_is_eop(tp, status) && pkt_size == RX_BUF_SIZE) {
                                u16 entry_next;
                                int pkt_size_next;
                                u32 status_next;

                                entry_next = (cur_rx + 1) % NUM_RX_DESC;
                                desc_next = rtl8125_get_rxdesc(tp, ring->RxDescArray, entry_next);
                                status_next = le32_to_cpu(rtl8125_rx_desc_opts1(tp, desc_next));
                                if (!(status_next & DescOwn)) {
                                        pkt_size_next = status_next & 0x00003fff;
                                        if (pkt_size_next < ETH_FCS_LEN)
                                                pkt_size -= (ETH_FCS_LEN - pkt_size_next);
                                }
                        }
#endif //ENABLE_RX_PACKET_FRAGMENT
/*
                        if (rtl8125_is_eop(tp, status)) {
                                if (unlikely(pkt_size < ETH_FCS_LEN)) {
#ifdef ENABLE_RX_PACKET_FRAGMENT
                                        pkt_size = 0;
#else
                                        goto drop_packet;
#endif //ENABLE_RX_PACKET_FRAGMENT
                                } else
                                        pkt_size -= ETH_FCS_LEN;
                        }
*/
                        pkt_size -= ETH_FCS_LEN;
                }

                //if (unlikely(pkt_size > RX_BUF_SIZE))
                //        goto drop_packet;

#if !defined(ENABLE_RX_PACKET_FRAGMENT)
                /* The driver does not support incoming fragmented
                 * frames. They are seen as a symptom of over-mtu sized frames. */
                if (unlikely(rtl8125_fragmented_frame(tp, status))) {
                        dev->stats.rx_length_errors++;
                        goto drop_packet;
                }
#endif //!ENABLE_RX_PACKET_FRAGMENT || !ENABLE_PAGE_REUSE

#ifdef ENABLE_PTP_SUPPORT // Implies RxDescV3
                desc_type = rtl8125_rx_desc_type(status);
                if (desc_type == RXDESC_TYPE_NORMAL ||
                        unlikely(!rtl8125_flag_is_set(tp, EnablePtp)))
                        goto ptp_done;

                // PTP Slave
                if (likely(desc_type == RXDESC_TYPE_NEXT && rx_packets < budget)) {
                        i = 0;
                        entry = (entry + 1) % NUM_RX_DESC;
                        //printk(KERN_INFO "r8125: PTP entry next %d\n", entry);
                        budget--;
                        desc_next = (struct RxDescV3 *)ring->RxDescArray + entry; //_next;
                        do {
                                status = le32_to_cpu(desc_next->RxDescNormalDDWord4.opts1);
                                if (likely(!(status & DescOwn)))
                                        goto ptp_desc_released;
                                udelay(1);
                        } while (i++ <= 2);
                        if (netif_msg_rx_err(tp))
                               printk(KERN_ERR "%s: Rx Next Desc ERROR. status = %08x\n", dev->name, status);
                        goto ptp_map_and_mark_to_asic;
ptp_desc_released:
                        rmb();
                        desc_type = rtl8125_rx_desc_type(status); // _next
                        if (likely(desc_type == RXDESC_TYPE_PTP)) {
                                ptp_desc = *desc_next;
                                rmb();
                        } //else WARN_ON(1);
ptp_map_and_mark_to_asic:
                        rtl8125_map_and_mark_to_asic_v3(desc_next, GetRxBufferDmaPhyAddr(ring, entry)); //_next
                        if (unlikely(desc_type != RXDESC_TYPE_PTP)) {
                                if (desc_type != RXDESC_TYPE_NEXT)
                                        printk(KERN_ERR "r8125: PTP desc_type error %d\n", desc_type);
                                goto drop_packet;
                        }

                } else {
                        printk(KERN_ERR "r8125: wrong desc_type 0x%x\n", desc_type);
                        WARN_ON(desc_type != RXDESC_TYPE_NORMAL);
                }
ptp_done:
#endif
                skb = napi_alloc_skb(&tp->napi[ring_index], pkt_size);
                if (unlikely(!skb)) {
                        netdev_err(dev, "Failed to allocate RX skb!\n");
                        goto drop_packet;
                }

#ifdef ENABLE_PTP_SUPPORT
                if (desc_type == RXDESC_TYPE_PTP) {
                        unsigned long flags;
                        spin_lock_irqsave(&tp->phy_lock, flags);
                        rtl8125_rx_ptp_pktstamp(tp, skb, &ptp_desc);
                        spin_unlock_irqrestore(&tp->phy_lock, flags);
                }
#endif //ENABLE_PTP_SUPPORT

                dma_sync_single_for_cpu(device, rx_buf_phy_addr, pkt_size, DMA_FROM_DEVICE);
                rx_buf = phys_to_virt(rx_buf_phy_addr);
                prefetch(rx_buf);
                skb_copy_to_linear_data(skb, rx_buf, pkt_size);
                // skb_put
                skb->tail += pkt_size;
                skb->len = pkt_size;
                rx_bytes += pkt_size;
                // Give ownership back to device
                dma_sync_single_for_device(device, rx_buf_phy_addr, pkt_size, DMA_FROM_DEVICE);

#ifdef ENABLE_RX_PACKET_FRAGMENT
                if (!rtl8125_is_eop(tp, status)) {
                        u16 entry_next;
                        entry_next = (entry + 1) % NUM_RX_DESC
                        rxb = &ring->rx_buffer[entry_next];
                        rxb->skb = skb;
                        continue;
                }
#endif //ENABLE_RX_PACKET_FRAGMENT

#ifdef ENABLE_RSS_SUPPORT
                rtl8125_rx_hash(tp, desc, skb);
#endif
                if (likely(rtl8125_rx_csum(tp->RxDescType, status)))
                //if (likely(rtl8125_rx_csum(tp->RxDescType, desc)))
                        skb->ip_summed = CHECKSUM_UNNECESSARY;
                else
                        skb_checksum_none_assert(skb);
                skb->protocol = eth_type_trans(skb, dev);

                if (skb->pkt_type == PACKET_MULTICAST)
                        dev->stats.multicast++;

                //if (rtl8125_rx_vlan_skb(tp, desc, skb) < 0)
                rtl8125_rx_vlan_skb(tp, desc, skb);
                napi_gro_receive(&tp->napi[ring_index], skb);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,11,0)
                dev->last_rx = jiffies;
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(4,11,0)

release_descriptor:
#if defined(ENABLE_RSS_SUPPORT) || defined(ENABLE_PTP_SUPPORT)
                // Rings types 3 & 4 require the buffer address to be mapped again
                if (tp->RxDescType == RX_DESC_RING_TYPE_3)
                        rtl8125_map_and_mark_to_asic_v3((struct RxDescV3 *)desc, rx_buf_phy_addr);
                else if (likely(tp->RxDescType == RX_DESC_RING_TYPE_4))
                        rtl8125_map_and_mark_to_asic_v4((struct RxDescV4 *)desc, rx_buf_phy_addr);
                else
#endif
                rtl8125_mark_to_asic_v1(desc);
                entry = (entry + 1) % NUM_RX_DESC;
        } while (++rx_packets < budget);

        //count = cur_rx - ring->cur_rx;
        dev->stats.rx_bytes += rx_bytes;
        dev->stats.rx_packets += rx_packets;
        ring->cur_rx += rx_packets;

        //printk(KERN_INFO "r8125 rtl8125_rx_interrupt return rx_packets %d, %d\n", rx_packets, rx_bytes);
        return rx_packets;

drop_packet:
        dev->stats.rx_dropped++;
        goto release_descriptor;
}

static inline bool
rtl8125_linkchg_interrupt(struct rtl8125_private *tp, u32 status)
{
        return status & (1 << tp->LinkChgShift);
}

static __always_inline u32
rtl8125_get_linkchg_message_id(struct rtl8125_private *tp)
{
        return (tp->LinkChgShift != 5) ? tp->LinkChgShift : 21;
}

/* The interrupt handler does all of the Rx thread work and cleans up after the Tx thread. */
static irqreturn_t rtl8125_interrupt(const int irq, void *const dev_instance)
{
        struct rtl8125_private * const tp = netdev_priv(((struct napi_struct *) dev_instance)->dev);
        const u32 status = RTL_R32(tp, ISR0_8125); //tp->isr_reg[0]);

        if (unlikely(tp->irq_nvecs < 1)) {  // Implies RTL_MSI | RTL_MSIX
                /* hotplug/major error/no more work/shared irq */
                if (!status || status == 0xFFFFFFFF ||
                        !(status & (tp->intr_mask | tp->timer_intr_mask)))
                        return IRQ_NONE;
        }
/*
#if defined(RTL_USE_NEW_INTR_API)
        if (!tp->irq_tbl[0].requested)
                return IRQ_HANDLED;
#endif
*/
        rtl8125_disable_hw_interrupt(tp);
        RTL_W32(tp, ISR0_8125, status & ~RxFIFOOver); // tp->isr_reg[0]
        //if (rtl8125_linkchg_interrupt(tp, status))
        /* test if this is a linkchg interrupt */
        if (status & BIT(tp->LinkChgShift))
                rtl8125_schedule_linkchg_work(tp);

        if (status & tp->intr_mask || tp->keep_intr_cnt-- > 0) {
                if (status & tp->intr_mask)
                        tp->keep_intr_cnt = RTL_KEEP_INTERRUPT_COUNT;
                if (likely(napi_schedule_prep(tp->napi)))
                        __napi_schedule(tp->napi);
                else if (netif_msg_intr(tp))
                        printk(KERN_INFO "%s: interrupt %04x in poll\n", tp->dev->name, status);
        } else {
                tp->keep_intr_cnt = RTL_KEEP_INTERRUPT_COUNT;
                rtl8125_switch_to_hw_interrupt(tp);
        }

        return IRQ_HANDLED;
}

static irqreturn_t rtl8125_interrupt_msix(const int irq, void * const dev_instance)
{
        struct napi_struct * const napi = dev_instance;
        struct rtl8125_private *const tp = netdev_priv(napi->dev);
        const u8 message_id = napi - tp->napi;
/*
#if defined(RTL_USE_NEW_INTR_API)
        if (!tp->irq_tbl[message_id].requested)
                return IRQ_HANDLED;
#endif
*/
        //link change
        if (unlikely(message_id == rtl8125_get_linkchg_message_id(tp))) {
                //printk(KERN_INFO "r8125: rtl8125_interrupt_msix link chg %d\n", message_id);
                rtl8125_disable_hw_interrupt_v2(tp, message_id);
                rtl8125_clear_hw_isr_v2(tp, message_id);
                rtl8125_schedule_linkchg_work(tp);
                return IRQ_HANDLED;
        }

        if (likely(napi_schedule_prep(napi))) {
                rtl8125_disable_hw_interrupt_v2(tp, message_id);
                __napi_schedule(napi);
        } else if (netif_msg_intr(tp))
                printk(KERN_INFO "%s: interrupt message id %d in poll_msix\n", tp->dev->name, message_id);
        rtl8125_clear_hw_isr_v2(tp, message_id);
        return IRQ_HANDLED;
}

static void rtl8125_down(struct rtl8125_private * const tp, struct net_device * const dev)
{
        //rtl8125_delete_esd_timer(dev, &tp->esd_timer);
        //rtl8125_delete_link_timer(dev, &tp->link_timer);
        netif_carrier_off(dev);
        netif_tx_disable(dev);
        rtl8125_wait_for_quiescence(tp);
        rtl8125_hw_reset(tp);
        rtl8125_tx_clear_rings(tp);
        rtl8125_rx_clear_rings(tp);
}

static int rtl8125_resource_freed(const struct rtl8125_private * const tp)
{
        return (tp->DescArrayPhyAddr == 0);
        /*
        int i;
        for (i = 0; i < tp->num_tx_rings; i++)
                if (tp->tx_ring[i].TxDescArray)
                        return 0;

        for (i = 0; i < tp->num_rx_rings; i++)
                if (tp->rx_ring[i].RxDescArray)
                        return 0;

        return 1;
        */
}

static int rtl8125_close(struct net_device * const dev)
{
        struct rtl8125_private *tp = netdev_priv(dev);

        if (!rtl8125_resource_freed(tp)) {
                set_bit(R8125_FLAG_DOWN, tp->task_flags);
                rtl8125_down(tp, dev);
                pci_clear_master(tp->pci_dev);

#ifdef ENABLE_PTP_SUPPORT
                rtl8125_ptp_stop(tp);
#endif
                rtl8125_hw_d3_para(tp);
                rtl8125_powerdown_pll(dev, 0);
                rtl8125_free_irq(tp);
                rtl8125_free_desc(tp);
        } else {
                rtl8125_hw_d3_para(tp);
                rtl8125_powerdown_pll(dev, 0);
        }
        return 0;
}

static void rtl8125_shutdown(struct pci_dev *pdev)
{
        struct net_device *dev = pci_get_drvdata(pdev);
        struct rtl8125_private *tp = netdev_priv(dev);

        rtnl_lock();

        if (s5_keep_curr_mac == 0 && !rtl8125_flag_is_set(tp, RandomMac))
                rtl8125_rar_set(tp, tp->org_mac_addr);

        if (s5wol == 0)
                tp->wol_enabled = WOL_DISABLED;

        rtl8125_close(dev);
        rtl8125_disable_msi(pdev, tp);
        rtnl_unlock();

        if (system_state == SYSTEM_POWER_OFF) {
                pci_clear_master(tp->pci_dev);
                pci_wake_from_d3(pdev, tp->wol_enabled);
                pci_set_power_state(pdev, PCI_D3hot);
        }
}

#ifdef CONFIG_PM

static int
rtl8125_suspend(struct device *device)
{
        struct pci_dev *pdev = to_pci_dev(device);
        struct net_device *dev = pci_get_drvdata(pdev);
        struct rtl8125_private *tp = netdev_priv(dev);
        if (!netif_running(dev))
                goto out;

        //rtl8125_cancel_all_schedule_work(tp);
        //rtl8125_delete_esd_timer(dev, &tp->esd_timer);
        //rtl8125_delete_link_timer(dev, &tp->link_timer);

        rtnl_lock();
        set_bit(R8125_FLAG_DOWN, tp->task_flags);
        netif_carrier_off(dev);
        netif_tx_disable(dev);
        netif_device_detach(dev);

#ifdef ENABLE_PTP_SUPPORT
        rtl8125_ptp_suspend(tp);
#endif
        rtl8125_hw_reset(tp);
        pci_clear_master(pdev);
        rtl8125_hw_d3_para(tp);
        rtl8125_powerdown_pll(dev, 1);

        rtnl_unlock();
out:
        pci_disable_device(pdev);
        pci_save_state(pdev);
        pci_prepare_to_sleep(pdev);
        return 0;
}

// CHECK
static inline int
rtl8125_hw_d3_not_power_off(struct rtl8125_private *tp)
{
        return rtl8125_check_hw_phy_mcu_code_ver(tp);
}

static int rtl8125_wait_phy_nway_complete_sleep(struct rtl8125_private *tp)
{
        for (int i = 0; i < 30; i++) {
                if (rtl8125_mdio_read(tp, MII_BMSR) & BMSR_ANEGCOMPLETE)
                        return 0;
                msleep(100);
        }
        return -1;
}

static int
rtl8125_resume(struct device *device)
{
        struct pci_dev *pdev = to_pci_dev(device);
        struct net_device *dev = pci_get_drvdata(pdev);
        struct rtl8125_private *tp = netdev_priv(dev);
        u32 err;

        rtnl_lock();
        err = pci_enable_device(pdev);
        if (err) {
                dev_err(&pdev->dev, "Cannot enable PCI device from suspend\n");
                goto out_unlock;
        }
        pci_restore_state(pdev);
        pci_enable_wake(pdev, PCI_D0, 0);

        /* restore last modified mac address */
        rtl8125_rar_set(tp, dev->dev_addr);
        rtl8125_check_hw_phy_mcu_code_ver(tp);

        rtl8125_clear_flag(tp, ResumeNoSpeedChange);
        if (rtl8125_flag_is_set(tp, CheckKeepLinkSpeed) &&
            rtl8125_hw_d3_not_power_off(tp) && rtl8125_wait_phy_nway_complete_sleep(tp) == 0)
            rtl8125_set_flag(tp, ResumeNoSpeedChange);

        if (!netif_running(dev))
                goto out_unlock;

        pci_set_master(pdev);
        rtl8125_exit_oob(tp);
        rtl8125_up(dev);
        clear_bit(R8125_FLAG_DOWN, tp->task_flags);
        rtl8125_schedule_reset_work(tp);
#ifdef ENABLE_ESD
        rtl8125_schedule_esd_work(tp);
#endif

        //mod_timer(&tp->esd_timer, jiffies + RTL8125_ESD_TIMEOUT);
        //mod_timer(&tp->link_timer, jiffies + RTL8125_LINK_TIMEOUT);
out_unlock:
        netif_device_attach(dev);
        rtnl_unlock();
        return err;
}

static struct dev_pm_ops rtl8125_pm_ops = {
        .suspend        = rtl8125_suspend,
        .resume         = rtl8125_resume,
        .freeze         = rtl8125_suspend,
        .thaw           = rtl8125_resume,
        .poweroff       = rtl8125_suspend,
        .restore        = rtl8125_resume,
};

#define RTL8125_PM_OPS        (&rtl8125_pm_ops)

#else /* !CONFIG_PM */
#define RTL8125_PM_OPS        NULL
#endif /* CONFIG_PM */

static struct pci_driver rtl8125_pci_driver = {
        .name           = MODULENAME,
        .id_table       = rtl8125_pci_tbl,
        .probe          = rtl8125_probe,
        .remove         = __devexit_p(rtl8125_remove),
        .shutdown       = rtl8125_shutdown,
#ifdef CONFIG_PM
        .driver.pm      = RTL8125_PM_OPS,
#endif
};

static int __init
rtl8125_init_module(void)
{
#ifdef ENABLE_R8125_PROCFS
        rtl8125_proc_module_init();
#endif
        return pci_register_driver(&rtl8125_pci_driver);
}

static void __exit
rtl8125_cleanup_module(void)
{
        pci_unregister_driver(&rtl8125_pci_driver);

#ifdef ENABLE_R8125_PROCFS
        if (rtl8125_proc) {
                remove_proc_subtree(MODULENAME, init_net.proc_net);
                rtl8125_proc = NULL;
        }
#endif
}

module_init(rtl8125_init_module);
module_exit(rtl8125_cleanup_module);
