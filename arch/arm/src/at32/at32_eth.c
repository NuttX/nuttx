/****************************************************************************
 * arch/arm/src/at32/at32_eth.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_AT32_ETHMAC)

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>
#include <nuttx/crc32.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/queue.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/phy.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/netdev.h>

#if defined(CONFIG_NET_PKT)
#  include <nuttx/net/pkt.h>
#endif

#include "arm_internal.h"
#include "chip.h"
#include "at32_gpio.h"
#include "at32_rcc.h"
#include "at32_syscfg.h"
#include "at32_eth.h"
#include "at32_uid.h"

#include <arch/board/board.h>

/* AT32_NETHERNET determines the number of physical interfaces
 * that will be supported.
 */

#if AT32_NETHERNET > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* See boards/arm/at32/at3240g-eval/README.txt for an explanation of the
 * configuration settings.
 */

#if AT32_NETHERNET > 1
#  error "Logic to support multiple Ethernet interfaces is incomplete"
#endif

/* Work queue support is required. */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required
#endif

/* The low priority work queue is preferred.  If it is not enabled, LPWORK
 * will be the same as HPWORK.
 *
 * NOTE:  However, the network should NEVER run on the high priority work
 * queue!  That queue is intended only to service short back end interrupt
 * processing that never suspends.  Suspending the high priority work queue
 * may bring the system to its knees!
 */

#define ETHWORK LPWORK

#if !defined(CONFIG_AT32_SYSCFG) && !defined(CONFIG_AT32_CONNECTIVITYLINE)
#  error "CONFIG_AT32_SYSCFG must be defined in the NuttX configuration"
#endif

#ifndef CONFIG_AT32_PHYADDR
#  error "CONFIG_AT32_PHYADDR must be defined in the NuttX configuration"
#endif

#if !defined(CONFIG_AT32_MII) && !defined(CONFIG_AT32_RMII)
#  warning "Neither CONFIG_AT32_MII nor CONFIG_AT32_RMII defined"
#endif

#if defined(CONFIG_AT32_MII) && defined(CONFIG_AT32_RMII)
#  error "Both CONFIG_AT32_MII and CONFIG_AT32_RMII defined"
#endif

#ifdef CONFIG_AT32_MII
#  if defined(CONFIG_AT32_AT32F43XX)
#    if !defined(CONFIG_AT32_MII_MCO1) && !defined(CONFIG_AT32_MII_MCO2) && !defined(CONFIG_AT32_MII_EXTCLK)
#      warning "Neither CONFIG_AT32_MII_MCO1, CONFIG_AT32_MII_MCO2, nor CONFIG_AT32_MII_EXTCLK defined"
#    endif
#    if defined(CONFIG_AT32_MII_MCO1) && defined(CONFIG_AT32_MII_MCO2)
#      error "Both CONFIG_AT32_MII_MCO1 and CONFIG_AT32_MII_MCO2 defined"
#    endif
#  elif defined(CONFIG_AT32_CONNECTIVITYLINE)
#    if !defined(CONFIG_AT32_MII_MCO) && !defined(CONFIG_AT32_MII_EXTCLK)
#      warning "Neither CONFIG_AT32_MII_MCO nor CONFIG_AT32_MII_EXTCLK defined"
#    endif
#  endif
#endif

#ifdef CONFIG_AT32_RMII
#  if defined(CONFIG_AT32_AT32F43XX)
#    if !defined(CONFIG_AT32_RMII_MCO1) && !defined(CONFIG_AT32_RMII_MCO2) && !defined(CONFIG_AT32_RMII_EXTCLK)
#      warning "Neither CONFIG_AT32_RMII_MCO1, CONFIG_AT32_RMII_MCO2, nor CONFIG_AT32_RMII_EXTCLK defined"
#    endif
#    if defined(CONFIG_AT32_RMII_MCO1) && defined(CONFIG_AT32_RMII_MCO2)
#      error "Both CONFIG_AT32_RMII_MCO1 and CONFIG_AT32_RMII_MCO2 defined"
#    endif
#  elif defined(CONFIG_AT32_CONNECTIVITYLINE)
#    if !defined(CONFIG_AT32_RMII_MCO) && !defined(CONFIG_AT32_RMII_EXTCLK)
#      warning "Neither CONFIG_AT32_RMII_MCO nor CONFIG_AT32_RMII_EXTCLK defined"
#    endif
#  endif
#endif

#ifdef CONFIG_AT32_AUTONEG
#  ifndef CONFIG_AT32_PHYSR
#    error "CONFIG_AT32_PHYSR must be defined in the NuttX configuration"
#  endif
#  ifdef CONFIG_AT32_PHYSR_ALTCONFIG
#    ifndef CONFIG_AT32_PHYSR_ALTMODE
#      error "CONFIG_AT32_PHYSR_ALTMODE must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_AT32_PHYSR_10HD
#      error "CONFIG_AT32_PHYSR_10HD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_AT32_PHYSR_100HD
#      error "CONFIG_AT32_PHYSR_100HD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_AT32_PHYSR_10FD
#      error "CONFIG_AT32_PHYSR_10FD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_AT32_PHYSR_100FD
#      error "CONFIG_AT32_PHYSR_100FD must be defined in the NuttX configuration"
#    endif
#  else
#    ifndef CONFIG_AT32_PHYSR_SPEED
#      error "CONFIG_AT32_PHYSR_SPEED must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_AT32_PHYSR_100MBPS
#      error "CONFIG_AT32_PHYSR_100MBPS must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_AT32_PHYSR_MODE
#      error "CONFIG_AT32_PHYSR_MODE must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_AT32_PHYSR_FULLDUPLEX
#      error "CONFIG_AT32_PHYSR_FULLDUPLEX must be defined in the NuttX configuration"
#    endif
#  endif
#endif

/* These definitions are used to enable the PHY interrupts */

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
#  if defined( CONFIG_ETH0_PHY_AM79C874)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_KS8721)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_KSZ8041)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_KSZ8051)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_KSZ8061)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_KSZ8081)
#    define MII_INT_REG    MII_KSZ8081_INT
#    define MII_INT_SETEN  MII_KSZ80X1_INT_LDEN | MII_KSZ80X1_INT_LUEN
#    define MII_INT_CLREN  0
#  elif defined( CONFIG_ETH0_PHY_KSZ90x1)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_DP83848C)
#    define MII_INT_REG    MII_DP83848C_MISR
#    define MII_INT_SETEN  MII_DP83848C_LINK_INT_EN
#    define MII_INT_CLREN  0
#  elif defined( CONFIG_ETH0_PHY_LAN8720)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_LAN8740)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_LAN8740A)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_LAN8742A)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_DM9161)
#    error missing logic
#  else
#    error unknown PHY
#  endif
#endif

#ifdef CONFIG_AT32_ETH_PTP
#  warning "CONFIG_AT32_ETH_PTP is not yet supported"
#endif

/* This driver does not use enhanced descriptors.  Enhanced descriptors must
 * be used, however, if time stamping or and/or IPv4 checksum offload is
 * supported.
 */

#undef CONFIG_AT32_ETH_ENHANCEDDESC

#define CONFIG_AT32_ETH_HWCHECKSUM

/* Add 4 to the configured buffer size to account for the 2 byte checksum
 * memory needed at the end of the maximum size packet.  Buffer sizes must
 * be an even multiple of 4, 8, or 16 bytes (depending on buswidth).  We
 * will use the 16-byte alignment in all cases.
 */

#define OPTIMAL_ETH_BUFSIZE ((CONFIG_NET_ETH_PKTSIZE + 4 + 15) & ~15)

#ifndef CONFIG_AT32_ETH_BUFSIZE
#  define CONFIG_AT32_ETH_BUFSIZE OPTIMAL_ETH_BUFSIZE
#endif

#if CONFIG_AT32_ETH_BUFSIZE > ETH_TDES1_TBS1_MASK
#  error "CONFIG_AT32_ETH_BUFSIZE is too large"
#endif

#if (CONFIG_AT32_ETH_BUFSIZE & 15) != 0
#  error "CONFIG_AT32_ETH_BUFSIZE must be aligned"
#endif

#if CONFIG_AT32_ETH_BUFSIZE != OPTIMAL_ETH_BUFSIZE
#  warning "You using an incomplete/untested configuration"
#endif

#ifndef CONFIG_AT32_ETH_NRXDESC
#  define CONFIG_AT32_ETH_NRXDESC 8
#endif
#ifndef CONFIG_AT32_ETH_NTXDESC
#  define CONFIG_AT32_ETH_NTXDESC 4
#endif

/* We need at least one more free buffer than transmit buffers */

#define AT32_ETH_NFREEBUFFERS (CONFIG_AT32_ETH_NTXDESC+1)

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#ifndef CONFIG_DEBUG_NET_INFO
#  undef CONFIG_AT32_ETHMAC_REGDEBUG
#endif

/* Clocking *****************************************************************/

/* Set MACMIIAR CR bits depending on HCLK setting */

#if AT32_HCLK_FREQUENCY >= 20000000 && AT32_HCLK_FREQUENCY < 35000000
#  define ETH_MACMIIAR_CR ETH_MACMIIAR_CR_20_35
#elif AT32_HCLK_FREQUENCY >= 35000000 && AT32_HCLK_FREQUENCY < 60000000
#  define ETH_MACMIIAR_CR ETH_MACMIIAR_CR_35_60
#elif AT32_HCLK_FREQUENCY >= 60000000 && AT32_HCLK_FREQUENCY < 100000000
#  define ETH_MACMIIAR_CR ETH_MACMIIAR_CR_60_100
#elif AT32_HCLK_FREQUENCY >= 100000000 && AT32_HCLK_FREQUENCY < 150000000
#  define ETH_MACMIIAR_CR ETH_MACMIIAR_CR_100_150
#elif AT32_HCLK_FREQUENCY >= 150000000 && AT32_HCLK_FREQUENCY < 250000000
#  define ETH_MACMIIAR_CR ETH_MACMIIAR_CR_150_250
#elif AT32_HCLK_FREQUENCY >= 250000000 && AT32_HCLK_FREQUENCY <= 300000000
#  define ETH_MACMIIAR_CR ETH_MACMIIAR_CR_250_300
#else
#  error "AT32_HCLK_FREQUENCY not supportable"
#endif

/* Timing *******************************************************************/

/* TX timeout = 1 minute */

#define AT32_TXTIMEOUT   (60*CLK_TCK)

/* PHY reset/configuration delays in milliseconds */

#define PHY_RESET_DELAY   (65)
#define PHY_CONFIG_DELAY  (1000)

/* PHY read/write delays in loop counts */

#define PHY_READ_TIMEOUT  (0x0004ffff)
#define PHY_WRITE_TIMEOUT (0x0004ffff)
#define PHY_RETRY_TIMEOUT (0x0004ffff)

/* Register values **********************************************************/

/* Clear the MACCR bits that will be setup during MAC initialization (or that
 * are cleared unconditionally).  Per the reference manual, all reserved bits
 * must be retained at their reset value.
 *
 * ETH_MACCR_RE   Bit 2:  Receiver enable
 * ETH_MACCR_TE   Bit 3:  Transmitter enable
 * ETH_MACCR_DC   Bit 4:  Deferral check
 * ETH_MACCR_BL   Bits 5-6: Back-off limit
 * ETH_MACCR_APCS Bit 7:  Automatic pad/CRC stripping
 * ETH_MACCR_RD   Bit 9:  Retry disable
 * ETH_MACCR_IPCO Bit 10: IPv4 checksum offload
 * ETH_MACCR_DM   Bit 11: Duplex mode
 * ETH_MACCR_LM   Bit 12: Loopback mode
 * ETH_MACCR_ROD  Bit 13: Receive own disable
 * ETH_MACCR_FES  Bit 14: Fast Ethernet speed
 * ETH_MACCR_CSD  Bit 16: Carrier sense disable
 * ETH_MACCR_IFG  Bits 17-19: Interframe gap
 * ETH_MACCR_JD   Bit 22: Jabber disable
 * ETH_MACCR_WD   Bit 23: Watchdog disable
 */

#define MACCR_CLEAR_BITS \
  (ETH_MACCR_RE | ETH_MACCR_TE | ETH_MACCR_DC | ETH_MACCR_BL_MASK | \
   ETH_MACCR_APCS | ETH_MACCR_RD | ETH_MACCR_IPCO | ETH_MACCR_DM | \
   ETH_MACCR_LM | ETH_MACCR_ROD | ETH_MACCR_FES | ETH_MACCR_CSD | \
   ETH_MACCR_IFG_MASK | ETH_MACCR_JD | ETH_MACCR_WD)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ETH_MACCR_RE   Receiver enable               0 (disabled)
 * ETH_MACCR_TE   Transmitter enable            0 (disabled)
 * ETH_MACCR_DC   Deferral check                0 (disabled)
 * ETH_MACCR_BL   Back-off limit                0 (10)
 * ETH_MACCR_APCS Automatic pad/CRC stripping   0 (disabled)
 * ETH_MACCR_RD   Retry disable                 1 (disabled)
 * ETH_MACCR_IPCO IPv4 checksum offload         Depends on
 *                                              CONFIG_AT32_ETH_HWCHECKSUM
 * ETH_MACCR_LM   Loopback mode                 0 (disabled)
 * ETH_MACCR_ROD  Receive own disable           0 (enabled)
 * ETH_MACCR_CSD  Carrier sense disable         0 (enabled)
 * ETH_MACCR_IFG  Interframe gap                0 (96 bits)
 * ETH_MACCR_JD   Jabber disable                0 (enabled)
 * ETH_MACCR_WD   Watchdog disable              0 (enabled)
 * ETH_MACCR_CSTF CRC stripping for Type frames 0 (disabled, F4 only)
 *
 * The following are set conditioinally based on mode and speed.
 *
 * ETH_MACCR_DM  Duplex mode                    Depends on priv->fduplex
 * ETH_MACCR_FES Fast Ethernet speed            Depends on priv->mbps100
 */

#ifdef CONFIG_AT32_ETH_HWCHECKSUM
#  define MACCR_SET_BITS \
     (ETH_MACCR_BL_10 | ETH_MACCR_RD | ETH_MACCR_IPCO | ETH_MACCR_IFG(96))
#else
#  define MACCR_SET_BITS \
     (ETH_MACCR_BL_10 | ETH_MACCR_RD | ETH_MACCR_IFG(96))
#endif

/* Clear the MACCR bits that will be setup during MAC initialization (or that
 * are cleared unconditionally).  Per the reference manual, all reserved bits
 * must be retained at their reset value.
 *
 * ETH_MACFFR_PM   Bit 0: Promiscuous mode
 * ETH_MACFFR_HU   Bit 1: Hash unicast
 * ETH_MACFFR_HM   Bit 2: Hash multicast
 * ETH_MACFFR_DAIF Bit 3: Destination address inverse filtering
 * ETH_MACFFR_PAM  Bit 4: Pass all multicast
 * ETH_MACFFR_BFD  Bit 5: Broadcast frames disable
 * ETH_MACFFR_PCF  Bits 6-7: Pass control frames
 * ETH_MACFFR_SAIF Bit 8: Source address inverse filtering
 * ETH_MACFFR_SAF  Bit 9: Source address filter
 * ETH_MACFFR_HPF  Bit 10: Hash or perfect filter
 * ETH_MACFFR_RA   Bit 31: Receive all
 */

#define MACFFR_CLEAR_BITS \
  (ETH_MACFFR_PM | ETH_MACFFR_HU | ETH_MACFFR_HM | ETH_MACFFR_DAIF | \
   ETH_MACFFR_PAM | ETH_MACFFR_BFD | ETH_MACFFR_PCF_MASK | ETH_MACFFR_SAIF | \
   ETH_MACFFR_SAF | ETH_MACFFR_HPF | ETH_MACFFR_RA)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ETH_MACFFR_HU   Hash unicast             0 (perfect dest filtering)
 * ETH_MACFFR_HM   Hash multicast           0 (perfect dest filtering)
 * ETH_MACFFR_DAIF Destination address      0 (normal)
 *                 inverse filtering
 * ETH_MACFFR_PAM  Pass all multicast       0 (Depends on HM bit)
 * ETH_MACFFR_BFD  Broadcast frames disable 0 (enabled)
 * ETH_MACFFR_PCF  Pass control frames      1 (block all but PAUSE)
 * ETH_MACFFR_SAIF Source address inverse   0 (not used)
 *                 filtering
 * ETH_MACFFR_SAF  Source address filter    0 (disabled)
 * ETH_MACFFR_HPF  Hash or perfect filter   0 (Only matching frames passed)
 * ETH_MACFFR_RA   Receive all              0 (disabled)
 */

#ifdef CONFIG_NET_PROMISCUOUS
#  define MACFFR_SET_BITS (ETH_MACFFR_PCF_PAUSE | ETH_MACFFR_PM)
#else
#  define MACFFR_SET_BITS (ETH_MACFFR_PCF_PAUSE)
#endif

/* Clear the MACFCR bits that will be setup during MAC initialization (or
 * that are cleared unconditionally). Per the reference manual, all reserved
 * bits must be retained at their reset value.
 *
 * ETH_MACFCR_FCB_BPA Bit 0: Flow control busy/back pressure activate
 * ETH_MACFCR_TFCE    Bit 1: Transmit flow control enable
 * ETH_MACFCR_RFCE    Bit 2: Receive flow control enable
 * ETH_MACFCR_UPFD    Bit 3: Unicast pause frame detect
 * ETH_MACFCR_PLT     Bits 4-5: Pause low threshold
 * ETH_MACFCR_ZQPD    Bit 7: Zero-quanta pause disable
 * ETH_MACFCR_PT      Bits 16-31: Pause time
 */

#define MACFCR_CLEAR_MASK \
  (ETH_MACFCR_FCB_BPA | ETH_MACFCR_TFCE | ETH_MACFCR_RFCE | ETH_MACFCR_UPFD | \
   ETH_MACFCR_PLT_MASK | ETH_MACFCR_ZQPD | ETH_MACFCR_PT_MASK)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ETH_MACFCR_FCB_BPA Flow control busy/back       0 (no pause control frame)
 *                    activate pressure
 * ETH_MACFCR_TFCE    Transmit flow control enable 0 (disabled)
 * ETH_MACFCR_RFCE    Receive flow control enable  0 (disabled)
 * ETH_MACFCR_UPFD    Unicast pause frame detect   0 (disabled)
 * ETH_MACFCR_PLT     Pause low threshold          0 (pause time - 4)
 * ETH_MACFCR_ZQPD    Zero-quanta pause disable    1 (disabled)
 * ETH_MACFCR_PT      Pause time                   0
 */

#define MACFCR_SET_MASK (ETH_MACFCR_PLT_M4 | ETH_MACFCR_ZQPD)

/* Clear the DMAOMR bits that will be setup during MAC initialization (or
 * that are cleared unconditionally). Per the reference manual, all reserved
 * bits must be retained at their reset value.
 *
 * ETH_DMAOMR_SR    Bit 1:  Start/stop receive
 * TH_DMAOMR_OSF    Bit 2:  Operate on second frame
 * ETH_DMAOMR_RTC   Bits 3-4: Receive threshold control
 * ETH_DMAOMR_FUGF  Bit 6:  Forward undersized good frames
 * ETH_DMAOMR_FEF   Bit 7:  Forward error frames
 * ETH_DMAOMR_ST    Bit 13: Start/stop transmission
 * ETH_DMAOMR_TTC   Bits 14-16: Transmit threshold control
 * ETH_DMAOMR_FTF   Bit 20: Flush transmit FIFO
 * ETH_DMAOMR_TSF   Bit 21: Transmit store and forward
 * ETH_DMAOMR_DFRF  Bit 24: Disable flushing of received frames
 * ETH_DMAOMR_RSF   Bit 25: Receive store and forward
 * TH_DMAOMR_DTCEFD Bit 26: Dropping of TCP/IP checksum error frames disable
 */

#define DMAOMR_CLEAR_MASK \
  (ETH_DMAOMR_SR | ETH_DMAOMR_OSF | ETH_DMAOMR_RTC_MASK | ETH_DMAOMR_FUGF | \
   ETH_DMAOMR_FEF | ETH_DMAOMR_ST | ETH_DMAOMR_TTC_MASK | ETH_DMAOMR_FTF | \
   ETH_DMAOMR_TSF | ETH_DMAOMR_DFRF | ETH_DMAOMR_RSF | ETH_DMAOMR_DTCEFD)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ETH_DMAOMR_SR    Start/stop receive           0 (not running)
 * TH_DMAOMR_OSF    Operate on second frame      1 (enabled)
 * ETH_DMAOMR_RTC   Receive threshold control    0 (64 bytes)
 * ETH_DMAOMR_FUGF  Forward undersized good      0 (disabled)
 *                  frames
 * ETH_DMAOMR_FEF   Forward error frames         0 (disabled)
 * ETH_DMAOMR_ST    Start/stop transmission      0 (not running)
 * ETH_DMAOMR_TTC   Transmit threshold control   0 (64 bytes)
 * ETH_DMAOMR_FTF   Flush transmit FIFO          0 (no flush)
 * ETH_DMAOMR_TSF   Transmit store and forward   1 (enabled)
 * ETH_DMAOMR_DFRF  Disable flushing of received 0 (enabled)
 *                  frames
 * ETH_DMAOMR_RSF   Receive store and forward    1 (enabled)
 * TH_DMAOMR_DTCEFD Dropping of TCP/IP checksum  Depends on
 *                  error frames disable         CONFIG_AT32_ETH_HWCHECKSUM
 *
 * When the checksum offload feature is enabled, we need to enable the Store
 * and Forward mode: the store and forward guarantee that a whole frame is
 * stored in the FIFO, so the MAC can insert/verify the checksum, if the
 * checksum is OK the DMA can handle the frame otherwise the frame is dropped
 */

#ifdef CONFIG_AT32_ETH_HWCHECKSUM
#  define DMAOMR_SET_MASK \
    (ETH_DMAOMR_OSF | ETH_DMAOMR_RTC_64 | ETH_DMAOMR_TTC_64 | \
     ETH_DMAOMR_TSF | ETH_DMAOMR_RSF)
#else
#  define DMAOMR_SET_MASK \
    (ETH_DMAOMR_OSF | ETH_DMAOMR_RTC_64 | ETH_DMAOMR_TTC_64 | \
     ETH_DMAOMR_TSF | ETH_DMAOMR_RSF | ETH_DMAOMR_DTCEFD)
#endif

/* Clear the DMABMR bits that will be setup during MAC initialization (or
 * that are cleared unconditionally). Per the reference manual, all reserved
 * bits must be retained at their reset value.
 *
 * ETH_DMABMR_SR   Bit 0: Software reset
 * ETH_DMABMR_DA   Bit 1: DMA Arbitration
 * ETH_DMABMR_DSL  Bits 2-6: Descriptor skip length
 * ETH_DMABMR_PBL  Bits 8-13: Programmable burst length
 * ETH_DMABMR_RTPR Bits 14-15: RX TX priority ratio
 * ETH_DMABMR_FB   Bit 16: Fixed burst
 * ETH_DMABMR_RDP  Bits 17-22: RX DMA PBL
 * ETH_DMABMR_USP  Bit 23: Use separate PBL
 * ETH_DMABMR_FPM  Bit 24: 8xPBL mode
 * ETH_DMABMR_AAB  Bit 25: Address-aligned beats
 */

#define DMABMR_CLEAR_MASK \
  (ETH_DMABMR_SR | ETH_DMABMR_DA | ETH_DMABMR_DSL_MASK  | \
   ETH_DMABMR_PBL_MASK | ETH_DMABMR_RTPR_MASK | ETH_DMABMR_FB | ETH_DMABMR_RDP_MASK | \
   ETH_DMABMR_USP | ETH_DMABMR_FPM | ETH_DMABMR_AAB)

/* The following bits are set or left zero unconditionally in all modes.
 *
 *
 * ETH_DMABMR_SR   Software reset             0 (no reset)
 * ETH_DMABMR_DA   DMA Arbitration            0 (round robin)
 * ETH_DMABMR_DSL  Descriptor skip length     0
 * ETH_DMABMR_PBL  Programmable burst length  32 beats
 * ETH_DMABMR_RTPR RX TX priority ratio       2:1
 * ETH_DMABMR_FB   Fixed burst                1 (enabled)
 * ETH_DMABMR_RDP  RX DMA PBL                 32 beats
 * ETH_DMABMR_USP  Use separate PBL           1 (enabled)
 * ETH_DMABMR_FPM  8xPBL mode                 0 (disabled)
 * ETH_DMABMR_AAB  Address-aligned beats      1 (enabled)
 * ETH_DMABMR_MB   Mixed burst                0 (disabled, F4 only)
 */

#define DMABMR_SET_MASK \
    (ETH_DMABMR_DSL(0) | ETH_DMABMR_PBL(32) | ETH_DMABMR_RTPR_2TO1 | ETH_DMABMR_FB | \
    ETH_DMABMR_RDP(32) | ETH_DMABMR_USP | ETH_DMABMR_AAB)

/* Interrupt bit sets *******************************************************/

/* All interrupts in the normal and abnormal interrupt summary. Early
 * transmit interrupt (ETI) is excluded from the abnormal set because it
 * causes too many interrupts and is not interesting.
 */

#define ETH_DMAINT_NORMAL \
  (ETH_DMAINT_TI | ETH_DMAINT_TBUI | ETH_DMAINT_RI | ETH_DMAINT_ERI)

#define ETH_DMAINT_ABNORMAL \
  (ETH_DMAINT_TPSI | ETH_DMAINT_TJTI | ETH_DMAINT_ROI | ETH_DMAINT_TUI | \
   ETH_DMAINT_RBUI | ETH_DMAINT_RPSI | ETH_DMAINT_RWTI | /* ETH_DMAINT_ETI | */ \
   ETH_DMAINT_FBEI)

/* Normal receive, transmit, error interrupt enable bit sets */

#define ETH_DMAINT_RECV_ENABLE    (ETH_DMAINT_NIS | ETH_DMAINT_RI)
#define ETH_DMAINT_XMIT_ENABLE    (ETH_DMAINT_NIS | ETH_DMAINT_TI)
#define ETH_DMAINT_XMIT_DISABLE   (ETH_DMAINT_TI)

#ifdef CONFIG_DEBUG_NET
#  define ETH_DMAINT_ERROR_ENABLE (ETH_DMAINT_AIS | ETH_DMAINT_ABNORMAL)
#else
#  define ETH_DMAINT_ERROR_ENABLE (0)
#endif

/* Helpers ******************************************************************/

/* This is a helper pointer for accessing the contents of the Ethernet
 * header
 */

#define BUF ((struct eth_hdr_s *)priv->dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The at32_ethmac_s encapsulates all state information for a single
 * hardware interface
 */

struct at32_ethmac_s
{
  uint8_t              ifup    : 1; /* true:ifup false:ifdown */
  uint8_t              mbps100 : 1; /* 100MBps operation (vs 10 MBps) */
  uint8_t              fduplex : 1; /* Full (vs. half) duplex */
  struct wdog_s        txtimeout;   /* TX timeout timer */
  struct work_s        irqwork;     /* For deferring interrupt work to the work queue */
  struct work_s        pollwork;    /* For deferring poll work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s  dev;         /* Interface understood by the network */

  /* Used to track transmit and receive descriptors */

  struct eth_txdesc_s *txhead;      /* Next available TX descriptor */
  struct eth_rxdesc_s *rxhead;      /* Next available RX descriptor */

  struct eth_txdesc_s *txtail;      /* First "in_flight" TX descriptor */
  struct eth_rxdesc_s *rxcurr;      /* First RX descriptor of the segment */
  uint16_t             segments;    /* RX segment count */
  uint16_t             inflight;    /* Number of TX transfers "in_flight" */
  sq_queue_t           freeb;       /* The free buffer list */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Descriptor allocations */

static struct eth_rxdesc_s g_rxtable[CONFIG_AT32_ETH_NRXDESC]
  aligned_data(4);
static struct eth_txdesc_s g_txtable[CONFIG_AT32_ETH_NTXDESC]
  aligned_data(4);

/* Buffer allocations */

static uint8_t g_rxbuffer[CONFIG_AT32_ETH_NRXDESC *
                          CONFIG_AT32_ETH_BUFSIZE] aligned_data(4);
static uint8_t g_alloc[AT32_ETH_NFREEBUFFERS *
                       CONFIG_AT32_ETH_BUFSIZE] aligned_data(4);

static struct at32_ethmac_s g_at32ethmac[AT32_NETHERNET];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#if defined(CONFIG_AT32_ETHMAC_REGDEBUG) && defined(CONFIG_DEBUG_FEATURES)
static uint32_t at32_getreg(uint32_t addr);
static void at32_putreg(uint32_t val, uint32_t addr);
static void at32_checksetup(void);
#else
# define at32_getreg(addr)      getreg32(addr)
# define at32_putreg(val,addr)  putreg32(val,addr)
# define at32_checksetup()
#endif

/* Free buffer management */

static void at32_initbuffer(struct at32_ethmac_s *priv, uint8_t *alloc);
static inline uint8_t *at32_allocbuffer(struct at32_ethmac_s *priv);
static inline void at32_freebuffer(struct at32_ethmac_s *priv,
              uint8_t *buffer);
static inline bool at32_isfreebuffer(struct at32_ethmac_s *priv);

/* Common TX logic */

static int  at32_transmit(struct at32_ethmac_s *priv);
static int  at32_txpoll(struct net_driver_s *dev);
static void at32_dopoll(struct at32_ethmac_s *priv);

/* Interrupt handling */

static void at32_enableint(struct at32_ethmac_s *priv,
              uint32_t ierbit);
static void at32_disableint(struct at32_ethmac_s *priv,
              uint32_t ierbit);

static void at32_freesegment(struct at32_ethmac_s *priv,
              struct eth_rxdesc_s *rxfirst, int segments);
static int  at32_recvframe(struct at32_ethmac_s *priv);
static void at32_receive(struct at32_ethmac_s *priv);
static void at32_freeframe(struct at32_ethmac_s *priv);
static void at32_txdone(struct at32_ethmac_s *priv);

static void at32_interrupt_work(void *arg);
static int  at32_interrupt(int irq, void *context, void *arg);

/* Watchdog timer expirations */

static void at32_txtimeout_work(void *arg);
static void at32_txtimeout_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  at32_ifup(struct net_driver_s *dev);
static int  at32_ifdown(struct net_driver_s *dev);

static void at32_txavail_work(void *arg);
static int  at32_txavail(struct net_driver_s *dev);

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int  at32_addmac(struct net_driver_s *dev, const uint8_t *mac);
#endif
#ifdef CONFIG_NET_MCASTGROUP
static int  at32_rmmac(struct net_driver_s *dev, const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  at32_ioctl(struct net_driver_s *dev, int cmd,
              unsigned long arg);
#endif

/* Descriptor Initialization */

static void at32_txdescinit(struct at32_ethmac_s *priv,
                             struct eth_txdesc_s *txtable);
static void at32_rxdescinit(struct at32_ethmac_s *priv,
                             struct eth_rxdesc_s *rxtable,
                             uint8_t *rxbuffer);

/* PHY Initialization */

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
static int  at32_phyintenable(struct at32_ethmac_s *priv);
#endif
#if defined(CONFIG_AT32_AUTONEG) || defined(CONFIG_NETDEV_PHY_IOCTL) || \
    defined(CONFIG_ETH0_PHY_DM9161)
static int  at32_phyread(uint16_t phydevaddr, uint16_t phyregaddr,
              uint16_t *value);
#endif
static int  at32_phywrite(uint16_t phydevaddr, uint16_t phyregaddr,
              uint16_t value);
#ifdef CONFIG_ETH0_PHY_DM9161
static inline int at32_dm9161(struct at32_ethmac_s *priv);
#endif
static int  at32_phyinit(struct at32_ethmac_s *priv);

/* MAC/DMA Initialization */

#ifdef CONFIG_AT32_MII
static inline void at32_selectmii(void);
#endif
#ifdef CONFIG_AT32_RMII
static inline void at32_selectrmii(void);
#endif
static inline void at32_ethgpioconfig(struct at32_ethmac_s *priv);
static int  at32_ethreset(struct at32_ethmac_s *priv);
static int  at32_macconfig(struct at32_ethmac_s *priv);
static void at32_macaddress(struct at32_ethmac_s *priv);
static int  at32_macenable(struct at32_ethmac_s *priv);
static int  at32_ethconfig(struct at32_ethmac_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32_getreg
 *
 * Description:
 *   This function may to used to intercept an monitor all register accesses.
 *   Clearly this is nothing you would want to do unless you are debugging
 *   this driver.
 *
 * Input Parameters:
 *   addr - The register address to read
 *
 * Returned Value:
 *   The value read from the register
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_ETHMAC_REGDEBUG
static uint32_t at32_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval   = 0;
  static uint32_t count    = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              ninfo("...\n");
            }

          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          ninfo("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  ninfo("%08x->%08x\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: at32_putreg
 *
 * Description:
 *   This function may to used to intercept an monitor all register accesses.
 *   Clearly this is nothing you would want to do unless you are debugging
 *   this driver.
 *
 * Input Parameters:
 *   val - The value to write to the register
 *   addr - The register address to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_AT32_ETHMAC_REGDEBUG) && defined(CONFIG_DEBUG_FEATURES)
static void at32_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  ninfo("%08x<-%08x\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: at32_checksetup
 *
 * Description:
 *   Show the state of critical configuration registers.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_ETHMAC_REGDEBUG
static void at32_checksetup(void)
{
}
#endif

/****************************************************************************
 * Function: at32_initbuffer
 *
 * Description:
 *   Initialize the free buffer list.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called during early driver initialization before Ethernet interrupts
 *   are enabled.
 *
 ****************************************************************************/

static void at32_initbuffer(struct at32_ethmac_s *priv, uint8_t *alloc)
{
  uint8_t *buffer;
  int i;

  /* Initialize the head of the free buffer list */

  sq_init(&priv->freeb);

  /* Add all of the pre-allocated buffers to the free buffer list */

  for (i = 0, buffer = alloc;
       i < AT32_ETH_NFREEBUFFERS;
       i++, buffer += CONFIG_AT32_ETH_BUFSIZE)
    {
      sq_addlast((sq_entry_t *)buffer, &priv->freeb);
    }
}

/****************************************************************************
 * Function: at32_allocbuffer
 *
 * Description:
 *   Allocate one buffer from the free buffer list.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   Pointer to the allocated buffer on success; NULL on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static inline uint8_t *at32_allocbuffer(struct at32_ethmac_s *priv)
{
  /* Allocate a buffer by returning the head of the free buffer list */

  return (uint8_t *)sq_remfirst(&priv->freeb);
}

/****************************************************************************
 * Function: at32_freebuffer
 *
 * Description:
 *   Return a buffer to the free buffer list.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *   buffer - A pointer to the buffer to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static inline void at32_freebuffer(struct at32_ethmac_s *priv,
                                    uint8_t *buffer)
{
  /* Free the buffer by adding it to the end of the free buffer list */

  sq_addlast((sq_entry_t *)buffer, &priv->freeb);
}

/****************************************************************************
 * Function: at32_isfreebuffer
 *
 * Description:
 *   Return TRUE if the free buffer list is not empty.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   True if there are one or more buffers in the free buffer list;
 *   false if the free buffer list is empty
 *
 * Assumptions:
 *   None.
 *
 ****************************************************************************/

static inline bool at32_isfreebuffer(struct at32_ethmac_s *priv)
{
  /* Return TRUE if the free buffer list is not empty */

  return !sq_empty(&priv->freeb);
}

/****************************************************************************
 * Function: at32_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int at32_transmit(struct at32_ethmac_s *priv)
{
  struct eth_txdesc_s *txdesc;
  struct eth_txdesc_s *txfirst;

  /* The internal (optimal) network buffer size may be configured to be
   * larger than the Ethernet buffer size.
   */

#if OPTIMAL_ETH_BUFSIZE > CONFIG_AT32_ETH_BUFSIZE
  uint8_t *buffer;
  int bufcount;
  int lastsize;
  int i;
#endif

  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  txdesc  = priv->txhead;
  txfirst = txdesc;

  ninfo("d_len: %d d_buf: %p txhead: %p tdes0: %08" PRIx32 "\n",
        priv->dev.d_len, priv->dev.d_buf, txdesc, txdesc->tdes0);

  DEBUGASSERT(txdesc && (txdesc->tdes0 & ETH_TDES0_OWN) == 0);

  /* Is the size to be sent greater than the size of the Ethernet buffer? */

  DEBUGASSERT(priv->dev.d_len > 0 && priv->dev.d_buf != NULL);

#if OPTIMAL_ETH_BUFSIZE > CONFIG_AT32_ETH_BUFSIZE
  if (priv->dev.d_len > CONFIG_AT32_ETH_BUFSIZE)
    {
      /* Yes... how many buffers will be need to send the packet? */

      bufcount = (priv->dev.d_len + (CONFIG_AT32_ETH_BUFSIZE - 1)) /
                 CONFIG_AT32_ETH_BUFSIZE;
      lastsize = priv->dev.d_len - (bufcount - 1) * CONFIG_AT32_ETH_BUFSIZE;

      ninfo("bufcount: %d lastsize: %d\n", bufcount, lastsize);

      /* Set the first segment bit in the first TX descriptor */

      txdesc->tdes0 |= ETH_TDES0_FS;

      /* Set up all but the last TX descriptor */

      buffer = priv->dev.d_buf;

      for (i = 0; i < bufcount; i++)
        {
          /* This could be a normal event but the design does not handle it */

          DEBUGASSERT((txdesc->tdes0 & ETH_TDES0_OWN) == 0);

          /* Set the Buffer1 address pointer */

          txdesc->tdes2 = (uint32_t)buffer;

          /* Set the buffer size in all TX descriptors */

          if (i == (bufcount - 1))
            {
              /* This is the last segment.  Set the last segment bit in the
               * last TX descriptor and ask for an interrupt when this
               * segment transfer completes.
               */

              txdesc->tdes0 |= (ETH_TDES0_LS | ETH_TDES0_IC);

              /* This segment is, most likely, of fractional buffersize */

              txdesc->tdes1  = lastsize;
              buffer        += lastsize;
            }
          else
            {
              /* This is not the last segment.  We don't want an interrupt
               * when this segment transfer completes.
               */

              txdesc->tdes0 &= ~ETH_TDES0_IC;

              /* The size of the transfer is the whole buffer */

              txdesc->tdes1  = CONFIG_AT32_ETH_BUFSIZE;
              buffer        += CONFIG_AT32_ETH_BUFSIZE;
            }

          /* Give the descriptor to DMA */

          txdesc->tdes0 |= ETH_TDES0_OWN;
          txdesc         = (struct eth_txdesc_s *)txdesc->tdes3;
        }
    }
  else
#endif
    {
      /* The single descriptor is both the first and last segment.  And we do
       * want an interrupt when the transfer completes.
       */

      txdesc->tdes0 |= (ETH_TDES0_FS | ETH_TDES0_LS | ETH_TDES0_IC);

      /* Set frame size */

      DEBUGASSERT(priv->dev.d_len <= CONFIG_AT32_ETH_BUFSIZE);
      txdesc->tdes1 = priv->dev.d_len;

      /* Set the Buffer1 address pointer */

      txdesc->tdes2 = (uint32_t)priv->dev.d_buf;

      /* Set OWN bit of the TX descriptor tdes0.  This gives the buffer to
       * Ethernet DMA
       */

      txdesc->tdes0 |= ETH_TDES0_OWN;

      /* Point to the next available TX descriptor */

      txdesc = (struct eth_txdesc_s *)txdesc->tdes3;
    }

  /* Remember where we left off in the TX descriptor chain */

  priv->txhead = txdesc;

  /* Detach the buffer from priv->dev structure.  That buffer is now
   * "in-flight".
   */

  priv->dev.d_buf = NULL;
  priv->dev.d_len = 0;

  /* If there is no other TX buffer, in flight, then remember the location
   * of the TX descriptor.  This is the location to check for TX done events.
   */

  if (!priv->txtail)
    {
      DEBUGASSERT(priv->inflight == 0);
      priv->txtail = txfirst;
    }

  /* Increment the number of TX transfer in-flight */

  priv->inflight++;

  ninfo("txhead: %p txtail: %p inflight: %d\n",
        priv->txhead, priv->txtail, priv->inflight);

  /* If all TX descriptors are in-flight, then we have to disable receive
   * interrupts too. This is because receive events can trigger more
   * un-stoppable transmit events.
   */

  if (priv->inflight >= CONFIG_AT32_ETH_NTXDESC)
    {
      at32_disableint(priv, ETH_DMAINT_RI);
    }

  /* Check if the TX Buffer unavailable flag is set */

  if ((at32_getreg(AT32_ETH_DMASR) & ETH_DMAINT_TBUI) != 0)
    {
      /* Clear TX Buffer unavailable flag */

      at32_putreg(ETH_DMAINT_TBUI, AT32_ETH_DMASR);

      /* Resume DMA transmission */

      at32_putreg(0, AT32_ETH_DMATPDR);
    }

  /* Enable TX interrupts */

  at32_enableint(priv, ETH_DMAINT_TI);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(&priv->txtimeout, AT32_TXTIMEOUT,
           at32_txtimeout_expiry, (wdparm_t)priv);
  return OK;
}

/****************************************************************************
 * Function: at32_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send. This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int at32_txpoll(struct net_driver_s *dev)
{
  struct at32_ethmac_s *priv =
    (struct at32_ethmac_s *)dev->d_private;

  DEBUGASSERT(priv->dev.d_buf != NULL);

  /* Send the packet */

  at32_transmit(priv);
  DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);

  /* Check if the next TX descriptor is owned by the Ethernet DMA or
   * CPU. We cannot perform the TX poll if we are unable to accept
   * another packet for transmission.
   *
   * In a race condition, ETH_TDES0_OWN may be cleared BUT still
   * not available because at32_freeframe() has not yet run. If
   * at32_freeframe() has run, the buffer1 pointer (tdes2) will be
   * nullified (and inflight should be < CONFIG_AT32_ETH_NTXDESC).
   */

  if ((priv->txhead->tdes0 & ETH_TDES0_OWN) != 0 ||
       priv->txhead->tdes2 != 0)
    {
      /* We have to terminate the poll if we have no more descriptors
       * available for another transfer.
       */

      return -EBUSY;
    }

  /* We have the descriptor, we can continue the poll. Allocate a new
   * buffer for the poll.
   */

  dev->d_buf = at32_allocbuffer(priv);

  /* We can't continue the poll if we have no buffers */

  if (dev->d_buf == NULL)
    {
      /* Terminate the poll. */

      return -ENOMEM;
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: at32_dopoll
 *
 * Description:
 *   The function is called in order to perform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. After completion of a transmission (at32_txdone),
 *   2. When new TX data is available (at32_txavail_process), and
 *   3. After a TX timeout to restart the sending process
 *      (at32_txtimeout_process).
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void at32_dopoll(struct at32_ethmac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;

  /* Check if the next TX descriptor is owned by the Ethernet DMA or
   * CPU.  We cannot perform the TX poll if we are unable to accept
   * another packet for transmission.
   *
   * In a race condition, ETH_TDES0_OWN may be cleared BUT still
   * not available because at32_freeframe() has not yet run. If
   * at32_freeframe() has run, the buffer1 pointer (tdes2) will be
   * nullified (and inflight should be < CONFIG_AT32_ETH_NTXDESC).
   */

  if ((priv->txhead->tdes0 & ETH_TDES0_OWN) == 0 &&
       priv->txhead->tdes2 == 0)
    {
      /* If we have the descriptor, then poll the network for new XMIT data.
       * Allocate a buffer for the poll.
       */

      DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);
      dev->d_buf = at32_allocbuffer(priv);

      /* We can't poll if we have no buffers */

      if (dev->d_buf)
        {
          devif_poll(dev, at32_txpoll);

          /* We will, most likely end up with a buffer to be freed.  But it
           * might not be the same one that we allocated above.
           */

          if (dev->d_buf)
            {
              DEBUGASSERT(dev->d_len == 0);
              at32_freebuffer(priv, dev->d_buf);
              dev->d_buf = NULL;
            }
        }
    }
}

/****************************************************************************
 * Function: at32_enableint
 *
 * Description:
 *   Enable a "normal" interrupt
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void at32_enableint(struct at32_ethmac_s *priv,
                             uint32_t ierbit)
{
  uint32_t regval;

  /* Enable the specified "normal" interrupt */

  regval  = at32_getreg(AT32_ETH_DMAIER);
  regval |= (ETH_DMAINT_NIS | ierbit);
  at32_putreg(regval, AT32_ETH_DMAIER);
}

/****************************************************************************
 * Function: at32_disableint
 *
 * Description:
 *   Disable a normal interrupt.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void at32_disableint(struct at32_ethmac_s *priv,
                             uint32_t ierbit)
{
  uint32_t regval;

  /* Disable the "normal" interrupt */

  regval  = at32_getreg(AT32_ETH_DMAIER);
  regval &= ~ierbit;

  /* Are all "normal" interrupts now disabled? */

  if ((regval & ETH_DMAINT_NORMAL) == 0)
    {
      /* Yes.. disable normal interrupts */

      regval &= ~ETH_DMAINT_NIS;
    }

  at32_putreg(regval, AT32_ETH_DMAIER);
}

/****************************************************************************
 * Function: at32_freesegment
 *
 * Description:
 *   The function is called when a frame is received using the DMA receive
 *   interrupt.  It scans the RX descriptors to the received frame.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void at32_freesegment(struct at32_ethmac_s *priv,
                              struct eth_rxdesc_s *rxfirst, int segments)
{
  struct eth_rxdesc_s *rxdesc;
  int i;

  ninfo("rxfirst: %p segments: %d\n", rxfirst, segments);

  /* Set OWN bit in RX descriptors.  This gives the buffers back to DMA */

  rxdesc = rxfirst;
  for (i = 0; i < segments; i++)
    {
      rxdesc->rdes0 = ETH_RDES0_OWN;
      rxdesc = (struct eth_rxdesc_s *)rxdesc->rdes3;
    }

  /* Reset the segment management logic */

  priv->rxcurr   = NULL;
  priv->segments = 0;

  /* Check if the RX Buffer unavailable flag is set */

  if ((at32_getreg(AT32_ETH_DMASR) & ETH_DMAINT_RBUI) != 0)
    {
      /* Clear RBUS Ethernet DMA flag */

      at32_putreg(ETH_DMAINT_RBUI, AT32_ETH_DMASR);

      /* Resume DMA reception */

      at32_putreg(0, AT32_ETH_DMARPDR);
    }
}

/****************************************************************************
 * Function: at32_recvframe
 *
 * Description:
 *   The function is called when a frame is received using the DMA receive
 *   interrupt.  It scans the RX descriptors of the received frame.
 *
 *   NOTE: This function will silently discard any packets containing errors.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK if a packet was successfully returned; -EAGAIN if there are no
 *   further packets available
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static int at32_recvframe(struct at32_ethmac_s *priv)
{
  struct eth_rxdesc_s *rxdesc;
  struct eth_rxdesc_s *rxcurr;
  uint8_t *buffer;
  int i;

  ninfo("rxhead: %p rxcurr: %p segments: %d\n",
        priv->rxhead, priv->rxcurr, priv->segments);

  /* Check if there are free buffers.  We cannot receive new frames in this
   * design unless there is at least one free buffer.
   */

  if (!at32_isfreebuffer(priv))
    {
      nerr("ERROR: No free buffers\n");
      return -ENOMEM;
    }

  /* Scan descriptors owned by the CPU.  Scan until:
   *
   *   1) We find a descriptor still owned by the DMA,
   *   2) We have examined all of the RX descriptors, or
   *   3) All of the TX descriptors are in flight.
   *
   * This last case is obscure.  It is due to that fact that each packet
   * that we receive can generate an unstoppable transmission.  So we have
   * to stop receiving when we can not longer transmit.  In this case, the
   * transmit logic should also have disabled further RX interrupts.
   */

  rxdesc = priv->rxhead;
  for (i = 0;
       (rxdesc->rdes0 & ETH_RDES0_OWN) == 0 &&
        i < CONFIG_AT32_ETH_NRXDESC &&
        priv->inflight < CONFIG_AT32_ETH_NTXDESC;
       i++)
    {
      /* Check if this is the first segment in the frame */

      if ((rxdesc->rdes0 & ETH_RDES0_FS) != 0 &&
          (rxdesc->rdes0 & ETH_RDES0_LS) == 0)
        {
          priv->rxcurr   = rxdesc;
          priv->segments = 1;
        }

      /* Check if this is an intermediate segment in the frame */

      else if (((rxdesc->rdes0 & ETH_RDES0_LS) == 0) &&
               ((rxdesc->rdes0 & ETH_RDES0_FS) == 0))
        {
          priv->segments++;
        }

      /* Otherwise, it is the last segment in the frame */

      else
        {
          priv->segments++;

          /* Check if there is only one segment in the frame */

          if (priv->segments == 1)
            {
              rxcurr = rxdesc;
            }
          else
            {
              rxcurr = priv->rxcurr;
            }

          ninfo("rxhead: %p rxcurr: %p segments: %d\n",
              priv->rxhead, priv->rxcurr, priv->segments);

          /* Check if any errors are reported in the frame */

          if ((rxdesc->rdes0 & ETH_RDES0_ES) == 0)
            {
              struct net_driver_s *dev = &priv->dev;

              /* Get the Frame Length of the received packet: substruct 4
               * bytes of the CRC
               */

              dev->d_len = ((rxdesc->rdes0 & ETH_RDES0_FL_MASK) >>
                            ETH_RDES0_FL_SHIFT) - 4;

              if (priv->segments > 1 ||
                  dev->d_len > CONFIG_AT32_ETH_BUFSIZE)
                {
                  /* The Frame is to big, it spans segments */

                  nerr("ERROR: Dropped, RX descriptor Too big: %d in %d "
                      "segments\n", dev->d_len, priv->segments);

                  at32_freesegment(priv, rxcurr, priv->segments);
                }

              else
                {
                  /* Get a buffer from the free list.  We don't even check if
                   * this is successful because we already assure the free
                   * list is not empty above.
                   */

                  buffer = at32_allocbuffer(priv);

                  /* Take the buffer from the RX descriptor of the first free
                   * segment, put it into the network device structure, then
                   * replace the buffer in the RX descriptor with the newly
                   * allocated buffer.
                   */

                  DEBUGASSERT(dev->d_buf == NULL);
                  dev->d_buf    = (uint8_t *)rxcurr->rdes2;
                  rxcurr->rdes2 = (uint32_t)buffer;

                  /* Return success, remembering where we should re-start
                   * scanning and resetting the segment scanning logic
                   */

                  priv->rxhead   = (struct eth_rxdesc_s *)rxdesc->rdes3;
                  at32_freesegment(priv, rxcurr, priv->segments);

                  ninfo("rxhead: %p d_buf: %p d_len: %d\n",
                        priv->rxhead, dev->d_buf, dev->d_len);

                  return OK;
                }
            }
          else
            {
              /* Drop the frame that contains the errors, reset the segment
               * scanning logic, and continue scanning with the next frame.
               */

              nerr("ERROR: Dropped, RX descriptor errors: %08" PRIx32 "\n",
                   rxdesc->rdes0);
              at32_freesegment(priv, rxcurr, priv->segments);
            }
        }

      /* Try the next descriptor */

      rxdesc = (struct eth_rxdesc_s *)rxdesc->rdes3;
    }

  /* We get here after all of the descriptors have been scanned or when
   * rxdesc points to the first descriptor owned by the DMA. Remember
   * where we left off.
   */

  priv->rxhead = rxdesc;

  ninfo("rxhead: %p rxcurr: %p segments: %d\n",
        priv->rxhead, priv->rxcurr, priv->segments);

  return -EAGAIN;
}

/****************************************************************************
 * Function: at32_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void at32_receive(struct at32_ethmac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;

  /* Loop while while at32_recvframe() successfully retrieves valid
   * Ethernet frames.
   */

  while (at32_recvframe(priv) == OK)
    {
#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the tap */

     pkt_input(&priv->dev);
#endif

      /* Check if the packet is a valid size for the network buffer
       * configuration (this should not happen)
       */

      if (dev->d_len > CONFIG_NET_ETH_PKTSIZE)
        {
          nwarn("WARNING: DROPPED Too big: %d\n", dev->d_len);

          /* Free dropped packet buffer */

          if (dev->d_buf)
            {
              at32_freebuffer(priv, dev->d_buf);
              dev->d_buf = NULL;
              dev->d_len = 0;
            }

          continue;
        }

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");

          /* Receive an IPv4 packet from the network device */

          ipv4_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              /* And send the packet */

              at32_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("IPv6 frame\n");

          /* Give the IPv6 packet to the network layer */

          ipv6_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              /* And send the packet */

              at32_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (BUF->type == HTONS(ETHTYPE_ARP))
        {
          ninfo("ARP frame\n");

          /* Handle ARP packet */

          arp_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              at32_transmit(priv);
            }
        }
      else
#endif
        {
          nerr("ERROR: Dropped, Unknown type: %04x\n", BUF->type);
        }

      /* We are finished with the RX buffer.  NOTE:  If the buffer is
       * reused for transmission, the dev->d_buf field will have been
       * nullified.
       */

      if (dev->d_buf)
        {
          /* Free the receive packet buffer */

          at32_freebuffer(priv, dev->d_buf);
          dev->d_buf = NULL;
          dev->d_len = 0;
        }
    }
}

/****************************************************************************
 * Function: at32_freeframe
 *
 * Description:
 *   Scans the TX descriptors and frees the buffers of completed transfers.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void at32_freeframe(struct at32_ethmac_s *priv)
{
  struct eth_txdesc_s *txdesc;

  ninfo("txhead: %p txtail: %p inflight: %d\n",
        priv->txhead, priv->txtail, priv->inflight);

  /* Scan for "in-flight" descriptors owned by the CPU */

  txdesc = priv->txtail;
  if (txdesc)
    {
      DEBUGASSERT(priv->inflight > 0);

      while ((txdesc->tdes0 & ETH_TDES0_OWN) == 0)
        {
          /* There should be a buffer assigned to all in-flight
           * TX descriptors.
           */

          ninfo("txtail: %p tdes0: %08" PRIx32
                " tdes2: %08" PRIx32 " tdes3: %08" PRIx32 "\n",
                txdesc, txdesc->tdes0, txdesc->tdes2, txdesc->tdes3);

          DEBUGASSERT(txdesc->tdes2 != 0);

          /* Check if this is the first segment of a TX frame. */

          if ((txdesc->tdes0 & ETH_TDES0_FS) != 0)
            {
              /* Yes.. Free the buffer */

              at32_freebuffer(priv, (uint8_t *)txdesc->tdes2);
            }

          /* In any event, make sure that TDES2 is nullified. */

          txdesc->tdes2 = 0;

          /* Check if this is the last segment of a TX frame */

          if ((txdesc->tdes0 & ETH_TDES0_LS) != 0)
            {
              /* Yes.. Decrement the number of frames "in-flight". */

              priv->inflight--;

              /* If all of the TX descriptors were in-flight,
               * then RX interrupts may have been disabled...
               * we can re-enable them now.
               */

              at32_enableint(priv, ETH_DMAINT_RI);

              /* If there are no more frames in-flight, then bail. */

              if (priv->inflight <= 0)
                {
                  priv->txtail   = NULL;
                  priv->inflight = 0;
                  return;
                }
            }

          /* Try the next descriptor in the TX chain */

          txdesc = (struct eth_txdesc_s *)txdesc->tdes3;
        }

      /* We get here if (1) there are still frames "in-flight". Remember
       * where we left off.
       */

      priv->txtail = txdesc;

      ninfo("txhead: %p txtail: %p inflight: %d\n",
            priv->txhead, priv->txtail, priv->inflight);
    }
}

/****************************************************************************
 * Function: at32_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet
 *   transfer(s) are complete.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void at32_txdone(struct at32_ethmac_s *priv)
{
  DEBUGASSERT(priv->txtail != NULL);

  /* Scan the TX descriptor change, returning buffers to free list */

  at32_freeframe(priv);

  /* If no further xmits are pending, then cancel the TX timeout */

  if (priv->inflight <= 0)
    {
      /* Cancel the TX timeout */

      wd_cancel(&priv->txtimeout);

      /* And disable further TX interrupts. */

      at32_disableint(priv, ETH_DMAINT_TI);
    }

  /* Then poll the network for new XMIT data */

  at32_dopoll(priv);
}

/****************************************************************************
 * Function: at32_interrupt_work
 *
 * Description:
 *   Perform interrupt related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void at32_interrupt_work(void *arg)
{
  struct at32_ethmac_s *priv = (struct at32_ethmac_s *)arg;
  uint32_t dmasr;

  DEBUGASSERT(priv);

  /* Process pending Ethernet interrupts */

  net_lock();

  /* Get the DMA interrupt status bits (no MAC interrupts are expected) */

  dmasr = at32_getreg(AT32_ETH_DMASR);

  /* Mask only enabled interrupts.  This depends on the fact that the
   * interrupt related bits (0-16) correspond in these two registers.
   */

  dmasr &= at32_getreg(AT32_ETH_DMAIER);

  /* Check if there are pending "normal" interrupts */

  if ((dmasr & ETH_DMAINT_NIS) != 0)
    {
      /* Yes.. Check if we received an incoming packet, if so, call
       * at32_receive()
       */

      if ((dmasr & ETH_DMAINT_RI) != 0)
        {
          /* Clear the pending receive interrupt */

          at32_putreg(ETH_DMAINT_RI, AT32_ETH_DMASR);

          /* Handle the received package */

          at32_receive(priv);
        }

      /* Check if a packet transmission just completed.  If so, call
       * at32_txdone(). This may disable further TX interrupts if there
       * are no pending transmissions.
       */

      if ((dmasr & ETH_DMAINT_TI) != 0)
        {
          /* Clear the pending receive interrupt */

          at32_putreg(ETH_DMAINT_TI, AT32_ETH_DMASR);

          /* Check if there are pending transmissions */

          at32_txdone(priv);
        }

      /* Clear the pending normal summary interrupt */

      at32_putreg(ETH_DMAINT_NIS, AT32_ETH_DMASR);
    }

  /* Handle error interrupt only if CONFIG_DEBUG_NET is enabled */

#ifdef CONFIG_DEBUG_NET

  /* Check if there are pending "abnormal" interrupts */

  if ((dmasr & ETH_DMAINT_AIS) != 0)
    {
      /* Just let the user know what happened */

      nerr("ERROR: Abormal event(s): %08x\n", dmasr);

      /* Clear all pending abnormal events */

      at32_putreg(ETH_DMAINT_ABNORMAL, AT32_ETH_DMASR);

      /* Clear the pending abnormal summary interrupt */

      at32_putreg(ETH_DMAINT_AIS, AT32_ETH_DMASR);
    }
#endif

  net_unlock();

  /* Re-enable Ethernet interrupts at the NVIC */

  up_enable_irq(AT32_IRQ_ETH);
}

/****************************************************************************
 * Function: at32_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int at32_interrupt(int irq, void *context, void *arg)
{
  struct at32_ethmac_s *priv = &g_at32ethmac[0];
  uint32_t dmasr;

  /* Get the DMA interrupt status bits (no MAC interrupts are expected) */

  dmasr = at32_getreg(AT32_ETH_DMASR);
  if (dmasr != 0)
    {
      /* Disable further Ethernet interrupts.  Because Ethernet interrupts
       * are also disabled if the TX timeout event occurs, there can be no
       * race condition here.
       */

      up_disable_irq(AT32_IRQ_ETH);

      /* Check if a packet transmission just completed. */

      if ((dmasr & ETH_DMAINT_TI) != 0)
        {
          /* If a TX transfer just completed, then cancel the TX timeout so
           * there will be no race condition between any subsequent timeout
           * expiration and the deferred interrupt processing.
           */

           wd_cancel(&priv->txtimeout);
        }

      /* Schedule to perform the interrupt processing on the worker thread. */

      work_queue(ETHWORK, &priv->irqwork, at32_interrupt_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: at32_txtimeout_work
 *
 * Description:
 *   Perform TX timeout related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void at32_txtimeout_work(void *arg)
{
  struct at32_ethmac_s *priv = (struct at32_ethmac_s *)arg;

  /* Reset the hardware.  Just take the interface down, then back up again. */

  net_lock();
  at32_ifdown(&priv->dev);
  at32_ifup(&priv->dev);

  /* Then poll for new XMIT data */

  at32_dopoll(priv);
  net_unlock();
}

/****************************************************************************
 * Function: at32_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Input Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void at32_txtimeout_expiry(wdparm_t arg)
{
  struct at32_ethmac_s *priv = (struct at32_ethmac_s *)arg;

  nerr("ERROR: Timeout!\n");

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   *
   * Interrupts will be re-enabled when at32_ifup() is called.
   */

  up_disable_irq(AT32_IRQ_ETH);

  /* Schedule to perform the TX timeout processing on the worker thread,
   * perhaps canceling any pending IRQ processing.
   */

  work_queue(ETHWORK, &priv->irqwork, at32_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: at32_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int at32_ifup(struct net_driver_s *dev)
{
  struct at32_ethmac_s *priv =
    (struct at32_ethmac_s *)dev->d_private;
  int ret;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %d.%d.%d.%d\n",
        (int)(dev->d_ipaddr & 0xff),
        (int)((dev->d_ipaddr >> 8) & 0xff),
        (int)((dev->d_ipaddr >> 16) & 0xff),
        (int)(dev->d_ipaddr >> 24));
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  /* Configure the Ethernet interface for DMA operation. */

  ret = at32_ethconfig(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable the Ethernet interrupt */

  priv->ifup = true;
  up_enable_irq(AT32_IRQ_ETH);

  at32_checksetup();
  return OK;
}

/****************************************************************************
 * Function: at32_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   Returns zero on success; a negated errno value is returned on any
 *   failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int at32_ifdown(struct net_driver_s *dev)
{
  struct at32_ethmac_s *priv =
    (struct at32_ethmac_s *)dev->d_private;
  irqstate_t flags;
  int ret = OK;

  ninfo("Taking the network down\n");

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(AT32_IRQ_ETH);

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the at32_ifup() always
   * successfully brings the interface back up.
   */

  ret = at32_ethreset(priv);
  if (ret < 0)
    {
      nerr("ERROR: at32_ethreset failed (timeout), "
           "still assuming it's going down.\n");
    }

  /* Mark the device "down" */

  priv->ifup = false;
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Function: at32_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg  - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void at32_txavail_work(void *arg)
{
  struct at32_ethmac_s *priv = (struct at32_ethmac_s *)arg;

  ninfo("ifup: %d\n", priv->ifup);

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->ifup)
    {
      /* Poll the network for new XMIT data */

      at32_dopoll(priv);
    }

  net_unlock();
}

/****************************************************************************
 * Function: at32_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int at32_txavail(struct net_driver_s *dev)
{
  struct at32_ethmac_s *priv =
    (struct at32_ethmac_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->pollwork, at32_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: at32_calcethcrc
 *
 * Description:
 *   Function to calculate the CRC used by AT32 to check an ethernet frame
 *
 * Input Parameters:
 *   data   - the data to be checked
 *   length - length of the data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static uint32_t at32_calcethcrc(const uint8_t *data, size_t length)
{
  uint32_t crc = 0xffffffff;
  size_t i;
  int j;

  for (i = 0; i < length; i++)
    {
      for (j = 0; j < 8; j++)
        {
          if (((crc >> 31) ^ (data[i] >> j)) & 0x01)
            {
              /* x^26+x^23+x^22+x^16+x^12+x^11+x^10+x^8+x^7+x^5+x^4+x^2+x+1 */

              crc = (crc << 1) ^ 0x04c11db7;
            }
          else
            {
              crc = crc << 1;
            }
        }
    }

  return ~crc;
}
#endif

/****************************************************************************
 * Function: at32_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int at32_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  uint32_t crc;
  uint32_t hashindex;
  uint32_t temp;
  uint32_t registeraddress;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Add the MAC address to the hardware multicast hash table */

  crc = at32_calcethcrc(mac, 6);

  hashindex = (crc >> 26) & 0x3f;

  if (hashindex > 31)
    {
      registeraddress = AT32_ETH_MACHTHR;
      hashindex -= 32;
    }
  else
    {
      registeraddress = AT32_ETH_MACHTLR;
    }

  temp = at32_getreg(registeraddress);
  temp |= 1 << hashindex;
  at32_putreg(temp, registeraddress);

  temp = at32_getreg(AT32_ETH_MACFFR);
  temp |= (ETH_MACFFR_HM | ETH_MACFFR_HPF);
  at32_putreg(temp, AT32_ETH_MACFFR);

  return OK;
}
#endif /* CONFIG_NET_MCASTGROUP || CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Function: at32_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware
 *   multicast address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int at32_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  uint32_t crc;
  uint32_t hashindex;
  uint32_t temp;
  uint32_t registeraddress;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Remove the MAC address to the hardware multicast hash table */

  crc = at32_calcethcrc(mac, 6);

  hashindex = (crc >> 26) & 0x3f;

  if (hashindex > 31)
    {
      registeraddress = AT32_ETH_MACHTHR;
      hashindex -= 32;
    }
  else
    {
      registeraddress = AT32_ETH_MACHTLR;
    }

  temp = at32_getreg(registeraddress);
  temp &= ~(1 << hashindex);
  at32_putreg(temp, registeraddress);

  /* If there is no address registered any more, delete multicast filtering */

  if (at32_getreg(AT32_ETH_MACHTHR) == 0 &&
      at32_getreg(AT32_ETH_MACHTLR) == 0)
    {
      temp = at32_getreg(AT32_ETH_MACFFR);
      temp &= ~(ETH_MACFFR_HM | ETH_MACFFR_HPF);
      at32_putreg(temp, AT32_ETH_MACFFR);
    }

  return OK;
}
#endif

/****************************************************************************
 * Function: at32_txdescinit
 *
 * Description:
 *   Initializes the DMA TX descriptors in chain mode.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void at32_txdescinit(struct at32_ethmac_s *priv,
                             struct eth_txdesc_s *txtable)
{
  struct eth_txdesc_s *txdesc;
  int i;

  /* priv->txhead point to the first, available TX descriptor in the chain.
   * Set the priv->txhead pointer to the first descriptor in the table.
   */

  priv->txhead = txtable;

  /* priv->txtail will point to the first segment of the oldest pending
   * "in-flight" TX transfer.  NULL means that there are no active TX
   * transfers.
   */

  priv->txtail   = NULL;
  priv->inflight = 0;

  /* Initialize each TX descriptor */

  for (i = 0; i < CONFIG_AT32_ETH_NTXDESC; i++)
    {
      txdesc = &txtable[i];

      /* Set Second Address Chained bit */

      txdesc->tdes0 = ETH_TDES0_TCH;

#ifdef CHECKSUM_BY_HARDWARE
      /* Enable the checksum insertion for the TX frames */

      txdesc->tdes0 |= ETH_TDES0_CIC_ALL;
#endif

      /* Clear Buffer1 address pointer (buffers will be assigned as they
       * are used)
       */

      txdesc->tdes2 = 0;

      /* Initialize the next descriptor with
       * the Next Descriptor Polling Enable
       */

      if (i < (CONFIG_AT32_ETH_NTXDESC - 1))
        {
          /* Set next descriptor address register with next descriptor base
           * address
           */

          txdesc->tdes3 = (uint32_t)&txtable[i + 1];
        }
      else
        {
          /* For last descriptor, set next descriptor address register equal
           * to the first descriptor base address
           */

          txdesc->tdes3 = (uint32_t)txtable;
        }
    }

  /* Set Transmit Descriptor List Address Register */

  at32_putreg((uint32_t)txtable, AT32_ETH_DMATDLAR);
}

/****************************************************************************
 * Function: at32_rxdescinit
 *
 * Description:
 *   Initializes the DMA RX descriptors in chain mode.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void at32_rxdescinit(struct at32_ethmac_s *priv,
                             struct eth_rxdesc_s *rxtable,
                             uint8_t *rxbuffer)
{
  struct eth_rxdesc_s *rxdesc;
  int i;

  /* priv->rxhead will point to the first,  RX descriptor in the chain.
   * This will be where we receive the first incomplete frame.
   */

  priv->rxhead = rxtable;

  /* If we accumulate the frame in segments, priv->rxcurr points to the
   * RX descriptor of the first segment in the current TX frame.
   */

  priv->rxcurr   = NULL;
  priv->segments = 0;

  /* Initialize each TX descriptor */

  for (i = 0; i < CONFIG_AT32_ETH_NRXDESC; i++)
    {
      rxdesc = &rxtable[i];

      /* Set Own bit of the RX descriptor rdes0 */

      rxdesc->rdes0 = ETH_RDES0_OWN;

      /* Set Buffer1 size and Second Address Chained bit and enabled DMA
       * RX desc receive interrupt
       */

      rxdesc->rdes1 = ETH_RDES1_RCH | (uint32_t)CONFIG_AT32_ETH_BUFSIZE;

      /* Set Buffer1 address pointer */

      rxdesc->rdes2 = (uint32_t)&rxbuffer[i * CONFIG_AT32_ETH_BUFSIZE];

      /* Initialize the next descriptor with
       * the Next Descriptor Polling Enable
       */

      if (i < (CONFIG_AT32_ETH_NRXDESC - 1))
        {
          /* Set next descriptor address register with next descriptor base
           * address
           */

          rxdesc->rdes3 = (uint32_t)&rxtable[i + 1];
        }
      else
        {
          /* For last descriptor, set next descriptor address register equal
           * to the first descriptor base address
           */

          rxdesc->rdes3 = (uint32_t)rxtable;
        }
    }

  /* Set Receive Descriptor List Address Register */

  at32_putreg((uint32_t)rxtable, AT32_ETH_DMARDLAR);
}

/****************************************************************************
 * Function: at32_ioctl
 *
 * Description:
 *  Executes the SIOCxMIIxxx command and responds using the request struct
 *  that must be provided as its 2nd parameter.
 *
 *  When called with SIOCGMIIPHY it will get the PHY address for the device
 *  and write it to the req->phy_id field of the request struct.
 *
 *  When called with SIOCGMIIREG it will read a register of the PHY that is
 *  specified using the req->reg_no struct field and then write its output
 *  to the req->val_out field.
 *
 *  When called with SIOCSMIIREG it will write to a register of the PHY that
 *  is specified using the req->reg_no struct field and use req->val_in as
 *  its input.
 *
 * Input Parameters:
 *   dev - Ethernet device structure
 *   cmd - SIOCxMIIxxx command code
 *   arg - Request structure also used to return values
 *
 * Returned Value: Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int at32_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg)
{
#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
  struct at32_ethmac_s *priv =
    (struct at32_ethmac_s *)dev->d_private;
#endif
  int ret;

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_PHY_IOCTL
#ifdef CONFIG_ARCH_PHY_INTERRUPT
      case SIOCMIINOTIFY: /* Set up for PHY event notifications */
        {
          struct mii_ioctl_notify_s *req =
        (struct mii_ioctl_notify_s *)((uintptr_t)arg);

          ret = phy_notify_subscribe(dev->d_ifname, req->pid, &req->event);
          if (ret == OK)
            {
              /* Enable PHY link up/down interrupts */

              ret = at32_phyintenable(priv);
            }
        }
        break;
#endif

      case SIOCGMIIPHY: /* Get MII PHY address */
        {
          struct mii_ioctl_data_s *req =
        (struct mii_ioctl_data_s *)((uintptr_t)arg);
          req->phy_id = CONFIG_AT32_PHYADDR;
          ret = OK;
        }
        break;

      case SIOCGMIIREG: /* Get register from MII PHY */
        {
          struct mii_ioctl_data_s *req =
        (struct mii_ioctl_data_s *)((uintptr_t)arg);
          ret = at32_phyread(req->phy_id, req->reg_num, &req->val_out);
        }
        break;

      case SIOCSMIIREG: /* Set register in MII PHY */
        {
          struct mii_ioctl_data_s *req =
        (struct mii_ioctl_data_s *)((uintptr_t)arg);
          ret = at32_phywrite(req->phy_id, req->reg_num, req->val_in);
        }
        break;
#endif /* CONFIG_NETDEV_PHY_IOCTL */

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Function: at32_phyintenable
 *
 * Description:
 *  Enable link up/down PHY interrupts.  The interrupt protocol is like this:
 *
 *  - Interrupt status is cleared when the interrupt is enabled.
 *  - Interrupt occurs.  Interrupt is disabled (at the processor level) when
 *    is received.
 *  - Interrupt status is cleared when the interrupt is re-enabled.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno (-ETIMEDOUT) on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
static int at32_phyintenable(struct at32_ethmac_s *priv)
{
  uint16_t phyval;
  int ret;

  ret = at32_phyread(CONFIG_AT32_PHYADDR, MII_INT_REG, &phyval);
  if (ret == OK)
    {
      /* Enable link up/down interrupts */

#ifdef CONFIG_ETH0_PHY_DP83848C
      ret = at32_phywrite(CONFIG_AT32_PHYADDR, MII_DP83848C_MICR,
                           MII_DP83848C_INT_EN | MII_DP83848C_INT_OEN);
#endif
      ret = at32_phywrite(CONFIG_AT32_PHYADDR, MII_INT_REG,
                           (phyval & ~MII_INT_CLREN) | MII_INT_SETEN);
    }

  return ret;
}
#endif

/****************************************************************************
 * Function: at32_phyread
 *
 * Description:
 *  Read a PHY register.
 *
 * Input Parameters:
 *   phydevaddr - The PHY device address
 *   phyregaddr - The PHY register address
 *   value - The location to return the 16-bit PHY register value.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_AT32_AUTONEG) || defined(CONFIG_NETDEV_PHY_IOCTL) || \
    defined(CONFIG_ETH0_PHY_DM9161)
static int at32_phyread(uint16_t phydevaddr,
                         uint16_t phyregaddr, uint16_t *value)
{
  volatile uint32_t timeout;
  uint32_t regval;

  regval = at32_getreg(AT32_ETH_MACMIIAR);

  /* Clear the busy bit before accessing the MACMIIAR register. */

  regval &= ~ETH_MACMIIAR_MB;
  at32_putreg(regval, AT32_ETH_MACMIIAR);

  /* Configure the MACMIIAR register,
   * preserving CSR Clock Range CR[2:0] bits
   */

  regval &= ETH_MACMIIAR_CR_MASK;

  /* Set the PHY device address, PHY register address, and set the busy bit.
   * the ETH_MACMIIAR_MW is clear, indicating a read operation.
   */

  regval |= (phydevaddr << ETH_MACMIIAR_PA_SHIFT) & ETH_MACMIIAR_PA_MASK;
  regval |= (phyregaddr << ETH_MACMIIAR_MR_SHIFT) & ETH_MACMIIAR_MR_MASK;
  regval |= ETH_MACMIIAR_MB;

  at32_putreg(regval, AT32_ETH_MACMIIAR);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < PHY_READ_TIMEOUT; timeout++)
    {
      if ((at32_getreg(AT32_ETH_MACMIIAR) & ETH_MACMIIAR_MB) == 0)
        {
          *value = (uint16_t)at32_getreg(AT32_ETH_MACMIIDR);
          return OK;
        }
    }

  nerr("ERROR: MII transfer timed out: phydevaddr: %04x phyregaddr: %04x\n",
       phydevaddr, phyregaddr);

  return -ETIMEDOUT;
}
#endif

/****************************************************************************
 * Function: at32_phywrite
 *
 * Description:
 *  Write to a PHY register.
 *
 * Input Parameters:
 *   phydevaddr - The PHY device address
 *   phyregaddr - The PHY register address
 *   value - The 16-bit value to write to the PHY register value.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int at32_phywrite(uint16_t phydevaddr,
                          uint16_t phyregaddr, uint16_t value)
{
  volatile uint32_t timeout;
  uint32_t regval;

  regval = at32_getreg(AT32_ETH_MACMIIAR);

  /* Clear the busy bit before accessing the MACMIIAR register. */

  regval &= ~ETH_MACMIIAR_MB;
  at32_putreg(regval, AT32_ETH_MACMIIAR);

  /* Configure the MACMIIAR register,
   * preserving CSR Clock Range CR[2:0] bits
   */

  regval &= ETH_MACMIIAR_CR_MASK;

  /* Set the PHY device address, PHY register address, and set the busy bit.
   * the ETH_MACMIIAR_MW is set, indicating a write operation.
   */

  regval |= (phydevaddr << ETH_MACMIIAR_PA_SHIFT) & ETH_MACMIIAR_PA_MASK;
  regval |= (phyregaddr << ETH_MACMIIAR_MR_SHIFT) & ETH_MACMIIAR_MR_MASK;
  regval |= (ETH_MACMIIAR_MB | ETH_MACMIIAR_MW);

  /* Write the value into the MACIIDR register before setting the new
   * MACMIIAR register value.
   */

  at32_putreg(value, AT32_ETH_MACMIIDR);
  at32_putreg(regval, AT32_ETH_MACMIIAR);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < PHY_WRITE_TIMEOUT; timeout++)
    {
      if ((at32_getreg(AT32_ETH_MACMIIAR) & ETH_MACMIIAR_MB) == 0)
        {
          return OK;
        }
    }

  nerr("ERROR: MII transfer timed out: "
       "phydevaddr: %04x phyregaddr: %04x value: %04x\n",
       phydevaddr, phyregaddr, value);

  return -ETIMEDOUT;
}

/****************************************************************************
 * Function: at32_dm9161
 *
 * Description:
 *   Special workaround for the Davicom DM9161 PHY is required.  On power,
 *   up, the PHY is not usually configured correctly but will work after
 *   a powered-up reset.  This is really a workaround for some more
 *   fundamental issue with the PHY clocking initialization, but the
 *   root cause has not been studied (nor will it be with this workaround).
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ETH0_PHY_DM9161
static inline int at32_dm9161(struct at32_ethmac_s *priv)
{
  uint16_t phyval;
  int ret;

  /* Read the PHYID1 register;  A failure to read the PHY ID is one
   * indication that check if the DM9161 PHY CHIP is not ready.
   */

  ret = at32_phyread(CONFIG_AT32_PHYADDR, MII_PHYID1, &phyval);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read the PHY ID1: %d\n", ret);
      return ret;
    }

  /* If we failed to read the PHY ID1 register,
   * then reset the MCU to recover
   */

  else if (phyval == 0xffff)
    {
      up_systemreset();
    }

  ninfo("PHY ID1: 0x%04X\n", phyval);

  /* Now check the "DAVICOM Specified Configuration Register (DSCR)"(16) */

  ret = at32_phyread(CONFIG_AT32_PHYADDR, 16, &phyval);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read the PHY Register 0x10: %d\n", ret);
      return ret;
    }

  /* Bit 8 of the DSCR register is zero, then the DM9161 has not selected
   * RMII. If RMII is not selected, then reset the MCU to recover.
   */

  else if ((phyval & (1 << 8)) == 0)
    {
      up_systemreset();
    }

  return OK;
}
#endif

/****************************************************************************
 * Function: at32_phyinit
 *
 * Description:
 *  Configure the PHY and determine the link speed/duplex.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int at32_phyinit(struct at32_ethmac_s *priv)
{
#ifdef CONFIG_AT32_AUTONEG
  volatile uint32_t timeout;
#endif

  uint32_t regval;
  uint16_t phyval;
  int ret;

  /* Assume 10MBps and half duplex */

  priv->mbps100 = 0;
  priv->fduplex = 0;

  /* Setup up PHY clocking by setting the SR field in the MACMIIAR register */

  regval  = at32_getreg(AT32_ETH_MACMIIAR);
  regval &= ~ETH_MACMIIAR_CR_MASK;
  regval |= ETH_MACMIIAR_CR;
  at32_putreg(regval, AT32_ETH_MACMIIAR);

  /* Put the PHY in reset mode */

  ret = at32_phywrite(CONFIG_AT32_PHYADDR, MII_MCR, MII_MCR_RESET);
  if (ret < 0)
    {
      nerr("ERROR: Failed to reset the PHY: %d\n", ret);
      return ret;
    }

  up_mdelay(PHY_RESET_DELAY);

  /* Perform any necessary, board-specific PHY initialization */

#ifdef CONFIG_AT32_PHYINIT
  ret = at32_phy_boardinitialize(0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to initialize the PHY: %d\n", ret);
      return ret;
    }
#endif

  /* Special workaround for the Davicom DM9161 PHY is required. */

#ifdef CONFIG_ETH0_PHY_DM9161
  ret = at32_dm9161(priv);
  if (ret < 0)
    {
      return ret;
    }
#endif

  /* Perform auto-negotiation if so configured */

#ifdef CONFIG_AT32_AUTONEG
  /* Wait for link status */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = at32_phyread(CONFIG_AT32_PHYADDR, MII_MSR, &phyval);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read the PHY MSR: %d\n", ret);
          return ret;
        }
      else if ((phyval & MII_MSR_LINKSTATUS) != 0)
        {
          break;
        }
    }

  if (timeout >= PHY_RETRY_TIMEOUT)
    {
      nerr("ERROR: Timed out waiting for link status: %04x\n", phyval);
      return -ETIMEDOUT;
    }

  /* Enable auto-gegotiation */

  ret = at32_phywrite(CONFIG_AT32_PHYADDR, MII_MCR, MII_MCR_ANENABLE);
  if (ret < 0)
    {
      nerr("ERROR: Failed to enable auto-negotiation: %d\n", ret);
      return ret;
    }

  /* Wait until auto-negotiation completes */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = at32_phyread(CONFIG_AT32_PHYADDR, MII_MSR, &phyval);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read the PHY MSR: %d\n", ret);
          return ret;
        }
      else if ((phyval & MII_MSR_ANEGCOMPLETE) != 0)
        {
          break;
        }
    }

  if (timeout >= PHY_RETRY_TIMEOUT)
    {
      nerr("ERROR: Timed out waiting for auto-negotiation\n");
      return -ETIMEDOUT;
    }

  /* Read the result of the auto-negotiation from the PHY-specific register */

  ret = at32_phyread(CONFIG_AT32_PHYADDR, CONFIG_AT32_PHYSR, &phyval);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read PHY status register\n");
      return ret;
    }

  /* Remember the selected speed and duplex modes */

  ninfo("PHYSR[%d]: %04x\n", CONFIG_AT32_PHYSR, phyval);

  /* Different PHYs present speed and mode information in different ways.
   * IF This CONFIG_AT32_PHYSR_ALTCONFIG is selected, this indicates that
   * the PHY represents speed and mode information are combined, for example,
   * with separate bits for 10HD, 100HD, 10FD and 100FD.
   */

#ifdef CONFIG_AT32_PHYSR_ALTCONFIG
  switch (phyval & CONFIG_AT32_PHYSR_ALTMODE)
    {
      default:
      case CONFIG_AT32_PHYSR_10HD:
        priv->fduplex = 0;
        priv->mbps100 = 0;
        break;

      case CONFIG_AT32_PHYSR_100HD:
        priv->fduplex = 0;
        priv->mbps100 = 1;
        break;

      case CONFIG_AT32_PHYSR_10FD:
        priv->fduplex = 1;
        priv->mbps100 = 0;
        break;

      case CONFIG_AT32_PHYSR_100FD:
        priv->fduplex = 1;
        priv->mbps100 = 1;
        break;
    }

  /* Different PHYs present speed and mode information in different ways.
   * Some will present separate information for speed and mode (this is the
   * default). Those PHYs, for example, may provide a 10/100 Mbps indication
   * and a separate full/half duplex indication.
   */

#else
  if ((phyval & CONFIG_AT32_PHYSR_MODE) == CONFIG_AT32_PHYSR_FULLDUPLEX)
    {
      priv->fduplex = 1;
    }

  if ((phyval & CONFIG_AT32_PHYSR_SPEED) == CONFIG_AT32_PHYSR_100MBPS)
    {
      priv->mbps100 = 1;
    }
#endif

#else /* Auto-negotiation not selected */

  phyval = 0;
#ifdef CONFIG_AT32_ETHFD
  phyval |= MII_MCR_FULLDPLX;
#endif
#ifdef CONFIG_AT32_ETH100MBPS
  phyval |= MII_MCR_SPEED100;
#endif

  ret = at32_phywrite(CONFIG_AT32_PHYADDR, MII_MCR, phyval);
  if (ret < 0)
    {
     nerr("ERROR: Failed to write the PHY MCR: %d\n", ret);
      return ret;
    }

  up_mdelay(PHY_CONFIG_DELAY);

  /* Remember the selected speed and duplex modes */

#ifdef CONFIG_AT32_ETHFD
  priv->fduplex = 1;
#endif
#ifdef CONFIG_AT32_ETH100MBPS
  priv->mbps100 = 1;
#endif
#endif

  ninfo("Duplex: %s Speed: %d MBps\n",
        priv->fduplex ? "FULL" : "HALF",
        priv->mbps100 ? 100 : 10);

  return OK;
}

/****************************************************************************
 * Name: at32_selectmii
 *
 * Description:
 *   Selects the MII interface.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_MII
static inline void at32_selectmii(void)
{
  uint32_t regval;

#ifdef CONFIG_AT32_CONNECTIVITYLINE
  regval  = getreg32(AT32_AFIO_MAPR);
  regval &= ~AFIO_MAPR_MII_RMII_SEL;
  putreg32(regval, AT32_AFIO_MAPR);
#else
  regval  = getreg32(AT32_SCFG_CFG2);
  regval &= ~SCFG_CFG2_MII_RMII_SEL;
  putreg32(regval, AT32_SCFG_CFG2);
#endif
}
#endif

/****************************************************************************
 * Name: at32_selectrmii
 *
 * Description:
 *   Selects the RMII interface.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_RMII
static inline void at32_selectrmii(void)
{
  uint32_t regval;

#ifdef CONFIG_AT32_CONNECTIVITYLINE
  regval  = getreg32(AT32_AFIO_MAPR);
  regval |= AFIO_MAPR_MII_RMII_SEL;
  putreg32(regval, AT32_AFIO_MAPR);
#else
  regval  = getreg32(AT32_SCFG_CFG2);
  regval |= SCFG_CFG2_MII_RMII_SEL;
  putreg32(regval, AT32_SCFG_CFG2);
#endif
}
#endif

/****************************************************************************
 * Function: at32_ethgpioconfig
 *
 * Description:
 *  Configure GPIOs for the Ethernet interface.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void at32_ethgpioconfig(struct at32_ethmac_s *priv)
{
  /* Configure GPIO pins to support Ethernet */

#if defined(CONFIG_AT32_MII) || defined(CONFIG_AT32_RMII)

  /* MDC and MDIO are common to both modes */

  at32_configgpio(GPIO_ETH_MDC);
  at32_configgpio(GPIO_ETH_MDIO);

  /* Set up the MII interface */

#if defined(CONFIG_AT32_MII)

  /* Select the MII interface */

  at32_selectmii();

  /* Provide clocking via MCO, MCO1 or MCO2:
   *
   * "MCO1 (microcontroller clock output), used to output HSI, LSE, HSE or
   *  PLL clock (through a configurable prescaler) on PA8 pin."
   *
   * "MCO2 (microcontroller clock output), used to output HSE, PLL, SYSCLK or
   *  PLLI2S clock (through a configurable prescaler) on PC9 pin."
   */

# if defined(CONFIG_AT32_MII_MCO1)
  /* Configure MC01 to drive the PHY.  Board logic must provide MC01 clocking
   * info.
   */

  at32_configgpio(GPIO_MCO1);
  at32_mco1config(BOARD_CFGR_MC01_SOURCE, BOARD_CFGR_MC01_DIVIDER);

# elif defined(CONFIG_AT32_MII_MCO2)
  /* Configure MC02 to drive the PHY.  Board logic must provide MC02 clocking
   * info.
   */

  at32_configgpio(GPIO_MCO2);
  at32_mco2config(BOARD_CFGR_MC02_SOURCE, BOARD_CFGR_MC02_DIVIDER);

# elif defined(CONFIG_AT32_MII_MCO)
  /* Setup MCO pin for alternative usage */

  at32_configgpio(GPIO_MCO);
  at32_mcoconfig(BOARD_CFGR_MCO_SOURCE);
# endif

  /* MII interface pins (17):
   *
   * MII_TX_CLK, MII_TXD[3:0], MII_TX_EN, MII_RX_CLK, MII_RXD[3:0],
   * MII_RX_ER, MII_RX_DV, MII_CRS, MII_COL, MDC, MDIO
   */

  at32_configgpio(GPIO_ETH_MII_COL);
  at32_configgpio(GPIO_ETH_MII_CRS);
  at32_configgpio(GPIO_ETH_MII_RXD0);
  at32_configgpio(GPIO_ETH_MII_RXD1);
  at32_configgpio(GPIO_ETH_MII_RXD2);
  at32_configgpio(GPIO_ETH_MII_RXD3);
  at32_configgpio(GPIO_ETH_MII_RX_CLK);
  at32_configgpio(GPIO_ETH_MII_RX_DV);
  at32_configgpio(GPIO_ETH_MII_RX_ER);
  at32_configgpio(GPIO_ETH_MII_TXD0);
  at32_configgpio(GPIO_ETH_MII_TXD1);
  at32_configgpio(GPIO_ETH_MII_TXD2);
  at32_configgpio(GPIO_ETH_MII_TXD3);
  at32_configgpio(GPIO_ETH_MII_TX_CLK);
  at32_configgpio(GPIO_ETH_MII_TX_EN);

  /* Set up the RMII interface. */

#elif defined(CONFIG_AT32_RMII)

  /* Select the RMII interface */

  at32_selectrmii();

  /* Provide clocking via MCO, MCO1 or MCO2:
   *
   * "MCO1 (microcontroller clock output), used to output HSI, LSE, HSE or
   *  PLL clock (through a configurable prescaler) on PA8 pin."
   *
   * "MCO2 (microcontroller clock output), used to output HSE, PLL, SYSCLK or
   *  PLLI2S clock (through a configurable prescaler) on PC9 pin."
   */

# if defined(CONFIG_AT32_RMII_MCO1)
  /* Configure MC01 to drive the PHY.  Board logic must provide MC01 clocking
   * info.
   */

  at32_configgpio(GPIO_MCO1);
  at32_mco1config(BOARD_CFGR_MC01_SOURCE, BOARD_CFGR_MC01_DIVIDER);

# elif defined(CONFIG_AT32_RMII_MCO2)
  /* Configure MC02 to drive the PHY.  Board logic must provide MC02 clocking
   * info.
   */

  at32_configgpio(GPIO_MCO2);
  at32_mco2config(BOARD_CFGR_MC02_SOURCE, BOARD_CFGR_MC02_DIVIDER);

# elif defined(CONFIG_AT32_RMII_MCO)
  /* Setup MCO pin for alternative usage */

  at32_configgpio(GPIO_MCO);
  at32_mcoconfig(BOARD_CFGR_MCO_SOURCE);
# endif

  /* RMII interface pins (7):
   *
   * RMII_TXD[1:0], RMII_TX_EN, RMII_RXD[1:0], RMII_CRS_DV, MDC, MDIO,
   * RMII_REF_CLK
   */

  at32_configgpio(GPIO_ETH_RMII_CRS_DV);
  at32_configgpio(GPIO_ETH_RMII_REF_CLK);
  at32_configgpio(GPIO_ETH_RMII_RXD0);
  at32_configgpio(GPIO_ETH_RMII_RXD1);
  at32_configgpio(GPIO_ETH_RMII_TXD0);
  at32_configgpio(GPIO_ETH_RMII_TXD1);
  at32_configgpio(GPIO_ETH_RMII_TX_EN);

#endif
#endif

#ifdef CONFIG_AT32_ETH_PTP
  /* Enable pulse-per-second (PPS) output signal */

  at32_configgpio(GPIO_ETH_PPS_OUT);
#endif
}

/****************************************************************************
 * Function: at32_ethreset
 *
 * Description:
 *  Reset the Ethernet block.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   Zero on success, or a negated errno value on any failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int at32_ethreset(struct at32_ethmac_s *priv)
{
  uint32_t regval;
  uint32_t retries;

#if defined(CONFIG_AT32_CONNECTIVITYLINE)
  regval  = at32_getreg(AT32_RCC_AHBRSTR);
  regval |= RCC_AHBRSTR_ETHMACRST;
  at32_putreg(regval, AT32_RCC_AHBRSTR);

  regval &= ~RCC_AHBRSTR_ETHMACRST;
  at32_putreg(regval, AT32_RCC_AHBRSTR);
#else
  regval  = at32_getreg(AT32_CRM_AHBRST1);
  regval |= CRM_AHBRST1_EMACRST;
  at32_putreg(regval, AT32_CRM_AHBRST1);

  regval &= ~CRM_AHBRST1_EMACRST;
  at32_putreg(regval, AT32_CRM_AHBRST1);
#endif

  /* Perform a software reset by setting the SR bit in the DMABMR register.
   * This Resets all MAC subsystem internal registers and logic.  After this
   * reset all the registers holds their reset values.
   */

  regval  = at32_getreg(AT32_ETH_DMABMR);
  regval |= ETH_DMABMR_SR;
  at32_putreg(regval, AT32_ETH_DMABMR);

  /* Wait for software reset to complete. The SR bit is cleared automatically
   * after the reset operation has completed in all core clock domains.
   */

  retries = 100;
  while (((at32_getreg(AT32_ETH_DMABMR) & ETH_DMABMR_SR) != 0) &&
         retries > 0)
    {
      retries--;
      up_mdelay(10);
    }

  if (retries == 0)
    {
      return -ETIMEDOUT;
    }

  return 0;
}

/****************************************************************************
 * Function: at32_macconfig
 *
 * Description:
 *  Configure the Ethernet MAC for DMA operation.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int at32_macconfig(struct at32_ethmac_s *priv)
{
  uint32_t regval;

  /* Set up the MACCR register */

  regval  = at32_getreg(AT32_ETH_MACCR);
  regval &= ~MACCR_CLEAR_BITS;
  regval |= MACCR_SET_BITS;

  if (priv->fduplex)
    {
      /* Set the DM bit for full duplex support */

      regval |= ETH_MACCR_DM;
    }

  if (priv->mbps100)
    {
      /* Set the FES bit for 100Mbps fast ethernet support */

      regval |= ETH_MACCR_FES;
    }

  at32_putreg(regval, AT32_ETH_MACCR);

  /* Set up the MACFFR register */

  regval  = at32_getreg(AT32_ETH_MACFFR);
  regval &= ~MACFFR_CLEAR_BITS;
  regval |= MACFFR_SET_BITS;
  at32_putreg(regval, AT32_ETH_MACFFR);

  /* Set up the MACHTHR and MACHTLR registers */

  at32_putreg(0, AT32_ETH_MACHTHR);
  at32_putreg(0, AT32_ETH_MACHTLR);

  /* Setup up the MACFCR register */

  regval  = at32_getreg(AT32_ETH_MACFCR);
  regval &= ~MACFCR_CLEAR_MASK;
  regval |= MACFCR_SET_MASK;
  at32_putreg(regval, AT32_ETH_MACFCR);

  /* Setup up the MACVLANTR register */

  at32_putreg(0, AT32_ETH_MACVLANTR);

  /* DMA Configuration */

  /* Set up the DMAOMR register */

  regval  = at32_getreg(AT32_ETH_DMAOMR);
  regval &= ~DMAOMR_CLEAR_MASK;
  regval |= DMAOMR_SET_MASK;
  at32_putreg(regval, AT32_ETH_DMAOMR);

  /* Set up the DMABMR register */

  regval  = at32_getreg(AT32_ETH_DMABMR);
  regval &= ~DMABMR_CLEAR_MASK;
  regval |= DMABMR_SET_MASK;
  at32_putreg(regval, AT32_ETH_DMABMR);

  return OK;
}

/****************************************************************************
 * Function: at32_macaddress
 *
 * Description:
 *   Configure the selected MAC address.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void at32_macaddress(struct at32_ethmac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;
  uint32_t regval;

  ninfo("%s MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        dev->d_ifname,
        dev->d_mac.ether.ether_addr_octet[0],
        dev->d_mac.ether.ether_addr_octet[1],
        dev->d_mac.ether.ether_addr_octet[2],
        dev->d_mac.ether.ether_addr_octet[3],
        dev->d_mac.ether.ether_addr_octet[4],
        dev->d_mac.ether.ether_addr_octet[5]);

  /* Set the MAC address high register */

  regval = ((uint32_t)dev->d_mac.ether.ether_addr_octet[5] << 8) |
            (uint32_t)dev->d_mac.ether.ether_addr_octet[4];
  at32_putreg(regval, AT32_ETH_MACA0HR);

  /* Set the MAC address low register */

  regval = ((uint32_t)dev->d_mac.ether.ether_addr_octet[3] << 24) |
           ((uint32_t)dev->d_mac.ether.ether_addr_octet[2] << 16) |
           ((uint32_t)dev->d_mac.ether.ether_addr_octet[1] <<  8) |
            (uint32_t)dev->d_mac.ether.ether_addr_octet[0];
  at32_putreg(regval, AT32_ETH_MACA0LR);
}

/****************************************************************************
 * Function: at32_macenable
 *
 * Description:
 *  Enable normal MAC operation.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int at32_macenable(struct at32_ethmac_s *priv)
{
  uint32_t regval;

  /* Set the MAC address */

  at32_macaddress(priv);

  /* Enable transmit state machine of the MAC for transmission on the MII */

  regval  = at32_getreg(AT32_ETH_MACCR);
  regval |= ETH_MACCR_TE;
  at32_putreg(regval, AT32_ETH_MACCR);

  /* Flush Transmit FIFO */

  regval  = at32_getreg(AT32_ETH_DMAOMR);
  regval |= ETH_DMAOMR_FTF;
  at32_putreg(regval, AT32_ETH_DMAOMR);

  /* Enable receive state machine of the MAC for reception from the MII */

  /* Enables or disables the MAC reception. */

  regval  = at32_getreg(AT32_ETH_MACCR);
  regval |= ETH_MACCR_RE;
  at32_putreg(regval, AT32_ETH_MACCR);

  /* Start DMA transmission */

  regval  = at32_getreg(AT32_ETH_DMAOMR);
  regval |= ETH_DMAOMR_ST;
  at32_putreg(regval, AT32_ETH_DMAOMR);

  /* Start DMA reception */

  regval  = at32_getreg(AT32_ETH_DMAOMR);
  regval |= ETH_DMAOMR_SR;
  at32_putreg(regval, AT32_ETH_DMAOMR);

  /* Enable Ethernet DMA interrupts.
   *
   * The AT32 hardware supports two interrupts: (1) one dedicated to normal
   * Ethernet operations and the other, used only for the Ethernet wakeup
   * event.  The wake-up interrupt is not used by this driver.
   *
   * The first Ethernet vector is reserved for interrupts generated by the
   * MAC and the DMA. The MAC provides PMT and time stamp trigger interrupts,
   * neither of which are used by this driver.
   */

  at32_putreg(ETH_MACIMR_ALLINTS, AT32_ETH_MACIMR);

  /* Ethernet DMA supports two classes of interrupts: Normal interrupt
   * summary (NIS) and Abnormal interrupt summary (AIS) with a variety
   * individual normal and abnormal interrupting events.  Here only
   * the normal receive event is enabled (unless DEBUG is enabled).  Transmit
   * events will only be enabled when a transmit interrupt is expected.
   */

  at32_putreg(ETH_DMAINT_RECV_ENABLE | ETH_DMAINT_ERROR_ENABLE,
               AT32_ETH_DMAIER);
  return OK;
}

/****************************************************************************
 * Function: at32_ethconfig
 *
 * Description:
 *  Configure the Ethernet interface for DMA operation.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int at32_ethconfig(struct at32_ethmac_s *priv)
{
  int ret;

  /* NOTE: The Ethernet clocks were initialized early in the boot-up
   * sequence in at32_rcc.c.
   */

  /* Reset the Ethernet block */

  ninfo("Reset the Ethernet block\n");
  ret = at32_ethreset(priv);
  if (ret < 0)
    {
      nerr("ERROR: Reset of Ethernet block failed\n");
      return ret;
    }

  /* Initialize the PHY */

  ninfo("Initialize the PHY\n");
  ret = at32_phyinit(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialize the MAC and DMA */

  ninfo("Initialize the MAC and DMA\n");
  ret = at32_macconfig(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialize the free buffer list */

  at32_initbuffer(priv, g_alloc);

  /* Initialize TX Descriptors list: Chain Mode */

  at32_txdescinit(priv, g_txtable);

  /* Initialize RX Descriptors list: Chain Mode  */

  at32_rxdescinit(priv, g_rxtable, g_rxbuffer);

  /* Enable normal MAC operation */

  ninfo("Enable normal operation\n");
  return at32_macenable(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: at32_ethinitialize
 *
 * Description:
 *   Initialize the Ethernet driver for one interface.  If the AT32 chip
 *   supports multiple Ethernet controllers, then board specific logic
 *   must implement arm_netinitialize() and call this function to initialize
 *   the desired interfaces.
 *
 * Input Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if AT32_NETHERNET == 1 || defined(CONFIG_NETDEV_LATEINIT)
static inline
#endif
int at32_ethinitialize(int intf)
{
  struct at32_ethmac_s *priv;
  int ret;
#ifdef CONFIG_NET_ETHERNET
  uint8_t uniqueid[12];
  uint32_t uid;
  uint8_t *mac;
#endif

  ninfo("intf: %d\n", intf);

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf < AT32_NETHERNET);
  priv = &g_at32ethmac[intf];

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct at32_ethmac_s));
  priv->dev.d_ifup    = at32_ifup;     /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = at32_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = at32_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac  = at32_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = at32_rmmac;    /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = at32_ioctl;    /* Support PHY ioctl() calls */
#endif
  priv->dev.d_private = g_at32ethmac;  /* Used to recover private state from dev */

  /* Configure GPIO pins to support Ethernet */

  at32_ethgpioconfig(priv);

#ifdef CONFIG_NET_ETHERNET
  /* Determine a unique MAC address from MCU UID */

  mac    = priv->dev.d_mac.ether.ether_addr_octet;

  at32_get_uniqueid(uniqueid);
  uid = crc32part(uniqueid, sizeof(uniqueid), 0);

  mac[0] = uniqueid[10] & 0xfe;
  mac[1] = uniqueid[11];
  mac[2] = (uid  & 0xff000000) >> 24;
  mac[3] = (uid  & 0x00ff0000) >> 16;
  mac[4] = (uid  & 0x0000ff00) >> 8;
  mac[5] = (uid  & 0x000000ff);

#endif

  /* Attach the IRQ to the driver */

  if (irq_attach(AT32_IRQ_ETH, at32_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Put the interface in the down state. */

  ret = at32_ifdown(&priv->dev);
  if (ret < 0)
    {
      nerr("ERROR: Initialization of Ethernet block failed: %d\n", ret);
      return ret;
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_ETHERNET);
  return OK;
}

/****************************************************************************
 * Function: arm_netinitialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in arm_initialize.c.  If AT32_NETHERNET
 *   greater than one, then board specific logic will have to supply a
 *   version of arm_netinitialize() that calls at32_ethinitialize() with
 *   the appropriate interface number.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if AT32_NETHERNET == 1 && !defined(CONFIG_NETDEV_LATEINIT)
void arm_netinitialize(void)
{
  at32_ethinitialize(0);

#ifdef CONFIG_AT32_CAN_SOCKET

  extern int at32_cansockinitialize(int port);

#ifdef CONFIG_AT32_CAN1
  at32_cansockinitialize(1);
#endif

#ifdef CONFIG_AT32_CAN2
  at32_cansockinitialize(2);
#endif

#endif
}
#endif

#endif /* AT32_NETHERNET > 0 */
#endif /* CONFIG_NET && CONFIG_AT32_ETHMAC */
