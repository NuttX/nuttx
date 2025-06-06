/****************************************************************************
 * include/nuttx/net/gmii.h
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

#ifndef __INCLUDE_NUTTX_NET_GMII_H
#define __INCLUDE_NUTTX_NET_GMII_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/net/mii.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MII register offsets *****************************************************/

/* Common MII management registers. The IEEE 802.3 standard specifies a
 * register set for controlling and gathering status from the PHY layer. The
 * registers are collectively known as the MII Management registers and are
 * detailed in Section 22.2.4 of the IEEE 802.3 specification.
 */

#define GMII_MCR                      MII_MCR         /* GMII management control */
#define GMII_MSR                      MII_MSR         /* GMII management status */
#define GMII_PHYID1                   MII_PHYID1      /* PHY ID 1 */
#define GMII_PHYID2                   MII_PHYID2      /* PHY ID 2 */
#define GMII_ADVERTISE                MII_ADVERTISE   /* Auto-negotiation advertisement */
#define GMII_LPA                      MII_LPA         /* Auto-negotiation link partner base page ability */
#define GMII_EXPANSION                MII_EXPANSION   /* Auto-negotiation expansion */
#define GMII_NEXTPAGE                 MII_NEXTPAGE    /* Auto-negotiation next page */
#define GMII_LPANEXTPAGE              MII_LPANEXTPAGE /* Auto-negotiation link partner received next page */
#define GMII_1000BTCR                 9               /* 1000BASE-T control */
#define GMII_1000BTSR                 10              /* 1000BASE-T status */
#define GMII_ERCR                     11              /* Extend Register - Control */
#define GMII_ERDWR                    12              /* Extend Register - Data Write Register */
#define GMII_ERDRR                    13              /* Extend Register - Data Read Register */
#define GMII_ESTATUS                  MII_ESTATUS     /* Extended MII status register */

/* Extended Registers:
 * Registers 16-31 may be used for vendor specific abilities
  */

/* Micrel KSZ9021/31 Vendor Specific Register Addresses *********************/

#define GMII_KSZ90x1_RLPBK            17              /* Remote loopback, LED mode */
#define GMII_KSZ90x1_LINKMD           18              /* LinkMD(c) cable diagnostic */
#define GMII_KSZ90x1_PMAPCS           19              /* Digital PMA/PCS status */
#define GMII_KSZ90x1_RXERR            21              /* RXER counter */
#define GMII_KSZ90X1_ICS              27              /* Interrupt control/status */
#define GMII_KSZ90x1_DBGCTRL1         28              /* Digital debug control 1 */
#define GMII_KSZ90x1_PHYCTRL          31              /* PHY control */

/* Micrel KSZ9021/31 Extended Register Addresses */

#define GMII_KSZ90x1_CCR              256             /* Common control */
#define GMII_KSZ90x1_SSR              257             /* Strap status */
#define GMII_KSZ90x1_OMSOR            258             /* Operation mode strap override */
#define GMII_KSZ90x1_OMSSR            259             /* Operation mode strap status */
#define GMII_KSZ90X1_RCCPSR           260             /* RGMII clock and control pad skew */
#define GMII_KSZ90X1_RRDPSR           261             /* RGMII RX data pad skew */
#define GMII_KSZ90x1_ATR              263             /* Analog test register */

/* Realtek RTL8211 PHY Extended Registers */

#define GMII_RTL8211F_NAME            "RTL8211F"
#define GMII_RTL8211F_INER_A42        18      /* Interrupt Enable Register */
#define GMII_RTL8211F_PHYCR1_A43      24      /* PHY Specific Control Register 1 */
#define GMII_RTL8211F_PHYCR2_A43      25      /* PHY Specific Control Register 2 */
#define GMII_RTL8211F_PHYSR_A43       26      /* PHY Specific Status Register */
#define GMII_RTL8211F_INSR_A43        29      /* Interrupt Status Register */
#define GMII_RTL8211F_PAGSR           31      /* Page Select Register */
#define GMII_RTL8211F_PHYSCR_A46      20      /* PHY Special Config Register */
#define GMII_RTL8211F_LCR_D04         16      /* LED Control Register */
#define GMII_RTL8211F_EEELCR_D04      17      /* EEE LED Control Register */
#define GMII_RTL8211F_MIICR1_D08      17      /* MII Control Register 1 */
#define GMII_RTL8211F_MIICR2_D08      21      /* MII Control Register 2 */
#define GMII_RTL8211F_INTBCR_D40      22      /* INTB Pin Control Register */

/* MII register bit settings ************************************************/

/* MII Control register bit definitions */

#define GMII_MCR_UNIDIR               MII_MCR_UNIDIR
#define GMII_MCR_SPEED1000            MII_MCR_SPEED1000
#define GMII_MCR_CTST                 MII_MCR_CTST
#define GMII_MCR_FULLDPLX             MII_MCR_FULLDPLX
#define GMII_MCR_ANRESTART            MII_MCR_ANRESTART
#define GMII_MCR_ISOLATE              MII_MCR_ISOLATE
#define GMII_MCR_PDOWN                MII_MCR_PDOWN
#define GMII_MCR_ANENABLE             MII_MCR_ANENABLE
#define GMII_MCR_SPEED100             MII_MCR_SPEED100
#define GMII_MCR_LOOPBACK             MII_MCR_LOOPBACK
#define GMII_MCR_RESET                MII_MCR_RESET

/* MII Status register bit definitions */

#define GMII_MSR_EXTCAP               MII_MSR_EXTCAP
#define GMII_MSR_JABBERDETECT         MII_MSR_JABBERDETECT
#define GMII_MSR_LINKSTATUS           MII_MSR_LINKSTATUS
#define GMII_MSR_ANEGABLE             MII_MSR_ANEGABLE
#define GMII_MSR_RFAULT               MII_MSR_RFAULT
#define GMII_MSR_ANEGCOMPLETE         MII_MSR_ANEGCOMPLETE
#define GMII_MSR_UNIDIR               MII_MSR_UNIDIR
#define GMII_MSR_MFRAMESUPPRESS       MII_MSR_MFRAMESUPPRESS
#define GMII_MSR_ESTATEN              MII_MSR_ESTATEN
#define GMII_MSR_100BASET2FULL        MII_MSR_100BASET2FULL
#define GMII_MSR_100BASET2HALF        MII_MSR_100BASET2HALF
#define GMII_MSR_10BASETXHALF         MII_MSR_10BASETXHALF
#define GMII_MSR_10BASETXFULL         MII_MSR_10BASETXFULL
#define GMII_MSR_100BASETXHALF        MII_MSR_100BASETXHALF
#define GMII_MSR_100BASETXFULL        MII_MSR_100BASETXFULL
#define GMII_MSR_100BASET4            MII_MSR_100BASET4

/* MII ID2 register bits */

#define GMII_PHYID2_REV_SHIFT         MII_PHYID2_REV_SHIFT
#define GMII_PHYID2_REV_MASK          MII_PHYID2_REV_MASK
#define GMII_PHYID2_REV(n)            MII_PHYID2_REV(n)
#define GMII_PHYID2_MODEL_SHIFT       MII_PHYID2_MODEL_SHIFT
#define GMII_PHYID2_MODEL_MASK        MII_PHYID2_MODEL_MASK
#  define GMII_PHYID2_MODEL(n)        MII_PHYID2_MODEL(n)
#define GMII_PHYID2_OUI_SHIFT         MII_PHYID2_OUI_SHIFT
#define GMII_PHYID2_OUI_MASK          MII_PHYID2_OUI_MASK
#  define GMII_PHYID2_OUI(n)          MII_PHYID2_OUI(n)

/* Advertisement control register bit definitions */

#define GMII_ADVERTISE_SELECT         MII_ADVERTISE_SELECT
#define GMII_ADVERTISE_CSMA           MII_ADVERTISE_CSMA
#define GMII_ADVERTISE_8023           MII_ADVERTISE_8023
#define GMII_ADVERTISE_8029           MII_ADVERTISE_8029
#define GMII_ADVERTISE_8025           MII_ADVERTISE_8025
#define GMII_ADVERTISE_1394           MII_ADVERTISE_1394
#define GMII_ADVERTISE_10BASETXHALF   MII_ADVERTISE_10BASETXHALF
#define GMII_ADVERTISE_1000XFULL      MII_ADVERTISE_1000XFULL
#define GMII_ADVERTISE_10BASETXFULL   MII_ADVERTISE_10BASETXFULL
#define GMII_ADVERTISE_1000XHALF      MII_ADVERTISE_1000XHALF
#define GMII_ADVERTISE_100BASETXHALF  MII_ADVERTISE_100BASETXHALF
#define GMII_ADVERTISE_1000XPAUSE     MII_ADVERTISE_1000XPAUSE
#define GMII_ADVERTISE_100BASETXFULL  MII_ADVERTISE_100BASETXFULL
#define GMII_ADVERTISE_1000XASYMPAU   MII_ADVERTISE_1000XASYMPAU
#define GMII_ADVERTISE_100BASET4      MII_ADVERTISE_100BASET4
#define GMII_ADVERTISE_FDXPAUSE       MII_ADVERTISE_FDXPAUSE
#define GMII_ADVERTISE_ASYMPAUSE      MII_ADVERTISE_ASYMPAUSE
#define GMII_ADVERTISE_RFAULT         MII_ADVERTISE_RFAULT
#define GMII_ADVERTISE_LPACK          MII_ADVERTISE_LPACK
#define GMII_ADVERTISE_NXTPAGE        MII_ADVERTISE_NXTPAGE

/* Link partner ability register bit definitions */

#define GMII_LPA_SELECT               MII_LPA_SELECT
#define GMII_LPA_CSMA                 MII_LPA_CSMA
#define GMII_LPA_8023                 MII_LPA_8023
#define GMII_LPA_8029                 MII_LPA_8029
#define GMII_LPA_8025                 MII_LPA_8025
#define GMII_LPA_8025                 MII_LPA_1394
#define GMII_LPA_1394                 MII_LPA_1394
#define GMII_LPA_10BASETXHALF         MII_LPA_10BASETXHALF
#define GMII_LPA_1000XFULL            MII_LPA_1000XFULL
#define GMII_LPA_1000XFULL            MII_LPA_10BASETXFULL
#define GMII_LPA_10BASETXFULL         MII_LPA_10BASETXFULL
#define GMII_LPA_1000XHALF            MII_LPA_1000XHALF
#define GMII_LPA_100BASETXHALF        MII_LPA_100BASETXHALF
#define GMII_LPA_1000XPAUSE           MII_LPA_1000XPAUSE
#define GMII_LPA_100BASETXFULL        MII_LPA_100BASETXFULL
#define GMII_LPA_1000XASYMPAU         MII_LPA_1000XASYMPAU
#define GMII_LPA_100BASET4            MII_LPA_100BASET4
#define GMII_LPA_FDXPAUSE             MII_LPA_FDXPAUSE
#define GMII_LPA_ASYMPAUSE            MII_LPA_ASYMPAUSE
#define GMII_LPA_RFAULT               MII_LPA_RFAULT
#define GMII_LPA_LPACK                MII_LPA_LPACK
#define GMII_LPA_NXTPAGE              MII_LPA_NXTPAGE

/* Link partner ability in next page format */

#define GMII_LPANP_MESSAGE            MII_LPANP_MESSAGE
#define GMII_LPANP_TOGGLE             MII_LPANP_TOGGLE
#define GMII_LPANP_LACK2              MII_LPANP_LACK2
#define GMII_LPANP_MSGPAGE            MII_LPANP_MSGPAGE
#define GMII_LPANP_LPACK              MII_LPANP_LPACK
#define GMII_LPANP_NXTPAGE            MII_LPANP_NXTPAGE

/* MII Auto-negotiation expansion register bit definitions */

#define GMII_EXPANSION_ANEGABLE       MII_EXPANSION_ANEGABLE
#define GMII_EXPANSION_PAGERECVD      MII_EXPANSION_PAGERECVD
#define GMII_EXPANSION_ENABLENPAGE    MII_EXPANSION_ENABLENPAGE
#define GMII_EXPANSION_NXTPAGEABLE    MII_EXPANSION_NXTPAGEABLE
#define GMII_EXPANSION_PARFAULTS      MII_EXPANSION_PARFAULTS

/* Auto-negotiation next page advertisement */

#define GMII_NPADVERTISE_CODE         MII_NPADVERTISE_CODE
#define GMII_NPADVERTISE_TOGGLE       MII_NPADVERTISE_TOGGLE
#define GMII_NPADVERTISE_ACK2         MII_NPADVERTISE_ACK2
#define GMII_NPADVERTISE_MSGPAGE      MII_NPADVERTISE_MSGPAGE
#define GMII_NPADVERTISE_NXTPAGE      MII_NPADVERTISE_NXTPAGE

/* MMD access control register */

#define GMII_MMDCONTROL_DEVAD_SHIFT   MII_MMDCONTROL_DEVAD_SHIFT
#define GMII_MMDCONTROL_DEVAD_MASK    MII_MMDCONTROL_DEVAD_MASK
#  define GMII_MMDCONTROL_DEVAD(n)    MII_MMDCONTROL_DEVAD(n)
#define GMII_MMDCONTROL_FUNC_SHIFT    MII_MMDCONTROL_FUNC_SHIFT
#define GMII_MMDCONTROL_FUNC_MASK     MII_MMDCONTROL_FUNC_MASK
#  define GMII_MMDCONTROL_FUNC_ADDR   MII_MMDCONTROL_FUNC_ADDR
#  define GMII_MMDCONTROL_FUNC_NOINCR MII_MMDCONTROL_FUNC_NOINCR
#  define GMII_MMDCONTROL_FUNC_RWINCR MII_MMDCONTROL_FUNC_RWINCR
#  define GMII_MMDCONTROL_FUNC_WINCR  MII_MMDCONTROL_FUNC_WINCR

/* Extended Status Register */

#define GMII_ESTATUS_1000BASETHALF    MII_ESTATUS_1000BASETHALF
#define GMII_ESTATUS_1000BASETFULL    MII_ESTATUS_1000BASETFULL
#define GMII_ESTATUS_1000BASEXHALF    MII_ESTATUS_1000BASEXHALF
#define GMII_ESTATUS_1000BASEXFULL    MII_ESTATUS_1000BASEXFULL

/* 1000BASE-T Control Register */

                                                /* Bits 0-7: Reserved */
#define GMII_1000BTCR_1000BASETHALF   (1 << 8)  /* Bit 8:  1000Base-T half duplex able */
#define GMII_1000BTCR_1000BASETFULL   (1 << 9)  /* Bit 9:  1000Base-T full duplex able */
#define GMII_1000BTCR_MULTIPLE        (1 << 10) /* Bit 10: Port type:  Prefer multiport device */
#define GMII_1000BTCR_MMASTER         (1 << 11) /* Bit 11: Configure PHY as master (manual) */
#define GMII_1000BTCR_MSMC            (1 << 12) /* Bit 12: Master/slave manual configuration */
#define GMII_1000BTCR_TESTMODE_SHIFT  (13)      /* Bits 13-15: Test Mode */
#define GMII_1000BTCR_TESTMODE_MASK   (7 << GMII_1000BTCR_TESTMODE_SHIFT)
#  define GMII_1000BTCR_MODE_NORMAL   (0 << GMII_1000BTCR_TESTMODE_SHIFT)
#  define GMII_1000BTCR_TESTMODE(n)   ((uint16_t)(n) << GMII_1000BTCR_TESTMODE_SHIFT) /* n=1-4 */

/* 1000BASE-T Status Register */

#define GMII_1000BTSR_IDLERR_SHIFT    (0)       /* Bits 0-7: Idle error count */
#define GMII_1000BTSR_IDLERR_MASK     (0xff << GMII_1000BTSR_IDLERR_SHIFT)
                                                /* Bits 8-9: Reserved */
#define GMII_1000BTSR_LP1000BASETHALF (1 << 10) /* Bit 10: Link partner 1000Base-T half duplex able */
#define GMII_1000BTSR_LP1000BASETFULL (1 << 11) /* Bit 11: Link partner 1000Base-T full duplex able */
#define GMII_1000BTSR_RROK            (1 << 12) /* Bit 12: Remote receiver OK */
#define GMII_1000BTSR_LROK            (1 << 13) /* Bit 13: Local receiver OK */
#define GMII_1000BTSR_MASTER          (1 << 14) /* Bit 14: Configuration resolved to master */
#define GMII_1000BTSR_MSFAULT         (1 << 15) /* Bit 15: Master/slave fault detected */

/* Extend Register - Control Register */

#define GMII_ERCR_ADDR_SHIFT          (0)       /* Bits 0-7: Select extended register address */
#define GMII_ERCR_ADDR_MASK           (0xff << GMII_ERCR_ADDR_SHIFT)
#  define GMII_ERCR_ADDR(n)           ((uint16_t)(n) << GMII_ERCR_ADDR_SHIFT)
#define GMII_ERCR_PAGE                (1 << 8)  /* Bit 8: Select page for extended register */
                                                /* Bits 9-14: Reserved */
#define GMII_ERCR_READ                (0)       /* Bit 15: 0=Read extended register */
#define GMII_ERCR_WRITE               (1 << 15) /* Bit 15: 1=Write extended register */

/* Extend Register - Data Write Register (16-bit data value) */

/* Extend Register - Data Read Register (16-bit data value) */

/* Micrel KSZ9021/31 Vendor Specific Register Bit Definitions ***************/

/* KSZ8021/31 Register 27: Interrupt control/status */

#define GMII_KSZ90x1_INT_JEN          (1 << 15) /* Jabber interrupt enable */
#define GMII_KSZ90x1_INT_REEN         (1 << 14) /* Receive error interrupt enable */
#define GMII_KSZ90x1_INT_PREN         (1 << 13) /* Page received interrupt enable */
#define GMII_KSZ90x1_INT_PDFEN        (1 << 12) /* Parallel detect fault interrupt enable */
#define GMII_KSZ90x1_INT_LPAEN        (1 << 11) /* Link partner acknowledge interrupt enable */
#define GMII_KSZ90X1_INT_LDEN         (1 << 10) /* Link down fault interrupt enable */
#define GMII_KSZ90x1_INT_RFEN         (1 << 9)  /* Remote fault interrupt enable */
#define GMII_KSZ90X1_INT_LUEN         (1 << 8)  /* Link up interrupt enable */

#define GMII_KSZ90x1_INT_J            (1 << 7)  /* Jabber interrupt */
#define GMII_KSZ90x1_INT_RE           (1 << 6)  /* Receive error interrupt */
#define GMII_KSZ90x1_INT_PR           (1 << 5)  /* Page received interrupt */
#define GMII_KSZ90x1_INT_PDF          (1 << 4)  /* Parallel detect fault interrupt */
#define GMII_KSZ90x1_INT_LPA          (1 << 3)  /* Link partner acknowledge interrupt */
#define GMII_KSZ90x1_INT_LD           (1 << 2)  /* Link down fault interrupt */
#define GMII_KSZ90x1_INT_RF           (1 << 1)  /* Remote fault interrupt */
#define GMII_KSZ90x1_INT_LU           (1 << 0)  /* Link up interrupt */

/* RTL8211 register bit settings ********************************************/

/* RTL8211F MII ID1/2 register bits */

#define GMII_PHYID1_RTL8211F           0x001c   /* ID1 value for Realtek RTL8211F */
#define GMII_PHYID2_RTL8211F           0xc878   /* ID2 value for Realtek RTL8211F */
#define GMII_RTL8211F_PHYSR_SPEED_MASK 0x30
#define GMII_RTL8211F_PHYSR_10MBPS     0x00
#define GMII_RTL8211F_PHYSR_100MBPS    0x10
#define GMII_RTL8211F_PHYSR_1000MBPS   0x20
#define GMII_RTL8211F_PHYSR_DUPLEX     0x8
#define GMII_RTL8211F_MIICR1_TX_DELAY  (1 << 8) /* Enable PHY internal TX delay */
#define GMII_RTL8211F_MIICR2_RX_DELAY  (1 << 3) /* Enable PHY internal RX delay */

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_GMII_H */
