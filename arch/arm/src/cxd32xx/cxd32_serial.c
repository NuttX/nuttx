/****************************************************************************
 * arch/arm/src/cxd32xx/cxd32_serial.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <nuttx/spinlock.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "hardware/cxd32_uart.h"
#include "cxd32_config.h"
#include "cxd32_serial.h"
#include "cxd32_uart.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#if defined(USE_SERIALDRIVER) && defined(HAVE_UART)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uintptr_t uartbase; /* Base address of UART registers */
  uint32_t basefreq;  /* Base frequency of input clock */
  uint32_t baud;      /* Configured baud */
  uint32_t ier;       /* Saved IER value */
  uint8_t id;         /* ID=0,1,2,3 */
  uint8_t irq;        /* IRQ associated with this UART */
  uint8_t parity;     /* 0=none, 1=odd, 2=even */
  uint8_t bits;       /* Number of bits (5,6,7 or 8) */
  bool stopbits2;     /* true: Configure with 2 stop bits instead of 1 */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  bool iflow;         /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  bool oflow;         /* output flow control (CTS) enabled */
#endif
#ifdef HAVE_RS485
  bool dtrdir;        /* DTR pin is the direction bit */
#endif
  void *pmhandle;
  spinlock_t lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_UART_CONFIG
static void up_set_format(struct uart_dev_s *dev);
#endif
static int up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int up_interrupt(int irq, void *context, void *arg);
static int up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup         = up_setup,
  .shutdown      = up_shutdown,
  .attach        = up_attach,
  .detach        = up_detach,
  .ioctl         = up_ioctl,
  .receive       = up_receive,
  .rxint         = up_rxint,
  .rxavailable   = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = NULL,
#endif
  .send          = up_send,
  .txint         = up_txint,
  .txready       = up_txready,
  .txempty       = up_txempty,
};

/* I/O buffers */

#ifdef CONFIG_CXD32_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif

/* This describes the state of the CXD32xx uart0 port. */

#ifdef CONFIG_CXD32_UART0
static struct up_dev_s g_uart0priv =
{
  .uartbase  = CXD32_UART0_BASE,
  .basefreq  = CXD32_UART_BASEFREQ,
  .baud      = CONFIG_UART0_BAUD,
  .id        = 1,
  .irq       = CXD32_IRQ_UART0,
  .parity    = CONFIG_UART0_PARITY,
  .bits      = CONFIG_UART0_BITS,
  .stopbits2 = CONFIG_UART0_2STOP,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .iflow     = false, /* flow control is not supported */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  .oflow     = false, /* flow control is not supported */
#endif
};

static uart_dev_t g_uart0port =
{
  .recv =
    {
      .size   = CONFIG_UART0_RXBUFSIZE,
      .buffer = g_uart0rxbuffer,
    },
  .xmit =
    {
      .size   = CONFIG_UART0_TXBUFSIZE,
      .buffer = g_uart0txbuffer,
    },
  .ops  = &g_uart_ops,
  .priv = &g_uart0priv,
};
#  define TTYS0_DEV g_uart0port /* UART0=ttyS0 */
#endif

/* Which UART with be tty0/console and which tty1? tty2? tty3? */

#ifdef HAVE_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0port /* UART0=console */
#  endif
#endif /* HAVE_CONSOLE */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset,
                                uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv,
                                     uint32_t *ier)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  if (ier)
    {
      *ier = priv->ier & UART_INTR_ALL;
    }

  priv->ier &= ~UART_INTR_ALL;
  up_serialout(priv, CXD32_UART_IMSC, priv->ier);
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct up_dev_s *priv, uint32_t ier)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  priv->ier |= ier & UART_INTR_ALL;
  up_serialout(priv, CXD32_UART_IMSC, priv->ier);
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: up_enablebreaks
 ****************************************************************************/

static inline void up_enablebreaks(struct up_dev_s *priv, bool enable)
{
  uint32_t lcr = up_serialin(priv, CXD32_UART_LCR_H);
  if (enable)
    {
      lcr |= UART_LCR_BRK;
    }
  else
    {
      lcr &= ~UART_LCR_BRK;
    }

  up_serialout(priv, CXD32_UART_LCR_H, lcr);
}

/****************************************************************************
 * Name: up_set_format
 *
 * Description:
 *   Set the serial line format and speed.
 *
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_UART_CONFIG
static void up_set_format(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t lcr;
  uint32_t cr;
  uint32_t cr_en;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  /* Get the original state of control register */

  cr    = up_serialin(priv, CXD32_UART_CR);
  cr_en = cr & UART_CR_EN;
  cr   &= ~UART_CR_EN;

  /* Disable until the format bits and baud rate registers are updated */

  up_serialout(priv, CXD32_UART_CR, cr);

  /* Set the BAUD divisor */

  cxd32_setbaud(priv->uartbase, priv->basefreq, priv->baud);

  /* Set up the LCR */

  lcr = up_serialin(priv, CXD32_UART_LCR_H);

  lcr &= ~(UART_LCR_WLEN(8) | UART_LCR_STP2 | UART_LCR_EPS | UART_LCR_PEN);

  if ((5 <= priv->bits) && (priv->bits < 8))
    {
      lcr |= UART_LCR_WLEN(priv->bits);
    }
  else
    {
      lcr |= UART_LCR_WLEN(8);
    }

  if (priv->stopbits2)
    {
      lcr |= UART_LCR_STP2;
    }

  if (priv->parity == 1)
    {
      lcr |= (UART_LCR_PEN);
    }
  else if (priv->parity == 2)
    {
      lcr |= (UART_LCR_PEN | UART_LCR_EPS);
    }

  up_serialout(priv, CXD32_UART_LCR_H, lcr);

  /* CXD32 does not have CTS/RTS pin, so these are disabled */

  cr &= ~(UART_CR_RTSEN | UART_CR_CTSEN);
  up_serialout(priv, CXD32_UART_CR, cr | cr_en);

  spin_unlock_irqrestore(&priv->lock, flags);
}
#endif /* CONFIG_SUPPRESS_UART_CONFIG */

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This method is
 *   called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t lcr;
  uint32_t cr;

  cxd32_uart_setup(priv->id);

  /* Init HW */

  up_serialout(priv, CXD32_UART_CR, 0);
  up_serialout(priv, CXD32_UART_LCR_H, 0);
  up_serialout(priv, CXD32_UART_DMACR, 0);
  up_serialout(priv, CXD32_UART_RSR_ECR, 0xf);

  /* Set up the IER */

  priv->ier = up_serialin(priv, CXD32_UART_IMSC);

  /* Configure the UART line format and speed. */

  up_set_format(dev);

  /* Set interrupt FIFO level */

  up_serialout(priv, CXD32_UART_IFLS, 0);

  /* Clear all interrupts */

  up_serialout(priv, CXD32_UART_ICR, 0x7ff);

  /* Enable FIFO and UART in the last */

  lcr = up_serialin(priv, CXD32_UART_LCR_H);
  lcr |= UART_LCR_FEN;
  up_serialout(priv, CXD32_UART_LCR_H, lcr);

  cr = up_serialin(priv, CXD32_UART_CR);
  cr |= UART_CR_RXE | UART_CR_TXE | UART_CR_EN;
  up_serialout(priv, CXD32_UART_CR, cr);
#endif

  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Disable further interrupts from the UART */

  up_disableuartint(priv, NULL);

  /* Put the UART hardware back its reset state */

  switch (priv->id)
    {
      case 0:
      case 1:
      case 2:
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.
 *   This method is called when the serial port is opened.
 *   Normally, this is just after the the setup() method is called,
 *   however, the serial console may operate in  a non-interrupt driven mode
 *   during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).
 *   The RX and TX interrupts are not enabled until the txint() and rxint()
 *   methods are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt, dev);
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the UART
       */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.
 *   The exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_rxflowcontrol
 *
 * Description:
 *   Called when Rx buffer is full (or exceeds configured watermark levels
 *   if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is defined).
 *   Return true if UART activated RX flow control to block more incoming
 *   data
 *
 * Input parameters:
 *   dev       - UART device instance
 *   nbuffered - the number of characters currently buffered
 *               (if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is
 *               not defined the value will be 0 for an empty buffer or the
 *               defined buffer size for a full buffer)
 *   upper     - true indicates the upper watermark was crossed where
 *               false indicates the lower watermark has been crossed
 *
 * Returned Value:
 *   true if RX flow control activated.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool up_rxflowcontrol(struct uart_dev_s *dev,
                             unsigned int nbuffered, bool upper)
{
  up_rxint(dev, !upper);
  return true;
}
#endif /* CONFIG_SERIAL_IFLOWCONTROL */

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s *priv;
  uint32_t status;
  int passes;

  priv = (struct up_dev_s *)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (passes = 0; passes < 256; passes++)
    {
      /* Get the current UART status and check for loop
       * termination conditions
       */

      status = up_serialin(priv, CXD32_UART_MIS);
      if (status == 0)
        {
          return OK;
        }

      up_serialout(priv, CXD32_UART_ICR, status);
      if (status & UART_INTR_RI)
        {
        }

      if (status & UART_INTR_CTS)
        {
        }

      if (status & UART_INTR_DCD)
        {
        }

      if (status & UART_INTR_DSR)
        {
        }

      if (status & (UART_INTR_RX | UART_INTR_RT))
        {
          uart_recvchars(dev);
        }

      if (status & UART_INTR_TX)
        {
          uart_xmitchars(dev);
        }

      if (status & UART_INTR_FE)
        {
        }

      if (status & UART_INTR_PE)
        {
        }

      if (status & UART_INTR_BE)
        {
        }

      if (status & UART_INTR_OE)
        {
        }
    }

  return OK;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode *inode    = filep->f_inode;
  struct uart_dev_s *dev = inode->i_private;
  struct up_dev_s *priv  = (struct up_dev_s *)dev->priv;
  int ret                = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
      case TIOCSERGSTRUCT:
        {
          struct up_dev_s *user = (struct up_dev_s *)arg;
          if (!user)
            {
              ret = -EINVAL;
            }
          else
            {
              memcpy(user, dev, sizeof(struct up_dev_s));
            }
        }
        break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
      case TCGETS:
        {
          struct termios *termiosp = (struct termios *)arg;
          irqstate_t flags;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          flags = spin_lock_irqsave(&priv->lock);

          termiosp->c_cflag = ((priv->parity != 0) ? PARENB : 0) |
                              ((priv->parity == 1) ? PARODD : 0) |
#ifdef CONFIG_SERIAL_OFLOWCONTROL
                              ((priv->oflow) ? CCTS_OFLOW : 0) |
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
                              ((priv->iflow) ? CRTS_IFLOW : 0) |
#endif
                              ((priv->stopbits2) ? CSTOPB : 0);

          cfsetispeed(termiosp, priv->baud);

          switch (priv->bits)
            {
              case 5:
                termiosp->c_cflag |= CS5;
                break;

              case 6:
                termiosp->c_cflag |= CS6;
                break;

              case 7:
                termiosp->c_cflag |= CS7;
                break;

              case 8:
              default:
                termiosp->c_cflag |= CS8;
                break;
            }

          spin_unlock_irqrestore(&priv->lock, flags);
        }
        break;

      case TCSETS:
        {
          struct termios *termiosp = (struct termios *)arg;
          irqstate_t flags;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          flags = spin_lock_irqsave(&priv->lock);

          switch (termiosp->c_cflag & CSIZE)
            {
              case CS5:
                priv->bits = 5;
                break;

              case CS6:
                priv->bits = 6;
                break;

              case CS7:
                priv->bits = 7;
                break;

              case CS8:
              default:
                priv->bits = 8;
                break;
            }

          if ((termiosp->c_cflag & PARENB) != 0)
            {
              priv->parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
            }
          else
            {
              priv->parity = 0;
            }

          priv->stopbits2 = (termiosp->c_cflag & CSTOPB) != 0;

#ifdef CONFIG_SERIAL_OFLOWCONTROL
          priv->oflow = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
          priv->iflow = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif
          priv->baud = cfgetispeed(termiosp);

          spin_unlock_irqrestore(&priv->lock, flags);

          /* Configure the UART line format and speed. */

          up_set_format(dev);
        }
        break;
#endif

      case TIOCSBRK: /* BSD compatibility: Turn break on, unconditionally */
        {
          irqstate_t flags = spin_lock_irqsave(&priv->lock);
          up_enablebreaks(priv, true);
          spin_unlock_irqrestore(&priv->lock, flags);
        }
        break;

      case TIOCCBRK: /* BSD compatibility: Turn break off, unconditionally */
        {
          irqstate_t flags;
          flags = spin_lock_irqsave(&priv->lock);
          up_enablebreaks(priv, false);
          spin_unlock_irqrestore(&priv->lock, flags);
        }
        break;

      case TCFLSH: /* Flush TX fifo etc. */
        {
          while (!up_txempty(dev));
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t rbr;

  rbr     = up_serialin(priv, CXD32_UART_DR);
  *status = rbr & 0xf00;
  return rbr & 0xff;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= (UART_INTR_RX | UART_INTR_RT);
#endif
    }
  else
    {
      priv->ier &= ~(UART_INTR_RX | UART_INTR_RT);
    }

  up_serialout(priv, CXD32_UART_IMSC, priv->ier);
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, CXD32_UART_FR) & UART_FLAG_RXFE) == 0);
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_serialout(priv, CXD32_UART_DR, (uint32_t)ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= UART_INTR_TX;
      up_serialout(priv, CXD32_UART_IMSC, priv->ier);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

#  ifdef CONFIG_SMP
      spin_unlock_irqrestore(&priv->lock, flags);
#  endif
      uart_xmitchars(dev);
#  ifdef CONFIG_SMP
      flags = spin_lock_irqsave(&priv->lock);
#  endif
#endif
    }
  else
    {
      priv->ier &= ~UART_INTR_TX;
      up_serialout(priv, CXD32_UART_IMSC, priv->ier);
    }

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, CXD32_UART_FR) & UART_FLAG_TXFF) == 0);
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t rbr = 0;
  rbr = up_serialin(priv, CXD32_UART_FR);
  return (((rbr & UART_FLAG_TXFE) != 0) && ((rbr & UART_FLAG_BUSY) == 0));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during boot up.  This must be called
 *   before arm_serialinit.
 *
 *   NOTE: Configuration of the CONSOLE UART was performed by up_lowsetup()
 *   very early in the boot sequence.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void arm_earlyserialinit(void)
{
  /* Configuration whichever one is the console */

#  ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#  endif
}
#endif

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   arm_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
#ifdef CONSOLE_DEV
  uart_register("/dev/console", &CONSOLE_DEV);
#endif
#ifdef TTYS0_DEV
  uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

void up_putc(int ch)
{
#ifdef HAVE_CONSOLE
  struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;
  uint32_t ier;
  up_disableuartint(priv, &ier);
#endif

  arm_lowputc(ch);
#ifdef HAVE_CONSOLE
  up_restoreuartint(priv, ier);
#endif
}

#endif /* USE_SERIALDRIVER */
