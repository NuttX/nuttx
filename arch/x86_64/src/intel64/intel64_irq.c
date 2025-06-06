/****************************************************************************
 * arch/x86_64/src/intel64/intel64_irq.c
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
#include <nuttx/compiler.h>

#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/pci/pci.h>
#include <arch/arch.h>
#include <arch/irq.h>
#include <arch/io.h>
#include <arch/board/board.h>

#include <nuttx/spinlock.h>
#include <nuttx/signal.h>

#include "sched/sched.h"
#include "x86_64_internal.h"
#include "intel64_cpu.h"
#include "intel64.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define X86_64_MAR_DEST      0xfee00000
#define X86_64_MDR_TYPE      0x4000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct intel64_irq_priv_s
{
  bool busy;
  bool msi;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void up_apic_init(void);
static void up_idtentry(unsigned int index, uint64_t base, uint16_t sel,
                        uint8_t flags, uint8_t ist);
static inline void up_idtinit(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct idt_entry_s        g_idt_entries[NR_IRQS];
static struct intel64_irq_priv_s g_irq_priv[NR_IRQS];
static int                       g_msi_now = IRQ_MSI_START;
static spinlock_t                g_irq_spinlock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ioapic_read
 *
 * Description:
 *  Read a IOAPIC register
 *
 ****************************************************************************/

uint32_t up_ioapic_read(uint32_t reg)
{
  mmio_write32((void *)IOAPIC_BASE + IOAPIC_REG_INDEX, reg);
  return mmio_read32((void *)IOAPIC_BASE + IOAPIC_REG_DATA);
}

/****************************************************************************
 * Name: up_ioapic_write
 *
 * Description:
 *  Write a IOAPIC register
 *
 ****************************************************************************/

void up_ioapic_write(uint32_t reg, uint32_t data)
{
  mmio_write32((void *)IOAPIC_BASE + IOAPIC_REG_INDEX, reg);
  mmio_write32((void *)IOAPIC_BASE + IOAPIC_REG_DATA, data);
}

/****************************************************************************
 * Name: up_ioapic_pin_set_vector
 *
 * Description:
 *  Bind a hardware IRQ number to APIC IRQ number
 *
 ****************************************************************************/

void up_ioapic_pin_set_vector(unsigned int pin,
                              enum ioapic_trigger_mode trigger_mode,
                              unsigned int vector)
{
  up_ioapic_write(IOAPIC_REG_TABLE + pin * 2 + 1,
      up_apic_cpu_id() << (56 - 32));
  up_ioapic_write(IOAPIC_REG_TABLE + pin * 2,
      trigger_mode | vector);
}

/****************************************************************************
 * Name: up_ioapic_mask_pin
 *
 * Description:
 *  Mask an IO-APIC interrupt
 *
 ****************************************************************************/

void up_ioapic_mask_pin(unsigned int pin)
{
  uint32_t cur;

  cur = up_ioapic_read(IOAPIC_REG_TABLE + pin * 2);
  up_ioapic_write(IOAPIC_REG_TABLE + pin * 2,
      cur | IOAPIC_PIN_DISABLE);
}

/****************************************************************************
 * Name: up_ioapic_unmask_pin
 *
 * Description:
 *  Unmask an IO-APIC interrupt
 *
 ****************************************************************************/

void up_ioapic_unmask_pin(unsigned int pin)
{
  uint32_t cur;

  cur = up_ioapic_read(IOAPIC_REG_TABLE + pin * 2);
  up_ioapic_write(IOAPIC_REG_TABLE + pin * 2,
      cur & ~(IOAPIC_PIN_DISABLE));
}

/****************************************************************************
 * Name: up_deinit_8259
 *
 * Description:
 *  Initialize the Legacy 8259 PIC
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_INTEL64_DISABLE_INT_INIT
static void up_deinit_8259(void)
{
  /* First do an initialization to for any pending interrupt to vanish */

  /* Starts the initialization sequence (in cascade mode) */

  outb(X86_PIC_INIT, X86_IO_PORT_PIC1_CMD);
  outb(X86_PIC_INIT, X86_IO_PORT_PIC2_CMD);

  /* Remap the IRQ to 32~ in case of a spurious interrupt */

  outb(IRQ0,     X86_IO_PORT_PIC1_DATA);  /* ICW2: Master PIC vector offset */
  outb(IRQ0 + 8, X86_IO_PORT_PIC2_DATA);  /* ICW2: Slave PIC vector offset */

  /* Tell the PICs they are in cascade */

  /* ICW3: tell Master PIC that there is a slave PIC at IRQ2 (0000 0100) */

  outb(X86_PIC1_CASCADE, X86_IO_PORT_PIC1_DATA);

  /* ICW3: tell Slave PIC its cascade identity (0000 0010) */

  outb(X86_PIC2_CASCADE, X86_IO_PORT_PIC2_DATA);

  /* Put the PICs into 8086 mode */

  outb(X86_PIC_8086, X86_IO_PORT_PIC1_DATA);
  outb(X86_PIC_8086, X86_IO_PORT_PIC2_DATA);

  /* Disable the 8259 PIC by masking all the interrupt and acking them */

  outb(0xff, X86_IO_PORT_PIC1_DATA);
  outb(0xff, X86_IO_PORT_PIC2_DATA);
  outb(X86_PIC_EOI, X86_IO_PORT_PIC1_CMD);
  outb(X86_PIC_EOI, X86_IO_PORT_PIC2_CMD);
}
#endif

/****************************************************************************
 * Name: up_init_apic
 *
 * Description:
 *  Initialize the APIC
 *
 ****************************************************************************/

static void up_apic_init(void)
{
  uint32_t ver;
  uint32_t icrl;
  uint32_t apic_base;

#ifndef CONFIG_ARCH_INTEL64_DISABLE_INT_INIT
  /* Enable the APIC in X2APIC MODE */

  apic_base = read_msr(MSR_IA32_APIC_BASE) & 0xfffff000;
  write_msr(MSR_IA32_APIC_BASE, apic_base | MSR_IA32_APIC_EN |
                                MSR_IA32_APIC_X2APIC | MSR_IA32_APIC_BSP);
#endif

  /* Enable the APIC and setup an spurious interrupt vector */

  write_msr(MSR_X2APIC_SPIV, MSR_X2APIC_SPIV_EN | IRQ_SPURIOUS);

#ifndef CONFIG_ARCH_INTEL64_DISABLE_INT_INIT
  /* Disable the LINT interrupt lines */

  write_msr(MSR_X2APIC_LINT0, MSR_X2APIC_MASKED);
  write_msr(MSR_X2APIC_LINT1, MSR_X2APIC_MASKED);

  /* Disable performance counter overflow interrupts on machines which
   * provide that interrupt entry.
   */

  ver = read_msr(MSR_X2APIC_VER);
  if (((ver >> 16) & 0xff) >= 4)
    {
      write_msr(MSR_X2APIC_LVTPMR, MSR_X2APIC_MASKED);
    }

  /* Map error interrupt to IRQ_ERROR. */

  write_msr(MSR_X2APIC_LERR, MSR_X2APIC_MASKED);

  /* Clear error status register (requires back-to-back writes). */

  write_msr(MSR_X2APIC_ESR, 0);
  write_msr(MSR_X2APIC_ESR, 0);

  /* Ack any outstanding interrupts. */

  write_msr(MSR_X2APIC_EOI, 0);

  /* Send an Init Level De-Assert to synchronize arbitration ID's. */

  write_msr(MSR_X2APIC_ICR, MSR_X2APIC_ICR_BCAST | MSR_X2APIC_ICR_INIT |
                            MSR_X2APIC_ICR_LEVEL);
  do
    {
      icrl = read_msr(MSR_X2APIC_ICR);
    }
  while (icrl & MSR_X2APIC_ICR_DELIVS);

  /* Enable interrupts on the APIC (but not on the processor). */

  write_msr(MSR_X2APIC_TPR, 0);
#endif
}

/****************************************************************************
 * Name: legacy_pic_irq_handler
 *
 * Description:
 *  This function will capture will legacy 8259 PIC IRQ using virtual wire
 *  mode
 *
 ****************************************************************************/

static int unused_code
legacy_pic_irq_handler(int irq, uint32_t *regs, void *arg)
{
  return 0;
}

/****************************************************************************
 * Name: up_init_ioapic
 *
 * Description:
 *  Initialize the IOAPIC
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_INTEL64_DISABLE_INT_INIT
static void up_ioapic_init(void)
{
  uint32_t maxintr;
  int i;

  up_map_region((void *)IOAPIC_BASE, HUGE_PAGE_SIZE,
                X86_PAGE_PRESENT | X86_PAGE_WR | X86_PAGE_NOCACHE);

  /* Setup the IO-APIC, remap the interrupt to 32~ */

  maxintr = (up_ioapic_read(IOAPIC_REG_VER) >> 16) & 0xff;

  for (i = 0; i < maxintr; i++)
    {
      up_ioapic_pin_set_vector(i, TRIGGER_RISING_EDGE |
                               IOAPIC_PIN_DISABLE, IRQ0 + i);
    }
}
#endif

/****************************************************************************
 * Name: up_idtentry
 *
 * Description:
 *   Initialize one IDT entry.
 *
 ****************************************************************************/

static void up_idtentry(unsigned int index, uint64_t base, uint16_t sel,
                        uint8_t flags, uint8_t ist)
{
  struct idt_entry_s *entry = &g_idt_entries[index];

  entry->lobase  = base & 0xffff;
  entry->hibase  = (base >> 16) & 0xffff;
  entry->xhibase = (base >> 32) & 0xffffffff;
  entry->ist     = ist & 0x7;
  entry->sel     = sel;
  entry->zero    = 0;

  /* We don't use software interrupts from user-space (INT) so DPL level
   * can be set to privilege level 0. DPL bits have no effect on hardware
   * interrupts.
   */

  entry->flags   = flags;
}

/****************************************************************************
 * Name: up_idtinit
 *
 * Description:
 *   Initialize the IDT. The Interrupt Descriptor Table (IDT) is a data
 *   structure used by the x86 architecture to implement an interrupt vector
 *   table. The IDT is used by the processor to determine the correct
 *   response to interrupts and exceptions.
 *
 ****************************************************************************/

static inline void up_idtinit(void)
{
  size_t   offset = 0;
  uint64_t vector = 0;
  int      irq    = 0;
  uint16_t sel    = X86_GDT_CODE_SEL;
  uint8_t  ist    = X86_GDT_IST_ISR;

  memset(&g_idt_entries, 0, sizeof(g_idt_entries));

  /* Set each ISR/IRQ to the appropriate vector with selector=8 and with
   * 32-bit interrupt gate.  Interrupt gate (vs. trap gate) will leave
   * interrupts enabled when the IRS/IRQ handler is entered.
   */

  up_idtentry(ISR0,  (uint64_t)vector_isr0 , sel, 0x8e, ist);
  up_idtentry(ISR1,  (uint64_t)vector_isr1 , sel, 0x8e, ist);
  up_idtentry(ISR2,  (uint64_t)vector_isr2 , sel, 0x8e, ist);
  up_idtentry(ISR3,  (uint64_t)vector_isr3 , sel, 0x8e, ist);
  up_idtentry(ISR4,  (uint64_t)vector_isr4 , sel, 0x8e, ist);
  up_idtentry(ISR5,  (uint64_t)vector_isr5 , sel, 0x8e, ist);
  up_idtentry(ISR6,  (uint64_t)vector_isr6 , sel, 0x8e, ist);
  up_idtentry(ISR7,  (uint64_t)vector_isr7 , sel, 0x8e, ist);
  up_idtentry(ISR8,  (uint64_t)vector_isr8 , sel, 0x8e, ist);
  up_idtentry(ISR9,  (uint64_t)vector_isr9 , sel, 0x8e, ist);
  up_idtentry(ISR10, (uint64_t)vector_isr10, sel, 0x8e, ist);
  up_idtentry(ISR11, (uint64_t)vector_isr11, sel, 0x8e, ist);
  up_idtentry(ISR12, (uint64_t)vector_isr12, sel, 0x8e, ist);
  up_idtentry(ISR13, (uint64_t)vector_isr13, sel, 0x8e, ist);
  up_idtentry(ISR14, (uint64_t)vector_isr14, sel, 0x8e, ist);
  up_idtentry(ISR15, (uint64_t)vector_isr15, sel, 0x8e, ist);
  up_idtentry(ISR16, (uint64_t)vector_isr16, sel, 0x8e, ist);
  up_idtentry(ISR17, (uint64_t)vector_isr17, sel, 0x8e, ist);
  up_idtentry(ISR18, (uint64_t)vector_isr18, sel, 0x8e, ist);
  up_idtentry(ISR19, (uint64_t)vector_isr19, sel, 0x8e, ist);
  up_idtentry(ISR20, (uint64_t)vector_isr20, sel, 0x8e, ist);
  up_idtentry(ISR21, (uint64_t)vector_isr21, sel, 0x8e, ist);
  up_idtentry(ISR22, (uint64_t)vector_isr22, sel, 0x8e, ist);
  up_idtentry(ISR23, (uint64_t)vector_isr23, sel, 0x8e, ist);
  up_idtentry(ISR24, (uint64_t)vector_isr24, sel, 0x8e, ist);
  up_idtentry(ISR25, (uint64_t)vector_isr25, sel, 0x8e, ist);
  up_idtentry(ISR26, (uint64_t)vector_isr26, sel, 0x8e, ist);
  up_idtentry(ISR27, (uint64_t)vector_isr27, sel, 0x8e, ist);
  up_idtentry(ISR28, (uint64_t)vector_isr28, sel, 0x8e, ist);
  up_idtentry(ISR29, (uint64_t)vector_isr29, sel, 0x8e, ist);
  up_idtentry(ISR30, (uint64_t)vector_isr30, sel, 0x8e, ist);
  up_idtentry(ISR31, (uint64_t)vector_isr31, sel, 0x8e, ist);

  /* Set all IRQ vectors and use separate stack from ISR */

  ist = X86_GDT_IST_IRQ;
  offset = (uint64_t)vector_irq1 - (uint64_t)vector_irq0;

  for (irq = IRQ0, vector = (uint64_t)vector_irq0;
       irq <= IRQ255;
       irq += 1, vector += offset)
    {
      up_idtentry(irq,  (uint64_t)vector,  sel, 0x8e, ist);
    }

  /* Then program the IDT */

  setidt(&g_idt_entries, sizeof(struct idt_entry_s) * NR_IRQS - 1);
}

/****************************************************************************
 * Name: arm_color_intstack
 *
 * Description:
 *   Set the interrupt stack to a value so that later we can determine how
 *   much stack space was used by interrupt handling logic
 *
 ****************************************************************************/

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 3
static inline void x86_64_color_intstack(void)
{
  x86_64_stack_color((void *)up_get_intstackbase(this_cpu()),
                     IRQ_STACK_SIZE);
}
#else
#  define x86_64_color_intstack()
#endif

/****************************************************************************
 * Name: x86_64_fault_panic_isr
 ****************************************************************************/

static int x86_64_fault_panic_isr(int irq, void *c, void *arg)
{
  uint64_t *regs = (uint64_t *)running_regs();

  /* Let's say, all ISR are asserted when REALLY BAD things happened.
   * Don't even brother to recover, just dump the regs and PANIC.
   */

  _alert("PANIC:\n");
  _alert("Exception %" PRId32 " occurred "
         "with error code %" PRId64 ":\n",
         irq, regs[REG_ERRCODE]);

  PANIC_WITH_REGS("panic", regs);

  up_trash_cpu();

  return 0;
}

/****************************************************************************
 * Name: x86_64_fault_kill_isr
 ****************************************************************************/

static int x86_64_fault_kill_isr(int irq, void *c, void *arg)
{
  __asm__ volatile("fnclex":::"memory");

  nxsig_kill(running_task()->pid, SIGFPE);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  int cpu = this_cpu();

  /* Initialize the TSS */

  x86_64_cpu_tss_init(cpu);

  /* Colorize the interrupt stack */

  x86_64_color_intstack();

  /* Initialize the APIC */

  up_apic_init();

  if (cpu == 0)
    {
#ifndef CONFIG_ARCH_INTEL64_DISABLE_INT_INIT
      /* Disable 8259 PIC */

      up_deinit_8259();

      /* Initialize the IOAPIC */

      up_ioapic_init();
#endif

      /* Initialize the IDT */

      up_idtinit();
    }

  /* Program the IDT - one per all cores */

  setidt(&g_idt_entries, sizeof(struct idt_entry_s) * NR_IRQS - 1);

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_restore(X86_64_RFLAGS_IF);
#endif

  /* Attach default handlers for faults */

  irq_attach(ISR0, x86_64_fault_kill_isr, NULL);
  irq_attach(ISR6, x86_64_fault_panic_isr, NULL);
  irq_attach(ISR8, x86_64_fault_panic_isr, NULL);
  irq_attach(ISR13, x86_64_fault_panic_isr, NULL);
  irq_attach(ISR14, x86_64_fault_panic_isr, NULL);
  irq_attach(ISR16, x86_64_fault_kill_isr, NULL);
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ line specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
#ifndef CONFIG_ARCH_INTEL64_DISABLE_INT_INIT
  irqstate_t flags = spin_lock_irqsave(&g_irq_spinlock);

  DEBUGASSERT(irq <= IRQ255);

  /* Do nothing if this is MSI/MSI-X */

  if (g_irq_priv[irq].msi)
    {
      spin_unlock_irqrestore(&g_irq_spinlock, flags);
      return;
    }

  if (g_irq_priv[irq].busy)
    {
      /* One time disable */

      if (irq >= IRQ0)
        {
          up_ioapic_mask_pin(irq - IRQ0);
        }

      g_irq_priv[irq].busy = false;
    }

  spin_unlock_irqrestore(&g_irq_spinlock, flags);
#endif
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ line specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
#ifndef CONFIG_ARCH_INTEL64_DISABLE_INT_INIT
  irqstate_t flags = spin_lock_irqsave(&g_irq_spinlock);

  DEBUGASSERT(irq <= IRQ255);

  /* Do nothing if this is MSI/MSI-X */

  if (g_irq_priv[irq].msi)
    {
      spin_unlock_irqrestore(&g_irq_spinlock, flags);
      return;
    }

#  ifndef CONFIG_IRQCHAIN
  /* Check if IRQ is free if we don't support IRQ chains */

  if (g_irq_priv[irq].busy)
    {
      ASSERT(0);
    }
#  endif

  if (!g_irq_priv[irq].busy)
    {
      /* One time enable */

      if (irq >= IRQ0)
        {
          up_ioapic_unmask_pin(irq - IRQ0);
        }

      g_irq_priv[irq].busy = true;
    }

  spin_unlock_irqrestore(&g_irq_spinlock, flags);
#endif
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
#warning "Missing Logic"
  return OK;
}
#endif

/****************************************************************************
 * Name: up_trigger_irq
 *
 * Description:
 *   Trigger IRQ interrupt.
 *
 ****************************************************************************/

void up_trigger_irq(int irq, cpu_set_t cpuset)
{
  uint32_t cpu;

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      if (CPU_ISSET(cpu, &cpuset))
        {
          write_msr(MSR_X2APIC_ICR,
                    MSR_X2APIC_ICR_FIXED |
                    MSR_X2APIC_ICR_ASSERT |
                    MSR_X2APIC_DESTINATION(
                      (uint64_t)x86_64_cpu_to_loapic(cpu)) |
                    irq);
        }
    }
}

/****************************************************************************
 * Name: up_get_legacy_irq
 *
 * Description:
 *   Reserve vector for legacy
 *
 ****************************************************************************/

int up_get_legacy_irq(uint32_t devfn, uint8_t line, uint8_t pin)
{
  UNUSED(devfn);
  UNUSED(pin);
  return IRQ0 + line;
}

/****************************************************************************
 * Name: up_alloc_irq_msi
 *
 * Description:
 *   Reserve vector for MSI/MSI-X
 *
 ****************************************************************************/

int up_alloc_irq_msi(uint8_t busno, uint32_t devfn, int *pirq, int num)
{
  irqstate_t flags = spin_lock_irqsave(&g_irq_spinlock);
  int        irq   = 0;
  int        i     = 0;

  /* Limit requested number of vectors */

  if (g_msi_now + num > IRQ255)
    {
      num = IRQ255 - g_msi_now;
    }

  if (num <= 0)
    {
      spin_unlock_irqrestore(&g_irq_spinlock, flags);

      /* No IRQs available */

      return -ENODEV;
    }

  irq = g_msi_now;
  g_msi_now += num;

  /* Mark IRQs as MSI/MSI-X */

  for (i = 0; i < num; i++)
    {
      ASSERT(g_irq_priv[irq + i].busy == false);
      g_irq_priv[irq + i].busy = true;
      g_irq_priv[irq + i].msi  = true;
      pirq[i] = irq + i;
    }

  spin_unlock_irqrestore(&g_irq_spinlock, flags);

  return num;
}

/****************************************************************************
 * Name: up_release_irq_msi
 *
 * Description:
 *   Release MSI/MSI-X vector
 *
 ****************************************************************************/

void up_release_irq_msi(int *irq, int num)
{
  irqstate_t flags = spin_lock_irqsave(&g_irq_spinlock);
  int        i     = 0;

  /* Mark IRQ as MSI/MSI-X */

  for (i = 0; i < num; i++)
    {
      g_irq_priv[irq[i]].busy = false;
      g_irq_priv[irq[i]].msi  = false;
    }

  spin_unlock_irqrestore(&g_irq_spinlock, flags);
}

/****************************************************************************
 * Name: up_connect_irq
 *
 * Description:
 *   Connect MSI/MSI-X vector to irq
 *
 ****************************************************************************/

int up_connect_irq(const int *irq, int num, uintptr_t *mar, uint32_t *mdr)
{
  UNUSED(num);

  if (mar != NULL)
    {
      *mar = X86_64_MAR_DEST |
        (up_apic_cpu_id() << PCI_MSI_DATA_CPUID_SHIFT);
    }

  if (mdr != NULL)
    {
      *mdr = X86_64_MDR_TYPE | irq[0];
    }

  return OK;
}
