/****************************************************************************
 * arch/arm/src/armv7-a/arm_gicv2.c
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
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/pci/pci.h>
#include <nuttx/spinlock.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "gic.h"

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE) || defined(CONFIG_ARCH_TRUSTZONE_NONSECURE)
# if defined(CONFIG_ARCH_HIPRI_INTERRUPT)
#  error "ARCH_HIPRI_INTERRUPT must configure with ARCH_TRUSTZONE_DISABLED or no trustzone"
# endif
#endif

#ifdef CONFIG_ARMV7A_HAVE_GICv2

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_SMP_NCPUS > 1
static atomic_t g_gic_init_done;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_gic_init_done
 *
 * Description:
 *   Indicates gic init done, and only cpu0 need wait the gic initialize
 *   done. Because cpu1 ~ (CONFIG_SMP_NCPUS - 1) may not response the
 *   interrupt request by cpu0 when the gic not initialize done.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_SMP_NCPUS > 1
static void arm_gic_init_done(void)
{
  atomic_fetch_or(&g_gic_init_done, 1 << this_cpu());
}

static void arm_gic_wait_done(cpu_set_t cpuset)
{
  cpu_set_t tmpset;

  do
    {
      tmpset = (cpu_set_t)atomic_read(&g_gic_init_done) & cpuset;
    }
  while (!CPU_EQUAL(&tmpset, &cpuset));
}
#else
#define arm_gic_init_done()
#define arm_gic_wait_done(cpuset)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE) || defined(CONFIG_ARCH_HIPRI_INTERRUPT)
/****************************************************************************
 * Name: up_set_secure_irq
 *
 * Description:
 *   Secure an IRQ
 *
 ****************************************************************************/

void up_secure_irq(int irq, bool secure)
{
  unsigned int val;

  if (secure)
    {
      val = getreg32(GIC_ICDISR(irq)) & (~GIC_ICDISR_INT(irq));  /* group 0 */
    }
  else
    {
      val = getreg32(GIC_ICDISR(irq)) | GIC_ICDISR_INT(irq);     /* group 1 */
    }

  putreg32(val, GIC_ICDISR(irq));
}

/****************************************************************************
 * Name: up_secure_irq_all
 *
 * Description:
 *   Secure all IRQ
 *
 ****************************************************************************/

void up_secure_irq_all(bool secure)
{
  unsigned int nlines = arm_gic_nlines();
  unsigned int irq;

  for (irq = 0; irq < nlines; irq += 32)
    {
      if (secure)
        {
          putreg32(0x00000000, GIC_ICDISR(irq));   /* group 0 */
        }
      else
        {
          putreg32(0xffffffff, GIC_ICDISR(irq));   /* group 1 */
        }
    }
}
#endif

/****************************************************************************
 * Name: arm_gic0_initialize
 *
 * Description:
 *   Perform common, one-time GIC initialization on CPU0 only.  Both
 *   arm_gic0_initialize() must be called on CPU0; arm_gic_initialize() must
 *   be called for all CPUs.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_gic0_initialize(void)
{
  unsigned int nlines = arm_gic_nlines();
  unsigned int irq;

  arm_gic_dump("Entry arm_gic0_initialize", true, 0);

  /* Initialize SPIs.  The following should be done only by CPU0. */

  /* A processor in Secure State sets:
   *
   * 1. Which interrupts are non-secure (ICDISR).  All set to one (group
   *    1).
   * 2. Trigger mode of the SPI (ICDICFR). All fields set to 0b01->Level
   *    sensitive, 1-N model.
   * 3. Interrupt Clear-Enable (ICDICER)
   * 3. Priority of the SPI using the priority set register (ICDIPR).
   *    Priority values are 8-bit unsigned binary. A GIC supports a
   *    minimum of 16 and a maximum of 256 priority levels. Here all
   *    are set to the middle priority 128 (0x80).
   * 4. Target that receives the SPI interrupt (ICDIPTR).  Set all to
   *    CPU0.
   */

  /* Registers with 1-bit per interrupt */

  for (irq = GIC_IRQ_SPI; irq < nlines; irq += 32)
    {
      putreg32(0xffffffff, GIC_ICDISR(irq));   /* SPIs group 1 */
      putreg32(0xffffffff, GIC_ICDICER(irq));  /* SPIs disabled */
    }

  /* Registers with 2-bits per interrupt */

  for (irq = GIC_IRQ_SPI; irq < nlines; irq += 16)
    {
      putreg32(0x55555555, GIC_ICDICFR(irq));  /* SPIs level sensitive */
    }

  /* Registers with 8-bits per interrupt */

  for (irq = GIC_IRQ_SPI; irq < nlines; irq += 4)
    {
      putreg32(0x80808080, GIC_ICDIPR(irq));   /* SPI priority */
      putreg32(0x01010101, GIC_ICDIPTR(irq));  /* SPI on CPU0 */
    }

#ifdef CONFIG_ARMV7A_GICv2M
  gic_v2m_initialize();
#endif

#ifdef CONFIG_SMP
  /* Attach SGI interrupt handlers. This attaches the handler to all CPUs. */

  DEBUGVERIFY(irq_attach(GIC_SMP_CPUSTART, arm_start_handler, NULL));
  DEBUGVERIFY(irq_attach(GIC_SMP_SCHED, arm_smp_sched_handler, NULL));
  DEBUGVERIFY(irq_attach(GIC_SMP_CALL, nxsched_smp_call_handler, NULL));
#endif

  arm_gic_dump("Exit arm_gic0_initialize", true, 0);
}

/****************************************************************************
 * Name: arm_gic_initialize
 *
 * Description:
 *   Perform common GIC initialization for the current CPU (all CPUs)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_gic_initialize(void)
{
  uint32_t iccicr;
  uint32_t icddcr;

  arm_gic_dump("Entry arm_gic_initialize", true, 0);

  /* Initialize PPIs.  The following steps need to be done by all CPUs */

  /* Initialize SGIs and PPIs.  NOTE: A processor in non-secure state cannot
   * program its interrupt security registers and must get a secure processor
   * to program the registers.
   */

  /* Registers with 1-bit per interrupt */

#ifdef CONFIG_ARCH_TRUSTZONE_SECURE
  /* per-CPU inerrupts config:
   * ID0-ID7(SGI)  for Non-secure interrupts
   * ID8-ID15(SGI)  for Secure interrupts.
   * All PPI config as secure interrupts.
   */

  putreg32(0x000000ff, GIC_ICDISR(0));
#else
  putreg32(0xffffffff, GIC_ICDISR(0));      /* SGIs and PPIs no-secure */
#endif
  putreg32(0xfe000000, GIC_ICDICER(0));     /* PPIs disabled */

  /* Registers with 8-bits per interrupt */

  putreg32(0x80808080, GIC_ICDIPR(0));      /* SGI[3:0] priority */
  putreg32(0x80808080, GIC_ICDIPR(4));      /* SGI[4:7] priority */
  putreg32(0x80808080, GIC_ICDIPR(8));      /* SGI[8:11] priority */
  putreg32(0x80808080, GIC_ICDIPR(12));     /* SGI[12:15] priority */
  putreg32(0x80808000, GIC_ICDIPR(24));     /* PPI[0] priority */
  putreg32(0x80808080, GIC_ICDIPR(28));     /* PPI[1:4] priority */

  /* Set the binary point register.
   *
   * Priority values are 8-bit unsigned binary.  The binary point is a 3-bit
   * field; the value n (n=0-6) specifies that bits (n+1) through bit 7 are
   * used in the comparison for interrupt pre-emption.  A GIC supports a
   * minimum of 16 and a maximum of 256 priority levels so not all binary
   * point settings may be meaningul. When CONFIG_ARCH_HIPRI_INTERRUPT is not
   * enabled, we set n=7 (GIC_ICCBPR_NOPREMPT) to disable interrupt nesting.
   * When CONFIG_ARCH_HIPRI_INTERRUPT is enabled, we set n=6 (GIC_ICCBPR_7_7)
   * (g.sssssss) to support group priority.
   */

#ifdef CONFIG_ARCH_HIPRI_INTERRUPT
  putreg32(GIC_ICCBPR_7_7, GIC_ICCBPR);
#else
  putreg32(GIC_ICCBPR_NOPREMPT, GIC_ICCBPR);
#endif

  /* Program the idle priority in the PMR */

  putreg32(GIC_ICCPMR_MASK, GIC_ICCPMR);

  /* Configure the  CPU Interface Control Register */

  iccicr  = getreg32(GIC_ICCICR);

#ifdef CONFIG_ARCH_TRUSTZONE_NONSECURE
  /* Clear non-secure state ICCICR bits to be configured below */

  iccicr &= ~(GIC_ICCICRU_EOIMODENS | GIC_ICCICRU_ENABLEGRP1 |
              GIC_ICCICRU_FIQBYPDISGRP1 | GIC_ICCICRU_IRQBYPDISGRP1);
#else
  /* Clear secure state ICCICR bits to be configured below */

  iccicr &= ~(GIC_ICCICRS_FIQEN | GIC_ICCICRS_ACKTCTL | GIC_ICCICRS_CBPR |
              GIC_ICCICRS_EOIMODES | GIC_ICCICRS_EOIMODENS |
              GIC_ICCICRS_ENABLEGRP0 | GIC_ICCICRS_ENABLEGRP1 |
              GIC_ICCICRS_FIQBYPDISGRP0 | GIC_ICCICRS_IRQBYPDISGRP0 |
              GIC_ICCICRS_FIQBYPDISGRP1 | GIC_ICCICRS_IRQBYPDISGRP1);
#endif

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE) || defined(CONFIG_ARCH_HIPRI_INTERRUPT)
  /* Set FIQn=1 if secure interrupts are to signal using nfiq_c.
   *
   * NOTE:  Only for processors that operate in secure state.
   * REVISIT: Do I need to do this?
   */

  iccicr |= GIC_ICCICRS_FIQEN;
#endif

#ifdef CONFIG_ARMV7A_GIC_EOIMODE
#  if defined(CONFIG_ARCH_TRUSTZONE_SECURE)
  /* Set EnableS=1 to enable CPU interface to signal secure interrupts.
   *
   * NOTE:  Only for processors that operate in secure state.
   */

  iccicr |= GIC_ICCICRS_EOIMODES;
#  else
  /* Set EnableNS=1 to enable the CPU to signal non-secure interrupts.
   *
   * NOTE:  Only for processors that operate in non-secure state.
   */

  iccicr |= GIC_ICCICRU_EOIMODENS;
#  endif
#endif

  iccicr |= GIC_ICCICRS_ACKTCTL;

#ifdef CONFIG_ARCH_TRUSTZONE_NONSECURE
  /* Enable the Group 1 interrupts and disable Group 1 bypass. */

  iccicr |= (GIC_ICCICRU_ENABLEGRP1 | GIC_ICCICRU_FIQBYPDISGRP1 |
             GIC_ICCICRU_IRQBYPDISGRP1);
  icddcr  = GIC_ICDDCR_ENABLE;
#else
  /* Enable the Group 0 interrupts, FIQEn and disable Group 0/1
   * bypass.
   */

  iccicr |= (GIC_ICCICRS_ENABLEGRP0 | GIC_ICCICRS_ENABLEGRP1 |
             GIC_ICCICRS_FIQBYPDISGRP0 | GIC_ICCICRS_IRQBYPDISGRP0 |
             GIC_ICCICRS_FIQBYPDISGRP1 | GIC_ICCICRS_IRQBYPDISGRP1);
  icddcr  = (GIC_ICDDCR_ENABLEGRP0 | GIC_ICDDCR_ENABLEGRP1);
#endif

  /* Write the final ICCICR value to enable the GIC. */

  putreg32(iccicr, GIC_ICCICR);

  /* Write the ICDDCR value to enable the forwarding of interrupt by the
   * distributor.
   */

  putreg32(icddcr, GIC_ICDDCR);
  arm_gic_dump("Exit arm_gic_initialize", true, 0);
  arm_gic_init_done();
}

/****************************************************************************
 * Name: arm_decodeirq
 *
 * Description:
 *   This function is called from the IRQ vector handler in arm_vectors.S.
 *   At this point, the interrupt has been taken and the registers have
 *   been saved on the stack.  This function simply needs to determine the
 *   the irq number of the interrupt and then to call arm_doirq to dispatch
 *   the interrupt.
 *
 *  Input Parameters:
 *   regs - A pointer to the register save area on the stack.
 *
 ****************************************************************************/

uint32_t *arm_decodeirq(uint32_t *regs)
{
  uint32_t regval;
  int irq;

  /* Read the interrupt acknowledge register and get the interrupt ID */

  regval = getreg32(GIC_ICCIAR);
  irq    = (regval & GIC_ICCIAR_INTID_MASK) >> GIC_ICCIAR_INTID_SHIFT;

#ifdef CONFIG_ARMV7A_GIC_EOIMODE
  putreg32(regval, GIC_ICCEOIR);
#endif

  /* Ignore spurions IRQs.  ICCIAR will report 1023 if there is no pending
   * interrupt.
   */

  DEBUGASSERT(irq < NR_IRQS || irq >= 1022);
  if (irq < NR_IRQS)
    {
      /* Dispatch the interrupt */

      regs = arm_doirq(irq, regs);
    }

  /* Write to the end-of-interrupt register */

#ifdef CONFIG_ARMV7A_GIC_EOIMODE
  putreg32(regval, GIC_ICCDIR);
#else
  putreg32(regval, GIC_ICCEOIR);
#endif
  return regs;
}

/****************************************************************************
 * Name: arm_decodefiq
 *
 * Description:
 *   This function is called from the FIQ vector handler in arm_vectors.S.
 *   At this point, the interrupt has been taken and the registers have
 *   been saved on the stack.  This function simply needs to determine the
 *   the irq number of the interrupt and then to call arm_doirq to dispatch
 *   the interrupt.
 *
 *  Input Parameters:
 *   regs - A pointer to the register save area on the stack.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE) || defined(CONFIG_ARCH_HIPRI_INTERRUPT)
uint32_t *arm_decodefiq(uint32_t *regs)
{
  uint32_t regval;
  int irq;

  /* Read the interrupt acknowledge register and get the interrupt ID */

  regval = getreg32(GIC_ICCIAR);
  irq    = (regval & GIC_ICCIAR_INTID_MASK) >> GIC_ICCIAR_INTID_SHIFT;

#  ifdef CONFIG_ARMV7A_GIC_EOIMODE
  putreg32(regval, GIC_ICCEOIR);
#  endif

  /* Ignore spurions IRQs.  ICCIAR will report 1023 if there is no pending
   * interrupt.
   */

  DEBUGASSERT(irq < NR_IRQS || irq >= 1022);

  if (irq < NR_IRQS)
    {
      /* Dispatch the interrupt */

#  ifdef CONFIG_ARCH_HIPRI_INTERRUPT
      regs = arm_dofiq(irq, regs);
#  else
      regs = arm_doirq(irq, regs);
#  endif
    }

  /* Write to the end-of-interrupt register */

#  ifdef CONFIG_ARMV7A_GIC_EOIMODE
  putreg32(regval, GIC_ICCDIR);
#  else
  putreg32(regval, GIC_ICCEOIR);
#  endif
  return regs;
}
#endif

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   On many architectures, there are three levels of interrupt enabling: (1)
 *   at the global level, (2) at the level of the interrupt controller,
 *   and (3) at the device level.  In order to receive interrupts, they
 *   must be enabled at all three levels.
 *
 *   This function implements enabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (up_irq_restore() supports the global level, the device level is
 *   hardware specific).
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  /* Ignore invalid interrupt IDs.  Also, in the Cortex-A9 MPCore, SGIs are
   * always enabled. The corresponding bits in the ICDISERn are read as
   * one, write ignored.
   */

  if (irq > GIC_IRQ_SGI15 && irq < NR_IRQS)
    {
      uintptr_t regaddr;

      /* Write '1' to the corresponding bit in the distributor Interrupt
       * Set-Enable Register (ICDISER)
       */

      regaddr = GIC_ICDISER(irq);
      putreg32(GIC_ICDISER_INT(irq), regaddr);

      arm_gic_dump("Exit up_enable_irq", false, irq);
    }
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   This function implements disabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (up_irq_save() supports the global level, the device level is hardware
 *   specific).
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  /* Ignore invalid interrupt IDs.  Also, in the Cortex-A9 MPCore, SGIs are
   * always enabled. The corresponding bits in the ICDISERn are read as
   * one, write ignored.
   */

  if (irq > GIC_IRQ_SGI15 && irq < NR_IRQS)
    {
      uintptr_t regaddr;

      /* Write '1' to the corresponding bit in the distributor Interrupt
       * Clear-Enable Register (ICDISER)
       */

      regaddr = GIC_ICDICER(irq);
      putreg32(GIC_ICDICER_INT(irq), regaddr);

      arm_gic_dump("Exit up_disable_irq", false, irq);
    }
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *   For group0, priority bit[7] must be 0;
 *   For group1, priority bit[7] must be 1;
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

int up_prioritize_irq(int irq, int priority)
{
  DEBUGASSERT(irq >= 0 && irq < NR_IRQS && priority >= 0 && priority <= 255);

  /* Ignore invalid interrupt IDs */

  if (irq >= 0 && irq < NR_IRQS)
    {
      uintptr_t regaddr;
      uint32_t regval;

      /* Write the new priority to the corresponding field in the in the
       * distributor Interrupt Priority Register (GIC_ICDIPR).
       */

      regaddr = GIC_ICDIPR(irq);
      regval  = getreg32(regaddr);
      regval &= ~GIC_ICDIPR_ID_MASK(irq);
      regval |= GIC_ICDIPR_ID(irq, priority);
      putreg32(regval, regaddr);

      arm_gic_dump("Exit up_prioritize_irq", false, irq);
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: up_affinity_irq
 *
 * Description:
 *   Set an IRQ affinity by software.
 *
 ****************************************************************************/

void up_affinity_irq(int irq, cpu_set_t cpuset)
{
  if (irq >= GIC_IRQ_SPI && irq < NR_IRQS)
    {
      uintptr_t regaddr;
      uint32_t regval;

      /* Write the new cpuset to the corresponding field in the in the
       * distributor Interrupt Processor Target Register (GIC_ICDIPTR).
       */

      regaddr = GIC_ICDIPTR(irq);
      regval  = getreg32(regaddr);
      regval &= ~GIC_ICDIPTR_ID_MASK(irq);
      regval |= GIC_ICDIPTR_ID(irq, cpuset);
      putreg32(regval, regaddr);
    }
}

/****************************************************************************
 * Name: up_trigger_irq
 *
 * Description:
 *   Perform a Software Generated Interrupt (SGI).  If CONFIG_SMP is
 *   selected, then the SGI is sent to all CPUs specified in the CPU set.
 *   That set may include the current CPU.
 *
 *   If CONFIG_SMP is not selected, the cpuset is ignored and SGI is sent
 *   only to the current CPU.
 *
 * Input Parameters
 *   irq    - The SGI interrupt ID (0-15)
 *   cpuset - The set of CPUs to receive the SGI
 *
 ****************************************************************************/

void up_trigger_irq(int irq, cpu_set_t cpuset)
{
  if (irq >= 0 && irq <= GIC_IRQ_SGI15)
    {
      arm_cpu_sgi(irq, cpuset);
    }
  else if (irq >= 0 && irq < NR_IRQS)
    {
      uintptr_t regaddr;

      /* Write '1' to the corresponding bit in the distributor Interrupt
       * Set-Pending (ICDISPR)
       */

      regaddr = GIC_ICDISPR(irq);
      putreg32(GIC_ICDISPR_INT(irq), regaddr);
    }
}

/****************************************************************************
 * Name: arm_gic_irq_trigger
 *
 * Description:
 *   Set the trigger type for the specified IRQ source and the current CPU.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 * Input Parameters:
 *   irq - The interrupt request to modify.
 *   edge - False: Active HIGH level sensitive, True: Rising edge sensitive
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int arm_gic_irq_trigger(int irq, bool edge)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t intcfg;

  if (irq > GIC_IRQ_SGI15 && irq < NR_IRQS)
    {
      /* Get the address of the Interrupt Configuration Register for this
       * irq.
       */

      regaddr = GIC_ICDICFR(irq);

      /* Get the new Interrupt configuration bit setting */

      intcfg = (edge ? (INT_ICDICFR_EDGE | INT_ICDICFR_1N) : INT_ICDICFR_1N);

      /* Write the correct interrupt trigger to the Interrupt Configuration
       * Register.
       */

      regval  = getreg32(regaddr);
      regval &= ~GIC_ICDICFR_ID_MASK(irq);
      regval |= GIC_ICDICFR_ID(irq, intcfg);
      putreg32(regval, regaddr);

      return OK;
    }

  return -EINVAL;
}

void arm_cpu_sgi(int sgi, unsigned int cpuset)
{
  uint32_t regval;

  arm_gic_wait_done(cpuset);

  regval = GIC_ICDSGIR_INTID(sgi) | GIC_ICDSGIR_CPUTARGET(cpuset) |
           GIC_ICDSGIR_TGTFILTER_LIST;

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE)
  if (sgi >= GIC_IRQ_SGI0 && sgi <= GIC_IRQ_SGI7)
#endif
    {
      /* Set NSATT be 1: forward the SGI specified in the SGIINTID field to a
       * specified CPU interfaces only if the SGI is configured as Group 1 on
       * that interface.
       * For non-secure context, the configuration of GIC_ICDSGIR_NSATT_GRP1
       * is not mandatory in the GICv2 specification, but for SMP scenarios,
       * this value needs to be configured, otherwise issues may occur in the
       * SMP scenario.
       */

      regval |= GIC_ICDSGIR_NSATT_GRP1;
    }

  putreg32(regval, GIC_ICDSGIR);
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
#if CONFIG_ARMV7A_GICV2_LEGACY_IRQ0 >= 0
  uint8_t slot;
  uint8_t tmp;

  UNUSED(line);
  slot = PCI_SLOT(devfn);
  tmp = (pin - 1 + slot) % 4;
  return CONFIG_ARMV7A_GICV2_LEGACY_IRQ0 + tmp;
#else
  return -ENOTSUP;
#endif
}

#endif /* CONFIG_ARMV7A_HAVE_GICv2 */
