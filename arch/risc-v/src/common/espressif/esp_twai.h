/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_twai.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_TWAI_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_TWAI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/can/can.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_twaiinitialize
 *
 * Description:
 *   Initialize CAN port
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple TWAI interfaces)
 *
 * Returned Value:
 *   Valid TWAI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

#if defined(CONFIG_CAN) && defined(CONFIG_ESPRESSIF_TWAI)
struct can_dev_s *esp_twaiinitialize(int port);
#endif

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_TWAI_H */
