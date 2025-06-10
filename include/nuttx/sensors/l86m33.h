/****************************************************************************
 * include/nuttx/sensors/l86m33.h
 *
 * NOTE: EXPERIMENTAL DRIVER
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

#ifndef __INCLUDE_NUTTX_SENSORS_L86M33_H
#define __INCLUDE_NUTTX_SENSORS_L86M33_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

typedef enum {
  L86M33_BAUD_4800 = 4800,
  L86M33_BAUD_9600 = 9600,
  L86M33_BAUD_14400 = 14400,
  L86M33_BAUD_19200 = 19200,
  L86M33_BAUD_38400 = 38400,
  L86M33_BAUD_57600 = 57600,
  L86M33_BAUD_115200 = 115200
} L866M33_SUPPORTED_BAUD_RATES;

typedef enum {
  L86M33_CHANGE_BAUD,
  L86M33_PQEPE_OUTPUT,
  L86M33_QUERY,
  L86M33_CHANGE_FIX_INT,
} L86M33_COMMAND;

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rn2xx3_register
 *
 * Description:
 *   Register the RN2xx3 LoRa transceiver driver.
 *
 * Arguments:
 *    devpath   -  The device path to use for the driver
 *    uartpath  -  The path to the UART character driver connected to the
 *                 transceiver
 *    devno     -  The device number to use for the topic (i.e. /dev/mag0)
 *    br        -  The baud rate to configure the device with (i.e 9600)
 *    ur        -  The update rate to configure the device with (i.e 1)
 *
 ****************************************************************************/

int l86m33_register(FAR const char *devpath, FAR const char *uartpath, int devno);

#endif /* __INCLUDE_NUTTX_SENSORS_L86M33_H */