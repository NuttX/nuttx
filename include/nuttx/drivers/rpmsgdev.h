/****************************************************************************
 * include/nuttx/drivers/rpmsgdev.h
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

#ifndef __INCLUDE_NUTTX_DRIVERS_RPMSGDEV_H
#define __INCLUDE_NUTTX_DRIVERS_RPMSGDEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define RPMSGDEV_NOFRAG_READ  0x1
#define RPMSGDEV_NOFRAG_WRITE 0x2

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

/****************************************************************************
 * Name: rpmsgdev_server_init
 *
 * Description:
 *   Rpmsg-device server initialize function, the server cpu should call
 *   this function.
 *
 * Parameters:
 *   None
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_RPMSG_SERVER
int rpmsgdev_server_init(void);
int rpmsgdev_export(FAR const char *remotecpu, FAR const char *localpath);
#endif

/****************************************************************************
 * Name: rpmsgdev_register
 *
 * Description:
 *   Rpmsg-device client initialize function, the client cpu should call
 *   this function in the board initialize process.
 *
 * Parameters:
 *   remotecpu  - the server cpu name
 *   remotepath - the device you want to access in the remote cpu
 *   localpath  - the device path in local cpu, if NULL, the localpath is
 *                same as the remotepath, provide this argument to support
 *                custom device path
 *   flags      - RPMSGDEV_NOFRAG_READ and RPMSGDEV_NOFRAG_WRITE can be set
 *                to indicates that the read and write data of the device
 *                cannot be split or aggregated
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_RPMSG
int rpmsgdev_register(FAR const char *remotecpu, FAR const char *remotepath,
                      FAR const char *localpath, uint32_t flags);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_DRIVERS_RPMSGDEV_H */
