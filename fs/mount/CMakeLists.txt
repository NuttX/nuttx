# ##############################################################################
# fs/mount/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

# Don't build anything if there is no mountpoint support

if(NOT CONFIG_DISABLE_MOUNTPOINT)
  set(SRCS fs_mount.c fs_umount2.c fs_foreachmountpoint.c)

  if(CONFIG_FS_AUTOMOUNTER)
    list(APPEND SRCS fs_automount.c)
  endif()

  if(CONFIG_FS_PROCFS AND NOT CONFIG_FS_PROCFS_EXCLUDE_MOUNT)
    list(APPEND SRCS fs_procfs_mount.c fs_gettype.c)
  endif()

  target_sources(fs PRIVATE ${SRCS})
endif()
