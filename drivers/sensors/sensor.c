/****************************************************************************
 * drivers/sensors/sensor.c
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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <poll.h>
#include <fcntl.h>
#include <nuttx/list.h>
#include <nuttx/kmalloc.h>
#include <nuttx/circbuf.h>
#include <nuttx/mutex.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/lib/lib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device naming ************************************************************/

#define ROUND_DOWN(x, y)    (((x) / (y)) * (y))
#define DEVNAME_FMT         "/dev/uorb/sensor_%s%d"
#define TIMING_BUF_ESIZE    (sizeof(uint32_t))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sensor_axis_map_s
{
  int8_t src_x;
  int8_t src_y;
  int8_t src_z;

  int8_t sign_x;
  int8_t sign_y;
  int8_t sign_z;
};

/* This structure describes sensor meta */

struct sensor_meta_s
{
  size_t esize;
  FAR char *name;
};

typedef enum sensor_role_e
{
  SENSOR_ROLE_NONE,
  SENSOR_ROLE_WR,
  SENSOR_ROLE_RD,
  SENSOR_ROLE_RDWR,
} sensor_role_t;

/* This structure describes user info of sensor, the user may be
 * advertiser or subscriber
 */

struct sensor_user_s
{
  /* The common info */

  struct list_node node;       /* Node of users list */
  struct pollfd   *fds;        /* The poll structure of thread waiting events */
  sensor_role_t    role;       /* The is used to indicate user's role based on open flags */
  bool             changed;    /* This is used to indicate event happens and need to
                                * asynchronous notify other users
                                */
  unsigned int     event;      /* The event of this sensor, eg: SENSOR_EVENT_FLUSH_COMPLETE. */
  bool             flushing;   /* The is used to indicate user is flushing */
  sem_t            buffersem;  /* Wakeup user waiting for data in circular buffer */
  size_t           bufferpos;  /* The index of user generation in buffer */

  /* The subscriber info
   * Support multi advertisers to subscribe their own data when they
   * appear in dual role
   */

  struct sensor_ustate_s state;
};

/* This structure describes the state of the upper half driver */

struct sensor_upperhalf_s
{
  FAR struct sensor_lowerhalf_s *lower;  /* The handle of lower half driver */
  struct sensor_state_s          state;  /* The state of sensor device */
  struct circbuf_s   timing;             /* The circular buffer of generation */
  struct circbuf_s   buffer;             /* The circular buffer of data */
  rmutex_t           lock;               /* Manages exclusive access to file operations */
  struct list_node   userlist;           /* List of users */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    sensor_pollnotify(FAR struct sensor_upperhalf_s *upper,
                                 pollevent_t eventset, sensor_role_t role);
static int     sensor_open(FAR struct file *filep);
static int     sensor_close(FAR struct file *filep);
static ssize_t sensor_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t sensor_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int     sensor_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
static int     sensor_poll(FAR struct file *filep, FAR struct pollfd *fds,
                           bool setup);
static ssize_t sensor_push_event(FAR void *priv, FAR const void *data,
                                 size_t bytes);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_axis_map_s g_remap_tbl[] =
{
  { 0, 1, 2,  1,  1,  1 }, /* P0 */
  { 1, 0, 2,  1, -1,  1 }, /* P1 */
  { 0, 1, 2, -1, -1,  1 }, /* P2 */
  { 1, 0, 2, -1,  1,  1 }, /* P3 */
  { 0, 1, 2, -1,  1, -1 }, /* P4 */
  { 1, 0, 2, -1, -1, -1 }, /* P5 */
  { 0, 1, 2,  1, -1, -1 }, /* P6 */
  { 1, 0, 2,  1,  1, -1 }, /* P7 */
};

static const struct sensor_meta_s g_sensor_meta[] =
{
  {0,                                         NULL},
  {sizeof(struct sensor_accel),               "accel"},
  {sizeof(struct sensor_mag),                 "mag"},
  {sizeof(struct sensor_orientation),         "orientation"},
  {sizeof(struct sensor_gyro),                "gyro"},
  {sizeof(struct sensor_light),               "light"},
  {sizeof(struct sensor_baro),                "baro"},
  {sizeof(struct sensor_temp),                "temp"},
  {sizeof(struct sensor_prox),                "prox"},
  {sizeof(struct sensor_rgb),                 "rgb"},
  {sizeof(struct sensor_accel),               "linear_accel"},
  {sizeof(struct sensor_rotation),            "rotation"},
  {sizeof(struct sensor_humi),                "humi"},
  {sizeof(struct sensor_temp),                "ambient_temp"},
  {sizeof(struct sensor_mag),                 "mag_uncal"},
  {sizeof(struct sensor_pm1p0),               "pm1p0"},
  {sizeof(struct sensor_gyro),                "gyro_uncal"},
  {sizeof(struct sensor_event),               "motion_detect"},
  {sizeof(struct sensor_event),               "step_detector"},
  {sizeof(struct sensor_step_counter),        "step_counter"},
  {sizeof(struct sensor_ph),                  "ph"},
  {sizeof(struct sensor_hrate),               "hrate"},
  {sizeof(struct sensor_event),               "tilt_detector"},
  {sizeof(struct sensor_event),               "wake_gesture"},
  {sizeof(struct sensor_event),               "glance_gesture"},
  {sizeof(struct sensor_event),               "pickup_gesture"},
  {sizeof(struct sensor_event),               "wrist_tilt"},
  {sizeof(struct sensor_orientation),         "device_orientation"},
  {sizeof(struct sensor_pose_6dof),           "pose_6dof"},
  {sizeof(struct sensor_gas),                 "gas"},
  {sizeof(struct sensor_event),               "significant_motion"},
  {sizeof(struct sensor_hbeat),               "hbeat"},
  {sizeof(struct sensor_force),               "force"},
  {sizeof(struct sensor_hall),                "hall"},
  {sizeof(struct sensor_event),               "offbody_detector"},
  {sizeof(struct sensor_accel),               "accel_uncal"},
  {sizeof(struct sensor_angle),               "hinge_angle"},
  {sizeof(struct sensor_ir),                  "ir"},
  {sizeof(struct sensor_hcho),                "hcho"},
  {sizeof(struct sensor_tvoc),                "tvoc"},
  {sizeof(struct sensor_dust),                "dust"},
  {sizeof(struct sensor_ecg),                 "ecg"},
  {sizeof(struct sensor_ppgd),                "ppgd"},
  {sizeof(struct sensor_ppgq),                "ppgq"},
  {sizeof(struct sensor_impd),                "impd"},
  {sizeof(struct sensor_ots),                 "ots"},
  {sizeof(struct sensor_co2),                 "co2"},
  {sizeof(struct sensor_cap),                 "cap"},
  {sizeof(struct sensor_gnss),                "gnss"},
  {sizeof(struct sensor_gnss_satellite),      "gnss_satellite"},
  {sizeof(struct sensor_gnss_measurement),    "gnss_measurement"},
  {sizeof(struct sensor_gnss_clock),          "gnss_clock"},
  {sizeof(struct sensor_gnss_geofence_event), "gnss_geofence_event"},
  {sizeof(struct sensor_velocity),            "velocity"},
  {sizeof(struct sensor_noise),               "noise"},
  {sizeof(struct sensor_pm25),                "pm25"},
  {sizeof(struct sensor_pm10),                "pm10"},
  {sizeof(struct sensor_uv),                  "uv"},
};

static const struct file_operations g_sensor_fops =
{
  sensor_open,    /* open  */
  sensor_close,   /* close */
  sensor_read,    /* read  */
  sensor_write,   /* write */
  NULL,           /* seek  */
  sensor_ioctl,   /* ioctl */
  NULL,           /* mmap */
  NULL,           /* truncate */
  sensor_poll     /* poll  */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void sensor_lock(FAR void *priv)
{
  FAR struct sensor_upperhalf_s *upper = priv;
  nxrmutex_lock(&upper->lock);
}

static void sensor_unlock(FAR void *priv)
{
  FAR struct sensor_upperhalf_s *upper = priv;
  nxrmutex_unlock(&upper->lock);
}

static int sensor_update_interval(FAR struct file *filep,
                                  FAR struct sensor_upperhalf_s *upper,
                                  FAR struct sensor_user_s *user,
                                  uint32_t interval)
{
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR struct sensor_user_s *tmp;
  uint32_t min_interval = interval;
  uint32_t min_latency = interval != UINT32_MAX ?
                         user->state.latency : UINT32_MAX;
  int ret = 0;

  if (interval == user->state.interval)
    {
      return 0;
    }

  list_for_every_entry(&upper->userlist, tmp, struct sensor_user_s, node)
    {
      if (tmp == user || tmp->state.interval == UINT32_MAX)
        {
          continue;
        }

      if (min_interval > tmp->state.interval)
        {
          min_interval = tmp->state.interval;
        }

      if (min_latency > tmp->state.latency)
        {
          min_latency = tmp->state.latency;
        }
    }

  if (lower->ops->set_interval)
    {
      if (min_interval != UINT32_MAX &&
          min_interval != upper->state.min_interval)
        {
          uint32_t expected_interval = min_interval;
          ret = lower->ops->set_interval(lower, filep, &min_interval);
          if (ret < 0)
            {
              return ret;
            }
          else if (min_interval > expected_interval)
            {
              return -EINVAL;
            }
        }

      if (min_latency == UINT32_MAX)
        {
          min_latency = 0;
        }

      if (lower->ops->batch &&
          (min_latency != upper->state.min_latency ||
          (min_interval != upper->state.min_interval && min_latency)))
        {
          ret = lower->ops->batch(lower, filep, &min_latency);
          if (ret >= 0)
            {
              upper->state.min_latency = min_latency;
            }
        }
    }

  upper->state.min_interval = min_interval;
  user->state.interval = interval;
  sensor_pollnotify(upper, POLLPRI, SENSOR_ROLE_WR);
  return ret;
}

static int sensor_update_latency(FAR struct file *filep,
                                 FAR struct sensor_upperhalf_s *upper,
                                 FAR struct sensor_user_s *user,
                                 uint32_t latency)
{
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR struct sensor_user_s *tmp;
  uint32_t min_latency = latency;
  int ret = 0;

  if (latency == user->state.latency)
    {
      return 0;
    }

  if (user->state.interval == UINT32_MAX)
    {
      user->state.latency = latency;
      return 0;
    }

  if (latency <= upper->state.min_latency)
    {
      goto update;
    }

  list_for_every_entry(&upper->userlist, tmp, struct sensor_user_s, node)
    {
      if (tmp == user || tmp->state.interval == UINT32_MAX)
        {
          continue;
        }

      if (min_latency > tmp->state.latency)
        {
          min_latency = tmp->state.latency;
        }
    }

update:
  if (min_latency == UINT32_MAX)
    {
      min_latency = 0;
    }

  if (min_latency == upper->state.min_latency)
    {
      user->state.latency = latency;
      return ret;
    }

  if (lower->ops->batch)
    {
      ret = lower->ops->batch(lower, filep, &min_latency);
      if (ret < 0)
        {
          return ret;
        }
    }

  upper->state.min_latency = min_latency;
  user->state.latency = latency;
  sensor_pollnotify(upper, POLLPRI, SENSOR_ROLE_WR);
  return ret;
}

static void sensor_generate_timing(FAR struct sensor_upperhalf_s *upper,
                                   unsigned long nums)
{
  uint32_t interval = upper->state.min_interval != UINT32_MAX ?
                      upper->state.min_interval : 1;
  while (nums-- > 0)
    {
      upper->state.generation += interval;
      circbuf_overwrite(&upper->timing, &upper->state.generation,
                        TIMING_BUF_ESIZE);
    }
}

static bool sensor_is_updated(FAR struct sensor_upperhalf_s *upper,
                              FAR struct sensor_user_s *user)
{
  long delta = (long long)upper->state.generation - user->state.generation;

  if (delta <= 0)
    {
      return false;
    }
  else if (user->state.interval == UINT32_MAX)
    {
      return true;
    }
  else
    {
      /* Check whether next generation user want in buffer.
       * generation     next generation(not published yet)
       * ____v_____________v
       * ////|//////^      |
       *         ^ middle point
       *   next generation user want
       */

      return delta >= user->state.interval -
                      (upper->state.min_interval >> 1);
    }
}

static void sensor_catch_up(FAR struct sensor_upperhalf_s *upper,
                            FAR struct sensor_user_s *user)
{
  uint32_t generation;
  long delta;

  circbuf_peek(&upper->timing, &generation, TIMING_BUF_ESIZE);
  delta = (long long)generation - user->state.generation;
  if (delta > 0)
    {
      user->bufferpos = upper->timing.tail / TIMING_BUF_ESIZE;
      if (user->state.interval == UINT32_MAX)
        {
          user->state.generation = generation - 1;
        }
      else
        {
          delta -= upper->state.min_interval >> 1;
          user->state.generation += ROUND_DOWN(delta,
                                               user->state.interval);
        }
    }
}

static ssize_t sensor_do_samples(FAR struct sensor_upperhalf_s *upper,
                                 FAR struct sensor_user_s *user,
                                 FAR char *buffer, size_t len)
{
  uint32_t generation;
  ssize_t ret = 0;
  size_t nums;
  size_t pos;
  size_t end;

  sensor_catch_up(upper, user);
  nums = upper->timing.head / TIMING_BUF_ESIZE - user->bufferpos;
  if (len < nums * upper->state.esize)
    {
      nums = len / upper->state.esize;
    }

  len = nums * upper->state.esize;

  /* Take samples continuously */

  if (user->state.interval == UINT32_MAX)
    {
      if (buffer != NULL)
        {
          ret = circbuf_peekat(&upper->buffer,
                               user->bufferpos * upper->state.esize,
                               buffer, len);
        }
      else
        {
          ret = len;
        }

      user->bufferpos += nums;
      circbuf_peekat(&upper->timing,
                     (user->bufferpos - 1) * TIMING_BUF_ESIZE,
                     &user->state.generation, TIMING_BUF_ESIZE);
      return ret;
    }

  /* Take samples one-bye-one, to determine whether a sample needed:
   *
   * If user's next generation is on the left side of middle point,
   * we should copy this sample for user.
   *                      next_generation(or end)
   *                ________________v____
   * timing buffer: //|//////.      |
   *                  ^   middle
   *              generation
   *                        next sample(or end)
   *                ________________v____
   *  data  buffer:   |             |
   *                  ^
   *                sample
   */

  pos = user->bufferpos;
  end = upper->timing.head / TIMING_BUF_ESIZE;
  circbuf_peekat(&upper->timing, pos * TIMING_BUF_ESIZE,
                 &generation, TIMING_BUF_ESIZE);
  while (pos++ != end)
    {
      uint32_t next_generation;
      long delta;

      if (pos * TIMING_BUF_ESIZE == upper->timing.head)
        {
          next_generation = upper->state.generation +
                            upper->state.min_interval;
        }
      else
        {
          circbuf_peekat(&upper->timing, pos * TIMING_BUF_ESIZE,
                         &next_generation, TIMING_BUF_ESIZE);
        }

      delta = next_generation + generation -
              ((user->state.generation + user->state.interval) << 1);
      if (delta >= 0)
        {
          if (buffer != NULL)
            {
              ret += circbuf_peekat(&upper->buffer,
                                    (pos - 1) * upper->state.esize,
                                    buffer + ret, upper->state.esize);
            }
          else
            {
              ret += upper->state.esize;
            }

          user->bufferpos = pos;
          user->state.generation += user->state.interval;
          if (ret >= len)
            {
              break;
            }
        }

      generation = next_generation;
    }

  if (pos - 1 == end && sensor_is_updated(upper, user))
    {
      generation = upper->state.generation - user->state.generation +
                   (upper->state.min_interval >> 1);
      user->state.generation += ROUND_DOWN(generation,
                                           user->state.interval);
    }

  return ret;
}

static void sensor_pollnotify_one(FAR struct sensor_user_s *user,
                                  pollevent_t eventset,
                                  sensor_role_t role)
{
  if (!(user->role & role))
    {
      return;
    }

  if (eventset == POLLPRI)
    {
      user->changed = true;
    }

  poll_notify(&user->fds, 1, eventset);
}

static void sensor_pollnotify(FAR struct sensor_upperhalf_s *upper,
                              pollevent_t eventset, sensor_role_t role)
{
  FAR struct sensor_user_s *user;

  list_for_every_entry(&upper->userlist, user, struct sensor_user_s, node)
    {
      sensor_pollnotify_one(user, eventset, role);
    }
}

static int sensor_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR struct sensor_user_s *user;
  int ret = 0;

  nxrmutex_lock(&upper->lock);
  user = kmm_zalloc(sizeof(struct sensor_user_s));
  if (user == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_lock;
    }

  if (lower->ops->open)
    {
      ret = lower->ops->open(lower, filep);
      if (ret < 0)
        {
          goto errout_with_user;
        }
    }

  if ((filep->f_oflags & O_DIRECT) == 0)
    {
      if (filep->f_oflags & O_RDOK)
        {
          if (upper->state.nsubscribers == 0 && lower->ops->activate)
            {
              ret = lower->ops->activate(lower, filep, true);
              if (ret < 0)
                {
                  goto errout_with_open;
                }
            }

          user->role |= SENSOR_ROLE_RD;
          upper->state.nsubscribers++;
        }

      if (filep->f_oflags & O_WROK)
        {
          user->role |= SENSOR_ROLE_WR;
          upper->state.nadvertisers++;
          if (filep->f_oflags & SENSOR_PERSIST)
            {
              lower->persist = true;
            }
        }
    }

  if (upper->state.generation && lower->persist)
    {
      user->state.generation = upper->state.generation - 1;
      user->bufferpos = upper->timing.head / TIMING_BUF_ESIZE - 1;
    }
  else
    {
      user->state.generation = upper->state.generation;
      user->bufferpos = upper->timing.head / TIMING_BUF_ESIZE;
    }

  user->state.interval = UINT32_MAX;
  user->state.esize = upper->state.esize;
  nxsem_init(&user->buffersem, 0, 0);
  list_add_tail(&upper->userlist, &user->node);

  /* The new user generation, notify to other users */

  sensor_pollnotify(upper, POLLPRI, SENSOR_ROLE_WR);

  filep->f_priv = user;
  goto errout_with_lock;

errout_with_open:
  if (lower->ops->close)
    {
      lower->ops->close(lower, filep);
    }

errout_with_user:
  kmm_free(user);
errout_with_lock:
  nxrmutex_unlock(&upper->lock);
  return ret;
}

static int sensor_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR struct sensor_user_s *user = filep->f_priv;
  int ret = 0;

  nxrmutex_lock(&upper->lock);
  if (lower->ops->close)
    {
      ret = lower->ops->close(lower, filep);
      if (ret < 0)
        {
          nxrmutex_unlock(&upper->lock);
          return ret;
        }
    }

  if ((filep->f_oflags & O_DIRECT) == 0)
    {
      if (filep->f_oflags & O_RDOK)
        {
          upper->state.nsubscribers--;
          if (upper->state.nsubscribers == 0 && lower->ops->activate)
            {
              lower->ops->activate(lower, filep, false);
            }
        }

      if (filep->f_oflags & O_WROK)
        {
          upper->state.nadvertisers--;
        }
    }

  list_delete(&user->node);
  sensor_update_latency(filep, upper, user, UINT32_MAX);
  sensor_update_interval(filep, upper, user, UINT32_MAX);
  nxsem_destroy(&user->buffersem);

  /* The user is closed, notify to other users */

  sensor_pollnotify(upper, POLLPRI, SENSOR_ROLE_WR);
  nxrmutex_unlock(&upper->lock);

  kmm_free(user);
  return ret;
}

static ssize_t sensor_read(FAR struct file *filep, FAR char *buffer,
                           size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR struct sensor_user_s *user = filep->f_priv;
  ssize_t ret;

  if (!len)
    {
      return -EINVAL;
    }

  nxrmutex_lock(&upper->lock);
  if (lower->ops->fetch)
    {
      if (buffer == NULL)
        {
          return -EINVAL;
        }

      if (!(filep->f_oflags & O_NONBLOCK))
        {
          nxrmutex_unlock(&upper->lock);
          ret = nxsem_wait_uninterruptible(&user->buffersem);
          if (ret < 0)
            {
              return ret;
            }

          nxrmutex_lock(&upper->lock);
        }
      else if (!upper->state.nsubscribers)
        {
          ret = -EAGAIN;
          goto out;
        }

        ret = lower->ops->fetch(lower, filep, buffer, len);
    }
  else if (circbuf_is_empty(&upper->buffer))
    {
      ret = -ENODATA;
    }
  else if (sensor_is_updated(upper, user))
    {
      ret = sensor_do_samples(upper, user, buffer, len);
    }
  else if (lower->persist)
    {
      if (buffer == NULL)
        {
          ret = upper->state.esize;
        }
      else
        {
          /* Persistent device can get latest old data if not updated. */

          ret = circbuf_peekat(&upper->buffer,
                               (user->bufferpos - 1) * upper->state.esize,
                               buffer, upper->state.esize);
        }
    }
  else
    {
      ret = -ENODATA;
    }

out:
  nxrmutex_unlock(&upper->lock);
  return ret;
}

static ssize_t sensor_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;

  return lower->push_event(lower->priv, buffer, buflen);
}

static int sensor_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR struct sensor_user_s *user = filep->f_priv;
  uint32_t arg1 = (uint32_t)arg;
  int ret = 0;

  switch (cmd)
    {
      case SNIOC_GET_STATE:
        {
          nxrmutex_lock(&upper->lock);
          memcpy((FAR void *)(uintptr_t)arg,
                 &upper->state, sizeof(upper->state));
          user->changed = false;
          nxrmutex_unlock(&upper->lock);
        }
        break;

      case SNIOC_GET_USTATE:
        {
          nxrmutex_lock(&upper->lock);
          memcpy((FAR void *)(uintptr_t)arg,
                 &user->state, sizeof(user->state));
          nxrmutex_unlock(&upper->lock);
        }
        break;

      case SNIOC_SET_INTERVAL:
        {
          nxrmutex_lock(&upper->lock);
          ret = sensor_update_interval(filep, upper, user,
                                       arg1 ? arg1 : UINT32_MAX);
          nxrmutex_unlock(&upper->lock);
        }
        break;

      case SNIOC_BATCH:
        {
          nxrmutex_lock(&upper->lock);
          ret = sensor_update_latency(filep, upper, user, arg1);
          nxrmutex_unlock(&upper->lock);
        }
        break;

      case SNIOC_SELFTEST:
        {
          if (lower->ops->selftest == NULL)
            {
              ret = -ENOTSUP;
              break;
            }

          ret = lower->ops->selftest(lower, filep, arg);
        }
        break;

      case SNIOC_SET_CALIBVALUE:
        {
          if (lower->ops->set_calibvalue == NULL)
            {
              ret = -ENOTSUP;
              break;
            }

          ret = lower->ops->set_calibvalue(lower, filep, arg);
        }
        break;

      case SNIOC_CALIBRATE:
        {
          if (lower->ops->calibrate == NULL)
            {
              ret = -ENOTSUP;
              break;
            }

          ret = lower->ops->calibrate(lower, filep, arg);
        }
        break;

      case SNIOC_SET_USERPRIV:
        {
          nxrmutex_lock(&upper->lock);
          upper->state.priv = (uint64_t)arg;
          nxrmutex_unlock(&upper->lock);
        }
        break;

      case SNIOC_SET_BUFFER_NUMBER:
        {
          nxrmutex_lock(&upper->lock);
          if (!circbuf_is_init(&upper->buffer))
            {
              if (arg1 >= lower->nbuffer)
                {
                  lower->nbuffer = arg1;
                  upper->state.nbuffer = arg1;
                }
              else
                {
                  ret = -ERANGE;
                }
            }
          else
            {
              ret = -EBUSY;
            }

          nxrmutex_unlock(&upper->lock);
        }
        break;

      case SNIOC_UPDATED:
        {
          nxrmutex_lock(&upper->lock);
          *(FAR bool *)(uintptr_t)arg = sensor_is_updated(upper, user);
          nxrmutex_unlock(&upper->lock);
        }
        break;

      case SNIOC_GET_INFO:
        {
          if (lower->ops->get_info == NULL)
            {
              ret = -ENOTSUP;
              break;
            }

          ret = lower->ops->get_info(lower, filep,
                          (FAR struct sensor_device_info_s *)(uintptr_t)arg);
        }
        break;

     case SNIOC_GET_EVENTS:
        {
          nxrmutex_lock(&upper->lock);
          *(FAR unsigned int *)(uintptr_t)arg = user->event;
          user->event = 0;
          user->changed = false;
          nxrmutex_unlock(&upper->lock);
        }
        break;

     case SNIOC_FLUSH:
        {
          nxrmutex_lock(&upper->lock);

          /* If the sensor is not activated, return -EINVAL. */

          if (upper->state.nsubscribers == 0)
            {
              nxrmutex_unlock(&upper->lock);
              return -EINVAL;
            }

          if (lower->ops->flush != NULL)
            {
              /* Lower half driver will do flush in asynchronous mode,
               * flush will be completed until push event happened with
               * bytes is zero.
               */

              ret = lower->ops->flush(lower, filep);
              if (ret >= 0)
                {
                  user->flushing = true;
                }
            }
          else
            {
              /* If flush is not supported, complete immediately */

              user->event |= SENSOR_EVENT_FLUSH_COMPLETE;
              sensor_pollnotify_one(user, POLLPRI, user->role);
            }

          nxrmutex_unlock(&upper->lock);
        }
        break;

      default:

        /* Lowerhalf driver process other cmd. */

        if (lower->ops->control)
          {
            ret = lower->ops->control(lower, filep, cmd, arg);
          }
        else
          {
            ret = -ENOTTY;
          }

        break;
    }

  return ret;
}

static int sensor_poll(FAR struct file *filep,
                       FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR struct sensor_user_s *user = filep->f_priv;
  pollevent_t eventset = 0;
  int semcount;
  int ret = 0;

  nxrmutex_lock(&upper->lock);
  if (setup)
    {
      /* Don't have enough space to store fds */

      if (user->fds)
        {
          ret = -ENOSPC;
          goto errout;
        }

      user->fds = fds;
      fds->priv = filep;
      if (lower->ops->fetch)
        {
          /* Always return POLLIN for fetch data directly(non-block) */

          if (filep->f_oflags & O_NONBLOCK)
            {
              eventset |= POLLIN;
            }
          else
            {
              nxsem_get_value(&user->buffersem, &semcount);
              if (semcount > 0)
                {
                  eventset |= POLLIN;
                }
            }
        }
      else if (sensor_is_updated(upper, user))
        {
          eventset |= POLLIN;
        }

      if (user->changed)
        {
          eventset |= POLLPRI;
        }

        poll_notify(&fds, 1, eventset);
    }
  else
    {
      user->fds = NULL;
      fds->priv = NULL;
    }

errout:
  nxrmutex_unlock(&upper->lock);
  return ret;
}

static ssize_t sensor_push_event(FAR void *priv, FAR const void *data,
                                 size_t bytes)
{
  FAR struct sensor_upperhalf_s *upper = priv;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR struct sensor_user_s *user;
  unsigned long envcount;
  int semcount;
  int ret;

  nxrmutex_lock(&upper->lock);
  if (bytes == 0)
    {
      list_for_every_entry(&upper->userlist, user, struct sensor_user_s,
                           node)
        {
          if (user->flushing)
            {
              user->flushing = false;
              user->event |= SENSOR_EVENT_FLUSH_COMPLETE;
              sensor_pollnotify_one(user, POLLPRI, user->role);
            }
        }

      nxrmutex_unlock(&upper->lock);
      return 0;
    }

  envcount = bytes / upper->state.esize;
  if (bytes != envcount * upper->state.esize)
    {
      nxrmutex_unlock(&upper->lock);
      return -EINVAL;
    }

  if (!circbuf_is_init(&upper->buffer))
    {
      /* Initialize sensor buffer when data is first generated */

      ret = circbuf_init(&upper->buffer, NULL, lower->nbuffer *
                         upper->state.esize);
      if (ret < 0)
        {
          nxrmutex_unlock(&upper->lock);
          return ret;
        }

      ret = circbuf_init(&upper->timing, NULL, lower->nbuffer *
                         TIMING_BUF_ESIZE);
      if (ret < 0)
        {
          circbuf_uninit(&upper->buffer);
          nxrmutex_unlock(&upper->lock);
          return ret;
        }
    }

  circbuf_overwrite(&upper->buffer, data, bytes);
  sensor_generate_timing(upper, envcount);
  list_for_every_entry(&upper->userlist, user, struct sensor_user_s, node)
    {
      if (sensor_is_updated(upper, user))
        {
          nxsem_get_value(&user->buffersem, &semcount);
          if (semcount < 1)
            {
              nxsem_post(&user->buffersem);
            }

          sensor_pollnotify_one(user, POLLIN, SENSOR_ROLE_RD);
        }
    }

  nxrmutex_unlock(&upper->lock);
  return bytes;
}

static void sensor_notify_event(FAR void *priv)
{
  FAR struct sensor_upperhalf_s *upper = priv;
  FAR struct sensor_user_s *user;
  int semcount;

  nxrmutex_lock(&upper->lock);
  list_for_every_entry(&upper->userlist, user, struct sensor_user_s, node)
    {
      nxsem_get_value(&user->buffersem, &semcount);
      if (semcount < 1)
        {
          nxsem_post(&user->buffersem);
        }

      sensor_pollnotify_one(user, POLLIN, SENSOR_ROLE_RD);
    }

  nxrmutex_unlock(&upper->lock);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sensor_remap_vector_raw16
 *
 * Description:
 *   This function remap the sensor data according to the place position on
 *   board. The value of place is determined base on g_remap_tbl.
 *
 * Input Parameters:
 *   in    - A pointer to input data need remap.
 *   out   - A pointer to output data.
 *   place - The place position of sensor on board,
 *           ex:SENSOR_BODY_COORDINATE_PX
 *
 ****************************************************************************/

void sensor_remap_vector_raw16(FAR const int16_t *in, FAR int16_t *out,
                               int place)
{
  FAR const struct sensor_axis_map_s *remap;
  int16_t tmp[3];

  DEBUGASSERT(place < (sizeof(g_remap_tbl) / sizeof(g_remap_tbl[0])));

  remap = &g_remap_tbl[place];
  tmp[0] = in[remap->src_x] * remap->sign_x;
  tmp[1] = in[remap->src_y] * remap->sign_y;
  tmp[2] = in[remap->src_z] * remap->sign_z;
  memcpy(out, tmp, sizeof(tmp));
}

/****************************************************************************
 * Name: sensor_register
 *
 * Description:
 *   This function binds an instance of a "lower half" Sensor driver with the
 *   "upper half" Sensor device and registers that device so that can be used
 *   by application code.
 *
 *   We will register the chararter device by node name format based on the
 *   type of sensor. Multiple types of the same type are distinguished by
 *   numbers. eg: accel0, accel1
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half sensor driver. This
 *           instance is bound to the sensor driver and must persists as long
 *           as the driver persists.
 *   devno - The user specifies which device of this type, from 0. If the
 *           devno already exists, -EEXIST will be returned.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int sensor_register(FAR struct sensor_lowerhalf_s *lower, int devno)
{
  FAR char *path;
  int ret;

  DEBUGASSERT(lower != NULL);

  path = lib_get_pathbuffer();
  if (path == NULL)
    {
      return -ENOMEM;
    }

  snprintf(path, PATH_MAX, DEVNAME_FMT,
           g_sensor_meta[lower->type].name,
           devno);
  ret = sensor_custom_register(lower, path,
                               g_sensor_meta[lower->type].esize);
  lib_put_pathbuffer(path);
  return ret;
}

/****************************************************************************
 * Name: sensor_custom_register
 *
 * Description:
 *   This function binds an instance of a "lower half" Sensor driver with the
 *   "upper half" Sensor device and registers that device so that can be used
 *   by application code.
 *
 *   You can register the character device type by specific path and esize.
 *   This API corresponds to the sensor_custom_unregister.
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half sensor driver. This
 *           instance is bound to the sensor driver and must persists as long
 *           as the driver persists.
 *   path  - The user specifies path of device. ex: /dev/uorb/xxx.
 *   esize - The element size of intermediate circular buffer.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int sensor_custom_register(FAR struct sensor_lowerhalf_s *lower,
                           FAR const char *path, size_t esize)
{
  FAR struct sensor_upperhalf_s *upper;
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL);

  if (lower->type >= SENSOR_TYPE_COUNT || !esize)
    {
      snerr("ERROR: type is invalid\n");
      return ret;
    }

  /* Allocate the upper-half data structure */

  upper = kmm_zalloc(sizeof(struct sensor_upperhalf_s));
  if (!upper)
    {
      snerr("ERROR: Allocation failed\n");
      return -ENOMEM;
    }

  /* Initialize the upper-half data structure */

  list_initialize(&upper->userlist);
  upper->state.esize = esize;
  upper->state.min_interval = UINT32_MAX;
  if (lower->ops->activate)
    {
      upper->state.nadvertisers = 1;
    }

  nxrmutex_init(&upper->lock);

  /* Bind the lower half data structure member */

  lower->priv          = upper;
  lower->sensor_lock   = sensor_lock;
  lower->sensor_unlock = sensor_unlock;

  if (!lower->ops->fetch)
    {
      if (!lower->nbuffer)
        {
          lower->nbuffer = 1;
        }

      lower->push_event = sensor_push_event;
    }
  else
    {
      lower->notify_event = sensor_notify_event;
      lower->nbuffer = 0;
    }

#ifdef CONFIG_SENSORS_RPMSG
  lower = sensor_rpmsg_register(lower, path);
  if (lower == NULL)
    {
      ret = -EIO;
      goto rpmsg_err;
    }
#endif

  upper->state.nbuffer = lower->nbuffer;
  upper->lower = lower;
  sninfo("Registering %s\n", path);
  ret = register_driver(path, &g_sensor_fops, 0666, upper);
  if (ret)
    {
      goto drv_err;
    }

  return ret;

drv_err:
#ifdef CONFIG_SENSORS_RPMSG
  sensor_rpmsg_unregister(lower);
rpmsg_err:
#endif

  nxrmutex_destroy(&upper->lock);

  kmm_free(upper);

  return ret;
}

/****************************************************************************
 * Name: sensor_unregister
 *
 * Description:
 *   This function unregister character node and release all resource about
 *   upper half driver.
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half sensor driver. This
 *           instance is bound to the sensor driver and must persists as long
 *           as the driver persists.
 *   devno - The user specifies which device of this type, from 0.
 *
 ****************************************************************************/

void sensor_unregister(FAR struct sensor_lowerhalf_s *lower, int devno)
{
  FAR char *path;

  path = lib_get_pathbuffer();
  if (path == NULL)
    {
      return;
    }

  snprintf(path, PATH_MAX, DEVNAME_FMT,
           g_sensor_meta[lower->type].name,
           devno);
  sensor_custom_unregister(lower, path);
  lib_put_pathbuffer(path);
}

/****************************************************************************
 * Name: sensor_custom_unregister
 *
 * Description:
 *   This function unregister character node and release all resource about
 *   upper half driver. This API corresponds to the sensor_custom_register.
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half sensor driver. This
 *           instance is bound to the sensor driver and must persists as long
 *           as the driver persists.
 *   path  - The user specifies path of device, ex: /dev/uorb/xxx
 *
 ****************************************************************************/

void sensor_custom_unregister(FAR struct sensor_lowerhalf_s *lower,
                              FAR const char *path)
{
  FAR struct sensor_upperhalf_s *upper;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(lower->priv != NULL);

  upper = lower->priv;

  sninfo("UnRegistering %s\n", path);
  unregister_driver(path);

#ifdef CONFIG_SENSORS_RPMSG
  sensor_rpmsg_unregister(lower);
#endif

  nxrmutex_destroy(&upper->lock);
  if (circbuf_is_init(&upper->buffer))
    {
      circbuf_uninit(&upper->buffer);
      circbuf_uninit(&upper->timing);
    }

  kmm_free(upper);
}
