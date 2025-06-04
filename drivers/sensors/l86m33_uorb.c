/****************************************************************************
 * drivers/sensors/l86m33_uorb.c
 * 
 * NOTE: EXPERIMENTAL DRIVER
 *
 * Contributed by Carleton University InSpace
 * 
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
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
#include <debug.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>
#include <nuttx/sensors/sensor.h>
#include <minmea/minmea.h>

#include <nuttx/sensors/l86m33.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SENSORS_L86_M33_THREAD_STACKSIZE
#define CONFIG_SENSORS_L86_M33_THREAD_STACKSIZE 10000
#endif

/* Helper to get array length */

#define MINMEA_MAX_LENGTH    256
#define array_len(arr) ((sizeof(arr)) / sizeof((arr)[0]))

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

/* GNSS device struct */

typedef struct
{
  FAR struct file uart;     /* UART interface */
  struct sensor_lowerhalf_s lower; /* UORB lower-half */
  mutex_t devlock;          /* Exclusive access */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  int16_t crefs; /* Number of open references */
#endif
} l86m33_dev_s;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_sensor_ops =
{
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

 /****************************************************************************
 * Name: l86m33_thread
 *
 * Description:
 *   Kernel thread to poll the l86m33
 ****************************************************************************/

static int l86m33_thread(int argc, FAR char *argv[]){
  FAR l86m33_dev_s *dev =
      (FAR l86m33_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  struct sensor_gnss gps;
  memset(&gps, 0, sizeof(gps));
  
  /* Read full line of NMEA output */
  for(;;){
    int line_len = 0;
    char line[MINMEA_MAX_LENGTH];
    char next_char;
    do
    {
        file_read(&dev->uart, &next_char, 1);
        if (next_char != '\r' && next_char != '\n')
        {
          line[line_len++] = next_char;
          }
        }
        while (next_char != '\r' && next_char != '\n');
        line[line_len] = '\0';
        
    /* Parse line based on NMEA sentence type */
    switch (minmea_sentence_id(line, false))
    {
      /* Time data is obtained from RMC sentence */
      case MINMEA_SENTENCE_RMC:
      {
        struct minmea_sentence_rmc frame;
              struct tm tm;
              if (minmea_check(line, false) && minmea_parse_rmc(&frame, line)){
                gps.timestamp = sensor_get_timestamp();
                minmea_getdatetime(&tm, &frame.date, &frame.time);
                gps.time_utc = mktime(&tm);
              }
              break;
            }
            /* Velocity data is obtained from VTG sentence*/
          case MINMEA_SENTENCE_VTG:
          {
            struct minmea_sentence_vtg frame;
            
            if (minmea_parse_vtg(&frame, line)){
              gps.ground_speed = minmea_tofloat(&frame.speed_kph) * 3.6; /* Convert speed in kph to mps*/
              gps.course = minmea_tofloat(&frame.true_track_degrees);
            }
            break;
          }
          /* 3D positional data is obtained from GGA sentence */
          case MINMEA_SENTENCE_GGA:
          {
            struct minmea_sentence_gga frame;
            
            if (minmea_parse_gga(&frame, line)){
              gps.latitude = minmea_tocoord(&frame.latitude); 
              gps.longitude = minmea_tocoord(&frame.longitude);
              gps.altitude = minmea_tofloat(&frame.altitude);
              gps.altitude_ellipsoid = minmea_tofloat(&frame.height);
            }
            break;
          }
          /* Precision dilution and sattelite data is obtained from GSA sentence */
          case MINMEA_SENTENCE_GSA:
          {
            struct minmea_sentence_gsa frame;
            
            if (minmea_parse_gsa(&frame, line)){
              gps.hdop = minmea_tofloat(&frame.hdop);
              gps.pdop = minmea_tofloat(&frame.pdop);
              gps.vdop = minmea_tofloat(&frame.vdop);
              uint32_t sats = 0;
              for (int i = 0; i < 12; ++i){
                if (frame.sats[i] != 0){
                  ++sats;
                }
              }
              gps.satellites_used = sats;
            }
            break;
          }
          /* GSV and GLL data are transmitted by the l86-m33 but do not provide
          additional information. Since GLL is always the last message transmitted,
          events will be pushed whenever that sentence is read */
          case MINMEA_SENTENCE_GLL:{
              dev->lower.push_event(dev->lower.priv, &gps, sizeof(gps));
            }
            /* All remaining sentences are not transmitted by the module */
            case MINMEA_SENTENCE_GSV:
            case MINMEA_SENTENCE_GBS:
            case MINMEA_SENTENCE_GST:
            case MINMEA_SENTENCE_ZDA:
          {
            break;
          }
          case MINMEA_INVALID:
          {
            wlerr("Invalid NMEA sentence read, skipping line...\n");
            break;
          }
          case MINMEA_UNKNOWN:
          {
            wlerr("Unknown NMEA sentence read, skipping line...\n");
            break;
          }
        }
  }
  
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: l86m33_register
 *
 * Description:
 *   Register the L86-M33 GNSS device driver.
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

int l86m33_register(FAR const char *devpath, FAR const char *uartpath, int devno, SUPPORTED_BAUD_RATES br, SUPPORTED_UPDATE_RATES ur)
{
  FAR l86m33_dev_s *priv = NULL;
  int err = 0;
  // int retries = 0;

  DEBUGASSERT(uartpath != NULL);
  DEBUGASSERT(devpath != NULL);

  /* Initialize device structure */

  priv = kmm_zalloc(sizeof(l86m33_dev_s));
  if (priv == NULL)
    {
      wlerr("Failed to allocate instance of L86-M33 driver.\n");
      return -ENOMEM;
    }

  memset(priv, 0, sizeof(l86m33_dev_s));

  /* Initialize mutex */

  err = nxmutex_init(&priv->devlock);
  if (err < 0)
    {
      wlerr("Failed to initialize mutex for L86-M33 device: %d\n", err);
      goto free_mem;
    }

  /* Open UART interface for use */

  err = file_open(&priv->uart, uartpath, O_RDWR | O_CLOEXEC);
  if (err < 0)
    {
      wlerr("Failed to open UART interface %s for L86-M33 driver: %d\n",
            uartpath, err);
      goto destroy_mutex;
    }

  /* Setup sensor with configured settings */
  

  /* Register UORB Sensor */
  // TODO: Write sensor functions and properly register sensor
  priv->lower.ops = &g_sensor_ops;
  priv->lower.type = SENSOR_TYPE_GNSS;

  err = sensor_register(&priv->lower, devno);
  if (err < 0)
  {
    snerr("Failed to register L86-M33 driver: %d\n", err);
    goto close_file;
  }

  
  FAR char *argv[2];
  char arg1[32];
  snprintf(arg1, 16, "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;
  
  err = kthread_create("l86m33_thread", SCHED_PRIORITY_DEFAULT,
                      CONFIG_SENSORS_L86_M33_THREAD_STACKSIZE,
                      l86m33_thread, argv);

  if (err < 0)
    {
      snerr("Failed to create the l86m33 notification kthread\n");
      goto sensor_unreg;
    }

    sninfo("Registered L86-M33 driver with kernel polling thread with baud rate %d and update rate %d", L86_M33_BAUD_RATE, L86_M33_FIX_INT);
    
  /* Cleanup items on error */
  
  if (err < 0)
    {
    sensor_unreg:
      sensor_unregister(&priv->lower, devno);
    close_file:
      file_close(&priv->uart);
    destroy_mutex:
      nxmutex_destroy(&priv->devlock);
    free_mem:
      kmm_free(priv);
    }

  return err;
}
