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
#include <termios.h>

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

#ifndef CONFIG_L86_M33_BAUD
#define CONFIG_L86_M33_BAUD 9600
#endif

#if CONFIG_L86_M33_BAUD == 4800
  #define L86_M33_BAUD_RATE 4800
#elif CONFIG_L86_M33_BAUD == 9600
    #define L86_M33_BAUD_RATE 9600
// #elif CONFIG_L86_M33_BAUD == 14400
//   #define L86_M33_BAUD_RATE 14400
#elif CONFIG_L86_M33_BAUD == 19200
  #define L86_M33_BAUD_RATE 19200
#elif CONFIG_L86_M33_BAUD == 38400
  #define L86_M33_BAUD_RATE 38400
#elif CONFIG_L86_M33_BAUD == 57600
  #define L86_M33_BAUD_RATE 57600
#elif CONFIG_L86_M33_BAUD == 115200
  #define L86_M33_BAUD_RATE 115200
#else
  #error "Invalid baud rate. Supported baud rates are: 4800, 5600, 14400, 19200, 38400, 57600, 115200"
#endif

#ifndef CONFIG_L86_M33_FIX_INT
#define CONFIG_L86_M33_FIX_INT 1000
#endif 

#ifdef CONFIG_L86_M33_FIX_INT
#define L86_M33_FIX_INT CONFIG_L86_M33_FIX_INT
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

static int l86m33_control(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, int cmd, unsigned long arg);
static int l86m33_activate(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep, bool enable);
static int l86m33_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                     FAR struct file *filep,
                                     FAR uint32_t *period_us);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_sensor_ops =
{
  // .control = l86m33_control,
  // .activate = l86m33_activate,
  .set_interval = l86m33_set_interval,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: calculate_checksum
 *
 * Description:
 *   Calculate checksum of PMTK command.
 *
 * Arguments:
 *    data      -  Char pointer to calculate checksum for
 *    len       -  Length of char string
 * 
 * Returns:
 *  1-byte checksum value to be interpreted as a hex byte
 ****************************************************************************/
 char calculate_checksum(char* data, int len){
  char ret = 0;
  for (int i = 0; i < len; ++i){
    ret = ret ^ *(data + i);
  }
  return ret;
}

/****************************************************************************
 * Name: send_command
 *
 * Description:
 *   Sends command L86-M33 GNSS device.
 *
 * Arguments:
 *    uart      -  Pointer to file struct of L86-M33 device
 *    cmd       -  L86M33_COMMAND enum
 *    arg       -  Dependent on command type. Could be used for preset
 *                 enum, numeric args or struct pointers
 ****************************************************************************/
bool send_command(FAR struct file *uart, L86M33_PMTK_COMMAND cmd, unsigned long arg){
  char buf[50];
  int bw1;
  switch (cmd)
  {
    case CMD_HOT_START:
    case CMD_WARM_START:
    case CMD_COLD_START:
    case CMD_FULL_COLD_START:
    {
      bw1 = snprintf(buf, 50, "$PMTK%d", cmd);
      break;
    }
    case CMD_STANDBY_MODE:
    {
      bw1 = snprintf(buf, 50, "$PMTK%d,%d", cmd, (int)arg);
      break;
    }
    case SET_NMEA_BAUDRATE:
    {
      bw1 = snprintf(buf, 50, "$PMTK%d,%d", cmd, (int)arg);
      break;
    }
    case SET_POS_FIX: 
    {
      bw1 = snprintf(buf, 50, "$PMTK%d,%d", cmd, (int)arg);
      break;
    }
    default:
      break;
  }
  char checksum = calculate_checksum(buf+1, bw1-1);
  int bw2 = snprintf(buf+bw1, 50-bw1, "*%02X\r\n", checksum);
  sninfo("About to send: %s size: %d\n", buf, bw1+bw2);
  int bw3 = file_write(uart, buf, bw1+bw2);
  sninfo("Bytes written: %d\n", bw3);
  return bw3;
}

char* read_line(FAR struct file *uart){
  int line_len = 0;
  char line[MINMEA_MAX_LENGTH];
  char next_char;
  do
  {
    file_read(uart, &next_char, 1);
    if (next_char != '\r' && next_char != '\n')
    {
      line[line_len++] = next_char;
    }
  } while (next_char != '\r' && next_char != '\n');
  line[line_len] = '\0';
  return line;
}

/****************************************************************************
 * Name: l86m33_set_interval
 *
 * Description:
 *   Send commands to the l86m33
 ****************************************************************************/
static int l86m33_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                     FAR struct file *filep,
                                     FAR uint32_t *period_us)
{
  if (period_us < 100 || period_us > 10000){
    // Invalid period
    return -1;
  }
  return 0;

}

 /****************************************************************************
 * Name: l86m33_control
 *
 * Description:
 *   Send commands to the l86m33
 ****************************************************************************/
static int l86m33_control(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, int cmd, unsigned long arg)
{
  return 0;
}

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
    } while (next_char != '\r' && next_char != '\n');
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
      case MINMEA_SENTENCE_GLL:
      {
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
        // snerr("Invalid NMEA sentence read %s, skipping line...\n", line);
        break;
      }
      case MINMEA_UNKNOWN:
      {
        // snerr("Unknown NMEA sentence read %s, skipping line...\n", line);
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

int l86m33_register(FAR const char *devpath, FAR const char *uartpath, int devno)
{
  FAR l86m33_dev_s *priv = NULL;
  struct termios opt;
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

  read_line(&priv->uart); // Wait until module is powered on

  #ifdef CONFIG_SERIAL_TERMIOS
  send_command(&priv->uart, SET_NMEA_BAUDRATE, L86_M33_BAUD_RATE);
  nxsig_usleep(20000);
  file_ioctl(&priv->uart, TCGETS, &opt);
  cfmakeraw(&opt);
  switch(L86_M33_BAUD_RATE){
    case 4800:
    {
      cfsetispeed(&opt, 4800);
      cfsetospeed(&opt, 4800);
      break;
    }
    case 9600:
    {
      cfsetispeed(&opt, 9600);
      cfsetospeed(&opt, 9600);
      break;
    }
    case 14400:
    {
      cfsetispeed(&opt, 14400);
      cfsetospeed(&opt, 14400);
      break;
    }
    case 19200:
    {
      cfsetispeed(&opt, 19200);
      cfsetospeed(&opt, 19200);
      break;
    }
    case 38400:
    {
      cfsetispeed(&opt, 38400);
      cfsetospeed(&opt, 38400);
      break;
    }
    case 57600:
    {
      cfsetispeed(&opt, 57600);
      cfsetospeed(&opt, 57600);
      break;
    }
    case 115200:
    {
      cfsetispeed(&opt, 115200);
      cfsetospeed(&opt, 115200);
      break;
    }
  }
  int res = file_ioctl(&priv->uart, TCSETS, &opt);
  // Wait for module to update
  for (int i = 0; i < 5; ++i){
    read_line(&priv->uart);
  }
  send_command(&priv->uart, SET_POS_FIX, L86_M33_FIX_INT);
  #endif

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
