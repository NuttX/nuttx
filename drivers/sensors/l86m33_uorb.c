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
#include <nuttx/nuttx.h>
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
#elif CONFIG_L86_M33_BAUD == 14400
  #define L86_M33_BAUD_RATE 14400
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
  FAR struct file uart;               /* UART interface */
  struct sensor_lowerhalf_s lower;    /* UORB lower-half */
  mutex_t devlock;                    /* Exclusive access */
  sem_t run;                          /* Start/stop collection thread */
  bool enabled;                       /* If module has started */
  char buffer[MINMEA_MAX_LENGTH]; /* Buffer for UART interface */
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
char calculate_checksum(char* data, int len);
int send_command(l86m33_dev_s *dev, L86M33_PMTK_COMMAND cmd, unsigned long arg);
void read_line(l86m33_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_sensor_ops =
{
  .control = l86m33_control,
  .activate = l86m33_activate,
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
 *   Sends command L86-M33 GNSS device and waits for acknowledgement 
 *   if command supports it.
 *
 * Arguments:
 *    dev       -  Pointer L86-M33 priv struct
 *    cmd       -  L86M33_COMMAND enum
 *    arg       -  Dependent on command type. Could be used for preset
 *                 enum, numeric args or struct pointers
 * 
 * Returns:
 *  Flag defined by device
 *  negative number - Command failed during writing
 *  0 - Invalid packet
 *  1 - Unsupported packet type
 *  2 - Valid packet, but action failed
 *  3 - Valid packet, action succeeded 
 ****************************************************************************/
int send_command(l86m33_dev_s *dev, L86M33_PMTK_COMMAND cmd, unsigned long arg){
  char buf[50];
  int bw1;
  nxmutex_lock(&dev->devlock);
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
    case FR_MODE:
    {
      bw1 = snprintf(buf, 50, "$PMTK%d,%d", cmd, (int)arg);
    }
    default:
      break;
  }
  
  char checksum = calculate_checksum(buf+1, bw1-1);
  int bw2 = snprintf(buf+bw1, 50-bw1, "*%02X\r\n", checksum);
  sninfo("Sending command: %s\n", buf);
  int err = file_write(&dev->uart, buf, bw1+bw2);
  if (err < 0)
  {
    snerr("Could not send command to device\n");
    return err;
  }

  // These commands do not send ACKs so just return after they've been written
  if (cmd == CMD_HOT_START || cmd == CMD_WARM_START || cmd == CMD_COLD_START ||
      cmd == CMD_FULL_COLD_START || cmd == SET_NMEA_BAUDRATE)
  {
        
    nxmutex_unlock(&dev->devlock);
    return 3;
  }

  // Some commands will send ACKs, wait for them here before unlocking the mutex
  memset(buf, '\0', 50);
  snprintf(buf, 50, "$PMTK001,%d", cmd); // ACK message will be $PMTK001,<cmd num>,<flag>
  sninfo("Waiting for ACK...\n");
  for(;;)
  {
    read_line(dev);
    if (strncmp(buf, dev->buffer, strlen(buf)) == 0) break;
  }
  sninfo("ACK received!\n");
  nxmutex_unlock(&dev->devlock);
  return dev->buffer[13] - '0';
}

void read_line(l86m33_dev_s *dev){
  memset(dev->buffer, '\0', MINMEA_MAX_LENGTH);
  int line_len = 0;
  char next_char;
  do
  {
    file_read(&dev->uart, &next_char, 1);
    if (next_char != '\r' && next_char != '\n')
    {
      dev->buffer[line_len++] = next_char;
    }
  } while (next_char != '\r' && next_char != '\n' && line_len < MINMEA_MAX_LENGTH);
  dev->buffer[line_len] = '\0';
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
  FAR l86m33_dev_s *dev = container_of(lower, FAR l86m33_dev_s, lower);
  return send_command(dev, (L86M33_PMTK_COMMAND)cmd, arg);
}

/****************************************************************************
 * Name: nau7802_activate
 ****************************************************************************/

static int l86m33_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  FAR l86m33_dev_s *dev = container_of(lower, FAR l86m33_dev_s, lower);
  
  /* If not already enabled, start gps*/
  if (enable && !dev->enabled)
  {
    nxsem_post(&dev->run);
    dev->enabled = true;
    send_command(dev, CMD_HOT_START, (int)NULL);
  } 
  /* If not already disabled, send gps into standby mode*/
  else if (!enable && dev->enabled) 
  {
    dev->enabled = false;
    send_command(dev, CMD_STANDBY_MODE, 0);
  }
  return 0;
}


/****************************************************************************
 * Name: l86m33_set_interval
 *
 * Description:
 *   Set position fix interval of L86-M33 GNSS module
 * 
 * Returns:
 *   -1 if invalid interval, else return value from send_command
 ****************************************************************************/
static int l86m33_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                     FAR struct file *filep,
                                     FAR uint32_t *period_us)
{
  FAR l86m33_dev_s *dev = container_of(lower, FAR l86m33_dev_s, lower);
  int fix_interval = *period_us;
  if (fix_interval < 100 || fix_interval > 10000){
    // Invalid period
    return -1;
  }
  int ret = send_command(dev, SET_POS_FIX, fix_interval);
  return ret;
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
  dev->enabled = true;
  int err;
  
  /* Read full line of NMEA output */
  for(;;){
    /* If the sensor is disabled wait until enabled with activate function */
    if (!dev->enabled)
    {
      err = nxsem_wait(&dev->run);
      if (err < 0)
      {
        continue;
      }
    }

    // Mutex required because some commands send ACKS
    nxmutex_lock(&dev->devlock);
    read_line(dev);
        
    /* Parse line based on NMEA sentence type */
    switch (minmea_sentence_id(dev->buffer, false))
    {
      /* Time data is obtained from RMC sentence */
      case MINMEA_SENTENCE_RMC:
      {
        struct minmea_sentence_rmc frame;
        struct tm tm;
        if (minmea_check(dev->buffer, false) && minmea_parse_rmc(&frame, dev->buffer)){
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
        
        if (minmea_parse_vtg(&frame, dev->buffer)){
          gps.ground_speed = minmea_tofloat(&frame.speed_kph) * 3.6; /* Convert speed in kph to mps*/
          gps.course = minmea_tofloat(&frame.true_track_degrees);
        }
        break;
      }

      /* 3D positional data is obtained from GGA sentence */
      case MINMEA_SENTENCE_GGA:
      {
        struct minmea_sentence_gga frame;
        
        if (minmea_parse_gga(&frame, dev->buffer)){
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
        
        if (minmea_parse_gsa(&frame, dev->buffer)){
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
        sninfo("Invalid NMEA sentence read %s, skipping line...\n", dev->buffer);
        break;
      }
      case MINMEA_UNKNOWN:
      {
        sninfo("Unknown NMEA sentence read %s, skipping line...\n", dev->buffer);
        break;
      }
    }
    nxmutex_unlock(&dev->devlock);
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
 ****************************************************************************/

int l86m33_register(FAR const char *devpath, FAR const char *uartpath, int devno)
{
  FAR l86m33_dev_s *priv = NULL;
  struct termios opt;
  int err;
  // int retries = 0;
  
  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(uartpath != NULL);
  
  /* Initialize device structure */
  
  priv = kmm_zalloc(sizeof(l86m33_dev_s));
  if (priv == NULL)
  {
      snerr("Failed to allocate instance of L86-M33 driver.\n");
      return -ENOMEM;
    }

  memset(priv, 0, sizeof(l86m33_dev_s));

  /* Initialize mutex */

  err = nxmutex_init(&priv->devlock);
  if (err < 0)
    {
      snerr("Failed to initialize mutex for L86-M33 device: %d\n", err);
      goto free_mem;
    }

  /* Initialize semaphore */

  err = nxsem_init(&priv->run, 0, 0);
  if (err < 0)
    {
      snerr("Failed to register nau7802 driver: %d\n", err);
      goto destroy_mutex;
    }

  /* Open UART interface for use */

  err = file_open(&priv->uart, uartpath, O_RDWR | O_CLOEXEC);
  if (err < 0)
    {
      wlerr("Failed to open UART interface %s for L86-M33 driver: %d\n",
            uartpath, err);
      goto destroy_sem;
    }

  /* Setup sensor with configured settings */

  read_line(priv); // Wait until module is powered on

  #ifdef CONFIG_SERIAL_TERMIOS
  err = send_command(priv, SET_NMEA_BAUDRATE, L86_M33_BAUD_RATE);
  if (err != 3)
    {
      snwarn("Couldn't set baud rate of device: %d\n", err);
    }
  
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

  err = file_ioctl(&priv->uart, TCSETS, &opt);
  if (err < 0)
    {
      snwarn("Couldn't change baud rate of U(S)ART interface: %d\n", err); 
    }

  // Wait for module to update
  for (int i = 0; i < 5; ++i){
    read_line(priv);
  }

  err = send_command(priv, SET_POS_FIX, L86_M33_FIX_INT);
  if (err != 3)
    {
      snwarn("Couldn't set position fix interval, %d\n", err);
    }
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
    destroy_sem:
      nxsem_destroy(&priv->run);
    destroy_mutex:
      nxmutex_destroy(&priv->devlock);
    free_mem:
      kmm_free(priv);
    }

  return err;
}
