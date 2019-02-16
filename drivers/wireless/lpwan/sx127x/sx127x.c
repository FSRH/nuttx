/****************************************************************************
 * drivers/wireless/sx127x.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <semaphore.h>
#include <poll.h>
#include <debug.h>
#include <time.h>
#include <fcntl.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>

#include <nuttx/wireless/lpwan/sx127x.h>
#include "sx127x.h"

/* This driver is WIP and depends on EXPERIMENTA flag mainly because
 * the communication betweew modules (RX+TX) hasn't been fully tested.
 *
 * TODO:
 *   - Channel Activity Detection (CAD) for LORA
 *   - frequency hopping for LORA and FSK/OOK
 *   - modulation shaping for FSK/OOK
 *   - support for long payload for FSK/OOK (len > FIFO size)
 *   - transmitter/receiver configuration
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_SCHED_HPWORK)
#  error SX127X requires CONFIG_SCHED_HPWORK
#endif

/* Configuration ************************************************************/

/* Default SPI bus frequency (in Hz) up to 10MHz */

#define SX127X_SPIFREQ                (1000000)
#define SX127X_MODULATION_DEFAULT     SX127X_MODULATION_FSK

/* Device name */

#define SX127X_DEV_NAME               "/dev/sx127x"

/* Default modulation */

#define SX127X_MODULATION_DEFAULT     SX127X_MODULATION_FSK

/* Payload fixlen default */

#define SX127X_RX_FIXLEN_DEFAULT      (0xff)

/* Calibration frequency (TODO: Kconfig) */

#define SX127X_FREQ_CALIBRATION       (868000000)

/* FSK/OOK bandwidth default */

#define SX127X_FSKOOK_RXBW_DEFAULT    FSKOOK_BANDWIDTH_2p6kHz
#define SX127X_FSKOOK_AFCBW_DEFAULT   FSKOOK_BANDWIDTH_2p6kHz

/* Default LORA bandwidth */

#define SX127X_LRM_BW_DEFAULT         LORA_BANDWIDTH_7p8kHz

/* Default SF for LORA */

#define SX127X_LRM_SF_DEFAULT         (7)

/* Disable implict header for LORA at default */

#define SX127X_LRM_IMPLICTHDR_DEFAULT (false)

/* FSK/OOK RX/TX FIFO size (two separate FIFOs) */

#define SX127X_FOM_FIFO_LEN           (64)

/* LORA RX/TX FIFO size (one FIFO) */

#define SX127X_LRM_FIFO_LEN           (256)

/* FSK default frequency deviation is 5 kHz */

#define SX127X_FREQ_DEV_DEFAULT       (5000)

/* Default preamble length for LORA and FSK/OOK */

#define SX127X_PREAMBLE_LEN_DEFAULT   (8)

/* LORA maximum payload length */

#define SX127X_LRM_PAYLOADMAX_DEFAULT (0xff)

/* FSK/OOK default shaping configuration */

#define SX127X_FSKOOK_SHAPING_DEFAULT SX127X_CMN_PARAMP_SHAPING_NONE

/* FSK/OOK default PARAMP configuration */

#define SX127X_FSKOOK_PARAMP_DEFAULT  SX127X_CMN_PARAMP_PARAMP_40us

/* Default code rate for LORA */

#define SX127X_LRM_CR_DEFAULT         LORA_CR_4d5

/* Default IDLE mode */

#define SX127X_IDLE_OPMODE            SX127X_OPMODE_STANDBY

/* Total size for local RX FIFO */

#define SX127X_RXFIFO_TOTAL_SIZE      (SX127X_RXFIFO_ITEM_SIZE*CONFIG_LPWAN_SX127X_RXFIFO_LEN)

/* Some assertions */

#if CONFIG_LPWAN_SX127X_RXFIFO_DATA_LEN > SX127X_FOM_FIFO_LEN
#  warning RX data length limited by chip RX FIFO size (FSK/OOK = 64, LORA = 256)
#endif

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

/* SPI access mode */

typedef enum
{
  MODE_READ,
  MODE_WRITE
} sx127x_access_mode_t;

/* SX127X modulation specific ops */

struct sx127x_dev_s;
struct sx127x_priv_ops_s
{
  /* Initialize configuration for modulation */

  CODE void (*init)(FAR struct sx127x_dev_s *dev);

  /* Process IRQ 0 */

  CODE int (*isr0_process)(FAR struct sx127x_dev_s *dev);

  /* Operation mode initialization */

  CODE int (*opmode_init)(FAR struct sx127x_dev_s *dev, uint8_t opmode);

  /* Change operation mode */

  CODE int (*opmode_set)(FAR struct sx127x_dev_s *dev, uint8_t opmode);

  /* Set preamble length */

  CODE void (*preamble_set)(FAR struct sx127x_dev_s *dev, uint32_t len);

  /* Get preamble length */

  CODE int (*preamble_get)(FAR struct sx127x_dev_s *dev);

  /* Get current RSSI */

  CODE int16_t (*rssi_get)(FAR struct sx127x_dev_s *dev);

  /* Set sync word */

  CODE int (*syncword_set)(FAR struct sx127x_dev_s *dev, FAR uint8_t *sw,
                           uint8_t len);

  /* Get sync word */

  CODE void (*syncword_get)(FAR struct sx127x_dev_s *dev, FAR uint8_t *sw,
                            FAR uint8_t *len);

#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
  /* Send packet */

  CODE int (*send)(FAR struct sx127x_dev_s *dev, FAR const uint8_t *data,
              size_t datalen);
#endif
#ifdef CONFIG_DEBUG_WIRELESS_INFO
  /* Dump registers for given modulation */

  CODE void (*dumpregs)(FAR struct sx127x_dev_s *dev);
#endif
};

#ifdef CONFIG_LPWAN_SX127X_FSKOOK

/* FSK/OOK private data */

struct sx127x_fskook_s
{
  uint32_t bitrate;             /* Bitrate */
  uint32_t fdev;                /* Frequency deviation */
  uint8_t  rx_bw;               /* RX bandwidth */
  uint8_t  afc_bw;              /* AFC bandwidth */
  bool     fixlen;              /* Fix length */
  bool     seqon;               /* Sequencer enabled */
};
#endif

#ifdef CONFIG_LPWAN_SX127X_LORA
/* LORA private data */

struct sx127x_lora_s
{
  uint32_t freqhop;             /* Frequency hopping (not supported) */
  uint8_t  bw;                  /* LORA banwidth */
  uint8_t  sf;                  /* Spreading factor */
  uint8_t  cr;                  /* Coding rate */
  bool     implicthdr;          /* Implict header mode ON */
  bool     invert_iq;           /* Invert I and Q signals */
};
#endif

/* SX127X private data */

struct sx127x_dev_s
{
  /* Reference to SPI bus device */

  FAR struct spi_dev_s *spi;

  /* Low-level MCU-specific support */

  FAR const struct sx127x_lower_s *lower;

  /* Operations specific for selected modulation scheme */

  struct sx127x_priv_ops_s ops;
  struct work_s irq0_work;        /* Interrupt DIO0 handling "bottom half" */

  uint32_t freq;                  /* RF carrier frequency */
  uint8_t  modulation;            /* Current modulation (LORA/FSK/OOK) */
  uint8_t  opmode;                /* Current operation mode */
  uint8_t  idle;                  /* IDLE opmode */
  bool     crcon;                 /* TX/RX CRC enable */
  bool     rx_cont;               /* RX in continuous mode (not supported) */
  bool     tx_cont;               /* TX in continuous mode (not supported) */

#ifdef CONFIG_LPWAN_SX127X_FSKOOK
  struct sx127x_fskook_s fskook;  /* FSK/OOK modulation specific data */
#endif
#ifdef CONFIG_LPWAN_SX127X_LORA
  struct sx127x_lora_s   lora;    /* LORA modulation specific data */
#endif

#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
  sem_t    tx_sem;                /* Wait for availability of send data */
  uint32_t tx_timeout;            /* TX timeout (not supported) */
#endif
#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
  uint32_t rx_timeout;            /* RX timeout (not supported) */
  uint16_t rx_fifo_len;           /* Number of bytes stored in fifo */
  uint16_t nxt_read;              /* Next read index */
  uint16_t nxt_write;             /* Next write index */

  /* Circular RX packet buffer */

  uint8_t  rx_buffer[SX127X_RXFIFO_TOTAL_SIZE];
  sem_t    rx_sem;                /* Wait for availability of received data */
  sem_t    rx_buffer_sem;         /* Protect access to rx fifo */
#endif

  uint8_t nopens;                 /* Number of times the device has been opened */
  sem_t   dev_sem;                /* Ensures exclusive access to this structure */
#ifndef CONFIG_DISABLE_POLL
  FAR struct pollfd *pfd;         /* Polled file descr  (or NULL if any) */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level SPI helpres */

static void sx127x_lock(FAR struct spi_dev_s *spi);
static void sx127x_unlock(FAR struct spi_dev_s *spi);
static uint8_t sx127x_readregbyte(FAR struct sx127x_dev_s *dev, uint8_t reg);
static void sx127x_writeregbyte(FAR struct sx127x_dev_s *dev, uint8_t reg,
                                uint8_t value);
static uint8_t sx127x_modregbyte(FAR struct sx127x_dev_s *dev, uint8_t reg,
                                 uint8_t setbits, uint8_t clrbits);

/* LORA specific functions */

#ifdef CONFIG_LPWAN_SX127X_LORA
static void sx127x_lora_init(FAR struct sx127x_dev_s *dev);
static int16_t sx127x_lora_rssi_get(FAR struct sx127x_dev_s *dev);
static int16_t sx127x_lora_rssi_correct(FAR struct sx127x_dev_s *dev,
                                        uint32_t freq, int8_t snr,
                                        uint8_t regval);
static void sx127x_lora_preamble_set(FAR struct sx127x_dev_s *dev,
                                     uint32_t len);
static int sx127x_lora_preamble_get(FAR struct sx127x_dev_s *dev);
static int sx127x_lora_opmode_set(FAR struct sx127x_dev_s *dev,
                                  uint8_t opmode);
static int sx127x_lora_opmode_init(FAR struct sx127x_dev_s *dev,
                                   uint8_t opmode);
static int sx127x_lora_syncword_set(FAR struct sx127x_dev_s *dev,
                                    FAR uint8_t *sw, uint8_t len);
static void sx127x_lora_syncword_get(FAR struct sx127x_dev_s *dev,
                                     FAR uint8_t *sw, uint8_t *len);

#  ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
static int8_t sx127x_lora_snr_get(FAR struct sx127x_dev_s *dev);
static int16_t sx127x_lora_pckrssi_get(FAR struct sx127x_dev_s *dev,
                                       int8_t snr);
static int sx127x_lora_rxhandle(FAR struct sx127x_dev_s *dev);
#  endif
#  ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
static int sx127x_lora_send(FAR struct sx127x_dev_s *dev,
                            FAR const uint8_t *data, size_t datalen);
#  endif
#  ifdef CONFIG_DEBUG_WIRELESS_INFO
static void sx127x_lora_dumpregs(FAR struct sx127x_dev_s *dev);
#  endif
#endif

/* FSK/OOK specific functions */

#ifdef CONFIG_LPWAN_SX127X_FSKOOK
static void sx127x_fskook_init(FAR struct sx127x_dev_s *dev);
static int sx127x_fskook_fdev_set(FAR struct sx127x_dev_s *dev,
                                  uint32_t freq);
static int16_t sx127x_fskook_rssi_get(FAR struct sx127x_dev_s *dev);
static int sx127x_fskook_bitrate_set(FAR struct sx127x_dev_s *dev,
                                     uint32_t bitrate);
static void sx127x_fskook_preamble_set(FAR struct sx127x_dev_s *dev,
                                       uint32_t len);
static int sx127x_fskook_preamble_get(FAR struct sx127x_dev_s *dev);
static int sx127x_fskook_opmode_init(FAR struct sx127x_dev_s *dev,
                                     uint8_t opmode);
static int sx127x_fskook_syncword_set(FAR struct sx127x_dev_s *dev,
                                      FAR uint8_t *sw, uint8_t len);
static void sx127x_fskook_syncword_get(FAR struct sx127x_dev_s *dev,
                                       FAR uint8_t *sw, FAR uint8_t *len);
#  ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
static int sx127x_fskook_rxhandle(FAR struct sx127x_dev_s *dev);
#  endif
#  ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
static int sx127x_fskook_send(FAR struct sx127x_dev_s *dev,
                              FAR const uint8_t *data, size_t datalen);
#  endif
#  ifdef CONFIG_DEBUG_WIRELESS_INFO
static void sx127x_fskook_dumpregs(FAR struct sx127x_dev_s *dev);
#  endif
#endif

/* Common for FSK/OOK and LORA */

static int sx127x_fskook_opmode_set(FAR struct sx127x_dev_s *dev,
                                    uint8_t opmode);
static int sx127x_init(FAR struct sx127x_dev_s *dev);
static int sx127x_deinit(FAR struct sx127x_dev_s *dev);
static int sx127x_unregister(FAR struct sx127x_dev_s *dev);
static inline int sx127x_attachirq0(FAR struct sx127x_dev_s *dev, xcpt_t isr,
                                    FAR void *arg);
static int sx127x_irq0handler(int irq, FAR void *context, FAR void *arg);

static int sx127x_modulation_set(FAR struct sx127x_dev_s *dev,
                                 uint8_t modulation);
static uint8_t sx127x_modulation_get(FAR struct sx127x_dev_s *dev);
static int16_t sx127x_rssi_get(FAR struct sx127x_dev_s *dev);
static int sx127x_frequency_set(FAR struct sx127x_dev_s *dev, uint32_t freq);
static uint32_t sx127x_frequency_get(FAR struct sx127x_dev_s *dev);
static void sx127x_preamble_set(FAR struct sx127x_dev_s *dev, uint32_t len);
static int sx127x_preamble_get(FAR struct sx127x_dev_s *dev);
static int sx127x_opmode_set(FAR struct sx127x_dev_s *dev, uint8_t opmode);
static uint8_t sx127x_opmode_get(FAR struct sx127x_dev_s *dev);
static int sx127x_opmode_init(FAR struct sx127x_dev_s *dev, uint8_t opmode);
static int sx127x_syncword_set(FAR struct sx127x_dev_s *dev, FAR uint8_t *sw,
                               uint8_t len);
static void sx127x_syncword_get(FAR struct sx127x_dev_s *dev, FAR uint8_t *sw,
                                FAR uint8_t *len);
#ifdef CONFIG_DEBUG_WIRELESS_INFO
static void sx127x_dumpregs(FAR struct sx127x_dev_s *dev);
#else
#  define sx127x_dumpregs(x)
#endif

static bool sx127x_channel_scan(FAR struct sx127x_dev_s *dev,
                                FAR struct sx127x_chanscan_ioc_s *chanscan);
static uint32_t sx127x_random_get(FAR struct sx127x_dev_s *dev);

#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
static int sx127x_txfifo_write(FAR struct sx127x_dev_s *dev,
                               FAR const uint8_t *data, size_t datalen);
#endif
#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
static ssize_t sx127x_rxfifo_get(struct sx127x_dev_s *dev,
                                 FAR uint8_t *buffer, size_t buflen);
static void sx127x_rxfifo_put(struct sx127x_dev_s *dev, FAR uint8_t *buffer,
                              size_t buflen);
#endif

/* POSIX API */

static int sx127x_open(FAR struct file *filep);
static int sx127x_close(FAR struct file *filep);
static ssize_t sx127x_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t sx127x_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int sx127x_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int sx127x_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Only one device is supported for now */

static struct sx127x_dev_s g_sx127x_devices[1];

/* File ops */

static const struct file_operations sx127x_fops =
{
  sx127x_open,    /* open */
  sx127x_close,   /* close */
  sx127x_read,    /* read */
  sx127x_write,   /* write */
  NULL,           /* seek */
  sx127x_ioctl    /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , sx127x_poll   /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL          /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sx127x_lock
 *
 * Description:
 *   Acquire exclusive access to the shared SPI bus.
 *
 ****************************************************************************/

static void sx127x_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, 1);
  SPI_SETBITS(spi, 8);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETFREQUENCY(spi, SX127X_SPIFREQ);
}

/****************************************************************************
 * Name: sx127x_unlock
 *
 * Description:
 *   Release exclusive access to the shared SPI bus.
 *
 ****************************************************************************/

static void sx127x_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, 0);
}

/****************************************************************************
 * Name: sx127x_select
 ****************************************************************************/

static inline void sx127x_select(struct sx127x_dev_s * dev)
{
  SPI_SELECT(dev->spi, SPIDEV_LPWAN(0), true);
}

/****************************************************************************
 * Name: sx127x_deselect
 ****************************************************************************/

static inline void sx127x_deselect(struct sx127x_dev_s * dev)
{
  SPI_SELECT(dev->spi, SPIDEV_LPWAN(0), false);
}

/****************************************************************************
 * Name: sx127x_access
 ****************************************************************************/

static uint8_t sx127x_access(FAR struct sx127x_dev_s *dev,
                             sx127x_access_mode_t mode, uint8_t cmd,
                             FAR uint8_t *buf, int length)
{
  uint8_t status = 0;

  /* Prepare SPI */

  sx127x_select(dev);

  /* Transfer */

  status = SPI_SEND(dev->spi, cmd);

  switch (mode)
    {
      case MODE_WRITE:
        {
          if (length > 0)
            {
              SPI_SNDBLOCK(dev->spi, buf, length);
            }

          break;
        }

      case MODE_READ:
        {
          SPI_RECVBLOCK(dev->spi, buf, length);
          break;
        }

      default:
        {
          wlerr("ERROR: unknown SPI access mode %d!\n", mode);
          break;
        }
    }

  sx127x_deselect(dev);

  return status;
}

/****************************************************************************
 * Name: sx127x_readreg
 *
 * Description:
 *   Read register from sx127x
 *
 ****************************************************************************/

static inline uint8_t sx127x_readreg(FAR struct sx127x_dev_s *dev,
                                     uint8_t reg, FAR uint8_t *value,
                                     int len)
{
  return sx127x_access(dev, MODE_READ, reg | SX127X_R_REGISTER, value, len);
}

/****************************************************************************
 * Name: sx127x_readregbyte
 *
 * Description:
 *   Read single byte value from a register of sx127x
 *
 ****************************************************************************/

static inline uint8_t sx127x_readregbyte(FAR struct sx127x_dev_s *dev,
                                         uint8_t reg)
{
  uint8_t val = 0;

  sx127x_readreg(dev, reg, &val, 1);

  return val;
}

/****************************************************************************
 * Name: sx127x_writereg
 *
 * Description:
 *   Write value to a register of sx127x
 *
 ****************************************************************************/

static inline int sx127x_writereg(FAR struct sx127x_dev_s *dev, uint8_t reg,
                                  FAR const uint8_t *value, int len)
{
  return sx127x_access(dev, MODE_WRITE, reg | SX127X_W_REGISTER,
                       (FAR uint8_t *)value, len);
}

/****************************************************************************
 * Name: sx127x_writeregbyte
 *
 * Description:
 *   Write single byte value to a register of sx127x
 *
 ****************************************************************************/

static inline void sx127x_writeregbyte(FAR struct sx127x_dev_s *dev,
                                       uint8_t reg, uint8_t value)
{
  sx127x_writereg(dev, reg, &value, 1);
}

/****************************************************************************
 * Name: sx127x_modreg
 *
 * Description:
 *  Modify register value of sx127x
 *
 ****************************************************************************/

static uint8_t sx127x_modregbyte(FAR struct sx127x_dev_s *dev, uint8_t reg,
                                 uint8_t setbits, uint8_t clrbits)
{
  uint8_t val = 0;

  sx127x_readreg(dev, reg, &val, 1);

  val &= ~clrbits;
  val |= setbits;

  sx127x_writereg(dev, reg, &val, 1);
  return val;
}

/****************************************************************************
 * Name: sx127x_attachirq0
 ****************************************************************************/

static inline int sx127x_attachirq0(FAR struct sx127x_dev_s *dev, xcpt_t isr,
                                    FAR void *arg)
{
  return dev->lower->irq0attach(isr, arg);
}

/****************************************************************************
 * Name: sx127x_attachirq1
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_DIO1
static inline int sx127x_attachirq1(FAR struct sx127x_dev_s *dev, xcpt_t isr,
                                    FAR void *arg)
{
  return dev->lower->irq1attach(isr, arg);
}
#endif

/****************************************************************************
 * Name: sx127x_attachirq2
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_DIO2
static inline int sx127x_attachirq2(FAR struct sx127x_dev_s *dev, xcpt_t isr,
                                    FAR void *arg)
{
  return dev->lower->irq2attach(isr, arg);
}
#endif

/****************************************************************************
 * Name: sx127x_attachirq3
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_DIO3
static inline int sx127x_attachirq3(FAR struct sx127x_dev_s *dev, xcpt_t isr,
                                    FAR void *arg)
{
  return dev->lower->irq3attach(isr, arg);
}
#endif

/****************************************************************************
 * Name: sx127x_attachirq4
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_DIO4
static inline int sx127x_attachirq4(FAR struct sx127x_dev_s *dev, xcpt_t isr,
                                    FAR void *arg)
{
  return dev->lower->irq4attach(isr, arg);
}
#endif

/****************************************************************************
 * Name: sx127x_attachirq5
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_DIO5
static inline int sx127x_attachirq5(FAR struct sx127x_dev_s *dev, xcpt_t isr,
                                    FAR void *arg)
{
  return dev->lower->irq5attach(isr, arg);
}
#endif

/****************************************************************************
 * Name: sx127x_reset
 *
 * Description:
 *   Reset radio
 *
 ****************************************************************************/

static void sx127x_reset(FAR struct sx127x_dev_s *dev)
{
  dev->lower->reset();
}

/****************************************************************************
 * Name: sx127x_open
 *
 * Description:
 *   This function is called whenever the SX127X device is opened.
 *
 ****************************************************************************/

static int sx127x_open(FAR struct file *filep)
{
  FAR struct sx127x_dev_s *dev   = NULL;
  FAR struct inode        *inode = NULL;
  int ret = 0;

  wlinfo("Opening sx127x dev\n");

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct sx127x_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxsem_wait(&dev->dev_sem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
      return ret;
    }

  /* Check if device is not already used */

  if (dev->nopens > 0)
    {
      ret = -EBUSY;
      goto errout;
    }

  /* Initialize device */

  ret = sx127x_init(dev);
  if (ret < 0)
    {
      wlerr("ERROR: failed to initialize sx127x\n");
      goto errout;
    }

  /* Dump registers after initial configuration */

  sx127x_dumpregs(dev);

  dev->nopens++;

errout:
  nxsem_post(&dev->dev_sem);

  return ret;
}

/****************************************************************************
 * Name: sx127x_close
 *
 * Description:
 *   This routine is called when the SX127X device is closed.
 *   It waits for the last remaining data to be sent.
 *
 ****************************************************************************/

static int sx127x_close(FAR struct file *filep)
{
  FAR struct sx127x_dev_s *dev   = NULL;
  FAR struct inode        *inode = NULL;
  int ret = 0;

  wlinfo("Closing sx127x dev\n");
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct sx127x_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxsem_wait(&dev->dev_sem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
      return ret;
    }

  ret = sx127x_deinit(dev);
  if (ret < 0)
    {
      wlerr("ERROR: failed to deinit sx127x\n");
    }

  dev->nopens--;

  nxsem_post(&dev->dev_sem);

  return OK;
}

/****************************************************************************
 * Name: sx127x_read
 *
 * Description:
 *   Standard driver read method
 *
 ****************************************************************************/

static ssize_t sx127x_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
#ifndef CONFIG_LPWAN_SX127X_RXSUPPORT
  return -ENOSYS;
#else
  FAR struct sx127x_dev_s *dev   = NULL;
  FAR struct inode        *inode = NULL;
  int ret = 0;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct sx127x_dev_s *)inode->i_private;

  ret = nxsem_wait(&dev->dev_sem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
      return ret;
    }

  if ((filep->f_oflags & O_NONBLOCK) != 0)
    {
      nxsem_trywait(&dev->rx_sem);
      ret = 0;
    }
  else
    {
      ret = nxsem_wait(&dev->rx_sem);
    }

  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
      return ret;
    }

  /* Get RX data from fifo */

  ret = sx127x_rxfifo_get(dev, (uint8_t *)buffer, buflen);

  nxsem_post(&dev->dev_sem);

  return ret;
#endif
}

/****************************************************************************
 * Name: sx127x_write
 *
 * Description:
 *   Standard driver write method.
 *
 ****************************************************************************/

static ssize_t sx127x_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
#ifndef CONFIG_LPWAN_SX127X_TXSUPPORT
  return -ENOSYS;
#else
  FAR struct sx127x_dev_s *dev   = NULL;
  FAR struct inode        *inode = NULL;
  int ret = 0;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct sx127x_dev_s *)inode->i_private;

  ret = nxsem_wait(&dev->dev_sem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
      return ret;
    }

  ret = dev->ops.send(dev, (uint8_t *)buffer, buflen);

  nxsem_post(&dev->dev_sem);

  return ret;
#endif
}

/****************************************************************************
 * Name: sx127x_ioctl
 *
 * Description:
 *   Standard driver ioctl method.
 *
 ****************************************************************************/

static int sx127x_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct sx127x_dev_s *dev = NULL;
  FAR struct inode *inode      = NULL;
  int ret                      = 0;

  wlinfo("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct sx127x_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxsem_wait(&dev->dev_sem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
      return ret;
    }

  /* Process the IOCTL by command */

  switch (cmd)
    {
      /* Set radio frequency. Arg: Pointer to
       * uint32_t frequency value in Hz ! */

      case WLIOC_SETRADIOFREQ:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          sx127x_frequency_set(dev, *ptr);
          break;
        }

      /* Get current radio frequency. arg: Pointer
       * to uint32_t frequency value in Hz! */

      case WLIOC_GETRADIOFREQ:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          *ptr = sx127x_frequency_get(dev);
          break;
        }

      /* Get RSSI */

      case SX127XIOC_RSSIGET:
        {
          FAR int16_t *ptr = (FAR int16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          *ptr = sx127x_rssi_get(dev);
          break;
        }

      /* Set modulation */

      case SX127XIOC_MODULATIONSET:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          ret = sx127x_modulation_set(dev, *ptr);
          break;
        }

      /* Get modulation */

      case SX127XIOC_MODULATIONGET:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          *ptr = sx127x_modulation_get(dev);
          break;
        }

      /* Operation mode set */

      case SX127XIOC_OPMODESET:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          ret = sx127x_opmode_set(dev, *ptr);
          break;
        }

      /* Operation mode get */

      case SX127XIOC_OPMODEGET:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          *ptr = sx127x_opmode_get(dev);
          break;
        }

      /* Channel scan */

      case SX127XIOC_CHANSCAN:
        {
          FAR struct sx127x_chanscan_ioc_s *ptr
              = (FAR struct sx127x_chanscan_ioc_s *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          (void)sx127x_channel_scan(dev, ptr);
          break;
        }

      /* Preamble length set */

      case SX127XIOC_PREAMBLESET:
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          sx127x_preamble_set(dev, *ptr);
          break;
        }

      /* Preamble length get */

      case SX127XIOC_PREAMBLEGET:
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          *ptr = sx127x_preamble_get(dev);
          break;
        }

      /* SyncWord set */

      case SX127XIOC_SYNCWORDSET:
        {
          ASSERT(0);
          sx127x_syncword_set(dev, NULL, 0);
          break;
        }

      /* SyncWord get */

      case SX127XIOC_SYNCWORDGET:
        {
          ASSERT(0);
          sx127x_syncword_get(dev, NULL, 0);
          break;
        }

      /* Get random number based on RSSI */

      case SX127XIOC_RANDOMGET:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          *ptr = sx127x_random_get(dev);
          break;
        }

      default:
        {
          ret = -ENOTTY;
          break;
        }
    }

  nxsem_post(&dev->dev_sem);
  return ret;
}

/****************************************************************************
 * Name: sx127x_poll
 *
 * Description:
 *   Standard driver poll method.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int sx127x_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup)
{
#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
  return -ENOSYS;
#else

  FAR struct sx127x_dev_s *dev   = NULL;
  FAR struct inode        *inode = NULL;
  int ret = 0;

  wlinfo("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev  = (FAR struct sx127x_dev_s *)inode->i_private;

  /* Exclusive access */

  ret = nxsem_wait(&dev->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
      return ret;
    }

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      /* Check if we can accept this poll.
       * For now, only one thread can poll the device at any time
       * (shorter / simpler code)
       */

      if (dev->pfd)
        {
          ret = -EBUSY;
          goto errout;
        }

      dev->pfd = fds;

      /* Is there is already data in the fifo? then trigger POLLIN now -
       * don't wait for RX.
       */

      (void)nxsem_wait(&dev->rx_buffer_sem);
      if (dev->rx_fifo_len > 0)
        {
          /* Data available for input */

          dev->pfd->revents |= POLLIN;
          nxsem_post(dev->pfd->sem);
        }

      nxsem_post(&dev->rx_buffer_sem);
    }
  else /* Tear it down */
    {
      dev->pfd = NULL;
    }

errout:
  nxsem_post(&dev->devsem);
  return ret;
#endif
}
#endif

/****************************************************************************
 * Name: sx127x_lora_isr0_process
 *
 * Description:
 *   Handle DIO0 interrupt for LORA radio
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_LORA
static int sx127x_lora_isr0_process(FAR struct sx127x_dev_s *dev)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
  bool    data_valid = true;
#endif
  uint8_t irq        = 0;
  int     ret        = OK;

  /* Get IRQ */

  sx127x_lock(dev->spi);
  irq = sx127x_readregbyte(dev, SX127X_LRM_IRQ);
  sx127x_unlock(dev->spi);

  wlinfo("ISR0: IRQ = 0x%02x\n", irq);

#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
  /* RX data valid ? */

  if (dev->opmode == SX127X_OPMODE_TX && dev->crcon == true &&
      (irq & SX127X_LRM_IRQ_PAYLOADCRCERR) != 0)
    {
      data_valid = false;
    }
#endif

  switch (dev->opmode)
    {
#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
      /* TX DONE for FSK/OOK and LORA */

      case SX127X_OPMODE_TX:
        {
          /* Release TX sem */

          nxsem_post(&dev->tx_sem);

          /* Clear TX interrupt */

          irq = SX127X_LRM_IRQ_TXDONE;
          break;
        }
#endif  /* CONFIG_LPWAN_SX127X_TXSUPPORT */

#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
      /* RX DONE */

      case SX127X_OPMODE_RX:
      case SX127X_OPMODE_RXSINGLE:
        {
          if (data_valid)
            {
              sx127x_lora_rxhandle(dev);

#ifndef CONFIG_DISABLE_POLL
              if (dev->pfd)
                {
                  /* Data available for input */

                  dev->pfd->revents |= POLLIN;

                  wlinfo("Wake up polled fd\n");
                  nxsem_post(dev->pfd->sem);
                }
#endif  /* CONFIG_DISABLE_POLL */

              /* Wake-up any thread waiting in recv */

              nxsem_post(&dev->rx_sem);
            }
          else
            {
              /* RX Data invalid */

              wlinfo("Invalid LORA RX data!\n");
            }

          /* After receiving the data in RXSINGLE mode the chip goes into
           * STANBY mode
           */

          if (dev->opmode == SX127X_OPMODE_RXSINGLE)
            {
              dev->opmode = SX127X_OPMODE_STANDBY;
            }

          /* Clear RX interrupts  */

          irq = SX127X_LRM_IRQ_RXDONE | SX127X_LRM_IRQ_PAYLOADCRCERR |
                SX127X_LRM_IRQ_VALIDHDR;
          break;
        }
#endif  /* CONFIG_LPWAN_SX127X_RXSUPPORT */

      /* Only LORA - CAD DONE */

      case SX127X_OPMODE_CAD:
        {
          /* TODO */

          wlerr("TODO: ISR0 in CAD mode not implemented yet!\n");

          /* Clear CAD interrupt */

          irq = SX127X_LRM_IRQ_CADDONE;
          break;
        }

      default:
        {
          wlwarn("WARNING: Interrupt not processed\n");
          ret = -EINVAL;
          break;
        }
    }

  /* Clear interrupts */

  sx127x_lock(dev->spi);
  sx127x_writeregbyte(dev, SX127X_LRM_IRQ, irq);
  sx127x_unlock(dev->spi);

  return ret;
}
#endif  /* CONFIG_LPWAN_SX127X_LORA */

/****************************************************************************
 * Name: sx127x_fskook_isr0_process
 *
 * Description:
 *   Handle DIO0 interrupt for FSK/OOK radio
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_FSKOOK
static int sx127x_fskook_isr0_process(FAR struct sx127x_dev_s *dev)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
  bool    data_valid = true;
#endif
  uint8_t irq1       = 0;
  uint8_t irq2       = 0;
  int     ret        = OK;

  /* Get IRQ1 and IRQ2 */

  sx127x_lock(dev->spi);
  irq1 = sx127x_readregbyte(dev, SX127X_FOM_IRQ1);
  irq2 = sx127x_readregbyte(dev, SX127X_FOM_IRQ2);
  sx127x_unlock(dev->spi);

  wlinfo("ISR0: IRQ1 = 0x%02x, IRQ2 = 0x%02x\n", irq1, irq2);

#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
  /* RX data valid ? */

  if (dev->opmode == SX127X_OPMODE_RX && dev->crcon == true &&
      (irq2 & SX127X_FOM_IRQ2_CRCOK) == 0)
    {
      data_valid = false;
    }
#endif

  switch (dev->opmode)
    {
#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
      /* TX DONE for FSK/OOK and LORA */

      case SX127X_OPMODE_TX:
        {
          /* Release TX sem */

          nxsem_post(&dev->tx_sem);
          break;
        }
#endif  /* CONFIG_LPWAN_SX127X_TXSUPPORT */

#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
      /* RX DONE */

      case SX127X_OPMODE_RX:
        {
          if (data_valid == true)
            {
              /* RX data valid */

              sx127x_fskook_rxhandle(dev);

#ifndef CONFIG_DISABLE_POLL
              if (dev->pfd)
                {
                  /* Data available for input */

                  dev->pfd->revents |= POLLIN;

                  wlinfo("Wake up polled fd\n");
                  nxsem_post(dev->pfd->sem);
                }
#endif  /* CONFIG_DISABLE_POLL */

              /* Wake-up any thread waiting in recv */

              nxsem_post(&dev->rx_sem);
            }
          else
            {
              /* RX Data invalid */

              wlinfo("Invalid FSK/OOK RX data!\n");
            }

          /* TODO: restart RX if continuous mode */

          break;
        }
#endif  /* CONFIG_LPWAN_SX127X_RXSUPPORT */

      default:
        {
          wlwarn("WARNING: Interrupt not processed\n");
          ret = -EINVAL;
          break;
        }
    }

  /* REVISIT: clear interrupts */

  irq1 = (SX127X_FOM_IRQ1_RSSI | SX127X_FOM_IRQ1_PREAMBE |
          SX127X_FOM_IRQ1_SYNCADDRMATCH);
  irq2 = SX127X_FOM_IRQ2_FIFOOVR;

  sx127x_lock(dev->spi);
  sx127x_writeregbyte(dev, SX127X_FOM_IRQ1, irq1);
  sx127x_writeregbyte(dev, SX127X_FOM_IRQ2, irq2);
  sx127x_unlock(dev->spi);

  return ret;
}
#endif  /* CONFIG_LPWAN_SX127X_FSKOOK */

/****************************************************************************
 * Name: sx127x_isr0_process
 *
 * Description:
 *   Handle DIO0 interrupt for LORA radio
 *
 ****************************************************************************/

static void sx127x_isr0_process(FAR void *arg)
{
  DEBUGASSERT(arg);

  FAR struct sx127x_dev_s *dev = (struct sx127x_dev_s *)arg;
  int ret = OK;

  /* Return immediately if isr0_process is not initialized */

  if (dev->ops.isr0_process == NULL)
    {
      return;
    }

  /* isr0_process depends on the current modulation scheme */

  ret = dev->ops.isr0_process(dev);
  if (ret < 0)
    {
      wlerr("Failed to process ISR0 %d\n", ret);
    }
}

/****************************************************************************
 * Name: sx127x_irq0handler
 ****************************************************************************/

static int sx127x_irq0handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct sx127x_dev_s *dev = (FAR struct sx127x_dev_s *)arg;

  DEBUGASSERT(arg);

  work_queue(HPWORK, &dev->irq0_work, sx127x_isr0_process, dev, 0);

  return 0;
}

#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT

/****************************************************************************
 * Name: sx127x_fskook_rxhandle
 *
 * Description:
 *   Receive data from FIFO for FSK/OOK radio
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_FSKOOK
static int sx127x_fskook_rxhandle(FAR struct sx127x_dev_s *dev)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  struct sx127x_read_hdr_s rxdata;
  uint8_t datalen = 0;
  uint8_t len     = 0;

  /* Get data from chip fifo */

  if (dev->fskook.fixlen == true)
    {
      /* Fixed packet length hardcoded */

      datalen = SX127X_RX_FIXLEN_DEFAULT;
    }
  else
    {
      /* First byte is payload length */

      datalen = sx127x_readregbyte(dev, SX127X_CMN_FIFO);
    }

  /* Read payload and store */

  sx127x_readreg(dev, SX127X_CMN_FIFO, rxdata.data, datalen);

  /* No RX SNR data for FSK/OOK */

  rxdata.snr = 0;

  /* Store last RSSI */

  rxdata.rssi = sx127x_fskook_rssi_get(dev);

  /* Store packet length */

  rxdata.datalen = datalen;

  /* Total length */

  len = datalen + SX127X_READ_DATA_HEADER_LEN;

  /* Put data on local fifo */

  sx127x_rxfifo_put(dev, (uint8_t *)&rxdata, len);

  /* Return total length */

  return len;
}
#endif  /* CONFIG_LPWAN_SX127X_FSKOOK */

/****************************************************************************
 * Name: sx127x_lora_rxhandle
 *
 * Description:
 *   Receive data from FIFO for LORA radio
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_LORA
static int sx127x_lora_rxhandle(FAR struct sx127x_dev_s *dev)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  struct sx127x_read_hdr_s rxdata;
  uint8_t datalen = 0;
  uint8_t len     = 0;
  uint8_t rx_ptr  = 0;

  /* Get payload length */

  datalen = sx127x_readregbyte(dev, SX127X_LRM_RXBYTES);

  /* Get start address of last packet received */

  rx_ptr = sx127x_readregbyte(dev, SX127X_LRM_RXCURR);

  /* Set FIFO pointer */

  sx127x_writeregbyte(dev, SX127X_LRM_ADDRPTR, rx_ptr);

  /* Read payload */

  sx127x_readreg(dev, SX127X_CMN_FIFO, rxdata.data, datalen);

  /* Store last RX SNR */

  rxdata.snr = sx127x_lora_snr_get(dev);

  /* Store last RX RSSI */

  rxdata.rssi = sx127x_lora_pckrssi_get(dev, rxdata.snr);

  /* Store packet length */

  rxdata.datalen = datalen;

  /* Total length */

  len = datalen + SX127X_READ_DATA_HEADER_LEN;

  /* Put data on local fifo */

  sx127x_rxfifo_put(dev, (uint8_t *)&rxdata, len);

  /* Return total length */

  return len;
}
#endif  /* CONFIG_LPWAN_SX127X_LORA */

/****************************************************************************
 * Name: sx127x_rxfifo_get
 *
 * Description:
 *   Get data from RX FIFO
 *
 ****************************************************************************/

static ssize_t sx127x_rxfifo_get(FAR struct sx127x_dev_s *dev,
                                 FAR uint8_t *buffer, size_t buflen)
{
  FAR struct sx127x_read_hdr_s *pkt = NULL;
  uint8_t pktlen = 0;
  uint8_t i      = 0;
  int     ret    = 0;

  ret = nxsem_wait(&dev->rx_buffer_sem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
      return ret;
    }

  /* No data on RX FIFO */

  if (dev->rx_fifo_len == 0)
    {
      pktlen = 0;
      goto no_data;
    }

  /* Get packet header */

  pkt = (struct sx127x_read_hdr_s *)(dev->rx_buffer +
                                     dev->nxt_read * SX127X_RXFIFO_ITEM_SIZE);

  /* Packet length is data length + header length */

  pktlen = pkt->datalen + SX127X_READ_DATA_HEADER_LEN;

  /* Get packet from FIFO */

  for (i = 0; i < pktlen && i < SX127X_RXFIFO_ITEM_SIZE; i += 1)
    {
      buffer[i] = dev->rx_buffer[dev->nxt_read * SX127X_RXFIFO_ITEM_SIZE + i];
    }

  dev->nxt_read = (dev->nxt_read + 1) % CONFIG_LPWAN_SX127X_RXFIFO_LEN;
  dev->rx_fifo_len--;

  ret = pktlen;

no_data:
  nxsem_post(&dev->rx_buffer_sem);
  return ret;
}

/****************************************************************************
 * Name: sx127x_rxfifo_put
 *
 * Description:
 *   Put packet data on RX FIFO
 *
 ****************************************************************************/

static void sx127x_rxfifo_put(FAR struct sx127x_dev_s *dev,
                              FAR uint8_t *buffer, size_t buflen)
{
  uint8_t i   = 0;
  int     ret = 0;

  ret = nxsem_wait(&dev->rx_buffer_sem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
      return;
    }

    dev->rx_fifo_len++;
    if (dev->rx_fifo_len > CONFIG_LPWAN_SX127X_RXFIFO_LEN)
      {
        dev->rx_fifo_len = CONFIG_LPWAN_SX127X_RXFIFO_LEN;
        dev->nxt_read = (dev->nxt_read + 1) % CONFIG_LPWAN_SX127X_RXFIFO_LEN;
      }

    /* Put packet on fifo */

    for (i = 0; i < (buflen + 1) && i < SX127X_RXFIFO_ITEM_SIZE; i += 1)
      {
        dev->rx_buffer[i + dev->nxt_write * SX127X_RXFIFO_ITEM_SIZE] =
          buffer[i];
      }

    dev->nxt_write = (dev->nxt_write + 1) % CONFIG_LPWAN_SX127X_RXFIFO_LEN;
    nxsem_post(&dev->rx_buffer_sem);
}

#endif /* CONFIG_LPWAN_SX127X_RXSUPPORT */

#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT

/****************************************************************************
 * Name: sx127x_txfifo_write
 *
 * Description:
 *   Write data to the SX127X TX FIFO
 *
 ****************************************************************************/

static int sx127x_txfifo_write(FAR struct sx127x_dev_s *dev,
                               FAR const uint8_t *data, size_t datalen)
{
  /* NOTE: Do not lock SPI here, it should be already locked! */

  /* Write buffer to FIFO */

  sx127x_writereg(dev, SX127X_CMN_FIFO, data, datalen);

  return OK;
}

/****************************************************************************
 * Name: sx127x_fskook_send
 *
 * Description:
 *   Send data in FSK/OOK radio mode
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_FSKOOK
static int sx127x_fskook_send(FAR struct sx127x_dev_s *dev,
                              FAR const uint8_t *data, size_t datalen)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  int ret = 0;

  /* Check payload length */

  if (datalen > SX127X_FOM_PAYLOADLEN_MAX)
    {
      wlerr("Not supported data len!\n");
      ret = -EINVAL;
      goto errout;
    }

#if 1
  /* For now we don't support datalen > FIFO_LEN for FSK/OOK */

  if (datalen > 64)
    {
      wlerr("Not supported data len!\n");
      ret = -EINVAL;
      goto errout;
    }
#endif

  /* Change mode to STANDBY */

  sx127x_opmode_set(dev, SX127X_OPMODE_STANDBY);

  /* Initialize TX mode */

  ret = sx127x_opmode_init(dev, SX127X_OPMODE_TX);
  if (ret < 0)
    {
      wlerr("Failed to initialize TX mode!\n");
      ret = -EINVAL;
      goto errout;
    }

  /* Lock SPI */

  sx127x_lock(dev->spi);

  if (dev->fskook.fixlen == true)
    {
      /* Write payload length reigster (only LSB for now) */

      sx127x_writeregbyte(dev, SX127X_FOM_PAYLOADLEN, datalen);
    }
  else
    {
      /* First byte is length */

      ret = sx127x_txfifo_write(dev, (uint8_t *)&datalen, 1);
    }

  /* Write payload */

  sx127x_txfifo_write(dev, data, datalen);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Change mode to TX to start data transfer */

  sx127x_opmode_set(dev, SX127X_OPMODE_TX);

  /* Wait for payload send IRQ */

  nxsem_wait(&dev->tx_sem);

  /* Change mode to IDLE after transfer
   * NOTE: if sequencer is on - this should be done automatically
   */

  sx127x_opmode_set(dev, dev->idle);

errout:
  return ret;
}
#endif  /* CONFIG_LPWAN_SX127X_FSKOOK */

/****************************************************************************
 * Name: sx127x_lora_send
 *
 * Description:
 *   Send data in LORA radio mode
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_LORA
static int sx127x_lora_send(FAR struct sx127x_dev_s *dev,
                            FAR const uint8_t *data, size_t datalen)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  int ret = 0;

  /* Check payload length */

  if (datalen > SX127X_LRM_PAYLOADLEN_MAX)
    {
      wlerr("Not supported data len!\n");
      ret = -EINVAL;
      goto errout;
    }

  /* Change mode to STANDBY */

  sx127x_opmode_set(dev, SX127X_OPMODE_STANDBY);

  /* Initialize TX mode */

  ret = sx127x_opmode_init(dev, SX127X_OPMODE_TX);
  if (ret < 0)
    {
      wlerr("Failed to initialize TX mode!\n");
      ret = -EINVAL;
      goto errout;
    }

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Configure payload length */

  sx127x_writeregbyte(dev, SX127X_LRM_PAYLOADLEN, datalen);

  /* Write payload */

  sx127x_txfifo_write(dev, data, datalen);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Change mode to TX to start data transfer */

  sx127x_opmode_set(dev, SX127X_OPMODE_TX);

  /* Wait for TXDONE */

  nxsem_wait(&dev->tx_sem);

  /* Change mode to IDLE after transfer */

  /* sx127x_opmode_set(dev, dev->idle); */
  dev->opmode = SX127X_OPMODE_STANDBY;

errout:
  return ret;
}
#endif  /* CONFIG_LPWAN_SX127X_LORA */
#endif  /* CONFIG_LPWAN_SX127X_TXSUPPORT */

/****************************************************************************
 * Name: sx127x_opmode_init
 *
 * Description:
 *   Initialize operation mode
 *
 ****************************************************************************/

static int sx127x_opmode_init(FAR struct sx127x_dev_s *dev, uint8_t opmode)
{
  int ret = OK;

  if (opmode == dev->opmode)
    {
      goto errout;
    }

  dev->ops.opmode_init(dev, opmode);

errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_opmode_set
 *
 * Description:
 *   Set operation mode
 *
 ****************************************************************************/

static int sx127x_opmode_set(FAR struct sx127x_dev_s *dev, uint8_t opmode)
{
  int ret = OK;

  wlinfo("opmode_set %d->%d\n", dev->opmode, opmode);

  if (opmode == dev->opmode)
    {
      goto errout;
    }

#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
  /* REVISIT: TX is initialized before data send,
   * but where we should initialize RX ?
   */

  if (opmode == SX127X_OPMODE_RX || opmode == SX127X_OPMODE_RXSINGLE)
    {
      ret = sx127x_opmode_init(dev, opmode);
    }
#endif

  /* Change mode */

  dev->ops.opmode_set(dev, opmode);

  /* Update local variable */

  dev->opmode = opmode;

errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_opmode_get
 *
 * Description:
 *   Get current operation mode
 *
 ****************************************************************************/

static uint8_t sx127x_opmode_get(FAR struct sx127x_dev_s *dev)
{
  wlerr("TODO: sx127x_opmode_get not implemented yet\n");
  return 0;
}

#ifdef CONFIG_LPWAN_SX127X_FSKOOK

/****************************************************************************
 * Name: sx127x_lora_opmode_init
 *
 * Description:
 *   Initialize operation mode for FSK/OOK
 *
 ****************************************************************************/

static int sx127x_fskook_opmode_init(FAR struct sx127x_dev_s *dev,
                                     uint8_t opmode)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  uint8_t dio0map = 0;
  uint8_t setbits = 0;
  uint8_t clrbits = 0;
  int     ret     = OK;

  sx127x_lock(dev->spi);

  /* Get mode specific configuration */

  switch (opmode)
    {
      case SX127X_OPMODE_SLEEP:
      case SX127X_OPMODE_STANDBY:
      case SX127X_OPMODE_FSRX:
      case SX127X_OPMODE_FSTX:
        {
          break;
        }

      case SX127X_OPMODE_TX:
        {
          /* Remap DIO0 to RXTX DONE */

          dio0map = SX127X_FOM_DIOMAP1_DIO0_RXTX;

          /* TX start condition on FIFO not empty */

          sx127x_writeregbyte(dev, SX127X_FOM_FIFOTHR,
                              SX127X_FOM_FIFOTHR_TXSTARTCOND);

          break;
        }

      case SX127X_OPMODE_RX:
        {
          /* Remap DIO0 to RXTX DONE */

          dio0map = SX127X_FOM_DIOMAP1_DIO0_RXTX;

          break;
        }

      default:
        {
          wlerr("ERROR: invalid mode %d\n", opmode);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Configure DIO0 pin */

  setbits = dio0map;
  clrbits = SX127X_CMN_DIOMAP1_DIO0_MASK;
  sx127x_modregbyte(dev, SX127X_CMN_DIOMAP1, setbits, clrbits);

  sx127x_unlock(dev->spi);

errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_fskook_opmode_set
 *
 * Description:
 *   Set operation mode for FSK/OOK
 *
 ****************************************************************************/

static int sx127x_fskook_opmode_set(FAR struct sx127x_dev_s *dev,
                                    uint8_t opmode)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  uint8_t regval  = 0;
  uint8_t setbits = 0;
  uint8_t clrbits = 0;
  int     ret     = OK;

  switch (opmode)
    {
      case SX127X_OPMODE_SLEEP:
      case SX127X_OPMODE_STANDBY:
      case SX127X_OPMODE_FSRX:
      case SX127X_OPMODE_FSTX:
      case SX127X_OPMODE_TX:
      case SX127X_OPMODE_RX:
        {
          /* Do nothing */

          break;
        }

      default:
        {
          wlerr("ERROR: invalid FSK/OOK mode %d\n", opmode);
          ret = -EINVAL;
          goto errout;
        }
    }

  sx127x_lock(dev->spi);

  /* Update mode */

  setbits = ((opmode-1) << SX127X_CMN_OPMODE_MODE_SHIFT);
  clrbits = SX127X_CMN_OPMODE_MODE_MASK;
  sx127x_modregbyte(dev, SX127X_CMN_OPMODE, setbits, clrbits);

  /* Wait for mode ready */

  do
    {
      usleep(1000);

      if (opmode == SX127X_OPMODE_SLEEP)
        {
          /* REVISIT: Somehow MODERDY doesnt work for SLEEP,
           * so we just break here
           */
          break;
        }

      regval = sx127x_readregbyte(dev, SX127X_FOM_IRQ1);
    }
  while (!(regval & SX127X_FOM_IRQ1_MODERDY));

  sx127x_unlock(dev->spi);

errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_fskook_rxbw_set
 *
 * Description:
 *  Set RX BW for FSK/OOK
 *
 ****************************************************************************/

static int sx127x_fskook_rxbw_set(FAR struct sx127x_dev_s *dev, uint8_t rx_bw)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  int ret = OK;

  if (rx_bw == dev->fskook.rx_bw)
    {
      goto errout;
    }

  switch (rx_bw)
    {
      case FSKOOK_BANDWIDTH_2p6kHz:
      case FSKOOK_BANDWIDTH_3p1kHz:
      case FSKOOK_BANDWIDTH_3p9kHz:
      case FSKOOK_BANDWIDTH_5p2kHz:
      case FSKOOK_BANDWIDTH_6p3kHz:
      case FSKOOK_BANDWIDTH_7p8kHz:
      case FSKOOK_BANDWIDTH_10p4kHz:
      case FSKOOK_BANDWIDTH_12p5kHz:
      case FSKOOK_BANDWIDTH_15p6kHz:
      case FSKOOK_BANDWIDTH_20p8kHz:
      case FSKOOK_BANDWIDTH_25kHz:
      case FSKOOK_BANDWIDTH_31p3kHz:
      case FSKOOK_BANDWIDTH_41p7kHz:
      case FSKOOK_BANDWIDTH_50kHz:
      case FSKOOK_BANDWIDTH_62p5kHz:
      case FSKOOK_BANDWIDTH_83p3kHz:
      case FSKOOK_BANDWIDTH_100kHz:
      case FSKOOK_BANDWIDTH_125kHz:
      case FSKOOK_BANDWIDTH_166p7kHz:
      case FSKOOK_BANDWIDTH_200kHz:
      case FSKOOK_BANDWIDTH_250kHz:
        {
          /* Lock SPI */

          sx127x_lock(dev->spi);

          /* Write register */

          sx127x_writeregbyte(dev, SX127X_FOM_RXBW, rx_bw);

          /* Unlock SPI */

          sx127x_unlock(dev->spi);

          break;
        }

      default:
        {
          wlerr("Unsupported bandwidth %d\n", rx_bw);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Update local */

  dev->fskook.rx_bw = rx_bw;

errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_fskook_afcbw_set
 *
 * Description:
 *  Set AFC BW for FSK/OOK
 *
 ****************************************************************************/

static int sx127x_fskook_afcbw_set(FAR struct sx127x_dev_s *dev,
                                   uint8_t afc_bw)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  int ret = OK;

  if (afc_bw == dev->fskook.afc_bw)
    {
      goto errout;
    }

  switch (afc_bw)
    {
      case FSKOOK_BANDWIDTH_2p6kHz:
      case FSKOOK_BANDWIDTH_3p1kHz:
      case FSKOOK_BANDWIDTH_3p9kHz:
      case FSKOOK_BANDWIDTH_5p2kHz:
      case FSKOOK_BANDWIDTH_6p3kHz:
      case FSKOOK_BANDWIDTH_7p8kHz:
      case FSKOOK_BANDWIDTH_10p4kHz:
      case FSKOOK_BANDWIDTH_12p5kHz:
      case FSKOOK_BANDWIDTH_15p6kHz:
      case FSKOOK_BANDWIDTH_20p8kHz:
      case FSKOOK_BANDWIDTH_25kHz:
      case FSKOOK_BANDWIDTH_31p3kHz:
      case FSKOOK_BANDWIDTH_41p7kHz:
      case FSKOOK_BANDWIDTH_50kHz:
      case FSKOOK_BANDWIDTH_62p5kHz:
      case FSKOOK_BANDWIDTH_83p3kHz:
      case FSKOOK_BANDWIDTH_100kHz:
      case FSKOOK_BANDWIDTH_125kHz:
      case FSKOOK_BANDWIDTH_166p7kHz:
      case FSKOOK_BANDWIDTH_200kHz:
      case FSKOOK_BANDWIDTH_250kHz:
        {
          /* Lock SPI */

          sx127x_lock(dev->spi);

          /* Write register */

          sx127x_writeregbyte(dev, SX127X_FOM_AFCBW, afc_bw);

          /* Unlock SPI */

          sx127x_unlock(dev->spi);

          break;
        }

      default:
        {
          wlerr("Unsupported bandwidth %d\n", afc_bw);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Update local */

  dev->fskook.afc_bw = afc_bw;

errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_fskook_seq_start
 ****************************************************************************/

static void sx127x_fskook_seq_start(FAR struct sx127x_dev_s *dev, bool state)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  /* Lock SPI */

  sx127x_lock(dev->spi);

  if (state == true)
    {
      /* Start sequencer */

      sx127x_modregbyte(dev, SX127X_FOM_SEQCFG1,
                        SX127X_FOM_SEQCFG1_SEQSTART, 0);
    }
  else
    {
      /* Stop sequencer */

      sx127x_modregbyte(dev, SX127X_FOM_SEQCFG1,
                        SX127X_FOM_SEQCFG1_SEQSTOP, 0);
    }

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Store sequencer state */

  dev->fskook.seqon = state;
}

/****************************************************************************
 * Name: sx127x_fskook_seq_init
 *
 * Description:
 *   Initialize FSK/OOK sequencer.
 *   This can be used to automate transitions between operation modes and
 *   thus further reduce energy consumption.
 *
 ****************************************************************************/

static int sx127x_fskook_seq_init(FAR struct sx127x_dev_s *dev)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  uint8_t seq1 = 0;
  uint8_t seq2 = 0;
  int     ret  = OK;

  /* Need sleep mode or standby mode */

  if (dev->opmode > SX127X_OPMODE_STANDBY)
    {
      sx127x_opmode_set(dev, SX127X_OPMODE_STANDBY);
    }

  /* Nothing here */

  seq1 = 0;
  seq2 = 0;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Write registers */

  sx127x_writeregbyte(dev, SX127X_FOM_SEQCFG1, seq1);
  sx127x_writeregbyte(dev, SX127X_FOM_SEQCFG2, seq2);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  return ret;
}

/****************************************************************************
 * Name: sx127x_fskook_syncword_get
 ****************************************************************************/

static void sx127x_fskook_syncword_get(FAR struct sx127x_dev_s *dev,
                                       FAR uint8_t *sw, FAR uint8_t *len)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  wlerr("sx127x_fskook_syncword_get not implemented yet\n");
}

/****************************************************************************
 * Name: sx127x_syncword_get
 ****************************************************************************/

static void sx127x_syncword_get(FAR struct sx127x_dev_s *dev, FAR uint8_t *sw,
                                FAR uint8_t *len)
{
  dev->ops.syncword_get(dev, sw, len);
}

/****************************************************************************
 * Name: sx127x_fskook_syncword_set
 *
 * Description:
 *   Set SyncWord for FSK/OOK
 *
 ****************************************************************************/

static int sx127x_fskook_syncword_set(FAR struct sx127x_dev_s *dev,
                                      FAR uint8_t *sw, uint8_t len)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  uint8_t setbits = 0;
  uint8_t clrbits = 0;
  uint8_t offset  = 0;
  int     ret     = OK;
  int     i       = 0;

  if (len > SX127X_FOM_SYNCSIZE_MAX)
    {
      wlerr("Unsupported sync word length %d!", len);
      ret = -EINVAL;
      goto errout;
    }

  /* Lock SPI */

  sx127x_lock(dev->spi);

  if (len == 0)
    {
      /* Disable sync word generation and detection */

      clrbits = SX127X_FOM_SYNCCFG_SYNCSIZE_MASK |
                SX127X_FOM_SYNCCFG_SYNCON;
      setbits = 0;

      sx127x_modregbyte(dev, SX127X_FOM_SYNCCFG, setbits, clrbits);

    }
  else
    {
      /* Configure sync word length */

      clrbits = SX127X_FOM_SYNCCFG_SYNCSIZE_MASK;
      setbits = SX127X_FOM_SYNCCFG_SYNCON |
                SX127X_FOM_SYNCCFG_SYNCSIZE(len - 1);

      sx127x_modregbyte(dev, SX127X_FOM_SYNCCFG, setbits, clrbits);

      /* Write sync words */

      for (i = 0; i < len; i += 1)
        {
          offset = SX127X_FOM_SYNCVAL1 + i;
          sx127x_writeregbyte(dev, offset, sw[i]);
        }
    }

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_syncword_set
 ****************************************************************************/

static int sx127x_syncword_set(FAR struct sx127x_dev_s *dev, FAR uint8_t *sw,
                               uint8_t len)
{
  return dev->ops.syncword_set(dev, sw, len);
}

/****************************************************************************
 * Name: sx127x_fskook_init
 *
 * Description:
 *   Initialization specific for FSK/OOK modulation
 *
 ****************************************************************************/

static void sx127x_fskook_init(FAR struct sx127x_dev_s *dev)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  uint8_t setbits = 0;
  uint8_t clrbits = 0;

  /* Set FDEV */

  sx127x_fskook_fdev_set(dev, SX127X_FREQ_DEV_DEFAULT);

  /* Set bitrate */

  sx127x_fskook_bitrate_set(dev, SX127X_FOM_BITRATE_DEFAULT);

  /* Configure sequencer
   * WARNING: sequencer is OFF for now!
   */

  sx127x_fskook_seq_init(dev);
  sx127x_fskook_seq_start(dev, false);

  /* Configure Sync Word - disable */

  sx127x_fskook_syncword_set(dev, NULL, 0);

  /* Configure bandwidth */

  sx127x_fskook_rxbw_set(dev, SX127X_FSKOOK_RXBW_DEFAULT);
  sx127x_fskook_afcbw_set(dev, SX127X_FSKOOK_AFCBW_DEFAULT);

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Configure packet mode settings 1:
   *   - fixlen
   *   - RX/TX CRC
   */

  setbits  = 0;
  clrbits  = 0;
  setbits |= (dev->fskook.fixlen == true ? SX127X_FOM_PKTCFG1_PCKFORMAT : 0);
  setbits |= (dev->crcon == true ? SX127X_FOM_PKTCFG1_CRCON : 0);

  /* Write packet mode settings 1 */

  sx127x_modregbyte(dev, SX127X_FOM_PKTCFG1, setbits, clrbits);

  /* Configure packet mode settings 2:
   *   - packet mode on
   */

  setbits  = 0;
  clrbits  = 0;
  setbits |= SX127X_FOM_PKTCFG2_DATAMODE;

  /* Write packet mode settings 2 */

  sx127x_modregbyte(dev, SX127X_FOM_PKTCFG1, setbits, clrbits);

  /* Configure PARAMP register */

  setbits = SX127X_FSKOOK_SHAPING_DEFAULT | SX127X_FSKOOK_PARAMP_DEFAULT;
  clrbits = SX127X_CMN_PARAMP_PARAMP_MASK | SX127X_CMN_PARAMP_SHAPING_MASK;

  /* Write PARAMP register */

  sx127x_modregbyte(dev, SX127X_CMN_PARAMP, setbits, clrbits);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);
}

/****************************************************************************
 * Name: sx127x_fskook_rssi_get
 *
 * Description:
 *   Get current RSSI for FSK/OOK modem
 *
 ****************************************************************************/

static int16_t sx127x_fskook_rssi_get(FAR struct sx127x_dev_s *dev)
{
  uint8_t regval = 0;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Get register value */

  regval = sx127x_readregbyte(dev, SX127X_FOM_RSSIVAL);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Return decoded RSSI value */

  return SX127X_FOM_RSSIVAL_GET(regval);
}

/****************************************************************************
 * Name: sx127x_fskook_fdev_set
 *
 * Description:
 *   Set frequency deviation
 *
 ****************************************************************************/

static int sx127x_fskook_fdev_set(FAR struct sx127x_dev_s *dev,
                                  uint32_t freq)
{
  uint32_t fdev = 0;
  int      ret  = OK;

  /* Only for FSK modulation */

  if (dev->modulation != SX127X_MODULATION_FSK)
    {
      ret = -EINVAL;
      goto errout;
    }

  if (freq == dev->fskook.fdev)
    {
      goto errout;
    }

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Get FDEV value */

  fdev = SX127X_FDEV_FROM_FREQ(freq);

  /* Write FDEV MSB */

  sx127x_writeregbyte(dev, SX127X_FOM_FDEVMSB, SX127X_FOM_FDEV_MSB(fdev));

  /* Write FDEV LSB */

  sx127x_writeregbyte(dev, SX127X_FOM_FDEVLSB, SX127X_FOM_FDEV_LSB(fdev));

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Update local variable */

  dev->fskook.fdev = freq;

  errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_fskook_bitrate_set
 *
 * Description:
 *   Set bitrate for FSK/OOK modulation
 *
 ****************************************************************************/

static int sx127x_fskook_bitrate_set(FAR struct sx127x_dev_s *dev,
                                     uint32_t bitrate)
{
  uint32_t br  = 0;
  int      ret = OK;

  if (bitrate == dev->fskook.bitrate)
    {
      goto errout;
    }

  /* Get bitrate register value */

  br = SX127X_FXOSC / bitrate;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Set fractial part to 0 */

  sx127x_writeregbyte(dev, SX127X_FOM_BITRATEFRAC, 0);

  /* Write MSB */

  sx127x_writeregbyte(dev, SX127X_FOM_BITRATEMSB,
                      SX127X_FOM_BITRATE_MSB(br));

  /* Write LSB */

  sx127x_writeregbyte(dev, SX127X_FOM_BITRATELSB,
                      SX127X_FOM_BITRATE_LSB(br));

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Update local variable */

  dev->fskook.bitrate = bitrate;

errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_fskook_preamble_set
 *
 * Description:
 *   Set preamble for FSK/OOK modulation
 *
 ****************************************************************************/

static void sx127x_fskook_preamble_set(FAR struct sx127x_dev_s *dev,
                                       uint32_t len)
{
  uint8_t regval = 0;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  if (len == 0)
    {
      /* Disable detector */

      regval = 0;
      sx127x_writeregbyte(dev, SX127X_FOM_PREDET, regval);
    }
  else
    {
      /* Configure preamble length */

      regval = SX127X_FOM_PRE_MSB(len);
      sx127x_writeregbyte(dev, SX127X_FOM_PREMSB, regval);
      regval = SX127X_FOM_PRE_LSB(len);
      sx127x_writeregbyte(dev, SX127X_FOM_PRELSB, regval);

      /* Configure preamble polarity to 0xAA */

      regval = SX127X_FOM_SYNCCFG_PREPOL;
      sx127x_modregbyte(dev, SX127X_FOM_SYNCCFG, regval, 0);

      /* Configure and enable preamble detector:
       *   - tolerance = 10
       *   - detector size = 2B
       */

      regval = (SX127X_FOM_PREDET_ON | SX127X_FOM_PREDET_SIZE_2B |
                  SX127X_FOM_PREDET_TOL(10));
      sx127x_writeregbyte(dev, SX127X_FOM_PREDET, regval);
    }

  /* Unlock SPI */

  sx127x_unlock(dev->spi);
}

/****************************************************************************
 * Name: sx127x_fskook_preamble_get
 *
 * Description:
 *   Get current preamble configuration for FSK/OOK
 *
 ****************************************************************************/

static int sx127x_fskook_preamble_get(FAR struct sx127x_dev_s *dev)
{
  wlerr("sx127x_fskook_preamble_get\n");
  return 0;
}

#endif  /* CONFIG_LPWAN_SX127X_FSKOOK */

#ifdef CONFIG_LPWAN_SX127X_LORA

/****************************************************************************
 * Name: sx127x_lora_opmode_init
 *
 * Description:
 *   Initialize operation mode for LORA
 *
 ****************************************************************************/

static int sx127x_lora_opmode_init(FAR struct sx127x_dev_s *dev,
                                   uint8_t opmode)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  uint8_t dio0map = 0;
  uint8_t setbits = 0;
  uint8_t clrbits = 0;
  int     ret     = OK;

  sx127x_lock(dev->spi);

  /* Get mode specific configuration */

  switch (opmode)
    {
      case SX127X_OPMODE_SLEEP:
      case SX127X_OPMODE_STANDBY:
      case SX127X_OPMODE_FSRX:
      case SX127X_OPMODE_FSTX:
        {
          break;
        }

      case SX127X_OPMODE_TX:
        {
          /* DIO0 is TX DONE */

          dio0map = SX127X_LRM_DIOMAP1_DIO0_TXDONE;

          /* Full buffer for TX */

          sx127x_writeregbyte(dev, SX127X_LRM_TXBASE, 0);

          /* Reset FIFO pointer */

          sx127x_writeregbyte(dev, SX127X_LRM_ADDRPTR, 0);

          break;
        }

      case SX127X_OPMODE_RX:
      case SX127X_OPMODE_RXSINGLE:
        {
          /* DIO0 is RX DONE */

          dio0map = SX127X_LRM_DIOMAP1_DIO0_RXDONE;

          /* Full buffer for RX */

          sx127x_writeregbyte(dev, SX127X_LRM_RXBASE, 0);

          /* Reset FIFO pointer */

          sx127x_writeregbyte(dev, SX127X_LRM_ADDRPTR, 0);

          break;
        }

      case SX127X_OPMODE_CAD:
        {
          /* DIO0 is CAD DONE */

          dio0map = SX127X_LRM_DIOMAP1_DIO0_CADDONE;

          break;
        }

      default:
        {
          wlerr("ERROR: invalid mode %d\n", opmode);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Configure DIO0 pin */

  setbits = dio0map;
  clrbits = SX127X_CMN_DIOMAP1_DIO0_MASK;
  sx127x_modregbyte(dev, SX127X_CMN_DIOMAP1, setbits, clrbits);

  sx127x_unlock(dev->spi);

errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_lora_opmode_set
 *
 * Description:
 *   Set operation mode for LORA
 *
 ****************************************************************************/

static int sx127x_lora_opmode_set(FAR struct sx127x_dev_s *dev,
                                  uint8_t opmode)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  int ret = OK;

  sx127x_lock(dev->spi);

  switch (opmode)
    {
      case SX127X_OPMODE_SLEEP:
      case SX127X_OPMODE_STANDBY:
      case SX127X_OPMODE_FSRX:
      case SX127X_OPMODE_FSTX:
      case SX127X_OPMODE_TX:
      case SX127X_OPMODE_RX:
      case SX127X_OPMODE_RXSINGLE:
      case SX127X_OPMODE_CAD:
        {
          /* Do nothing */

          break;
        }

      default:
        {
          wlerr("ERROR: invalid LORA mode %d\n", opmode);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Update mode */

  sx127x_modregbyte(dev, SX127X_CMN_OPMODE,
                    ((opmode-1) << SX127X_CMN_OPMODE_MODE_SHIFT),
                    SX127X_CMN_OPMODE_MODE_MASK);

  sx127x_unlock(dev->spi);

  /* Wait for mode ready. REVISIT: do we need this ? */

  usleep(250);

errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_lora_syncword_get
 ****************************************************************************/

static void sx127x_lora_syncword_get(FAR struct sx127x_dev_s *dev,
                                     FAR uint8_t *sw, FAR uint8_t *len)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_FSK ||
              dev->modulation == SX127X_MODULATION_OOK);

  wlerr("sx127x_lora_syncword_get not implemented yet\n");
}

/****************************************************************************
 * Name: sx127x_lora_syncword_set
 *
 * Description:
 *   Set SyncWord for LORA
 *
 ****************************************************************************/

static int sx127x_lora_syncword_set(FAR struct sx127x_dev_s *dev,
                                    FAR uint8_t *sw, uint8_t len)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  int ret = OK;

  if (len != 1)
    {
      wlerr("LORA support sync word with len = 1 but len = %d\n", len);
      ret = -EINVAL;
      goto errout;
    }

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Write sync word */

  sx127x_writeregbyte(dev, SX127X_LRM_SYNCWORD, sw[0]);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_lora_bw_set
 *
 * Description:
 *   Configure LORA bandwidth
 *
 ****************************************************************************/

static int sx127x_lora_bw_set(FAR struct sx127x_dev_s *dev, uint8_t bw)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  uint8_t clrbits = 0;
  uint8_t setbits = 0;
  int     ret     = OK;

  if (bw == dev->lora.bw)
    {
      goto errout;
    }

  switch (bw)
    {
      case LORA_BANDWIDTH_7p8kHz:
      case LORA_BANDWIDTH_10p4kHz:
      case LORA_BANDWIDTH_15p6kHz:
      case LORA_BANDWIDTH_20p8kHz:
      case LORA_BANDWIDTH_31p2kHz:
      case LORA_BANDWIDTH_41p4kHz:
      case LORA_BANDWIDTH_62p5kHz:
      case LORA_BANDWIDTH_125kHz:
      case LORA_BANDWIDTH_250kHz:
        {
          /* Lock SPI */

          sx127x_lock(dev->spi);

          setbits = bw << SX127X_LRM_MDMCFG1_BW_SHIFT;
          clrbits = SX127X_LRM_MDMCFG1_BW_MASK;
          sx127x_modregbyte(dev, SX127X_LRM_MDMCFG1, setbits, clrbits);

          /* Unlock SPI */

          sx127x_unlock(dev->spi);

          break;
        }

      default:
        {
          ret = -EINVAL;
          wlerr("Unsupported bandwidth %d\n", bw);
          goto errout;
        }
    }

  dev->lora.bw = bw;

errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_lora_cr_set
 *
 * Description:
 *   Configure LORA coding rate
 *
 ****************************************************************************/

static int sx127x_lora_cr_set(FAR struct sx127x_dev_s *dev, uint8_t cr)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  uint8_t clrbits = 0;
  uint8_t setbits = 0;
  int     ret     = OK;

  if (cr == dev->lora.cr)
    {
      goto errout;
    }

  switch (cr)
    {
      case LORA_CR_4d5:
      case LORA_CR_4d6:
      case LORA_CR_4d7:
      case LORA_CR_4d8:
        {
          /* Lock SPI */

          sx127x_lock(dev->spi);

          setbits = cr << SX127X_LRM_MDMCFG1_CDRATE_SHIFT;
          clrbits = SX127X_LRM_MDMCFG1_CDRATE_MASK;
          sx127x_modregbyte(dev, SX127X_LRM_MDMCFG1, setbits, clrbits);

          /* Unlock SPI */

          sx127x_unlock(dev->spi);

          break;
        }

      default:
        {
          ret = -EINVAL;
          wlerr("Unsupported code rate %d\n", cr);
          goto errout;
        }
    }

  dev->lora.cr = cr;

errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_lora_sf_set
 *
 * Description:
 *   Configure LORA SF
 *
 ****************************************************************************/

static int sx127x_lora_sf_set(FAR struct sx127x_dev_s *dev, uint8_t sf)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  uint8_t dopt    = SX127X_LRM_DETECTOPT_DO_SF7SF12;
  uint8_t dthr    = SX127X_LRM_DETECTTHR_SF7SF12;
  uint8_t setbits = 0;
  uint8_t clrbits = 0;
  int     ret     = OK;

  if (dev->lora.sf == sf)
    {
      goto errout;
    }

  /* Special configuration required by SF6 (highest data rate transmission):
   *   - implicit header mode ON
   *   - Detection optimize for SF6
   *   - Detection threshold for SF6
   */

  if (dev->lora.sf == 6)
    {
      if (dev->lora.implicthdr == true)
        {
          wlerr("SF6 needs implicit header ON!\n");
          ret = -EINVAL;
          goto errout;
        }

      dopt = SX127X_LRM_DETECTOPT_DO_SF6;
      dthr = SX127X_LRM_DETECTTHR_SF6;
    }

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Write spreading factor */

  clrbits = SX127X_LRM_MDMCFG2_SPRFACT_MASK;
  setbits = (sf << SX127X_LRM_MDMCFG2_SPRFACT_SHIFT);
  sx127x_modregbyte(dev, SX127X_LRM_MDMCFG2, setbits, clrbits);

  sx127x_writeregbyte(dev, SX127X_LRM_DETECTOPT, dopt);
  sx127x_writeregbyte(dev, SX127X_LRM_DETECTTHR, dthr);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Update local variable */

  dev->lora.sf = sf;

errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_lora_implicthdr_set
 *
 * Description:
 *  Enable/disable implicit header for LORA
 *
 ****************************************************************************/

static int sx127x_lora_implicthdr_set(FAR struct sx127x_dev_s *dev,
                                      bool enable)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  uint8_t setbits = 0;
  uint8_t clrbits = 0;
  int     ret     = OK;

  if (dev->lora.sf == 6 && enable == false)
    {
      wlerr("SF=6 requires implicit header ON\n");
      ret = -EINVAL;
      goto errout;
    }

  if (enable == dev->lora.implicthdr)
    {
      goto errout;
    }

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Modify MDMCFG1 register */

  clrbits = 0;
  setbits = SX127X_LRM_MDMCFG1_IMPLHDRON;

  sx127x_modregbyte(dev, SX127X_LRM_MDMCFG1, setbits, clrbits);

  sx127x_modregbyte(dev, SX127X_LRM_HOPCHAN, setbits, clrbits);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Update local variable */

  dev->lora.implicthdr = enable;

errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_lora_init
 *
 * Description:
 *   Initialization specific for LORA modulation
 *
 ****************************************************************************/

static void sx127x_lora_init(FAR struct sx127x_dev_s *dev)
{
  DEBUGASSERT(dev->modulation == SX127X_MODULATION_LORA);

  uint8_t setbits = 0;
  uint8_t clrbits = 0;

  /* Configure sync word for LORA modulation */

  setbits = SX127X_LRM_SYNCWORD_DEFAULT;
  sx127x_lora_syncword_set(dev, &setbits, 1);

  /* Configure bandwidth */

  sx127x_lora_bw_set(dev, SX127X_LRM_BW_DEFAULT);

  /* Configure coding rate */

  sx127x_lora_cr_set(dev, SX127X_LRM_CR_DEFAULT);

  /* TODO: Configure frequency hopping */

  /* sx127x_lora_fhop_set(dev,) */

  /* Configure spreading factor */

  sx127x_lora_sf_set(dev, SX127X_LRM_SF_DEFAULT);

  /* Configure LORA header */

  sx127x_lora_implicthdr_set(dev, SX127X_LRM_IMPLICTHDR_DEFAULT);

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Configure maximum payload */

  sx127x_writeregbyte(dev, SX127X_LRM_PAYLOADMAX,
                      SX127X_LRM_PAYLOADMAX_DEFAULT);

  /* Modem PHY config 2:
   *   - TX crc ON
   *   - packet mode
   */

  setbits = (dev->crcon == true ? SX127X_LRM_MDMCFG2_RXCRCON : 0);
  clrbits = SX127X_LRM_MDMCFG2_TXCONT;
  sx127x_modregbyte(dev, SX127X_LRM_MDMCFG2, setbits, clrbits);

  /* Invert I and Q signals if configured */

  setbits = (dev->lora.invert_iq == true ? SX127X_LRM_INVERTIQ_IIQ : 0);
  clrbits = 0;
  sx127x_modregbyte(dev, SX127X_LRM_INVERTIQ, setbits, clrbits);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);
}

/****************************************************************************
 * Name: sx127x_lora_rssi_correct
 *
 * Description:
 *   Correct RSSI for LORA radio according to datasheet
 *
 ****************************************************************************/

static int16_t sx127x_lora_rssi_correct(FAR struct sx127x_dev_s *dev,
                                        uint32_t freq, int8_t snr,
                                        uint8_t regval)
{
  int16_t offset = 0;
  int16_t ret    = 0;

  /* Ignore SNR if >= 0 */

  if (snr >= 0)
    {
      snr = 0;
    }

  /* RSSI offset depends on RF frequency */

  offset = (freq > SX127X_HFBAND_THR ?
            SX127X_LRM_RSSIVAL_HF_OFFSET : SX127X_LRM_RSSIVAL_LF_OFFSET);

  /* Get corrected RSSI value */

  ret = regval + offset + snr;

  return ret;
}

/****************************************************************************
 * Name: sx127x_lora_snr_get
 *
 * Description:
 *   Get estimation of SNR on last packet received for LORA modem
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
static int8_t sx127x_lora_snr_get(FAR struct sx127x_dev_s *dev)
{
  uint8_t regval = 0;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Get register value */

  regval = sx127x_readregbyte(dev, SX127X_LRM_PKTSNR);

  /* Get SNR */

  regval = regval / 4;

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Return corrected RSSI */

  return (int8_t)regval;
}

/****************************************************************************
 * Name: sx127x_lora_pckrssi_get
 *
 * Description:
 *   Get RSSI of the last received LORA packet
 *
 ****************************************************************************/

static int16_t sx127x_lora_pckrssi_get(FAR struct sx127x_dev_s *dev,
                                       int8_t snr)
{
  uint8_t regval = 0;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Get register value */

  regval = sx127x_readregbyte(dev, SX127X_LRM_PKTRSSI);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Return corrected RSSI */

  return sx127x_lora_rssi_correct(dev, dev->freq, snr, regval);
}
#endif

/****************************************************************************
 * Name: sx127x_lora_rssi_get
 *
 * Description:
 *   Get current RSSI for LORA modem
 *
 ****************************************************************************/

static int16_t sx127x_lora_rssi_get(FAR struct sx127x_dev_s *dev)
{
  uint8_t regval = 0;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Get register value */

  regval = sx127x_readregbyte(dev, SX127X_LRM_RSSIVAL);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Return corrected RSSI */

  return sx127x_lora_rssi_correct(dev, dev->freq, 0, regval);
}

/****************************************************************************
 * Name: sx127x_lora_preamble_set
 *
 * Description:
 *   Set preamble for LORA modulation
 *
 ****************************************************************************/

static void sx127x_lora_preamble_set(FAR struct sx127x_dev_s *dev,
                                     uint32_t len)
{
  uint8_t regval = 0;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Configure preamble len */

  regval = SX127X_LRM_PRE_MSB(len);
  sx127x_writeregbyte(dev, SX127X_LRM_PREMSB, regval);
  regval = SX127X_LRM_PRE_LSB(len);
  sx127x_writeregbyte(dev, SX127X_LRM_PRELSB, regval);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);
}

/****************************************************************************
 * Name: sx127x_lora_preamble_get
 *
 * Description:
 *
 ****************************************************************************/

static int sx127x_lora_preamble_get(FAR struct sx127x_dev_s *dev)
{
  wlerr("sx127x_lora_preamble_get\n");
  return 0;
}

#endif  /* CONFIG_LPWAN_SX127X_LORA */

/****************************************************************************
 * Name: sx127x_modulation_get
 *
 * Description:
 *   Get current radio modulation
 *
 ****************************************************************************/

static uint8_t sx127x_modulation_get(FAR struct sx127x_dev_s *dev)
{
  uint8_t regval = 0;
  uint8_t ret    = 0;

  /* Get OPMODE register */

  regval = sx127x_readregbyte(dev, SX127X_CMN_OPMODE);

  if (regval & SX127X_CMN_OPMODE_LRMODE)
    {
      /* LORA modulation */

      ret = SX127X_MODULATION_LORA;
    }
  else
    {
      /* FSK or OOK modulation */

      ret = (regval & SX127X_CMN_OPMODE_MODTYPE_FSK ?
             SX127X_MODULATION_FSK : SX127X_MODULATION_OOK);
    }

  return ret;
}

/****************************************************************************
 * Name: sx127x_ops_set
 ****************************************************************************/

static void sx127x_ops_set(FAR struct sx127x_dev_s *dev, uint8_t modulation)
{
#ifdef CONFIG_LPWAN_SX127X_FSKOOK
  if (modulation <= SX127X_MODULATION_OOK)
    {
      dev->ops.init         = sx127x_fskook_init;
      dev->ops.isr0_process = sx127x_fskook_isr0_process;
      dev->ops.opmode_init  = sx127x_fskook_opmode_init;
      dev->ops.opmode_set   = sx127x_fskook_opmode_set;
      dev->ops.preamble_set = sx127x_fskook_preamble_set;
      dev->ops.preamble_get = sx127x_fskook_preamble_get;
      dev->ops.rssi_get     = sx127x_fskook_rssi_get;
      dev->ops.syncword_set = sx127x_fskook_syncword_set;
      dev->ops.syncword_get = sx127x_fskook_syncword_get;
#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
      dev->ops.send         = sx127x_fskook_send;
#endif
#ifdef CONFIG_DEBUG_WIRELESS_INFO
      dev->ops.dumpregs     = sx127x_fskook_dumpregs;
#endif
    }
#endif
#ifdef CONFIG_LPWAN_SX127X_LORA
  if (modulation == SX127X_MODULATION_LORA)
    {
      dev->ops.init         = sx127x_lora_init;
      dev->ops.isr0_process = sx127x_lora_isr0_process;
      dev->ops.opmode_init  = sx127x_lora_opmode_init;
      dev->ops.opmode_set   = sx127x_lora_opmode_set;
      dev->ops.preamble_set = sx127x_lora_preamble_set;
      dev->ops.preamble_get = sx127x_lora_preamble_get;
      dev->ops.rssi_get     = sx127x_lora_rssi_get;
      dev->ops.syncword_set = sx127x_lora_syncword_set;
      dev->ops.syncword_get = sx127x_lora_syncword_get;
#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
      dev->ops.send         = sx127x_lora_send;
#endif
#ifdef CONFIG_DEBUG_WIRELESS_INFO
      dev->ops.dumpregs     = sx127x_lora_dumpregs;
#endif
    }
#endif
}

/****************************************************************************
 * Name: sx127x_modulation_set
 *
 * Description:
 *   Set radio modulation and configure
 *
 ****************************************************************************/

static int sx127x_modulation_set(FAR struct sx127x_dev_s *dev,
                                 uint8_t modulation)
{
  uint8_t setbits = 0;
  uint8_t clrbits = 0;
  int     ret     = OK;

  wlinfo("modulation_set %d->%d\n", dev->modulation, modulation);

  if (modulation == dev->modulation)
    {
      goto errout;
    }

  /* Modulation can be only changed in SLEEP mode */

  sx127x_opmode_set(dev, SX127X_OPMODE_SLEEP);

  /* Change modulation */

  switch (modulation)
    {
#ifdef CONFIG_LPWAN_SX127X_FSKOOK
      case SX127X_MODULATION_FSK:
        {
          clrbits = (SX127X_CMN_OPMODE_LRMODE |
                     SX127X_CMN_OPMODE_MODTYPE_MASK);
          setbits = SX127X_CMN_OPMODE_MODTYPE_FSK;

          break;
        }

      case SX127X_MODULATION_OOK:
        {
          clrbits = (SX127X_CMN_OPMODE_LRMODE |
                     SX127X_CMN_OPMODE_MODTYPE_MASK);
          setbits = SX127X_CMN_OPMODE_MODTYPE_OOK;

          break;
        }
#endif /* CONFIG_LPWAN_SX127X_FSKOOK */

#ifdef CONFIG_LPWAN_SX127X_LORA
      case SX127X_MODULATION_LORA:
        {
          clrbits = SX127X_CMN_OPMODE_MODTYPE_MASK;
          setbits = SX127X_CMN_OPMODE_LRMODE;

          break;
        }
#endif /* CONFIG_LPWAN_SX127X_LORA */

      default:
        {
          wlerr("ERROR: Unsupported modulation type %d\n", modulation);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Modify register */

  sx127x_modregbyte(dev, SX127X_CMN_OPMODE, setbits, clrbits);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Initialization specific for modulation and initialize private ops */

  sx127x_ops_set(dev, modulation);

  /* Update local variable */

  dev->modulation = modulation;

  /* Initial configuration */

  dev->ops.init(dev);

errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_fskook_rssi_get
 ****************************************************************************/

static int16_t sx127x_rssi_get(FAR struct sx127x_dev_s *dev)
{
  return dev->ops.rssi_get(dev);
}

/****************************************************************************
 * Name: sx127x_channel_scan
 *
 * Description:
 *
 ****************************************************************************/

static bool sx127x_channel_scan(FAR struct sx127x_dev_s *dev,
                                FAR struct sx127x_chanscan_ioc_s *chanscan)
{
  struct timespec tstart;
  struct timespec tnow;
  bool    ret  = true;
  int16_t rssi = 0;
  int16_t max = 0;
  int16_t min = 0;

  /* Set frequency */

  sx127x_frequency_set(dev, chanscan->freq);

  /* Set mode to RX */

  dev->ops.opmode_set(dev, SX127X_OPMODE_RX);

  /* Get start time */

  clock_gettime(CLOCK_REALTIME, &tstart);

  /* Initialize min/max */

  max = INT16_MIN;
  min = INT16_MAX;

  do
    {
      /* Get time now */

      clock_gettime(CLOCK_REALTIME, &tnow);

      /* Check RSSI */

      rssi = dev->ops.rssi_get(dev);

      /* Store maximum/minimum value */

      if (rssi > max)
        {
          max = rssi;
        }
      else if (rssi < min)
        {
          min = rssi;
        }

      if (rssi > chanscan->rssi_thr)
        {
          ret = false;
          break;
        }

    }
  while (tstart.tv_sec + chanscan->stime > tnow.tv_sec);

  /* Store limit values */

  chanscan->rssi_max = max;
  chanscan->rssi_min = min;

  /* Store return value in struct */

  chanscan->free = ret;

  return ret;
}

/****************************************************************************
 * Name: sx127x_random_get
 *
 * Description:
 *
 ****************************************************************************/

static uint32_t sx127x_random_get(FAR struct sx127x_dev_s *dev)
{
  wlerr("sx127x_random_get not implemented yet\n");
  return 0;
}

/****************************************************************************
 * Name: sx127x_frequency_get
 *
 * Description:
 *   Get RF carrier frequency
 *
 ****************************************************************************/

static uint32_t sx127x_frequency_get(FAR struct sx127x_dev_s *dev)
{
  wlerr("sx127x_frequency_get\n");
  return 0;
}

/****************************************************************************
 * Name: sx127x_frequency_set
 *
 * Description:
 *   Set RF carrier frequency for LORA and FSK/OOK modulation
 *
 ****************************************************************************/

static int sx127x_frequency_set(FAR struct sx127x_dev_s *dev, uint32_t freq)
{
  uint32_t frf = 0;
  int      ret = OK;

  if (freq == dev->freq)
    {
      goto errout;
    }

  /* REVISIT: needs sleep/standby mode ? */

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Get FRF value */

  frf = SX127X_FRF_FROM_FREQ(freq);

  /* Write FRF MSB */

  sx127x_writeregbyte(dev, SX127X_CMN_FRFMSB, SX127X_CMN_FRF_MSB(frf));

  /* Write FRF MID */

  sx127x_writeregbyte(dev, SX127X_CMN_FRFMID, SX127X_CMN_FRF_MID(frf));

  /* Write FRF LSB */

  sx127x_writeregbyte(dev, SX127X_CMN_FRFLSB, SX127X_CMN_FRF_LSB(frf));

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  /* Update local variable */

  dev->freq = freq;

errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_preamble_set
 ****************************************************************************/

static void sx127x_preamble_set(FAR struct sx127x_dev_s *dev, uint32_t len)
{
  dev->ops.preamble_set(dev, len);
}

/****************************************************************************
 * Name: sx127x_preamble_get
 ****************************************************************************/

static int sx127x_preamble_get(FAR struct sx127x_dev_s *dev)
{
  return dev->ops.preamble_get(dev);
}

/****************************************************************************
 * Name: sx127x_version_get
 *
 * Description:
 *   Get chip version
 *
 ****************************************************************************/

static uint8_t sx127x_version_get(FAR struct sx127x_dev_s *dev)
{
  uint8_t regval = 0;

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Get version */

  regval = sx127x_readregbyte(dev, SX127X_CMN_VERSION);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  return regval;
}

/****************************************************************************
 * Name: sx127x_calibration
 *
 * Description:
 *   Calibrate radio for given frequency
 *
 ****************************************************************************/

static int sx127x_calibration(FAR struct sx127x_dev_s *dev, uint32_t freq)
{
  uint8_t regval = 0;
  int     ret    = OK;

  /* NOTE: The automatic calibration at POR and Reset is only valid at
   * 434 MHz.
   */

  wlinfo("SX127X calibration for %d\n", freq);

  /* Calibration is supported only in FSK/OOK mode */

  ret = sx127x_modulation_set(dev, SX127X_MODULATION_FSK);
  if (ret < 0)
    {
      wlerr("ERROR: can't change modulation to FSK \n");
      goto errout;
    }

  /* We need standby mode ? */

  ret = sx127x_opmode_set(dev, SX127X_OPMODE_STANDBY);

  /* Set calibration frequency */

  sx127x_frequency_set(dev, freq);

  /* Lock SPI */

  sx127x_lock(dev->spi);

  /* Start calibration */

  sx127x_modregbyte(dev, SX127X_FOM_IMAGECAL,
                    SX127X_FOM_IMAGECAL_IMGCALSTART, 0);

  /* Wait for calibration done */

  do
    {
      /* Wait 10ms */

      usleep(10000);

      /* Get register */

      regval = sx127x_readregbyte(dev, SX127X_FOM_IMAGECAL);
    }
  while (regval & SX127X_FOM_IMAGECAL_IMGCALRUN);

  /* Unlock SPI */

  sx127x_unlock(dev->spi);

  wlinfo("Calibration done\n");

errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_init
 *
 * Description:
 *   Initialize SX127X chip
 *
 ****************************************************************************/

static int sx127x_init(FAR struct sx127x_dev_s *dev)
{
  int     ret     = OK;
  uint8_t regval  = 0;

  wlinfo("Init sx127x dev\n");

  /* Reset radio */

  sx127x_reset(dev);

  /* Get initial modem state */

  regval = sx127x_readregbyte(dev, SX127X_CMN_OPMODE);

  dev->opmode = (((regval >> SX127X_CMN_OPMODE_MODE_SHIFT) &
                  SX127X_CMN_OPMODE_MODE_MASK) + 1);
  dev->modulation = ((regval & SX127X_CMN_OPMODE_LRMODE) ?
                     SX127X_MODULATION_LORA :
                     (regval & SX127X_CMN_OPMODE_MODTYPE_OOK ?
                      SX127X_MODULATION_OOK :
                      SX127X_MODULATION_FSK));

  wlinfo("Init state: modulation=%d, opmode=%d\n",
         dev->modulation, dev->opmode);

  /* Set ops */

  sx127x_ops_set(dev, dev->modulation);

  /* Get chip version */

  regval = sx127x_version_get(dev);
  if (regval == 0x00)
    {
      /* Probably sth wrong with communication */

      wlerr("ERROR: failed to get chip version!\n");
      ret = -ENODATA;
      goto errout;
    }

  wlinfo("SX127X version = 0x%02x\n", regval);

  /* Calibration */

  ret = sx127x_calibration(dev, SX127X_FREQ_CALIBRATION);
  if (ret < 0)
    {
      wlerr("ERROR: calibration failed \n");
    }

  /* Set default modulation and configure */

  sx127x_modulation_set(dev, SX127X_MODULATION_DEFAULT);

  /* Enter SLEEP mode */

  sx127x_opmode_set(dev, SX127X_OPMODE_SLEEP);

  /* Set channel frequency */

  sx127x_frequency_set(dev, SX127X_FREQ_RF_DEFAULT);

  /* Configure preamble */

  sx127x_preamble_set(dev, SX127X_PREAMBLE_LEN_DEFAULT);

  /* TODO: Configure RF output */

  /* TODO: Configure RF input */

  wlinfo("Init sx127x dev - DONE\n");

errout:
  return ret;
}

/****************************************************************************
 * Name: sx127x_deinit
 *
 * Description:
 *   Deinitialize SX127X chip
 *
 ****************************************************************************/

static int sx127x_deinit(FAR struct sx127x_dev_s *dev)
{
  wlinfo("Deinit sx127x dev\n");

  /* Enter SLEEP mode */

  sx127x_opmode_set(dev, SX127X_OPMODE_SLEEP);

  /* Reset radio */

  sx127x_reset(dev);

  return OK;
}

#ifdef CONFIG_DEBUG_WIRELESS_INFO

/****************************************************************************
 * Name: sx127x_lora_dumpregs
 *
 * Description:
 *   Dump registers for LORA modem
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_LORA
static void sx127x_lora_dumpregs(FAR struct sx127x_dev_s *dev)
{
  sx127x_lock(dev->spi);
  wlinfo("LORA dump:\n");
  wlinfo("FIFO:         %02x\n", sx127x_readregbyte(dev, SX127X_CMN_FIFO));
  wlinfo("OPMODE:       %02x\n", sx127x_readregbyte(dev, SX127X_CMN_OPMODE));
  wlinfo("FRFMSB:       %02x\n", sx127x_readregbyte(dev, SX127X_CMN_FRFMSB));
  wlinfo("FRFMID:       %02x\n", sx127x_readregbyte(dev, SX127X_CMN_FRFMID));
  wlinfo("FRFLSB:       %02x\n", sx127x_readregbyte(dev, SX127X_CMN_FRFLSB));
  wlinfo("PACFG:        %02x\n", sx127x_readregbyte(dev, SX127X_CMN_PACFG));
  wlinfo("PARAMP:       %02x\n", sx127x_readregbyte(dev, SX127X_CMN_PARAMP));
  wlinfo("OCP:          %02x\n", sx127x_readregbyte(dev, SX127X_CMN_OCP));
  wlinfo("LNA:          %02x\n", sx127x_readregbyte(dev, SX127X_CMN_LNA));
  wlinfo("ADDRPTR:      %02x\n", sx127x_readregbyte(dev, SX127X_LRM_ADDRPTR));
  wlinfo("TXBASE:       %02x\n", sx127x_readregbyte(dev, SX127X_LRM_TXBASE));
  wlinfo("RXBASE:       %02x\n", sx127x_readregbyte(dev, SX127X_LRM_RXBASE));
  wlinfo("RXCURR:       %02x\n", sx127x_readregbyte(dev, SX127X_LRM_RXCURR));
  wlinfo("IRQMASK:      %02x\n", sx127x_readregbyte(dev, SX127X_LRM_IRQMASK));
  wlinfo("IRQ:          %02x\n", sx127x_readregbyte(dev, SX127X_LRM_IRQ));
  wlinfo("RXBYTES:      %02x\n", sx127x_readregbyte(dev, SX127X_LRM_RXBYTES));
  wlinfo("RXHDRMSB:     %02x\n", sx127x_readregbyte(dev, SX127X_LRM_RXHDRMSB));
  wlinfo("RXHDRLSB:     %02x\n", sx127x_readregbyte(dev, SX127X_LRM_RXHDRLSB));
  wlinfo("RXPKTMSB:     %02x\n", sx127x_readregbyte(dev, SX127X_LRM_RXPKTMSB));
  wlinfo("RXPKTLSB:     %02x\n", sx127x_readregbyte(dev, SX127X_LRM_RXPKTLSB));
  wlinfo("MODSTAT:      %02x\n", sx127x_readregbyte(dev, SX127X_LRM_MODSTAT));
  wlinfo("PKTSNR:       %02x\n", sx127x_readregbyte(dev, SX127X_LRM_PKTSNR));
  wlinfo("PKTRSSI:      %02x\n", sx127x_readregbyte(dev, SX127X_LRM_PKTRSSI));
  wlinfo("RSSI:         %02x\n", sx127x_readregbyte(dev, SX127X_LRM_RSSIVAL));
  wlinfo("HOPCHAN:      %02x\n", sx127x_readregbyte(dev, SX127X_LRM_HOPCHAN));
  wlinfo("MDMCFG1:      %02x\n", sx127x_readregbyte(dev, SX127X_LRM_MDMCFG1));
  wlinfo("MDMCFG2:      %02x\n", sx127x_readregbyte(dev, SX127X_LRM_MDMCFG2));
  wlinfo("RXTIMEOUTLSB: %02x\n", sx127x_readregbyte(dev, SX127X_LRM_RXTIMEOUTLSB));
  wlinfo("PREMSB:       %02x\n", sx127x_readregbyte(dev, SX127X_LRM_PREMSB));
  wlinfo("PRELSB:       %02x\n", sx127x_readregbyte(dev, SX127X_LRM_PRELSB));
  wlinfo("PAYLOADLEN:   %02x\n", sx127x_readregbyte(dev, SX127X_LRM_PAYLOADLEN));
  wlinfo("PAYLOADMAX:   %02x\n", sx127x_readregbyte(dev, SX127X_LRM_PAYLOADMAX));
  wlinfo("HOPPER:       %02x\n", sx127x_readregbyte(dev, SX127X_LRM_HOPPER));
  wlinfo("RXFIFOADDR:   %02x\n", sx127x_readregbyte(dev, SX127X_LRM_RXFIFOADDR));
  wlinfo("MODEMCFG3:    %02x\n", sx127x_readregbyte(dev, SX127X_LRM_MODEMCFG3));
  wlinfo("FEIMSB:       %02x\n", sx127x_readregbyte(dev, SX127X_LRM_FEIMSB));
  wlinfo("FEIMID:       %02x\n", sx127x_readregbyte(dev, SX127X_LRM_FEIMID));
  wlinfo("FEILSB:       %02x\n", sx127x_readregbyte(dev, SX127X_LRM_FEILSB));
  wlinfo("RSSIWIDEBAND: %02x\n", sx127x_readregbyte(dev, SX127X_LRM_RSSIWIDEBAND));
  wlinfo("DETECTOPT:    %02x\n", sx127x_readregbyte(dev, SX127X_LRM_DETECTOPT));
  wlinfo("INVERTIQ:     %02x\n", sx127x_readregbyte(dev, SX127X_LRM_INVERTIQ));
  wlinfo("DETECTTHR:    %02x\n", sx127x_readregbyte(dev, SX127X_LRM_DETECTTHR));
  wlinfo("SYNCWORD:     %02x\n", sx127x_readregbyte(dev, SX127X_LRM_SYNCWORD));
  wlinfo("DIOMAP1:      %02x\n", sx127x_readregbyte(dev, SX127X_CMN_DIOMAP1));
  wlinfo("DIOMAP2:      %02x\n", sx127x_readregbyte(dev, SX127X_CMN_DIOMAP2));
  wlinfo("VERSION:      %02x\n", sx127x_readregbyte(dev, SX127X_CMN_VERSION));
  wlinfo("TCXO:         %02x\n", sx127x_readregbyte(dev, SX127X_CMN_TCXO));
  wlinfo("PADAC:        %02x\n", sx127x_readregbyte(dev, SX127X_CMN_PADAC));
  wlinfo("FTEMP:        %02x\n", sx127x_readregbyte(dev, SX127X_CMN_FTEMP));
  wlinfo("AGCREF:       %02x\n", sx127x_readregbyte(dev, SX127X_CMN_AGCREF));
  wlinfo("AGCTHR1:      %02x\n", sx127x_readregbyte(dev, SX127X_CMN_AGCTHR1));
  wlinfo("AGCTHR2:      %02x\n", sx127x_readregbyte(dev, SX127X_CMN_AGCTHR2));
  wlinfo("AGCTHR3:      %02x\n", sx127x_readregbyte(dev, SX127X_CMN_AGCTHR3));
  wlinfo("PLL:          %02x\n", sx127x_readregbyte(dev, SX127X_CMN_PLL));
  sx127x_unlock(dev->spi);
}
#endif  /* CONFIG_LPWAN_SX127X_LORA */

/****************************************************************************
 * Name: sx127x_fskook_dumpregs
 *
 * Description:
 *   Dump registers for FSK/OOK modem
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_FSKOOK
static void sx127x_fskook_dumpregs(FAR struct sx127x_dev_s *dev)
{
  sx127x_lock(dev->spi);
  wlinfo("FSK/OOK dump:\n");
  wlinfo("FIFO:         %02x\n", sx127x_readregbyte(dev, SX127X_CMN_FIFO));
  wlinfo("OPMODE:       %02x\n", sx127x_readregbyte(dev, SX127X_CMN_OPMODE));
  wlinfo("FRFMSB:       %02x\n", sx127x_readregbyte(dev, SX127X_CMN_FRFMSB));
  wlinfo("FRFMID:       %02x\n", sx127x_readregbyte(dev, SX127X_CMN_FRFMID));
  wlinfo("FRFLSB:       %02x\n", sx127x_readregbyte(dev, SX127X_CMN_FRFLSB));
  wlinfo("PACFG:        %02x\n", sx127x_readregbyte(dev, SX127X_CMN_PACFG));
  wlinfo("PARAMP:       %02x\n", sx127x_readregbyte(dev, SX127X_CMN_PARAMP));
  wlinfo("OCP:          %02x\n", sx127x_readregbyte(dev, SX127X_CMN_OCP));
  wlinfo("LNA:          %02x\n", sx127x_readregbyte(dev, SX127X_CMN_LNA));
  wlinfo("BITRATEMSB:   %02x\n", sx127x_readregbyte(dev, SX127X_FOM_BITRATEMSB));
  wlinfo("BITRATELSM:   %02x\n", sx127x_readregbyte(dev, SX127X_FOM_BITRATELSB));
  wlinfo("FDEVMSB:      %02x\n", sx127x_readregbyte(dev, SX127X_FOM_FDEVMSB));
  wlinfo("FDEVLSB:      %02x\n", sx127x_readregbyte(dev, SX127X_FOM_FDEVLSB));
  wlinfo("RXCFG:        %02x\n", sx127x_readregbyte(dev, SX127X_FOM_RXCFG));
  wlinfo("RSSICFG:      %02x\n", sx127x_readregbyte(dev, SX127X_FOM_RSSICFG));
  wlinfo("RSSICOLL:     %02x\n", sx127x_readregbyte(dev, SX127X_FOM_RSSICOLL));
  wlinfo("RSSITHR:      %02x\n", sx127x_readregbyte(dev, SX127X_FOM_RSSITHR));
  wlinfo("RSSIVAL:      %02x\n", sx127x_readregbyte(dev, SX127X_FOM_RSSIVAL));
  wlinfo("RXBW:         %02x\n", sx127x_readregbyte(dev, SX127X_FOM_RXBW));
  wlinfo("AFCBW:        %02x\n", sx127x_readregbyte(dev, SX127X_FOM_AFCBW));
  wlinfo("OOKPEAK:      %02x\n", sx127x_readregbyte(dev, SX127X_FOM_OOKPEAK));
  wlinfo("OOKFIX:       %02x\n", sx127x_readregbyte(dev, SX127X_FOM_OOKFIX));
  wlinfo("AFCFEI:       %02x\n", sx127x_readregbyte(dev, SX127X_FOM_AFCFEI));
  wlinfo("AFCMSB:       %02x\n", sx127x_readregbyte(dev, SX127X_FOM_AFCMSB));
  wlinfo("AFCLSB:       %02x\n", sx127x_readregbyte(dev, SX127X_FOM_AFCLSB));
  wlinfo("FEIMSB:       %02x\n", sx127x_readregbyte(dev, SX127X_FOM_FEIMSB));
  wlinfo("FEILSB:       %02x\n", sx127x_readregbyte(dev, SX127X_FOM_FEILSB));
  wlinfo("PREDET:       %02x\n", sx127x_readregbyte(dev, SX127X_FOM_PREDET));
  wlinfo("RXTIMEOUT1:   %02x\n", sx127x_readregbyte(dev, SX127X_FOM_RXTIMEOUT1));
  wlinfo("RXTIMEOUT2:   %02x\n", sx127x_readregbyte(dev, SX127X_FOM_RXTIMEOUT1));
  wlinfo("RXTIMEOUT3:   %02x\n", sx127x_readregbyte(dev, SX127X_FOM_RXTIMEOUT1));
  wlinfo("RXDELAY:      %02x\n", sx127x_readregbyte(dev, SX127X_FOM_RXDELAY));
  wlinfo("OSC:          %02x\n", sx127x_readregbyte(dev, SX127X_FOM_OSC));
  wlinfo("PREMSB:       %02x\n", sx127x_readregbyte(dev, SX127X_FOM_PREMSB));
  wlinfo("PRELSB:       %02x\n", sx127x_readregbyte(dev, SX127X_FOM_PRELSB));
  wlinfo("SYNCCFG:      %02x\n", sx127x_readregbyte(dev, SX127X_FOM_SYNCCFG));
  wlinfo("SYNCVAL1:     %02x\n", sx127x_readregbyte(dev, SX127X_FOM_SYNCVAL1));
  wlinfo("SYNCVAL2:     %02x\n", sx127x_readregbyte(dev, SX127X_FOM_SYNCVAL2));
  wlinfo("SYNCVAL3:     %02x\n", sx127x_readregbyte(dev, SX127X_FOM_SYNCVAL3));
  wlinfo("SYNCVAL4:     %02x\n", sx127x_readregbyte(dev, SX127X_FOM_SYNCVAL4));
  wlinfo("SYNCVAL5:     %02x\n", sx127x_readregbyte(dev, SX127X_FOM_SYNCVAL5));
  wlinfo("PKTCFG1:      %02x\n", sx127x_readregbyte(dev, SX127X_FOM_PKTCFG1));
  wlinfo("PKTCFG2:      %02x\n", sx127x_readregbyte(dev, SX127X_FOM_PKTCFG2));
  wlinfo("PAYLOADLEN:   %02x\n", sx127x_readregbyte(dev, SX127X_FOM_PAYLOADLEN));
  wlinfo("NODEADDR:     %02x\n", sx127x_readregbyte(dev, SX127X_FOM_NODEADDR));
  wlinfo("BROADCAST:    %02x\n", sx127x_readregbyte(dev, SX127X_FOM_BROADCAST));
  wlinfo("FIFOTHR:      %02x\n", sx127x_readregbyte(dev, SX127X_FOM_FIFOTHR));
  wlinfo("SEQCFG1:      %02x\n", sx127x_readregbyte(dev, SX127X_FOM_SEQCFG1));
  wlinfo("SEQCFG2:      %02x\n", sx127x_readregbyte(dev, SX127X_FOM_SEQCFG2));
  wlinfo("TIMRES:       %02x\n", sx127x_readregbyte(dev, SX127X_FOM_TIMRES));
  wlinfo("TIMER1COEF:   %02x\n", sx127x_readregbyte(dev, SX127X_FOM_TIMER1COEF));
  wlinfo("TIMER2COEF:   %02x\n", sx127x_readregbyte(dev, SX127X_FOM_TIMER2COEF));
  wlinfo("IMAGECAL:     %02x\n", sx127x_readregbyte(dev, SX127X_FOM_IMAGECAL));
  wlinfo("TEMP:         %02x\n", sx127x_readregbyte(dev, SX127X_FOM_TEMP));
  wlinfo("LOWBAT:       %02x\n", sx127x_readregbyte(dev, SX127X_FOM_LOWBAT));
  wlinfo("IRQ1:         %02x\n", sx127x_readregbyte(dev, SX127X_FOM_IRQ1));
  wlinfo("IRQ2:         %02x\n", sx127x_readregbyte(dev, SX127X_FOM_IRQ2));
  wlinfo("PLLHOP:       %02x\n", sx127x_readregbyte(dev, SX127X_FOM_PLLHOP));
  wlinfo("BITRATEFRAC:  %02x\n", sx127x_readregbyte(dev, SX127X_FOM_BITRATEFRAC));
  wlinfo("DIOMAP1:      %02x\n", sx127x_readregbyte(dev, SX127X_CMN_DIOMAP1));
  wlinfo("DIOMAP2:      %02x\n", sx127x_readregbyte(dev, SX127X_CMN_DIOMAP2));
  wlinfo("VERSION:      %02x\n", sx127x_readregbyte(dev, SX127X_CMN_VERSION));
  wlinfo("TCXO:         %02x\n", sx127x_readregbyte(dev, SX127X_CMN_TCXO));
  wlinfo("PADAC:        %02x\n", sx127x_readregbyte(dev, SX127X_CMN_PADAC));
  wlinfo("FTEMP:        %02x\n", sx127x_readregbyte(dev, SX127X_CMN_FTEMP));
  wlinfo("AGCREF:       %02x\n", sx127x_readregbyte(dev, SX127X_CMN_AGCREF));
  wlinfo("AGCTHR1:      %02x\n", sx127x_readregbyte(dev, SX127X_CMN_AGCTHR1));
  wlinfo("AGCTHR2:      %02x\n", sx127x_readregbyte(dev, SX127X_CMN_AGCTHR2));
  wlinfo("AGCTHR3:      %02x\n", sx127x_readregbyte(dev, SX127X_CMN_AGCTHR3));
  wlinfo("PLL:          %02x\n", sx127x_readregbyte(dev, SX127X_CMN_PLL));
  sx127x_unlock(dev->spi);
}
#endif  /* CONFIG_LPWAN_SX127X_FSKOOK */

/****************************************************************************
 * Name: sx127x_dumpregs
 *
 * Description:
 *   Dump registers according to current modulation
 *
 ****************************************************************************/

static void sx127x_dumpregs(FAR struct sx127x_dev_s *dev)
{
  switch (dev->modulation)
    {
#ifdef CONFIG_LPWAN_SX127X_LORA
      case SX127X_MODULATION_LORA:
        {
          sx127x_lora_dumpregs(dev);
          break;
        }
#endif
#ifdef CONFIG_LPWAN_SX127X_FSKOOK
      case SX127X_MODULATION_FSK:
      case SX127X_MODULATION_OOK:
        {
          sx127x_fskook_dumpregs(dev);
          break;
        }
#endif
      default:
        {
          wlinfo("Unknown SX127X modulation\n");
          break;
        }
    }
}
#endif  /* CONFIG_DEBUG_WIRELESS_INFO */

/****************************************************************************
 * Name: sx127x_unregister
 *
 * Description:
 *   Unregister SX127X device
 *
 ****************************************************************************/

static int sx127x_unregister(FAR struct sx127x_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  /* Release IRQ */

  sx127x_attachirq0(dev, NULL, NULL);

  /* Destroy semaphores */

  nxsem_destroy(&dev->dev_sem);
#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
  nxsem_destroy(&dev->tx_sem);
#endif
#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
  nxsem_destroy(&dev->rx_sem);
  nxsem_destroy(&dev->rx_buffer_sem);
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sx127x_register
 *
 * Description:
 *   Register sx127x driver
 *
 ****************************************************************************/

int sx127x_register(FAR struct spi_dev_s *spi,
                    FAR const struct sx127x_lower_s *lower)
{
  FAR struct sx127x_dev_s *dev = NULL;
  int ret = OK;

  DEBUGASSERT(spi != NULL);
  DEBUGASSERT(lower != NULL);

  /* Only one sx127x device supported for now */

  dev = &g_sx127x_devices[0];

  /* Reset data */

  memset(dev, 0, sizeof(struct sx127x_dev_s));

  /* Attach the interface, lower driver */

  dev->spi   = spi;
  dev->lower = lower;

  /* Initlaize IDLE mode */

  dev->idle = SX127X_IDLE_OPMODE;

#ifndef CONFIG_DISABLE_POLL
  dev->pfd        = NULL;
#endif

  /* Initialize sem */

  nxsem_init(&dev->dev_sem, 0, 1);
#ifdef CONFIG_LPWAN_SX127X_TXSUPPORT
  nxsem_init(&dev->tx_sem, 0, 0);
#endif
#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
  nxsem_init(&dev->rx_sem, 0, 0);
  nxsem_init(&dev->rx_buffer_sem, 0, 1);
#endif

  /* Attach irq0 - TXDONE/RXDONE/CADDONE */

  sx127x_attachirq0(dev, sx127x_irq0handler, dev);

  /* TODO: support for irq1-5 */

  wlinfo("Registering " SX127X_DEV_NAME "\n");

  ret = register_driver(SX127X_DEV_NAME, &sx127x_fops, 0666, dev);
  if (ret < 0)
    {
      wlerr("ERROR: register_driver() failed: %d\n", ret);
      sx127x_unregister(dev);
    }

  return ret;
}
