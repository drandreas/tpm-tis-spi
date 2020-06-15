/*
 * Device driver for TCG/TCPA TPM (trusted platform module).
 * Specifications at www.trustedcomputinggroup.org
 *
 * This device driver implements the TPM interface as defined in
 * the TCG TPM Interface Spec version 1.3, revision 27 via _raw/native
 * SPI access_.
 *
 * It is based on linux/drivers/char/tpm/tpm_tis_spi_main.c
 */
#define DT_DRV_COMPAT infineon_tpm

#include <init.h>
#include <drivers/spi.h>
#include <sys/byteorder.h>
#include <tpm-tis-spi.h>
#include <logging/log.h>

/* Grotesque hack for pinmux boards */
#if defined(CONFIG_BOARD_FRDM_K64F) || defined(CONFIG_BOARD_RV32M1_VEGA)
#include <drivers/pinmux.h>
#include <fsl_port.h>
#endif

#define TPM_MAX_SPI_FRAMESIZE        64
#define TPM_ZEPHYR_LOCALITY           1

#define TPM_HEADER_SIZE              10
#define TPM_RETRY_COUNT              50
#define TPM_POLL_INTERVAL            50 /* msec */

#define TIS_SHORT_TIMEOUT           750 /* msec */
#define TIS_LONG_TIMEOUT           2000 /* msec */

/* Status Flags */
#define	TPM_STS_VALID              0x80
#define	TPM_STS_COMMAND_READY      0x40
#define	TPM_STS_GO                 0x20
#define	TPM_STS_DATA_AVAIL         0x10
#define	TPM_STS_DATA_EXPECT        0x08
#define	TPM_STS_SELF_TEST_DONE     0x04
#define	TPM_STS_RESP_RETRY         0x02

/* Access Flags */
#define	TPM_ACCESS_VALID           0x80
#define	TPM_ACCESS_ACTIVE_LOCALITY 0x20
#define	TPM_ACCESS_REQUEST_PENDING 0x04
#define	TPM_ACCESS_REQUEST_USE     0x02

/* Address List */
#define	TPM_ACCESS(l)       (0x0000 | ((l) << 12))
#define	TPM_INT_ENABLE(l)   (0x0008 | ((l) << 12))
#define	TPM_INT_VECTOR(l)   (0x000C | ((l) << 12))
#define	TPM_INT_STATUS(l)   (0x0010 | ((l) << 12))
#define	TPM_INTF_CAPS(l)    (0x0014 | ((l) << 12))
#define	TPM_STS(l)          (0x0018 | ((l) << 12))
#define	TPM_STS3(l)         (0x001b | ((l) << 12))
#define	TPM_DATA_FIFO(l)    (0x0024 | ((l) << 12))

#define	TPM_DID_VID(l)      (0x0F00 | ((l) << 12))
#define	TPM_RID(l)          (0x0F04 | ((l) << 12))

LOG_MODULE_REGISTER(tpm_tis_spi, LOG_LEVEL_DBG);

/* TPM Commands */
static uint8_t CMD_START_UP[] = {
  0x80, 0x01,             //TPMI_ST_COMMAND_TAG = TPM_ST_NO_SESSIONS
  0x00, 0x00, 0x00, 0x0C, //UINT32 (cmd size)   = 12
  0x00, 0x00, 0x01, 0x44, //TPM_CC              = TPM_CC_SelfTest
  0x00, 0x00              //TPM_SU              = TPM_SU_CLEAR
};

static uint8_t CMD_SELF_TEST[] = {
  0x80, 0x01,             //TPMI_ST_COMMAND_TAG = TPM_ST_NO_SESSIONS
  0x00, 0x00, 0x00, 0x0B, //UINT32 (cmd size)   = 11
  0x00, 0x00, 0x01, 0x43, //TPM_CC              = TPM_CC_Startup
  0x00                    //TPMI_YES_NO         = NO
};

struct tpm_device_data {
  struct device *spi_dev;
  struct spi_config spi_cfg;
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
  struct spi_cs_control cs_ctrl;
#endif
  u8_t locality;
};

static struct tpm_device_data tpm_data;

/*
 * TCG SPI flow control is documented in section 6.4 of the spec[1]. In short,
 * keep trying to read from the device until MISO goes high indicating the
 * wait state has ended.
 *
 * [1] https://trustedcomputinggroup.org/resource/pc-client-platform-tpm-profile-ptp-specification/
 */
static int tpm_flow_control(struct tpm_device_data* tpm, const struct spi_buf_set* buf_set)
{
  u8_t* iobuf = (u8_t*)buf_set->buffers->buf;
  size_t* iolen = (size_t*)buf_set->buffers->len;

  if((iobuf[3] & 0x01) == 0) {
    // handle SPI wait states
    iobuf[0] = 0;
    *iolen = 1;

    for (int i = 0; i < TPM_RETRY_COUNT; i++) {
      k_sleep(K_MSEC(5));

      int ret = spi_read(tpm->spi_dev, &tpm->spi_cfg, buf_set);
      if(ret < 0) {
        return ret;
      }
      if(iobuf[0] & 0x01) {
        return 0;
      }
    }
    return -ETIMEDOUT;
  }

  return 0;
}

static int tpm_transfer(struct tpm_device_data* tpm, u16_t addr, u16_t len, u8_t* in, const u8_t* out)
{
  int ret = 0;

  while (len) {
    u8_t transfer_len = (len < TPM_MAX_SPI_FRAMESIZE) ? len : TPM_MAX_SPI_FRAMESIZE;

    u8_t iobuf[TPM_MAX_SPI_FRAMESIZE];
    iobuf[0] = (in ? 0x80 : 0) | (transfer_len - 1);
    iobuf[1] = 0xd4;
    iobuf[2] = addr >> 8;
    iobuf[3] = addr;

    struct spi_buf buf = {
      .buf = &iobuf,
      .len = 4
    };

    const struct spi_buf_set buf_set = {
      .buffers = &buf,
      .count   = 1
    };
    
    ret = spi_transceive(tpm->spi_dev, &tpm->spi_cfg, &buf_set, &buf_set);
    if(ret < 0) {
      break;
    }

    ret = tpm_flow_control(tpm, &buf_set);
    if(ret < 0) {
      break;
    }

    buf.len = transfer_len;

    if(in) {
      ret = spi_read(tpm->spi_dev, &tpm->spi_cfg, &buf_set);

      memcpy(in, &iobuf[0], transfer_len);
      in += transfer_len;
    } else if(out) {
      memcpy(&iobuf[0], out, transfer_len);
      out += transfer_len;

      ret = spi_write(tpm->spi_dev, &tpm->spi_cfg, &buf_set);
    }

    if(ret < 0) {
      break;
    }

    len -= transfer_len;
  }

  // Release Chip Select and SPI Bus
  spi_release(tpm->spi_dev, &tpm->spi_cfg);

  return ret;
}

static int tpm_read_bytes(struct tpm_device_data *tpm, u16_t addr, u16_t len, u8_t* result)
{
  return tpm_transfer(tpm, addr, len, result, NULL);
}

static int tpm_write_bytes(struct tpm_device_data *tpm, u16_t addr, u16_t len, const u8_t* value)
{
  return tpm_transfer(tpm, addr, len, NULL, value);
}

static int tpm_read32(struct tpm_device_data *tpm, u16_t addr, u32_t* result)
{
  int ret = tpm_read_bytes(tpm, addr, sizeof(u32_t), (u8_t*)result);
  *result = sys_le32_to_cpu(*result);
  return ret;
}

static int tpm_read8(struct tpm_device_data *tpm, u16_t addr, u8_t* result)
{
  return tpm_read_bytes(tpm, addr, sizeof(u8_t), result);
}

static int tpm_write8(struct tpm_device_data *tpm, u16_t addr, u8_t value)
{
  return tpm_write_bytes(tpm, addr, sizeof(u8_t), &value);
}

static u8_t tpm_status(struct tpm_device_data *tpm)
{
  u8_t status;

  int rc = tpm_read8(tpm, TPM_STS(tpm->locality), &status);
  if (rc < 0) {
    return 0;
  } else {
    return status;
  }
}

static int wait_tpm_status(struct tpm_device_data *tpm, const u8_t status, const u32_t timeout)
{
  for(int i = 0; i < timeout/TPM_POLL_INTERVAL; i++) {
    if((tpm_status(tpm) & status) == status) {
      return status;
    }
    k_sleep(K_MSEC(TPM_POLL_INTERVAL));
  }
  return -EBUSY;
}


static int tpm_request_access(struct tpm_device_data *tpm, u8_t access)
{
  return tpm_write8(tpm, TPM_ACCESS(tpm->locality), access);
}

static u8_t tpm_access(struct tpm_device_data *tpm)
{
  u8_t access;
  int rc = tpm_read8(tpm, TPM_ACCESS(tpm->locality), &access);
  if (rc < 0) {
    return 0;
  } else {
    return access;
  }
}

static int wait_tpm_access(struct tpm_device_data *tpm, u8_t access, const u32_t timeout)
{
  for(int i = 0; i < timeout/TPM_POLL_INTERVAL; i++) {
    if((tpm_access(tpm) & access) == access) {
      return access;
    }
    k_sleep(K_MSEC(TPM_POLL_INTERVAL));
  }
  return -EBUSY;
}

static int tpm_get_burstcount(struct tpm_device_data *tpm)
{
  u32_t value;

  for(int i = 0; i < TIS_SHORT_TIMEOUT/TPM_POLL_INTERVAL; i++) {
    int rc = tpm_read32(tpm, TPM_STS(tpm->locality), &value);
    if(rc < 0) {
      return rc;
    }

    int burstcount = (value >> 8) & 0xFFFF;
    if(burstcount != 0) {
      return burstcount;
    }
    k_sleep(K_USEC(TPM_POLL_INTERVAL));
  }
  return -EBUSY;
}


static int tpm_read_segmented_bytes(struct tpm_device_data *tpm, u16_t len, u8_t* value)
{
  size_t count = 0;
  while (count < len) {
    if(wait_tpm_status(tpm, TPM_STS_DATA_AVAIL | TPM_STS_VALID, TIS_LONG_TIMEOUT) < 0) {
      return -ETIME;
    }

    int burstcount = tpm_get_burstcount(tpm);
    if (burstcount < 0) {
      return burstcount;
    }

    if(len - count < burstcount) {
      burstcount = len - count;
    }

    int rc = tpm_read_bytes(tpm, TPM_DATA_FIFO(tpm->locality), burstcount, &value[count]);
    if(rc < 0) {
      return rc;
    }

    count += burstcount;
  }
  return len;
}

static int tpm_cancel(struct device *dev)
{
  struct tpm_device_data *tpm = dev->driver_data;

  // Return to ready causes the current command to be canceled
  return tpm_write8(tpm, TPM_STS(tpm->locality), TPM_STS_COMMAND_READY);
}

static int tpm_transmit(struct device *dev,
                        size_t command_size,
                        const u8_t *command_buffer)
{
  struct tpm_device_data *tpm = dev->driver_data;

  if((tpm_status(tpm) & TPM_STS_COMMAND_READY) == 0) {
    tpm_cancel(dev);

    if(wait_tpm_status(tpm, TPM_STS_COMMAND_READY, TIS_LONG_TIMEOUT) < 0) {
      return -ETIME;
    }
  }

  // Transmit all bytes except last
  size_t count = 0;
  while(count < command_size - 1) {
    int burstcount = tpm_get_burstcount(tpm);
    if (burstcount < 0) {
      return burstcount;
    }

    if(command_size - count - 1 < burstcount) {
      burstcount = command_size - count - 1;
    }

    int rc = tpm_write_bytes(tpm, TPM_DATA_FIFO(tpm->locality), burstcount, command_buffer + count);
    if(rc < 0) {
      return rc;
    }
    count += burstcount;

    if(wait_tpm_status(tpm, TPM_STS_VALID, TIS_SHORT_TIMEOUT) < 0) {
      return -ETIME;
    }

    if((tpm_status(tpm) & TPM_STS_DATA_EXPECT) == 0) {
      return -EIO;
    }
  }

  // Transmit last byte
  int rc = tpm_write8(tpm, TPM_DATA_FIFO(tpm->locality), command_buffer[count]);
  if(rc < 0) {
    return rc;
  }

  if(wait_tpm_status(tpm, TPM_STS_VALID, TIS_SHORT_TIMEOUT) < 0) {
    return -ETIME;
  }

  if((tpm_status(tpm) & TPM_STS_DATA_EXPECT) != 0) {
    return -EIO;
  }

  // Start Execution
  rc = tpm_write8(tpm, TPM_STS(tpm->locality), TPM_STS_GO);
  if (rc < 0) {
    return -EIO;
  }

  return 0;
}

static int tpm_receive(struct device *dev,
                       size_t *response_size,
                       u8_t *response_buffer,
                       s32_t timeout)
{
  struct tpm_device_data *tpm = dev->driver_data;

  // Responde to size query with max size
  if(response_buffer == NULL) {
    *response_size = 4096;
    return 0;
  }

  // Check if buffer is sufficiently large for header
  if (*response_size < TPM_HEADER_SIZE) {
    return -EIO;
  }

  // Receive header (tag uint16, paramsize uint32, result code uint32)
  int rc = tpm_read_segmented_bytes(tpm, TPM_HEADER_SIZE, response_buffer);
  if (rc < TPM_HEADER_SIZE) {
    return -EIO;
  }

  // Extract expected receive size (paramsize uint32)
  u32_t expected = sys_be32_to_cpu(*(u32_t*)(response_buffer + 2));
  if((expected > *response_size) || (expected < TPM_HEADER_SIZE)) {
    return -EIO;
  }

  rc = tpm_read_segmented_bytes(tpm, expected - TPM_HEADER_SIZE,
                                &response_buffer[TPM_HEADER_SIZE]);
  if(rc + TPM_HEADER_SIZE < expected) {
    return -EIO;
  }

  // Wait for TPM to return to idle
  if(wait_tpm_status(tpm, TPM_STS_VALID, TIS_SHORT_TIMEOUT) < 0) {
    return -ETIME;
  }

  return 0;
}

static struct tpm_device_api tpm_api = {
  .transmit = tpm_transmit,
  .receive = tpm_receive,
  .cancel = tpm_cancel
};

int tpm_init(struct device *dev) {
  struct tpm_device_data *tpm = dev->driver_data;

  // Configure SPI Driver and TPM CS
  tpm->spi_dev = device_get_binding(DT_INST_BUS_LABEL(0));
  if(tpm->spi_dev == NULL) {
    LOG_ERR("Could not get SPI device for TPM 2.0");
    return -EINVAL;
  }

  tpm->spi_cfg.frequency = DT_INST_PROP(0, spi_max_frequency);
  tpm->spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_HOLD_ON_CS |
                           SPI_LOCK_ON | SPI_WORD_SET(8);
  tpm->spi_cfg.slave     = DT_INST_REG_ADDR(0);

#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
  tpm->cs_ctrl.gpio_dev = device_get_binding(DT_INST_SPI_DEV_CS_GPIOS_LABEL(0));
  tpm->cs_ctrl.gpio_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(0);
  tpm->cs_ctrl.delay    = 0U;
  tpm->spi_cfg.cs       = &(tpm->cs_ctrl);
#if defined(CONFIG_BOARD_FRDM_K64F) || defined(CONFIG_BOARD_RV32M1_VEGA)
  pinmux_pin_set(device_get_binding(CONFIG_PINMUX_MCUX_PORTD_NAME), //TODO
                 tpm->cs_ctrl.gpio_pin,
                 PORT_PCR_MUX(kPORT_MuxAsGpio));
#endif
#else
  tpm->spi_cfg.cs       = NULL;
#endif
  tpm->locality         = 0;

  // Probe TPM
  u32_t vendor = 0;
  if(tpm_read32(tpm, TPM_DID_VID(tpm->locality), &vendor) < 0) {
    LOG_ERR("Could not find TPM 2.0");
    return -EIO;
  }

  u8_t rid = 0;
  if(tpm_read8(tpm, TPM_RID(tpm->locality), &rid) < 0) {
    LOG_ERR("Could not find TPM 2.0");
    return -EIO;
  }

  // Request Locality
  if(tpm_request_access(tpm, TPM_ACCESS_REQUEST_USE) < 0) {
    LOG_ERR("Could not request locality");
    return -EIO;
  }

  // Wait for Locality to become available
  if(wait_tpm_access(tpm, TPM_ACCESS_ACTIVE_LOCALITY | TPM_ACCESS_VALID, TIS_LONG_TIMEOUT) < 0) {
    LOG_ERR("Could not request locality");
    return -EIO;
  }

  // Execute StartUp
  if(tpm_transmit(dev, sizeof(CMD_START_UP), CMD_START_UP) < 0) {
    LOG_ERR("Could not start tpm");
    return -EIO;
  }

  size_t buf_sz = TPM_HEADER_SIZE;
  u8_t buf[TPM_HEADER_SIZE];
  if(tpm_receive(dev, &buf_sz, &buf[0], TIS_LONG_TIMEOUT) < 0) {
    LOG_ERR("Could not start tpm");
    return -EIO;
  }

  // Execute SelfTest
  if(tpm_transmit(dev, sizeof(CMD_SELF_TEST), CMD_SELF_TEST) < 0) {
    LOG_ERR("Could not execute selftest");
    return -EIO;
  }

  if(tpm_receive(dev, &buf_sz, &buf[0], TIS_LONG_TIMEOUT) < 0) {
    LOG_ERR("Could not execute selftest");
    return -EIO;
  }

  LOG_INF("TPM 2.0 (device-id 0x%X, rev-id %d)", vendor >> 16, rid);

  return 0;
}

DEVICE_AND_API_INIT(tpm,
                    DT_INST_LABEL(0),
                    &tpm_init,
                    &tpm_data,
                    NULL,
                    APPLICATION,
                    CONFIG_APPLICATION_INIT_PRIORITY,
                    &tpm_api);
