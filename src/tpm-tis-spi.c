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
#define DT_DRV_COMPAT infineon_tpm20

#include <init.h>
#include <drivers/spi.h>
#include <sys/byteorder.h>
#include <logging/log.h>

/* Grotesque hack for pinmux boards */
#if defined(CONFIG_BOARD_FRDM_K64F) || defined(CONFIG_BOARD_RV32M1_VEGA)
#include <drivers/pinmux.h>
#include <fsl_port.h>
#endif

#define MAX_SPI_FRAMESIZE   64
#define TPM_RETRY		        50

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

LOG_MODULE_REGISTER(tpm, LOG_LEVEL_DBG);

struct tpm20_device_data {
  struct device *spi_dev;
  struct spi_config spi_cfg;
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
  struct spi_cs_control cs_ctrl;
#endif
};

struct tpm20_device_data tpm20_data;

/*
 * TCG SPI flow control is documented in section 6.4 of the spec[1]. In short,
 * keep trying to read from the device until MISO goes high indicating the
 * wait state has ended.
 *
 * [1] https://trustedcomputinggroup.org/resource/pc-client-platform-tpm-profile-ptp-specification/
 */
static int tpm20_flow_control(struct tpm20_device_data* tpm20, const struct spi_buf_set* buf_set)
{
  u8_t* iobuf = (u8_t*)buf_set->buffers->buf;
  size_t* iolen = (size_t*)buf_set->buffers->len;

  if((iobuf[3] & 0x01) == 0) {
    // handle SPI wait states
    iobuf[0] = 0;
    *iolen = 1;

    for (int i = 0; i < TPM_RETRY; i++) {
      k_sleep(K_USEC(5));

      int ret = spi_read(tpm20->spi_dev, &tpm20->spi_cfg, buf_set);
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

static int tpm20_transfer(struct tpm20_device_data* tpm20, u16_t addr, u16_t len, u8_t* in, const u8_t* out)
{
  int ret = 0;
  
  while (len) {
    u8_t transfer_len = (len < MAX_SPI_FRAMESIZE) ? len : MAX_SPI_FRAMESIZE; 

    u8_t iobuf[MAX_SPI_FRAMESIZE];
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
    
    ret = spi_transceive(tpm20->spi_dev, &tpm20->spi_cfg, &buf_set, &buf_set);
    if(ret < 0) {
      break;
    }

    ret = tpm20_flow_control(tpm20, &buf_set);
    if(ret < 0) {
      break;
    }

    buf.len = transfer_len;

    if(in) {
      ret = spi_read(tpm20->spi_dev, &tpm20->spi_cfg, &buf_set);

      memcpy(in, &iobuf[0], transfer_len);
      in += transfer_len;
    } else if(out) {
      memcpy(&iobuf[0], out, transfer_len);
      out += transfer_len;

      ret = spi_write(tpm20->spi_dev, &tpm20->spi_cfg, &buf_set);
    }

    if(ret < 0) {
      break;
    }

    len -= transfer_len;
  }

  // Release Chip Select and SPI Bus
  spi_release(tpm20->spi_dev, &tpm20->spi_cfg);

  return ret;
}

static int tpm20_read_bytes(struct tpm20_device_data *tpm20, u16_t addr, u16_t len, u8_t* result)
{
  return tpm20_transfer(tpm20, addr, len, result, NULL);
}

static int tpm20_write_bytes(struct tpm20_device_data *tpm20, u16_t addr, u16_t len, const u8_t* value)
{
  return tpm20_transfer(tpm20, addr, len, NULL, value);
}

static int tpm20_read32(struct tpm20_device_data *tpm20, u16_t addr, u32_t* result)
{
  int ret = tpm20_read_bytes(tpm20, addr, sizeof(u32_t), (u8_t*)result);
  *result = sys_le32_to_cpu(*result);
  return ret;
}

static int tpm20_read8(struct tpm20_device_data *tpm20, u16_t addr, u8_t* result)
{
  return tpm20_read_bytes(tpm20, addr, sizeof(u8_t), result);
}

static u8_t tpm20_status(struct tpm20_device_data *tpm20)
{
  u8_t status;

  int rc = tpm20_read8(tpm20, TPM_STS(0), &status);
  if (rc < 0) {
    return 0;
  } else {
    return status;
  }
}

int tpm20_init(struct device *dev) {
  struct tpm20_device_data *tpm20 = dev->driver_data;

  // Configure SPI Driver and TPM CS
  tpm20->spi_dev = device_get_binding(DT_INST_BUS_LABEL(0));
  if(tpm20->spi_dev == NULL) {
    LOG_ERR("Could not get SPI device for TPM 2.0");
    return -EINVAL;
  }

  tpm20->spi_cfg.frequency = DT_INST_PROP(0, spi_max_frequency);
  tpm20->spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_HOLD_ON_CS |
                             SPI_LOCK_ON | SPI_WORD_SET(8);
  tpm20->spi_cfg.slave     = DT_INST_REG_ADDR(0);

#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
  tpm20->cs_ctrl.gpio_dev = device_get_binding(DT_INST_SPI_DEV_CS_GPIOS_LABEL(0));
  tpm20->cs_ctrl.gpio_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(0);
  tpm20->cs_ctrl.delay    = 0U;
  tpm20->spi_cfg.cs       = &(tpm20->cs_ctrl);
#if defined(CONFIG_BOARD_FRDM_K64F) || defined(CONFIG_BOARD_RV32M1_VEGA)
  pinmux_pin_set(device_get_binding(CONFIG_PINMUX_MCUX_PORTD_NAME), //TODO Fix
                 tpm20->cs_ctrl.gpio_pin,
                 PORT_PCR_MUX(kPORT_MuxAsGpio));
#endif
#else
  tpm20->spi_cfg.cs       = NULL;
#endif

  // Probe TPM
  u32_t vendor = 0;
  if(tpm20_read32(tpm20, TPM_DID_VID(0), &vendor) < 0) {
    LOG_ERR("Could not find TPM 2.0");
    return -EIO;
  }

  u8_t rid = 0;
	if(tpm20_read8(tpm20, TPM_RID(0), &rid) < 0) {
    LOG_ERR("Could not find TPM 2.0");
    return -EIO;
  }

  LOG_INF("TPM 2.0 (device-id 0x%X, rev-id %d)", vendor >> 16, rid);

  return 0;
}

DEVICE_AND_API_INIT(tpm20,
                    DT_INST_LABEL(0),
                    &tpm20_init,
                    &tpm20_data,
                    NULL,
                    APPLICATION,
                    CONFIG_APPLICATION_INIT_PRIORITY,
                    NULL);