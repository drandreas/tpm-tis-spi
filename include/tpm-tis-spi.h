#pragma once

#include <device.h>

typedef int (*tpm_transmit_t)(struct device *device,
                              size_t command_size,
                              const u8_t *command_buffer);

typedef int (*tpm_receive_t)(struct device *device,
                             size_t *response_size,
                             u8_t *response_buffer,
                             s32_t timeout);

typedef int (*tpm_cancel_t)(struct device *device);

struct tpm_device_api {
  tpm_transmit_t transmit;
  tpm_receive_t receive;
  tpm_cancel_t cancel;
};

static inline int tpm_device_transmit(struct device *device,
                                      size_t command_size,
                                      const u8_t *command_buffer)
{
  struct tpm_device_api *api;

  api = (struct tpm_device_api *)device->driver_api;
  return api->transmit(device, command_size, command_buffer);
}

static inline int tpm_device_receive(struct device *device,
                                     size_t *response_size,
                                     u8_t *response_buffer,
                                     s32_t timeout)
{
  struct tpm_device_api *api;

  api = (struct tpm_device_api *)device->driver_api;
  return api->receive(device, response_size, response_buffer, timeout);
}

static inline int tpm_device_cancel(struct device *device)
{
  struct tpm_device_api *api;

  api = (struct tpm_device_api *)device->driver_api;
  return api->cancel(device);
}
