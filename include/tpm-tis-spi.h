#pragma once

#include <device.h>

typedef int (*tpm_transmit_t)(const struct device *device,
                              size_t command_size,
                              const uint8_t *command_buffer);

typedef int (*tpm_receive_t)(const struct device *device,
                             size_t *response_size,
                             uint8_t *response_buffer,
                             k_timeout_t timeout);

typedef int (*tpm_cancel_t)(const struct device *device);

struct tpm_device_api {
  tpm_transmit_t transmit;
  tpm_receive_t receive;
  tpm_cancel_t cancel;
};

static inline int tpm_device_transmit(const struct device *device,
                                      size_t command_size,
                                      const uint8_t *command_buffer)
{
  struct tpm_device_api *api;

  api = (struct tpm_device_api *)device->api;
  return api->transmit(device, command_size, command_buffer);
}

static inline int tpm_device_receive(const struct device *device,
                                     size_t *response_size,
                                     uint8_t *response_buffer,
                                     k_timeout_t timeout)
{
  struct tpm_device_api *api;

  api = (struct tpm_device_api *)device->api;
  return api->receive(device, response_size, response_buffer, timeout);
}

static inline int tpm_device_cancel(const struct device *device)
{
  struct tpm_device_api *api;

  api = (struct tpm_device_api *)device->api;
  return api->cancel(device);
}
