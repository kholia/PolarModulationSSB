#include "esp_i2c.h"

#include "driver/i2c_master.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "hal/i2c_ll.h"
#include "sdkconfig.h"

static const char *TAG = "polar-i2c";
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t device_handle;
static uint8_t device_address;
static uint32_t bus_speed_hz;
static bool fast_ll_enabled;
static bool fast_ll_inflight;
static i2c_dev_t *const fast_ll_hw = I2C_LL_GET_HW(0);

uint32_t polar_i2c_default_speed_hz(void) {
#if defined(CONFIG_POLAR_PROFILE_3MHZ_32K)
  return 3000000;
#elif defined(CONFIG_POLAR_PROFILE_2MHZ_16K) || defined(CONFIG_POLAR_PROFILE_2MHZ_32K)
  return 2000000;
#else
  return 1000000;
#endif
}

static esp_err_t IRAM_ATTR fast_ll_wait(void) {
  if (!fast_ll_inflight) return ESP_OK;
  for (uint32_t elapsed_us = 0; elapsed_us < 2000; ++elapsed_us) {
    if (i2c_ll_master_is_cmd_done(fast_ll_hw, 2)) {
      fast_ll_inflight = false;
      uint32_t status = fast_ll_hw->int_raw.val;
      i2c_ll_clear_intr_mask(fast_ll_hw, I2C_LL_INTR_MASK);
      if (status & (I2C_LL_INTR_NACK | I2C_LL_INTR_TIMEOUT |
                    I2C_LL_INTR_ARBITRATION))
        return ESP_ERR_INVALID_RESPONSE;
      return ESP_OK;
    }
    esp_rom_delay_us(1);
  }
  return ESP_ERR_TIMEOUT;
}

static esp_err_t IRAM_ATTR fast_ll_submit(const uint8_t *data, size_t length) {
  if (length == 0 || length + 1 > I2C_LL_GET(FIFO_LEN))
    return ESP_ERR_INVALID_SIZE;
  ESP_RETURN_ON_ERROR(fast_ll_wait(), TAG, "previous fast I2C transfer failed");

  // The ordinary driver has already configured pins, clock source, 2 MHz bus
  // timing and timeouts. Leave those registers untouched. For each phase
  // update, only refill the FIFO and replace the three hardware commands.
  i2c_ll_disable_intr_mask(fast_ll_hw, I2C_LL_INTR_MASK);
  i2c_ll_clear_intr_mask(fast_ll_hw, I2C_LL_INTR_MASK);
  i2c_ll_txfifo_rst(fast_ll_hw);

  uint8_t write_address = static_cast<uint8_t>(device_address << 1);
  i2c_ll_write_txfifo(fast_ll_hw, &write_address, 1);
  i2c_ll_write_txfifo(fast_ll_hw, data, static_cast<uint8_t>(length));

  i2c_ll_hw_cmd_t start = {};
  start.op_code = I2C_LL_CMD_RESTART;
  i2c_ll_hw_cmd_t write = {};
  write.op_code = I2C_LL_CMD_WRITE;
  write.byte_num = static_cast<uint8_t>(length + 1);
  write.ack_en = 1;
  i2c_ll_hw_cmd_t stop = {};
  stop.op_code = I2C_LL_CMD_STOP;
  i2c_ll_master_write_cmd_reg(fast_ll_hw, start, 0);
  i2c_ll_master_write_cmd_reg(fast_ll_hw, write, 1);
  i2c_ll_master_write_cmd_reg(fast_ll_hw, stop, 2);
  i2c_ll_update(fast_ll_hw);
  fast_ll_inflight = true;
  i2c_ll_start_trans(fast_ll_hw);
  return ESP_OK;
}

static esp_err_t add_device(uint32_t speed_hz) {
  i2c_device_config_t config = {};
  config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  config.device_address = device_address;
  config.scl_speed_hz = speed_hz;
  config.scl_wait_us = 0;
  config.flags.disable_ack_check = false;
  ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus_handle, &config, &device_handle),
                      TAG, "failed to add Si5351 at %lu Hz",
                      static_cast<unsigned long>(speed_hz));
  bus_speed_hz = speed_hz;
  if (speed_hz > 400000) {
    ESP_LOGW(TAG,
             "Si5351 bus at %lu Hz is above the published 400 kHz limit; use "
             "short traces and external pull-ups",
             static_cast<unsigned long>(speed_hz));
  }
  return ESP_OK;
}

static esp_err_t create_bus(uint32_t speed_hz) {
  i2c_master_bus_config_t config = {};
  config.i2c_port = 0;
  config.sda_io_num = static_cast<gpio_num_t>(CONFIG_POLAR_I2C_SDA_GPIO);
  config.scl_io_num = static_cast<gpio_num_t>(CONFIG_POLAR_I2C_SCL_GPIO);
  config.clk_source = I2C_CLK_SRC_DEFAULT;
  // At 1--2 MHz, a seven-cycle glitch filter consumes too much of each edge.
  config.glitch_ignore_cnt = 2;
  config.intr_priority = 0;
  config.trans_queue_depth = 0;
  config.flags.enable_internal_pullup = false;
  config.flags.allow_pd = false;

  ESP_RETURN_ON_ERROR(i2c_new_master_bus(&config, &bus_handle), TAG,
                      "failed to create I2C bus");
  esp_err_t err = add_device(speed_hz);
  if (err != ESP_OK) {
    (void)i2c_del_master_bus(bus_handle);
    bus_handle = nullptr;
    return err;
  }
  return ESP_OK;
}

esp_err_t polar_i2c_init(uint8_t address, uint32_t speed_hz) {
  if (bus_handle != nullptr) return ESP_OK;

  device_address = address;
  // Register initialization uses ordinary blocking transfers. The low-level
  // FIFO path takes over only after probing and device setup are complete.
  return create_bus(speed_hz);
}

esp_err_t polar_i2c_enable_async(void) {
  if (fast_ll_enabled) return ESP_OK;
  if (!bus_handle || !device_handle) return ESP_ERR_INVALID_STATE;
  // The IDF driver applies a device's SCL timing lazily at transaction start.
  // A one-byte register-pointer write is harmless and primes the peripheral
  // with the newly selected 1/2 MHz timing before the low-level takeover.
  uint8_t register_pointer = 0;
  ESP_RETURN_ON_ERROR(
      i2c_master_transmit(device_handle, &register_pointer, 1, 10), TAG,
      "failed to prime low-level I2C timing");
  // Synchronous ESP-IDF transactions leave the event interrupt disabled.
  // Explicitly mask it before taking over the FIFO/command registers.
  i2c_ll_disable_intr_mask(fast_ll_hw, I2C_LL_INTR_MASK);
  i2c_ll_clear_intr_mask(fast_ll_hw, I2C_LL_INTR_MASK);
  fast_ll_inflight = false;
  fast_ll_enabled = true;
  ESP_LOGW(TAG, "Si5351 low-level FIFO fast path enabled");
  return ESP_OK;
}

esp_err_t polar_i2c_wait(void) {
  if (!bus_handle || !fast_ll_enabled) return ESP_OK;
  return fast_ll_wait();
}

esp_err_t polar_i2c_set_speed(uint32_t speed_hz) {
  if (!bus_handle || !device_handle) return ESP_ERR_INVALID_STATE;
  if (speed_hz == bus_speed_hz) return ESP_OK;

  ESP_RETURN_ON_ERROR(polar_i2c_wait(), TAG, "I2C did not become idle");
  bool restore_fast_ll = fast_ll_enabled;
  fast_ll_enabled = false;
  ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(device_handle), TAG,
                      "failed to remove old I2C profile");
  device_handle = nullptr;
  ESP_RETURN_ON_ERROR(add_device(speed_hz), TAG, "failed to select I2C profile");
  if (restore_fast_ll) {
    ESP_RETURN_ON_ERROR(polar_i2c_enable_async(), TAG,
                        "failed to restore low-level I2C");
  }
  ESP_LOGW(TAG, "I2C profile changed to %lu Hz",
           static_cast<unsigned long>(speed_hz));
  return ESP_OK;
}

esp_err_t polar_i2c_submit(const uint8_t *data, size_t length) {
  if (!device_handle) return ESP_ERR_INVALID_STATE;
  if (fast_ll_enabled) return fast_ll_submit(data, length);
  return i2c_master_transmit(device_handle, data, length, 10);
}

esp_err_t polar_i2c_write(const uint8_t *data, size_t length) {
  ESP_RETURN_ON_ERROR(polar_i2c_wait(), TAG, "previous I2C transfer failed");
  ESP_RETURN_ON_ERROR(polar_i2c_submit(data, length), TAG, "I2C write failed");
  return polar_i2c_wait();
}

esp_err_t polar_i2c_write_read(const uint8_t *write_data, size_t write_length,
                               uint8_t *read_data, size_t read_length) {
  ESP_RETURN_ON_ERROR(polar_i2c_wait(), TAG, "previous I2C transfer failed");
  if (fast_ll_enabled) return ESP_ERR_NOT_SUPPORTED;
  ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(
                          device_handle, write_data, write_length, read_data,
                          read_length, 10),
                      TAG, "I2C register read failed");
  return polar_i2c_wait();
}
