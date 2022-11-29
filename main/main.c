#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_system.h"
#include "esp_pm.h"
#include "esp_wifi.h"
#include "esp_tls.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "config.h"

#define MAX_CPU_FREQ_MHZ 80
#define MIN_CPU_FREQ_MHZ 10

#define I2C_PORT I2C_NUM_0
#if CONFIG_IDF_TARGET_ESP32
#define I2C_SDA_PIN GPIO_NUM_25
#define I2C_SCL_PIN GPIO_NUM_21
#elif CONFIG_IDF_TARGET_ESP32C3
#define I2C_SDA_PIN GPIO_NUM_6
#define I2C_SCL_PIN GPIO_NUM_7
#endif
#define I2C_FREQ_HZ 100000

#define BME680_I2C_ADDR 0x77

#define BME680_COEFF_ADDR1 0x89
#define BME680_COEFF_ADDR2 0xE1
#define BME680_COEFF_ADDR1_LEN 25
#define BME680_COEFF_ADDR2_LEN 16
#define BME680_CONFIG_ADDR 0x75
#define BME680_CTRL_MEAS_ADDR 0x74
#define BME680_CTRL_HUM_ADDR 0x72

// base addr = cofficient address1
#define BME680_T2_LSB 0x01
#define BME680_T2_MSB 0x02
#define BME680_T3 0x03
#define BME680_P1_LSB 0x05
#define BME680_P1_MSB 0x06
#define BME680_P2_LSB 0x07
#define BME680_P2_MSB 0x08
#define BME680_P3 0x09
#define BME680_P4_LSB 0x0B
#define BME680_P4_MSB 0x0C
#define BME680_P5_LSB 0x0D
#define BME680_P5_MSB 0x0E
#define BME680_P7 0x09
#define BME680_P6 0x10
#define BME680_P8_LSB 0x13
#define BME680_P8_MSB 0x14
#define BME680_P9_LSB 0x15
#define BME680_P9_MSB 0x16
#define BME680_P10 0x17

// base addr = cofficient address2
#define BME680_H2_MSB 0x00
#define BME680_H2_LSB 0x01
#define BME680_H1_LSB 0x01
#define BME680_H1_MSB 0x02
#define BME680_H3 0x03
#define BME680_H4 0x04
#define BME680_H5 0x05
#define BME680_H6 0x06
#define BME680_H7 0x07
#define BME680_T1_LSB 0x08
#define BME680_T1_MSB 0x09
#define BME680_GH2_LSB 0x0A
#define BME680_GH2_MSB 0x0B
#define BME680_GH1 0x0C
#define BME680_GH3 0x0D

#define BME680_HEAT_RANGE_ADDR 0x02
#define BME680_HEAT_VAL_ADDR 0x00
#define BME680_RANGE_SW_ERR_ADDR 0x04

#define BME680_TEMP_ADDR 0x22
#define BME680_TEMP_ADDR_LEN 3
#define BME680_TEMP_MSB 0x00
#define BME680_TEMP_LSB 0x01
#define BME680_TEMP_XLSB 0x02

#define BME680_HUM_ADDR 0x25
#define BME680_HUM_ADDR_LEN 2
#define BME680_HUM_MSB 0x00
#define BME680_HUM_LSB 0x01

#define BME680_PRESS_ADDR 0x1F
#define BME680_PRESS_ADDR_LEN 3
#define BME680_PRESS_MSB 0x00
#define BME680_PRESS_LSB 0x01
#define BME680_PRESS_XLSB 0x02

#define BME680_OS_DISABLE 0b000
#define BME680_OS_1X 0b001
#define BME680_OS_2X 0b010
#define BME680_OS_4X 0b011
#define BME680_OS_8X 0b100
#define BME680_OS_16X 0b101

#define BME680_FILTER_DISABLE 0b000
#define BME680_FILTER_1 0b001
#define BME680_FILTER_3 0b010
#define BME680_FILTER_7 0b011
#define BME680_FILTER_15 0b100
#define BME680_FILTER_31 0b101
#define BME680_FILTER_63 0b110
#define BME680_FILTER_127 0b111

#define BME680_MODE_SLEEP 0b00
#define BME680_MODE_FORCE 0b01

#define CONCAT_BYTES_12(msb, lsb) ((uint16_t)msb << 4 | (uint16_t)lsb >> 4)
#define CONCAT_BYTES_16(msb, lsb) ((uint16_t)msb << 8 | (uint16_t)lsb)
#define CONCAT_BYTES_20(msb, lsb, xlsb) ((uint32_t)msb << 12 | (uint32_t)lsb << 4 | (uint32_t)xlsb >> 4)

const char *TAG = "main";

extern const uint8_t root_ca_pem_start[] asm("_binary_root_ca_pem_start");
extern const uint8_t root_ca_pem_end[] asm("_binary_root_ca_pem_end");

static esp_netif_t *sta_netif = NULL;
static esp_mqtt_client_handle_t client = NULL;
static bool esp_mqtt_connected = false;

static uint16_t par_h1;
static uint16_t par_h2;
static int8_t par_h3;
static int8_t par_h4;
static int8_t par_h5;
static uint8_t par_h6;
static int8_t par_h7;
static uint16_t par_t1;
static int16_t par_t2;
static int8_t par_t3;
static uint16_t par_p1;
static int16_t par_p2;
static int8_t par_p3;
static int16_t par_p4;
static int16_t par_p5;
static int8_t par_p6;
static int8_t par_p7;
static int16_t par_p8;
static int16_t par_p9;
static uint8_t par_p10;
static uint32_t temp_adc, hum_adc, press_adc;
static double temp_comp, t_fine, hum_comp, press_comp;

esp_err_t pm_init()
{
#if CONFIG_PM_ENABLE
#if CONFIG_IDF_TARGET_ESP32
  esp_pm_config_esp32_t pm_config = {
#elif CONFIG_IDF_TARGET_ESP32C3
  esp_pm_config_esp32c3_t pm_config = {
#endif
    .max_freq_mhz = MAX_CPU_FREQ_MHZ,
    .min_freq_mhz = MIN_CPU_FREQ_MHZ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
    .light_sleep_enable = true
#endif
#if CONFIG_IDF_TARGET_ESP32
  };
  ESP_RETURN_ON_ERROR(esp_pm_configure(&pm_config), TAG, "esp_pm_configure failed");
#elif CONFIG_IDF_TARGET_ESP32C3
  };
  ESP_RETURN_ON_ERROR(esp_pm_configure(&pm_config), TAG, "esp_pm_configure failed");
#endif
#endif
  return ESP_OK;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    esp_wifi_connect();
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
  {
    esp_netif_create_ip6_linklocal(sta_netif);
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    ESP_LOGI(TAG, "Disconnected from AP, trying to reconnect...");
    esp_wifi_connect();
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    esp_mqtt_client_reconnect(client);
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_GOT_IP6)
  {
    ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
    ESP_LOGI(TAG, "got ip6:" IPV6STR, IPV62STR(event->ip6_info.ip));
    esp_mqtt_client_reconnect(client);
  }
}

static void mqtt_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  if (event_id == MQTT_EVENT_CONNECTED)
  {
    ESP_LOGI(TAG, "connected to mqtt broker");
    esp_mqtt_connected = true;
  }
  else if (event_id == MQTT_EVENT_DISCONNECTED)
  {
    ESP_LOGI(TAG, "disconnected from mqtt broker");
    esp_mqtt_connected = false;
  }
  else if (event_id == MQTT_EVENT_BEFORE_CONNECT)
  {
    ESP_LOGI(TAG, "connecting to mqtt broker");
    esp_mqtt_connected = false;
  }
  else if (event_id == MQTT_EVENT_ERROR)
  {
    ESP_LOGI(TAG, "mqtt error");
    esp_mqtt_connected = false;
  }
}

esp_err_t wifi_init()
{
  sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "esp_wifi_init failed");
  wifi_config_t wifi_config = {
      .sta = {
          .ssid = WIFI_SSID,
          .password = WIFI_PASSWORD,
          .threshold.authmode = WIFI_AUTHMODE,
          .listen_interval = 3,
      },
  };
  ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "esp_wifi_set_mode failed");
  ESP_RETURN_ON_ERROR(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config), TAG, "esp_wifi_set_config failed");
  ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL), TAG, "esp_event_handler_instance_register failed");
  ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL), TAG, "esp_event_handler_instance_register failed");
  ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start failed");
  ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_MAX_MODEM), TAG, "esp_wifi_set_ps failed");
  return ESP_OK;
}

esp_err_t mqtt_init()
{
  ESP_RETURN_ON_ERROR(esp_tls_init_global_ca_store(), TAG, "esp_tls_init_global_ca_store failed");
  ESP_RETURN_ON_ERROR(esp_tls_set_global_ca_store(root_ca_pem_start, root_ca_pem_end - root_ca_pem_start), TAG, "esp_tls_set_global_ca_store failed");

  esp_mqtt_client_config_t mqtt_cfg = {
      .broker.address.uri = MQTT_BROKER_URL,
      .broker.verification.use_global_ca_store = true,
      .credentials.username = MQTT_USERNAME,
      .credentials.authentication.password = MQTT_PASSWORD,
  };
  client = esp_mqtt_client_init(&mqtt_cfg);
  ESP_RETURN_ON_ERROR(esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, &mqtt_event_handler, NULL), TAG, "esp_event_handler_instance_register failed");
  return esp_mqtt_client_start(client);
}

esp_err_t i2c_init()
{
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_SDA_PIN,
      .scl_io_num = I2C_SCL_PIN,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_FREQ_HZ,
  };
  ESP_RETURN_ON_ERROR(i2c_param_config(I2C_PORT, &conf), TAG, "i2c_param_config failed");
  ESP_RETURN_ON_ERROR(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0), TAG, "i2c_driver_install failed");
  return ESP_OK;
}

esp_err_t i2c_write(uint8_t addr, uint8_t reg, uint8_t *data, size_t size)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  ESP_RETURN_ON_ERROR(i2c_master_start(cmd), TAG, "i2c_master_start failed");
  ESP_RETURN_ON_ERROR(i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true), TAG, "i2c_master_write_byte failed");
  ESP_RETURN_ON_ERROR(i2c_master_write_byte(cmd, reg, true), TAG, "i2c_master_write_byte failed");
  ESP_RETURN_ON_ERROR(i2c_master_write(cmd, data, size, true), TAG, "i2c_master_write failed");
  ESP_RETURN_ON_ERROR(i2c_master_stop(cmd), TAG, "i2c_master_stop failed");
  ESP_RETURN_ON_ERROR(i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS), TAG, "i2c_master_cmd_begin failed");
  i2c_cmd_link_delete(cmd);
  return ESP_OK;
}

esp_err_t i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t size)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  ESP_RETURN_ON_ERROR(i2c_master_start(cmd), TAG, "i2c_master_start failed");
  ESP_RETURN_ON_ERROR(i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true), TAG, "i2c_master_write_byte failed");
  ESP_RETURN_ON_ERROR(i2c_master_write_byte(cmd, reg, true), TAG, "i2c_master_write_byte failed");
  ESP_RETURN_ON_ERROR(i2c_master_start(cmd), TAG, "i2c_master_start failed");
  ESP_RETURN_ON_ERROR(i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true), TAG, "i2c_master_write_byte failed");
  if (size > 1)
  {
    ESP_RETURN_ON_ERROR(i2c_master_read(cmd, data, size - 1, I2C_MASTER_ACK), TAG, "i2c_master_read failed");
  }
  ESP_RETURN_ON_ERROR(i2c_master_read_byte(cmd, data + size - 1, I2C_MASTER_NACK), TAG, "i2c_master_read_byte failed");
  ESP_RETURN_ON_ERROR(i2c_master_stop(cmd), TAG, "i2c_master_stop failed");
  ESP_RETURN_ON_ERROR(i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS), TAG, "i2c_master_cmd_begin failed");
  i2c_cmd_link_delete(cmd);
  return ESP_OK;
}

esp_err_t fetch_calib_data()
{
  uint8_t coeff_array1[BME680_COEFF_ADDR1_LEN] = {};
  ESP_RETURN_ON_ERROR(i2c_read(BME680_I2C_ADDR, BME680_COEFF_ADDR1, coeff_array1, BME680_COEFF_ADDR1_LEN), TAG, "i2c_read failed");
  par_t2 = CONCAT_BYTES_16(coeff_array1[BME680_T2_MSB], coeff_array1[BME680_T2_LSB]);
  par_t3 = coeff_array1[BME680_T3];
  par_p1 = CONCAT_BYTES_16(coeff_array1[BME680_P1_MSB], coeff_array1[BME680_P1_LSB]);
  par_p2 = CONCAT_BYTES_16(coeff_array1[BME680_P2_MSB], coeff_array1[BME680_P2_LSB]);
  par_p3 = coeff_array1[BME680_P3];
  par_p4 = CONCAT_BYTES_16(coeff_array1[BME680_P4_MSB], coeff_array1[BME680_P4_LSB]);
  par_p5 = CONCAT_BYTES_16(coeff_array1[BME680_P5_MSB], coeff_array1[BME680_P5_LSB]);
  par_p6 = coeff_array1[BME680_P6];
  par_p7 = coeff_array1[BME680_P7];
  par_p8 = CONCAT_BYTES_16(coeff_array1[BME680_P8_MSB], coeff_array1[BME680_P8_LSB]);
  par_p9 = CONCAT_BYTES_16(coeff_array1[BME680_P9_MSB], coeff_array1[BME680_P9_LSB]);
  par_p10 = coeff_array1[BME680_P10];
  uint8_t coeff_array2[BME680_COEFF_ADDR2_LEN] = {};
  ESP_RETURN_ON_ERROR(i2c_read(BME680_I2C_ADDR, BME680_COEFF_ADDR2, coeff_array2, BME680_COEFF_ADDR2_LEN), TAG, "i2c_read failed");
  par_t1 = CONCAT_BYTES_16(coeff_array2[BME680_T1_MSB], coeff_array2[BME680_T1_LSB]);
  par_h1 = CONCAT_BYTES_12(coeff_array2[BME680_H1_MSB], coeff_array2[BME680_H1_LSB]);
  par_h2 = CONCAT_BYTES_12(coeff_array2[BME680_H2_MSB], coeff_array2[BME680_H2_LSB]);
  par_h3 = coeff_array2[BME680_H3];
  par_h4 = coeff_array2[BME680_H4];
  par_h5 = coeff_array2[BME680_H5];
  par_h6 = coeff_array2[BME680_H6];
  par_h7 = coeff_array2[BME680_H7];
  return ESP_OK;
}

esp_err_t fetch_temp_data()
{
  uint8_t data[BME680_TEMP_ADDR_LEN];
  ESP_RETURN_ON_ERROR(i2c_read(BME680_I2C_ADDR, BME680_TEMP_ADDR, data, BME680_TEMP_ADDR_LEN), TAG, "i2c_read failed");
  temp_adc = CONCAT_BYTES_20(data[BME680_TEMP_MSB], data[BME680_TEMP_LSB], data[BME680_TEMP_XLSB]);
  if (temp_adc == 0x80000)
  {
    return ESP_FAIL;
  }
  ESP_LOGD(TAG, "temp_adc: %lx", temp_adc);
  return ESP_OK;
}

esp_err_t fetch_hum_data()
{
  uint8_t data[BME680_HUM_ADDR_LEN];
  ESP_RETURN_ON_ERROR(i2c_read(BME680_I2C_ADDR, BME680_HUM_ADDR, data, BME680_HUM_ADDR_LEN), TAG, "i2c_read failed");
  hum_adc = CONCAT_BYTES_16(data[BME680_HUM_MSB], data[BME680_HUM_LSB]);
  if (hum_adc == 0x8000)
  {
    return ESP_FAIL;
  }
  ESP_LOGD(TAG, "hum_adc: %lx", hum_adc);
  return ESP_OK;
}

esp_err_t fetch_press_data()
{
  uint8_t data[BME680_PRESS_ADDR_LEN];
  ESP_RETURN_ON_ERROR(i2c_read(BME680_I2C_ADDR, BME680_PRESS_ADDR, data, BME680_PRESS_ADDR_LEN), TAG, "i2c_read failed");
  press_adc = CONCAT_BYTES_20(data[BME680_PRESS_MSB], data[BME680_PRESS_LSB], data[BME680_PRESS_XLSB]);
  if (press_adc == 0x80000)
  {
    return ESP_FAIL;
  }
  ESP_LOGD(TAG, "press_adc: %lx", press_adc);
  return ESP_OK;
}

esp_err_t set_config(uint8_t os_temp, uint8_t os_pres, uint8_t os_hum, uint8_t filter)
{
  uint8_t data_config;
  uint8_t data_ctrl_meas;
  uint8_t data_ctrl_hum;
  ESP_RETURN_ON_ERROR(i2c_read(BME680_I2C_ADDR, BME680_CONFIG_ADDR, &data_config, 1), TAG, "i2c_read failed");
  ESP_RETURN_ON_ERROR(i2c_read(BME680_I2C_ADDR, BME680_CTRL_MEAS_ADDR, &data_ctrl_meas, 1), TAG, "i2c_read failed");
  ESP_RETURN_ON_ERROR(i2c_read(BME680_I2C_ADDR, BME680_CTRL_HUM_ADDR, &data_ctrl_hum, 1), TAG, "i2c_read failed");
  data_config = (data_config & 0b11100011) | (filter << 2);
  data_ctrl_meas = (data_ctrl_meas & 0b00000011) | (os_temp << 5 | os_pres << 2);
  data_ctrl_hum = (data_ctrl_hum & 0b11111000) | os_hum;
  ESP_RETURN_ON_ERROR(i2c_write(BME680_I2C_ADDR, BME680_CONFIG_ADDR, &data_config, 1), TAG, "i2c_write failed");
  ESP_RETURN_ON_ERROR(i2c_write(BME680_I2C_ADDR, BME680_CTRL_MEAS_ADDR, &data_ctrl_meas, 1), TAG, "i2c_write failed");
  ESP_RETURN_ON_ERROR(i2c_write(BME680_I2C_ADDR, BME680_CTRL_HUM_ADDR, &data_ctrl_hum, 1), TAG, "i2c_write failed");
  return ESP_OK;
}

esp_err_t set_mode(uint8_t mode)
{
  uint8_t data_ctrl_meas;
  ESP_RETURN_ON_ERROR(i2c_read(BME680_I2C_ADDR, BME680_CTRL_MEAS_ADDR, &data_ctrl_meas, 1), TAG, "i2c_read failed");
  data_ctrl_meas = (data_ctrl_meas & 0b11111100) | mode;
  ESP_RETURN_ON_ERROR(i2c_write(BME680_I2C_ADDR, BME680_CTRL_MEAS_ADDR, &data_ctrl_meas, 1), TAG, "i2c_write failed");
  return ESP_OK;
}

double calc_temp()
{
  double var1, var2;
  var1 = (((double)temp_adc / 16384.0) - ((double)par_t1 / 1024.0)) * (double)par_t2;
  var2 = ((((double)temp_adc / 131072.0) - ((double)par_t1 / 8192.0)) *
          (((double)temp_adc / 131072.0) - ((double)par_t1 / 8192.0))) *
         ((double)par_t3 * 16.0);
  t_fine = var1 + var2;
  temp_comp = t_fine / 5120.0;
  return temp_comp;
}

double calc_hum()
{
  double var1, var2, var3, var4;
  var1 = hum_adc - (((double)par_h1 * 16.0) + (((double)par_h3 / 2.0) * temp_comp));
  var2 = var1 * (((double)par_h2 / 262144.0) * (1.0 + (((double)par_h4 / 16384.0) * temp_comp) + (((double)par_h5 / 1048576.0) * temp_comp * temp_comp)));
  var3 = (double)par_h6 / 16384.0;
  var4 = (double)par_h7 / 2097152.0;
  hum_comp = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);
  if (hum_comp > 100.0)
    hum_comp = 100.0;
  else if (hum_comp < 0.0)
    hum_comp = 0.0;
  return hum_comp;
}

double calc_press()
{
  double var1, var2, var3;
  var1 = ((double)t_fine / 2.0) - 64000.0;
  var2 = var1 * var1 * ((double)par_p6 / 131072.0);
  var2 = var2 + (var1 * (double)par_p5 * 2.0);
  var2 = (var2 / 4.0) + ((double)par_p4 * 65536.0);
  var1 = ((((double)par_p3 * var1 * var1) / 16384.0) +
          ((double)par_p2 * var1)) /
         524288.0;
  var1 = (1.0 + (var1 / 32768.0)) * (double)par_p1;
  press_comp = 1048576.0 - (double)press_adc;
  press_comp = ((press_comp - (var2 / 4096.0)) * 6250.0) / var1;
  var1 = ((double)par_p9 * press_comp * press_comp) / 2147483648.0;
  var2 = press_comp * ((double)par_p8 / 32768.0);
  var3 = (press_comp / 256.0) * (press_comp / 256.0) *
         (press_comp / 256.0) * (par_p10 / 131072.0);
  press_comp = press_comp + (var1 + var2 + var3 +
                             ((double)par_p7 * 128.0)) /
                                16.0;
  return press_comp;
}

void app_main(void)
{
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  ESP_ERROR_CHECK(pm_init());
  ESP_ERROR_CHECK(wifi_init());
  ESP_ERROR_CHECK(mqtt_init());
  ESP_ERROR_CHECK(i2c_init());
  while (true)
  {
    ESP_ERROR_CHECK(set_config(BME680_OS_16X, BME680_OS_16X, BME680_OS_16X, BME680_FILTER_DISABLE));
    ESP_ERROR_CHECK(set_mode(BME680_MODE_FORCE));
    ESP_ERROR_CHECK(fetch_calib_data());
    ESP_ERROR_CHECK(fetch_temp_data());
    ESP_ERROR_CHECK(fetch_hum_data());
    ESP_ERROR_CHECK(fetch_press_data());
    double temp = calc_temp();
    double hum = calc_hum();
    double press = calc_press();
    ESP_LOGI(TAG, "t:%f, h:%f, p:%f", temp, hum, press);
    if (esp_mqtt_connected)
    {
      char temp_str[16] = {};
      char hum_str[16] = {};
      char press_str[16] = {};
      sprintf(temp_str, "%f", temp);
      sprintf(hum_str, "%f", hum);
      sprintf(press_str, "%f", press);
      ESP_ERROR_CHECK(esp_mqtt_client_publish(client, MQTT_TOPIC_TEMP, temp_str, 0, 0, 0));
      ESP_ERROR_CHECK(esp_mqtt_client_publish(client, MQTT_TOPIC_HUM, hum_str, 0, 0, 0));
      ESP_ERROR_CHECK(esp_mqtt_client_publish(client, MQTT_TOPIC_PRESS, press_str, 0, 0, 0));
    }
    else
    {
      ESP_LOGE(TAG, "mqtt not connected");
    }

    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}
