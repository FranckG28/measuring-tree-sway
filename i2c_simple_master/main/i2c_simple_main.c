#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           8
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_SLAVE_ADDR              0x28

static const char *TAG = "I2C_Master";

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void app_main() {
    i2c_master_init();

    uint8_t data_to_send = 0x55;
    uint8_t data_received;
    esp_err_t ret;
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Envoie des données à l'esclave
        ret = i2c_master_write_to_device(I2C_MASTER_NUM, I2C_SLAVE_ADDR, &data_to_send, 1, 1000 / portTICK_PERIOD_MS);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Data sent: 0x%02X", data_to_send);
        } else {
            ESP_LOGE(TAG, "Error sending data: %s", esp_err_to_name(ret));
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Demande des données à l'esclave
        ret = i2c_master_read_from_device(I2C_MASTER_NUM, I2C_SLAVE_ADDR, &data_received, 1, 1000 / portTICK_PERIOD_MS);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Data received: 0x%02X", data_received);
        } else {
            ESP_LOGE(TAG, "Error receiving data: %s", esp_err_to_name(ret));
        }
    }
}
