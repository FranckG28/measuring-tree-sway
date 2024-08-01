#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/task.h"

#define I2C_SLAVE_SCL_IO           9
#define I2C_SLAVE_SDA_IO           8
#define I2C_SLAVE_NUM              I2C_NUM_1
#define I2C_SLAVE_ADDR             0x28
#define I2C_SLAVE_TX_BUF_LEN       (2 * 512)
#define I2C_SLAVE_RX_BUF_LEN       (2 * 512)

static const char *TAG = "I2C_Slave";

void i2c_slave_init() {
    i2c_config_t conf_slave = {
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .mode = I2C_MODE_SLAVE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = I2C_SLAVE_ADDR,
    };
    i2c_param_config(I2C_SLAVE_NUM, &conf_slave);
    i2c_driver_install(I2C_SLAVE_NUM, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
}

void app_main() {
    i2c_slave_init();

    uint8_t data_received;
    uint8_t data_to_send = 0;
    while (1) {
        // Réception des données du maître
        int ret = i2c_slave_read_buffer(I2C_SLAVE_NUM, &data_received, 1, portMAX_DELAY);
        if (ret > 0) {
            ESP_LOGI(TAG, "Data received: 0x%02X", data_received);
            data_to_send = data_received + 1; // Traitement des données reçues (par exemple, incrémenter de 1)
        }

        // Envoie des données au maître
        ret = i2c_slave_write_buffer(I2C_SLAVE_NUM, &data_to_send, 1, portMAX_DELAY);
        if (ret > 0) {
            ESP_LOGI(TAG, "Data sent: 0x%02X", data_to_send);
        }
    }
}
