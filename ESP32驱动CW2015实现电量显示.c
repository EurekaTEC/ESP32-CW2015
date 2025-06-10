#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 42          // SCL引脚
#define I2C_MASTER_SDA_IO 41          // SDA引脚
#define I2C_MASTER_NUM I2C_NUM_0      // I2C端口号
#define I2C_MASTER_FREQ_HZ 100000     // I2C时钟频率
#define CW2015_I2C_ADDRESS 0xC4       // CW2015的I2C写地址
#define CW2015_I2C_READ_ADDRESS 0xC5  // CW2015的I2C读地址
#define ALRT_PIN GPIO_NUM_35          // ALRT#引脚

static const char* TAG = "CW2015 Driver";

// 初始化I2C
void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

// 读取CW2015寄存器
esp_err_t cw2015_read_register(uint8_t reg, uint8_t* data, size_t size) {
    if (size == 0) return ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (CW2015_I2C_ADDRESS), true); // 写地址
    i2c_master_write_byte(cmd, reg, true); // 写寄存器地址
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (CW2015_I2C_READ_ADDRESS), true); // 读地址
    if (size > 1) {
        i2c_master_read(cmd, data, size - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + size - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// 写入CW2015寄存器
esp_err_t cw2015_write_register(uint8_t reg, uint8_t* data, size_t size) {
    if (size == 0) return ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (CW2015_I2C_ADDRESS), true); // 写地址
    i2c_master_write_byte(cmd, reg, true); // 写寄存器地址
    i2c_master_write(cmd, data, size, true); // 写数据
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ALRT#中断处理函数
void alrt_isr_handler(void* arg) {
    ESP_LOGI(TAG, "ALRT# Interrupt triggered");
    // 在这里处理报警事件
}

// 初始化ALRT#引脚
void alrt_pin_init() {
    esp_rom_gpio_pad_select_gpio(ALRT_PIN);
    gpio_set_direction(ALRT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ALRT_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(ALRT_PIN, GPIO_INTR_LOW_LEVEL);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ALRT_PIN, alrt_isr_handler, NULL);
}

void app_main() {
    // 初始化I2C
    i2c_master_init();
    ESP_LOGI(TAG, "I2C initialized");

    while(1){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        // 读取版本号寄存器
        uint8_t reg_value;
        if (cw2015_read_register(0x00, &reg_value, 1) == ESP_OK) {
            ESP_LOGI(TAG, "Read register 0x00: 0x%02X", reg_value);
        } else {
            ESP_LOGE(TAG, "Failed to read register 0x00");
        }

        // 读取电池电压寄存器 (VCELL)
        uint8_t vcell_reg[2] = {0};
        if (cw2015_read_register(0x02, vcell_reg, 2) == ESP_OK) {
            uint16_t vcell = (vcell_reg[0] << 6) | (vcell_reg[1] >> 2);
            float voltage = vcell * 0.000305; // 305uV分辨率
            ESP_LOGI(TAG, "Battery Voltage: %.3f V", voltage);
        } else {
            ESP_LOGE(TAG, "Failed to read VCELL register");
        }

        // 读取SOC寄存器
        uint8_t soc_reg[2] = {0};
        if (cw2015_read_register(0x04, soc_reg, 2) == ESP_OK) {
            uint16_t soc = (soc_reg[0] << 8) | soc_reg[1];
            float soc_percentage = soc / 256.0; // 1/256%分辨率
            ESP_LOGI(TAG, "SOC: %.2f%%", soc_percentage);
        } else {
            ESP_LOGE(TAG, "Failed to read SOC register");
        }

        // 读取CONFIG寄存器
        if (cw2015_read_register(0x08, &reg_value, 1) == ESP_OK) {
            ESP_LOGI(TAG, "CONFIG register: 0x%02X", reg_value);
        } else {
            ESP_LOGE(TAG, "Failed to read CONFIG register");
        }
    }
}