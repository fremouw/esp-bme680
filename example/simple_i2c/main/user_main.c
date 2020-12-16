#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include "bme680.h"
#include "bme680_defs.h"

#define BME680_I2C_ADDR BME680_I2C_ADDR_SECONDARY
#define BME680_I2C_PORT_NUM I2C_NUM_0
#define BME680_I2C_SDA_PIN GPIO_NUM_21
#define BME680_I2C_SCL_PIN GPIO_NUM_22

static const char TAG[] = "main";

void user_delay_ms(uint32_t period) {
    vTaskDelay(pdMS_TO_TICKS(period));
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    esp_err_t err = ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME680_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME680_I2C_ADDR << 1) | I2C_MASTER_READ, true);

    if (len > 1) {
        i2c_master_read(cmd, reg_data, len-1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data+len-1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(BME680_I2C_PORT_NUM, cmd, pdMS_TO_TICKS(10));
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "error i2c read (%d)", err);

        rslt = -1;
    }

    i2c_cmd_link_delete(cmd);

    return rslt;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    esp_err_t err = ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME680_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, len, true);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(BME680_I2C_PORT_NUM, cmd, pdMS_TO_TICKS(10));
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "error i2c write (%d)", err);

        rslt = -1;
    }

    i2c_cmd_link_delete(cmd);

    return rslt;
}

static void MeasureTask(void* pvParam) {
    struct bme680_dev gas_sensor;

    gas_sensor.dev_id = BME680_I2C_ADDR;
    gas_sensor.intf = BME680_I2C_INTF;
    gas_sensor.read = user_i2c_read;
    gas_sensor.write = user_i2c_write;
    gas_sensor.delay_ms = user_delay_ms;
    /* amb_temp can be set to 25 prior to configuring the gas sensor 
     * or by performing a few temperature readings without operating the gas sensor.
     */
    gas_sensor.amb_temp = 25;

    int8_t rslt = BME680_OK;
    rslt = bme680_init(&gas_sensor);
    if(rslt != BME680_OK) {
        ESP_LOGE(TAG, "error initializing BME680 (%d)", rslt);

        vTaskDelete(NULL);
    }

    uint8_t set_required_settings;

    /* Set the temperature, pressure and humidity settings */
    gas_sensor.tph_sett.os_hum = BME680_OS_2X;
    gas_sensor.tph_sett.os_pres = BME680_OS_4X;
    gas_sensor.tph_sett.os_temp = BME680_OS_8X;
    gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

    /* Set the remaining gas sensor settings and link the heating profile */
    gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    /* Create a ramp heat waveform in 3 steps */
    gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
    gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */

    /* Select the power mode */
    /* Must be set before writing the sensor configuration */
    gas_sensor.power_mode = BME680_FORCED_MODE; 

    /* Set the required sensor settings needed */
    set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL 
        | BME680_GAS_SENSOR_SEL;

    /* Set the desired sensor configuration */
    rslt = bme680_set_sensor_settings(set_required_settings,&gas_sensor);

    /* Set the power mode */
    rslt = bme680_set_sensor_mode(&gas_sensor);


    uint16_t meas_period;
    bme680_get_profile_dur(&meas_period, &gas_sensor);

    struct bme680_field_data data;

    for(;;) {
        user_delay_ms(meas_period); /* Delay till the measurement is ready */

        rslt = bme680_get_sensor_data(&data, &gas_sensor);

        ESP_LOGI(TAG, "T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", data.temperature / 100.0f,
            data.pressure / 100.0f, data.humidity / 1000.0f );
        /* Avoid using measurements from an unstable heating setup */
        if(data.status & BME680_GASM_VALID_MSK)
            ESP_LOGI(TAG, "G: %d ohms", data.gas_resistance);

        /* Trigger the next measurement if you would like to read data out continuously */
        if (gas_sensor.power_mode == BME680_FORCED_MODE) {
            rslt = bme680_set_sensor_mode(&gas_sensor);
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }

    vTaskDelete(NULL);
}

void app_main() {
    ESP_LOGI(TAG, "esp-bme680");

    i2c_config_t i2c_config = {};

    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = BME680_I2C_SDA_PIN;
    i2c_config.scl_io_num = BME680_I2C_SCL_PIN;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.master.clk_speed = 400000;
    
    i2c_param_config(BME680_I2C_PORT_NUM, &i2c_config);
    i2c_driver_install(BME680_I2C_PORT_NUM, I2C_MODE_MASTER, 0, 0, 0);

    xTaskCreate(MeasureTask, "MeasureTask", 4096, NULL, 10, NULL);
}
