/*
 * Copyright (c) 2020 M5Stack <https://www.m5stack.com>
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file mpu6886.h
 * @defgroup mpu6886 mpu6886
 * @brief Functions for the MPU6886 inertial measurement unit (IMU).
 * @{
 *
 * ESP-IDF driver for the MPU6886 on I2C using @ropg — Rop Gonggrijp's I2C Manager
 * https://github.com/ropg/i2c_manager
 * 
 * Copyright (c) 2019 M5Stack <https://www.m5stack.com>
 * Copyright (c) 2021 Rashed Talukder <https://github.com/rashedtalukder>
 * 
 * MIT Licensed as described in the file LICENSE
 * }
 */

#include "mpu6886.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "MPU8886"

#define I2C_MASTER_SCL_IO           GPIO_NUM_39                 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_38                 /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

static gyro_scale_t gyro_scale = MPU6886_GFS_2000DPS;
static acc_scale_t acc_scale = MPU6886_AFS_16G;
static float acc_res, gyro_res;

/**
 * @brief Read a sequence of bytes from a MPU6886 sensor registers
 */
static esp_err_t mpu6886_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6886_ADDRESS, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU6886 sensor register
 */
static esp_err_t mpu6886_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6886_ADDRESS, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void mpu6886_init(void)
{
    uint8_t whoami;
    unsigned char tempdata[1];
    unsigned char regdata;
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read the MPU6886 WHO_AM_I register, on power up the register should have the value 0x19 */
    ESP_ERROR_CHECK(mpu6886_register_read(MPU6886_WHOAMI, &whoami, 1));
    ESP_LOGI(TAG, "MPU6886_WHOAMI = %x", whoami);

    regdata = 0x00;
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_PWR_MGMT_1, regdata));
    vTaskDelay(pdMS_TO_TICKS(10));
    
    regdata = (0x01 << 7);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_PWR_MGMT_1, regdata));
    vTaskDelay(pdMS_TO_TICKS(10));
    
    regdata = (0x01 << 0);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_PWR_MGMT_1, regdata));
    vTaskDelay(pdMS_TO_TICKS(10));

    regdata = 0x10;
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_ACCEL_CONFIG, regdata));
    vTaskDelay(pdMS_TO_TICKS(10));

    regdata = 0x18;
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_GYRO_CONFIG, regdata));
    vTaskDelay(pdMS_TO_TICKS(10));

    regdata = 0x01;
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_CONFIG, regdata));
    vTaskDelay(pdMS_TO_TICKS(10));

    regdata = 0x05;
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_SMPLRT_DIV, regdata));
    vTaskDelay(pdMS_TO_TICKS(10));

    regdata = 0x00;    
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_INT_ENABLE, regdata));
    vTaskDelay(pdMS_TO_TICKS(10));

    regdata = 0x00;
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_ACCEL_CONFIG2, regdata));
    vTaskDelay(pdMS_TO_TICKS(10));

    regdata = 0x00;
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_USER_CTRL, regdata));
    vTaskDelay(pdMS_TO_TICKS(10));

    regdata = 0x00;
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_FIFO_EN, regdata));
    vTaskDelay(pdMS_TO_TICKS(10));

    regdata = 0x22;
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_INT_PIN_CFG, regdata));
    vTaskDelay(pdMS_TO_TICKS(10));

    regdata = 0x01;
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_INT_ENABLE, regdata));
    vTaskDelay(pdMS_TO_TICKS(100));

    //mpu6886_gyro_res_get(gyro_scale, &gyro_res);
    //mpu6886_accel_res_get(acc_scale, &acc_res);

    mpu6886_fsr_gyro_set(MPU6886_GFS_1000DPS);
    vTaskDelay(pdMS_TO_TICKS(10));
    mpu6886_fsr_accel_set(MPU6886_AFS_2G);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void mpu6886_adc_accel_get(int16_t *ax, int16_t *ay, int16_t *az)
{   
    uint8_t buf[MPU6886_ADC_ACCEL_NUM_BYTES];
    
    ESP_ERROR_CHECK(mpu6886_register_read(MPU6886_ACCEL_XOUT_H, buf, MPU6886_ADC_ACCEL_NUM_BYTES));

    *ax = ( ( int16_t )buf[ 0 ] << 8 ) | buf[ 1 ];
    *ay = ( ( int16_t )buf[ 2 ] << 8 ) | buf[ 3 ];
    *az = ( ( int16_t )buf[ 4 ] << 8 ) | buf[ 5 ];
}

void mpu6886_adc_gyro_get(int16_t *gr, int16_t *gy, int16_t *gp)
{
    uint8_t buf[MPU6886_ADC_GYRO_NUM_BYTES];
    
    ESP_ERROR_CHECK(mpu6886_register_read(MPU6886_GYRO_XOUT_H, buf, MPU6886_ADC_GYRO_NUM_BYTES));

    *gr = ( ( uint16_t )buf[ 0 ] << 8 ) | buf[ 1 ];
    *gy = ( ( uint16_t )buf[ 2 ] << 8 ) | buf[ 3 ];
    *gp = ( ( uint16_t )buf[ 4 ] << 8 ) | buf[ 5 ];
}

void mpu6886_adc_temp_get(int16_t *t)
{
    uint8_t buf[MPU6886_ADC_TEMP_NUM_BYTES];

    ESP_ERROR_CHECK(mpu6886_register_read(MPU6886_TEMP_OUT_H, buf, MPU6886_ADC_TEMP_NUM_BYTES));
    
    *t = ( ( uint16_t )buf[ 0 ] << 8 ) | buf[ 1 ];
}

void mpu6886_gyro_res_get(gyro_scale_t scale, float *resolution)
{
    float gyro_res = 0.0;
    
    switch( scale )
    {
        case MPU6886_GFS_250DPS:
            gyro_res = 250.0 / 32768.0;
            break;
        case MPU6886_GFS_500DPS:
            gyro_res = 500.0 / 32768.0;
            break;
        case MPU6886_GFS_1000DPS:
            gyro_res = 1000.0 / 32768.0;
            break;
        case MPU6886_GFS_2000DPS:
        default:
            gyro_res = 2000.0 / 32768.0;
            break;
    }
    
    *resolution = gyro_res;
}

void mpu6886_accel_res_get(acc_scale_t scale, float *resolution)
{
    float accel_res = 0.0;
    switch(scale)
    {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs ( 10 ), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case MPU6886_AFS_2G:
            accel_res = 2.0 / 32768.0;
            break;
        case MPU6886_AFS_4G:
            accel_res = 4.0 / 32768.0;
            break;
        case MPU6886_AFS_8G:
            accel_res = 8.0 / 32768.0;
            break;
        case MPU6886_AFS_16G:
        default:
            accel_res = 16.0 / 32768.0;
            break;
    }
    *resolution = accel_res;
}

void mpu6886_fsr_gyro_set(gyro_scale_t scale)
{
    unsigned char regdata;
    regdata = (scale << 3);
    
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_GYRO_CONFIG, regdata));
    
    gyro_scale = scale;
    mpu6886_gyro_res_get(scale, &gyro_res);
}

void mpu6886_fsr_accel_set(acc_scale_t scale)
{
    unsigned char regdata;
    regdata = (scale << 3);
    
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_ACCEL_CONFIG, regdata));
    
    acc_scale = scale;
    mpu6886_accel_res_get(scale, &acc_res);
}

void mpu6886_accel_data_get(float *ax, float *ay, float *az)
{
    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;

    mpu6886_adc_accel_get( &accX, &accY, &accZ );

    *ax = ( float )accX * acc_res;
    *ay = ( float )accY * acc_res;
    *az = ( float )accZ * acc_res;
}

void mpu6886_gyro_data_get(float *gr, float *gy, float *gp)
{
    int16_t gyroR = 0;
    int16_t gyroP = 0;
    int16_t gyroY = 0;

    mpu6886_adc_gyro_get(&gyroR, &gyroY, &gyroP);

    *gr = ( float )gyroR * gyro_res;
    *gp = ( float )gyroP * gyro_res;
    *gy = ( float )gyroY * gyro_res;
}

void mpu6886_temp_data_get(float *t)
{
    int16_t temp = 0;
    mpu6886_adc_temp_get( &temp );
    *t = ( float )temp / 326.8 + 25.0;
}