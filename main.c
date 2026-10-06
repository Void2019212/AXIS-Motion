#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "nvs_flash.h"

#include "driver/i2c.h"


#define AXIS_NAME "AXIS"

#define MPU_ADDR 0x68

#define SDA_PIN 21
#define SCL_PIN 22

#define I2C_PORT I2C_NUM_0

#define WIFI_SSID "YOUR_WIFI"
#define WIFI_PASSWORD "YOUR_PASSWORD"

static const char *TAG = "AXIS";


/* -----------------------------
   Sensor data
----------------------------- */

typedef struct
{
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float pitch;
    float roll;
    float yaw;

    int moving;

} AxisData;


static AxisData axis;


/* -----------------------------
   I2C
----------------------------- */

static void i2c_init(void)
{
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,

        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,

        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,

        .master.clk_speed = 400000
    };

    i2c_param_config(I2C_PORT, &config);

    i2c_driver_install(
        I2C_PORT,
        I2C_MODE_MASTER,
        0,
        0,
        0
    );
}


/* -----------------------------
   MPU6050 write
----------------------------- */

static void mpu_write(
    uint8_t reg,
    uint8_t value
)
{
    uint8_t data[2];

    data[0] = reg;
    data[1] = value;

    i2c_master_write_to_device(
        I2C_PORT,
        MPU_ADDR,
        data,
        2,
        pdMS_TO_TICKS(100)
    );
}


/* -----------------------------
   MPU6050 read
----------------------------- */

static void mpu_read(
    uint8_t reg,
    uint8_t *data,
    size_t length
)
{
    i2c_master_write_read_device(
        I2C_PORT,
        MPU_ADDR,
        &reg,
        1,
        data,
        length,
        pdMS_TO_TICKS(100)
    );
}


/* -----------------------------
   MPU6050 setup
----------------------------- */

static void mpu_init(void)
{
    // Wake up
    mpu_write(0x6B, 0x00);

    // Accelerometer ±2g
    mpu_write(0x1C, 0x00);

    // Gyroscope ±250°/s
    mpu_write(0x1B, 0x00);

    vTaskDelay(
        pdMS_TO_TICKS(100)
    );
}


/* -----------------------------
   Read MPU6050
----------------------------- */

static void read_mpu(void)
{
    uint8_t data[14];

    mpu_read(
        0x3B,
        data,
        14
    );

    int16_t ax =
        (data[0] << 8) | data[1];

    int16_t ay =
        (data[2] << 8) | data[3];

    int16_t az =
        (data[4] << 8) | data[5];

    int16_t gx =
        (data[8] << 8) | data[9];

    int16_t gy =
        (data[10] << 8) | data[11];

    int16_t gz =
        (data[12] << 8) | data[13];


    axis.ax =
        ax / 16384.0f;

    axis.ay =
        ay / 16384.0f;

    axis.az =
        az / 16384.0f;


    axis.gx =
        gx / 131.0f;

    axis.gy =
        gy / 131.0f;

    axis.gz =
        gz / 131.0f;


    /* Motion detection */

    float acceleration =
        sqrtf(
            axis.ax * axis.ax +
            axis.ay * axis.ay +
            axis.az * axis.az
        );


    float movement =
        fabsf(acceleration - 1.0f);


    axis.moving =
        movement > 0.15f;


    /* Tilt */

    axis.pitch =
        atan2f(
            axis.ax,
            sqrtf(
                axis.ay * axis.ay +
                axis.az * axis.az
            )
        ) * 57.2958f;


    axis.roll =
        atan2f(
            axis.ay,
            sqrtf(
                axis.ax * axis.ax +
                axis.az * axis.az
            )
        ) * 57.2958f;


    /*
     * This is only gyro integration.
     * It will drift over time.
     */

    axis.yaw +=
        axis.gz * 0.01f;
}


/* -----------------------------
   Sensor task
----------------------------- */

static void sensor_task(void *arg)
{
    while (1)
    {
        read_mpu();

        vTaskDelay(
            pdMS_TO_TICKS(10)
        );
    }
}