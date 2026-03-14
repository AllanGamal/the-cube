#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <math.h>
#include <stdint.h>

static const char *TAG = "bringup";

static const gpio_num_t MOTOR_PWM_GPIO = GPIO_NUM_8;
static const gpio_num_t MOTOR_DIR_GPIO = GPIO_NUM_10;
static const gpio_num_t IMU_SDA_GPIO = GPIO_NUM_6;
static const gpio_num_t IMU_SCL_GPIO = GPIO_NUM_7;
static const uint32_t PWM_FREQUENCY_HZ = 20000;
static const uint32_t PWM_DUTY_MAX_PERCENT = 5;
static const float MAX_TILT_DEGREES = 30.0f;
static const float TILT_DEADZONE_DEGREES = 3.0f;
static const TickType_t CONTROL_LOOP_DELAY = pdMS_TO_TICKS(100);

static const i2c_port_t IMU_I2C_PORT = I2C_NUM_0;
static const uint8_t MPU6050_ADDRESS = 0x68;
static const uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;
static const float MPU6050_ACCEL_SCALE = 16384.0f;

static void configure_i2c(void)
{
    const i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = IMU_SDA_GPIO,
        .scl_io_num = IMU_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };

    ESP_ERROR_CHECK(i2c_param_config(IMU_I2C_PORT, &config));
    ESP_ERROR_CHECK(i2c_driver_install(IMU_I2C_PORT, config.mode, 0, 0, 0));
}

static void wake_mpu6050(void)
{
    const uint8_t payload[] = {MPU6050_PWR_MGMT_1, 0x00};

    ESP_ERROR_CHECK(i2c_master_write_to_device(
        IMU_I2C_PORT,
        MPU6050_ADDRESS,
        payload,
        sizeof(payload),
        pdMS_TO_TICKS(100)));
}

static void configure_motor_pwm(void)
{
    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    const ledc_channel_config_t channel_config = {
        .gpio_num = MOTOR_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
}

static void configure_motor_direction(void)
{
    ESP_ERROR_CHECK(gpio_reset_pin(MOTOR_DIR_GPIO));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_DIR_GPIO, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_DIR_GPIO, 0));
}

static void set_motor_pwm_percent(uint32_t duty_percent)
{
    const uint32_t max_duty = (1U << LEDC_TIMER_10_BIT) - 1U;
    const uint32_t duty = (max_duty * duty_percent) / 100U;

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static void set_motor_direction(bool reverse_direction)
{
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_DIR_GPIO, reverse_direction ? 1 : 0));
}

static int16_t read_big_endian_i16(const uint8_t *buffer)
{
    return (int16_t)((buffer[0] << 8) | buffer[1]);
}

static float read_tilt_degrees(void)
{
    const uint8_t start_register = MPU6050_ACCEL_XOUT_H;
    uint8_t raw_data[6] = {0};

    ESP_ERROR_CHECK(i2c_master_write_read_device(
        IMU_I2C_PORT,
        MPU6050_ADDRESS,
        &start_register,
        1,
        raw_data,
        sizeof(raw_data),
        pdMS_TO_TICKS(100)));

    const float accel_x = read_big_endian_i16(&raw_data[0]) / MPU6050_ACCEL_SCALE;
    const float accel_z = read_big_endian_i16(&raw_data[4]) / MPU6050_ACCEL_SCALE;

    return atan2f(accel_x, accel_z) * (180.0f / (float)M_PI);
}

static uint32_t tilt_to_duty_percent(float tilt_degrees)
{
    const float absolute_tilt = fabsf(tilt_degrees);
    const float clamped_tilt = absolute_tilt > MAX_TILT_DEGREES ? MAX_TILT_DEGREES : absolute_tilt;
    const float scaled_percent = (clamped_tilt / MAX_TILT_DEGREES) * PWM_DUTY_MAX_PERCENT;

    return (uint32_t)lroundf(scaled_percent);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Application started");
    ESP_LOGI(TAG, "PWM on GPIO %d at %lu Hz", MOTOR_PWM_GPIO, (unsigned long)PWM_FREQUENCY_HZ);
    ESP_LOGI(TAG, "DIR on GPIO %d", MOTOR_DIR_GPIO);
    ESP_LOGI(TAG, "MPU6050 on SDA=%d SCL=%d", IMU_SDA_GPIO, IMU_SCL_GPIO);
    ESP_LOGI(TAG, "Tilt maps to 0-%lu%% PWM with %.1f deg deadzone",
             (unsigned long)PWM_DUTY_MAX_PERCENT,
             (double)TILT_DEADZONE_DEGREES);

    configure_i2c();
    wake_mpu6050();
    configure_motor_pwm();
    configure_motor_direction();
    set_motor_direction(false);
    set_motor_pwm_percent(0);

    while (1) {
        const float tilt_degrees = read_tilt_degrees();
        const bool in_deadzone = fabsf(tilt_degrees) <= TILT_DEADZONE_DEGREES;
        const uint32_t duty_percent = in_deadzone ? 0 : tilt_to_duty_percent(tilt_degrees);
        const bool reverse_direction = tilt_degrees > 0.0f;

        set_motor_direction(reverse_direction);
        set_motor_pwm_percent(duty_percent);

        ESP_LOGI(
            TAG,
            "tilt=%.1f deg dir=%s duty=%lu%%",
            (double)tilt_degrees,
            reverse_direction ? "reverse" : "forward",
            (unsigned long)duty_percent);

        vTaskDelay(CONTROL_LOOP_DELAY);
    }
}
