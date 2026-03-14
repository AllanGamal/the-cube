#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static const char *TAG = "bringup";

static const gpio_num_t MOTOR_PWM_GPIO = GPIO_NUM_8;
static const gpio_num_t MOTOR_DIR_GPIO = GPIO_NUM_10;
static const gpio_num_t IMU_SDA_GPIO = GPIO_NUM_6;
static const gpio_num_t IMU_SCL_GPIO = GPIO_NUM_7;

static const uint32_t PWM_FREQUENCY_HZ = 20000;
static const uint32_t PWM_DUTY_MAX_PERCENT = 20;
static const uint32_t PWM_BIAS_DUTY_PERCENT = 10;
static const float CONTROL_OUTPUT_LIMIT = 30.0f;
static const float TILT_DEADZONE_DEGREES = 2.0f;
static const float SHUTDOWN_TILT_DEGREES = 60.0f;
static const TickType_t CONTROL_LOOP_DELAY = pdMS_TO_TICKS(10);
static const TickType_t DIRECTION_CHANGE_DELAY = pdMS_TO_TICKS(50);
static const TickType_t DIRECTION_CHANGE_CONFIRM_DELAY = pdMS_TO_TICKS(50);
static const TickType_t DIRECTION_CHANGE_HOLD_DELAY = pdMS_TO_TICKS(300);
static const TickType_t PWM_RAMP_STEP_DELAY = pdMS_TO_TICKS(10);
static const TickType_t LOG_DELAY = pdMS_TO_TICKS(100);
static const uint32_t PWM_RAMP_STEP_PERCENT = 2;
static const float DIRECTION_CHANGE_TRIGGER_DEGREES = 2.0f;

static const i2c_port_t IMU_I2C_PORT = I2C_NUM_0;
static const uint8_t MPU6050_ADDRESS = 0x68;
static const uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;
static const float MPU6050_ACCEL_SCALE = 16384.0f;
static const float MPU6050_GYRO_SCALE = 131.0f;

static const float PID_KP = 5.0f;
static const float PID_KI = 0.01f;
static const float PID_KD = 0.88f;
static const float INTEGRAL_LIMIT = 20.0f;
static const float UPRIGHT_REFERENCE_DEGREES = 182.0f
;
static const float FORWARD_BALANCE_TARGET_DEGREES = 8.0f;
static const float REVERSE_BALANCE_TARGET_DEGREES = -8.0f;
static const float X_AXIS_NEUTRAL_DEGREES = 100.0f;
static const float X_AXIS_NEUTRAL_TOLERANCE_DEGREES = 20.0f;

typedef struct {
    float accel_tilt_x_degrees;
    float accel_tilt_y_degrees;
    float gyro_rate_dps;
} imu_state_t;

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

static float normalize_upright_angle(float raw_angle_degrees)
{
    float normalized = raw_angle_degrees - UPRIGHT_REFERENCE_DEGREES;

    while (normalized <= -180.0f) {
        normalized += 360.0f;
    }

    while (normalized > 180.0f) {
        normalized -= 360.0f;
    }

    return normalized;
}

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

static uint32_t ramp_motor_pwm_percent(uint32_t current_duty_percent, uint32_t target_duty_percent)
{
    while (current_duty_percent != target_duty_percent) {
        if (current_duty_percent < target_duty_percent) {
            const uint32_t remaining = target_duty_percent - current_duty_percent;
            current_duty_percent += remaining < PWM_RAMP_STEP_PERCENT ? remaining : PWM_RAMP_STEP_PERCENT;
        } else {
            const uint32_t remaining = current_duty_percent - target_duty_percent;
            current_duty_percent -= remaining < PWM_RAMP_STEP_PERCENT ? remaining : PWM_RAMP_STEP_PERCENT;
        }

        set_motor_pwm_percent(current_duty_percent);

        if (current_duty_percent != target_duty_percent) {
            vTaskDelay(PWM_RAMP_STEP_DELAY);
        }
    }

    return current_duty_percent;
}

static int16_t read_big_endian_i16(const uint8_t *buffer)
{
    return (int16_t)((buffer[0] << 8) | buffer[1]);
}

static imu_state_t read_imu_state(void)
{
    const uint8_t start_register = MPU6050_ACCEL_XOUT_H;
    uint8_t raw_data[14] = {0};

    ESP_ERROR_CHECK(i2c_master_write_read_device(
        IMU_I2C_PORT,
        MPU6050_ADDRESS,
        &start_register,
        1,
        raw_data,
        sizeof(raw_data),
        pdMS_TO_TICKS(100)));

    const float accel_x = read_big_endian_i16(&raw_data[0]) / MPU6050_ACCEL_SCALE;
    const float accel_y = read_big_endian_i16(&raw_data[2]) / MPU6050_ACCEL_SCALE;
    const float accel_z = read_big_endian_i16(&raw_data[4]) / MPU6050_ACCEL_SCALE;
    const float gyro_y_dps = read_big_endian_i16(&raw_data[10]) / MPU6050_GYRO_SCALE;

    return (imu_state_t){
        .accel_tilt_x_degrees = atan2f(accel_y, accel_z) * (180.0f / (float)M_PI),
        .accel_tilt_y_degrees = atan2f(accel_x, accel_z) * (180.0f / (float)M_PI),
        .gyro_rate_dps = gyro_y_dps,
    };
}

static uint32_t control_to_duty_percent(float control_signal)
{
    const float adjustment_span = (float)(PWM_DUTY_MAX_PERCENT - PWM_BIAS_DUTY_PERCENT);
    const float normalized = clamp_float(fabsf(control_signal), 0.0f, CONTROL_OUTPUT_LIMIT);
    const float scaled_adjustment = (normalized / CONTROL_OUTPUT_LIMIT) * adjustment_span;
    const float requested_percent = clamp_float(
        (float)PWM_BIAS_DUTY_PERCENT + scaled_adjustment,
        0.0f,
        (float)PWM_DUTY_MAX_PERCENT);

    return (uint32_t)lroundf(requested_percent);
}

void app_main(void)
{
    int64_t previous_time_us = esp_timer_get_time();
    TickType_t last_log_tick = 0;
    TickType_t last_direction_change_tick = 0;
    TickType_t pending_direction_change_tick = 0;
    float estimated_angle_degrees = 0.0f;
    float integral_error = 0.0f;
    bool applied_reverse_direction = false;
    bool pending_reverse_direction = false;
    uint32_t applied_duty_percent = PWM_BIAS_DUTY_PERCENT;

    ESP_LOGI(TAG, "Application started");
    ESP_LOGI(TAG, "PWM on GPIO %d at %lu Hz", MOTOR_PWM_GPIO, (unsigned long)PWM_FREQUENCY_HZ);
    ESP_LOGI(TAG, "DIR on GPIO %d", MOTOR_DIR_GPIO);
    ESP_LOGI(TAG, "MPU6050 on SDA=%d SCL=%d", IMU_SDA_GPIO, IMU_SCL_GPIO);
    ESP_LOGI(TAG, "Using tilt_y with %.1f degrees as upright reference", (double)UPRIGHT_REFERENCE_DEGREES);
    ESP_LOGI(TAG, "Motor bias duty: %lu%% in forward direction", (unsigned long)PWM_BIAS_DUTY_PERCENT);
    ESP_LOGI(TAG, "Balance targets: forward=%.1f deg reverse=%.1f deg",
             (double)FORWARD_BALANCE_TARGET_DEGREES,
             (double)REVERSE_BALANCE_TARGET_DEGREES);
    ESP_LOGI(TAG, "Direction changes only when |tilt| > %.1f deg for %lu ms",
             (double)DIRECTION_CHANGE_TRIGGER_DEGREES,
             (unsigned long)pdTICKS_TO_MS(DIRECTION_CHANGE_CONFIRM_DELAY));
    ESP_LOGI(TAG, "PID gains Kp=%.2f Ki=%.2f Kd=%.2f",
             (double)PID_KP,
             (double)PID_KI,
             (double)PID_KD);

    configure_i2c();
    wake_mpu6050();
    configure_motor_pwm();
    configure_motor_direction();
    set_motor_direction(applied_reverse_direction);
    set_motor_pwm_percent(0);
    applied_duty_percent = ramp_motor_pwm_percent(0, applied_duty_percent);

    while (1) {
        const imu_state_t imu_state = read_imu_state();
        const int64_t current_time_us = esp_timer_get_time();
        const TickType_t current_tick = xTaskGetTickCount();
        const float tilt_y_upright_degrees = normalize_upright_angle(imu_state.accel_tilt_y_degrees);
        float dt_seconds = (float)(current_time_us - previous_time_us) / 1000000.0f;

        if (dt_seconds <= 0.0f || dt_seconds > 0.1f) {
            dt_seconds = 0.01f;
        }

        previous_time_us = current_time_us;

        estimated_angle_degrees = tilt_y_upright_degrees;
        const float target_angle_degrees =
            applied_reverse_direction ? REVERSE_BALANCE_TARGET_DEGREES : FORWARD_BALANCE_TARGET_DEGREES;
        const float angle_error = target_angle_degrees - estimated_angle_degrees;
        integral_error = clamp_float(integral_error + (angle_error * dt_seconds), -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        float control_signal =
            (PID_KP * angle_error) +
            (PID_KI * integral_error) -
            (PID_KD * imu_state.gyro_rate_dps);

        const bool tilt_is_unsafe = fabsf(angle_error) >= SHUTDOWN_TILT_DEGREES;
        const bool in_deadzone = fabsf(angle_error) <= TILT_DEADZONE_DEGREES;
        const bool x_axis_inhibit =
            fabsf(imu_state.accel_tilt_x_degrees - X_AXIS_NEUTRAL_DEGREES) > X_AXIS_NEUTRAL_TOLERANCE_DEGREES;

        if (tilt_is_unsafe || in_deadzone || x_axis_inhibit) {
            control_signal = 0.0f;

            if (tilt_is_unsafe) {
                integral_error = 0.0f;
            }
        }

        uint32_t duty_percent = control_to_duty_percent(control_signal);
        const bool desired_reverse_direction = estimated_angle_degrees < 0.0f;
        const bool direction_change_allowed =
            (current_tick - last_direction_change_tick) >= DIRECTION_CHANGE_HOLD_DELAY;
        const bool direction_change_requested =
            fabsf(estimated_angle_degrees) >= DIRECTION_CHANGE_TRIGGER_DEGREES &&
            desired_reverse_direction != applied_reverse_direction;

        if (x_axis_inhibit) {
            duty_percent = 0;
            pending_direction_change_tick = 0;
        }

        if (direction_change_requested && direction_change_allowed) {
            if (pending_direction_change_tick == 0 || pending_reverse_direction != desired_reverse_direction) {
                pending_reverse_direction = desired_reverse_direction;
                pending_direction_change_tick = current_tick;
            } else if ((current_tick - pending_direction_change_tick) >= DIRECTION_CHANGE_CONFIRM_DELAY) {
                applied_duty_percent = ramp_motor_pwm_percent(applied_duty_percent, 0);
                vTaskDelay(DIRECTION_CHANGE_DELAY);
                set_motor_direction(pending_reverse_direction);
                applied_reverse_direction = pending_reverse_direction;
                last_direction_change_tick = xTaskGetTickCount();
                pending_direction_change_tick = 0;
            }
        } else {
            pending_direction_change_tick = 0;
        }

        if (duty_percent != applied_duty_percent) {
            applied_duty_percent = ramp_motor_pwm_percent(applied_duty_percent, duty_percent);
        }

        if ((current_tick - last_log_tick) >= LOG_DELAY) {
            ESP_LOGI(
                TAG,
                "x=%.1f y=%.1f desired=%s dir=%s duty=%lu%% pending=%s",
                (double)imu_state.accel_tilt_x_degrees,
                (double)tilt_y_upright_degrees,
                desired_reverse_direction ? "reverse" : "forward",
                applied_reverse_direction ? "reverse" : "forward",
                (unsigned long)applied_duty_percent,
                pending_direction_change_tick == 0 ? "no" : (pending_reverse_direction ? "reverse" : "forward"));
            last_log_tick = current_tick;
        }

        vTaskDelay(CONTROL_LOOP_DELAY);
    }
}
