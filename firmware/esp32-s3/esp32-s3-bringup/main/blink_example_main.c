#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static const char *TAG = "b";

static const gpio_num_t MOTOR_PWM_GPIO = GPIO_NUM_8;
static const gpio_num_t MOTOR_DIR_GPIO = GPIO_NUM_10;
static const gpio_num_t MOTOR_BRAKE_GPIO = GPIO_NUM_12;
static const gpio_num_t IMU_SDA_GPIO = GPIO_NUM_6;
static const gpio_num_t IMU_SCL_GPIO = GPIO_NUM_7;

static const i2c_port_t IMU_I2C_PORT = I2C_NUM_0;
static const uint8_t MPU6050_ADDRESS = 0x68;
static const uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU6050_CONFIG = 0x1A; 
static const uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;
static const uint8_t MPU6050_DLPF_CFG_94HZ = 0x02;

static const uint32_t PWM_FREQUENCY_HZ = 20000;
static const uint32_t PWM_DUTY_MAX_PERCENT = 100;
static const float CONTROL_LIMIT = 255.0f;
static const float BALANCE_KP = 40.0f;
static const float BALANCE_KI = 1.0f;
static const float BALANCE_KD = 12.50f;
static const float BALANCE_KS = 1.4f;
static const float INTEGRAL_LIMIT = 100.0f;
static const float GYRO_RATE_FILTER_ALPHA = 0.40f;
static const float BRAKE_ANGLE_DEGREES = 0.00f;
static const float WARN_ANGLE_DEGREES = 5.0f;
static const float DISABLE_ANGLE_DEGREES = 10.0f;


static const TickType_t CONTROL_LOOP_DELAY = pdMS_TO_TICKS(10);
static const TickType_t LOG_DELAY = pdMS_TO_TICKS(100);
static const TickType_t GYRO_CALIBRATION_SAMPLE_DELAY = pdMS_TO_TICKS(5);
static const uint32_t GYRO_CALIBRATION_SAMPLES = 256;

static const float MPU6050_ACCEL_SCALE = 16384.0f;
static const float MPU6050_GYRO_SCALE = 131.0f;
static const float Y_AXIS_UPRIGHT_REFERENCE_DEGREES = 0.0f;
static const float ANGLE_TRIM_DEGREES = -3.0f;
static const float COMPLEMENTARY_GYRO_WEIGHT = 0.92f;
static const bool PLOT_OUTPUT_ENABLED = false;

typedef struct {
    float tilt_y_degrees;
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

static float clamp_symmetric(float value, float limit)
{
    return clamp_float(value, -limit, limit);
}

static float normalize_angle(float raw_angle_degrees, float reference_degrees)
{
    float normalized = raw_angle_degrees - reference_degrees;

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

static void configure_mpu6050_filter(void)
{
    const uint8_t payload[] = {MPU6050_CONFIG, MPU6050_DLPF_CFG_94HZ};

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

static void configure_motor_brake(void)
{
    ESP_ERROR_CHECK(gpio_reset_pin(MOTOR_BRAKE_GPIO));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_BRAKE_GPIO, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_BRAKE_GPIO, 1));
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

static void set_motor_brake(bool enabled)
{
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_BRAKE_GPIO, enabled ? 1 : 0));
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
    const float accel_z = read_big_endian_i16(&raw_data[4]) / MPU6050_ACCEL_SCALE;
    const float gyro_y_dps = read_big_endian_i16(&raw_data[10]) / MPU6050_GYRO_SCALE;
    const float raw_tilt_y_degrees = atan2f(accel_x, accel_z) * (180.0f / (float)M_PI);

    return (imu_state_t){
        .tilt_y_degrees = normalize_angle(raw_tilt_y_degrees, Y_AXIS_UPRIGHT_REFERENCE_DEGREES) + ANGLE_TRIM_DEGREES,
        .gyro_rate_dps = -gyro_y_dps,
    };
}

static float calibrate_gyro_offset_dps(void)
{
    float gyro_sum_dps = 0.0f;

    for (uint32_t sample_index = 0; sample_index < GYRO_CALIBRATION_SAMPLES; ++sample_index) {
        const imu_state_t imu_state = read_imu_state();
        gyro_sum_dps += imu_state.gyro_rate_dps;
        vTaskDelay(GYRO_CALIBRATION_SAMPLE_DELAY);
    }

    return gyro_sum_dps / (float)GYRO_CALIBRATION_SAMPLES;
}

static uint32_t control_to_duty_percent(float control_signal)
{
    const float normalized = clamp_float(fabsf(control_signal), 0.0f, CONTROL_LIMIT);
    const float requested_percent = (normalized / CONTROL_LIMIT) * PWM_DUTY_MAX_PERCENT;
    return (uint32_t)lroundf(requested_percent);
}

static void print_plot_header_once(void)
{
    static bool header_printed = false;

    if (!PLOT_OUTPUT_ENABLED || header_printed) {
        return;
    }

    //ets_printf("plot,p,i,d,angle,gyro,speed,control,duty,brake,state\n");
    header_printed = true;
}

void app_main(void)
{
    TickType_t last_log_tick = 0;
    int64_t previous_time_us = esp_timer_get_time();
    float gyro_offset_dps = 0.0f;
    float filtered_angle_degrees = 0.0f;
    float filtered_gyro_dps = 0.0f;
    float motor_speed_estimate = 0.0f;
    float angle_integral = 0.0f;
    bool angle_is_initialized = false;
    bool gyro_filter_is_initialized = false;
    bool limit_state_announced = false;
    bool warn_state_announced = false;

    ESP_LOGI(TAG, "Application started");
    ESP_LOGI(TAG, "Using minimal PD balance controller with MPU6050 DLPF and complementary filter");
    ESP_LOGI(TAG, "Gyro Y sign: inverted for test");
    ESP_LOGI(TAG, "PWM on GPIO %d at %lu Hz", MOTOR_PWM_GPIO, (unsigned long)PWM_FREQUENCY_HZ);
    ESP_LOGI(TAG, "DIR on GPIO %d", MOTOR_DIR_GPIO);
    ESP_LOGI(TAG, "BRAKE on GPIO %d", MOTOR_BRAKE_GPIO);
    ESP_LOGI(TAG, "MPU6050 on SDA=%d SCL=%d", IMU_SDA_GPIO, IMU_SCL_GPIO);
    ESP_LOGI(
        TAG,
        "Controller gains kp=%.2f ki=%.2f kd=%.2f ks=%.2f i_limit=%.2f gyro_alpha=%.2f",
        (double)BALANCE_KP,
        (double)BALANCE_KI,
        (double)BALANCE_KD,
        (double)BALANCE_KS,
        (double)INTEGRAL_LIMIT,
        (double)GYRO_RATE_FILTER_ALPHA);

    configure_i2c();
    wake_mpu6050();
    configure_mpu6050_filter();
    gyro_offset_dps = calibrate_gyro_offset_dps();
    configure_motor_pwm();
    configure_motor_direction();
    configure_motor_brake();
    set_motor_pwm_percent(0);
    set_motor_brake(true);
    print_plot_header_once();

    while (1) {
        const imu_state_t imu_state = read_imu_state();
        const TickType_t current_tick = xTaskGetTickCount();
        const int64_t current_time_us = esp_timer_get_time();
        const float gyro_dps = imu_state.gyro_rate_dps - gyro_offset_dps;
        float gyro_for_control_dps = 0.0f;
        float dt_seconds = (float)(current_time_us - previous_time_us) / 1000000.0f;
        float control_signal = 0.0f;
        uint32_t duty_percent = 0;
        bool brake_enabled = false;
        bool reverse_direction = false;

        if (dt_seconds <= 0.0f || dt_seconds > 0.1f) {
            dt_seconds = 0.01f;
        }

        previous_time_us = current_time_us;

        if (!angle_is_initialized) {
            filtered_angle_degrees = imu_state.tilt_y_degrees;
            angle_is_initialized = true;
        } else {
            const float gyro_angle_degrees = filtered_angle_degrees + (gyro_dps * dt_seconds);
            filtered_angle_degrees =
                (COMPLEMENTARY_GYRO_WEIGHT * gyro_angle_degrees) +
                ((1.0f - COMPLEMENTARY_GYRO_WEIGHT) * imu_state.tilt_y_degrees);
        }

        if (!gyro_filter_is_initialized) {
            filtered_gyro_dps = gyro_dps;
            gyro_filter_is_initialized = true;
        } else {
            filtered_gyro_dps =
                (GYRO_RATE_FILTER_ALPHA * gyro_dps) +
                ((1.0f - GYRO_RATE_FILTER_ALPHA) * filtered_gyro_dps);
        }

        gyro_for_control_dps = filtered_gyro_dps;

        const float angle_degrees = filtered_angle_degrees;
        const bool outside_warn_angle = fabsf(angle_degrees) > WARN_ANGLE_DEGREES;
        const bool outside_safe_angle = fabsf(angle_degrees) > DISABLE_ANGLE_DEGREES;
        const bool close_to_upright = fabsf(angle_degrees) < BRAKE_ANGLE_DEGREES;
        const bool control_saturated = fabsf(control_signal) >= CONTROL_LIMIT;

        if (outside_warn_angle && !outside_safe_angle && !warn_state_announced) {
            ESP_LOGW(
                TAG,
                "Balance warning. Angle %.2f is outside +/-%.2f degrees.",
                (double)angle_degrees,
                (double)WARN_ANGLE_DEGREES);
            warn_state_announced = true;
        } else if (!outside_warn_angle && warn_state_announced) {
            ESP_LOGI(
                TAG,
                "Balance warning cleared. Angle %.2f is back within +/-%.2f degrees.",
                (double)angle_degrees,
                (double)WARN_ANGLE_DEGREES);
            warn_state_announced = false;
        }

        if (outside_safe_angle) {
            warn_state_announced = false;
            brake_enabled = true;
            motor_speed_estimate = 0.0f;
            angle_integral = 0.0f;
            set_motor_pwm_percent(0);
            set_motor_brake(true);
            if (!limit_state_announced) {
                ESP_LOGW(
                    TAG,
                    "Balance disabled. Angle %.2f is outside +/-%.2f degrees. Waiting to re-enter the safe range.",
                    (double)angle_degrees,
                    (double)DISABLE_ANGLE_DEGREES);
                limit_state_announced = true;
            }
        } else if (close_to_upright) {
            limit_state_announced = false;
            brake_enabled = true;
            motor_speed_estimate = 0.0f;
            angle_integral = 0.0f;
            set_motor_pwm_percent(0);
            set_motor_brake(true);
        } else {
            if (limit_state_announced) {
                ESP_LOGI(
                    TAG,
                    "Balance re-enabled. Angle %.2f is back within +/-%.2f degrees.",
                    (double)angle_degrees,
                    (double)DISABLE_ANGLE_DEGREES);
                limit_state_announced = false;
            }
            if (!control_saturated) {
                angle_integral += angle_degrees * dt_seconds;
                angle_integral = clamp_symmetric(angle_integral, INTEGRAL_LIMIT);
            }

            control_signal = -((BALANCE_KP * angle_degrees) +
                               (BALANCE_KI * angle_integral) +
                               (BALANCE_KD * gyro_for_control_dps) -
                               (BALANCE_KS * motor_speed_estimate));
            control_signal = clamp_float(control_signal, -CONTROL_LIMIT, CONTROL_LIMIT);
            duty_percent = control_to_duty_percent(control_signal);
            reverse_direction = control_signal < 0.0f;
            motor_speed_estimate += control_signal * dt_seconds;

            set_motor_direction(reverse_direction);
            set_motor_brake(false);
            set_motor_pwm_percent(duty_percent);
        }

        if (!outside_safe_angle && (current_tick - last_log_tick) >= LOG_DELAY) {
            if (PLOT_OUTPUT_ENABLED) {
                ets_printf(
                    "plot,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%lu,%d,%d\n",
                    (double)BALANCE_KP,
                    (double)BALANCE_KI,
                    (double)BALANCE_KD,
                    (double)angle_degrees,
                    (double)gyro_for_control_dps,
                    (double)motor_speed_estimate,
                    (double)control_signal,
                    (unsigned long)duty_percent,
                    brake_enabled ? 1 : 0,
                    outside_safe_angle ? 2 : (close_to_upright ? 1 : 0));
            }

            ESP_LOGI(
                TAG,
                "p=%.2f i=%.2f d=%.2f ks=%.2f i_acc=%.2f angle=%.2f gyro=%.2f speed=%.2f control=%.2f duty=%lu dir=%s brake=%s state=%s",
                (double)BALANCE_KP,
                (double)BALANCE_KI,
                (double)BALANCE_KD,
                (double)BALANCE_KS,
                (double)angle_integral,
                (double)angle_degrees,
                (double)gyro_for_control_dps,
                (double)motor_speed_estimate,
                (double)control_signal,
                (unsigned long)duty_percent,
                reverse_direction ? "r" : "f",
                brake_enabled ? "on" : "off",
                outside_safe_angle ? "limit" : (close_to_upright ? "deadband" : "drive"));
            last_log_tick = current_tick;
        }

        vTaskDelay(CONTROL_LOOP_DELAY);
    }
}
