#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define SCL_PIN 17
#define SDA_PIN 16

// Définitions des adresses
#define MPU6050_ADDR 0x68
#define REG_PWR_MGMT_1    0x6B
#define REG_ACCEL_XOUT_H  0x3B

// Prototype des fonctions
void init_pico(void);
void mpu_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                  int16_t *temp, int16_t *gx, int16_t *gy, int16_t *gz);
void calibrate(int samples);
void led_accel(float accel_x, float accel_y, float accel_z);
void led_gyro(float gyro_x, float gyro_y, float gyro_z);

// Stockage de la calibration
int32_t ax_offset = 0, ay_offset = 0, az_offset = 0;
float accel_scale = 1.0f; // facteur global

int main() {
    stdio_init_all();
    init_pico();

    // wake MPU
    uint8_t wake[2] = {REG_PWR_MGMT_1, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, wake, 2, false);

    sleep_ms(500);
    printf("Starting calibration (place sensor flat and keep still)...\n");
    calibrate(500); // exemple : 500 échantillons

    while (1) {
        int16_t ax_raw, ay_raw, az_raw, temp, gx_raw, gy_raw, gz_raw;
        mpu_read_raw(&ax_raw,&ay_raw,&az_raw,&temp,&gx_raw,&gy_raw,&gz_raw);

        // remove offsets, apply global scale
        float ax_corr = ((float)ax_raw - (float)ax_offset) * accel_scale;
        float ay_corr = ((float)ay_raw - (float)ay_offset) * accel_scale;
        float az_corr = ((float)az_raw - (float)az_offset) * accel_scale;

        // convert to g
        float ax_g = ax_corr / 16384.0f;
        float ay_g = ay_corr / 16384.0f;
        float az_g = az_corr / 16384.0f;

        // gyro: subtract mean offset only (we didn't calibrate gyro gain)
        // you can compute gyro offsets similarly by averaging when static
        static int32_t gx_off=0, gy_off=0, gz_off=0;
        // on first loop we could compute them; omitted here for brevity

        float gx_dps = (float)(gx_raw - gx_off) / 131.0f;
        float gy_dps = (float)(gy_raw - gy_off) / 131.0f;
        float gz_dps = (float)(gz_raw - gz_off) / 131.0f;

        // temp convert
        float temp_c = ((float)temp)/340.0f + 36.53f;

        printf("Ax=%.3fg Ay=%.3fg Az=%.3fg | Gx=%.2fdps Gy=%.2fdps Gz=%.2fdps | T=%.2fC\n",
               ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temp_c);

        led_accel(ax_g, ay_g, az_g);
        led_gyro(gx_dps, gy_dps, gz_dps);
        sleep_ms(100);
    }
}

void init_pico(void)
{
    // init I2C
    i2c_init(I2C_PORT, 100*1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // init LED
    gpio_init(5);
    gpio_set_dir(5, true);

    gpio_init(6);
    gpio_set_dir(6, true);

    gpio_init(8);
    gpio_set_dir(8, true);

    gpio_init(28);
    gpio_set_dir(28, true);

    gpio_init(27);
    gpio_set_dir(27, true);

    gpio_init(22);
    gpio_set_dir(22, true);
}

// read raw 14 bytes (accel,temp,gyro)
void mpu_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                  int16_t *temp, int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t reg = REG_ACCEL_XOUT_H;
    uint8_t buf[14];
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buf, 14, false);

    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
    *temp = (int16_t)((buf[6] << 8) | buf[7]);
    *gx = (int16_t)((buf[8] << 8) | buf[9]);
    *gy = (int16_t)((buf[10] << 8) | buf[11]);
    *gz = (int16_t)((buf[12] << 8) | buf[13]);
}

// calibration : offsets + magnitude-based scale
void calibrate(int samples) {
    int64_t ax_sum=0, ay_sum=0, az_sum=0;
    for (int i=0;i<samples;i++) {
        int16_t ax,ay,az,temp,gx,gy,gz;
        mpu_read_raw(&ax,&ay,&az,&temp,&gx,&gy,&gz);
        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;
        sleep_ms(5);
    }
    // offsets = mean (for Z we will subtract 1g later)
    ax_offset = (int32_t)(ax_sum / samples);
    ay_offset = (int32_t)(ay_sum / samples);
    az_offset = (int32_t)(az_sum / samples);

    // compute magnitude of mean vector and compute global scale so mag -> 16384
    float ax_mean = (float)ax_offset;
    float ay_mean = (float)ay_offset;
    float az_mean = (float)az_offset;
    float mag = sqrtf(ax_mean*ax_mean + ay_mean*ay_mean + az_mean*az_mean);
    if (mag > 0.001f) {
        accel_scale = 16384.0f / mag;
    } else {
        accel_scale = 1.0f;
    }

    // adjust az_offset to remove gravity contribution after scaling:
    // we want (az_mean - az_offset_scaled) * scale = 16384 => but we keep az_offset as mean and scale later
    printf("Calib done: ax_off=%d ay_off=%d az_off=%d scale=%.5f mag=%.2f\n",
           ax_offset, ay_offset, az_offset, accel_scale, mag);
}

// Visualisation sur une LED RGB des trois axes de l'accelerometre
void led_accel(float accel_x, float accel_y, float accel_z)
{
    if(accel_x > 0.300 || accel_x < -0.300)
        {
            gpio_put(5, 1);
        }
    else {
            gpio_put(5, 0);
        }

    if(accel_y > 0.300 || accel_y < -0.300)
        {
            gpio_put(6, 1);
        }
    else {
            gpio_put(6, 0);
        }

    if(accel_z < -1.8)
        {
            gpio_put(8, 1);
        }
    else {
            gpio_put(8, 0);
        }
}

// Visualisation sur une LED RGB des trois axes du gyroscope
void led_gyro(float gyro_x, float gyro_y, float gyro_z)
{
    if(gyro_x > 50 || gyro_x < -50)
        {
            gpio_put(28, 1);
        }
    else {
            gpio_put(28, 0);
        }

    if(gyro_y > 50 || gyro_y < -50)
        {
            gpio_put(27, 1);
        }
    else {
            gpio_put(27, 0);
        }

    if(gyro_z > 50 || gyro_z < -50)
        {
            gpio_put(22, 1);
        }
    else {
            gpio_put(22, 0);
        }
}