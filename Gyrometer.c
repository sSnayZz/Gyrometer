#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define SDA_PIN 20
#define SCL_PIN 21

#define MPU6050_ADDR 0x68
#define REG_PWR_MGMT_1 0x6B
#define REG_WHO_AM_I   0x75
#define REG_ACCEL_XOUT_H 0x3B

#define LED_PIN 2

int main() {
    stdio_init_all();

    // Init I2C0 à 100 kHz
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Init LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, true);

    sleep_ms(2000); // attendre TeraTerm

    printf("=== Test MPU6050 avec LED ===\n");

    // Wake up MPU6050
    uint8_t buf[2];
    buf[0] = REG_PWR_MGMT_1;
    buf[1] = 0x00; // clear sleep bit
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);

    // Lire WHO_AM_I
    uint8_t reg = REG_WHO_AM_I;
    uint8_t who = 0;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, &who, 1, false);

    printf("WHO_AM_I = 0x%02X (attendu 0x68)\n", who);

    while (1) {
        // Lire Accel + Gyro (14 octets)
        reg = REG_ACCEL_XOUT_H;
        uint8_t data[14];
        i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
        i2c_read_blocking(I2C_PORT, MPU6050_ADDR, data, 14, false);

        int16_t accel_x = (data[0] << 8) | data[1];
        int16_t accel_y = (data[2] << 8) | data[3];
        int16_t accel_z = (data[4] << 8) | data[5];
        int16_t temp    = (data[6] << 8) | data[7];
        int16_t gyro_x  = (data[8] << 8) | data[9];
        int16_t gyro_y  = (data[10] << 8) | data[11];
        int16_t gyro_z  = (data[12] << 8) | data[13];

        int16_t real_temp = (temp / 340.0) + 36.53;

        printf("Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d | Temp=%d\n",
               accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, real_temp);

        // Allumer LED si accel_x > seuil
        if (accel_x > 10000 || accel_x < -10000) {
            gpio_put(LED_PIN, 1);
        } else {
            gpio_put(LED_PIN, 0);
        }

        sleep_ms(500);
    }
}
