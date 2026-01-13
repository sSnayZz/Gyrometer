#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ssd1306.h"

#define MPU_I2C_PORT i2c0
#define OLED_I2C_PORT i2c1
#define MPU_SCL_PIN 17
#define MPU_SDA_PIN 16
#define OLED_SDA_PIN 14
#define OLED_SCL_PIN 15
//#define FS_PIN 27

// Définitions des adresses
#define MPU6050_ADDR 0x68
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B

// Prototype des fonctions
void init_pico(void);
void mpu_read_raw(int16_t *ax, int16_t *ay, int16_t *az, int16_t *temp, int16_t *gx, int16_t *gy, int16_t *gz);
void calibrate(int samples);
void led_accel(float accel_x, float accel_y, float accel_z);
void led_gyro(float gyro_x, float gyro_y, float gyro_z);
void affichage_axes(float ax_g, float ay_g, float az_g, float gx_dps, float gy_dps, float gz_dps, float temp_c);
//uint32_t read_pulse_us(uint gpio);
//void led_fs(uint LED_PIN, uint RX_PIN);
void mpu_calc(void);

volatile float ax, ay, az, gx, gy, gz, temperature;

// Stockage de la calibration
int32_t ax_offset = 0, ay_offset = 0, az_offset = 0;
float accel_scale = 1.0f; // facteur global
// Création de l’objet écran
ssd1306_t disp;

int main()
{
    ssd1306_clear(&disp);

    stdio_init_all();
    init_pico();

    // wake MPU
    uint8_t wake[2] = {REG_PWR_MGMT_1, 0x00};
    i2c_write_blocking(MPU_I2C_PORT, MPU6050_ADDR, wake, 2, false);

    ssd1306_init(&disp, 128, 64, false, 0x3C, i2c1); // adresse par défaut 0x3C

    sleep_ms(500);
    printf("Starting calibration (place sensor flat and keep still)...\n");
    
    ssd1306_draw_string(&disp, 8, 32, true, "Calibration...");
    ssd1306_show(&disp);

    calibrate(500); // exemple : 500 échantillons

    char buffer[24];

    while (1)
    {
        mpu_calc();
        affichage_axes(ax, ay, az, gx, gy, gz, temperature);
        led_accel(ax, ay, az);
        led_gyro(gx, gy, gz);
        //led_fs(22, FS_PIN);

        ssd1306_clear(&disp);

        ssd1306_draw_string(&disp, 0, 0, true, "MPU6050 - Donnees");

        sprintf(buffer, "AX: %2.2f g", ax);
        ssd1306_draw_string(&disp, 2, 16, true, buffer);
        
        sprintf(buffer, "AY: %2.2f g", ay);
        ssd1306_draw_string(&disp, 2, 24, true, buffer);
        
        sprintf(buffer, "AZ: %2.2f g", az);
        ssd1306_draw_string(&disp, 2, 32, true, buffer);
        
        sprintf(buffer, "GX: %2.1f dps", gx);
        ssd1306_draw_string(&disp, 2, 40, true, buffer);
        
        sprintf(buffer, "GY: %2.1f dps", gy);
        ssd1306_draw_string(&disp, 2, 48, true, buffer);

        sprintf(buffer, "GZ: %2.1f dps", gz);
        ssd1306_draw_string(&disp, 2, 56, true, buffer);

        ssd1306_show(&disp);

        sleep_ms(250);
    }
}

void init_pico(void)
{
    // init I2C MPU
    i2c_init(MPU_I2C_PORT, 100 * 1000);
    gpio_set_function(MPU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MPU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MPU_SDA_PIN);
    gpio_pull_up(MPU_SCL_PIN);

    // Initialisation I2C OLED
    i2c_init(OLED_I2C_PORT, 400000);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);

    // init LED
    gpio_init(9);
    gpio_set_dir(9, true);

    gpio_init(10);
    gpio_set_dir(10, true);

    gpio_init(11);
    gpio_set_dir(11, true);

    gpio_init(22);
    gpio_set_dir(22, true);

    gpio_init(21);
    gpio_set_dir(21, true);

    gpio_init(20);
    gpio_set_dir(20, true);

    //gpio_init(FS_PIN);
    //gpio_set_dir(FS_PIN, GPIO_IN);
}

void affichage_axes(float ax, float ay, float az, float gx, float gy, float gz, float temp)
{
    printf("Ax=%.3fg Ay=%.3fg Az=%.3fg | Gx=%.2fdps Gy=%.2fdps Gz=%.2fdps | T=%.2fC\n", ax, ay, az, gx, gy, gz, temp);
}

// read raw 14 bytes (accel,temp,gyro)
void mpu_read_raw(int16_t *ax, int16_t *ay, int16_t *az, int16_t *temp, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t reg = REG_ACCEL_XOUT_H;
    uint8_t buf[14];
    i2c_write_blocking(MPU_I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(MPU_I2C_PORT, MPU6050_ADDR, buf, 14, false);

    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
    *temp = (int16_t)((buf[6] << 8) | buf[7]);
    *gx = (int16_t)((buf[8] << 8) | buf[9]);
    *gy = (int16_t)((buf[10] << 8) | buf[11]);
    *gz = (int16_t)((buf[12] << 8) | buf[13]);
}

// calibration : offsets + magnitude-based scale
void calibrate(int samples)
{
    int64_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    for (int i = 0; i < samples; i++)
    {
        int16_t ax, ay, az, temp, gx, gy, gz;
        mpu_read_raw(&ax, &ay, &az, &temp, &gx, &gy, &gz);
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
    float mag = sqrtf(ax_mean * ax_mean + ay_mean * ay_mean + az_mean * az_mean);
    if (mag > 0.001f)
    {
        accel_scale = 16384.0f / mag;
    }
    else
    {
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
    if (accel_x > 0.300 || accel_x < -0.300)
    {
        gpio_put(9, 1);
    }
    else
    {
        gpio_put(9, 0);
    }

    if (accel_y > 0.300 || accel_y < -0.300)
    {
        gpio_put(10, 1);
    }
    else
    {
        gpio_put(10, 0);
    }

    if (accel_z < -1.8)
    {
        gpio_put(11, 1);
    }
    else
    {
        gpio_put(11, 0);
    }
}

// Visualisation sur une LED RGB des trois axes du gyroscope
void led_gyro(float gyro_x, float gyro_y, float gyro_z)
{
    if (gyro_x > 50 || gyro_x < -50)
    {
        gpio_put(21, 1);
    }
    else
    {
        gpio_put(21, 0);
    }

    if (gyro_y > 50 || gyro_y < -50)
    {
        gpio_put(22, 1);
    }
    else
    {
        gpio_put(22, 0);
    }

    if (gyro_z > 50 || gyro_z < -50)
    {
        gpio_put(20, 1);
    }
    else
    {
        gpio_put(20, 0);
    }
}
/*
// mesure la largeur d'impulsion en microsecondes
uint32_t read_pulse_us(uint gpio)
{
    // attendre front montant
    while (gpio_get(gpio) == 0)
        ;

    // début impulsion
    absolute_time_t start = get_absolute_time();
    // attendre passage à 0 → fin impulsion
    while (gpio_get(gpio) == 1)
        ;

    absolute_time_t end = get_absolute_time();

    return absolute_time_diff_us(start, end);
}
*/

/*void led_fs(uint LED_PIN, uint RX_PIN)
{
    uint32_t pulse = read_pulse_us(RX_PIN);

    // SWa bas ≈ 1000 µs, haut ≈ 2000 µs
    if (pulse > 1500)
    {
        gpio_put(LED_PIN, 1); // LED ON
    }
    else
    {
        gpio_put(LED_PIN, 0); // LED OFF
    }
}
*/
void mpu_calc(void)
{
    int16_t ax_raw, ay_raw, az_raw, temp, gx_raw, gy_raw, gz_raw;
    mpu_read_raw(&ax_raw, &ay_raw, &az_raw, &temp, &gx_raw, &gy_raw, &gz_raw);

    // remove offsets, apply global scale
    float ax_corr = ((float)ax_raw - (float)ax_offset) * accel_scale;
    float ay_corr = ((float)ay_raw - (float)ay_offset) * accel_scale;
    float az_corr = ((float)az_raw - (float)az_offset) * accel_scale;

    // convert to g
    float ax_g = ax_corr / 16384.0f;
    float ay_g = ay_corr / 16384.0f;
    float az_g = az_corr / 16384.0f;

    ax = ax_g;
    ay = ay_g;
    az = az_g;

    // gyro: subtract mean offset only (we didn't calibrate gyro gain)
    // you can compute gyro offsets similarly by averaging when static
    static int32_t gx_off = 0, gy_off = 0, gz_off = 0;
    // on first loop we could compute them; omitted here for brevity

    float gx_dps = (float)(gx_raw - gx_off) / 131.0f;
    float gy_dps = (float)(gy_raw - gy_off) / 131.0f;
    float gz_dps = (float)(gz_raw - gz_off) / 131.0f;

    gx = gx_dps;
    gy = gy_dps;
    gz = gz_dps;

    // temp convert
    float temp_c = ((float)temp) / 340.0f + 36.53f;

    temperature = temp_c;
}