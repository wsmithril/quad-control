#include <Wire.h>
#include <stdint.h>

class MPU6050 {
public:
    enum AD0 { AD0_Low = 0, AD0_High = 1 };
    enum GYRO_FS  { FS_250 = 0, FS_500 = 1, FS_1000 = 2, FS_2000 = 3 };
    enum ACCEL_FS { FS_2G  = 0, FS_4G  = 1, FS_8G   = 2, FS_16G  = 3 };

private:
    static const uint8_t I2C_ADDR = 0x68;
    AD0 ad0;
    GYRO_FS  gyro_scale;
    ACCEL_FS accel_scale;

    static void i2c_write(uint8_t addr, uint8_t data) {
        Wire.beginTransmission(I2C_ADDR);
        Wire.write(addr);
        Wire.write(data);
        Wire.endTransmission();
    }

    static void i2c_read(uint8_t addr, uint8_t buffer[], uint8_t s) {
        Wire.beginTransmission(I2C_ADDR);
        Wire.write(addr);
        Wire.endTransmission();
        Wire.requestFrom(I2C_ADDR, s);
        for (int i = 0; Wire.available() && i < s; i++) buffer[i] = Wire.read();
    }

public:

    MPU6050():
        ad0(AD0_Low),
        gyro_scale(FS_250),
        accel_scale(FS_4G) {}

    MPU6050 * init() {
        i2c_write(0x1A, 0x00); // disable EXT_SYNC, DLPF_CFG = 0
        i2c_write(0x1B, 0x00 | (gyro_scale  << 3));
        i2c_write(0x1C, 0x00 | (accel_scale << 3));
        i2c_write(0x6B, 0x01); // wake and set reference to Gx PLL
        return this;
    }

    MPU6050 * set_gyro_scale(GYRO_FS g) {
        gyro_scale = g;
        return this;
    }

    MPU6050 * set_accel_scale(ACCEL_FS g) {
        accel_scale = g;
        return this;
    }

    MPU6050 * set_AD0(AD0 a) {
        ad0 = a;
        return this;
    }

    void read_raw(
            int16_t * ax, int16_t * ay, int16_t * az,
            int16_t * gx, int16_t * gy, int16_t * gz) {
        uint8_t buffer[6];
#define i16(n) (((int16_t)buffer[n] << 8) | buffer[n + 1])
        i2c_read(0x3B, buffer, 6);
        *ax = i16(0);
        *ay = i16(2);
        *az = i16(4);
        i2c_read(0x43, buffer, 6);
        static struct {int16_t x, y, z;} drift = {450, -940, 250};
        *gx = i16(0) - (drift.x >> gyro_scale);
        *gy = i16(2) - (drift.y >> gyro_scale);
        *gz = i16(4) - (drift.z >> gyro_scale);
#undef i16
    }

    void read_scaled(
            float * ax, float * ay, float * az,
            float * gx, float * gy, float * gz) {
        int16_t xa, ya, za, xg, yg, zg;
        read_raw(&xa, &ya, &za, &xg, &yg, &zg);
        scale_accel(xa, ya, za, ax, ay, az);
        scale_gyro(xg, yg, zg, gx, gy, gz);
    }

    void scale_gyro(
            int16_t  x, int16_t  y, int16_t  z,
            float * ox, float * oy, float * oz) {
        static float gf[] = { 131.064, 65.532, 32.768, 16.384 };
        *ox = x / gf[gyro_scale];
        *oy = y / gf[gyro_scale];
        *oz = z / gf[gyro_scale];
    }

    void scale_accel(
            int16_t  ax, int16_t  ay, int16_t  az,
            float * oax, float * oay, float * oaz) {
        static float af[] = { 16384.0, 8192.0, 4096.0, 2048.0 };
        *oax = ax / af[accel_scale];
        *oay = ay / af[accel_scale];
        *oaz = az / af[accel_scale];
    }

    float read_temperature() {
        uint8_t buffer[2];
        i2c_read(0x41, buffer, 2);
        return ((buffer[0] << 8) | buffer[1]) / 340.0 + 36.53;
    }
};
