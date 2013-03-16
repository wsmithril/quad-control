#include <Wire.h>
#include <stdint.h>
#include <limits.h>

// do this the nonblocking way
class BMP085 {
private:
    static const uint8_t I2C_ADDRESS = 0x77;
    static const int32_t p0 = 101325;

    static void i2c_write(uint8_t addr, uint8_t data) {
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(addr);
        Wire.write(data);
        Wire.endTransmission();
    }

    static void i2c_read(uint8_t addr, uint8_t * buffer, uint8_t s) {
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(addr);
        Wire.endTransmission();
        Wire.requestFrom(I2C_ADDRESS, s);
        for (int i = 0; Wire.available() && i < s; i++) buffer[i] = Wire.read();
    }

    void read_calibration() {
        uint8_t buffer[22];
        i2c_read(0xaa, buffer, 22);
#define int16(x) ((int16_t)(buffer[x] << 8) | buffer[x + 1])
        ac1 = int16(0);
        ac2 = int16(2);
        ac3 = int16(4);
        ac4 = int16(6);
        ac5 = int16(8);
        ac6 = int16(10);
        b1  = int16(12);
        b2  = int16(14);
        mb  = int16(16);
        mc  = int16(18);
        md  = int16(20);
#undef int16
    }

    struct {
        int32_t ut;
        int32_t up;
    } reading;

    unsigned long last_check;

    int16_t  ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;

public:

    enum OSS {
        Ultra_Low  = 0x00,
        Standard   = 0x01,
        High       = 0x02,
        Ultra_High = 0x03,
    };
    OSS oss;

    enum { Ready = 0, Waiting_UT, Waiting_UP } state;

    BMP085():
        oss(Ultra_Low),
        state(Ready) {};

    BMP085 * set_oss(OSS o) {
        oss = o;
        return this;
    }

    void init() {
        read_calibration();
        last_check = micros();
    }

    // non-blocking read()
    int check_reading() {
        uint8_t buffer[3];
        static const uint16_t wait_ut = 4500;
        static const uint16_t wait_up[] = { 4500, 7500, 13500, 25500 };
        unsigned long now = micros();

        // since overflow happens every 70 min, we needs to check it
#define between(a, b) (a >= b? ((a) - (b)): ((a) + (ULONG_MAX - (b))))

        switch (state) {
            case Ready:
                i2c_write(0xF4, 0x2E);
                last_check = now;
                state = Waiting_UT;
            case Waiting_UT:
                if (between(now, last_check) <= wait_ut) goto next;
                // now we can get the result
                i2c_read(0xF6, buffer, 2);
                reading.ut = (buffer[0] << 8) | buffer[1];
                i2c_write(0xF4, (0x34 | (oss << 6)));
                state = Waiting_UP;
                last_check = now;
            case Waiting_UP:
                if (between(now, last_check) <= wait_up[oss]) goto next;
                i2c_read(0xF6, buffer, 3);
                reading.up = ((buffer[0] << 16) | (buffer[1] << 8)
                           | (buffer[2])) >> (8 - oss);
                state = Ready;
        }
next:
        return state;
    }

    // THE TYPE CONVERSION ISKILLING ME!!!!!!!!!!!!!!!
    int read_raw(int32_t * temp, int32_t * presure) {
        if (state != Ready) return state;

        int32_t X1, X2, X3, b3, b5, b6, b7, P;
        uint32_t b4;

        X1 = ((reading.ut - ac6) * ac5) >> 15;
        X2 = ((int32_t)mc << 11) / (X1 + md);
        b5 = X1 + X2;
        *temp = ((b5 + 8) >> 4);
        b6 = b5 - 4000;
        X1 = ((int32_t)b2 * ((b6 * b6) >> 12)) >> 11;
        X2 = (ac2 * b6) >> 11;
        X3 = X1 + X2;
        b3 = (((((int32_t)ac1 << 2) + X3) << oss) + 2) >> 2;
        X1 = (ac3 * b6) >> 13;
        X1 = ((int32_t)b1 * ((b6 * b6) >> 12)) >> 16;
        X3 = (X1 + X2 + 2) >> 2;
        b4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
        b7 = (reading.up - b3) * (50000UL >> oss);
        P  = b7 < 0x08000000? b7 * 2 / b4: b7 / b4 * 2;
        X1 = (P >> 8) * (P >> 8);
        X1 = (X1 * 3038) >> 16;
        X2 = (-7357 * P) >> 16;
        *presure = P + ((X1 + X2 + 3791) >> 4);
    }

    static void scale(int32_t tin, int32_t pin, float * t, float * p) {
        *p = pin / 1.0;
        *t = tin / 10.0;
    }

    void read_scale(float * t, float * p) {
        int32_t rt, rp;
        read_raw(&rt, &rp);
        scale(rt, rp, t, p);
    }

    static float altitude(int32_t t, int32_t p) {
        float p1 = p * (2371.5 + 150) / (2371.5 + t);
        return 44330 * (1 - pow(p / p1, (1 / 5.255)));
    }

    static float altitude(float t, float p) {
        float p1 = p0 / (237.15 + 15) * (237.15 + t);
        return 44330 * (1 - pow(p / p1, (1 / 5.255)));
    }
};
