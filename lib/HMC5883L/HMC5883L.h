#include <Wire.h>
#include <stdint.h>


class HMC5883L {
private:
    // I2C address
    static const uint8_t DEFAULT_I2C_ADDR = 0x1e;

    // register address
    static const uint8_t ADDR_CONFIG_A  = 0x00;
    static const uint8_t ADDR_CONFIG_B  = 0x01;
    static const uint8_t ADDR_MODE      = 0x02;
    static const uint8_t ADDR_X_MSB     = 0x03;
    static const uint8_t ADDR_X_LSB     = 0x04;
    static const uint8_t ADDR_Y_MSB     = 0x05;
    static const uint8_t ADDR_Y_LSB     = 0x06;
    static const uint8_t ADDR_Z_MSB     = 0x07;
    static const uint8_t ADDR_Z_LSB     = 0x08;
    static const uint8_t ADDR_STATUS    = 0x09;
    static const uint8_t ADDR_ID_A      = 0x10;
    static const uint8_t ADDR_ID_B      = 0x11;
    static const uint8_t ADDR_ID_C      = 0x12;

    // const stored in ID register, namely "H43"
    static const uint8_t ID_A = 0x48; // 'H'
    static const uint8_t ID_B = 0x34; // '4'
    static const uint8_t ID_C = 0x33; // '3'

    float declination;

    static void i2c_write(uint8_t addr, uint8_t data) {
        Wire.beginTransmission(DEFAULT_I2C_ADDR);
        Wire.write(addr);
        Wire.write(data);
        Wire.endTransmission();
    }

    static void i2c_read(uint8_t addr, uint8_t * buffer, uint8_t s) {
        Wire.beginTransmission(DEFAULT_I2C_ADDR);
        Wire.write(addr);
        Wire.endTransmission();
        Wire.requestFrom(DEFAULT_I2C_ADDR, s);
        for (int i = 0; Wire.available() && i < s; i++) buffer[i] = Wire.read();
    }

public:
    // configurations
    enum MEASURE_AVG {
        Avg1 = 0x00,
        Avg2 = 0x01,
        Avg4 = 0x02,
        Avg8 = 0x03, // default
    };
    MEASURE_AVG measure_avg;

    enum DATA_RATE {
        Rate_0075 = 0x00,
        Rate_0150 = 0x01,
        Rate_0300 = 0x02,
        Rate_0750 = 0x03,
        Rate_1500 = 0x04, // default
        Rate_3000 = 0x05,
        Rate_7500 = 0x06,
        Not_Used  = 0x07,
    };
    DATA_RATE data_rate;

    enum MEASUREMENT_MODE {
        Normal        = 0x00,
        Positive_Bias = 0x01,
        Negative_Bias = 0x02,
        Reversed      = 0x03,
    };
    MEASUREMENT_MODE measurement_mode;

    enum GAIN {
        Gain_1370 = 0x00,
        Gain_1090 = 0x01, // default
        Gain_820  = 0x02,
        Gain_660  = 0x03,
        Gain_440  = 0x04,
        Gain_390  = 0x05,
        Gain_330  = 0x06,
        Gain_230  = 0x07,
    };
    GAIN gain;

    enum MODE {
        Continuous_Measurement = 0x00,
        Single_Measurement     = 0x01,
        Idle                   = 0x02,
    };
    MODE mode;

    HMC5883L():
        measure_avg(Avg8),
        mode(Single_Measurement),
        measurement_mode(Normal),
        data_rate(Rate_1500),
        gain(Gain_1090),
        declination(0) {};

    HMC5883L * set_measure_avg(MEASURE_AVG ma) {
        measure_avg = ma;
        return this;
    }

    HMC5883L * set_data_rate(DATA_RATE dr) {
        data_rate = dr;
        return this;
    }

    HMC5883L * set_gain(GAIN g) {
        gain = g;
        return this;
    }

    HMC5883L * set_measure_mode(MEASUREMENT_MODE m) {
        measurement_mode = m;
        return this;
    }

    HMC5883L * set_mode(MODE m) {
        mode = m;
        return this;
    }

    HMC5883L * set_declination(float d) {
        declination = d;
        return this;
    }

    // init the device
    int init() {
        uint8_t conf_A = (0x80) | (measure_avg << 5) | (data_rate << 2);
        uint8_t conf_B = (0x00) | (gain << 5);
        uint8_t m      = (0x80) | (mode);

        i2c_write(ADDR_CONFIG_A, conf_A);
        i2c_write(ADDR_CONFIG_B, conf_B);
        i2c_write(ADDR_MODE, m);
        return 0;
    }

    int read_raw(int16_t * x, int16_t * y, int16_t * z) {
        uint8_t buffer[6], i = 0;
        i2c_read(ADDR_X_MSB, buffer, 6);

        *x = (buffer[0] << 8) + buffer[1];
        *z = (buffer[2] << 8) + buffer[3];
        *y = (buffer[4] << 8) + buffer[5];
        return 0;
    }

    int read_scaled(float * x, float * y, float * z) {
        static const float scale[] = {0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1};
        float factor = scale[gain];
        int16_t rx, ry, rz;

        read_raw(&rx, &ry, &rz);
        *x = rx == -4096? 0: rx * factor / 2048;
        *y = ry == -4096? 0: ry * factor / 2048;
        *z = rz == -4096? 0: rz * factor / 2048;
        return 0;
    }
};
