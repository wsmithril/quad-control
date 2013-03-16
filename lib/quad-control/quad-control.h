#include "MPU6050.h"
#include "HMC5883L.h"
#include "BMP085.h"

class Reading {
private:
    struct Reads {
        struct { int16_t x, y, z; } accel;
        struct { int16_t x, y, z; } gyro;
        struct { int16_t x, y, z; } mag;
        int32_t temperature;
        int32_t pressure;
        uint32_t time;
    };
    static const uint8_t SIZE = 2;
    Reads reading[SIZE];

    uint8_t current_ptr;
    HMC5883L compass;
    MPU6050  accelgyro;
    BMP085   pressure;

    float declination;

public:
    Reading():
        current_ptr(SIZE - 1) {
        declination = -94.83 / 1000;
    }

    void init() {
        // initialize all sensors
        (&accelgyro)->set_gyro_scale(MPU6050::FS_250)
                    ->set_accel_scale(MPU6050::FS_2G)
                    ->init();

        (&compass)->set_mode(HMC5883L::Continuous_Measurement)
                  ->set_data_rate(HMC5883L::Rate_1500)
                  ->set_gain(HMC5883L::Gain_440)
                  ->init();

        (&pressure)->set_oss(BMP085::Ultra_High)
                   ->init();
    }

    void get_readings() {
        current_ptr = (current_ptr + 1) % SIZE;
        Reads * r = reading + current_ptr;

        accelgyro.read_raw(
                &r->accel.x, &r->accel.y, &r->accel.z,
                &r->gyro.x,  &r->gyro.y,  &r->gyro.z);
        compass.read_raw(&r->mag.x, &r->mag.y, &r->mag.z);
        if (pressure.check_reading() == BMP085::Ready)
            pressure.read_raw(&r->temperature, &r->pressure);
        r->time = millis();
    }

    void print() {
        Reads * r = reading + current_ptr;
        float ax, ay, az, mx, my, mz, gx, gy, gz, t, p;
        accelgyro.scale_accel(r->accel.x, r->accel.y, r->accel.z, &ax, &ay, &az);
        accelgyro.scale_gyro(r->gyro.x, r->gyro.y, r->gyro.z, &gx, &gy, &gz);
        compass.scale(r->mag.x, r->mag.y, r->mag.z, &mx, &my, &mz);
        pressure.scale(r->temperature, r->pressure, &t, &p);
#define show(x) Serial.print(#x " = "); Serial.print(x); Serial.print("\t")
        Serial.println();
        Serial.print("=== time: "); Serial.print(r->time); Serial.println(" ===");
        show(ax); show(ay); show(az); Serial.println();
        show(gx); show(gy); show(gz); Serial.println();
        show(mx); show(my); show(mz); Serial.println();
        Serial.print("Pressure     = "); Serial.println(p);
        Serial.print("Temperature  = "); Serial.println(t);
        Serial.print("Altitude     = "); Serial.println(pressure.altitude(t, p));
#undef show
    }

    void show_heading() {
        Reads * r = reading + current_ptr;
        Serial.print("Heading: "); Serial.print(atan2(r->mag.y, r->mag.x) * 180.0 / M_PI); Serial.print("\t");
    }

    void position(float * roll, float * pitch, float * heading) {
        Reads * r = reading + current_ptr;
        *roll  = atan2(r->accel.y, r->accel.z);
        *pitch = atan2(r->accel.x, r->accel.z);
        float sin_roll  = sin(*roll);
        float cos_roll  = cos(*roll);
        float sin_pitch = sin(*pitch);
        float cos_pitch = cos(*pitch);
        float xh = r->mag.x * cos_pitch - r->mag.z * sin_pitch;
        float yh = r->mag.x * sin_roll * sin_pitch + r->mag.y * cos_roll
                 + r->mag.z * sin_roll * cos_pitch;
#define fence(x) (x < 0? x + 2 * M_PI: (x > 2 * M_PI? x - 2 * M_PI: x))
        *heading = fence(atan2(yh, xh) + declination);
#undef fence
    }
};
