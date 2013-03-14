#include "MPU6050.h"
#include "HMC5883L.h"
#include "BMP085.h"

class Reading {
private:
    struct Reads {
        struct { int16_t x, y, z; } accel;
        struct { int16_t x, y, z; } gyro;
        struct { float x, y, z; } meg;
        float    temperature;
        uint32_t pressure;
        uint32_t time;
    };
    static const uint8_t SIZE = 2;
    Reads reading[SIZE];

    uint8_t current_ptr;

public:
    Reading():
        current_ptr(SIZE - 1) {};

    void get_readings(HMC5883L * compass,
                      MPU6050  * accelgyro,
                      BMP085   * pressure) {
        current_ptr = (current_ptr + 1) % SIZE;
        Reads * r = reading + current_ptr;

        accelgyro->getMotion6(
            &r->accel.x, &r->accel.y, &r->accel.z,
            &r->gyro.x,  &r->gyro.y,  &r->gyro.z);
        compass->read_scaled(&r->meg.x, &r->meg.y, &r->meg.z);
        if (pressure->check_reading() == BMP085::Ready)
            pressure->calculate(&r->temperature, &r->pressure);
        r->time = millis();
    }

    void print() {
        Reads * r = reading + current_ptr;
        Serial.println();
        Serial.print("=== time: "); Serial.print(r->time); Serial.println(" ===");
        Serial.print("ax = "); Serial.print(r->accel.x); Serial.print("\t");
        Serial.print("ay = "); Serial.print(r->accel.y); Serial.print("\t");
        Serial.print("az = "); Serial.print(r->accel.z); Serial.println();
        Serial.print("gx = "); Serial.print(r->gyro.x); Serial.print("\t");
        Serial.print("gy = "); Serial.print(r->gyro.y); Serial.print("\t");
        Serial.print("gz = "); Serial.print(r->gyro.z); Serial.println();
        Serial.print("mx = "); Serial.print(r->meg.x); Serial.print("\t");
        Serial.print("my = "); Serial.print(r->meg.y); Serial.print("\t");
        Serial.print("mz = "); Serial.print(r->meg.z); Serial.println();
        Serial.print("Pressure     = "); Serial.println(r->pressure);
        Serial.print("Temperature  = "); Serial.println(r->temperature);

    }
};
