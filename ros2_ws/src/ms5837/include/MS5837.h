#ifndef MS5837_H_BLUEROBOTICS
#define MS5837_H_BLUEROBOTICS

#include <cstdint>
#include <chrono>
#include <thread>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <string>
#include <iostream>

class MS5837 {
public:
    static const float Pa;
    static const float bar;
    static const float mbar;

    static const uint8_t MS5837_30BA;
    static const uint8_t MS5837_02BA;
    static const uint8_t MS5837_UNRECOGNISED;

    MS5837();
    ~MS5837();

    bool init(const std::string& i2c_device);
    void setModel(uint8_t model);
    uint8_t getModel();
    void setFluidDensity(float density);

    void readSensor();
    float pressure(float conversion = 1.0f);
    float temperature();
    float depth();
    float altitude();

private:
    int i2c_fd;
    uint16_t C[8];
    uint32_t D1_pres, D2_temp;
    int32_t TEMP;
    int32_t P;
    uint8_t _model;
    float fluidDensity;

    void calculate();
    uint8_t crc4(uint16_t n_prom[]);
    bool writeByte(uint8_t reg);
    bool readBytes(uint8_t reg, uint8_t* buffer, size_t length);
};

#endif
