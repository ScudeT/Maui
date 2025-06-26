
#include "QuaternionFilter.h"
#include "MPU9250RegisterMap.h"

#include "Wire.h"

#include <memory>

enum class ACCEL_FS_SEL {
    A2G,
    A4G,
    A8G,
    A16G
};
enum class GYRO_FS_SEL {
    G250DPS,
    G500DPS,
    G1000DPS,
    G2000DPS
};
enum class MAG_OUTPUT_BITS {
    M14BITS,
    M16BITS
};

enum class FIFO_SAMPLE_RATE : uint8_t {
    SMPL_1000HZ,
    SMPL_500HZ,
    SMPL_333HZ,
    SMPL_250HZ,
    SMPL_200HZ,
    SMPL_167HZ,
    SMPL_143HZ,
    SMPL_125HZ,
};

enum class GYRO_DLPF_CFG : uint8_t {
    DLPF_250HZ,
    DLPF_184HZ,
    DLPF_92HZ,
    DLPF_41HZ,
    DLPF_20HZ,
    DLPF_10HZ,
    DLPF_5HZ,
    DLPF_3600HZ,
};

enum class ACCEL_DLPF_CFG : uint8_t {
    DLPF_218HZ_0,
    DLPF_218HZ_1,
    DLPF_99HZ,
    DLPF_45HZ,
    DLPF_21HZ,
    DLPF_10HZ,
    DLPF_5HZ,
    DLPF_420HZ,
};

static constexpr uint8_t MPU9250_WHOAMI_DEFAULT_VALUE {0x71};
static constexpr uint8_t MPU9255_WHOAMI_DEFAULT_VALUE {0x73};
static constexpr uint8_t MPU6500_WHOAMI_DEFAULT_VALUE {0x70};

struct MPU9250Setting {
    ACCEL_FS_SEL accel_fs_sel {ACCEL_FS_SEL::A2G};
    GYRO_FS_SEL gyro_fs_sel {GYRO_FS_SEL::G250DPS};
    MAG_OUTPUT_BITS mag_output_bits {MAG_OUTPUT_BITS::M16BITS};
    FIFO_SAMPLE_RATE fifo_sample_rate {FIFO_SAMPLE_RATE::SMPL_200HZ};
    uint8_t gyro_fchoice {0x03};
    GYRO_DLPF_CFG gyro_dlpf_cfg {GYRO_DLPF_CFG::DLPF_41HZ};
    uint8_t accel_fchoice {0x01};
    ACCEL_DLPF_CFG accel_dlpf_cfg {ACCEL_DLPF_CFG::DLPF_45HZ};
};

class MPU9250 {
    static constexpr uint8_t MPU9250_DEFAULT_ADDRESS {0x68};  // Device address when ADO = 0
    static constexpr uint8_t AK8963_ADDRESS {0x0C};           //  Address of magnetometer
    static constexpr uint8_t AK8963_WHOAMI_DEFAULT_VALUE {0x48};
    uint8_t mpu_i2c_addr {MPU9250_DEFAULT_ADDRESS};

    // settings
    MPU9250Setting setting;
    // TODO: this should be configured!!
    static constexpr uint8_t MAG_MODE {0x06};  // 0x02 for 8 Hz, 0x06 for 100 Hz continuous magnetometer data read
    float acc_resolution {0.f};                // scale resolutions per LSB for the sensors
    float gyro_resolution {0.f};               // scale resolutions per LSB for the sensors
    float mag_resolution {0.f};                // scale resolutions per LSB for the sensors

    // Calibration Parameters
    float acc_bias[3] {0., 0., 0.};   // acc calibration value in ACCEL_FS_SEL: 2g
    float gyro_bias[3] {0., 0., 0.};  // gyro calibration value in GYRO_FS_SEL: 250dps
    float mag_bias_factory[3] {0., 0., 0.};
    float mag_bias[3] {0., 0., 0.};  // mag calibration value in MAG_OUTPUT_BITS: 16BITS
    float mag_scale[3] {1., 1., 1.};
    float magnetic_declination = 3.68;  // Italy, Lecco, 18th November

    // Temperature
    int16_t temperature_count {0};  // temperature raw count output
    float temperature {0.f};        // Stores the real internal chip temperature in degrees Celsius

    // Self Test
    float self_test_result[6] {0.f};  // holds results of gyro and accelerometer self test

    // IMU Data
    float a[3] {0.f, 0.f, 0.f};
    float g[3] {0.f, 0.f, 0.f};
    float m[3] {0.f, 0.f, 0.f};
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // vector to hold quaternion
    float rpy[3] {0.f, 0.f, 0.f};
    float lin_acc[3] {0.f, 0.f, 0.f};  // linear acceleration (acceleration with gravity component subtracted)
    QuaternionFilter quat_filter;
    size_t n_filter_iter {1};

    // Other settings
    bool has_connected {false};
    bool b_ahrs {true};
    bool b_verbose {true};

    // I2C
    I2CHandler i2cBus_;

public:
    static constexpr uint16_t CALIB_GYRO_SENSITIVITY {131};     // LSB/degrees/sec
    static constexpr uint16_t CALIB_ACCEL_SENSITIVITY {16384};  // LSB/g

    MPU9250(const uint8_t addr, const MPU9250Setting& mpu_setting, const I2CHandler &i2cBus_);
    MPU9250();
    ~MPU9250();

    void sleep(bool b);             // set mpu on sleep mode
    void verbose(const bool b);
    void ahrs(const bool b);
    void calibrateAccelGyro();      // calls calibrate_acc_gyro_impl()
    void calibrateMag();
    bool isConnected();
    bool isConnectedMPU9250();
    bool isConnectedAK8963();
    bool isSleeping();
    bool available();
    bool update();

    float getRoll() const;
    float getPitch() const;
    float getYaw() const;

    float getEulerX() const;
    float getEulerY() const;
    float getEulerZ() const;

    float getQuaternionX() const;
    float getQuaternionY() const;
    float getQuaternionZ() const;
    float getQuaternionW() const;

    float getAcc(const uint8_t i) const;
    float getGyro(const uint8_t i) const;
    float getMag(const uint8_t i) const;
    float getLinearAcc(const uint8_t i) const;

    float getAccX() const;
    float getAccY() const;
    float getAccZ() const;
    float getGyroX() const;
    float getGyroY() const;
    float getGyroZ() const;
    float getMagX() const;
    float getMagY() const;
    float getMagZ() const;
    float getLinearAccX() const;
    float getLinearAccY() const ;
    float getLinearAccZ() const;

    float getAccBias(const uint8_t i) const;
    float getGyroBias(const uint8_t i) const;
    float getMagBias(const uint8_t i) const;
    float getMagScale(const uint8_t i) const;

    float getAccBiasX() const;
    float getAccBiasY() const;
    float getAccBiasZ() const;
    float getGyroBiasX() const;
    float getGyroBiasY() const;
    float getGyroBiasZ() const;
    float getMagBiasX() const;
    float getMagBiasY() const;
    float getMagBiasZ() const;
    float getMagScaleX() const;
    float getMagScaleY() const;
    float getMagScaleZ() const;

    float getTemperature() const;

    void setAccBias(const float x, const float y, const float z);
    void setGyroBias(const float x, const float y, const float z);
    void setMagBias(const float x, const float y, const float z);
    void setMagScale(const float x, const float y, const float z);
    void setMagneticDeclination(const float d);

    void selectFilter(QuatFilterSel sel);
    void setFilterIterations(const size_t n);

    bool selftest();

private:
    void initMPU9250();
    void initAK8963();

public:
    void update_rpy(float qw, float qx, float qy, float qz);
    void update_accel_gyro();
private:
    void read_accel_gyro(int16_t* destination);

public:
    void update_mag();
private:
    bool read_mag(int16_t* destination);
    int16_t read_temperature_data();

    void calibrate_acc_gyro_impl();
    void set_acc_gyro_to_calibration();
    void collect_acc_gyro_data_to(float* a_bias, float* g_bias);
    void write_accel_offset();
    void write_gyro_offset();
    void calibrate_mag_impl();
    void collect_mag_data_to(float* m_bias, float* m_scale);
    bool self_test_impl();
    float get_acc_resolution(const ACCEL_FS_SEL accel_af_sel) const;
    float get_gyro_resolution(const GYRO_FS_SEL gyro_fs_sel) const;
    float get_mag_resolution(const MAG_OUTPUT_BITS mag_output_bits) const;
};