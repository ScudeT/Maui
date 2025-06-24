#ifndef MPU9250_HPP
#define MPU9250_HPP

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "Wire.hpp"
#include "MPU9250RegisterMap.h"
#include "QuaternionFilter.hpp"

// Full-scale selection enums
enum class ACCEL_FS_SEL { A2G, A4G, A8G, A16G };
enum class GYRO_FS_SEL { G250DPS, G500DPS, G1000DPS, G2000DPS };
enum class MAG_OUTPUT_BITS { M14BITS, M16BITS };
enum class FIFO_SAMPLE_RATE : uint8_t {
    SMPL_200HZ, SMPL_167HZ, SMPL_143HZ, SMPL_125HZ
};
enum class GYRO_DLPF_CFG : uint8_t {
    DLPF_250HZ, DLPF_184HZ, DLPF_92HZ, DLPF_41HZ,
    DLPF_20HZ, DLPF_10HZ, DLPF_5HZ, DLPF_3600HZ
};
enum class ACCEL_DLPF_CFG : uint8_t {
    DLPF_218HZ_0, DLPF_218HZ_1, DLPF_99HZ,
    DLPF_45HZ, DLPF_21HZ, DLPF_10HZ, DLPF_5HZ, DLPF_420HZ
};

// Default WHO_AM_I values
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
public:
    static constexpr uint8_t MPU9250_DEFAULT_ADDRESS {0x68};
    static constexpr uint8_t AK8963_ADDRESS {0x0C};
    static constexpr uint8_t AK8963_WHOAMI_DEFAULT_VALUE {0x48};
    static constexpr uint8_t MAG_MODE {0x06};
    static constexpr uint16_t CALIB_GYRO_SENSITIVITY {131};
    static constexpr uint16_t CALIB_ACCEL_SENSITIVITY {16384};

    MPU9250(rclcpp::Logger logger,
            uint8_t addr = MPU9250_DEFAULT_ADDRESS,
            const MPU9250Setting& setting = MPU9250Setting(),
            const I2CHandler& i2cBus = I2CHandler())
    : logger_(logger), mpu_i2c_addr(addr), setting(setting), i2cBus_(i2cBus) {
        if (addr < MPU9250_DEFAULT_ADDRESS || addr > MPU9250_DEFAULT_ADDRESS + 7) {
            RCLCPP_ERROR(logger_, "Invalid I2C address 0x%02x for MPU9250", addr);
            throw std::invalid_argument("Invalid MPU9250 I2C address");
        }
        if (!isConnectedMPU9250()) {
            RCLCPP_ERROR(logger_, "MPU9250 not found at 0x%02x", addr);
            throw std::runtime_error("MPU9250 core not connected");
        }
        initMPU9250();
        if (!isConnectedAK8963()) {
            RCLCPP_ERROR(logger_, "AK8963 magnetometer not found");
            throw std::runtime_error("AK8963 not connected");
        }
        initAK8963();
        has_connected = true;
    }

    ~MPU9250() = default;

    void verbose(bool b) { b_verbose = b; }
    void ahrs(bool b)    { b_ahrs = b; }
    void sleep(bool b) {
        uint8_t c = i2cBus_.read_byte(mpu_i2c_addr, PWR_MGMT_1);
        c = b ? (c | 0x40) : (c & ~0x40);
        i2cBus_.write_byte(mpu_i2c_addr, PWR_MGMT_1, c);
    }
    bool isConnected() const { return has_connected; }
    bool available() const   {
        return has_connected && (i2cBus_.read_byte(mpu_i2c_addr, INT_STATUS) & 0x01);
    }
    bool update() {
        if (!available()) return false;
        update_accel_gyro();
        update_mag();
        if (b_ahrs) update_rpy(q[0],q[1],q[2],q[3]);
        else {
            temperature_count = read_temperature_data();
            temperature = temperature_count/333.87f + 21.0f;
        }
        return true;
    }

    // getters omitted for brevity...

    void setMagBias(float x, float y, float z) {
        mag_bias[0]=x; mag_bias[1]=y; mag_bias[2]=z;
    }
    void setMagScale(float x, float y, float z) {
        mag_scale[0]=x; mag_scale[1]=y; mag_scale[2]=z;
    }

    void calibrateMag() {
        mag_bias[0]=mag_bias[1]=mag_bias[2]=0.0f;
        mag_scale[0]=mag_scale[1]=mag_scale[2]=1.0f;
        calibrate_mag_impl();
        initAK8963();
    }

private:
    void initMPU9250() {
        acc_resolution = get_acc_resolution(setting.accel_fs_sel);
        gyro_resolution = get_gyro_resolution(setting.gyro_fs_sel);
        mag_resolution = get_mag_resolution(setting.mag_output_bits);
        i2cBus_.write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x80);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        i2cBus_.write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x01);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        i2cBus_.write_byte(mpu_i2c_addr, MPU_CONFIG, static_cast<uint8_t>(setting.gyro_dlpf_cfg));
        i2cBus_.write_byte(mpu_i2c_addr, SMPLRT_DIV, static_cast<uint8_t>(setting.fifo_sample_rate));
        i2cBus_.write_byte(mpu_i2c_addr, GYRO_CONFIG,
                            setting.gyro_fchoice<<3 | static_cast<uint8_t>(setting.gyro_dlpf_cfg));
        i2cBus_.write_byte(mpu_i2c_addr, ACCEL_CONFIG,
                            setting.accel_fchoice<<3 | static_cast<uint8_t>(setting.accel_dlpf_cfg));
        i2cBus_.write_byte(mpu_i2c_addr, ACCEL_CONFIG2,
                            setting.accel_fchoice<<3 | static_cast<uint8_t>(setting.accel_dlpf_cfg));
        i2cBus_.write_byte(mpu_i2c_addr, INT_PIN_CFG, 0x22);
        i2cBus_.write_byte(mpu_i2c_addr, INT_ENABLE, 0x01);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void initAK8963() {
        uint8_t raw[3];
        i2cBus_.write_byte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        i2cBus_.write_byte(AK8963_ADDRESS, AK8963_CNTL, 0x0F);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        i2cBus_.read_bytes(AK8963_ADDRESS, AK8963_ASAX, 3, raw);
        for(int i=0;i<3;++i)
            mag_bias_factory[i] = ((raw[i]-128)/256.0f) + 1.0f;
        i2cBus_.write_byte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        i2cBus_.write_byte(AK8963_ADDRESS, AK8963_CNTL,
                            (static_cast<uint8_t>(setting.mag_output_bits)<<4)|MAG_MODE);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    void update_accel_gyro() {
        uint8_t buf[14];
        i2cBus_.read_bytes(mpu_i2c_addr, ACCEL_XOUT_H, 14, buf);
        int16_t raw[7];
        for(int i=0;i<7;++i)
            raw[i] = (static_cast<int16_t>(buf[2*i])<<8) | buf[2*i+1];
        for(int j=0;j<3;++j)
            a[j] = raw[j] * acc_resolution;
        temperature_count = raw[3];
        for(int j=0;j<3;++j)
            g[j] = raw[j+4] * gyro_resolution;
    }

    void update_mag() {
        uint8_t buf[7];
        i2cBus_.read_bytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, buf);
        if(buf[6]&0x08) {
            RCLCPP_ERROR(logger_, "Magnetometer overflow");
            return;
        }
        int16_t rx = (buf[1]<<8)|buf[0];
        int16_t ry = (buf[3]<<8)|buf[2];
        int16_t rz = (buf[5]<<8)|buf[4];
        float fx = rx * mag_resolution * mag_bias_factory[0];
        float fy = ry * mag_resolution * mag_bias_factory[1];
        float fz = rz * mag_resolution * mag_bias_factory[2];
        m[0] = (fx - mag_bias[0]) * mag_scale[0];
        m[1] = (fy - mag_bias[1]) * mag_scale[1];
        m[2] = (fz - mag_bias[2]) * mag_scale[2];
        float an = -a[0], ae = a[1], ad = a[2];
        float gn = g[0]*M_PI/180, ge = -g[1]*M_PI/180, gd = -g[2]*M_PI/180;
        float mn = m[1], me = -m[0], md = m[2];
        for(size_t i=0;i<n_filter_iter;++i)
            quat_filter.update(an,ae,ad,gn,ge,gd,mn,me,md,q);
        update_rpy(q[0],q[1],q[2],q[3]);
    }

    int16_t read_temperature_data() {
        uint8_t raw[2];
        i2cBus_.read_bytes(mpu_i2c_addr, TEMP_OUT_H, 2, raw);
        return (static_cast<int16_t>(raw[0])<<8) | raw[1];
    }

    void update_rpy(float qw,float qx,float qy,float qz) {
        float a12 = 2*(qx*qy + qw*qz);
        float a22 = qw*qw + qx*qx - qy*qy - qz*qz;
        float a31 = 2*(qw*qx + qy*qz);
        float a32 = 2*(qx*qz - qw*qy);
        float a33 = qw*qw - qx*qx - qy*qy + qz*qz;
        rpy[0] = std::atan2(a31,a33) * 180.0f/M_PI;
        rpy[1] = -std::asin(a32) * 180.0f/M_PI;
        rpy[2] = std::atan2(a12,a22) * 180.0f/M_PI + magnetic_declination;
        if(rpy[2] >= 180) rpy[2] -= 360;
        else if(rpy[2] < -180) rpy[2] += 360;
    }

    bool isConnectedMPU9250() {
        uint8_t c = i2cBus_.read_byte(mpu_i2c_addr, WHO_AM_I_MPU9250);
        if(b_verbose) RCLCPP_INFO(logger_, "MPU9250 WHO_AM_I=0x%02x", c);
        return c==MPU9250_WHOAMI_DEFAULT_VALUE || c==MPU9255_WHOAMI_DEFAULT_VALUE || c==MPU6500_WHOAMI_DEFAULT_VALUE;
    }
    bool isConnectedAK8963() {
        uint8_t c = i2cBus_.read_byte(AK8963_ADDRESS, AK8963_WHO_AM_I);
        if(b_verbose) RCLCPP_INFO(logger_, "AK8963 WHO_AM_I=0x%02x", c);
        return c==AK8963_WHOAMI_DEFAULT_VALUE;
    }

    float get_acc_resolution(ACCEL_FS_SEL fs) const {
        switch(fs) {
            case ACCEL_FS_SEL::A2G:  return 2.0f/32768;
            case ACCEL_FS_SEL::A4G:  return 4.0f/32768;
            case ACCEL_FS_SEL::A8G:  return 8.0f/32768;
            case ACCEL_FS_SEL::A16G: return 16.0f/32768;
        }
        return 0.0f;
    }
    float get_gyro_resolution(GYRO_FS_SEL fs) const {
        switch(fs) {
            case GYRO_FS_SEL::G250DPS:  return 250.0f/32768;
            case GYRO_FS_SEL::G500DPS:  return 500.0f/32768;
            case GYRO_FS_SEL::G1000DPS: return 1000.0f/32768;
            case GYRO_FS_SEL::G2000DPS: return 2000.0f/32768;
        }
        return 0.0f;
    }
    float get_mag_resolution(MAG_OUTPUT_BITS bits) const {
        switch(bits) {
            case MAG_OUTPUT_BITS::M14BITS: return 10.0f*4912/8190;
            case MAG_OUTPUT_BITS::M16BITS: return 10.0f*4912/32760;
        }
        return 0.0f;
    }

        void write_accel_offset() {
        // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
        // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
        // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
        // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
        // the accelerometer biases calculated above must be divided by 8.

        uint8_t read_data[2] = {0};
        int16_t acc_bias_reg[3] = {0, 0, 0};                      // A place to hold the factory accelerometer trim biases
        i2cBus_.read_bytes(mpu_i2c_addr, XA_OFFSET_H, 2, &read_data[0]);  // Read factory accelerometer trim values
        acc_bias_reg[0] = ((int16_t)read_data[0] << 8) | read_data[1];
        i2cBus_.read_bytes(mpu_i2c_addr, YA_OFFSET_H, 2, &read_data[0]);
        acc_bias_reg[1] = ((int16_t)read_data[0] << 8) | read_data[1];
        i2cBus_.read_bytes(mpu_i2c_addr, ZA_OFFSET_H, 2, &read_data[0]);
        acc_bias_reg[2] = ((int16_t)read_data[0] << 8) | read_data[1];

        int16_t mask_bit[3] = {1, 1, 1};  // Define array to hold mask bit for each accelerometer bias axis
        for (int i = 0; i < 3; i++) {
            if (acc_bias_reg[i] % 2) {
                mask_bit[i] = 0;
            }
            acc_bias_reg[i] -= (int16_t)acc_bias[i] >> 3;  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
            if (mask_bit[i]) {
                acc_bias_reg[i] = acc_bias_reg[i] & ~mask_bit[i];  // Preserve temperature compensation bit
            } else {
                acc_bias_reg[i] = acc_bias_reg[i] | 0x0001;  // Preserve temperature compensation bit
            }
        }

        uint8_t write_data[6] = {0};
        write_data[0] = (acc_bias_reg[0] >> 8) & 0xFF;
        write_data[1] = (acc_bias_reg[0]) & 0xFF;
        write_data[2] = (acc_bias_reg[1] >> 8) & 0xFF;
        write_data[3] = (acc_bias_reg[1]) & 0xFF;
        write_data[4] = (acc_bias_reg[2] >> 8) & 0xFF;
        write_data[5] = (acc_bias_reg[2]) & 0xFF;

        // Push accelerometer biases to hardware registers
        i2cBus_.write_byte(mpu_i2c_addr, XA_OFFSET_H, write_data[0]);
        i2cBus_.write_byte(mpu_i2c_addr, XA_OFFSET_L, write_data[1]);
        i2cBus_.write_byte(mpu_i2c_addr, YA_OFFSET_H, write_data[2]);
        i2cBus_.write_byte(mpu_i2c_addr, YA_OFFSET_L, write_data[3]);
        i2cBus_.write_byte(mpu_i2c_addr, ZA_OFFSET_H, write_data[4]);
        i2cBus_.write_byte(mpu_i2c_addr, ZA_OFFSET_L, write_data[5]);

    }
    void write_gyro_offset() {
        // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
        uint8_t gyro_offset_data[6] {0};
        gyro_offset_data[0] = (-(int16_t)gyro_bias[0] / 4 >> 8) & 0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
        gyro_offset_data[1] = (-(int16_t)gyro_bias[0] / 4) & 0xFF;       // Biases are additive, so change sign on calculated average gyro biases
        gyro_offset_data[2] = (-(int16_t)gyro_bias[1] / 4 >> 8) & 0xFF;
        gyro_offset_data[3] = (-(int16_t)gyro_bias[1] / 4) & 0xFF;
        gyro_offset_data[4] = (-(int16_t)gyro_bias[2] / 4 >> 8) & 0xFF;
        gyro_offset_data[5] = (-(int16_t)gyro_bias[2] / 4) & 0xFF;

        // Push gyro biases to hardware registers
        i2cBus_.write_byte(mpu_i2c_addr, XG_OFFSET_H, gyro_offset_data[0]);
        i2cBus_.write_byte(mpu_i2c_addr, XG_OFFSET_L, gyro_offset_data[1]);
        i2cBus_.write_byte(mpu_i2c_addr, YG_OFFSET_H, gyro_offset_data[2]);
        i2cBus_.write_byte(mpu_i2c_addr, YG_OFFSET_L, gyro_offset_data[3]);
        i2cBus_.write_byte(mpu_i2c_addr, ZG_OFFSET_H, gyro_offset_data[4]);
        i2cBus_.write_byte(mpu_i2c_addr, ZG_OFFSET_L, gyro_offset_data[5]);
    }

    void calibrate_mag_impl() {
        constexpr int samples = 1500;
        float mag_min[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
        float mag_max[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
        for(int i=0; i<samples; ++i) {
            update();
            for(int j=0; j<3; ++j) {
                mag_min[j] = std::min(mag_min[j], m[j]);
                mag_max[j] = std::max(mag_max[j], m[j]);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        for(int j=0; j<3; ++j) {
            mag_bias[j]  = (mag_max[j] + mag_min[j]) * 0.5f;
            mag_scale[j] = (mag_max[j] - mag_min[j]) * 0.5f;
        }
        float avg_scale = (mag_scale[0] + mag_scale[1] + mag_scale[2]) / 3.0f;
        for(int j=0; j<3; ++j)
            mag_scale[j] = avg_scale / mag_scale[j];
    }

    // members
    rclcpp::Logger logger_;
    uint8_t mpu_i2c_addr;
    MPU9250Setting setting;
    float acc_resolution {0}, gyro_resolution {0}, mag_resolution {0};
    float acc_bias[3] {}, gyro_bias[3] {}, mag_bias_factory[3] {}, mag_bias[3] {}, mag_scale[3] {1,1,1};
    float magnetic_declination {0.0f};
    int16_t temperature_count {};
    float temperature {};
    float a[3] {}, g[3] {}, m[3] {}, q[4] {1,0,0,0}, rpy[3] {};
    QuaternionFilter quat_filter;
    size_t n_filter_iter {1};
    bool has_connected {false};
    bool b_ahrs {true}, b_verbose {true};
    I2CHandler i2cBus_;
};

#endif // MPU9250_HPP
