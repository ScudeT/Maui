#include <cmath>
#include <cstdint>
#include <chrono>

enum class QuatFilterSel {
    NONE,
    MADGWICK,
    MAHONY,
};

class QuaternionFilter {
    // Madgwick parameters
    float GyroMeasError = M_PI * (40.0f / 180.0f);  // gyroscope measurement error in rads/s (start at 40 deg/s)
    float GyroMeasDrift = M_PI * (0.0f / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    float beta = std::sqrt(3.0f / 4.0f) * GyroMeasError;
    float zeta = std::sqrt(3.0f / 4.0f) * GyroMeasDrift;

    // Mahony parameters
    float Kp = 30.0;
    float Ki = 0.0;

    QuatFilterSel filter_sel{QuatFilterSel::MADGWICK};
    double deltaT{0.0};
    uint64_t newTime{0}, oldTime{0};

public:
    void select_filter(QuatFilterSel sel);
    void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
    void no_filter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
    void madgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
    void mahony(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
};

