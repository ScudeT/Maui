#ifndef QUATERNION_FILTER_HPP
#define QUATERNION_FILTER_HPP

#include <cmath>
#include <cstdint>
#include <chrono>

/**
 * @brief Selection enum for filter type
 */
enum class QuatFilterSel {
    NONE,
    MADGWICK,
    MAHONY
};

/**
 * @brief Header‐only quaternion filter implementing Madgwick and Mahony AHRS
 */
class QuaternionFilter {
private:
    // Madgwick parameters
    float GyroMeasError = M_PI * (40.0f / 180.0f);
    float GyroMeasDrift = M_PI * (0.0f / 180.0f);
    float beta         = std::sqrt(3.0f / 4.0f) * GyroMeasError;
    float zeta         = std::sqrt(3.0f / 4.0f) * GyroMeasDrift;

    // Mahony parameters
    float Kp = 30.0f;
    float Ki = 0.0f;

    QuatFilterSel filter_sel{QuatFilterSel::MADGWICK};
    double       deltaT{0.0};
    uint64_t     newTime{0}, oldTime{0};

public:
    /**
     * @brief Select the filter algorithm
     */
    inline void select_filter(QuatFilterSel sel) {
        filter_sel = sel;
    }

    /**
     * @brief Update quaternion given sensor data
     */
    inline void update(float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float mx, float my, float mz,
                       float* q) {
        // compute deltaT
        newTime = std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count();
        deltaT = (newTime - oldTime) * 1e-6;
        oldTime = newTime;

        switch (filter_sel) {
            case QuatFilterSel::MADGWICK:
                madgwick(ax,ay,az,gx,gy,gz,mx,my,mz,q);
                break;
            case QuatFilterSel::MAHONY:
                mahony(ax,ay,az,gx,gy,gz,mx,my,mz,q);
                break;
            default:
                no_filter(ax,ay,az,gx,gy,gz,mx,my,mz,q);
                break;
        }
    }

    /**
     * @brief Integrate gyro only (no fusion)
     */
    inline void no_filter(float ax, float ay, float az,
                          float gx, float gy, float gz,
                          float mx, float my, float mz,
                          float* q) {
        float q0=q[0], q1=q[1], q2=q[2], q3=q[3];
        q[0] += 0.5f * (-q1*gx - q2*gy - q3*gz) * deltaT;
        q[1] += 0.5f * ( q0*gx + q2*gz - q3*gy) * deltaT;
        q[2] += 0.5f * ( q0*gy - q1*gz + q3*gx) * deltaT;
        q[3] += 0.5f * ( q0*gz + q1*gy - q2*gx) * deltaT;
        float recipNorm = 1.0f / std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        q[0]*=recipNorm; q[1]*=recipNorm; q[2]*=recipNorm; q[3]*=recipNorm;
    }

    /**
     * @brief Madgwick filter implementation
     */
    inline void madgwick(float ax, float ay, float az,
                         float gx, float gy, float gz,
                         float mx, float my, float mz,
                         float* q) {
        double q0=q[0], q1=q[1], q2=q[2], q3=q[3];
        double recipNorm;
        double s0,s1,s2,s3;
        double qDot1,qDot2,qDot3,qDot4;
        double hx,hy;
        double _2q0mx,_2q0my,_2q0mz,_2q1mx;
        double _2q0,_2q1,_2q2,_2q3;
        double _2q0q2,_2q2q3;
        double _2bx,_2bz,_4bx,_4bz;
        // rate of change from gyroscope
        qDot1 = 0.5*( -q1*gx - q2*gy - q3*gz);
        qDot2 = 0.5*(  q0*gx + q2*gz - q3*gy);
        qDot3 = 0.5*(  q0*gy - q1*gz + q3*gx);
        qDot4 = 0.5*(  q0*gz + q1*gy - q2*gx);
        // normalize accelerometer
        double a_norm = ax*ax + ay*ay + az*az;
        if(a_norm==0) return;
        recipNorm=1.0/std::sqrt(a_norm); ax*=recipNorm; ay*=recipNorm; az*=recipNorm;
        // normalize magnetometer
        double m_norm = mx*mx + my*my + mz*mz;
        if(m_norm==0) return;
        recipNorm=1.0/std::sqrt(m_norm); mx*=recipNorm; my*=recipNorm; mz*=recipNorm;
        // auxiliary vars
        _2q0mx=2*q0*mx; _2q0my=2*q0*my; _2q0mz=2*q0*mz;
        _2q1mx=2*q1*mx;
        _2q0=2*q0; _2q1=2*q1; _2q2=2*q2; _2q3=2*q3;
        _2q0q2=2*q0*q2; _2q2q3=2*q2*q3;
        double q0q0=q0*q0, q1q1=q1*q1, q2q2=q2*q2, q3q3=q3*q3;
        double q1q3=q1*q3, q0q2=q0*q2, q0q1=q0*q1, q2q3=q2*q3;
        hx = mx*q0q0 - _2q0my*q3 + _2q0mz*q2 + mx*q1q1 + _2q1*my*q2 + _2q1*mz*q3 - mx*q2q2 - mx*q3q3;
        hy = _2q0mx*q3 + my*q0q0 - _2q0mz*q1 + _2q1mx*q2 - my*q1q1 + my*q2q2 + _2q2*mz*q3 - my*q3q3;
        _2bx = std::sqrt(hx*hx + hy*hy);
        _2bz = -_2q0mx*q2 + _2q0my*q1 + mz*q0q0 + _2q1mx*q3 - mz*q1q1 + _2q2*my*q3 - mz*q2q2 + mz*q3q3;
        _4bx = 2*_2bx; _4bz = 2*_2bz;
        // gradient descent algorithm corrective step
        s0 = -_2q2*(2*q1q3 - _2q0q2 - ax) + _2q1*(2*q0q1 + _2q2q3 - ay)
             - _2bz*q2*(_2bx*(0.5- q2q2- q3q3) + _2bz*(q1*q3- q0*q2) - mx)
             + (-_2bx*q3+ _2bz*q1)*( _2bx*(q1*q2- q0*q3)+ _2bz*(q0*q1+ q2*q3)- my)
             + _2bx*q2*( _2bx*(q0*q2+ q1*q3)+ _2bz*(0.5- q1q1- q2q2)- mz);
        s1 =  _2q3*(2*q1q3- _2q0q2- ax) + _2q0*(2*q0q1+ _2q2q3- ay)
             - 4*q1*(1-2*q1q1-2*q2q2- az)
             + _2bz*q3*( _2bx*(0.5- q2q2- q3q3)+ _2bz*(q1*q3- q0*q2)- mx)
             + (_2bx*q2+ _2bz*q0)*( _2bx*(q1*q2- q0*q3)+ _2bz*(q0*q1+ q2*q3)- my)
             + (_2bx*q3- _4bz*q1)*( _2bx*(q0*q2+ q1*q3)+ _2bz*(0.5- q1q1- q2q2)- mz);
        s2 = -_2q0*(2*q1q3- _2q0q2- ax) + _2q3*(2*q0q1+ _2q2q3- ay)
             - 4*q2*(1-2*q1q1-2*q2q2- az)
             + (-_4bx*q2- _2bz*q0)*( _2bx*(0.5- q2q2- q3q3)+ _2bz*(q1*q3- q0*q2)- mx)
             + (_2bx*q1+ _2bz*q3)*( _2bx*(q1*q2- q0*q3)+ _2bz*(q0*q1+ q2*q3)- my)
             + (_2bx*q0- _4bz*q2)*( _2bx*(q0*q2+ q1*q3)+ _2bz*(0.5- q1q1- q2q2)- mz);
        s3 =  _2q1*(2*q1q3- _2q0q2- ax) + _2q2*(2*q0q1+ _2q2q3- ay)
             + (-_4bx*q3+ _2bz*q1)*( _2bx*(0.5- q2q2- q3q3)+ _2bz*(q1*q3- q0*q2)- mx)
             + (-_2bx*q0+ _2bz*q2)*( _2bx*(q1*q2- q0*q3)+ _2bz*(q0*q1+ q2*q3)- my)
             + _2bx*q1*( _2bx*(q0*q2+ q1*q3)+ _2bz*(0.5- q1q1- q2q2)- mz);
        recipNorm = 1.0f/std::sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0*=recipNorm; s1*=recipNorm; s2*=recipNorm; s3*=recipNorm;
        // apply feedback step
        qDot1 -= beta * s0; qDot2 -= beta * s1; qDot3 -= beta * s2; qDot4 -= beta * s3;
        // integrate rate of change
        q0 += qDot1 * deltaT; q1 += qDot2 * deltaT; q2 += qDot3 * deltaT; q3 += qDot4 * deltaT;
        // normalize quaternion
        recipNorm = 1.0/std::sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
        q0*=recipNorm; q1*=recipNorm; q2*=recipNorm; q3*=recipNorm;
        q[0]=(float)q0; q[1]=(float)q1; q[2]=(float)q2; q[3]=(float)q3;
    }

    /**
     * @brief Mahony filter implementation
     */
    inline void mahony(float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float mx, float my, float mz,
                       float* q) {
        float recipNorm, vx, vy, vz, ex, ey, ez;
        float qa=q[0], qb=q[1], qc=q[2], qd=q[3];
        static float ix=0, iy=0, iz=0;
        float tmp;
        // normalize accel
        tmp=ax*ax+ay*ay+az*az;
        if(tmp>0) { recipNorm=1/std::sqrt(tmp); ax*=recipNorm; ay*=recipNorm; az*=recipNorm; }
        // estimated gravity direction
        vx=qb*qd - qa*qc;
        vy=qa*qb + qc*qd;
        vz=qa*qa - 0.5f + qd*qd;
        // error
        ex=(ay*vz - az*vy);
        ey=(az*vx - ax*vz);
        ez=(ax*vy - ay*vx);
        // integral feedback
        if(Ki>0) { ix+=Ki*ex*deltaT; iy+=Ki*ey*deltaT; iz+=Ki*ez*deltaT; gx+=ix; gy+=iy; gz+=iz; }
        // proportional feedback
        gx+=Kp*ex; gy+=Kp*ey; gz+=Kp*ez;
        // integrate rate of change
        tmp=0.5f*deltaT;
        gx*=tmp; gy*=tmp; gz*=tmp;
        qa+=(-qb*gx - qc*gy - qd*gz);
        qb+=( qa*gx + qc*gz - qd*gy);
        qc+=( qa*gy - qb*gz + qd*gx);
        qd+=( qa*gz + qb*gy - qc*gx);
        // normalize quaternion
        recipNorm=1/std::sqrt(qa*qa+qb*qb+qc*qc+qd*qd);
        q[0]=qa*recipNorm; q[1]=qb*recipNorm; q[2]=qc*recipNorm; q[3]=qd*recipNorm;
    }
};

#endif // QUATERNION_FILTER_HPP