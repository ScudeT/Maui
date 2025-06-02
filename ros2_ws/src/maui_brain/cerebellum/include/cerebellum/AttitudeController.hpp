#pragma once

#include "PIDController.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <array>

class AttitudeController {
public:
    // Constructor: receive 3 PIDController instances (by value)
    AttitudeController( tf2::Vector3 K_qerr,
                        const PIDController& pid_x,
                        const PIDController& pid_y,
                        const PIDController& pid_z)
        : K_qerr_(K_qerr), pid_x_(pid_x), pid_y_(pid_y), pid_z_(pid_z) {}

    // Compute control input u = [ux, uy, uz]
    // Inputs: 
    //    q_set: tf2::Quaternion (desired orientation)
    //    q_mes: tf2::Quaternion (measured orientation)
    //    w_mes: tf2::Vector3 (measured angular velocity)
    // Output: std::array<float, 3> (ux, uy, uz)
    tf2::Vector3 get_u( const tf2::Quaternion& q_set,
                        const tf2::Quaternion& q_mes,
                        const tf2::Vector3& w_mes)
    {
        // Compute quaternion error
        tf2::Quaternion q_err = q_mes.inverse() * q_set;

        // Get shortest rotation
        if (q_err.getW() < 0) {
            q_err = tf2::Quaternion(-q_err.getX(), -q_err.getY(), -q_err.getZ(), -q_err.getW());
        }

        // Compute angular velocity setpoint from quaternion error
        tf2::Vector3 w_sp = 2.0 * K_qerr_ * q_err.getAxis() * q_err.getAngle();

        // Get control inputs for the 3 axes
        tf2::Vector3 u;
        u.setX(pid_x_.compute(w_sp.x(), w_mes.x()));
        u.setY(pid_y_.compute(w_sp.y(), w_mes.y()));
        u.setZ(pid_z_.compute(w_sp.z(), w_mes.z()));
        return u;
    }

private:
    tf2::Vector3 K_qerr_;
    PIDController pid_x_, pid_y_, pid_z_;
};

