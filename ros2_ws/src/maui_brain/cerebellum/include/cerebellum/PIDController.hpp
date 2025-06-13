#ifndef PID_CONTROLLER
#define PID_CONTROLLER

#include <algorithm>  
#include <cstdint>
#include <rclcpp/rclcpp.hpp>

class PIDController {
public:
  /**
   * @brief Construct a new PID Controller object.
   * 
   * @param kp Proportional gain.
   * @param ki Integral gain.
   * @param kd Derivative gain.
   * @param dt Time step between updates.
   * @param output_min Minimum output limit.
   * @param output_max Maximum output limit.
   * @param deriv_filter_coef Derivative filter coefficient (0 = full filtering, 1 = no filtering).
   */
  PIDController(double kp, double ki, double kd, double dt,
                double output_min, double output_max, double cutoff_freq)
    : kp_(kp), ki_(ki), kd_(kd), dt_(dt),
      output_min_(output_min), output_max_(output_max),
      prev_error_(0.0), integral_(0.0), prev_derivative_(0.0)
  {
    kf_ = - (M_PI * cutoff_freq * dt_ - 1.0)/ (M_PI * cutoff_freq * dt_ +1.0);
    kr_ = + (M_PI * cutoff_freq * dt_)/ (M_PI * cutoff_freq * dt_ + 1.0);
  }

  void setVerbose(bool verbose) { verbose_ = verbose; }

  /**
   * @brief Compute the PID output given a setpoint and current measurement.
   * 
   * @param setpoint The desired value.
   * @param measurement The current value.
   * @return double The computed control output.
   */
  double compute(double setpoint, double measurement) {
    // Calculate error
    double error = setpoint - measurement;
    
    // Proportional term
    double P = kp_ * error;

    // Integral term with anti-windup (by clamping)
    integral_ += error * dt_;
    double I = ki_ * integral_;
    double max_I = output_max_ - P;
    double min_I = output_min_ - P;
    I = std::clamp(I, min_I, max_I);
    if (ki_ != 0.0) {
      integral_ = I / ki_;
    }


    double derivative = filter_error(error)/ dt_;
    double D = kd_ * derivative;

    // Combine terms and clamp final output
    double output = P + I + D;
    output = std::clamp(output, output_min_, output_max_);

    // Update internal state
    prev_error_ = error;
    prev_derivative_ = derivative;

    if (verbose_) {
        RCLCPP_INFO(rclcpp::get_logger("PIDController"),
            "set: %.3f, mes: %.3f, error: %.3f, output: %.3f, P: %.3f, I: %.3f, D: %.3f",
            setpoint, measurement, error, output, P, I, D);
    }

    return output;
  }

  double compute(double error) {
    
    // Proportional term
    double P = kp_ * error;

    // Integral term with anti-windup (by clamping)
    integral_ += error * dt_;
    double I = ki_ * integral_;
    double max_I = output_max_ - P;
    double min_I = output_min_ - P;
    I = std::clamp(I, min_I, max_I);
    if (ki_ != 0.0) {
      integral_ = I / ki_;
    }


    double derivative = filter_error(error)/ dt_;
    double D = kd_ * derivative;

    // Combine terms and clamp final output
    double output = P + I + D;
    output = std::clamp(output, output_min_, output_max_);

    // Update internal state
    prev_error_ = error;
    prev_derivative_ = derivative;

    if (verbose_) {
        RCLCPP_INFO(rclcpp::get_logger("PIDController"),
            "set: %.3f, mes: %.3f, error: %.3f, output: %.3f, P: %.3f, I: %.3f, D: %.3f",
            setpoint, measurement, error, output, P, I, D);
    }

    return output;
  }

  double filter_error(const double& raw_err)
  {

    // Se prima chiamata: inizializza
    if (!prev_err_set_) {
      prev_err_filtered_err_ = raw_err;
      prev_raw_err_ = raw_err;
      prev_err_set_ = true;
      return raw_err;
    }

    double err_filtered;
    err_filtered = kf_ * prev_err_filtered_err_ + kr_ * (raw_err + prev_raw_err_);

    prev_err_filtered_err_ = err_filtered;
    prev_raw_err_ = raw_err;

    return err_filtered;
  }

  /**
   * @brief Update the PID coefficients and parameters at runtime.
   * 
   * @param kp New proportional gain.
   * @param ki New integral gain.
   * @param kd New derivative gain.
   * @param dt New time step between updates.
   * @param output_min New minimum output limit.
   * @param output_max New maximum output limit.
   * @param deriv_filter_coef New derivative filter coefficient.
   */
  void updateCoefficients(double kp, double ki, double kd,
                          double output_min, double output_max, double cutoff_freq) {
    kp_ = double(kp);
    ki_ = double(ki);
    kd_ = double(kd);
    output_min_ = double(output_min);
    output_max_ = double(output_max);
    kf_ = - (M_PI * cutoff_freq * dt_ - 1.0)/ (M_PI * cutoff_freq * dt_ +1.0);
    kr_ = + (M_PI * cutoff_freq * dt_)/ (M_PI * cutoff_freq * dt_ + 1.0);                       
  }

  void updateCoefficients(double kp, double ki, double kd) {
    kp_ = double(kp);
    ki_ = double(ki);
    kd_ = double(kd);
  } 

  void reset(){
    prev_error_ = 0.0;
    integral_ = 0.0;
    prev_derivative_ = 0.0;
  }

private:
  // PID gains
  double kp_;
  double ki_;
  double kd_;

  // Sample time
  double dt_;

  // Output limits
  double output_min_;
  double output_max_;

  // Derivative low-pass filter coefficient
  double kf_;
  double kr_;
  double prev_err_filtered_err_;
  double prev_raw_err_;
  bool prev_err_set_ = false;
  

  // Internal state variables
  double prev_error_;
  double integral_;
  double prev_derivative_;

  bool verbose_ = false;
};

#endif // PID_CONTROLLER_HPP
