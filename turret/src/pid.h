#ifndef TURRET_PID_H_
#define TURRET_PID_H_

namespace turret {

struct Ks {
  float p;
  float d;
  float i;
};

template <typename T> class Pid {
public:
  Pid(Ks ks) : kP_(ks.p), kI_(ks.i), kD_(ks.d) {}

  T Compute(const T error, const uint64_t dt_micros);

  void Reset() {
    integral_ = T();
    last_error_ = T();
  }

  // Setters for tuning parameters on the fly
  void SetGains(float kP, float kI, float kD) {
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
  }

  T GetLastError() { return last_error_; }

private:
  static constexpr double kMicrosPerSecond = 1000000.0;

  float kP_;
  float kI_;
  float kD_;

  // Accumulation of the area under the error curve
  T integral_ = T();
  // For an "intantaneous" difference between the last step and this one.
  T last_error_ = T();
};

template <typename T>
T Pid<T>::Compute(const T error, const uint64_t dt_micros) {
  if (dt_micros == 0) {
    return T();
  }

  float dt = static_cast<float>(dt_micros) / kMicrosPerSecond;

  T p_term = error * kP_;

  // Integral term
  // Simple rectangular integration
  integral_ = integral_ + (error * dt);
  T i_term = integral_ * kI_;

  // Derivative term
  T derivative = (error - last_error_) / dt;
  T d_term = derivative * kD_;

  // Save state for next iteration
  last_error_ = error;

  return p_term + i_term + d_term;
}

} // namespace turret

#endif // TURRET_PID_H_
