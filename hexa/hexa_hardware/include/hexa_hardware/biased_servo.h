#ifndef BIASED_SERVO_H
#define BIASED_SERVO_H

class BiasedServo
{
private:
public:
  BiasedServo(float bias, bool flip_dir, float min_deg = 0, float max_deg = 180)
    : bias_(bias), min_deg_(min_deg), max_deg_(max_deg), flip_dir_(flip_dir){}

  float GetRawAngle(float pos) const {
    float raw_angle = flip_dir_ ? bias_ - pos : bias_ + pos;
    if (raw_angle < min_deg_) {
        raw_angle = min_deg_;
    } else if (raw_angle > max_deg_) {
        raw_angle = max_deg_;
    }
    return raw_angle;
  }
private:
  float bias_, min_deg_, max_deg_;
  bool flip_dir_;
};

#endif