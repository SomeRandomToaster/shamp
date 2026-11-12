#ifndef LEG_H
#define LEG_H

#include "hexa_hardware/biased_servo.h"

#include <tf2/LinearMath/Vector3.hpp>

#include <cmath>

class Leg {
public:
  Leg(const BiasedServo& coxa_servo, const BiasedServo& tibia_servo, const BiasedServo& femur_servo)
    : coxa_servo_(coxa_servo), tibia_servo_(tibia_servo), femur_servo_(femur_servo) {}

  // Convert joint angles to servo angles
  tf2::Vector3 JointAnglesToServoAngles(const tf2::Vector3& joint_angles) const;
  // Convert foot position in local CS to joint angles 
  static tf2::Vector3 LocalPosToJointAngles(const tf2::Vector3& pos);
private:
  BiasedServo coxa_servo_;
  BiasedServo tibia_servo_;
  BiasedServo femur_servo_;
};

#endif