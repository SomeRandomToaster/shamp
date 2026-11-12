#include "hexa_hardware/leg.h"

tf2::Vector3 Leg::JointAnglesToServoAngles(const tf2::Vector3& joint_angles) const {
  float phi = joint_angles.x();
  float theta = joint_angles.y();
  float psi = joint_angles.z();

  tf2::Vector3 raw_angles;
  raw_angles.setX(coxa_servo_.GetRawAngle(phi));
  raw_angles.setY(tibia_servo_.GetRawAngle(theta));
  raw_angles.setZ(femur_servo_.GetRawAngle(psi));

  return raw_angles;
}

tf2::Vector3 Leg::LocalPosToJointAngles(const tf2::Vector3& pos) {
  /* Coxa, tibia and femur axes are the symmetry axes of the respective servo motor shafts.
   * Coxa plane is a horizontal plane through the tibia axis.
   * Leg plane is a plane through the coxa axis perpendicular to the tibia axis
   * 
   * CS origin (O) is the intersection point of coxa plane and coxa axis 
   * Point C is the intersection point of the tibia axis and leg plane
   * Point T the intersection point of the femur axis and leg plane
   * Point F is the center of the foot ball
   * Point F' is the lowest point of the foot ball
   * Point D is an orthogonal projection of C on the horizontal plane through F
   * 
   * Input coordinates are the F' coordinates.
   */

  constexpr float kFootBallRadius = 15;
  constexpr float kCoxaLen = 57.25;
  constexpr float kTibiaLen = 110;
  constexpr float kFemurLen = 197.5;

  tf2::Vector3 F_prime = pos;

  tf2::Vector3 F = F_prime + tf2::Vector3(0, 0, kFootBallRadius);
  
  float phi = (180 / M_PI) * atan2(F.x(), F.y());

  tf2::Vector3 C(kCoxaLen * sin(phi), kCoxaLen * cos(phi), 0);
  tf2::Vector3 D(kCoxaLen * sin(phi), kCoxaLen * cos(phi), F.z());
  tf2::Vector3 CD = D - C;
  tf2::Vector3 CF = F - C;
  float CD_len = CD.length();
  float CF_len = CF.length();

  float alpha = (180 / M_PI) * acos((kTibiaLen * kTibiaLen + kFemurLen * kFemurLen - CF_len * CF_len) / (2 * kTibiaLen * kFemurLen));
  float psi = 90 - alpha;

  float beta = (180 / M_PI) * acos((kTibiaLen * kTibiaLen + CF_len * CF_len - kFemurLen * kFemurLen) / (2 * CF_len * kTibiaLen));
  float gamma = (180 / M_PI) * acos(CD.dot(CF) / (CD_len * CF_len));
  float theta = 180 - beta - gamma;

  return {phi, theta, psi};
}