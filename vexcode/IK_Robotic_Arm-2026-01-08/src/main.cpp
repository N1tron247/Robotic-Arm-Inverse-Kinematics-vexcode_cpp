/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\4804244148                                       */
/*    Created:      Wed Jan 08 2026                                           */
/*    Description:  Inverse Kinematics Robotic Arm Code                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/
// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#define vel velocityUnits::pct


using namespace vex;

namespace arm{


  const double shoulder_height = 2.4;
  const double L1 = 3.7, L2 = 8.9;

  const double SHOULDER_OFFSET_DEG = -90.0; // true angles are yet to be determined
  const double ELBOW_OFFSET_DEG = 0; // true angles are yet to be determined

  double radToDeg(double arg){
    return (arg)*(180.0/3.141592653589793);
  }


  double rangesafe(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
  }

  void manual_home(){
    waist.resetPosition();
    shoulder.resetPosition();
    elbow.resetPosition();

    waist.stop(hold);
    shoulder.stop(hold);
    elbow.stop(hold);
  }

  void auto_home(){

  }

  void rotate(double deg, double speed){
    waist.spinFor(forward, deg, degrees, speed, vel);
    waist.stop(hold);
  }
  
  void shoulder_up(double deg, double speed){
    shoulder.spinFor(forward, deg*5, degrees, speed*5, vel);
    shoulder.stop(hold);
  }
  void shoulder_down(double deg, double speed){
    shoulder.spinFor(reverse, deg*5, degrees, speed*5, vel);
    shoulder.stop(hold);
  }

  void elbow_up(double deg, double speed){
    elbow.spinFor(forward, deg, degrees, speed, vel);
    elbow.stop(hold);
  }
  void elbow_down(double deg, double speed){
    elbow.spinFor(reverse, deg, degrees, speed, vel);
    elbow.stop(hold);
  }
  
  void claw(char cleave){

    if(cleave == 'o'){
      claws.setPosition(0, degrees);
    }
    else if(cleave == 'c'){
      claws.setPosition(45, degrees);
    }

  }

  double nearestEquivalentDeg(double currentDeg, double targetDeg) {
    while (targetDeg - currentDeg > 180.0) targetDeg -= 360.0;
    while (targetDeg - currentDeg < -180.0) targetDeg += 360.0;
    return targetDeg;
  }


  bool move_to_point(double x, double y, double z, double speed){
    double target_radius = std::hypot(x, y);
    double shoulder_z = z - shoulder_height;
    double target_vertical = shoulder_z;
    double waist_yaw = std::atan2(y, x);
    double target_shoulder_dist = std::hypot(target_radius, target_vertical);

    double max_reach = L1 + L2;
    double min_reach = std::fabs(L1 - L2);

    if(target_shoulder_dist > max_reach || target_shoulder_dist < min_reach){
      return false;
    }

    double elbow_breakover = (pow(target_shoulder_dist, 2) - pow(L1, 2) - pow(L2, 2)) / (2.0 * L1 * L2);
    elbow_breakover = rangesafe(elbow_breakover, -1.0, 1.0);

    double elbow_pitch = std::acos(elbow_breakover);

    double alphaRad = std::atan2(target_vertical, target_radius); // alpha is measured in radians as the angle between shoulder and the xy distance line to the goal

    double beta = ( pow(L1, 2) + pow(target_shoulder_dist, 2) - pow(L2, 2)) / (2.0 * L1 * target_shoulder_dist);
    beta = rangesafe(beta, -1.0, 1.0);
    double betaRad = std::acos(beta);

    double shoulder_pitch = alphaRad - betaRad;

    double waist_yaw_deg = radToDeg(waist_yaw);
    double shoulder_pitch_deg = radToDeg(shoulder_pitch);
    double elbow_pitch_deg = radToDeg(elbow_pitch);
    elbow_pitch_deg += ELBOW_OFFSET_DEG;
    shoulder_pitch_deg += SHOULDER_OFFSET_DEG;

    double shoulderMotorNow = shoulder.position(degrees);
    double shoulderTargetMotor = (shoulder_pitch_deg) * 5.0; // 5:1
    shoulderTargetMotor = nearestEquivalentDeg(shoulderMotorNow, shoulderTargetMotor);

    double waistMotorNow = waist.position(degrees);
    double waistTargetMotor = waist_yaw_deg * 5.0;
    waistTargetMotor = nearestEquivalentDeg(waistMotorNow, waistTargetMotor);
    
    waist.spinToPosition(waistTargetMotor, degrees, speed*5, vel, false);
    shoulder.spinToPosition(shoulderTargetMotor, degrees, speed*5, vel, false);
    elbow.spinToPosition(elbow_pitch_deg, degrees, speed, vel, true);
    //wait(.5, seconds);

    return true;
  }

}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  
}
