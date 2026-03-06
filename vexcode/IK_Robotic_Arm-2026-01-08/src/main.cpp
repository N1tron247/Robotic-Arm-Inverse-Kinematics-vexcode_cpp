/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\4804244148                                       */
/*    Created:      Wed Jan 08 2026                                           */
/*    Description:  Inverse Kinematics Robotic Arm Code                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>
#define vel velocityUnits::pct


using namespace vex;

namespace arm{ // namespace for organization and to avoid conflicting method names


  const double shoulder_height = 2.4; // how high the center of the shoulder joint is from a given plane, normally the bottom of the arm/workspace
  const double upper_arm_length = 3.7 // the distance between the shoulder joint and the elbow joint
  const double forearm_length = 8.9; // the distance between the elbow joint and the end of the manipulator

  const double SHOULDER_OFFSET_DEG = -90.0; // given offsets depending on your arm starting position
  const double ELBOW_OFFSET_DEG = 0; // offsets are defaulted to -90.0 and 0, these assume that the arm starts with all joints aligned and pointing straight up
  // see diagram in docs folder for visual represenations

  double radToDeg(double rad_arg){ // helper function: converts radians to degrees
    return (rad_arg)*(180.0/3.141592653589793);
  }


  double rangesafe(double value, double min, double max) { // helper function: checks for out of range calculations and corrects them
    if (value < min) return min;
    if (value > max) return max;
    return value;
  }

  void manual_home(){ // function for manual homing, see docs for more detail
    waist.resetPosition();
    shoulder.resetPosition();
    elbow.resetPosition();

    waist.stop(hold);
    shoulder.stop(hold);
    elbow.stop(hold);
  }

  void auto_home(){ // function to hardstop-autohome the arm

  }

  void rotate(double deg, double speed){ // function to independently rotate the arm's waist
    waist.spinFor(forward, deg, degrees, speed, vel);
    waist.stop(hold);
  }
  
  void shoulder_up(double deg, double speed){ // function to independently move the arm's shoulder joint up
    shoulder.spinFor(forward, deg*5, degrees, speed*5, vel);
    shoulder.stop(hold);
  }
  void shoulder_down(double deg, double speed){ // function to independently move the arm's shoulder joint down
    shoulder.spinFor(reverse, deg*5, degrees, speed*5, vel);
    shoulder.stop(hold);
  }

  void elbow_up(double deg, double speed){ // function to independently move the arm's elbow up
    elbow.spinFor(forward, deg, degrees, speed, vel);
    elbow.stop(hold);
  }
  void elbow_down(double deg, double speed){ // function to independently move the arm's elbow down
    elbow.spinFor(reverse, deg, degrees, speed, vel);
    elbow.stop(hold);
  }
  
  void claw(char cleave){ // function to open and close the manipulator's claw
    if(cleave == 'o'){
      claws.setPosition(0, degrees);
    }
    else if(cleave == 'c'){
      claws.setPosition(45, degrees);
    }

  }

  double nearest_equivalent_deg(double currentDeg, double targetDeg) { // helper function: find the nearest equivalent degree since degrees have a rest at 360 deg
    while (targetDeg - currentDeg > 180.0) targetDeg -= 360.0;
    while (targetDeg - currentDeg < -180.0) targetDeg += 360.0;
    return targetDeg;
  }


  bool move_to_point(double x, double y, double z, double speed){ // function which takes the x, y, and z coordinates of the desired points
                                                            // uses the points to calculate necassary motor movements and runs those motors
    
    double target_radius = std::hypot(x, y); // horizontal distance from the base axis to the target projection in the xy plane(surface the robot is on)
    double shoulder_z = z - shoulder_height; 
    double target_vertical = shoulder_z;   // ^^^vertical displacement from the shoulder joint to the target^^^

    double waist_yaw = std::atan2(y, x); // wiast yaw angle required to rotate the arm toward the target in the xy plane
    
    double target_shoulder_dist = std::hypot(target_radius, target_vertical); // the straight distance from the shoulder joint to the target point

    double max_reach = upper_arm_length + forearm_length; 
    double min_reach = std::fabs(upper_arm_length - forearm_length);
    // ^^^maximum and minimum reachable distances for a 2 two jointed arm(shoulder and elbow)^^^

    if(target_shoulder_dist > max_reach || target_shoulder_dist < min_reach){ // identifies impossible motion
      return false;
      // if the target distance is out of reach the function is terminted and returns false
    }

    double elbow_breakover = (pow(target_shoulder_dist, 2) - pow(upper_arm_length, 2) - pow(forearm_length, 2)) / (2.0 * upper_arm_length * forearm_length);
    elbow_breakover = rangesafe(elbow_breakover, -1.0, 1.0);
    // ^^^implementation of the law of cosines to find the elbow joint angle^^^
    
    double elbow_pitch = std::acos(elbow_breakover); // elbow joint angle in radians

    double alphaRad = std::atan2(target_vertical, target_radius); // alpha is measured in radians as the angle between shoulder and the xy distance line to the goal

    double beta = ( pow(upper_arm_length, 2) + pow(target_shoulder_dist, 2) - pow(forearm_length, 2)) / (2.0 * upper_arm_length * target_shoulder_dist);
    beta = rangesafe(beta, -1.0, 1.0);
    double betaRad = std::acos(beta);
    // ^^^interior angle between the upper arm and the shoulder's target line^^^

    double shoulder_pitch = alphaRad - betaRad;
    // ^^^shoulder joint angle required to align the upper arm with the target^^^

    double waist_yaw_deg = radToDeg(waist_yaw);
    double shoulder_pitch_deg = radToDeg(shoulder_pitch);
    double elbow_pitch_deg = radToDeg(elbow_pitch);
    // ^^^convert solved joint angles from radians to degrees^^^
    
    elbow_pitch_deg += ELBOW_OFFSET_DEG;
    shoulder_pitch_deg += SHOULDER_OFFSET_DEG;
    // ^^^apply the physical offsets to the calulated values for the arm joints^^^

    double shoulderMotorNow = shoulder.position(degrees);
    double shoulderTargetMotor = (shoulder_pitch_deg) * 5.0;
    // ^^^convert shoulder joint angle to motor rotation using 5:1 gearing (can be changed if you are using a different gearing)^^^
    
    shoulderTargetMotor = nearest_equivalent_deg(shoulderMotorNow, shoulderTargetMotor);
    // find the equivalent wrap-around angle for the shoulder motor's target angle to minimize motions

    double waistMotorNow = waist.position(degrees);
    double waistTargetMotor = waist_yaw_deg * 5.0;
    // convert waist yaw angle to motor rotation using 5:1 gearing
    
    waistTargetMotor = nearest_equivalent_deg(waistMotorNow, waistTargetMotor);
    // minimize rotations at the waist joint by finding an equivalent degree
    
    waist.spinToPosition(waistTargetMotor, degrees, speed*5, vel, false); 
    shoulder.spinToPosition(shoulderTargetMotor, degrees, speed*5, vel, false);  
    elbow.spinToPosition(elbow_pitch_deg, degrees, speed, vel, true);
    // command all the motors in the joints to turn in order to reach the specified point

    return true; // returns true if the motor commands have been succesfully generated and executed
  }

}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  
}
