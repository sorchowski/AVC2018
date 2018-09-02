// Example program
#include <iostream>
#include <string>
#include <cmath>

#define MAX_VELOCITY 2.0

#define ESC_VELOCITY_MAX_REVERSE 1000
#define ESC_VELOCITY_ZERO 1500
#define ESC_VELOCITY_MAX_FORWARD 2000
#define ESC_VELOCITY_MOVE_ADJ 100

#define SERVO_MAX_LEFT 1000
#define SERVO_CENTER 1500
#define SERVO_MAX_RIGHT 2000

#define STEERING_MAX_THETA_THEORETICAL 1.57079632679
#define STEERING_MAX_THETA 1.25663706144


int mapVelocityToServo(float input_velocity) {
  // add max_velocity to input
  // upper velocity range = max_velocity*2
  float upper_velocity_range = MAX_VELOCITY*2.0;

  // temp should be between 0.0 and 2*MAX_VELOCITY
  float temp_velocity_value = input_velocity+MAX_VELOCITY; // We always assume max forward = abs(max reverse)

  float target_range = ESC_VELOCITY_MAX_FORWARD-ESC_VELOCITY_MAX_REVERSE;

  // now map temp velocity to target range
  float temp = (temp_velocity_value/upper_velocity_range)*target_range;

  // adjust the range back to our expected esc values
  int final_esc_velocity = temp+ESC_VELOCITY_MAX_REVERSE;

  return final_esc_velocity;
}

int newMapMethod(float velocity) {
  // map 0 to 2.5m/s to 1600 to 2000 for + velocities, (i.e. 0.0-2.5 -> 100 to 500)
  // map 0 to -2.5m/s to 1000 to 1400 for - velocities
  int escValue = ESC_VELOCITY_ZERO;
  float absoluteVelocity = std::abs(velocity);
  float adjustedEscVelocity = (((absoluteVelocity/2.5)*400.0)+100.0);
  if (velocity > 0.0) {
    escValue = ESC_VELOCITY_ZERO+(int)adjustedEscVelocity;
  }
  else if (velocity < 0.0) {
    escValue = ESC_VELOCITY_ZERO-(int)adjustedEscVelocity;   
  }
  return escValue;
}

int main()
{
    float velocity = 0.45;
    int escVelocity = mapVelocityToServo(velocity);
    int newEscVelocity = newMapMethod(velocity);
    
    std::cout << "ESC velocity: " << escVelocity << std::endl;
    std::cout << "New ESC velocity: " << newEscVelocity  << std::endl;

    std::cout << "New ESC velocity, 2.5: " << newMapMethod(2.5) << std::endl;
    std::cout << "New ESC velocity, 1.5: " << newMapMethod(1.5) << std::endl;
    std::cout << "New ESC velocity, 0.0: " << newMapMethod(0.0) << std::endl;
    std::cout << "New ESC velocity, 0.1: " << newMapMethod(0.1) << std::endl;
    std::cout << "New ESC velocity, -0.1: " << newMapMethod(-0.1) << std::endl;
    std::cout << "New ESC velocity, -1.5: " << newMapMethod(-1.5) << std::endl;
    std::cout << "New ESC velocity, -2.5: " << newMapMethod(-2.5) << std::endl;
}