#include "vex.h"
#include <iostream>
using namespace std;

// This constructor remains for basic PID without settling.
PID::PID(float error, float kp, float ki, float kd, float starti) :
  error(error),
  kp(kp),
  ki(ki),
  kd(kd),
  starti(starti)
{};



// Now takes optional settle_flags. It decides which mode to use.
PID::PID(float error, float kp, float ki, float kd, float starti, 
float settle_error, float settle_time, float timeout, int settle_flags) :
  error(error),
  kp(kp),
  ki(ki),
  kd(kd),
  starti(starti),
  settle_error(settle_error),
  settle_time(settle_time),
  timeout(timeout)
{
  // If settle_flags were provided (is > 0), enable flag mode.
  if (settle_flags > 0) {
    this->use_settle_flags = true;
    this->settle_flags_requirement = settle_flags;
  }
};


// Same logic for the constructor that includes a custom update period.
PID::PID(float error, float kp, float ki, float kd, float starti, 
float settle_error, float settle_time, float timeout, float update_period, int settle_flags) :
  error(error),
  kp(kp),
  ki(ki),
  kd(kd),
  starti(starti),
  settle_error(settle_error),
  settle_time(settle_time),
  timeout(timeout),
  update_period(update_period)
{
  // If settle_flags were provided (is > 0), enable flag mode.
  if (settle_flags > 0) {
    this->use_settle_flags = true;
    this->settle_flags_requirement = settle_flags;
  }
};


/**
 * Computes the output power and updates settlement counters.
 * The logic for tracking both time and consecutive cycles is always running.
 * 
 * @param error Difference in desired and current position.
 * @return Output power.
 */
float PID::compute(float error){
  if (fabs(error) < starti){
    accumulated_error+=error;
  }
  if ((error>0 && previous_error<0)||(error<0 && previous_error>0)){ 
    accumulated_error = 0; 
  }

  output = kp*error + ki*accumulated_error + kd*(error-previous_error);

  previous_error=error;

  // Update settlement counters
  if(fabs(error) < settle_error){
    time_spent_settled += update_period;
    consecutive_settled_count++;
  } else {
    time_spent_settled = 0;
    consecutive_settled_count = 0;
  }

  time_spent_running += update_period;

  return output;
}

/**
 * Checks if the movement is settled based on the selected mode (flags or time).
 * Timeout is the ultimate failsafe for both modes.
 * 
 * @return Whether the movement is settled.
 */
bool PID::is_settled(){
  // 1. Check for timeout first, as it overrides everything.
  if (time_spent_running > timeout && timeout != 0){
    cout<<"timeout reached"<<endl;
    return true;
  }

  // 2. Check settlement based on the mode determined by the constructor.
  if (use_settle_flags) {
    // Use flag-based settlement
    if (consecutive_settled_count >= settle_flags_requirement) {
      cout<<"settle_flags reached"<<endl;
      return true;
    }
  } else {
    // Use time-based settlement
    if (time_spent_settled > settle_time) {
      cout<<"settle_time reached"<<endl;
      return true;
    }
  }

  // If neither timeout nor settlement condition is met, return false.
  return false;
}