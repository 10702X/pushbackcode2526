#include "vex.h"
#include <iostream>
using namespace std;
/**
 * PID constructor with P, I, D, and starti.
 * Starti keeps the I term at 0 until error is less than starti.
 * 
 * @param error Difference in desired and current position.
 * @param kp Proportional constant.
 * @param ki Integral constant.
 * @param kd Derivative constant.
 * @param starti Maximum error to start integrating.
 */

PID::PID(float error, float kp, float ki, float kd, float starti) :
  error(error),
  kp(kp),
  ki(ki),
  kd(kd),
  starti(starti)
{};

/**
 * PID constructor with settling inputs.
 * The settling system works like this: The robot is settled
 * when error is less than settle_error for a duration of settle_time,
 * or if the function has gone on for longer than timeout. Otherwise
 * it is not settled. Starti keeps the I term at 0 until error is less 
 * than starti.
 * 
 * @param error Difference in desired and current position.
 * @param kp Proportional constant.
 * @param ki Integral constant.
 * @param kd Derivative constant.
 * @param starti Maximum error to start integrating.
 * @param settle_error Maximum error to be considered settled.
 * @param settle_time Minimum time to be considered settled.
 * @param timeout Time after which to give up and move on.
 */

PID::PID(float error, float kp, float ki, float kd, float starti, 
float settle_error, float settle_time, float timeout) :
  error(error),
  kp(kp),
  ki(ki),
  kd(kd),
  starti(starti),
  settle_error(settle_error),
  settle_time(settle_time),
  timeout(timeout)
{};

/**
 * PID constructor with custom update period.
 * The default update period is 10ms, but if you want to run
 * a faster or slower loop, you need to let the settler know.
 * 
 * @param error Difference in desired and current position.
 * @param kp Proportional constant.
 * @param ki Integral constant.
 * @param kd Derivative constant.
 * @param starti Maximum error to start integrating.
 * @param settle_error Maximum error to be considered settled.
 * @param settle_time Minimum time to be considered settled.
 * @param timeout Time after which to give up and move on.
 * @param update_period Loop delay time in ms.
 */

PID::PID(float error, float kp, float ki, float kd, float starti, 
float settle_error, float settle_time, float timeout, float update_period) :
  error(error),
  kp(kp),
  ki(ki),
  kd(kd),
  starti(starti),
  settle_error(settle_error),
  settle_time(settle_time),
  timeout(timeout),
  update_period(update_period)
{};

/**
 * PID constructor for flag-based settling.
 * This constructor initializes the PID for the new settling method, which
 * considers the movement settled after the error is within settle_error for
 * a specific number of consecutive checks (settle_flags).
 * 
 * @param error Difference in desired and current position.
 * @param kp Proportional constant.
 * @param ki Integral constant.
 * @param kd Derivative constant.
 * @param starti Maximum error to start integrating.
 * @param settle_error Maximum error to be considered settled.
 * @param settle_flags Number of consecutive settled cycles required.
 * @param timeout Time after which to give up and move on.
 */
PID::PID(float error, float kp, float ki, float kd, float starti, 
float settle_error, int settle_flags, float timeout) :
  error(error),
  kp(kp),
  ki(ki),
  kd(kd),
  starti(starti),
  settle_error(settle_error),
  timeout(timeout),
  settle_flags_requirement(settle_flags)
{};

/**
 * PID constructor for flag-based settling with custom update period.
 *
 * @param error Difference in desired and current position.
 * @param kp Proportional constant.
 * @param ki Integral constant.
 * @param kd Derivative constant.
 * @param starti Maximum error to start integrating.
 * @param settle_error Maximum error to be considered settled.
 * @param settle_flags Number of consecutive settled cycles required.
 * @param timeout Time after which to give up and move on.
 * @param update_period Loop delay time in ms.
 */
PID::PID(float error, float kp, float ki, float kd, float starti, 
float settle_error, int settle_flags, float timeout, float update_period) :
  error(error),
  kp(kp),
  ki(ki),
  kd(kd),
  starti(starti),
  settle_error(settle_error),
  timeout(timeout),
  update_period(update_period),
  settle_flags_requirement(settle_flags)
{};

/**
 * Computes the output power based on the error.
 * This is also updated to handle the logic for the new flag-based settling method
 * by counting consecutive cycles where the error is within the settlement range.
 * 
 * @param error Difference in desired and current position.
 * @return Output power.
 */

float PID::compute(float error){
  if (fabs(error) < starti){
    accumulated_error+=error;
  }
  // Checks if the error has crossed 0, and if it has, it eliminates the integral term.
  if ((error>0 && previous_error<0)||(error<0 && previous_error>0)){ 
    accumulated_error = 0; 
  }

  output = kp*error + ki*accumulated_error + kd*(error-previous_error);

  previous_error=error;

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
 * Computes whether the movement has settled using either time-based or flag-based logic.
 * If settle_flags_requirement is set, it will check for consecutive settled cycles.
 * Otherwise, it defaults to the original time-based method.
 * The timeout remains a failsafe for both methods.
 * 
 * @return Whether the movement is settled.
 */

bool PID::is_settled(){
  if (time_spent_running > timeout && timeout != 0){cout<<"time out reached, settle time ="<<time_spent_settled<<endl;
    return(true);
  }
  
  if (settle_flags_requirement > 0) {
    if (consecutive_settled_count >= settle_flags_requirement) {
      cout << "settle_flags reached\n";
      return true;
    }
  } else {
    if (time_spent_settled > settle_time){ cout<<"settle_time reached\n";
      return(true);
    }
  }

  return(false);
}