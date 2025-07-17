#pragma once
#include "vex.h"

/**
 * General-use PID class for drivetrains. It includes both
 * control calculation and settling calculation. The default
 * update period is 10ms or 100Hz.
 */

class PID
{
public:
  float error = 0;
  float kp = 0;
  float ki = 0;
  float kd = 0;
  float starti = 0;
  float settle_error = 0;
  float settle_time = 0;
  float timeout = 0;
  float accumulated_error = 0;
  float previous_error = 0;
  float output = 0;
  float time_spent_settled = 0;
  float time_spent_running = 0;
  float update_period = 10;


  // This will be true if flags are used, false if time is used.
  bool use_settle_flags = false; 
  // The required number of consecutive settled cycles.
  int settle_flags_requirement = 0; 
  // The current count of consecutive settled cycles.
  int consecutive_settled_count = 0;

  PID(float error, float kp, float ki, float kd, float starti);


  // Added optional settle_flags parameter to the end.
  PID(float error, float kp, float ki, float kd, float starti, float settle_error, float settle_time, float timeout, int settle_flags = 0);

  PID(float error, float kp, float ki, float kd, float starti, float settle_error, float settle_time, float timeout, float update_period, int settle_flags = 0);

  float compute(float error);

  bool is_settled();
};