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
// On the timeouts, we could redesign it to be a softer cap.
// Instead of a hard-cut designated time to complete a task, we'd instead just define
// The maximum of time we'd allow it to go on.
// This way, timeouts could be configured with much more breathing room while retaining accuracy.
// Currently, timeouts define the absolute amount of time a task will go on for, so if
// a task is set to 1000ms timeout, it will absolutely finish and move on in 1000ms.
// Instead, we propose a timeout that only limits the task if it has not been completed within
// X amount of time. Setting a task to 1000ms timeout will only finish in 1000ms if the task is 
// not detected as completed. However, if it successfully completes before the 1000ms timeout, 
// We can be assured that the task is at its fastest possible completion time.
// For quality of life features (for debugging and programming), we can allow for this "completion time"
// to be tracked, making sure that the developer knows, on average, how much time a task will take.
// This feature is useful in the event that the developer is short on time and needs to know an accurate
// representation of how long each task takes/the total amount of time (on average). 
// This feature is slightly more complex so we will leave it for later.
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
 * Computes the output power based on the error.
 * Typical PID calculation with some optimizations: When the robot crosses
 * error=0, the i-term gets reset to 0. And, of course, the robot only
 * accumulates i-term when error is less than starti. Read about these at
 * https://georgegillard.com/resources/documents.
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

  if(fabs(error)<settle_error){
    time_spent_settled+=10; //cout<<"error="<<error<<" settle_error="<<settle_error<<" time_spent_settled="<<time_spent_settled<<endl;
    // We could add something where it has a window of checking how many times in a row fabs(error) has been < settle_error.
    // This could be achieved through a queue?
    // Have queue be a set capacity of 5, and upon initializing the queue every value is "False"
    // We update values in the queue to True or False on each PID::compute on the condition of "is fabs(error)<settle_error", so it keeps up with the existing logic
    // Each iteration, check the validity of this queue, for example:
    /*
    #include <string>
    
    struct Flags {
        bool flag1;
        bool flag2;
        bool flag3;
        // Add more boolean members as needed
    };

bool allFlagsTrue(const Flags& flags) {
    return flags.flag1 && flags.flag2 && flags.flag3;
}

int main() {
    Flags myFlags = {true, true, true};

    if (allFlagsTrue(myFlags)) {
        std::cout << "All flags are true" << std::endl;
    } else {
        std::cout << "Not all flags are true" << std::endl;
    }

    myFlags.flag2 = false;
     if (allFlagsTrue(myFlags)) {
        std::cout << "All flags are true" << std::endl;
    } else {
        std::cout << "Not all flags are true" << std::endl;
    }

    return 0;
}
*/
    // And with the above structure add functions to input a new value, adding it to the first value and "pushing" the last value out. 
    // This could be done with #include <queue>. 
  } else {
    time_spent_settled = 0;
  }

  time_spent_running+=10;

  return output;
}

/**
 * Computes whether or not the movement has settled.
 * The robot is considered settled when error is less than settle_error 
 * for a duration of settle_time, or if the function has gone on for 
 * longer than timeout. Otherwise it is not settled.
 * 
 * @return Whether the movement is settled.
 */

bool PID::is_settled(){
  if (time_spent_running>timeout && timeout != 0){cout<<"time out reached, settle time ="<<time_spent_settled<<endl;
    return(true);
  } // If timeout does equal 0, the move will never actually time out. Setting timeout to 0 is the 
    // equivalent of setting it to infinity.
  if (time_spent_settled>settle_time){ cout<<"settle_time reached\n";
    return(true);
    // to add the function with the sliding window + consecutive checks, we'd only return(true) upon checking the entire
    // queue to make sure that every flag is set to true. To change this throughout the code, we'd probably make it so that
    // instead of configuring the settle_time, you'd change the # of settle_flags. 
  }
  // #################
  // What advantage could this bring?
  // #################
  // settle_time is in theory the safest option. If you don't reach the angle in time due to being blocked,
  // there's a chance you can continue on with your code and score.
  // You will always reach the timeout time, which makes the code consistent.
  // However, it can become frustrating when the robot is inconsistent due to physical limitations,
  // and sometimes the robot may over/undershoot repeatedly, and settle_time being a static value cannot 
  // account for this. So, we propose that we have a more dynamic method of checking whether the robot is settled:
  // We know that settle_time is based on "if robot is within x of y angle, we increment settle_time, so that
  // once the robot is definitely at y angle, it will relatively quickly (based on the settle_time) be considered to be completely settled."
  // We then can change the logic to be "if robot is within x of y angle for z consecutive times, then we
  // can be absolutely sure that the robot is completely settled." This ensures that we can use a singular value, 
  // Z, for the number of flags, without needing to bother with the settle_time. Additionally, tinkering 
  // with settle_time throughout the code can produce varying results due to physical variables,
  // but with our flag implementation we can dynamically allocate time to the robot to find the angle.
  // ----
  // #################
  // Downsides
  // #################
  // If the robot's inertial sensor is wrong or extremely inconsistent, it may completely break the logic.
  // A way to safeguard against this is to instead check for the number of flags set to true, rather than 
  // every flag being set to true. This path could lead to a robot missing the target though,
  // so it's better to go with all flags set to true.
  // ----
  return(false);
}
