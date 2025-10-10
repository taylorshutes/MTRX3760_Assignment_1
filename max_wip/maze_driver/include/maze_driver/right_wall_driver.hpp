#ifndef RIGHT_WALL_DRIVER_HPP
#define RIGHT_WALL_DRIVER_HPP

#include "maze_driver/base_driver.hpp"

/**
 * RightWallDriver implements a right-wall-following maze solver strategy.
 * It derives from BaseDriver and overrides compute_command() with its control logic.
 * 
 * It operates by  FILL IN XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
 */
class RightWallDriver : public BaseDriver
{
public:
  // simple constructor and deconstructor
  RightWallDriver();
  ~RightWallDriver();

private:
  // store current action state
  enum class State { FORWARD, TURN, FIND_RIGHT };
  State state_;

  double target_yaw_{0.0};          // target yaw to assist whilst turning
  bool skip_right_{false};          // bool values to assist in decision making 
  bool green_wall_seen_{false};
  bool right_wall_seen_{false};
  int counter_{0};                  // lap counter to change behaviour after each lap to assist in locating islands
  int target_counter_{1};

  geometry_msgs::msg::TwistStamped compute_command() override;    // override with own logic for computing command

  // helper functions       
  bool reached_target_yaw(double tol);    
  double normalize_angle(double a);
};

#endif // RIGHT_WALL_DRIVER_HPP
