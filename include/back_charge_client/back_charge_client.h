//
// Created by JC on 2022/11/21
//

#ifndef CHARGEBACK_H
#define CHARGEBACK_H

#include <string>

#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <iostream>
#include <thread> // std::this_thread::sleep_for

#include "rclcpp/rclcpp.hpp"
#include <chrono>

#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <tf2_ros/transform_broadcaster.h>

// Add Custom Message
#include "rcsbot_interface/msg/error_pub.hpp"
#include "rcsbot_interface/msg/io_pub.hpp"
#include "rcsbot_interface/msg/rob_state_pub.hpp"
#include "rcsbot_interface/msg/sonar_pub.hpp"

#include "charge_interface/action/charge_back.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "back_charge_client/visibility_control.h"

#define TARGET 12
#define kp 0.05
#define ki 0
#define kd 0.05
#define STEP 20

namespace action_ChargeBack {
class ChargeBackClient : public rclcpp::Node {
public:
  using ChargeBackAction = charge_interface::action::ChargeBack;
  using GoalHandleChargeBackAction =
      rclcpp_action::ClientGoalHandle<ChargeBackAction>;
  // ACTION_TUTORIALS_CPP_PUBLIC
  explicit ChargeBackClient(const rclcpp::NodeOptions &options);
  ~ChargeBackClient();
  // void state_callback();

private:
  void back_charge_trig();
  void goal_response_callback(
      std::shared_future<GoalHandleChargeBackAction::SharedPtr> future);
  void feedback_callback(
      GoalHandleChargeBackAction::SharedPtr,
      const std::shared_ptr<const ChargeBackAction::Feedback> feedback);
  void result_callback(const GoalHandleChargeBackAction::WrappedResult &result);
  rclcpp_action::Client<ChargeBackAction>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  int search_count;
  bool info_get;
  int nav2_goal_get;
};
} // namespace action_ChargeBack

RCLCPP_COMPONENTS_REGISTER_NODE(action_ChargeBack::ChargeBackClient)
#endif // CHARGEBACK_H
