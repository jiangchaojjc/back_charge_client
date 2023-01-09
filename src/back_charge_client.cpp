/*
author:jiangchao
date:2022.12.23
description:back charge server
*/

#include "back_charge_client/back_charge_client.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

/* This example creates a subclass of Node and uses std::bind() to control robot
 * back to charge */

namespace action_ChargeBack {

ChargeBackClient::ChargeBackClient(const rclcpp::NodeOptions &options)
    : Node("ChargeBackClient", options) {
  RCLCPP_INFO(this->get_logger(), "backfunction  client start");
  using namespace std::placeholders;

  this->client_ptr_ =
      rclcpp_action::create_client<ChargeBackAction>(this, "chargeback");
  // this->timer_ =
  //     this->create_wall_timer(std::chrono::milliseconds(500),
  //                             std::bind(&ChargeBackClient::send_goal, this));
  back_charge_trig();
}

void ChargeBackClient::back_charge_trig() {
  using namespace std::placeholders;
  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto back_trig = ChargeBackAction::Goal();
  back_trig.back_charge = true;
  std::stringstream ss;
  ss << "goal send: ";

  ss << back_trig.back_charge << " ";

  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  RCLCPP_INFO(this->get_logger(), "send_goal_options");
  auto send_goal_options =
      rclcpp_action::Client<ChargeBackAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&ChargeBackClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&ChargeBackClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&ChargeBackClient::result_callback, this, _1);

  this->client_ptr_->async_send_goal(back_trig, send_goal_options);
}

void ChargeBackClient::goal_response_callback(
    std::shared_future<GoalHandleChargeBackAction::SharedPtr> future) {
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void ChargeBackClient::feedback_callback(
    GoalHandleChargeBackAction::SharedPtr,
    const std::shared_ptr<const ChargeBackAction::Feedback> feedback) {
  std::stringstream ss;
  ss << "feedback_callback received: ";

  ss << feedback->current_status << " ";

  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}

void ChargeBackClient::result_callback(
    const GoalHandleChargeBackAction::WrappedResult &result) {
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }

  std::stringstream ss;
  ss << "Result received: ";

  ss << result.result->final_status << " ";

  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  rclcpp::shutdown();
}

ChargeBackClient::~ChargeBackClient() {}

} // namespace action_ChargeBack
