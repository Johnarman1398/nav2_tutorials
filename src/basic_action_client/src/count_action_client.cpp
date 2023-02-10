#include <functional>
#include <future>
#include <memory>
#include <string>

#include "count_interfaces/action/count.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace example_action
{

class CountActionClient : public rclcpp::Node
{
    public:
    using Counter = count_interfaces::action::Count;
    using GoalHandleCount = rclcpp_action::ClientGoalHandle<Counter>;

    explicit CountActionClient(const rclcpp::NodeOptions& Options);

    void send_goal();

    private:
    rclcpp_action::Client<Counter> ::SharedPtr client_ptr;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(std::shared_future<GoalHandleCount::SharedPtr> future);

    void feedback_callback(GoalHandleCount::SharedPtr, const std::shared_ptr<const Counter::Feedback> feedback);

    void result_callback(const GoalHandleCount::WrappedResult& result);

    int current_goal;
};

CountActionClient::CountActionClient(const rclcpp::NodeOptions& Options) : Node("count_action_client", Options)
{
  this->declare_parameter("target", 20);
  current_goal = 20;
  
  this->client_ptr = rclcpp_action::create_client<Counter>(this, "count");
  this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CountActionClient::send_goal, this));
}

void CountActionClient::send_goal()
{
    using namespace std::placeholders;
    this->timer_->cancel();

    if(!this->client_ptr->wait_for_action_server())
    {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Enter an integer target");
    std::cin >> current_goal;
    auto goal_msg = Counter::Goal();
    goal_msg.request = current_goal;

    RCLCPP_INFO(this->get_logger(), "Sending goal: current target is %d", current_goal);


    auto send_goal_options = rclcpp_action::Client<Counter>::SendGoalOptions();

    send_goal_options.goal_response_callback =
    std::bind(&CountActionClient::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
    std::bind(&CountActionClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
    std::bind(&CountActionClient::result_callback, this, _1);

    this->client_ptr->async_send_goal(goal_msg, send_goal_options);
   // this->client_ptr->async_cancel_all_goals();
}

void CountActionClient::goal_response_callback(std::shared_future<GoalHandleCount::SharedPtr> future)
{
    auto goal_handle = future.get();
    if (!goal_handle) 
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void CountActionClient::feedback_callback(GoalHandleCount::SharedPtr, const std::shared_ptr<const Counter::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Current count: %d", feedback->feedback);
}

void CountActionClient::result_callback(const GoalHandleCount::WrappedResult& result)
{
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

    RCLCPP_INFO(this->get_logger(), "The goal integer is reached: %d", result.result->result);
   // rclcpp::shutdown();
}

}// end of namespace example_action

RCLCPP_COMPONENTS_REGISTER_NODE(example_action::CountActionClient)