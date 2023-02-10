#include <functional>
#include <memory>
#include <thread>

#include "count_interfaces/action/count.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace example_action
{

class CountActionServer : public rclcpp::Node
{
    public:
    using Counter = count_interfaces::action::Count;
    using GoalHandleCount = rclcpp_action::ServerGoalHandle<Counter>;

    explicit CountActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
    rclcpp_action::Server<Counter>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Counter::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleCount> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleCount> goal_handle);

    void execute(const std::shared_ptr<GoalHandleCount> goal_handle);

};

CountActionServer::CountActionServer(const rclcpp::NodeOptions& options) : Node("count_action_server", options)
{
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Counter>(
    this,
    "count",
    std::bind(&CountActionServer::handle_goal, this, _1, _2),
    std::bind(&CountActionServer::handle_cancel, this, _1),
    std::bind(&CountActionServer::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse CountActionServer::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Counter::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Count until %d", goal->request);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CountActionServer::handle_cancel(const std::shared_ptr<GoalHandleCount> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CountActionServer::handle_accepted(const std::shared_ptr<GoalHandleCount> goal_handle)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&CountActionServer::execute, this, _1), goal_handle}.detach();
}

void CountActionServer::execute(const std::shared_ptr<GoalHandleCount> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Counter::Feedback>();
    int & current = feedback->feedback;
    
    auto result = std::make_shared<Counter::Result>();

    for (int i = 0; (i < goal->request) && rclcpp::ok(); ++i) {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
    result->result = current;
    goal_handle->canceled(result);
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    return;
    }


    current += 1;
    // Publish feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Publish feedback");

    loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->result = current;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}

} //end of namespace example_action
RCLCPP_COMPONENTS_REGISTER_NODE(example_action::CountActionServer)