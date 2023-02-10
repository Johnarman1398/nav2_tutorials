#include "rclcpp/rclcpp.hpp"
#include "matrix_interfaces/srv/det_matrix.hpp"

#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <vector>

using std::placeholders::_1;
using namespace std::chrono_literals;

class client_node : public rclcpp::Node
{
  public:
    client_node(); //constructor

  private:
    
    void timer_callback();

    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    rclcpp::Client<matrix_interfaces::srv::DetMatrix>::SharedPtr client_ptr_;

    rclcpp::TimerBase::SharedPtr timer_ptr_;
};

client_node::client_node() : Node("det_matrix_client")
{
    client_cb_group_ = nullptr;
    timer_cb_group_ = nullptr;
    // client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    client_ptr_ = this->create_client<matrix_interfaces::srv::DetMatrix>("find_determinant", rmw_qos_profile_services_default, client_cb_group_);
    timer_ptr_ = this->create_wall_timer(1s, std::bind(&client_node::timer_callback, this), timer_cb_group_);
}

void client_node::timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "Sending request");
    auto request = std::make_shared<matrix_interfaces::srv::DetMatrix::Request>();

    // Form a matrix
    request->matrix[0] = 2;
    request->matrix[1] = 4;
    request->matrix[2] = 6;
    request->matrix[3] = 8;

    auto result_future = client_ptr_->async_send_request(request);
    std::future_status status = result_future.wait_for(10s);  
    if(status == std::future_status::ready) 
    {
      RCLCPP_INFO(this->get_logger(), "Received response, The determinant is %ld", result_future.get()->det);
      //RCLCPP_INFO(this->get_logger(), "Ending Task");
      //rclcpp::shutdown();
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "No response");
    }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto client = std::make_shared<client_node>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}



// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);

//   std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("det_matrix_client");
//   rclcpp::Client<matrix_interfaces::srv::DetMatrix>::SharedPtr client =
//     node->create_client<matrix_interfaces::srv::DetMatrix>("find_determinant");

//   auto request = std::make_shared<matrix_interfaces::srv::DetMatrix::Request>();
//   request->matrix[0] = 2;
//   request->matrix[1] = 4;
//   request->matrix[2] = 6;
//   request->matrix[3] = 8;

//   while (!client->wait_for_service(1s)) {
//     if (!rclcpp::ok()) {
//       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
//       return 0;
//     }
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
//   }

//   auto result = client->async_send_request(request);
//   // Wait for the result.
//   // if (rclcpp::spin_until_future_complete(node, result) ==
//   //   rclcpp::FutureReturnCode::SUCCESS)
//   // {
//   //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "received");
//   // } else {
//   //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service find_determinant");
//   // }

//   rclcpp::spin_until_future_complete(node, result);
//   auto response = result.get();
//   RCLCPP_INFO(node->get_logger(), "received determinant: %d", response->det);


//   rclcpp::shutdown();
//   return 0;
// }