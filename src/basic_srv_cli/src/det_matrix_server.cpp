#include "rclcpp/rclcpp.hpp"
#include "matrix_interfaces/srv/det_matrix.hpp"

#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <vector>
#include <thread>

using std::placeholders::_1;
using namespace std::chrono_literals;

class server_node : public rclcpp::Node
{
  public:
    server_node(); //constructor

  private:
    rclcpp::Service<matrix_interfaces::srv::DetMatrix>::SharedPtr det_matrix_service;

    void det(const std::shared_ptr<matrix_interfaces::srv::DetMatrix::Request> request,
          std::shared_ptr<matrix_interfaces::srv::DetMatrix::Response> response);
};

server_node::server_node() : Node("det_matrix_server")
{
  det_matrix_service = this->create_service<matrix_interfaces::srv::DetMatrix>("find_determinant", std::bind(&server_node::det, this, std::placeholders::_1, std::placeholders::_2));
}

void server_node::det(const std::shared_ptr<matrix_interfaces::srv::DetMatrix::Request> request,
          std::shared_ptr<matrix_interfaces::srv::DetMatrix::Response> response)
{
  //Find Determinant

  std::this_thread::sleep_for(2s); // simulate longer process 
  response->det = (request->matrix[0] * request->matrix[3]) - (request->matrix[1] * request->matrix[2]);

  

   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\n %ld %ld\n %ld %ld",
                  request->matrix[0], request->matrix[1], request->matrix[2], request->matrix[3]);
   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->det);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<server_node>());
  rclcpp::shutdown();
  return 0;
}