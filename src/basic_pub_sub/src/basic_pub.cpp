#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"                            //ROS2 CPP Client Library
#include "std_msgs/msg/string.hpp"                      //ROS2 Messages

#include "rcl_interfaces/msg/set_parameters_result.hpp" //For Parameter Callbacks

using namespace std::chrono_literals;

/*****************************************************************************/
class BasicPublisher : public rclcpp::Node
/*****************************************************************************/
{
  public:
    BasicPublisher();

  private:

    void timer_callback();

    rcl_interfaces::msg::SetParametersResult paramCB(const std::vector<rclcpp::Parameter> &params);
    

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    size_t count_;

    OnSetParametersCallbackHandle::SharedPtr cb_handle; 
    const rcl_interfaces::msg::ParameterDescriptor param_desc;

};

/*****************************************************************************/
BasicPublisher::BasicPublisher(): Node("publisher"), count_(0)
/*****************************************************************************/
{
  /* QoS */
  rclcpp::QoS qos(10);
  qos.keep_last(10);
  qos.reliable();
  qos.durability_volatile();

  /* Publishers */
  publisher = this->create_publisher<std_msgs::msg::String>("topic", qos);

  /* Timers */
  timer = this->create_wall_timer(
  500ms, std::bind(&BasicPublisher::timer_callback, this));

  /* ROS Parameters */
  rcl_interfaces::msg::ParameterDescriptor param_desc;

  /* ROS Static Parameters */
  this->declare_parameter("my_string", "Hello from executable! ");
  //this->declare_parameter("my_string"); //Throws 'rclcpp::exceptions::ParameterNotDeclaredException' if un-init

  /* ROS Dyanmic Parameters */
  cb_handle = this->add_on_set_parameters_callback(std::bind(&BasicPublisher::paramCB, this, std::placeholders::_1));
}
/*****************************************************************************/
void BasicPublisher::timer_callback()
/*****************************************************************************/
{
  ///std::string str_prefix = this->get_parameter("my_string").get_parameter_value().get<std::string>();
  std::string str_prefix = this->get_parameter("my_string").as_string();

  auto message = std_msgs::msg::String();
  
  message.data = str_prefix + " " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher->publish(message);
}

/*****************************************************************************/
rcl_interfaces::msg::SetParametersResult BasicPublisher::paramCB(const std::vector<rclcpp::Parameter> &params)
/*****************************************************************************/
{
  rcl_interfaces::msg::SetParametersResult result;

  for( auto &tmp : params)
  {
    if (!tmp.get_name().compare("my_string"))
     { 
      if(tmp.as_string().find("-") != std::string::npos)
      {
       result.successful = false;
       result.reason = "Input string have dash!";
       return result;
      }
     } 
  }
  result.successful = true;
  result.reason = "Valid input";
  return result;
}

/*****************************************************************************/
int main(int argc, char * argv[])
/*****************************************************************************/
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasicPublisher>());
  rclcpp::shutdown();
  return 0;
}
