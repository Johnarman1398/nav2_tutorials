#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
/*****************************************************************************/
class BasicSubscriber : public rclcpp::Node
/*****************************************************************************/
{
  public:
    BasicSubscriber();

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;
};

/*****************************************************************************/
BasicSubscriber::BasicSubscriber():Node("subscriber")
/*****************************************************************************/
{
    //QoS
    rclcpp::QoS qos(10);
    qos.keep_last(10);
    qos.reliable();
    qos.durability_volatile();

    subscriber = this->create_subscription<std_msgs::msg::String>(
    "topic", qos, std::bind(&BasicSubscriber::topic_callback, this, _1));
}
/*****************************************************************************/
void BasicSubscriber::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
/*****************************************************************************/
{
    RCLCPP_INFO(this->get_logger(), "I received: '%s'", msg->data.c_str());
}

/*****************************************************************************/
int main(int argc, char * argv[])
/*****************************************************************************/
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasicSubscriber>());
  rclcpp::shutdown();
  return 0;
}