#include "sensor_status_publisher/sensor_status_publisher.hxx"

int main(int argc, const char *const *argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node_ptr_ = std::make_shared<sensor_status_publisher::StatusPublisher>();

    signal(SIGINT, &sensor_status_publisher::StatusPublisher::signal_handler);
	signal(SIGTSTP, &sensor_status_publisher::StatusPublisher::signal_handler);

    rclcpp::spin(node_ptr_);
    rclcpp::shutdown();

    return RCL_STOP_FLAG;
}