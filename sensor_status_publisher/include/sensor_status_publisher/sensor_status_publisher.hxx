#ifndef SENSOR_STATUS_PUBLISHER__HXX
#define SENSOR_STATUS_PUBLISHER__HXX

#include "sensor_status_publisher/utils.hxx"

namespace sensor_status_publisher
{
    class StatusPublisher final : public rclcpp::Node
    {
    private:
        rclcpp::Node::SharedPtr node_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;

        sensor_msgs::msg::Imu::SharedPtr imu_cb_;
        rclcpp::CallbackGroup::SharedPtr imu_status_publisher_cb_group_;
        rclcpp::Publisher<robot_status_msgs::msg::SensorStatus>::SharedPtr imu_status_publisher_;
        rclcpp::CallbackGroup::SharedPtr imu_subscription_cb_group_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

        sensor_msgs::msg::LaserScan::SharedPtr scan_cb_;
        rclcpp::CallbackGroup::SharedPtr scan_status_publisher_cb_group_;
        rclcpp::Publisher<robot_status_msgs::msg::SensorStatus>::SharedPtr scan_status_publisher_;
        rclcpp::CallbackGroup::SharedPtr scan_subscription_cb_group_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

        sensor_msgs::msg::NavSatFix::SharedPtr gps_cb_;
        rclcpp::CallbackGroup::SharedPtr gps_status_publisher_cb_group_;
        rclcpp::Publisher<robot_status_msgs::msg::SensorStatus>::SharedPtr gps_status_publisher_;
        rclcpp::CallbackGroup::SharedPtr gps_subscription_cb_group_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;

        sensor_msgs::msg::BatteryState::SharedPtr battery_state_cb_;
        rclcpp::CallbackGroup::SharedPtr battery_state_status_publisher_cb_group_;
        rclcpp::Publisher<robot_status_msgs::msg::SensorStatus>::SharedPtr battery_state_status_publisher_;
        rclcpp::CallbackGroup::SharedPtr battery_state_subscription_cb_group_;
        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_subscription_;

        void flag_rcl_connections(const char *connection_type, const char *connection_name);
        void timer_callback();
        std_msgs::msg::Header build_sensor_status_header(const char *header_frame_id);
        robot_status_msgs::msg::SensorStatus build_sensor_status(const char *header_frame_id, int32_t status_code, const char *status_message);
        void imu_subscription_cb(const sensor_msgs::msg::Imu::SharedPtr imu_subscription_cb_data);
        void scan_subscription_cb(const sensor_msgs::msg::LaserScan::SharedPtr scan_subscription_cb_data);
        void gps_subscription_cb(const sensor_msgs::msg::NavSatFix::SharedPtr gps_subscription_cb_data);
        void battery_state_subscription_cb(const sensor_msgs::msg::BatteryState::SharedPtr battery_state_subscription_cb_data);

    public:
        explicit StatusPublisher();
        virtual ~StatusPublisher();
        static void signal_handler(int signal_input);
    };
}

#endif