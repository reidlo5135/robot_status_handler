#include "sensor_status_publisher/sensor_status_publisher.hxx"

sensor_status_publisher::StatusPublisher::StatusPublisher()
    : Node(RCL_NODE_NAME),
      imu_cb_(nullptr),
      scan_cb_(nullptr),
      battery_state_cb_(nullptr)
{
    this->node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

    RCLCPP_INFO(this->node_ptr_->get_logger(), "[%s] has been started...", RCL_NODE_NAME);
    RCLCPP_LINE_INFO();

    this->timer_ = this->node_ptr_->create_wall_timer(std::chrono::seconds(1), std::bind(&sensor_status_publisher::StatusPublisher::timer_callback, this));

    this->imu_status_publisher_cb_group_ = this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions imu_status_publisher_opts;
    imu_status_publisher_opts.callback_group = this->imu_status_publisher_cb_group_;

    this->imu_status_publisher_ = this->node_ptr_->create_publisher<robot_status_msgs::msg::SensorStatus>(
        RCL_IMU_STATUS_PUBLISHER_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        imu_status_publisher_opts);

    this->flag_rcl_connections(RCL_PUBLISHER_FLAG, RCL_IMU_STATUS_PUBLISHER_TOPIC);

    this->imu_subscription_cb_group_ = this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions imu_subscription_opts;
    imu_subscription_opts.callback_group = this->imu_subscription_cb_group_;

    this->imu_subscription_ = this->node_ptr_->create_subscription<sensor_msgs::msg::Imu>(
        RCL_IMU_SUBSCRIPTION_TOPIC,
        rclcpp::SensorDataQoS(),
        std::bind(&sensor_status_publisher::StatusPublisher::imu_subscription_cb, this, _1),
        imu_subscription_opts);

    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_IMU_SUBSCRIPTION_TOPIC);

    this->scan_status_publisher_cb_group_ = this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions scan_status_publisher_opts;
    scan_status_publisher_opts.callback_group = this->scan_status_publisher_cb_group_;

    this->scan_status_publisher_ = this->node_ptr_->create_publisher<robot_status_msgs::msg::SensorStatus>(
        RCL_SCAN_STATUS_PUBLISHER_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        scan_status_publisher_opts);

    this->flag_rcl_connections(RCL_PUBLISHER_FLAG, RCL_SCAN_STATUS_PUBLISHER_TOPIC);

    this->scan_subscription_cb_group_ = this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions scan_subscription_opts;
    scan_subscription_opts.callback_group = this->scan_subscription_cb_group_;

    this->scan_subscription_ = this->node_ptr_->create_subscription<sensor_msgs::msg::LaserScan>(
        RCL_SCAN_SUBSCRIPTION_TOPIC,
        rclcpp::SensorDataQoS(),
        std::bind(&sensor_status_publisher::StatusPublisher::scan_subscription_cb, this, _1),
        scan_subscription_opts);

    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_SCAN_SUBSCRIPTION_TOPIC);

    this->gps_status_publisher_cb_group_ = this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions gps_status_publisher_opts;
    gps_status_publisher_opts.callback_group = this->gps_status_publisher_cb_group_;

    this->gps_status_publisher_ = this->node_ptr_->create_publisher<robot_status_msgs::msg::SensorStatus>(
        RCL_GPS_STATUS_PUBLISHER_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        gps_status_publisher_opts);

    this->flag_rcl_connections(RCL_PUBLISHER_FLAG, RCL_GPS_STATUS_PUBLISHER_TOPIC);

    this->gps_subscription_cb_group_ = this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions gps_subscription_opts;
    gps_subscription_opts.callback_group = this->gps_subscription_cb_group_;

    this->gps_subscription_ = this->node_ptr_->create_subscription<sensor_msgs::msg::NavSatFix>(
        RCL_GPS_SUBSCRIPTION_TOPIC,
        rclcpp::SensorDataQoS(),
        std::bind(&sensor_status_publisher::StatusPublisher::gps_subscription_cb, this, _1),
        gps_subscription_opts);

    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_GPS_SUBSCRIPTION_TOPIC);

    this->battery_state_status_publisher_cb_group_ = this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions battery_state_status_publisher_opts;
    battery_state_status_publisher_opts.callback_group = this->battery_state_status_publisher_cb_group_;

    this->battery_state_status_publisher_ = this->node_ptr_->create_publisher<robot_status_msgs::msg::SensorStatus>(
        RCL_BATTERY_STATE_STATUS_PUBLISHER_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        battery_state_status_publisher_opts);

    this->flag_rcl_connections(RCL_PUBLISHER_FLAG, RCL_BATTERY_STATE_STATUS_PUBLISHER_TOPIC);

    this->battery_error_status_subscription_cb_group_ = this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions battery_error_status_subscription_opts;
    battery_error_status_subscription_opts.callback_group = this->battery_error_status_subscription_cb_group_;

    this->battery_error_status_subscription_ = this->node_ptr_->create_subscription<robot_status_msgs::msg::SensorStatus>(
        RCL_BATTERY_ERROR_STATUS_SUBSCRIPTION_TOPIC,
        rclcpp::SensorDataQoS(),
        std::bind(&sensor_status_publisher::StatusPublisher::battery_error_status_subscription_cb, this, _1),
        battery_error_status_subscription_opts);

    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_BATTERY_ERROR_STATUS_SUBSCRIPTION_TOPIC);
}

sensor_status_publisher::StatusPublisher::~StatusPublisher()
{
}

void sensor_status_publisher::StatusPublisher::signal_handler(int signal_input)
{
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "===== %s has been terminated with SIG [%d] =====", RCL_NODE_NAME, signal_input);
    signal(signal_input, SIG_IGN);
    exit(RCL_STOP_FLAG);
}

void sensor_status_publisher::StatusPublisher::flag_rcl_connections(const char *connection_type, const char *connection_name)
{
    RCLCPP_INFO(this->node_ptr_->get_logger(), "RCL [%s - %s] created...", connection_type, connection_name);
    RCLCPP_LINE_INFO();
}

void sensor_status_publisher::StatusPublisher::timer_callback()
{
    bool is_imu_cb_nullptr = (this->imu_cb_ == nullptr);

    if (is_imu_cb_nullptr)
    {
        RCLCPP_ERROR(this->node_ptr_->get_logger(), RCL_IMU_STATUS_NULL_STATUS_MESSAGE);
        RCLCPP_LINE_ERROR();

        const robot_status_msgs::msg::SensorStatus &imu_null_status = this->build_sensor_status(
            RCL_IMU_STATUS_HEADER_FRAME_ID,
            RCL_IMU_STATUS_NULL_STATUS_CODE,
            RCL_IMU_STATUS_NULL_STATUS_MESSAGE);

        this->imu_status_publisher_->publish(imu_null_status);
    }
    else
    {
        RCLCPP_INFO(this->node_ptr_->get_logger(), RCL_IMU_STATUS_OK_STATUS_MESSAGE);
        RCLCPP_LINE_INFO();

        const robot_status_msgs::msg::SensorStatus &imu_ok_status = this->build_sensor_status(
            RCL_IMU_STATUS_HEADER_FRAME_ID,
            RCL_IMU_STATUS_OK_STATUS_CODE,
            RCL_IMU_STATUS_OK_STATUS_MESSAGE);

        this->imu_status_publisher_->publish(imu_ok_status);
        this->imu_cb_ = nullptr;
    }

    bool is_scan_cb_nullptr = (this->scan_cb_ == nullptr);

    if (is_scan_cb_nullptr)
    {
        RCLCPP_ERROR(this->node_ptr_->get_logger(), RCL_SCAN_STATUS_NULL_STATUS_MESSAGE);
        RCLCPP_LINE_ERROR();

        const robot_status_msgs::msg::SensorStatus &scan_null_status = this->build_sensor_status(
            RCL_SCAN_STATUS_HEADER_FRAME_ID,
            RCL_SCAN_STATUS_NULL_STATUS_CODE,
            RCL_SCAN_STATUS_NULL_STATUS_MESSAGE);

        this->scan_status_publisher_->publish(scan_null_status);
    }
    else
    {
        RCLCPP_INFO(this->node_ptr_->get_logger(), RCL_SCAN_STATUS_OK_STATUS_MESSAGE);
        RCLCPP_LINE_INFO();

        const robot_status_msgs::msg::SensorStatus &scan_ok_status = this->build_sensor_status(
            RCL_SCAN_STATUS_HEADER_FRAME_ID,
            RCL_SCAN_STATUS_OK_STATUS_CODE,
            RCL_SCAN_STATUS_OK_STATUS_MESSAGE);

        this->scan_status_publisher_->publish(scan_ok_status);
        this->scan_cb_ = nullptr;
    }

    bool is_gps_cb_nullptr = (this->gps_cb_ == nullptr);

    if (is_gps_cb_nullptr)
    {
        RCLCPP_ERROR(this->node_ptr_->get_logger(), RCL_GPS_STATUS_NULL_STATUS_MESSAGE);
        RCLCPP_LINE_ERROR();

        const robot_status_msgs::msg::SensorStatus &gps_null_status = this->build_sensor_status(
            RCL_GPS_STATUS_HEADER_FRAME_ID,
            RCL_GPS_STATUS_NULL_STATUS_CODE,
            RCL_GPS_STATUS_NULL_STATUS_MESSAGE);

        this->gps_status_publisher_->publish(gps_null_status);
    }
    else
    {
        RCLCPP_INFO(this->node_ptr_->get_logger(), RCL_GPS_STATUS_OK_STATUS_MESSAGE);
        RCLCPP_LINE_INFO();

        const robot_status_msgs::msg::SensorStatus &gps_ok_status = this->build_sensor_status(
            RCL_GPS_STATUS_HEADER_FRAME_ID,
            RCL_GPS_STATUS_OK_STATUS_CODE,
            RCL_GPS_STATUS_OK_STATUS_MESSAGE);

        this->gps_status_publisher_->publish(gps_ok_status);
        this->gps_cb_ = nullptr;
    }
}

std_msgs::msg::Header sensor_status_publisher::StatusPublisher::build_sensor_status_header(const char *header_frame_id)
{
    std::chrono::_V2::system_clock::time_point current_time = std::chrono::system_clock::now();
    std::chrono::_V2::system_clock::duration current_time_duration = current_time.time_since_epoch();

    const int32_t &current_time_sec = std::chrono::duration_cast<std::chrono::seconds>(current_time_duration).count();
    const int32_t &current_time_nanosec = current_time_sec % 1000000000;

    builtin_interfaces::msg::Time::UniquePtr stamp = std::make_unique<builtin_interfaces::msg::Time>();
    stamp->set__sec(current_time_sec);
    stamp->set__nanosec(current_time_nanosec);

    std_msgs::msg::Header::UniquePtr header = std::make_unique<std_msgs::msg::Header>();
    header->set__frame_id(header_frame_id);

    const builtin_interfaces::msg::Time &&stamp_moved = std::move(*stamp);
    header->set__stamp(stamp_moved);

    const std_msgs::msg::Header &&header_moved = std::move(*header);

    return header_moved;
}

robot_status_msgs::msg::SensorStatus sensor_status_publisher::StatusPublisher::build_sensor_status(const char *header_frame_id, int32_t status_code, const char *status_message)
{
    RCLCPP_INFO(
        this->node_ptr_->get_logger(),
        "building sensor_status\n\theader_frame_id : [%s]\n\tstatus_code : [%d]\n\tmessage : [%s]",
        header_frame_id,
        status_code,
        status_message);
    RCLCPP_LINE_INFO();

    robot_status_msgs::msg::SensorStatus::UniquePtr sensor_status = std::make_unique<robot_status_msgs::msg::SensorStatus>();

    const std_msgs::msg::Header &built_header = this->build_sensor_status_header(header_frame_id);
    sensor_status->set__header(built_header);
    sensor_status->set__status_code(status_code);
    sensor_status->set__status_message(status_message);

    const robot_status_msgs::msg::SensorStatus &&sensor_status_moved = std::move(*sensor_status);

    return sensor_status_moved;
}

void sensor_status_publisher::StatusPublisher::imu_subscription_cb(const sensor_msgs::msg::Imu::SharedPtr imu_subscription_cb_data)
{
    this->imu_cb_ = imu_subscription_cb_data;
}

void sensor_status_publisher::StatusPublisher::scan_subscription_cb(const sensor_msgs::msg::LaserScan::SharedPtr scan_subscription_cb_data)
{
    this->scan_cb_ = scan_subscription_cb_data;
}

void sensor_status_publisher::StatusPublisher::gps_subscription_cb(const sensor_msgs::msg::NavSatFix::SharedPtr gps_subscription_cb_data)
{
    this->gps_cb_ = gps_subscription_cb_data;
}

void sensor_status_publisher::StatusPublisher::battery_error_status_publish(int32_t status_code, const char *status_message)
{
    const robot_status_msgs::msg::SensorStatus &battery_state_error_status = this->build_sensor_status(
        RCL_BATTERY_STATE_STATUS_HEADER_FRAME_ID,
        status_code,
        status_message);

    this->battery_state_status_publisher_->publish(battery_state_error_status);
}

void sensor_status_publisher::StatusPublisher::battery_error_status_subscription_cb(const robot_status_msgs::msg::SensorStatus::SharedPtr battery_error_status_subscription_cb_data)
{
    const int32_t &status_code = battery_error_status_subscription_cb_data->status_code;
    const char *status_message = battery_error_status_subscription_cb_data->status_message.c_str();

    this->battery_error_status_publish(status_code, status_message);
}