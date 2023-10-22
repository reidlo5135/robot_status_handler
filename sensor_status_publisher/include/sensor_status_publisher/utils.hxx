#ifndef UTILS__HXX
#define UTILS__HXX

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <robot_status_msgs/msg/sensor_status.hpp>

/**
 * ------------------------------------------------------
 * ------------------ RCL AREA START --------------------
 * ------------------------------------------------------
 */

/**
 * @brief static const instance for define name of rclcpp::Node
 */
static constexpr const char *RCL_NODE_NAME = "sensor_status_publisher";

/**
 * @brief static const instance for define default value of rclcpp::QoS
 */
static constexpr const int &RCL_DEFAULT_QOS = 10;

/**
 * @brief static const instance for define int for flag stopping RCL
 */
static constexpr const int &RCL_STOP_FLAG = 0;

/**
 * @brief static const instance for define default Double value
 */
static constexpr const double &RCL_DEFAULT_DOUBLE = 0.0;

/**
 * @brief static const instance for define flag of rclcpp::Subscription
 */
static constexpr const char *RCL_SUBSCRIPTION_FLAG = "subscription";

/**
 * @brief static const instance for define flag of rclcpp::Publisher
 */
static constexpr const char *RCL_PUBLISHER_FLAG = "publisher";


static constexpr const char *RCL_IMU_STATUS_HEADER_FRAME_ID = "imu_status";

static constexpr const int32_t &RCL_IMU_STATUS_NULL_STATUS_CODE = -1000;

static constexpr const char *RCL_IMU_STATUS_NULL_STATUS_MESSAGE = "IMU is NULL";

static constexpr const int32_t &RCL_IMU_STATUS_OK_STATUS_CODE = 1000;

static constexpr const char *RCL_IMU_STATUS_OK_STATUS_MESSAGE = "IMU is OK";

static constexpr const char *RCL_IMU_STATUS_PUBLISHER_TOPIC = "/imu/status";

static constexpr const char *RCL_IMU_SUBSCRIPTION_TOPIC = "/imu";


static constexpr const char *RCL_SCAN_STATUS_HEADER_FRAME_ID = "scan_status";

static constexpr const int32_t &RCL_SCAN_STATUS_NULL_STATUS_CODE = -1001;

static constexpr const char *RCL_SCAN_STATUS_NULL_STATUS_MESSAGE = "LiDAR is NULL";

static constexpr const int32_t &RCL_SCAN_STATUS_OK_STATUS_CODE = 1001;

static constexpr const char *RCL_SCAN_STATUS_OK_STATUS_MESSAGE = "LiDAR is OK";

static constexpr const char *RCL_SCAN_STATUS_PUBLISHER_TOPIC = "/scan/status";

static constexpr const char *RCL_SCAN_SUBSCRIPTION_TOPIC = "/scan";


static constexpr const char *RCL_GPS_STATUS_HEADER_FRAME_ID = "gps_status";

static constexpr const int32_t &RCL_GPS_STATUS_NULL_STATUS_CODE = -1002;

static constexpr const char *RCL_GPS_STATUS_NULL_STATUS_MESSAGE = "GPS is NULL";

static constexpr const int32_t &RCL_GPS_STATUS_OK_STATUS_CODE = 1002;

static constexpr const char *RCL_GPS_STATUS_OK_STATUS_MESSAGE = "GPS is OK";

static constexpr const char *RCL_GPS_STATUS_PUBLISHER_TOPIC = "/ublox/fix/status";

static constexpr const char *RCL_GPS_SUBSCRIPTION_TOPIC = "/ublox/fix";


static constexpr const char *RCL_BATTERY_STATE_STATUS_HEADER_FRAME_ID = "battery_state_status";

static constexpr const int32_t &RCL_BATTERY_STATE_STATUS_NULL_STATUS_CODE = -1003;

static constexpr const char *RCL_BATTERY_STATE_STATUS_NULL_STATUS_MESSAGE = "Battery is NULL";

static constexpr const int32_t &RCL_BATTERY_STATE_STATUS_OK_STATUS_CODE = 1003;

static constexpr const char *RCL_BATTERY_STATE_STATUS_OK_STATUS_MESSAGE = "Battery is OK";

static constexpr const char *RCL_BATTERY_STATE_STATUS_PUBLISHER_TOPIC = "/battery_state/status";

static constexpr const char *RCL_BATTERY_STATE_SUBSCRIPTION_TOPIC = "/battery_state";

/**
 * --------------------------------------------------.---
 * ------------------- RCL AREA END ---------------------
 * ------------------------------------------------------
 */


/**
 * @brief define macros area
 */
#define RCLCPP_LINE_INFO() \
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "LINE : [%d]", __LINE__)

#define RCLCPP_LINE_ERROR() \
    RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "LINE : [%d]", __LINE__)

#define RCLCPP_LINE_WARN() \
    RCUTILS_LOG_WARN_NAMED(RCL_NODE_NAME, "LINE : [%d]", __LINE__)

/**
 * @brief using namespace area
 */
using std::placeholders::_1;
using std::placeholders::_2;

#endif