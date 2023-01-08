#include "rclcpp/rclcpp.hpp"
#include "rclcpp/callback_group.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Eigen>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/vfr_hud.hpp"
#include <soaring_interface/msg/aircraft_state.hpp>
#include <soaring_interface/msg/wind_state.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <mavros/frame_tf.hpp>
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include <soaring_interface/msg/airspeed_flaps_command.hpp>
#include "mavros_msgs/srv/param_set_v2.hpp"
#include "mavros_msgs/srv/command_long.hpp"
#include "mavros_msgs/msg/command_code.hpp"
#include <soaring_interface/msg/aircraft_configuration.hpp>
#include "mavros_msgs/msg/rc_in.hpp"
#include <soaring_interface/msg/ground_control_command.hpp>
#include "mavros_msgs/srv/waypoint_push.hpp"
#include "mavros_msgs/msg/waypoint.hpp"
#include "mavros_msgs/msg/waypoint_reached.hpp"
#include <soaring_interface/srv/upload_flight_plan.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono;
using namespace std::chrono_literals;

#define PWM_LOW_MIN 900
#define PWM_LOW_MAX 1300
#define PWM_HIGH_MIN 1700
#define PWM_HIGH_MAX 2200

class SMART_GUIDANCE_COM : public rclcpp::Node
{
public:
    SMART_GUIDANCE_COM();

    enum SMART_GUIDANCE_MODE
    {
        SMART_GUIDANCE_MODE_DISABLED,
        SMART_GUIDANCE_MODE_SAFE,
        SMART_GUIDANCE_MODE_ACTIVE
    } smart_guidance_state{SMART_GUIDANCE_MODE_DISABLED};

    enum THERMALLING_MODE
    {
        THERMALLING_DISABLED,
        THERMALLING_ENABLED
    } thermalling_state{THERMALLING_DISABLED};

    enum SG_SYSTEM_MODE
    {
        SG_SYSTEM_DISABLED = -1,
        SG_SYSTEM_SAFE,
        SG_SYSTEM_ENABLED
    };

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<soaring_interface::msg::AircraftState>::SharedPtr aircraft_state_publisher_;
    rclcpp::Publisher<soaring_interface::msg::GroundControlCommand>::SharedPtr ground_command_publisher_;
    rclcpp::Publisher<soaring_interface::msg::WindState>::SharedPtr wind_state_publisher_;

    rclcpp::Subscription<mavros_msgs::msg::VfrHud>::SharedPtr vfr_hud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_twist_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr wind_sub_;
    rclcpp::Subscription<soaring_interface::msg::AirspeedFlapsCommand>::SharedPtr airspd_flap_sub_;
    rclcpp::Subscription<soaring_interface::msg::AircraftConfiguration>::SharedPtr aircraft_config_sub_;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_in_sub_;
    rclcpp::Subscription<mavros_msgs::msg::WaypointReached>::SharedPtr waypoint_reached_sub_;

    rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedPtr set_mav_param_client_;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr send_mav_command_client_;
    rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr push_mav_waypoints_client_;
    rclcpp::Service<soaring_interface::srv::UploadFlightPlan>::SharedPtr flight_plan_service_;

    rclcpp::callback_group::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::callback_group::CallbackGroup::SharedPtr timer_cb_group_;

    void publish_aircraft_state() const;
    void publish_wind_state(float wind_north, float wind_east) const;
    void publish_ground_command(uint8_t system_state, uint8_t thermalling_state) const;
    void set_mav_parameter(const char *param_id, uint8_t param_value, uint8_t param_type);
    void send_mav_command(uint16_t command_id, float param1, float param2, float param3,
                          float param4, float param5, float param6, float param7);
    void push_mav_waypoints(std::vector<mavros_msgs::msg::Waypoint> flight_path);

    SMART_GUIDANCE_COM::SMART_GUIDANCE_MODE get_smart_guidance_state(uint16_t rc_switch);
    SMART_GUIDANCE_COM::THERMALLING_MODE get_thermalling_state(uint16_t rc_switch);

    void timer_callback();

    tf2::Quaternion q_FLU_to_FRD = tf2::Quaternion(1, 0, 0, 0);
    tf2::Quaternion q_ENU_to_NED = tf2::Quaternion(0.70711, 0.70711, 0, 0);

    float i_airspeed, airspd_cmd;
    float lat, lon, alt;
    float throttle;
    float velocity_x, velocity_y, velocity_z;
    float accel_x, accel_y, accel_z;
    float omega_x, omega_y, omega_z;
    double euler[3], omega[3];
    tf2::Vector3 omega_FRD, accel_FRD;

    int num_waypoints;
    int previous_smart_guidance_state{-1};
    int previous_thermalling_state{-1};

    std::vector<mavros_msgs::msg::Waypoint> flight_path_waypoints;
};