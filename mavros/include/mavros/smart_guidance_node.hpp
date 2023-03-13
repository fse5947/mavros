#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/callback_group.hpp"

#include <eigen3/Eigen/Eigen>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include <mavros/frame_tf.hpp>

#include "mavros_msgs/msg/command_code.hpp"
#include "mavros_msgs/msg/waypoint.hpp"
#include "mavros_msgs/msg/waypoint_reached.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/vfr_hud.hpp"

#include "mavros_msgs/srv/param_set_v2.hpp"
#include "mavros_msgs/srv/command_long.hpp"
#include "mavros_msgs/srv/waypoint_push.hpp"

#include <soaring_interface/msg/aircraft_state.hpp>
#include <soaring_interface/msg/wind_state.hpp>
#include <soaring_interface/msg/airspeed_flaps_command.hpp>
#include <soaring_interface/msg/aircraft_configuration.hpp>

#include <soaring_interface/srv/upload_flight_plan.hpp>
#include <soaring_interface/srv/ground_control_command.hpp>

#include <soaring_interface/utils/soaring_modes.hpp>

class SmartGuidanceCom : public rclcpp::Node
{
public:
    SmartGuidanceCom();

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<soaring_interface::msg::AircraftState>::SharedPtr aircraft_state_publisher_;
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
    rclcpp::Client<soaring_interface::srv::GroundControlCommand>::SharedPtr ground_control_client_;

    rclcpp::callback_group::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::callback_group::CallbackGroup::SharedPtr timer_cb_group_;

    void PublishAircraftState() const;
    void PublishWindState(float wind_north, float wind_east) const;
    void SendGroundCommand(uint8_t system_state, uint8_t thermalling_state);
    void SetMavParameter(const char *param_id, uint8_t param_value, uint8_t param_type);
    void SendMavCommand(uint16_t command_id, float param1, float param2, float param3,
                        float param4, float param5, float param6, float param7);
    void SendMavCommand(uint16_t);
    void SendMavCommand(uint16_t, float param1);
    void SendMavCommand(uint16_t, float param1, float param2);
    void PushMavWaypoints(std::vector<mavros_msgs::msg::Waypoint> flight_path);
    template <typename T1, typename T2>
    void HandleClientRequest(T1 client_ptr, T2 request, std::string service_name = "Service");

    void VfrHudCallback(const mavros_msgs::msg::VfrHud::SharedPtr msg);
    void GlobalPoseCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void GlobalTwistCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void ImuDataCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void WindCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
    void AirspdFlapCallback(const soaring_interface::msg::AirspeedFlapsCommand::SharedPtr msg);
    void AircraftConfigCallback(const soaring_interface::msg::AircraftConfiguration::SharedPtr msg);
    void RcInCallback(const mavros_msgs::msg::RCIn::SharedPtr msg);
    void WaypointReachedCallback(const mavros_msgs::msg::WaypointReached::SharedPtr msg);
    void FlightPlanCallback(const std::shared_ptr<soaring_interface::srv::UploadFlightPlan::Request> request,
                            std::shared_ptr<soaring_interface::srv::UploadFlightPlan::Response> response);
    void TimerCallback();

    SmartGuidanceMode get_SmartGuidanceState(uint16_t rc_switch);
    ThermallingMode get_ThermallingState(uint16_t rc_switch);

    rclcpp::Logger node_logger_{rclcpp::get_logger("SmartGuidanceCom")};

    float indicated_airspeed_{0.0f};
    float airspd_cmd_{0.0f};
    float lat_{0.0f};
    float lon_{0.0f};
    float alt_{0.0f};
    float throttle_{0.0f};
    float velocity_x_{0.0f};
    float velocity_y_{0.0f};
    float velocity_z_{0.0f};
    float accel_x_{0.0f};
    float accel_y_{0.0f};
    float accel_z_{0.0f};
    float omega_x_{0.0f};
    float omega_y_{0.0f};
    float omega_z_{0.0f};

    int aircraft_state_publish_rate_{100};
    int num_waypoints_{0};
    int previous_smart_guidance_state_{-1};
    int previous_thermalling_state_{-1};

    std::vector<double> euler_{0.0, 0.0, 0.0};
    std::vector<double> omega_{0.0, 0.0, 0.0};

    std::vector<mavros_msgs::msg::Waypoint> flight_path_waypoints_;

    tf2::Vector3 omega_FRD_{0, 0, 0};
    tf2::Vector3 accel_FRD_{0, 0, 0};

    const tf2::Quaternion q_FLU_to_FRD = tf2::Quaternion(1, 0, 0, 0);
    const tf2::Quaternion q_ENU_to_NED = tf2::Quaternion(0.70711, 0.70711, 0, 0);

    static constexpr uint16_t kPwmLowMin = 900;
    static constexpr uint16_t kPwmLowMax = 1300;
    static constexpr uint16_t kPwmHighMin = 1700;
    static constexpr uint16_t kPwmHighMax = 2200;
};