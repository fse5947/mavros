#include "rclcpp/rclcpp.hpp"
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
#include <soaring_interface/msg/aircraft_configuration.hpp>
#include "mavros_msgs/msg/rc_in.hpp"
#include <soaring_interface/msg/ground_control_command.hpp>
#include <soaring_interface/msg/waypoint_vector.hpp>
#include <soaring_interface/msg/waypoint.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono;
using namespace std::chrono_literals;

class AUTOSOAR_COM : public rclcpp::Node
{
public:
    AUTOSOAR_COM() : Node("autosoar_node")
    {

        auto sensor_qos = rclcpp::SensorDataQoS();

        aircraft_state_publisher_ =
            this->create_publisher<soaring_interface::msg::AircraftState>("/aircraft_state", 10);

        ground_command_publisher_ =
            this->create_publisher<soaring_interface::msg::GroundControlCommand>("/ground_command", 10);

        vfr_hud_sub_ =
            this->create_subscription<mavros_msgs::msg::VfrHud>(
                "/mavros/vfr_hud", sensor_qos, [this](const mavros_msgs::msg::VfrHud::SharedPtr msg)
                {
                i_airspeed = msg->airspeed;
                throttle = msg->throttle;
                alt = msg->altitude; });

        global_pose_sub_ =
            this->create_subscription<sensor_msgs::msg::NavSatFix>(
                "/mavros/global_position/global", sensor_qos, [this](const sensor_msgs::msg::NavSatFix::UniquePtr msg)
                {
                lat = msg->latitude;
                lon = msg->longitude; });

        global_twist_sub_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "/mavros/global_position/local", sensor_qos, [this](const nav_msgs::msg::Odometry::UniquePtr msg)
                {
                velocity_x = msg->twist.twist.linear.x;
                velocity_y = msg->twist.twist.linear.y;
                velocity_z = msg->twist.twist.linear.z; });

        imu_data_sub_ =
            this->create_subscription<sensor_msgs::msg::Imu>(
                "/mavros/imu/data", sensor_qos, [this](const sensor_msgs::msg::Imu::UniquePtr msg)
                {
        
                tf2::Quaternion q_FLU_to_FRD = tf2::Quaternion(1, 0, 0, 0);
                tf2::Quaternion q_ENU_to_NED = tf2::Quaternion(0.70711, 0.70711, 0, 0);

                tf2::Quaternion q_FLU_to_ENU;
                tf2::convert(msg->orientation, q_FLU_to_ENU);

                tf2::Quaternion q_FRD_to_ENU = q_FLU_to_ENU * q_FLU_to_FRD.inverse();
                tf2::Quaternion q_FRD_to_NED = q_ENU_to_NED * q_FRD_to_ENU;

                tf2::Vector3 accel_FLU{msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
                
                accel_FRD = tf2::quatRotate(q_FLU_to_FRD, accel_FLU);
                
                accel_x = accel_FRD[0];
                accel_y = accel_FRD[1];
                accel_z = accel_FRD[2];

                tf2::getEulerYPR(q_FRD_to_NED, euler[2], euler[1], euler[0]);

                tf2::Vector3 omega_FLU{msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};

                omega_FRD = tf2::quatRotate(q_FLU_to_FRD, omega_FLU);

                omega[0] = omega_FRD[0]; 
                omega[1] = omega_FRD[1];
                omega[2] = omega_FRD[2]; });

        wind_state_publisher_ =
            this->create_publisher<soaring_interface::msg::WindState>("/wind_state", 10);

        wind_sub_ =
            this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
                "/mavros/wind_estimation", sensor_qos, [this](const geometry_msgs::msg::TwistWithCovarianceStamped::UniquePtr msg)
                { publish_wind_state(msg->twist.twist.linear.x, msg->twist.twist.linear.y); });

        airspd_flap_sub_ =
            this->create_subscription<soaring_interface::msg::AirspeedFlapsCommand>(
                "/airspeed_flaps_command", 10, [this](const soaring_interface::msg::AirspeedFlapsCommand::UniquePtr msg)
                {
                RCLCPP_INFO(rclcpp::get_logger("AUTOSOAR_COM"), "Received new airspeed setpoint from Autosoar");
                airspd_cmd = msg->v_ias;
                this->send_mav_command(178, 0.0f, airspd_cmd, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); });

        aircraft_config_sub_ =
            this->create_subscription<soaring_interface::msg::AircraftConfiguration>(
                "/aircraft_configuration", 10, [this](const soaring_interface::msg::AircraftConfiguration::UniquePtr msg)
                {
                if (msg->is_motor_enabled)
                {
                    RCLCPP_INFO(rclcpp::get_logger("AUTOSOAR_COM"), "Received Powered command from Autosoar");
                    this->set_mav_parameter("NAV_FW_GLIDE_EN", 0.0);
                }
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("AUTOSOAR_COM"), "Received Gliding command from Autosoar");
                    this->set_mav_parameter("NAV_FW_GLIDE_EN", 1.0);
                }
                motor_set = false; });

        rc_in_sub_ =
            this->create_subscription<mavros_msgs::msg::RCIn>(
                "/mavros/rc/in", 10, [this](const mavros_msgs::msg::RCIn::UniquePtr msg)
                {
                uint16_t system_rc_switch = msg->channels[14];
                uint16_t thermalling_rc_switch = msg->channels[6];

                if (system_rc_switch > 900 && system_rc_switch < 1300){
                    autosoar_state = AUTOSOAR_MODE_DISABLED;
                } else if (system_rc_switch >= 1300 && system_rc_switch < 1700){
                    autosoar_state = AUTOSOAR_MODE_SAFE;
                } else if (system_rc_switch >=1700 && system_rc_switch < 2200){
                    autosoar_state = AUTOSOAR_MODE_ACTIVE;
                } else {
                    autosoar_state = AUTOSOAR_MODE_DISABLED;
                }

                if (thermalling_rc_switch > 900 && thermalling_rc_switch < 1300){
                    thermalling_state = THERMALLING_DISABLED;
                } else if (thermalling_rc_switch > 1300){
                    thermalling_state = THERMALLING_ENABLED;
                }
                
                if (autosoar_state != previous_autosoar_state || thermalling_state != previous_thermalling_state){
                    publish_ground_command(autosoar_state, thermalling_state);
                    previous_autosoar_state = autosoar_state;
                    previous_thermalling_state = thermalling_state;
                } });

        waypoint_publisher_ =
            this->create_publisher<soaring_interface::msg::Waypoint>("/waypoint_information", 10);

        waypoint_sub_ =
            this->create_subscription<soaring_interface::msg::WaypointVector>(
                "/flight_plan", 10, [this](const soaring_interface::msg::WaypointVector::UniquePtr msg)
                {
                num_waypoints = (int) msg->n_waypoints;
                RCLCPP_INFO(rclcpp::get_logger("PX4_COM"), "Received new flight plan from Autosoar with %i waypoints", num_waypoints);
                flight_path_waypoints.clear();
                for (int i = 0; i < num_waypoints; i++){
                    WP point;
                    point.longitude = msg->waypoints[i].position.longitude;
                    point.latitude = msg->waypoints[i].position.latitude;
                    point.altitude = msg->waypoints[i].position.altitude;
                    point.orb_radius = msg->waypoints[i].radius_orbit;
                    flight_path_waypoints.push_back(point);
                    waypoint_publisher_->publish(msg->waypoints[i]);
                }
                Waypoints_set = false;
                current_waypoint = 0; });

        auto timer_callback = [this]() -> void
        {
            publish_aircraft_state();
        };

        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<soaring_interface::msg::AircraftState>::SharedPtr aircraft_state_publisher_;
    rclcpp::Subscription<mavros_msgs::msg::VfrHud>::SharedPtr vfr_hud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_twist_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr wind_sub_;
    rclcpp::Publisher<soaring_interface::msg::WindState>::SharedPtr wind_state_publisher_;
    rclcpp::Subscription<soaring_interface::msg::AirspeedFlapsCommand>::SharedPtr airspd_flap_sub_;
    rclcpp::Subscription<soaring_interface::msg::AircraftConfiguration>::SharedPtr aircraft_config_sub_;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_in_sub_;
    rclcpp::Publisher<soaring_interface::msg::GroundControlCommand>::SharedPtr ground_command_publisher_;
    rclcpp::Subscription<soaring_interface::msg::WaypointVector>::SharedPtr waypoint_sub_;
    rclcpp::Publisher<soaring_interface::msg::Waypoint>::SharedPtr waypoint_publisher_;

    void publish_aircraft_state() const;
    void publish_wind_state(float wind_north, float wind_east) const;
    void set_mav_parameter(const char *param_id, double param_value);
    void send_mav_command(uint16_t command_id, float param1, float param2, float param3,
                          float param4, float param5, float param6, float param7);
    void publish_ground_command(uint8_t system_state, uint8_t thermalling_state) const;

    float i_airspeed, lat, lon, alt, throttle, velocity_x, velocity_y, velocity_z;
    float accel_x, accel_y, accel_z, omega_x, omega_y, omega_z, airspd_cmd;
    double euler[3], omega[3];
    tf2::Vector3 omega_FRD, accel_FRD;
    bool motor_set;
    int num_waypoints;
    int current_waypoint;
    struct WP
    {
        float longitude;
        float latitude;
        float altitude;
        float orb_radius = 0.0;
    };

    std::vector<WP> flight_path_waypoints;
    bool Waypoints_set = true;

    enum AUTOSOAR_MODE
    {
        AUTOSOAR_MODE_DISABLED,
        AUTOSOAR_MODE_SAFE,
        AUTOSOAR_MODE_ACTIVE
    } autosoar_state{AUTOSOAR_MODE_DISABLED};

    enum THERMALLING_MODE
    {
        THERMALLING_DISABLED,
        THERMALLING_ENABLED
    } thermalling_state{THERMALLING_DISABLED};

    uint8_t previous_autosoar_state = -1;
    uint8_t previous_thermalling_state = -1;
};

void AUTOSOAR_COM::publish_aircraft_state() const
{
    soaring_interface::msg::AircraftState msg{};

    msg.header.stamp = this->now();

    msg.v_ias = this->i_airspeed;
    msg.euler.x = this->euler[0];
    msg.euler.y = this->euler[1];
    msg.euler.z = this->euler[2];
    msg.omega.x = this->omega[0];
    msg.omega.y = this->omega[1];
    msg.omega.z = this->omega[2];
    msg.v_gps.x = this->velocity_x;
    msg.v_gps.y = this->velocity_y;
    msg.v_gps.z = this->velocity_z;
    msg.body_accel.x = this->accel_x;
    msg.body_accel.y = this->accel_y;
    msg.body_accel.z = this->accel_z;
    msg.position.latitude = this->lat;
    msg.position.longitude = this->lon;
    msg.position.altitude = this->alt;
    msg.throttle = this->throttle;

    aircraft_state_publisher_->publish(msg);
}

void AUTOSOAR_COM::publish_ground_command(uint8_t system_state, uint8_t thermalling_state) const
{
    soaring_interface::msg::GroundControlCommand gc_msg{};

    switch (system_state)
    {
    case AUTOSOAR_MODE_DISABLED:
    {
        gc_msg.system_state = -1;
        break;
    }
    case AUTOSOAR_MODE_SAFE:
    {
        gc_msg.system_state = 0;
        break;
    }
    case AUTOSOAR_MODE_ACTIVE:
    {
        gc_msg.system_state = 1;
        break;
    }
    }

    switch (thermalling_state)
    {
    case AUTOSOAR_MODE_DISABLED:
    {
        gc_msg.latch_enable = 0;
        break;
    }
    case AUTOSOAR_MODE_SAFE:
    {
        gc_msg.latch_enable = 1;
        break;
    }
    }

    ground_command_publisher_->publish(gc_msg);
}

void AUTOSOAR_COM::publish_wind_state(float wind_north, float wind_east) const
{
    soaring_interface::msg::WindState wind_msg{};

    wind_msg.wind_south = wind_north;
    wind_msg.wind_west = wind_east;

    wind_state_publisher_->publish(wind_msg);
}

void AUTOSOAR_COM::set_mav_parameter(const char *param_id, double param_value)
{
    mavros_msgs::srv::ParamSetV2::Response::SharedPtr res;
    auto client = this->create_client<mavros_msgs::srv::ParamSetV2>("/mavros/param/set");

    auto cmdrq = std::make_shared<mavros_msgs::srv::ParamSetV2::Request>();
    cmdrq->param_id = param_id;
    cmdrq->value.type = 3;
    cmdrq->value.double_value = param_value;

    auto future = client->async_send_request(cmdrq);
    const auto future_status = future.wait_for(1s);
    if (future_status == std::future_status::ready)
    {
        auto response = future.get();
        res->success = response->success;
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Set param service not ready");
    }
}

void AUTOSOAR_COM::send_mav_command(uint16_t command_id, float param1 = 0.0, float param2 = 0.0,
                                    float param3 = 0.0, float param4 = 0.0, float param5 = 0.0,
                                    float param6 = 0.0, float param7 = 0.0)
{
    mavros_msgs::srv::CommandLong::Response::SharedPtr res;
    auto client = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");

    auto cmdrq = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    cmdrq->command = command_id;
    cmdrq->param1 = param1;
    cmdrq->param2 = param2;
    cmdrq->param3 = param3;
    cmdrq->param4 = param4;
    cmdrq->param5 = param5;
    cmdrq->param6 = param6;
    cmdrq->param7 = param7;

    auto future = client->async_send_request(cmdrq);
    const auto future_status = future.wait_for(1s);
    if (future_status == std::future_status::ready)
    {
        auto response = future.get();
        res->success = response->success;
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Mav command service not ready");
    }
}

int main(int argc, char *argv[])
{
    std::cout << "Starting autosoar communication node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AUTOSOAR_COM>());

    rclcpp::shutdown();
    return 0;
}
