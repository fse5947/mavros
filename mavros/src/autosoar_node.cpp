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
// #include "mavros_msgs/msg/com"

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
        // auto event_qos = rclcpp::ParameterEventsQoS();
        // auto qos = rclcpp::ParametersQoS().get_rmw_qos_profile();

        aircraft_state_publisher_ =
            this->create_publisher<soaring_interface::msg::AircraftState>("/aircraft_state", 10);

        // param_srv_ = this->create_client<mavros_msgs::srv::ParamSetV2>("/mavros/param/set");

        // param_srv_->async_send_request(param_srv_request_);

        vfr_hud_sub_ =
            this->create_subscription<mavros_msgs::msg::VfrHud>(
                "/mavros/vfr_hud", sensor_qos, [this](const mavros_msgs::msg::VfrHud::SharedPtr msg)
                {
                i_airspeed = msg->airspeed;
                throttle = msg->throttle;
                alt = msg->altitude; });

        global_pose_sub_ =
            this->create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global", sensor_qos, [this](const sensor_msgs::msg::NavSatFix::UniquePtr msg)
                                                                   {
                lat = msg->latitude;
                lon = msg->longitude; });

        global_twist_sub_ =
            this->create_subscription<nav_msgs::msg::Odometry>("/mavros/global_position/local", sensor_qos, [this](const nav_msgs::msg::Odometry::UniquePtr msg)
                                                               {
                velocity_x = msg->twist.twist.linear.x;
                velocity_y = msg->twist.twist.linear.y;
                velocity_z = msg->twist.twist.linear.z; });

        imu_data_sub_ =
            this->create_subscription<sensor_msgs::msg::Imu>("/mavros/imu/data", sensor_qos, [this](const sensor_msgs::msg::Imu::UniquePtr msg)
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
            this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/mavros/wind_estimation", sensor_qos, [this](const geometry_msgs::msg::TwistWithCovarianceStamped::UniquePtr msg)
                                                                                      { publish_wind_state(msg->twist.twist.linear.x, msg->twist.twist.linear.y); });

        airspd_flap_sub_ =
            this->create_subscription<soaring_interface::msg::AirspeedFlapsCommand>("/airspeed_flaps_command", 10, [this](const soaring_interface::msg::AirspeedFlapsCommand::UniquePtr msg)
                                                                                    {
            RCLCPP_INFO(rclcpp::get_logger("PX4_COM"), "Received new airspeed setpoint from Autosoar");
            airspd_cmd = (double)msg->v_ias; 
            param_set_succ = this->set_parameter("FW_AIRSPD_TRIM", airspd_cmd);
            std::cout << param_set_succ << std::endl; });

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
    rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedPtr param_srv_;

    void publish_aircraft_state() const;
    void publish_wind_state(float wind_north, float wind_east) const;
    bool set_parameter(const char *param_id, double param_value);

    float i_airspeed, lat, lon, alt, throttle, velocity_x, velocity_y, velocity_z;
    float accel_x, accel_y, accel_z, omega_x, omega_y, omega_z;
    double euler[3], omega[3], airspd_cmd;
    bool param_set_succ;
    tf2::Vector3 omega_FRD, accel_FRD;
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

void AUTOSOAR_COM::publish_wind_state(float wind_north, float wind_east) const
{
    soaring_interface::msg::WindState wind_msg{};

    wind_msg.wind_south = wind_north;
    wind_msg.wind_west = wind_east;

    wind_state_publisher_->publish(wind_msg);
}

bool AUTOSOAR_COM::set_parameter(const char *param_id, double param_value)
{
    auto lg = get_logger();
    bool set_param_result = false;
    auto param_client_ = this->create_client<mavros_msgs::srv::ParamSetV2>("/mavros/param/set");

    auto param_request = std::make_shared<mavros_msgs::srv::ParamSetV2::Request>();
    param_request->param_id = param_id;
    param_request->value.type = 3;
    param_request->value.double_value = param_value;

    auto set_param_future = param_client_->async_send_request(param_request);
    const auto future_status = set_param_future.wait_for(1s);

    auto response = set_param_future.get();
    set_param_result = response->success;
    // if (future_status == std::future_status::ready)
    // {
    //     auto response = set_param_future.get();
    //     set_param_result = response->success;
    // }

    return set_param_result;
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
