#include "rclcpp/rclcpp.hpp"
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

#include <chrono>

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

        vfr_hud_sub_ =
            this->create_subscription<mavros_msgs::msg::VfrHud>("/mavros/vfr_hud", sensor_qos,
                                                                [this](const mavros_msgs::msg::VfrHud::SharedPtr msg)
                                                                {
                                                                    i_airspeed = msg->airspeed;
                                                                    throttle = msg->throttle;
                                                                    alt = msg->altitude;
                                                                });

        global_pose_sub_ =
            this->create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global", sensor_qos,
                                                                   [this](const sensor_msgs::msg::NavSatFix::UniquePtr msg)
                                                                   {
                                                                       lat = msg->latitude;
                                                                       lon = msg->longitude;
                                                                   });

        global_twist_sub_ =
            this->create_subscription<nav_msgs::msg::Odometry>("/mavros/global_position/local", sensor_qos,
                                                               [this](const nav_msgs::msg::Odometry::UniquePtr msg)
                                                               {
                                                                   velocity_x = msg->twist.twist.linear.x;
                                                                   velocity_y = msg->twist.twist.linear.y;
                                                                   velocity_z = msg->twist.twist.linear.z;
                                                               });

        imu_data_sub_ =
            this->create_subscription<sensor_msgs::msg::Imu>("/mavros/imu/data", sensor_qos,
                                                             [this](const sensor_msgs::msg::Imu::UniquePtr msg)
                                                             {
                                                                 accel_x = msg->linear_acceleration.x;
                                                                 accel_y = msg->linear_acceleration.y;
                                                                 accel_z = msg->linear_acceleration.z;
                                                                 omega_x = msg->angular_velocity.x;
                                                                 omega_y = msg->angular_velocity.y;
                                                                 omega_z = msg->angular_velocity.z;
                                                             });

        // FIX ORIENTATION FRAME
        local_pose_sub_ =
            this->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", sensor_qos,
                                                                       [this](const geometry_msgs::msg::PoseStamped::UniquePtr msg)
                                                                       {
                                                                           Eigen::Quaterniond quat(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
                                                                                                   msg->pose.orientation.z);

                                                                           Eigen::Quaterniond quat_ned = mavros::ftf::detail::transform_orientation(quat, mavros::ftf::StaticTF::ENU_TO_NED);

                                                                           Eigen::Matrix3d dcm = quat_ned.toRotationMatrix();

                                                                           euler[1] = asin(-dcm(2, 0));

                                                                           if ((fabs(euler[1] - M_PI / 2)) < 1.0e-3)
                                                                           {
                                                                               euler[0] = 0;
                                                                               euler[2] = atan2(dcm(1, 2), dcm(0, 2));
                                                                           }
                                                                           else if ((fabs(euler[1] + M_PI / 2)) < 1.0e-3)
                                                                           {
                                                                               euler[0] = 0;
                                                                               euler[2] = atan2(-dcm(1, 2), -dcm(0, 2));
                                                                           }
                                                                           else
                                                                           {
                                                                               euler[0] = atan2(dcm(2, 1), dcm(2, 2));
                                                                               euler[2] = atan2(dcm(1, 0), dcm(0, 0));
                                                                           }
                                                                       });

        wind_sub_ =
            this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/mavros/wind_estimation", sensor_qos,
                                                                                      [this](const geometry_msgs::msg::TwistWithCovarianceStamped::UniquePtr msg)
                                                                                      {
                                                                                          publish_wind_state(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
                                                                                      });
        auto timer_callback = [this]() -> void
        {
            publish_aircraft_state();
        };

        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
    rclcpp::Publisher<soaring_interface::msg::AircraftState>::SharedPtr aircraft_state_publisher_;
    rclcpp::Subscription<mavros_msgs::msg::VfrHud>::SharedPtr vfr_hud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_twist_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr wind_sub_;
    rclcpp::Publisher<soaring_interface::msg::WindState>::SharedPtr wind_state_publisher_;

    void publish_aircraft_state() const;
    void publish_wind_state(float wind_north, float wind_east) const;

    float i_airspeed, lat, lon, alt, throttle, velocity_x, velocity_y, velocity_z;
    float accel_x, accel_y, accel_z, omega_x, omega_y, omega_z;
    float euler[3];
};

void AUTOSOAR_COM::publish_aircraft_state() const
{
    soaring_interface::msg::AircraftState msg{};

    msg.v_ias = this->i_airspeed;
    msg.euler.x = this->euler[0];
    msg.euler.y = this->euler[1];
    msg.euler.z = this->euler[2];
    msg.omega.x = this->omega_x;
    msg.omega.y = this->omega_y;
    msg.omega.z = this->omega_z;
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

int main(int argc, char *argv[])
{
    std::cout << "Starting autosoar communication node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AUTOSOAR_COM>());

    rclcpp::shutdown();
    return 0;
}
