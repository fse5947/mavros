#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/vfr_hud.hpp"
#include <soaring_interface/msg/aircraft_state.hpp>

#include <chrono>

using namespace std::chrono;
using namespace std::chrono_literals;

class AUTOSOAR_COM : public rclcpp::Node {
public:
	AUTOSOAR_COM() : Node("autosoar_node") {

        auto sensor_qos = rclcpp::SensorDataQoS();

        aircraft_state_publisher_ =
			this->create_publisher<soaring_interface::msg::AircraftState>("/aircraft_state", 10);

        local_pose_sub_ =
			this->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", sensor_qos,
				[this](const geometry_msgs::msg::PoseStamped::UniquePtr msg) {
					std::cout << msg->pose.position.x << std::endl;
				});

        vfr_hud_sub_ = 
            this->create_subscription<mavros_msgs::msg::VfrHud>("/mavros/vfr_hud", sensor_qos,
				[this](const mavros_msgs::msg::VfrHud::SharedPtr msg) {
					i_airspeed = msg->airspeed;
                    throttle = msg->throttle;
				});

        auto timer_callback = [this]() -> void {

            publish_aircraft_state();
        };

        timer_ = this->create_wall_timer(100ms, timer_callback);

    }
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
    rclcpp::Publisher<soaring_interface::msg::AircraftState>::SharedPtr aircraft_state_publisher_;
    rclcpp::Subscription<mavros_msgs::msg::VfrHud>::SharedPtr vfr_hud_sub_;

   	void publish_aircraft_state() const;
    float i_airspeed, throttle;

};

void AUTOSOAR_COM::publish_aircraft_state() const {
	soaring_interface::msg::AircraftState msg{};

	msg.v_ias = this->i_airspeed;
	// msg.euler.x = this->euler[0];
	// msg.euler.y = this->euler[1];
	// msg.euler.z = this->euler[2];
	// msg.omega.x = this->omega[0];
	// msg.omega.y = this->omega[1];
	// msg.omega.z = this->omega[2];
	// msg.v_gps.x = this->vel[0];
	// msg.v_gps.y = this->vel[1];
	// msg.v_gps.z = this->vel[2];
	// msg.body_accel.x = this->accel[0];
	// msg.body_accel.y = this->accel[1];
	// msg.body_accel.z = this->accel[2];
	// msg.position.latitude = this->lat;
	// msg.position.longitude = this->lon;
	// msg.position.altitude = this->alt;
	msg.throttle = this->throttle;

	aircraft_state_publisher_->publish(msg);
}

int main(int argc, char* argv[]) {
	std::cout << "Starting autosoar communication node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<AUTOSOAR_COM>());

	rclcpp::shutdown();
	return 0;
}
