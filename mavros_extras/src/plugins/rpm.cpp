
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/rpm.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief RPM plugin.
 * @plugin rpm
 */
class RPMPlugin : public plugin::Plugin
{
public:
  explicit RPMPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "rpm")
  {
    auto sensor_qos = rclcpp::SensorDataQoS();
    
    rpm_pub = node->create_publisher<mavros_msgs::msg::RPM>("rpm", sensor_qos);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&RPMPlugin::handle_rpm),
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::RPM>::SharedPtr rpm_pub;

  void handle_rpm(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::ardupilotmega::msg::RPM & rpm,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto vmsg = mavros_msgs::msg::RPM();

    vmsg.header.stamp = node->now();
    vmsg.rpm1 = rpm.rpm1;
    vmsg.rpm2 = rpm.rpm2;

    rpm_pub->publish(vmsg);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::RPMPlugin)