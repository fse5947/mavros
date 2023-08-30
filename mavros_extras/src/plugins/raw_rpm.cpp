
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/raw_rpm.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief RAW RPM plugin.
 * @plugin raw_rpm
 */
class RawRPMPlugin : public plugin::Plugin
{
public:
  explicit RawRPMPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "raw_rpm")
  {
    auto sensor_qos = rclcpp::SensorDataQoS();
    
    rpm_pub = node->create_publisher<mavros_msgs::msg::RawRPM>("raw_rpm", sensor_qos);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&RawRPMPlugin::handle_raw_rpm),
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::RawRPM>::SharedPtr rpm_pub;

  void handle_raw_rpm(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::RAW_RPM & raw_rpm,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto vmsg = mavros_msgs::msg::RawRPM();

    vmsg.header.stamp = node->now();
    vmsg.frequency = raw_rpm.frequency;

    rpm_pub->publish(vmsg);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::RawRPMPlugin)