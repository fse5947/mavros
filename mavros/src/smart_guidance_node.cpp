#include "smart_guidance_node.hpp"

SmartGuidanceCom::SmartGuidanceCom() : Node("smart_guidance_node")
{
    auto sensor_qos = rclcpp::SensorDataQoS();
    auto wp_qos = rclcpp::QoS(10).transient_local();

    this->declare_parameter("aircraft_state_publish_rate", 100);
    aircraft_state_publish_rate_ = this->get_parameter("aircraft_state_publish_rate").get_parameter_value().get<int>();

    aircraft_state_publisher_ =
        this->create_publisher<soaring_interface::msg::AircraftState>("/aircraft_state", 10);

    wind_state_publisher_ =
        this->create_publisher<soaring_interface::msg::WindState>("/wind_state", 10);

    vfr_hud_sub_ =
        this->create_subscription<mavros_msgs::msg::VfrHud>(
            "/mavros/vfr_hud", sensor_qos, [this](const mavros_msgs::msg::VfrHud::SharedPtr msg)
            { SmartGuidanceCom::VfrHudCallback(msg); });

    global_pose_sub_ =
        this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global", sensor_qos, [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg)
            { SmartGuidanceCom::GlobalPoseCallback(msg); });

    global_twist_sub_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "/mavros/global_position/local", sensor_qos, [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            { SmartGuidanceCom::GlobalTwistCallback(msg); });

    imu_data_sub_ =
        this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", sensor_qos, [this](const sensor_msgs::msg::Imu::SharedPtr msg)
            { SmartGuidanceCom::ImuDataCallback(msg); });

    wind_sub_ =
        this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/mavros/wind_estimation", sensor_qos, [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
            { SmartGuidanceCom::WindCallback(msg); });

    airspd_flap_sub_ =
        this->create_subscription<soaring_interface::msg::AirspeedFlapsCommand>(
            "/airspeed_flaps_command", 10, [this](const soaring_interface::msg::AirspeedFlapsCommand::SharedPtr msg)
            { SmartGuidanceCom::AirspdFlapCallback(msg); });

    aircraft_config_sub_ =
        this->create_subscription<soaring_interface::msg::AircraftConfiguration>(
            "/aircraft_configuration", 10, [this](const soaring_interface::msg::AircraftConfiguration::SharedPtr msg)
            { SmartGuidanceCom::AircraftConfigCallback(msg); });

    rc_in_sub_ =
        this->create_subscription<mavros_msgs::msg::RCIn>(
            "/mavros/rc/in", 10, [this](const mavros_msgs::msg::RCIn::SharedPtr msg)
            { SmartGuidanceCom::RcInCallback(msg); });

    waypoint_reached_sub_ = this->create_subscription<mavros_msgs::msg::WaypointReached>(
        "/mavros/mission/reached", wp_qos, [this](const mavros_msgs::msg::WaypointReached::SharedPtr msg)
        { SmartGuidanceCom::WaypointReachedCallback(msg); });

    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    flight_plan_service_ = this->create_service<soaring_interface::srv::UploadFlightPlan>(
        "/flight_plan",
        [this](const std::shared_ptr<soaring_interface::srv::UploadFlightPlan::Request> request,
               std::shared_ptr<soaring_interface::srv::UploadFlightPlan::Response> response)
        { SmartGuidanceCom::FlightPlanCallback(request, response); },
        rmw_qos_profile_services_default, service_cb_group_);

    ground_control_client_ = this->create_client<soaring_interface::srv::GroundControlCommand>("/ground_command", 
                                                                                                rmw_qos_profile_services_default, service_cb_group_);

    set_mav_param_client_ = this->create_client<mavros_msgs::srv::ParamSetV2>("/mavros/param/set",
                                                                              rmw_qos_profile_services_default, service_cb_group_);
    send_mav_command_client_ = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command",
                                                                                  rmw_qos_profile_services_default, service_cb_group_);
    push_mav_waypoints_client_ = this->create_client<mavros_msgs::srv::WaypointPush>("/mavros/mission/push",
                                                                                     rmw_qos_profile_services_default, service_cb_group_);

    timer_cb_group_ = nullptr;

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(aircraft_state_publish_rate_), [this]()
        { SmartGuidanceCom::TimerCallback(); },
        timer_cb_group_);
}

void SmartGuidanceCom::PublishAircraftState() const
{
    soaring_interface::msg::AircraftState msg{};

    msg.header.stamp = this->now();

    msg.v_ias = this->indicated_airspeed_;
    msg.euler.x = this->euler_[0];
    msg.euler.y = this->euler_[1];
    msg.euler.z = this->euler_[2];
    msg.omega.x = this->omega_[0];
    msg.omega.y = this->omega_[1];
    msg.omega.z = this->omega_[2];
    msg.v_gps.x = this->velocity_x_;
    msg.v_gps.y = this->velocity_y_;
    msg.v_gps.z = this->velocity_z_;
    msg.body_accel.x = this->accel_x_;
    msg.body_accel.y = this->accel_y_;
    msg.body_accel.z = this->accel_z_;
    msg.position.latitude = this->lat_;
    msg.position.longitude = this->lon_;
    msg.position.altitude = this->alt_;
    msg.throttle = this->throttle_;

    aircraft_state_publisher_->publish(msg);
}


void SmartGuidanceCom::PublishWindState(float wind_north, float wind_east) const
{
    soaring_interface::msg::WindState wind_msg{};

    wind_msg.wind_south = wind_north;
    wind_msg.wind_west = wind_east;

    wind_state_publisher_->publish(wind_msg);
}

void SmartGuidanceCom::VfrHudCallback(const mavros_msgs::msg::VfrHud::SharedPtr msg)
{
    indicated_airspeed_ = msg->airspeed;
    throttle_ = msg->throttle;
    alt_ = msg->altitude;
}

void SmartGuidanceCom::GlobalPoseCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    lat_ = msg->latitude;
    lon_ = msg->longitude;
}

void SmartGuidanceCom::GlobalTwistCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    velocity_x_ = msg->twist.twist.linear.x;
    velocity_y_ = msg->twist.twist.linear.y;
    velocity_z_ = msg->twist.twist.linear.z;
}

void SmartGuidanceCom::ImuDataCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion q_FLU_to_ENU;
    tf2::convert(msg->orientation, q_FLU_to_ENU);

    tf2::Quaternion q_FRD_to_ENU = q_FLU_to_ENU * q_FLU_to_FRD.inverse();
    tf2::Quaternion q_FRD_to_NED = q_ENU_to_NED * q_FRD_to_ENU;

    tf2::Vector3 accel_FLU{msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};

    accel_FRD_ = tf2::quatRotate(q_FLU_to_FRD, accel_FLU);

    accel_x_ = accel_FRD_[0];
    accel_y_ = accel_FRD_[1];
    accel_z_ = accel_FRD_[2];

    tf2::getEulerYPR(q_FRD_to_NED, euler_[2], euler_[1], euler_[0]);

    tf2::Vector3 omega_FLU{msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};

    omega_FRD_ = tf2::quatRotate(q_FLU_to_FRD, omega_FLU);

    omega_[0] = omega_FRD_[0];
    omega_[1] = omega_FRD_[1];
    omega_[2] = omega_FRD_[2];
}

void SmartGuidanceCom::WindCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
    PublishWindState(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
}

void SmartGuidanceCom::AirspdFlapCallback(const soaring_interface::msg::AirspeedFlapsCommand::SharedPtr msg)
{
    RCLCPP_INFO(node_logger_, "Received new airspeed setpoint from Smart Guidance");
    airspd_cmd_ = msg->v_ias;
    this->SendMavCommand(mavros_msgs::msg::CommandCode::DO_CHANGE_SPEED, 0.0f, airspd_cmd_);
}

void SmartGuidanceCom::AircraftConfigCallback(const soaring_interface::msg::AircraftConfiguration::SharedPtr msg)
{
    if (msg->is_motor_enabled)
    {
        RCLCPP_INFO(node_logger_, "Received Powered command from Smart Guidance");
        this->SetMavParameter("AA_GLIDE_EN", 0, 2);
    }
    else
    {
        RCLCPP_INFO(node_logger_, "Received Gliding command from Smart Guidance");
        this->SetMavParameter("AA_GLIDE_EN", 1, 2);
    }
}

void SmartGuidanceCom::RcInCallback(const mavros_msgs::msg::RCIn::SharedPtr msg)
{
    uint16_t system_rc_switch = msg->channels[14];
    uint16_t thermalling_rc_switch = msg->channels[6];

    smart_guidance_state = get_SmartGuidanceState(system_rc_switch);
    thermalling_state = get_ThermallingState(thermalling_rc_switch);

    if (smart_guidance_state != previous_smart_guidance_state_ || thermalling_state != previous_thermalling_state_)
    {
        if (smart_guidance_state == kSmartGuidanceModeSafe || smart_guidance_state == kSmartGuidanceModeActive)
        {
            // Allows a mission to be uploaded without defining a landing waypoint
            this->SetMavParameter("RTL_TYPE", 0, 2);
            // Change px4 mode to auto (mission)
            this->SendMavCommand(mavros_msgs::msg::CommandCode::DO_SET_MODE, 4.0f);
        }
        else if (smart_guidance_state == kSmartGuidanceModeDisabled)
        {
            this->SendMavCommand(mavros_msgs::msg::CommandCode::NAV_RETURN_TO_LAUNCH);
        }
        this->SendGroundCommand(smart_guidance_state, thermalling_state);
        previous_smart_guidance_state_ = smart_guidance_state;
        previous_thermalling_state_ = thermalling_state;
    }
}

void SmartGuidanceCom::WaypointReachedCallback(const mavros_msgs::msg::WaypointReached::SharedPtr msg)
{
    RCLCPP_INFO(node_logger_, "Reached waypoint %i", msg->wp_seq);
}

void SmartGuidanceCom::FlightPlanCallback(const std::shared_ptr<soaring_interface::srv::UploadFlightPlan::Request> request,
                                          std::shared_ptr<soaring_interface::srv::UploadFlightPlan::Response> response)
{
    num_waypoints_ = (int)request->flight_plan.n_waypoints;
    RCLCPP_INFO(node_logger_, "Received new flight plan from Smart Guidance with %i waypoints",
                num_waypoints_);
    flight_path_waypoints_.clear();

    for (int i = 0; i < num_waypoints_; i++)
    {
        mavros_msgs::msg::Waypoint waypoint;

        waypoint.command = mavros_msgs::msg::CommandCode::NAV_WAYPOINT;
        waypoint.autocontinue = 1;
        waypoint.x_lat = request->flight_plan.waypoints[i].position.latitude;
        waypoint.y_long = request->flight_plan.waypoints[i].position.longitude;
        waypoint.z_alt = request->flight_plan.waypoints[i].position.altitude;

        RCLCPP_INFO(node_logger_, "Waypoint %i: Lat: %f, Lon: %f, Alt: %f",
                    i, waypoint.x_lat, waypoint.y_long, waypoint.z_alt);

        flight_path_waypoints_.push_back(waypoint);
    }

    // Make sure the first waypoint is the current waypoint of the mission, and the last waypoint is a loiter waypoint.
    flight_path_waypoints_[0].is_current = 1;
    flight_path_waypoints_[num_waypoints_ - 1].command = mavros_msgs::msg::CommandCode::NAV_LOITER_UNLIM;
    flight_path_waypoints_[num_waypoints_ - 1].param3 = request->flight_plan.waypoints[num_waypoints_ - 1].radius_orbit;

    response->result = true;
    this->PushMavWaypoints(flight_path_waypoints_);
}

void SmartGuidanceCom::TimerCallback()
{
    PublishAircraftState();
}

void SmartGuidanceCom::SendGroundCommand(uint8_t system_state, uint8_t thermalling_state)
{
    auto cmdrq = std::make_shared<soaring_interface::srv::GroundControlCommand_Request>();

    switch (system_state)
    {
    case kSmartGuidanceModeDisabled:
    {
        cmdrq->system_state = kSystemDisabled;
        break;
    }
    case kSmartGuidanceModeSafe:
    {
        cmdrq->system_state = kSystemSafe;
        break;
    }
    case kSmartGuidanceModeActive:
    {
        cmdrq->system_state = kSystemEnabled;
        break;
    }
    }

    switch (thermalling_state)
    {
    case kThermallingDisabled:
    {
        cmdrq->latch_enable = kThermallingDisabled;
        break;
    }
    case kThermallingEnabled:
    {
        cmdrq->latch_enable = kThermallingEnabled;
        break;
    }
    }

    HandleClientRequest(ground_control_client_, cmdrq, "Ground Control Command");
}

void SmartGuidanceCom::SetMavParameter(const char *param_id, uint8_t param_value, uint8_t param_type)
{
    auto cmdrq = std::make_shared<mavros_msgs::srv::ParamSetV2::Request>();
    cmdrq->param_id = param_id;
    cmdrq->value.type = param_type;
    cmdrq->value.integer_value = param_value;
    cmdrq->value.double_value = double(param_value);

    HandleClientRequest(set_mav_param_client_, cmdrq, "Mav Set Param");
}

void SmartGuidanceCom::SendMavCommand(uint16_t command_id, float param1 = 0.0, float param2 = 0.0,
                                      float param3 = 0.0, float param4 = 0.0, float param5 = 0.0,
                                      float param6 = 0.0, float param7 = 0.0)
{
    auto cmdrq = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    cmdrq->command = command_id;
    cmdrq->param1 = param1;
    cmdrq->param2 = param2;
    cmdrq->param3 = param3;
    cmdrq->param4 = param4;
    cmdrq->param5 = param5;
    cmdrq->param6 = param6;
    cmdrq->param7 = param7;

    HandleClientRequest(send_mav_command_client_, cmdrq, "Mav Command");
}

void SmartGuidanceCom::SendMavCommand(u_int16_t command_id)
{
    SendMavCommand(command_id, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void SmartGuidanceCom::SendMavCommand(u_int16_t command_id, float param1)
{
    SendMavCommand(command_id, param1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void SmartGuidanceCom::SendMavCommand(u_int16_t command_id, float param1, float param2)
{
    SendMavCommand(command_id, param1, param2, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void SmartGuidanceCom::PushMavWaypoints(std::vector<mavros_msgs::msg::Waypoint> flight_path)
{
    auto cmdrq = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();

    cmdrq->waypoints = flight_path;

    HandleClientRequest(push_mav_waypoints_client_, cmdrq, "Push Waypoints to Mav");
}

template <typename T1, typename T2>
void SmartGuidanceCom::HandleClientRequest(T1 client_ptr, T2 request, std::string service_name)
{
    while (!client_ptr->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node_logger_, "Process Interrupted.");
            return;
        }
        RCLCPP_INFO(node_logger_, "Service not Available");
    }

    auto result_future = client_ptr->async_send_request(request);

    if (result_future.wait_for(std::chrono::seconds(1)) == std::future_status::ready)
    {
        auto response = result_future.get();
        if (response->success)
        {
            RCLCPP_INFO(node_logger_, service_name + " Response Successful");
        }
        else
        {
            RCLCPP_INFO(node_logger_, service_name + " Service Request Failed");
        }
    }
    else
    {
        RCLCPP_INFO(node_logger_, service_name + " Response not Ready");
    }
}

SmartGuidanceMode SmartGuidanceCom::get_SmartGuidanceState(uint16_t rc_switch)
{
    if (rc_switch >= kPwmLowMin && rc_switch < kPwmLowMax)
    {
        return kSmartGuidanceModeDisabled;
    }
    else if (rc_switch >= kPwmLowMax && rc_switch < kPwmHighMin)
    {
        return kSmartGuidanceModeSafe;
    }
    else if (rc_switch >= kPwmHighMin && rc_switch < kPwmHighMax)
    {
        return kSmartGuidanceModeActive;
    }
    else
    {
        return kSmartGuidanceModeDisabled;
    }
}

ThermallingMode SmartGuidanceCom::get_ThermallingState(uint16_t rc_switch)
{
    if (rc_switch > kPwmLowMin && rc_switch < kPwmLowMax)
    {
        return kThermallingDisabled;
    }
    else if (rc_switch > kPwmLowMax)
    {
        return kThermallingEnabled;
    }
    else
    {
        return kThermallingDisabled;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto smart_guidance_node = std::make_shared<SmartGuidanceCom>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(smart_guidance_node);

    RCLCPP_INFO(rclcpp::get_logger("smart_guidance_node"), "Starting Smart Guidance communication node...");
    executor.spin();
    RCLCPP_INFO(rclcpp::get_logger("smart_guidance_node"), "Keyboard interrupt, shutting down...");
    rclcpp::shutdown();
    return 0;
}
