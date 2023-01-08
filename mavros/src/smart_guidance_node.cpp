#include "smart_guidance_node.hpp"

SMART_GUIDANCE_COM::SMART_GUIDANCE_COM() : Node("smart_guidance_node")
{
    auto sensor_qos = rclcpp::SensorDataQoS();
    auto wp_qos = rclcpp::QoS(10).transient_local();

    aircraft_state_publisher_ =
        this->create_publisher<soaring_interface::msg::AircraftState>("/aircraft_state", 10);

    ground_command_publisher_ =
        this->create_publisher<soaring_interface::msg::GroundControlCommand>("/ground_command", 10);

    wind_state_publisher_ =
        this->create_publisher<soaring_interface::msg::WindState>("/wind_state", 10);

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

    wind_sub_ =
        this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/mavros/wind_estimation", sensor_qos, [this](const geometry_msgs::msg::TwistWithCovarianceStamped::UniquePtr msg)
            { publish_wind_state(msg->twist.twist.linear.x, msg->twist.twist.linear.y); });

    airspd_flap_sub_ =
        this->create_subscription<soaring_interface::msg::AirspeedFlapsCommand>(
            "/airspeed_flaps_command", 10, [this](const soaring_interface::msg::AirspeedFlapsCommand::UniquePtr msg)
            {
                RCLCPP_INFO(rclcpp::get_logger("SMART_GUIDANCE_COM"), "Received new airspeed setpoint from Smart Guidance");
                airspd_cmd = msg->v_ias;
                this->send_mav_command(mavros_msgs::msg::CommandCode::DO_CHANGE_SPEED, 0.0f, airspd_cmd, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); });

    aircraft_config_sub_ =
        this->create_subscription<soaring_interface::msg::AircraftConfiguration>(
            "/aircraft_configuration", 10, [this](const soaring_interface::msg::AircraftConfiguration::UniquePtr msg)
            {
                if (msg->is_motor_enabled)
                {
                    RCLCPP_INFO(rclcpp::get_logger("SMART_GUIDANCE_COM"), "Received Powered command from Smart Guidance");
                    this->set_mav_parameter("NAV_FW_GLIDE_EN", 0, 2);
                }
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("SMART_GUIDANCE_COM"), "Received Gliding command from Smart Guidance");
                    this->set_mav_parameter("NAV_FW_GLIDE_EN", 1, 2);
                } });

    rc_in_sub_ =
        this->create_subscription<mavros_msgs::msg::RCIn>(
            "/mavros/rc/in", 10, [this](const mavros_msgs::msg::RCIn::UniquePtr msg)
            {
                uint16_t system_rc_switch = msg->channels[14];
                uint16_t thermalling_rc_switch = msg->channels[6];

                smart_guidance_state = get_smart_guidance_state(system_rc_switch);
				thermalling_state = get_thermalling_state(thermalling_rc_switch);
                
                if (smart_guidance_state != previous_smart_guidance_state || thermalling_state != previous_thermalling_state){
                    if (smart_guidance_state == SMART_GUIDANCE_MODE_SAFE || smart_guidance_state == SMART_GUIDANCE_MODE_ACTIVE){
                        // Allows a mission to be uploaded without defining a landing waypoint
                        this->set_mav_parameter("RTL_TYPE", 0, 3);
                        // Change px4 mode to auto (mission)
                        this->send_mav_command(mavros_msgs::msg::CommandCode::DO_SET_MODE, 4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    }
                    publish_ground_command(smart_guidance_state, thermalling_state);
                    previous_smart_guidance_state = smart_guidance_state;
                    previous_thermalling_state = thermalling_state;
                } });

    waypoint_reached_sub_ = this->create_subscription<mavros_msgs::msg::WaypointReached>(
        "/mavros/mission/reached", wp_qos, [this](const mavros_msgs::msg::WaypointReached::UniquePtr msg)
        { RCLCPP_INFO(rclcpp::get_logger("SMART_GUIDANCE_COM"),
                      "Reached waypoint %i", msg->wp_seq); });

    flight_plan_service_ = this->create_service<soaring_interface::srv::UploadFlightPlan>(
        "/flight_plan",
        [this](const std::shared_ptr<soaring_interface::srv::UploadFlightPlan::Request> request,
               std::shared_ptr<soaring_interface::srv::UploadFlightPlan::Response> response)
        {
            num_waypoints = (int)request->flight_plan.n_waypoints;
            RCLCPP_INFO(rclcpp::get_logger("SMART_GUIDANCE_COM"), "Received new flight plan from Smart Guidance with %i waypoints", num_waypoints);
            flight_path_waypoints.clear();
            for (int i = 0; i < num_waypoints; i++)
            {
                mavros_msgs::msg::Waypoint waypoint;

                waypoint.command = mavros_msgs::msg::CommandCode::NAV_WAYPOINT;
                waypoint.autocontinue = 1;
                waypoint.x_lat = request->flight_plan.waypoints[i].position.latitude;
                waypoint.y_long = request->flight_plan.waypoints[i].position.longitude;
                waypoint.z_alt = request->flight_plan.waypoints[i].position.altitude;
                if (i == 0)
                {
                    waypoint.is_current = 1;
                }
                else if (i == num_waypoints - 1)
                {
                    waypoint.command = mavros_msgs::msg::CommandCode::NAV_LOITER_UNLIM;
                    waypoint.param3 = request->flight_plan.waypoints[i].radius_orbit;
                }

                flight_path_waypoints.push_back(waypoint);
            }
            response->result = true;
            this->push_mav_waypoints(flight_path_waypoints);
        },
        rmw_qos_profile_services_default, service_cb_group_);

    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    timer_cb_group_ = nullptr;

    set_mav_param_client_ = this->create_client<mavros_msgs::srv::ParamSetV2>("/mavros/param/set",
                                                                              rmw_qos_profile_services_default, service_cb_group_);
    send_mav_command_client_ = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command",
                                                                                  rmw_qos_profile_services_default, service_cb_group_);
    push_mav_waypoints_client_ = this->create_client<mavros_msgs::srv::WaypointPush>("/mavros/mission/push",
                                                                                     rmw_qos_profile_services_default, service_cb_group_);

    timer_ = this->create_wall_timer(100ms, std::bind(&SMART_GUIDANCE_COM::timer_callback, this), timer_cb_group_);
}

void SMART_GUIDANCE_COM::publish_aircraft_state() const
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

void SMART_GUIDANCE_COM::publish_ground_command(uint8_t system_state, uint8_t thermalling_state) const
{
    soaring_interface::msg::GroundControlCommand gc_msg{};

    switch (system_state)
    {
    case SMART_GUIDANCE_MODE_DISABLED:
    {
        gc_msg.system_state = SG_SYSTEM_DISABLED;
        break;
    }
    case SMART_GUIDANCE_MODE_SAFE:
    {
        gc_msg.system_state = SG_SYSTEM_SAFE;
        break;
    }
    case SMART_GUIDANCE_MODE_ACTIVE:
    {
        gc_msg.system_state = SG_SYSTEM_ENABLED;
        break;
    }
    }

    switch (thermalling_state)
    {
    case THERMALLING_DISABLED:
    {
        gc_msg.latch_enable = THERMALLING_DISABLED;
        break;
    }
    case THERMALLING_ENABLED:
    {
        gc_msg.latch_enable = THERMALLING_ENABLED;
        break;
    }
    }

    ground_command_publisher_->publish(gc_msg);
}

void SMART_GUIDANCE_COM::publish_wind_state(float wind_north, float wind_east) const
{
    soaring_interface::msg::WindState wind_msg{};

    wind_msg.wind_south = wind_north;
    wind_msg.wind_west = wind_east;

    wind_state_publisher_->publish(wind_msg);
}

void SMART_GUIDANCE_COM::set_mav_parameter(const char *param_id, uint8_t param_value, uint8_t param_type)
{

    auto cmdrq = std::make_shared<mavros_msgs::srv::ParamSetV2::Request>();
    cmdrq->param_id = param_id;
    cmdrq->value.type = param_type;
    cmdrq->value.integer_value = param_value;
    cmdrq->value.double_value = double(param_value);

    auto result_future = set_mav_param_client_->async_send_request(cmdrq);
    std::future_status status = result_future.wait_for(1s);
    if (status == std::future_status::ready)
    {
        auto response = result_future.get();
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "Mav Set Param Response Successful");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Mav Set Param Service Request Failed");
        }
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Mav Set Param Service Not Ready");
    }
}

void SMART_GUIDANCE_COM::send_mav_command(uint16_t command_id, float param1 = 0.0, float param2 = 0.0,
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

    auto result_future = send_mav_command_client_->async_send_request(cmdrq);
    std::future_status future_status = result_future.wait_for(1s);
    if (future_status == std::future_status::ready)
    {
        auto response = result_future.get();
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "Mav Command Response Successful");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Mav Command Service Request Failed");
        }
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Mav Command Service Not Ready");
    }
}

void SMART_GUIDANCE_COM::push_mav_waypoints(std::vector<mavros_msgs::msg::Waypoint> flight_path)
{
    auto cmdrq = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();

    cmdrq->waypoints = flight_path;

    auto result_future = push_mav_waypoints_client_->async_send_request(cmdrq);
    std::future_status future_status = result_future.wait_for(1s);
    if (future_status == std::future_status::ready)
    {
        auto response = result_future.get();
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "Push Waypoints to Mav Response Successful");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Push Waypoints to Mav Service Request Failed");
        }
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Push Waypoints to Mav Service Not Ready");
    }
}

SMART_GUIDANCE_COM::SMART_GUIDANCE_MODE SMART_GUIDANCE_COM::get_smart_guidance_state(uint16_t rc_switch)
{

    if (rc_switch > PWM_LOW_MIN && rc_switch < PWM_LOW_MAX)
    {
        return SMART_GUIDANCE_MODE_DISABLED;
    }
    else if (rc_switch >= PWM_LOW_MAX && rc_switch < PWM_HIGH_MIN)
    {
        return SMART_GUIDANCE_MODE_SAFE;
    }
    else if (rc_switch >= PWM_HIGH_MIN && rc_switch < PWM_HIGH_MAX)
    {
        return SMART_GUIDANCE_MODE_ACTIVE;
    }
    else
    {
        return SMART_GUIDANCE_MODE_DISABLED;
    }
}

SMART_GUIDANCE_COM::THERMALLING_MODE SMART_GUIDANCE_COM::get_thermalling_state(uint16_t rc_switch)
{
    if (rc_switch > PWM_LOW_MIN && rc_switch < PWM_LOW_MAX)
    {
        return THERMALLING_DISABLED;
    }
    else if (rc_switch > PWM_LOW_MAX)
    {
        return THERMALLING_ENABLED;
    }
    else
    {
        return THERMALLING_DISABLED;
    }
}

void SMART_GUIDANCE_COM::timer_callback()
{
    publish_aircraft_state();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto smart_guidance_node = std::make_shared<SMART_GUIDANCE_COM>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(smart_guidance_node);

    RCLCPP_INFO(smart_guidance_node->get_logger(), "Starting Smart Guidance communication node...");
    executor.spin();
    RCLCPP_INFO(smart_guidance_node->get_logger(), "Keyboard interrupt, shutting down...");
    rclcpp::shutdown();
    return 0;
}
