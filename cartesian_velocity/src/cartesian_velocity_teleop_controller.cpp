#include "cartesian_velocity/cartesian_velocity_teleop_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace cartesian_velocity_controller
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CartesianVelocityTeleopController::CartesianVelocityTeleopController()
      : ControllerInterface(), latest_twist_(), filtered_linear_(Eigen::Vector3d::Zero()),
        filtered_angular_(Eigen::Vector3d::Zero()),
        current_orientation_(Eigen::Quaterniond::Identity())
  {
  }

  CartesianVelocityTeleopController::~CartesianVelocityTeleopController() = default;

  controller_interface::InterfaceConfiguration CartesianVelocityTeleopController::
      command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names = robot_vel_interface_->get_commands_names();
    return config;
  }

  controller_interface::InterfaceConfiguration CartesianVelocityTeleopController::
      state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names = robot_vel_interface_->get_states_names();
    return config;
  }

  CallbackReturn CartesianVelocityTeleopController::on_init()
  {
    // Create a subscription to the "cmd_vel" topic to receive teleoperation commands
    twist_sub_ = get_node()->create_subscription<extender_msgs::msg::TeleopCommand>(
        "/teleop_cmd", 10,
        std::bind(&CartesianVelocityTeleopController::twistCallback, this, std::placeholders::_1));

    return CallbackReturn::SUCCESS;
  }

  void CartesianVelocityTeleopController::loadParameters()
  {
    declare_and_get_parameters("gain", gain_, 1.0);
    declare_and_get_parameters("initial_filter_cutoff_frequency", initial_filter_cutoff_frequency_,
                               0.8);
    declare_and_get_parameters("max_linear_delta", max_linear_delta_, 0.007);
    declare_and_get_parameters("max_angular_delta", max_angular_delta_, 0.014);
    declare_and_get_parameters("input_twist_frame", input_twist_frame_, std::string("base"));
    declare_and_get_parameters("robot_type", robot_type_, std::string("franka_velocity"));
    declare_and_get_parameters("command_names", command_names_, std::vector<std::string>{});
    declare_and_get_parameters("excluded_joints", excluded_joints_, std::vector<std::string>{});
  }

  void CartesianVelocityTeleopController::declareSubscribers()
  {
    twist_sub_ = get_node()->create_subscription<extender_msgs::msg::TeleopCommand>(
        "/teleop_cmd", 10,
        std::bind(&CartesianVelocityTeleopController::twistCallback, this, std::placeholders::_1));
  }

  void CartesianVelocityTeleopController::declarePublishers()
  {
    // No publishers for now
  }

  bool CartesianVelocityTeleopController::setupRobotInterface()
  {
    auto node = get_node();
    std::string robot_description;

    if (!node->get_parameter("robot_description", robot_description))
    {
      RCLCPP_ERROR(node->get_logger(), "Missing robot_description");
      return false;
    }

    robot_vel_interface_ = robot_interfaces::create_robot_component(robot_type_);
    if (!robot_vel_interface_ ||
        !robot_vel_interface_->initKinematics(robot_description,
                                              node->get_parameter("tool_frame").as_string(), excluded_joints_))
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize robot interface.");
      return false;
    }
    robot_vel_interface_->set_commands_names(command_names_);

    return true;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  CartesianVelocityTeleopController::on_configure(const rclcpp_lifecycle::State &)
  {
    auto node = get_node();

    loadParameters();

    // Sanitize
    std::transform(input_twist_frame_.begin(), input_twist_frame_.end(), input_twist_frame_.begin(),
                   ::tolower);
    if (input_twist_frame_ != "base" && input_twist_frame_ != "ee")
    {
      RCLCPP_WARN(node->get_logger(),
                  "Invalid input_twist_frame '%s'. Use 'base' or 'ee'. Defaulting to 'base'.",
                  input_twist_frame_.c_str());
      input_twist_frame_ = "base";
    }

    // Logging
    RCLCPP_INFO(node->get_logger(), "Cartesian Velocity Teleop Controller: ");
    RCLCPP_INFO(node->get_logger(), "  gain: %.4f", gain_);
    RCLCPP_INFO(node->get_logger(), "  initial_filter_cutoff_frequency: %.4f Hz",
                initial_filter_cutoff_frequency_);
    RCLCPP_INFO(node->get_logger(), "  max_linear_delta: %.4f", max_linear_delta_);
    RCLCPP_INFO(node->get_logger(), "  max_angular_delta: %.4f", max_angular_delta_);
    RCLCPP_INFO(node->get_logger(), "  input_twist_frame: %s", input_twist_frame_.c_str());

    // Create robot interface
    if (!setupRobotInterface())
    {
      return CallbackReturn::ERROR;
    }

    // Reset LPF state
    filtered_linear_.setZero();
    filtered_angular_.setZero();
    lpf_initialized_ = false;

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn CartesianVelocityTeleopController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Assign the loaned command interfaces to the Franka Cartesian velocity interface
    robot_vel_interface_->assign_loaned_command(command_interfaces_);
    robot_vel_interface_->assign_loaned_state(state_interfaces_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn CartesianVelocityTeleopController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    robot_vel_interface_->release_all_interfaces();
    // Reset LPF state
    filtered_linear_.setZero();
    filtered_angular_.setZero();
    lpf_initialized_ = false;
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type CartesianVelocityTeleopController::update(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    robot_vel_interface_->syncState();

    typedef extender_msgs::msg::TeleopCommand Mode;
    // Get current EE pose.
    robot_interfaces::CartesianPosition temp_pose =
        robot_vel_interface_->getCurrentEndEffectorPose();

    current_orientation_ = temp_pose.quaternion;

    // Convert the latest teleop Twist into raw Eigen vectors (no mode gating yet)
    Eigen::Vector3d raw_linear(latest_twist_.linear.x, latest_twist_.linear.y,
                               latest_twist_.linear.z);
    Eigen::Vector3d raw_angular(latest_twist_.angular.x, latest_twist_.angular.y,
                                latest_twist_.angular.z);

    // Compute alpha from desired cutoff frequency: alpha = T / (T + tau)
    const double dt_sec = period.seconds();
    double alpha = 1.0; // default: passthrough if fc <= 0 or dt <= 0
    if (initial_filter_cutoff_frequency_ > 0.0 && dt_sec > 0.0)
    {
      const double tau = 1.0 / (2.0 * M_PI * initial_filter_cutoff_frequency_);
      alpha = std::clamp(dt_sec / (dt_sec + tau), 1e-9, 1.0 - 1e-9);
    }

    // On first run, initialise filtered vectors to avoid startup transients
    if (!lpf_initialized_)
    {
      filtered_linear_ = raw_linear;
      filtered_angular_ = raw_angular;
      lpf_initialized_ = true;
    }

    // Save previous filtered values for rate limiting
    Eigen::Vector3d prev_linear = filtered_linear_;
    Eigen::Vector3d prev_angular = filtered_angular_;

    // Low-pass filter
    filtered_linear_ = applyLowPassFilterVector(raw_linear, filtered_linear_, alpha);
    filtered_angular_ = applyLowPassFilterVector(raw_angular, filtered_angular_, alpha);

    // Rate limiting
    std::tie(filtered_linear_, filtered_angular_) =
        applyVelocitySaturation(filtered_linear_, prev_linear, max_linear_delta_, filtered_angular_,
                                prev_angular, max_angular_delta_);

    // Apply teleop mode selection on the filtered Cartesian velocities
    Eigen::Vector3d cartesian_linear_velocity = filtered_linear_;
    Eigen::Vector3d cartesian_angular_velocity = filtered_angular_;

    if (input_twist_frame_ == "ee")
    {
      const Eigen::Matrix3d R_BE = current_orientation_.toRotationMatrix();
      cartesian_angular_velocity = R_BE * cartesian_angular_velocity;
    }
    // else: base frame -> do nothing

    switch (mode_)
    {
    case Mode::TRANSLATION:
      // Only translation: ignore angular component
      cartesian_angular_velocity.setZero();
      break;
    case Mode::ROTATION:
      // Only rotation: ignore linear component
      cartesian_linear_velocity.setZero();
      break;
    case Mode::TRANSLATION_ROTATION:
      cartesian_angular_velocity =
          computeBaselineAngularVelocity(temp_pose.translation, cartesian_linear_velocity);
      break;
    default:
      // Use both linear and angular components
      break;
    }

    // Apply overall gain
    cartesian_linear_velocity *= gain_;
    cartesian_angular_velocity *= gain_;

    robot_interfaces::CartesianVelocity vel_cmd;
    vel_cmd.linear = cartesian_linear_velocity;
    vel_cmd.angular = cartesian_angular_velocity;

    if (robot_vel_interface_->setCommand(vel_cmd))
    {
      return controller_interface::return_type::OK;
    }
    else
    {
      RCLCPP_FATAL(get_node()->get_logger(), "Set command failed.");
      return controller_interface::return_type::ERROR;
    }
  }

  Eigen::Vector3d CartesianVelocityTeleopController::applyLowPassFilterVector(
      const Eigen::Vector3d &input, const Eigen::Vector3d &previous, double alpha) const
  {
    return alpha * input + (1.0 - alpha) * previous;
  }

  std::pair<Eigen::Vector3d, Eigen::Vector3d> CartesianVelocityTeleopController::
      applyVelocitySaturation(const Eigen::Vector3d &current_linear,
                              const Eigen::Vector3d &previous_linear, double max_linear_delta,
                              const Eigen::Vector3d &current_angular,
                              const Eigen::Vector3d &previous_angular,
                              double max_angular_delta) const
  {
    Eigen::Vector3d saturated_linear = previous_linear;
    Eigen::Vector3d saturated_angular = previous_angular;

    for (int i = 0; i < 3; ++i)
    {
      const double delta_linear =
          std::clamp(current_linear[i] - previous_linear[i], -max_linear_delta, max_linear_delta);
      const double delta_angular = std::clamp(current_angular[i] - previous_angular[i],
                                              -max_angular_delta, max_angular_delta);

      saturated_linear[i] += delta_linear;
      saturated_angular[i] += delta_angular;
    }

    return {saturated_linear, saturated_angular};
  }

  Eigen::Vector3d CartesianVelocityTeleopController::computeBaselineAngularVelocity(
      const Eigen::Vector3d &current_position, const Eigen::Vector3d &linear_velocity) const
  {
    const double x_E = current_position.x();
    const double y_E = current_position.y();
    const double v_x = linear_velocity.x();
    const double v_y = linear_velocity.y();
    const double denom = x_E * x_E + y_E * y_E;

    double omega_z_baseline = 0.0;
    if (denom > 0.0)
    {
      omega_z_baseline = (x_E * v_y - y_E * v_x) / denom;
    }

    return Eigen::Vector3d(0.0, 0.0, omega_z_baseline); // ω_{E,u,T}
  }

  void CartesianVelocityTeleopController::twistCallback(
      const extender_msgs::msg::TeleopCommand::SharedPtr msg)
  {
    // Update the stored Twist command with the latest message
    latest_twist_ = msg->twist;
    mode_ = msg->mode;
  }

} // namespace cartesian_velocity_controller

PLUGINLIB_EXPORT_CLASS(cartesian_velocity_controller::CartesianVelocityTeleopController,
                       controller_interface::ControllerInterface)
