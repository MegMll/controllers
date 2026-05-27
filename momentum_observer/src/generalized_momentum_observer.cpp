#include "momentum_observer/generalized_momentum_observer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace cartesian_velocity_controller
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  GeneralizedMomentumObserver::GeneralizedMomentumObserver()
      : controller_interface::ChainableControllerInterface()
  {
  }

  std::vector<hardware_interface::CommandInterface> GeneralizedMomentumObserver::
      on_export_reference_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> reference_interfaces;
    reference_interfaces.reserve(reference_command_interface_names_.size());
    reference_interfaces_.resize(reference_command_interface_names_.size());

    size_t index = 0;
    for (const auto &name : reference_command_interface_names_)
    {
      reference_interfaces.push_back(hardware_interface::CommandInterface(
          get_node()->get_name(), name, &reference_interfaces_.at(index)));
      index++;
    }

    return reference_interfaces;
  }

  controller_interface::return_type GeneralizedMomentumObserver::update_reference_from_subscribers()
  {

    reference_interfaces_[0] = 0.;
    reference_interfaces_[1] = 0.;
    reference_interfaces_[2] = 0.;
    reference_interfaces_[3] = 0.;
    reference_interfaces_[4] = 0.;
    reference_interfaces_[5] = 0.;

    return controller_interface::return_type::OK;
  }

  controller_interface::InterfaceConfiguration GeneralizedMomentumObserver::
      command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names = robot_interface_->get_commands_names();
    return config;
  }

  controller_interface::InterfaceConfiguration GeneralizedMomentumObserver::
      state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names = robot_interface_->get_states_names();
    return config;
  }

  CallbackReturn GeneralizedMomentumObserver::on_init()
  {
    param_listener_ = std::make_shared<generalized_momentum_observer::ParamListener>(get_node());

    return CallbackReturn::SUCCESS;
  }

  void GeneralizedMomentumObserver::loadParameters()
  {
    declare_and_get_parameters("robot_type", robot_type_, std::string("explorer_velocity"));
    declare_and_get_parameters("command_names", command_names_, std::vector<std::string>{});
    declare_and_get_parameters("tool_frame", tool_frame_, std::string("end_effector_link"));
    declare_and_get_parameters("excluded_joints", excluded_gripper_joints,
                               std::vector<std::string>{});

    params_ = param_listener_->get_params();

    // Setup gain matrix
    Eigen::Map<Eigen::VectorXd> gains_vector_eigen(params_.observer_gains.data(),
                                                   params_.observer_gains.size());
    observer_gain = gains_vector_eigen.asDiagonal();

    admittance_gain = params_.admittance_gain;
    lpf_alpha = params_.lpf_alpha;
    bias_alpha = params_.bias_alpha;
  }

  void GeneralizedMomentumObserver::updateParameters()
  {
    Eigen::Map<Eigen::VectorXd> gains_vector_eigen(params_.observer_gains.data(),
                                                   params_.observer_gains.size());
    observer_gain = gains_vector_eigen.asDiagonal();

    admittance_gain = params_.admittance_gain;
    lpf_alpha = params_.lpf_alpha;
    bias_alpha = params_.bias_alpha;
  }

  void GeneralizedMomentumObserver::setupSubscribers()
  {
    auto node = get_node();
    payload_sub_ = node ->create_subscription<std_msgs::msg::Bool>(
        "~/has_payload", 10,
        std::bind(&GeneralizedMomentumObserver::payloadCallback, this, std::placeholders::_1));
  }

  void GeneralizedMomentumObserver::setupPublishers()
  {
    auto node = get_node();
    op_vel_command_pub =
        node->create_publisher<geometry_msgs::msg::TwistStamped>("~/velocity_command", 10);
    op_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("~/ee_pose", 10);
    residuals_pub_ =
        node->create_publisher<sensor_msgs::msg::JointState>("~/estimated_residuals", 10);
  }

  void GeneralizedMomentumObserver::activatePublishers()
  {
    op_vel_command_pub->on_activate();
    op_pose_pub->on_activate();
    residuals_pub_->on_activate();
  }

  void GeneralizedMomentumObserver::deactivatePublishers()
  {
    op_vel_command_pub->on_deactivate();
    op_pose_pub->on_deactivate();
    residuals_pub_->on_deactivate();
  }

  bool GeneralizedMomentumObserver::setupRobotInterface()
  {
    auto node = get_node();
    std::string robot_description;

    if (!node->get_parameter("robot_description", robot_description))
    {
      RCLCPP_ERROR(node->get_logger(), "Missing robot_description");
      return false;
    }

    robot_interface_ = robot_interfaces::create_robot_component(robot_type_);
    if (!robot_interface_ || !robot_interface_->initKinematics(
                                 robot_description, node->get_parameter("tool_frame").as_string(),
                                 excluded_gripper_joints))
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize robot interface.");
      return false;
    }
    robot_interface_->set_commands_names(command_names_);

    integral_sum.setZero(robot_interface_->getJointTorques().size());
    estimated_residuals.setZero(robot_interface_->getJointTorques().size());
    current_momentum.setZero(robot_interface_->getJointTorques().size());
    momentum_start.setZero(robot_interface_->getJointTorques().size());
    jacobian_ee.setZero(robot_interface_->getJointTorques().size(),
                        robot_interface_->getJointTorques().size());
    jacobian_ee_inverted.setZero(robot_interface_->getJointTorques().size(),
                                 robot_interface_->getJointTorques().size());

    return true;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  GeneralizedMomentumObserver::on_configure(const rclcpp_lifecycle::State &)
  {
    auto node = get_node();
    loadParameters();

    // Create robot interface
    if (!setupRobotInterface())
    {
      return CallbackReturn::ERROR;
    }

    setupSubscribers();
    setupPublishers();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn GeneralizedMomentumObserver::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Assign the loaned command interfaces to the velocity interface
    robot_interface_->assign_loaned_command(command_interfaces_);
    robot_interface_->assign_loaned_state(state_interfaces_);

    activatePublishers();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn GeneralizedMomentumObserver::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    robot_interface_->release_all_interfaces();
    deactivatePublishers();
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type GeneralizedMomentumObserver::update_and_write_commands(
      const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    robot_interface_->syncState();

    if (param_listener_->is_old(params_))
    {
      params_ = param_listener_->get_params();
      updateParameters();
      RCLCPP_INFO(get_node()->get_logger(), "Parameters updated!");
    }

    auto q = robot_interface_->getJointPositions();
    auto q_dot = robot_interface_->getJointVelocities();
    current_ee_pose = robot_interface_->getCurrentEndEffectorPose();

    Eigen::VectorXd v_final = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd v_nominal = Eigen::VectorXd::Zero(6);
    if (!std::isnan(reference_interfaces_[0]))
    {
      for (size_t i = 0; i < 6; ++i)
        v_nominal(i) = reference_interfaces_[i];
    }

    if (q.hasNaN())
    {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                           "Waiting for valid joint states...");
      return controller_interface::return_type::OK;
    }

    if (!params_.admittance_active)
    {
      v_final = v_nominal;
    }
    else
    {

      if (has_payload != prev_has_payload_)
      {
        // Reset the integral and the starting momentum
        integral_sum.setZero();
        momentum_start = robot_interface_->getMassMatrix() * q_dot;
        estimated_residuals.setZero();
        f_ext_filtered.setZero();
        f_ext_bias.setZero();

        if (has_payload)
        {
          state_ = ObserverState::LIFTING;
          mass_buffer_.clear();
          RCLCPP_INFO(get_node()->get_logger(),
                      "Payload attached: Resetting observer & Calibrating...");
        }
        else
        {
          state_ = ObserverState::NORMAL;
          payload_mass_ = 0.0;
          payload_com_local_.setZero();
          RCLCPP_INFO(get_node()->get_logger(), "Payload removed: Resetting observer...");
        }
      }
      prev_has_payload_ = has_payload;

      // p = M(q) * q_dot
      current_momentum.noalias() = robot_interface_->getMassMatrix() * q_dot;

      // r = K * (p - integral(tau + C'*q_dot - g + r)dt)
      integral_sum += (params_.sensors_gravity_comp * robot_interface_->getJointTorques() +
                       robot_interface_->getCoriolisMatrix().transpose() * q_dot -
                       robot_interface_->getGravityEffort() + estimated_residuals) *
                      period.seconds();

      estimated_residuals = observer_gain * (current_momentum - momentum_start - integral_sum);

      jacobian_ee = robot_interface_->getEndEffectorJacobian();
      double lambda = 0.05; // Damping for pseudo-inverse
      jacobian_ee_inverted = ((jacobian_ee * jacobian_ee.transpose()) +
                              (lambda * lambda * Eigen::MatrixXd::Identity(6, 6)))
                                 .inverse() *
                             jacobian_ee;

      f_ext = jacobian_ee_inverted * estimated_residuals;

      // Filter the raw force estimate
      f_ext_filtered = (1.0 - lpf_alpha) * f_ext_filtered + (lpf_alpha)*f_ext;

      Eigen::VectorXd v_admittance = Eigen::VectorXd::Zero(6);
      Eigen::Matrix3d R_base_to_ee = current_ee_pose.quaternion.toRotationMatrix();

      if (state_ == ObserverState::LIFTING)
      {
        v_admittance.setZero();
        v_admittance(2) = 0.02;

        lifting_count++;
        if (lifting_count >= SAMPLES_LIFT)
        {
          state_ = ObserverState::CALIBRATING;
          lifting_count = 0;
        }
      }
      else if (state_ == ObserverState::CALIBRATING)
      {
        v_admittance.setZero(); // Command zero velocity

        // Accumulate mass samples (Z-force / gravity)
        double instant_mass = f_ext_filtered(2) / -9.81;
        mass_buffer_.push_back(instant_mass);

        if (mass_buffer_.size() >= SAMPLES_FOR_CALIBRATION)
        {
          double sum = std::accumulate(mass_buffer_.begin(), mass_buffer_.end(), 0.0);
          payload_mass_ = sum / mass_buffer_.size();

          // Get Force and Torque in the TOOL frame
          Eigen::Vector3d F_base = f_ext_filtered.head<3>();
          Eigen::Vector3d T_base = f_ext_filtered.tail<3>();
          Eigen::Vector3d F_ee = R_base_to_ee * F_base;
          Eigen::Vector3d T_ee = R_base_to_ee * T_base;

          // Solve Torque = r x F for r.
          if (F_ee.norm() > 1.0)
          { // Only if force is significant
            payload_com_local_ = F_ee.cross(T_ee) / F_ee.squaredNorm();
          }

          state_ = ObserverState::NORMAL;
          f_ext_bias.setZero(); // Reset bias to re-zero for human interaction
          RCLCPP_INFO(get_node()->get_logger(),
                      "CALIBRATION DONE. Mass: %.3f kg | CoM: [%.2f, %.2f, %.2f]", payload_mass_,
                      payload_com_local_(0), payload_com_local_(1), payload_com_local_(2));
        }
      }
      else
      {
        Eigen::VectorXd payload_wrench_base = Eigen::VectorXd::Zero(6);
        if (has_payload && std::abs(payload_mass_) > 0.01)
        {
          Eigen::Vector3d gravity_base(0, 0, -9.81);
          Eigen::Vector3d gravity_ee = R_base_to_ee * gravity_base;

          // Force/Torque in Tool Frame
          Eigen::Vector3d f_p_ee = payload_mass_ * gravity_ee;
          Eigen::Vector3d tau_p_ee = payload_com_local_.cross(f_p_ee);

          // Rotate back to Base Frame to subtract from world-frame observer
          payload_wrench_base.head<3>() = R_base_to_ee.transpose() * f_p_ee;
          payload_wrench_base.tail<3>() = R_base_to_ee.transpose() * tau_p_ee;
        }

        // Subtract compensation so the admittance controller doesn't "see" the payload
        f_ext = f_ext_filtered - payload_wrench_base;
        if (q_dot.norm() < 0.1 && f_ext.norm() < params_.force_threshold * 2.0)
        {
          f_ext_bias = (1.0 - bias_alpha) * f_ext_bias + (bias_alpha)*f_ext;
        }
        f_ext = f_ext - f_ext_bias;

        // Apply Deadbands (Thresholds)
        Eigen::Vector3d f_linear = f_ext.head<3>();
        double f_norm = f_linear.norm();

        if (f_norm < params_.force_threshold)
        {
          f_ext.head<3>().setZero();
        }
        else
        {
          // Subtract the threshold from the magnitude while preserving direction
          // New vector = (f_linear / norm) * (norm - threshold)
          // Simplified as: f_linear * (1.0 - threshold / norm)
          f_ext.head<3>() = f_linear * (1.0 - params_.force_threshold / f_norm);
        }

        // 2. Handle Angular Torques (Indices 3, 4, 5)
        Eigen::Vector3d f_angular = f_ext.tail<3>();
        double t_norm = f_angular.norm();

        if (t_norm < params_.torque_threshold)
        {
          f_ext.tail<3>().setZero();
        }
        else
        {
          // Same logic for torque
          f_ext.tail<3>() = f_angular * (1.0 - params_.torque_threshold / t_norm);
        }
        // for (int i = 0; i < 6; ++i)
        // {
        //   double thresh = (i < 3) ? params_.force_threshold : params_.torque_threshold;
        //   if (std::abs(f_ext(i)) < thresh)
        //   {
        //     f_ext(i) = 0.0;
        //   }
        //   else
        //   {
        //     // Smooth ramp: subtract threshold from the magnitude
        //     f_ext(i) = f_ext(i) - (thresh * (f_ext(i) > 0 ? 1 : -1));
        //   }
        // }
        v_admittance = params_.admittance_gain * f_ext;
      }

      // RCLCPP_INFO_THROTTLE(
      //     get_node()->get_logger(), *get_node()->get_clock(), 1000,
      //     "v_admittance -> Linear(X: %.3f, Y: %.3f, Z: %.3f) | Angular(R: %.3f, P: %.3f, Y:
      //     %.3f)", v_admittance(0), v_admittance(1), v_admittance(2), v_admittance(3),
      //     v_admittance(4), v_admittance(5));
      // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      //                      "State: %s | Mass: %.2f kg | Vel Norm: %.3f",
      //                      (state_ == ObserverState::CALIBRATING ? "CALIB" : "NORMAL"),
      //                      payload_mass_, v_admittance.norm());

      v_final = v_nominal + v_admittance;
    }
    latest_vel_cmd.linear[0] = v_final(0);
    latest_vel_cmd.linear[1] = v_final(1);
    latest_vel_cmd.linear[2] = v_final(2);
    latest_vel_cmd.angular[0] = v_final(3);
    latest_vel_cmd.angular[1] = v_final(4);
    latest_vel_cmd.angular[2] = v_final(5);

    // latest_vel_cmd.linear[0] = 0.;
    // latest_vel_cmd.linear[1] = 0.;
    // latest_vel_cmd.linear[2] = 0.;
    // latest_vel_cmd.angular[0] = 0.;
    // latest_vel_cmd.angular[1] = 0.;
    // latest_vel_cmd.angular[2] = 0.;

    publishInfo();

    if (robot_interface_->setCommand(latest_vel_cmd))
    {
      return controller_interface::return_type::OK;
    }
    else
    {
      RCLCPP_FATAL(get_node()->get_logger(), "Hardware command failed.");
      return controller_interface::return_type::ERROR;
    }
  }

  void GeneralizedMomentumObserver::publishInfo()
  {
    // Publish operational pose
    robot_interfaces::CartesianPosition temp_pose = robot_interface_->getCurrentEndEffectorPose();

    geometry_msgs::msg::PoseStamped pose_to_pub;

    pose_to_pub.header.stamp = get_node()->now();
    pose_to_pub.header.frame_id = tool_frame_;

    pose_to_pub.pose.position.x = temp_pose.translation[0];
    pose_to_pub.pose.position.y = temp_pose.translation[1];
    pose_to_pub.pose.position.z = temp_pose.translation[2];
    pose_to_pub.pose.orientation.x = temp_pose.quaternion.x();
    pose_to_pub.pose.orientation.y = temp_pose.quaternion.y();
    pose_to_pub.pose.orientation.z = temp_pose.quaternion.z();
    pose_to_pub.pose.orientation.w = temp_pose.quaternion.w();

    op_pose_pub->publish(pose_to_pub);

    // Publish computed velocity
    geometry_msgs::msg::TwistStamped vel_to_pub;

    vel_to_pub.header.stamp = get_node()->now();
    vel_to_pub.header.frame_id = tool_frame_;

    vel_to_pub.twist.linear.x = latest_vel_cmd.linear[0];
    vel_to_pub.twist.linear.y = latest_vel_cmd.linear[1];
    vel_to_pub.twist.linear.z = latest_vel_cmd.linear[2];
    vel_to_pub.twist.angular.x = latest_vel_cmd.angular[0];
    vel_to_pub.twist.angular.y = latest_vel_cmd.angular[1];
    vel_to_pub.twist.angular.z = latest_vel_cmd.angular[2];

    op_vel_command_pub->publish(vel_to_pub);

    sensor_msgs::msg::JointState residuals_msg;
    residuals_msg.header.stamp = get_node()->now();
    // Generate simple names: joint_0, joint_1, etc.
    size_t num_joints = estimated_residuals.size();
    residuals_msg.name.reserve(num_joints);
    for (size_t i = 0; i < num_joints; ++i)
    {
      residuals_msg.name.push_back("joint_" + std::to_string(i));
    }
    residuals_msg.effort.assign(estimated_residuals.data(),
                                estimated_residuals.data() + estimated_residuals.size());

    residuals_pub_->publish(residuals_msg);
  }

  void GeneralizedMomentumObserver::payloadCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    has_payload = msg->data;
  }

} // namespace cartesian_velocity_controller

PLUGINLIB_EXPORT_CLASS(cartesian_velocity_controller::GeneralizedMomentumObserver,
                       controller_interface::ChainableControllerInterface)
