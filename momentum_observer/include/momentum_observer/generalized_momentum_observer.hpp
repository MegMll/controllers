#pragma once

#include <numeric>

#include <Eigen/Dense>

// This includes are mandatory
#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "robot_interfaces/generic_component.hpp"
#include "robot_interfaces/robot_interfaces_algos.hpp"

#include <momentum_observer/momentum_observer_param_lib.hpp>
// Add all includes your project needs here

namespace cartesian_velocity_controller
{
  /// @brief
  class GeneralizedMomentumObserver : public controller_interface::ChainableControllerInterface
  {
  public:
    GeneralizedMomentumObserver();
    virtual ~GeneralizedMomentumObserver() = default;

    // Configure command and state interfaces
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    /// Main update loop called periodically by the controller manager
    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    // Lifecycle callbacks
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
    controller_interface::return_type update_reference_from_subscribers() override;

  private:
    /**
     * @brief Template function to declare and get parameters with default values.
     * @tparam T Type of the parameter.
     * @param name Parameter name.
     * @param variable Reference to store the parameter value.
     * @param default_value Default value if parameter is not set.
     */
    template <typename T>
    void declare_and_get_parameters(const std::string &name, T &variable, const T &default_value)
    {
      auto node = get_node();
      if (!node->has_parameter(name))
      {
        node->declare_parameter(name, default_value);
      }
      variable = node->get_parameter(name).get_value<T>();
    }

    void loadParameters();
    void updateParameters();
    void setupSubscribers();
    void setupPublishers();
    bool setupRobotInterface();

    void activatePublishers();
    void deactivatePublishers();

    /// Callback to receive Twist commands from the teleop node
    void payloadCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void publishInfo();

    enum class ObserverState
    {
      NORMAL,
      CALIBRATING,
      LIFTING
    };
    ObserverState state_ = ObserverState::NORMAL;

    /// Generic component to interface with robot hardware
    std::string robot_type_{"explorer_velocity"};
    std::vector<std::string> command_names_;
    std::string tool_frame_;
    std::vector<std::string> excluded_gripper_joints;
    std::unique_ptr<robot_interfaces::GenericComponent> robot_interface_;

    robot_interfaces::CartesianVelocity latest_vel_cmd;

    // Publishers
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr
        op_vel_command_pub;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr op_pose_pub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr residuals_pub_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr payload_sub_;
    // Param Listener
    std::shared_ptr<generalized_momentum_observer::ParamListener> param_listener_;
    generalized_momentum_observer::Params params_;

    robot_interfaces::CartesianPosition current_ee_pose;
    Eigen::VectorXd momentum_start;
    Eigen::VectorXd current_momentum;
    Eigen::VectorXd estimated_residuals;
    Eigen::VectorXd integral_sum;
    Eigen::VectorXd f_ext;
    Eigen::VectorXd f_ext_filtered = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd f_ext_bias = Eigen::VectorXd::Zero(6);

    Eigen::MatrixXd jacobian_ee;
    Eigen::MatrixXd jacobian_ee_inverted;
    Eigen::MatrixXd observer_gain;
    double admittance_gain;
    double lpf_alpha;
    double bias_alpha;

    // Payload internal state
    bool has_payload = false;
    bool prev_has_payload_ = false;
    double payload_mass_ = 0.0;
    size_t lifting_count = 0;
    Eigen::Vector3d payload_com_local_ = Eigen::Vector3d::Zero();

    // Calibration buffer
    std::vector<double> mass_buffer_;
    const size_t SAMPLES_FOR_CALIBRATION = 500; // 0.5s at 1kHz
    const size_t SAMPLES_LIFT = 500;

    std::array<std::string, 6> reference_command_interface_names_ = {
        "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"};
  };
} // namespace cartesian_velocity_controller