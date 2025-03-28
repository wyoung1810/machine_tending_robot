// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_
#define CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/controller_interface_base.hpp"

#include "controller_manager/controller_spec.hpp"
#include "controller_manager/visibility_control.h"
#include "controller_manager_msgs/srv/configure_controller.hpp"
#include "controller_manager_msgs/srv/list_controller_types.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "controller_manager_msgs/srv/list_hardware_interfaces.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/reload_controller_libraries.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/unload_controller.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/resource_manager.hpp"

#include "pluginlib/class_loader.hpp"

#include "rclcpp/executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace controller_manager
{
using ControllersListIterator = std::vector<controller_manager::ControllerSpec>::const_iterator;

CONTROLLER_MANAGER_PUBLIC rclcpp::NodeOptions get_cm_node_options();

class ControllerManager : public rclcpp::Node
{
public:
  static constexpr bool kWaitForAllResources = false;
  static constexpr auto kInfiniteTimeout = 0;

  CONTROLLER_MANAGER_PUBLIC
  ControllerManager(
    std::unique_ptr<hardware_interface::ResourceManager> resource_manager,
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & manager_node_name = "controller_manager",
    const std::string & namespace_ = "",
    const rclcpp::NodeOptions & options = get_cm_node_options());

  CONTROLLER_MANAGER_PUBLIC
  ControllerManager(
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & manager_node_name = "controller_manager",
    const std::string & namespace_ = "",
    const rclcpp::NodeOptions & options = get_cm_node_options());

  CONTROLLER_MANAGER_PUBLIC
  virtual ~ControllerManager();

  /// Shutdown all controllers in the controller manager.
  /**
   * \return true if all controllers are successfully shutdown, false otherwise.
   */
  CONTROLLER_MANAGER_PUBLIC
  bool shutdown_controllers();

  CONTROLLER_MANAGER_PUBLIC
  void robot_description_callback(const std_msgs::msg::String & msg);

  CONTROLLER_MANAGER_PUBLIC
  void init_resource_manager(const std::string & robot_description);

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::ControllerInterfaceBaseSharedPtr load_controller(
    const std::string & controller_name, const std::string & controller_type);

  /// load_controller loads a controller by name, the type must be defined in the parameter server.
  /**
   * \param[in] controller_name as a string.
   * \return controller
   * \see Documentation in controller_manager_msgs/LoadController.srv
   */
  CONTROLLER_MANAGER_PUBLIC
  controller_interface::ControllerInterfaceBaseSharedPtr load_controller(
    const std::string & controller_name);

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type unload_controller(const std::string & controller_name);

  CONTROLLER_MANAGER_PUBLIC
  std::vector<ControllerSpec> get_loaded_controllers() const;

  template <
    typename T, typename std::enable_if<
                  std::is_convertible<T *, controller_interface::ControllerInterfaceBase *>::value,
                  T>::type * = nullptr>
  controller_interface::ControllerInterfaceBaseSharedPtr add_controller(
    std::shared_ptr<T> controller, const std::string & controller_name,
    const std::string & controller_type)
  {
    ControllerSpec controller_spec;
    controller_spec.c = controller;
    controller_spec.info.name = controller_name;
    controller_spec.info.type = controller_type;
    return add_controller_impl(controller_spec);
  }

  /// configure_controller Configure controller by name calling their "configure" method.
  /**
   * \param[in] controller_name as a string.
   * \return configure controller response
   * \see Documentation in controller_manager_msgs/ConfigureController.srv
   */
  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type configure_controller(const std::string & controller_name);

  /// switch_controller Stops some controllers and start others.
  /**
   * \param[in] start_controllers is a list of controllers to start
   * \param[in] stop_controllers is a list of controllers to stop
   * \param[in] set level of strictness (BEST_EFFORT or STRICT)
   * \see Documentation in controller_manager_msgs/SwitchController.srv
   */
  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type switch_controller(
    const std::vector<std::string> & start_controllers,
    const std::vector<std::string> & stop_controllers, int strictness,
    bool activate_asap = kWaitForAllResources,
    const rclcpp::Duration & timeout = rclcpp::Duration::from_nanoseconds(kInfiniteTimeout));

  /// Read values to state interfaces.
  /**
   * Read current values from hardware to state interfaces.
   * **The method called in the (real-time) control loop.**
   *
   * \param[in]  time    The time at the start of this control loop iteration
   * \param[in]  period  The measured period of the last control loop iteration
   */
  CONTROLLER_MANAGER_PUBLIC
  void read(const rclcpp::Time & time, const rclcpp::Duration & period);

  /// Run update on controllers
  /**
   * Call update of all controllers.
   * **The method called in the (real-time) control loop.**
   *
   * \param[in]  time    The time at the start of this control loop iteration
   * \param[in]  period  The measured period of the last control loop iteration
   */
  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period);

  /// Write values from command interfaces.
  /**
   * Write values from command interface into hardware.
   * **The method called in the (real-time) control loop.**
   *
   * \param[in]  time    The time at the start of this control loop iteration
   * \param[in]  period  The measured period of the last control loop iteration
   */
  CONTROLLER_MANAGER_PUBLIC
  void write(const rclcpp::Time & time, const rclcpp::Duration & period);

  /// Deterministic (real-time safe) callback group, e.g., update function.
  /**
   * Deterministic (real-time safe) callback group for the update function. Default behavior
   * is read hardware, update controller and finally write new values to the hardware.
   */
  // TODO(anyone): Due to issues with the MutliThreadedExecutor, this control loop does not rely on
  // the executor (see issue #260).
  // rclcpp::CallbackGroup::SharedPtr deterministic_callback_group_;

  // Per controller update rate support
  CONTROLLER_MANAGER_PUBLIC
  unsigned int get_update_rate() const;

protected:
  CONTROLLER_MANAGER_PUBLIC
  void init_services();

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::ControllerInterfaceBaseSharedPtr add_controller_impl(
    const ControllerSpec & controller);

  CONTROLLER_MANAGER_PUBLIC
  void manage_switch();

  CONTROLLER_MANAGER_PUBLIC
  void deactivate_controllers();

  /**
   * Switch chained mode for all the controllers with respect to the following cases:
   * - a preceding controller is getting activated --> switch controller to chained mode;
   * - all preceding controllers are deactivated --> switch controller from chained mode.
   *
   * \param[in] chained_mode_switch_list list of controller to switch chained mode.
   * \param[in] to_chained_mode flag if controller should be switched *to* or *from* chained mode.
   */
  CONTROLLER_MANAGER_PUBLIC
  void switch_chained_mode(
    const std::vector<std::string> & chained_mode_switch_list, bool to_chained_mode);

  CONTROLLER_MANAGER_PUBLIC
  void activate_controllers();

  CONTROLLER_MANAGER_PUBLIC
  void activate_controllers_asap();

  CONTROLLER_MANAGER_PUBLIC
  void list_controllers_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void list_hardware_interfaces_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListHardwareInterfaces::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListHardwareInterfaces::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void load_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::LoadController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::LoadController::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void configure_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ConfigureController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ConfigureController::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void reload_controller_libraries_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ReloadControllerLibraries::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ReloadControllerLibraries::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void switch_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void unload_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::UnloadController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::UnloadController::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void list_controller_types_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void list_hardware_components_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListHardwareComponents::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListHardwareComponents::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void set_hardware_component_state_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::SetHardwareComponentState::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::SetHardwareComponentState::Response> response);

  // Per controller update rate support
  unsigned int update_loop_counter_ = 0;
  unsigned int update_rate_ = 100;
  std::vector<std::vector<std::string>> chained_controllers_configuration_;

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_;

private:
  std::vector<std::string> get_controller_names();
  std::pair<std::string, std::string> split_command_interface(
    const std::string & command_interface);
  void subscribe_to_robot_description_topic();

  /**
   * Clear request lists used when switching controllers. The lists are shared between "callback"
   * and "control loop" threads.
   */
  void clear_requests();

  /**
   * If a controller is deactivated all following controllers (if any exist) should be switched
   * 'from' the chained mode.
   *
   * \param[in] controllers list with controllers.
   */
  void propagate_deactivation_of_chained_mode(const std::vector<ControllerSpec> & controllers);

  /// Check if all the following controllers will be in active state and in the chained mode
  /// after controllers' switch.
  /**
   * Check recursively that all following controllers of the @controller_it
   * - are already active,
   * - will not be deactivated,
   * - or will be activated.
   * The following controllers are added to the request to switch in the chained mode or removed
   * from the request to switch from the chained mode.
   *
   * For each controller the whole chain of following controllers is checked.
   *
   * NOTE: The automatically adding of following controller into starting list is not implemented
   * yet.
   *
   * \param[in] controllers list with controllers.
   * \param[in] strictness if value is equal "MANIPULATE_CONTROLLERS_CHAIN" then all following
   * controllers will be automatically added to the activate request list if they are not in the
   * deactivate request.
   * \param[in] controller_it iterator to the controller for which the following controllers are
   * checked.
   *
   * \returns return_type::OK if all following controllers pass the checks, otherwise
   * return_type::ERROR.
   */
  controller_interface::return_type check_following_controllers_for_activate(
    const std::vector<ControllerSpec> & controllers, int strictness,
    const ControllersListIterator controller_it);

  /// Check if all the preceding controllers will be in inactive state after controllers' switch.
  /**
   * Check that all preceding controllers of the @controller_it
   * - are inactive,
   * - will be deactivated,
   * - and will not be activated.
   *
   * NOTE: The automatically adding of preceding controllers into stopping list is not implemented
   * yet.
   *
   * \param[in] controllers list with controllers.
   * \param[in] strictness if value is equal "MANIPULATE_CONTROLLERS_CHAIN" then all preceding
   * controllers will be automatically added to the deactivate request list.
   * \param[in] controller_it iterator to the controller for which the preceding controllers are
   * checked.
   *
   * \returns return_type::OK if all preceding controllers pass the checks, otherwise
   * return_type::ERROR.
   */
  controller_interface::return_type check_preceeding_controllers_for_deactivate(
    const std::vector<ControllerSpec> & controllers, int strictness,
    const ControllersListIterator controller_it);

  /// A method to be used in the std::sort method to sort the controllers to be able to
  /// execute them in a proper order
  /**
   * Compares the controllers ctrl_a and ctrl_b and then returns which comes first in the sequence
   *
   *  @note The following conditions needs to be handled while ordering the controller list
   *  1. The controllers that do not use any state or command interfaces are updated first
   *  2. The controllers that use only the state system interfaces only are updated next
   *  3. The controllers that use any of an another controller's reference interface are updated
   * before the preceding controller
   *  4. The controllers that use the controller's estimated interfaces are updated after the
   * preceding controller
   *  5. The controllers that only use the hardware command interfaces are updated last
   *  6. All inactive controllers go at the end of the list
   *
   * \param[in] controllers list of controllers to compare their names to interface's prefix.
   *
   * @return true, if ctrl_a needs to execute first, else false
   */
  bool controller_sorting(
    const ControllerSpec & ctrl_a, const ControllerSpec & ctrl_b,
    const std::vector<controller_manager::ControllerSpec> & controllers);

  /**
   * @brief determine_controller_node_options - A method that retrieves the controller defined node
   * options and adapts them, based on if there is a params file to be loaded or the use_sim_time
   * needs to be set
   * @param controller - controller info
   * @return The node options that will be set to the controller LifeCycleNode
   */
  rclcpp::NodeOptions determine_controller_node_options(const ControllerSpec & controller) const;

  std::shared_ptr<rclcpp::Executor> executor_;

  std::shared_ptr<pluginlib::ClassLoader<controller_interface::ControllerInterface>> loader_;
  std::shared_ptr<pluginlib::ClassLoader<controller_interface::ChainableControllerInterface>>
    chainable_loader_;

  /// Best effort (non real-time safe) callback group, e.g., service callbacks.
  /**
   * Best effort (non real-time safe) callback group for callbacks that can possibly break
   * real-time requirements, for example, service callbacks.
   */
  rclcpp::CallbackGroup::SharedPtr best_effort_callback_group_;

  /**
   * The RTControllerListWrapper class wraps a double-buffered list of controllers
   * to avoid needing to lock the real-time thread when switching controllers in
   * the non-real-time thread.
   *
   * There's always an "updated" list and an "outdated" one
   * There's always an "used by rt" list and an "unused by rt" list
   *
   * The updated state changes on the switch_updated_list()
   * The rt usage state changes on the update_and_get_used_by_rt_list()
   */
  class RTControllerListWrapper
  {
    // *INDENT-OFF*
  public:
    // *INDENT-ON*
    /// update_and_get_used_by_rt_list Makes the "updated" list the "used by rt" list
    /**
     * \warning Should only be called by the RT thread, no one should modify the
     * updated list while it's being used
     * \return reference to the updated list
     */
    std::vector<ControllerSpec> & update_and_get_used_by_rt_list();

    /**
     * get_unused_list Waits until the "outdated" and "unused by rt"
     * lists match and returns a reference to it
     * This referenced list can be modified safely until switch_updated_controller_list()
     * is called, at this point the RT thread may start using it at any time
     * \param[in] guard Guard needed to make sure the caller is the only one accessing the unused by
     * rt list
     */
    std::vector<ControllerSpec> & get_unused_list(
      const std::lock_guard<std::recursive_mutex> & guard);

    /// get_updated_list Returns a const reference to the most updated list.
    /**
     * \warning May or may not being used by the realtime thread, read-only reference for safety
     * \param[in] guard Guard needed to make sure the caller is the only one accessing the unused by
     * rt list
     */
    const std::vector<ControllerSpec> & get_updated_list(
      const std::lock_guard<std::recursive_mutex> & guard) const;

    /**
     * switch_updated_list Switches the "updated" and "outdated" lists, and waits
     *  until the RT thread is using the new "updated" list.
     * \param[in] guard Guard needed to make sure the caller is the only one accessing the unused by
     * rt list
     */
    void switch_updated_list(const std::lock_guard<std::recursive_mutex> & guard);

    // Mutex protecting the controllers list
    // must be acquired before using any list other than the "used by rt"
    mutable std::recursive_mutex controllers_lock_;

    // *INDENT-OFF*
  private:
    // *INDENT-ON*
    /// get_other_list get the list not pointed by index
    /**
     * \param[in] index int
     */
    int get_other_list(int index) const;

    void wait_until_rt_not_using(
      int index, std::chrono::microseconds sleep_delay = std::chrono::microseconds(200)) const;

    std::vector<ControllerSpec> controllers_lists_[2];
    /// The index of the controller list with the most updated information
    int updated_controllers_index_ = 0;
    /// The index of the controllers list being used in the real-time thread.
    int used_by_realtime_controllers_index_ = -1;
  };

  std::unique_ptr<rclcpp::PreShutdownCallbackHandle> preshutdown_cb_handle_{nullptr};
  RTControllerListWrapper rt_controllers_wrapper_;
  /// mutex copied from ROS1 Control, protects service callbacks
  /// not needed if we're guaranteed that the callbacks don't come from multiple threads
  std::mutex services_lock_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllers>::SharedPtr
    list_controllers_service_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllerTypes>::SharedPtr
    list_controller_types_service_;
  rclcpp::Service<controller_manager_msgs::srv::LoadController>::SharedPtr load_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::ConfigureController>::SharedPtr
    configure_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::ReloadControllerLibraries>::SharedPtr
    reload_controller_libraries_service_;
  rclcpp::Service<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::UnloadController>::SharedPtr
    unload_controller_service_;

  rclcpp::Service<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr
    list_hardware_components_service_;
  rclcpp::Service<controller_manager_msgs::srv::ListHardwareInterfaces>::SharedPtr
    list_hardware_interfaces_service_;
  rclcpp::Service<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr
    set_hardware_component_state_service_;

  std::vector<std::string> activate_request_, deactivate_request_;
  std::vector<std::string> to_chained_mode_request_, from_chained_mode_request_;
  std::vector<std::string> activate_command_interface_request_,
    deactivate_command_interface_request_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;

  struct SwitchParams
  {
    void reset()
    {
      do_switch = false;
      started = false;
      strictness = 0;
      activate_asap = false;
    }

    bool do_switch;
    bool started;

    // Switch options
    int strictness;
    bool activate_asap;
    std::chrono::nanoseconds timeout;

    // conditional variable and mutex to wait for the switch to complete
    std::condition_variable cv;
    std::mutex mutex;
  };

  SwitchParams switch_params_;
};

}  // namespace controller_manager

#endif  // CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_
