// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#ifndef SINGLE_TRIGGER_CONTROLLER__GPIO_COMMAND_CONTROLLER_HPP_
#define SINGLE_TRIGGER_CONTROLLER__GPIO_COMMAND_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "control_msgs/msg/dynamic_interface_group_values.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

#include "single_trigger_controller/single_trigger_controller_parameters.hpp"

namespace single_trigger_controller
{
  using CmdType = control_msgs::msg::DynamicInterfaceGroupValues;
  using StateType = control_msgs::msg::DynamicInterfaceGroupValues;
  using CallbackReturn = controller_interface::CallbackReturn;
  using InterfacesNames = std::vector<std::string>;
  using MapOfReferencesToCommandInterfaces = std::unordered_map<
      std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>>;
  using MapOfReferencesToStateInterfaces =
      std::unordered_map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>>;
  using StateInterfaces =
      std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>;

  /**
   * @brief A general purpose controller which can be used to trigger command interfaces.
   * 
   * This controller is a derivative of the ros2_control [GpioCommandController](https://control.ros.org/master/doc/ros2_controllers/gpio_controllers/doc/userdoc.html).
   * The key differences between this controller and the GpioCommandController is two-fold:
   * 1. Besides GPIO interfaces, this controller allows Joint interfaces to be used, enabling the user
   * to utilize the controller in broader applications and simplifying the URDF description.
   * 2. The GpioCommandController wirtes the latest received command to the configured state interfaces
   * on every update cycle. On the contrary, the SingleTriggerController only writes to the command interfaces
   * once in the subscription callback when a new command has been received. The user can for example
   * leverage this by clearing the command interface (writing NaN to it) in the hardware interface once the latest trigger has been handled.
   * 
   * The controller subscribes and publishes to the same topics as the GpioCommandController.
   */
  class SingleTriggerController : public controller_interface::ControllerInterface
  {
  public:
    SingleTriggerController();

    /**
     * @brief Sets the command interface configuration to INDIVIDUAL.
     * 
     * See ControllerInterfaceBase::command_interface_configuration()
     * 
     * @return controller_interface::InterfaceConfiguration 
     */
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    /**
     * @brief Sets the state interface configuration to INDIVIDUAL.
     * 
     * See ControllerInterfaceBase::state_interface_configuration()
     * 
     * @return controller_interface::InterfaceConfiguration 
     */
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    /**
     * @brief Called on initialization to the `unconfigured` state.
     * 
     * Retrieves and stores the parameters supplied to the controller in #params_
     * 
     * @return hardware_interface::CallbackReturn
     */
    CallbackReturn on_init() override;

    /**
     * @brief Called on the transistion from the `unconfigured` to the `inactive` state.
     * 
     * Stores all configured command and state interfaces using the 
     * store_command_interface_types() and store_state_interface_types() methods.
     * 
     * Creates a subscription to the '~/commands' topic if command interfaces are configured.
     * Creates a realtime publisher to the '~/states' topic is state interfaces are configured.
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;


    /**
     * @brief Called on the transistion from the `inactive` to the `active` state.
     * 
     * Populates command_interfaces_map_ and state_interfaces_map_ 
     * by invoking create_map_of_references_to_interfaces() for both command and state interfaces.
     * 
     * Initializes an empty state message by calling initialize_state_msg().
     * 
     * @param previous_state 
     * @return CallbackReturn 
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    /**
     * @brief Called on the transistion from the `active` to the `inactive` state. 
     * @param previous_state 
     * @return CallbackReturn 
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    /**
     * @brief Realtime update method called when in `active` state.
     * 
     * Only publishes the states by invoking update_states().
     * 
     * @param time 
     * @param period 
     * @return controller_interface::return_type 
     */
    controller_interface::return_type update(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:

    /**
     * @brief Stores all configured command interfaces in #command_interface_types_.
     */
    void store_command_interface_types();

    /**
     * @brief Stores all configured state interfaces in #state_interface_types_.
     */
    void store_state_interface_types();

    /**
     * @brief Initializes the state message #state_msg_ with NaN values for each configured state interface.
     *
     * For example the state message could look like this: \n
     * ```
     * {
     *  header:
     *  {
     *    stamp: 12345678
     *  },
     *  interface_groups: ['j1', 'gpio1'],
     *  interface_values:
     *  [
     *    {
     *      interface_names: ['home'],
     *      values: [NaN]
     *    },
     *    {
     *      interface_names: ['ai0', 'ai1'],
     *      values: [NaN, NaN]
     *    }
     *  ]
     * }
     * ```
     */
    void initialize_state_msg();

    /**
     * @brief Publishes current state values.
     *
     * Loops over all interface groups and command interfaces
     * and calls the apply_state_value(). Then publishes the result using the #realtime_state_publisher_.
     *
     */
    void update_states();

    /**
     * @brief Callback method for received command msg. Calls apply_command() for each command interface.
     * 
     * Sets all command interaces to 0.0 if an empty command message is received.
     * @param commands
     * @return controller_interface::return_type
     */
    controller_interface::return_type update_commands(const CmdType &commands);

    /**
     * @brief Create a map of references to the hardware components interfaces.
     * 
     * @tparam T 
     * @param interfaces_from_params 
     * @param configured_interfaces 
     * @return std::unordered_map<std::string, std::reference_wrapper<T>> 
     */
    template <typename T>
    std::unordered_map<std::string, std::reference_wrapper<T>> create_map_of_references_to_interfaces(
        const InterfacesNames &interfaces_from_params, std::vector<T> &configured_interfaces);

    /**
     * @brief performs checks on the configured interfaces vs received interfaces.
     * 
     * @tparam T 
     * @param interfaces_from_params 
     * @param configured_interfaces 
     * @return std::unordered_map<std::string, std::reference_wrapper<T>> 
     */
    template <typename T>
    bool check_if_configured_interfaces_matches_received(
        const InterfacesNames &interfaces_from_params, const T &configured_interfaces);

    /**
     * @brief retrieves the state values from the hardware and populates the state_msg with the result.
     *
     * calls the get_optional() method of each state interface from the #state_interfaces_map_ for the state referenced by the
     * index and interface_index.
     *
     * @param state_msg
     * @param index
     * @param interface_index
     */
    void apply_state_value(
        StateType &state_msg, std::size_t index, std::size_t interface_index) const;

    /**
     * @brief calls the set_value() method for the hardwares command interface 
     * references from the #command_interfaces_map_
     *
     * @param commands
     * @param index
     * @param command_interface_index
     */
    void apply_command(
        const CmdType &commands, std::size_t index,
        std::size_t command_interface_index) const;


    /**
     * @brief Gets the all qualified state interface names from the #state_interface_types_ for the joint matching `name`.
     *
     * @param name
     * @return InterfacesNames
     */
    InterfacesNames get_state_interfaces_names(const std::string &name) const;

    /**
     * @brief Update the controller parameters.
     * 
     * Only called in on_configure() to update the parameters in case they have been changed.
     * 
     * @return true 
     */
    bool update_dynamic_map_parameters();

  protected:
    /**
     * @brief vector of string holding all command interface names as configured from the parameters.
     *
     * Looks like this for example: \n
     * ```
     * ['j1/home', 'gpio1/ao0', 'gpio1/ao1']
     * ```
     */
    InterfacesNames command_interface_types_;

    /**
     * @brief vector of string holding all state interface names as configured from the parameters.
     *
     * Looks like this for example: \n
     * ```
     * ['j1/home', 'gpio1/ai0', 'gpio1/ai1']
     * ```
     */
    InterfacesNames state_interface_types_;

    /**
     * @brief map containing a reference to the actual hardwares command interfaces
     *
     */
    MapOfReferencesToCommandInterfaces command_interfaces_map_;

    /**
     * @brief map containing a reference to the actual hardwares state interfaces
     *
     */
    MapOfReferencesToStateInterfaces state_interfaces_map_;

    /**
     * @brief Subscriber object for the '~/commands' topic.
     * 
     */
    rclcpp::Subscription<CmdType>::SharedPtr command_subscriber_{};

    /**
     * @brief Publisher object to the '~/states' topic.
     * 
     */
    std::shared_ptr<rclcpp::Publisher<StateType>> state_publisher_{};

    /**
     * @brief Realtime publisher wrapper of the #state_publisher_.
     */
    std::shared_ptr<realtime_tools::RealtimePublisher<StateType>> realtime_state_publisher_{};

    /**
     * @brief Message object holding the states to be published.
     * 
     */
    StateType state_msg_;

    /**
     * @brief parameter listerner object.
     * 
     */
    std::shared_ptr<single_trigger_controller_parameters::ParamListener> param_listener_{};

    /**
     * @brief Parameter structure holding controller configuration parameters.
     * 
     */
    single_trigger_controller_parameters::Params params_;

  };

} // namespace single_trigger_controller

#endif // SINGLE_TRIGGER_CONTROLLER__GPIO_COMMAND_CONTROLLER_HPP_
