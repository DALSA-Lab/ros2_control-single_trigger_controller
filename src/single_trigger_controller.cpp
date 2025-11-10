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

#include "single_trigger_controller/single_trigger_controller.hpp"

#include <algorithm>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/component_parser.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"

namespace
{ // utility
    template <typename T>
    void print_interface(const rclcpp::Logger &logger, const T &command_interfaces)
    {
        for (const auto &[interface_name, value] : command_interfaces)
        {
            RCLCPP_ERROR(logger, "Got %s", interface_name.c_str());
        }
    }
} // namespace

namespace single_trigger_controller
{

    SingleTriggerController::SingleTriggerController() : controller_interface::ControllerInterface() {}

    CallbackReturn SingleTriggerController::on_init()
    {
        try
        {
            param_listener_ = std::make_shared<single_trigger_controller_parameters::ParamListener>(get_node());
            params_ = param_listener_->get_params();

            return CallbackReturn::SUCCESS;
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
    }

    CallbackReturn SingleTriggerController::on_configure(const rclcpp_lifecycle::State &)
    try
    {
        // Get latest controller parameter update
        if (!update_dynamic_map_parameters())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        store_command_interface_types();
        store_state_interface_types();

        if (command_interface_types_.empty() && state_interface_types_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "No command or state interfaces are configured");
            return CallbackReturn::ERROR;
        }

        /* create command subscription, if command interfaces are configured. */
        if (!command_interface_types_.empty())
        {
            command_subscriber_ = get_node()->create_subscription<CmdType>(
                "~/commands", rclcpp::SystemDefaultsQoS(),
                std::bind(&SingleTriggerController::update_commands, this, std::placeholders::_1));
        }

        /* create state publisher, if state interfaces are configured. */
        if (!state_interface_types_.empty())
        {
            state_publisher_ =
                get_node()->create_publisher<StateType>("~/states", rclcpp::SystemDefaultsQoS());

            realtime_state_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<StateType>>(state_publisher_);
            RCLCPP_INFO(get_node()->get_logger(), "configure successful");
        }

        return CallbackReturn::SUCCESS;
    }
    catch (const std::exception &e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    controller_interface::InterfaceConfiguration
    SingleTriggerController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        command_interfaces_config.names = command_interface_types_;

        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration SingleTriggerController::state_interface_configuration()
        const
    {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        state_interfaces_config.names = state_interface_types_;

        return state_interfaces_config;
    }

    CallbackReturn SingleTriggerController::on_activate(const rclcpp_lifecycle::State &)
    {
        command_interfaces_map_ =
            create_map_of_references_to_interfaces(command_interface_types_, command_interfaces_);
        state_interfaces_map_ =
            create_map_of_references_to_interfaces(state_interface_types_, state_interfaces_);
        if (
            !check_if_configured_interfaces_matches_received(
                command_interface_types_, command_interfaces_map_) ||
            !check_if_configured_interfaces_matches_received(state_interface_types_, state_interfaces_map_))
        {
            return CallbackReturn::ERROR;
        }

        initialize_state_msg();
        
        RCLCPP_INFO(get_node()->get_logger(), "activate successful");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn SingleTriggerController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type SingleTriggerController::update(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        update_states();
        return controller_interface::return_type::OK;
    }

    bool SingleTriggerController::update_dynamic_map_parameters()
    {
        auto logger = get_node()->get_logger();
        // update the dynamic map parameters
        param_listener_->refresh_dynamic_parameters();
        // get parameters from the listener in case they were updated
        params_ = param_listener_->get_params();
        return true;
    }

    void SingleTriggerController::store_command_interface_types()
    {
        // todo: do checks that either gpio and/or joints are defined

        for (const auto &[gpio_name, interfaces] : params_.command_interfaces.gpios.gpios_map)
        {
            std::transform(
                interfaces.interfaces.cbegin(), interfaces.interfaces.cend(),
                std::back_inserter(command_interface_types_),
                [&](const auto &interface_name)
                { return gpio_name + "/" + interface_name; });
        }

        for (const auto &[joint_name, interfaces] : params_.command_interfaces.joints.joints_map)
        {
            std::transform(
                interfaces.interfaces.cbegin(), interfaces.interfaces.cend(),
                std::back_inserter(command_interface_types_),
                [&](const auto &interface_name)
                { return joint_name + "/" + interface_name; });
        }
    }

    void SingleTriggerController::store_state_interface_types()
    {
        for (const auto &[gpio_name, interfaces] : params_.state_interfaces.gpios.gpios_map)
        {
            std::transform(
                interfaces.interfaces.cbegin(), interfaces.interfaces.cend(),
                std::back_inserter(state_interface_types_),
                [&](const auto &interface_name)
                { return gpio_name + "/" + interface_name; });
        }

        for (const auto &[joint_name, interfaces] : params_.state_interfaces.joints.joints_map)
        {
            std::transform(
                interfaces.interfaces.cbegin(), interfaces.interfaces.cend(),
                std::back_inserter(state_interface_types_),
                [&](const auto &interface_name)
                { return joint_name + "/" + interface_name; });
        }
    }

    void SingleTriggerController::initialize_state_msg()
    {
        state_msg_.header.stamp = get_node()->now();
        state_msg_.interface_groups.resize(params_.gpios.size() + params_.joints.size());
        state_msg_.interface_values.resize(params_.gpios.size() + params_.joints.size());

        for (std::size_t gpio_index = 0; gpio_index < params_.gpios.size(); ++gpio_index)
        {
            const auto gpio_name = params_.gpios[gpio_index];
            state_msg_.interface_groups[gpio_index] = gpio_name;
            state_msg_.interface_values[gpio_index].interface_names =
                get_state_interfaces_names(gpio_name);
            state_msg_.interface_values[gpio_index].values = std::vector<double>(
                state_msg_.interface_values[gpio_index].interface_names.size(),
                std::numeric_limits<double>::quiet_NaN());
        }

        for (std::size_t joint_index = 0; joint_index < params_.joints.size(); ++joint_index)
        {
            const auto joint_name = params_.joints[joint_index];
            state_msg_.interface_groups[joint_index] = joint_name;
            state_msg_.interface_values[joint_index].interface_names =
                get_state_interfaces_names(joint_name);
            state_msg_.interface_values[joint_index].values = std::vector<double>(
                state_msg_.interface_values[joint_index].interface_names.size(),
                std::numeric_limits<double>::quiet_NaN());
        }
    }

    InterfacesNames SingleTriggerController::get_state_interfaces_names(
        const std::string &name) const
    {
        InterfacesNames result;
        for (const auto &interface_name : state_interface_types_)
        {
            const auto it = state_interfaces_map_.find(interface_name);
            if (it != state_interfaces_map_.cend() && it->second.get().get_prefix_name() == name)
            {
                result.emplace_back(it->second.get().get_interface_name());
            }
        }
        return result;
    }

    template <typename T>
    std::unordered_map<std::string, std::reference_wrapper<T>>
    SingleTriggerController::create_map_of_references_to_interfaces(
        const InterfacesNames &interfaces_from_params, std::vector<T> &configured_interfaces)
    {
        std::unordered_map<std::string, std::reference_wrapper<T>> map;
        for (const auto &interface_name : interfaces_from_params)
        {
            auto interface = std::find_if(
                configured_interfaces.begin(), configured_interfaces.end(),
                [&](const auto &configured_interface)
                {
                    const auto full_name_interface_name = configured_interface.get_name();
                    return full_name_interface_name == interface_name;
                });
            if (interface != configured_interfaces.end())
            {
                map.emplace(interface_name, std::ref(*interface));
            }
        }
        return map;
    }

    template <typename T>
    bool SingleTriggerController::check_if_configured_interfaces_matches_received(
        const InterfacesNames &interfaces_from_params, const T &configured_interfaces)
    {
        if (!(configured_interfaces.size() == interfaces_from_params.size()))
        {
            RCLCPP_ERROR(
                get_node()->get_logger(), "Expected %ld interfaces, got %ld", interfaces_from_params.size(),
                configured_interfaces.size());
            for (const auto &interface : interfaces_from_params)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Expected %s", interface.c_str());
            }
            print_interface(get_node()->get_logger(), configured_interfaces);
            return false;
        }
        return true;
    }

    controller_interface::return_type SingleTriggerController::update_commands(const CmdType &commands)
    {
        /* control_msgs::msg::DynamicInterfaceGroupValues has following structure:
        {
          string[] interface_groups: ['j1', 'j2','...'],
          InterfaceValue[] interface_values:
          [
            {
              string[] interface_names: ['home', 'home', '...'],
              float64[] values: [0.0, 0.1, ...],
            },
            {...}
          ]
        }
        */

        if (commands.interface_groups.empty() || commands.interface_values.empty())
        {
            // if the interfaces are empty, set all command interfaces to 0.0.
            for (auto full_command_interface_name : command_interface_types_)
            {
                std::ignore = command_interfaces_map_.at(full_command_interface_name).get().set_value(0.0);
            }

            return controller_interface::return_type::OK;
        }

        for (std::size_t gpio_index = 0; gpio_index < commands.interface_groups.size();
             ++gpio_index)
        {
            const auto &gpio_name = commands.interface_groups[gpio_index];
            if (
                commands.interface_values[gpio_index].values.size() !=
                commands.interface_values[gpio_index].interface_names.size())
            {
                RCLCPP_ERROR(
                    get_node()->get_logger(), "For gpio %s interfaces_names do not match values",
                    gpio_name.c_str());
                return controller_interface::return_type::ERROR;
            }
            for (std::size_t command_interface_index = 0;
                 command_interface_index < commands.interface_values[gpio_index].values.size();
                 ++command_interface_index)
            {
                apply_command(commands, gpio_index, command_interface_index);
            }
        }
        return controller_interface::return_type::OK;
    }

    void SingleTriggerController::apply_command(
        const CmdType &commands, std::size_t index, std::size_t command_interface_index) const
    {
        const auto full_command_interface_name =
            commands.interface_groups[index] + '/' +
            commands.interface_values[index].interface_names[command_interface_index];

        const auto &command_value =
            commands.interface_values[index].values[command_interface_index];

        try
        {
            if (!command_interfaces_map_.at(full_command_interface_name).get().set_value(command_value))
            {
                RCLCPP_WARN(
                    get_node()->get_logger(), "Unable to set the command for interface '%s' with value '%f'.",
                    full_command_interface_name.c_str(), command_value);
            }
        }
        catch (const std::exception &e)
        {
            fprintf(
                stderr, "Exception thrown during applying command stage of %s with message: %s . Verify that the controller is in 'active' state.\n",
                full_command_interface_name.c_str(), e.what());
        }
    }

    void SingleTriggerController::update_states()
    {
        if (!realtime_state_publisher_)
        {
            return;
        }

        state_msg_.header.stamp = get_node()->now();
        for (std::size_t index = 0; index < state_msg_.interface_groups.size();
             ++index)
        {
            for (std::size_t interface_index = 0;
                 interface_index < state_msg_.interface_values[index].interface_names.size();
                 ++interface_index)
            {
                apply_state_value(state_msg_, index, interface_index);
            }
        }
        realtime_state_publisher_->try_publish(state_msg_);
    }

    void SingleTriggerController::apply_state_value(
        StateType &state_msg, std::size_t index, std::size_t interface_index) const
    {
        const auto interface_name =
            state_msg.interface_groups[index] + '/' +
            state_msg.interface_values[index].interface_names[interface_index];
        try
        {
            auto state_msg_interface_value_op =
                state_interfaces_map_.at(interface_name).get().get_optional();

            if (!state_msg_interface_value_op.has_value())
            {
                RCLCPP_DEBUG(
                    get_node()->get_logger(), "Unable to retrieve the data from state: %s \n",
                    interface_name.c_str());
            }
            else
            {
                state_msg.interface_values[index].values[interface_index] =
                    state_msg_interface_value_op.value();
            }
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during reading state of: %s \n", interface_name.c_str());
        }
    }

} // namespace single_trigger_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    single_trigger_controller::SingleTriggerController, controller_interface::ControllerInterface)
