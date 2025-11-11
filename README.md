# A general purpose controller which can be used to trigger command interfaces.

This controller is a derivative of the ros2_control [GpioCommandController](https://control.ros.org/master/doc/ros2_controllers/gpio_controllers/doc/userdoc.html).

## Features
The key differences between this controller and the GpioCommandController is two-fold:
### Use Joint interfaces
Besides GPIO interfaces, this controller allows Joint interfaces to be used, enabling the user to utilize the controller in broader applications and simplifying the URDF description.
### Single write to command interfaces
The GpioCommandController writes the latest received command to the configured state interfaces on every update cycle. On the contrary, the SingleTriggerController only writes to the command interfaces once in the subscription callback when a new command has been received. The user can for example leverage this by clearing the command interface (writing NaN or 0.0 to it) in the hardware interface once the latest trigger has been handled.

## Interfaces
### Subscribers
- `/<controller_name>/commands` [`control_msgs/msg/DynamicJointState`]: A subscriber for configured command interfaces.
### Publishers
- `/<controller_name>/states` [`control_msgs/msg/DynamicJointState`]: A subscriber for configured command interfaces.

## Parameters
The parameter configuration is slightly adapted to accomodate for gpios and joints. Take a look at [parameters.md](./parameters.md) for reference.

> [!NOTE]
>
> Contrary to the GpioCommandController, if no state interfaces are configured no states will be published.

## Example
A single trigger controller to trigger the homing of a joint.

### controllers.yaml
The relevant part of the *controllers.yaml* file could look like this:
```yaml
...
homing_controller:
  ros__parameters:
    type: single_trigger_controller/SingleTriggerController
    joints:
      - j1
      - j2
      - j3
      - j4
    command_interfaces:
      joints:
        j1:
          - interfaces:
            - home
        j2:
          - interfaces:
            - home
        j3:
          - interfaces:
            - home
        j4:
          - interfaces:
            - home

    state_interfaces:
      joints:
        j1:
          - interfaces:
            - home
        j2:
          - interfaces:
            - home
        j3:
          - interfaces:
            - home
        j4:
          - interfaces:
            - home
```
### ros2_control URDF
The relevant part of the ros2_control tag in the robots URDF description file could look like this:
```xml
...
    <joint name="${prefix}j1">
        ...
        <command_interface name="home">
            <param name="<a-parameter>">0.0</param>
        </command_interface>
        <state_interface name="home"/>
        ...
    </joint>
...
```
### Trigger the homing
One can for example manually trigger homing via the ROS2 control CLI like this:
```bash
ros2 topic pub -1 /homing_controller/commands control_msgs/msg/DynamicInterfaceGroupValues "{interface_groups: [j1], interface_values: [{interface_names: [home], values: [1.0]}]}"
```

### Handling the Trigger
Inside the harware interfaces write() method one can handle a trigger like described in the following pseudo-code:

```cpp
std::string interface_name = "j1/home";
float command = get_command(interface_name);
if(command){
    // Call triggered function
    call_to_some_function();
    set_command(interface_name, 0.0);
}
```

## Dependencies
This controller has been tested with ROS2 Jazzy and ROS2_control Jazzy.

## Install
Clone the repository into the ros2 workspace and build.
