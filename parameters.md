# Single Trigger Controller Parameters Parameters

Default Config
```yaml
single_trigger_controller_parameters:
  ros__parameters:
    command_interfaces:
      gpios:
        <gpios>:
          interfaces: '{}'
      joints:
        <joints>:
          interfaces: '{}'
    gpios: '{}'
    joints: '{}'
    state_interfaces:
      gpios:
        <gpios>:
          interfaces: '{}'
      joints:
        <joints>:
          interfaces: '{}'

```

## gpios

List of gpios

* Type: `string_array`
* Default Value: {}
* Read only: True

*Constraints:*
 - contains no duplicates

*Additional Constraints:*



## joints

List of joints

* Type: `string_array`
* Default Value: {}
* Read only: True

*Constraints:*
 - contains no duplicates

*Additional Constraints:*



## command_interfaces.gpios.<gpios>.interfaces

List of command interfaces for each gpio.

* Type: `string_array`
* Default Value: {}
* Read only: True

*Constraints:*
 - contains no duplicates

*Additional Constraints:*



## command_interfaces.joints.<joints>.interfaces

List of command interfaces for each joint.

* Type: `string_array`
* Default Value: {}
* Read only: True

*Constraints:*
 - contains no duplicates

*Additional Constraints:*



## state_interfaces.gpios.<gpios>.interfaces

List of state interfaces for each gpio. If empty no states are broadcasted

* Type: `string_array`
* Default Value: {}
* Read only: True

*Constraints:*
 - contains no duplicates

*Additional Constraints:*



## state_interfaces.joints.<joints>.interfaces

List of state interfaces for each joint. If empty no states are broadcasted

* Type: `string_array`
* Default Value: {}
* Read only: True

*Constraints:*
 - contains no duplicates

*Additional Constraints:*