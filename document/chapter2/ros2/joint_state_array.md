## `JointStateArray`
用于描述一组关节的状态信息，该协议来自 aimrt 扩展的ros2_msg消息包`aimrt_msgs/msg`中

| Field  | Type                         | Description  |
| ------ | ---------------------------- | ------------ |
| header | [MessageHeader](./common.md) | 消息头信息   |
| joints | [JointState[]](#jointstate)  | 关节状态数组 |


#### `JointState`
用于描述单个关节的状态信息。
| Field    | Type    | Description                          |
| -------- | ------- | ------------------------------------ |
| name     | string  | 关节名称                             |
| position | float64 | 位置 (旋转关节:rad / 移动关节:m)     |
| velocity | float64 | 速度 (旋转关节:rad/s / 移动关节:m/s) |
| effort   | float64 | 力/力矩 (旋转关节:N⋅m / 移动关节:N)  |

[返回目录](../chapter2_protoclos.md)