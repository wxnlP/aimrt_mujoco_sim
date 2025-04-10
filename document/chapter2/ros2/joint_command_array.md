## `JointCommandArray`
用于描述组关节的控制指令信息，该协议来自 aimrt 扩展的ros2_msg消息包`aimrt_msgs/msg`中

| Field  | Type                            | Description      |
| ------ | ------------------------------- | ---------------- |
| header | [MessageHeader](./common.md)    | 消息头信息       |
| joints | [JointCommand[]](#jointcommand) | 关节控制指令数组 |

### `JointCommand`
用于描述单个关节的控制指令信息。

| Field     | Type    | Description                                |
| --------- | ------- | ------------------------------------------ |
| name      | string  | 关节名称                                   |
| position  | float64 | 位置 (旋转关节:rad / 移动关节:m)           |
| velocity  | float64 | 速度 (旋转关节:rad/s / 移动关节:m/s)       |
| effort    | float64 | 力/力矩 (旋转关节:N⋅m / 移动关节:N)        |
| stiffness | float64 | 刚度 (旋转关节:N/m / 移动关节:N/m)         |
| damping   | float64 | 阻尼 (旋转关节:N/(m/s) / 移动关节:N/(m/s)) |

[返回目录](../chapter2_protoclos.md)