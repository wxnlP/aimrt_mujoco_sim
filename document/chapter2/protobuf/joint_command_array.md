## `JointCommandArray`
用于描述组关节的控制指令信息。

| Field  | Type                                         | Description      |
| ------ | -------------------------------------------- | ---------------- |
| header | [aimrt.protocols.common.Header](./common.md) | 消息头信息       |
| joints | [repeated JointCommand ](#jointcommand)      | 关节控制指令数组 |


#### `JointCommand`
用于描述单个关节的控制指令信息。
| Field     | Type   | Description                                |
| --------- | ------ | ------------------------------------------ |
| name      | string | 关节名称                                   |
| position  | double | 位置 (旋转关节:rad / 移动关节:m)           |
| velocity  | double | 速度 (旋转关节:rad/s / 移动关节:m/s)       |
| effort    | double | 力/力矩 (旋转关节:N⋅m / 移动关节:N)        |
| stiffness | double | 刚度 (旋转关节:N/m / 移动关节:N/m)         |
| damping   | double | 阻尼 (旋转关节:N/(m/s) / 移动关节:N/(m/s)) |

[返回目录](../chapter2_protoclos.md)