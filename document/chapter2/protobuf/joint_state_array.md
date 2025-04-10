## `JointStateArray`
用于描述一组关节的状态信息，具体结构如下：。

| Field  | Type                                         | Description  |
| ------ | -------------------------------------------- | ------------ |
| header | [aimrt.protocols.common.Header](./common.md) | 消息头信息   |
| joints | [repeated JointState ](#jointstate)          | 关节状态数组 |


#### `JointState`
用于描述单个关节的状态信息。
| Field    | Type   | Description                          |
| -------- | ------ | ------------------------------------ |
| name     | string | 关节名称                             |
| position | double | 位置 (旋转关节:rad / 移动关节:m)     |
| velocity | double | 速度 (旋转关节:rad/s / 移动关节:m/s) |
| effort   | double | 力/力矩 (旋转关节:N⋅m / 移动关节:N)  |

[返回目录](../chapter2_protoclos.md)