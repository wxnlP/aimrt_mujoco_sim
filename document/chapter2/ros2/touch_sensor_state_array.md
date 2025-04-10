## `TouchSensorStateArray`
用于描述多个触觉传感器的状态信息，该协议来自 aimrt 扩展的ros2_msg消息包`aimrt_msgs/msg`中

| Field  | Type                                    | Description        |
| ------ | --------------------------------------- | ------------------ |
| header | [MessageHeader](./common.md)            | 消息头信息         |
| names  | string[]                                | 传感器名称数组     |
| states | [TouchSensorState[]](#touchsensorstate) | 触觉传感器状态数组 |

## `TouchSensorState`
用于描述单个触觉传感器的状态信息， 其中包含了若干个压力传感器的数据。

| Field    | Type    | Description        |
| -------- | ------- | ------------------ |
| pressure | int16[] | 压力传感器数据数组 |

[返回目录](../chapter2_protoclos.md)