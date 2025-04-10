## `TouchSensorStateArray`
用于描述一个或多个接触传感器的状态信息。

| Field  | Type                                           | Description                                         |
| ------ | ---------------------------------------------- | --------------------------------------------------- |
| header | [aimrt.protocols.common.Header](./common.md)   | 消息头信息，包含时间戳、序列号等元数据              |
| names  | repeated string                                | 触觉传感器名称列表（例如：["finger_tip", "palm"]）  |
| states | repeated [TouchSensorState](#touchsensorstate) | 各触觉传感器的压力状态数组（顺序与 names 字段对应） |

#### `TouchSensorState`
| Field    | Type           | Description                                                       |
| -------- | -------------- | ----------------------------------------------------------------- |
| pressure | repeated int32 | ​触觉单元压力值数组，每个元素代表传感器上一个触觉单元的压力测量值 |

[返回目录](../chapter2_protoclos.md)