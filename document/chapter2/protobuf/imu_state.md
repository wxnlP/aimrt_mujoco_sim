## `ImuState`
用于描述一个或多个IMU传感器的状态信息。

| Field                          | Type                                                                     | Description                                                |
| ------------------------------ | ------------------------------------------------------------------------ | ---------------------------------------------------------- |
| header                         | [aimrt.protocols.common.Header](./common.md)                             | 消息头信息，包含时间戳、序列号等元数据                     |
| orientation                    | [aimrt.protocols.geometry.Quaternion](#aimrtprotocolsgeometryquaternion) | 姿态四元数 (x,y,z,w)                                       |
| orientation_covariance         | repeated double                                                          | 3x3姿态协方差矩阵（行优先存储，9个元素，单位：rad²）       |
| angular_velocity               | [aimrt.protocols.geometry.Vector3](#aimrtprotocolsgeometryvector3)       | 三轴角速度测量值（单位：rad/s）                            |
| angular_velocity_covariance    | repeated double                                                          | 3x3角速度协方差矩阵（行优先存储，9个元素，单位：(rad/s)²） |
| linear_acceleration            | [aimrt.protocols.geometry.Vector3](#aimrtprotocolsgeometryvector3)       | 三轴线性加速度测量值（单位：m/s²）                         |
| linear_acceleration_covariance | repeated double                                                          | 3x3加速度协方差矩阵（行优先存储，9个元素，单位：(m/s²)²）  |


#### `aimrt.protocols.geometry.Quaternion`
用于描述四元数姿态。
| Field | Type   | Description     |
| ----- | ------ | --------------- |
| w     | double | 四元数实部      |
| x     | double | 四元数虚部x分量 |
| y     | double | 四元数虚部y分量 |
| z     | double | 四元数虚部z分量 |

#### `aimrt.protocols.geometry.Vector3`
用于描述三维向量。
| Field | Type   | Description |
| ----- | ------ | ----------- |
| x     | double | x轴分量     |
| y     | double | y轴分量     |
| z     | double | z轴分量     |

[返回目录](../chapter2_protoclos.md)