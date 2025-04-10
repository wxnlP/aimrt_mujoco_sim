## `Imu`

用于描述IMU(惯性测量单元)的数据, 该协议来自ROS2标准消息包`sensor_msgs/msg/Imu.msg`。

| Field                          | Type                                    | Description                                    |
| ------------------------------ | --------------------------------------- | ---------------------------------------------- |
| header                         | [std_msgs/Header](#header)              | 消息头信息，包含时间戳、坐标系ID               |
| orientation                    | [geometry_msgs/Quaternion](#quaternion) | 姿态四元数 (x,y,z,w)                           |
| orientation_covariance         | float64[9]                              | 3x3姿态协方差矩阵（行优先存储，关于x,y,z轴）   |
| angular_velocity               | [geometry_msgs/Vector3](#vector3)       | 三轴角速度测量值                               |
| angular_velocity_covariance    | float64[9]                              | 3x3角速度协方差矩阵（行优先存储，关于x,y,z轴） |
| linear_acceleration            | [geometry_msgs/Vector3](#vector3)       | 三轴线性加速度测量值                           |
| linear_acceleration_covariance | float64[9]                              | 3x3加速度协方差矩阵（行优先存储，x,y,z）       |

### `Header`
| Field    | Type                    | Description |
| -------- | ----------------------- | ----------- |
| stamp    | builtin_interfaces/Time | 时间戳      |
|          | - int32 sec             | 秒          |
|          | - uint32 nanosec        | 纳秒        |
| frame_id | string                  | 坐标系ID    |

### `Quaternion`
| Field | Type    | Description | Default |
| ----- | ------- | ----------- | ------- |
| x     | float64 | 四元数x分量 | 0       |
| y     | float64 | 四元数y分量 | 0       |
| z     | float64 | 四元数z分量 | 0       |
| w     | float64 | 四元数w分量 | 1       |

### `Vector3`
| Field | Type    | Description |
| ----- | ------- | ----------- |
| x     | float64 | x轴分量     |
| y     | float64 | y轴分量     |
| z     | float64 | z轴分量     |

[返回目录](../chapter2_protoclos.md)