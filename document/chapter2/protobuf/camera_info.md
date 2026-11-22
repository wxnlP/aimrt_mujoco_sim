## `CameraInfo`
用于描述相机的内参和标定信息，字段与 ROS2 标准消息 `sensor_msgs/msg/CameraInfo.msg` 一致。

| Field            | Type                                         | Description                            |
| ---------------- | -------------------------------------------- | -------------------------------------- |
| header           | [aimrt.protocols.common.Header](./common.md) | 消息头信息，包含时间戳、坐标系ID        |
| height           | uint32                                       | 图像高度（像素）                       |
| width            | uint32                                       | 图像宽度（像素）                       |
| distortion_model | string                                       | 畸变模型，本项目为 `plumb_bob`          |
| d                | repeated double                              | 畸变系数，本项目为 5 个 0，表示无畸变   |
| k                | repeated double                              | 内参矩阵 K（3x3，行优先，9 个元素）     |
| r                | repeated double                              | 校正矩阵 R（3x3，行优先，9 个元素）     |
| p                | repeated double                              | 投影矩阵 P（3x4，行优先，12 个元素）    |
| binning_x        | uint32                                       | 水平方向像素合并系数，0 表示不合并     |
| binning_y        | uint32                                       | 垂直方向像素合并系数，0 表示不合并     |
| roi              | [RegionOfInterest](#regionofinterest)        | 感兴趣区域                             |

#### `RegionOfInterest`
| Field      | Type   | Description          |
| ---------- | ------ | -------------------- |
| x_offset   | uint32 | 区域左上角 x 偏移    |
| y_offset   | uint32 | 区域左上角 y 偏移    |
| height     | uint32 | 区域高度             |
| width      | uint32 | 区域宽度             |
| do_rectify | bool   | 是否对该区域做校正   |

使用注意点如下：
- 内参矩阵 `k` 按行优先存储为 `[fx, 0, cx, 0, fy, cy, 0, 0, 1]`，焦距由 xml 中相机的 fovy 和输出图像的 height 计算
- 项目中的相机没有镜头畸变，`distortion_model` 固定为 `plumb_bob`，`d` 全为 0
- 光心 `cx`、`cy` 默认位于图像中心

[返回目录](../chapter2_protoclos.md)
