## `Image`

用于描述一帧图像数据, 该协议来自ROS2标准消息包`sensor_msgs/msg/Image.msg`。

| Field        | Type                       | Description                                          |
| ------------ | -------------------------- | ---------------------------------------------------- |
| header       | [std_msgs/Header](#header) | 消息头信息，包含时间戳、坐标系ID                     |
| height       | uint32                     | 图像高度（像素）                                     |
| width        | uint32                     | 图像宽度（像素）                                     |
| encoding     | string                     | 图像编码，本项目使用 `rgb8`（彩色图）和 `16UC1`（深度图，单位毫米） |
| is_bigendian | uint8                      | 数据是否为大端存储，0 表示小端                       |
| step         | uint32                     | 每一行数据占用的字节数                               |
| data         | uint8[]                    | 图像数据，总长度为 `step * height`                   |

使用注意点如下：
- `step` 表示行跨距，等于 `width * 每像素字节数`；消费端需要按 `step` 定位每一行的起始位置
- 深度图为 `16UC1`，每个像素为 2 字节的 `uint16`，单位为毫米；`0` 表示无效值，实际可用范围为 1~65535 mm
- 彩色图为 `rgb8`，每个像素为 3 字节，顺序为 R、G、B

### `Header`
| Field    | Type                    | Description |
| -------- | ----------------------- | ----------- |
| stamp    | builtin_interfaces/Time | 时间戳      |
|          | - int32 sec             | 秒          |
|          | - uint32 nanosec        | 纳秒        |
| frame_id | string                  | 坐标系ID    |

[返回目录](../chapter2_protoclos.md)
