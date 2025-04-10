## `Common`

### `MessageHeader`
用于描述消息的头部信息，该协议来自 aimrt 扩展的ros2_msg消息包`aimrt_msgs/msg`中。

| Field    | Type                    | Description                |
| -------- | ----------------------- | -------------------------- |
| stamp    | builtin_interfaces/Time | 世界时间戳，以秒和纳秒表示 |
| frame_id | string                  | 与此数据关联的坐标系       |
| sequence | uint32                  | 消息的连续递增ID           |

[返回目录](../chapter2_protoclos.md)