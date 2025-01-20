# 第2章 消息类型 

本章介绍本项目关于消息协议类型的规范。通信协议采用 `protobuf`标准消息格式，这些消息将作为系统的状态与控制信息的载体。本章将介绍每种消息类型的详细信息， 用户需要根据实际应用场景选择合适的消息类型以实现和 Aimrt_Mujoco_Sim 模块的通信。

### 3.1 导航
| 主要消息类型                 | 功能描述                               | 推荐应用场景               |
| ---------------------------- | -------------------------------------- | -------------------------- |
| [JointState](#32-jointstate) | 关节状态信息，包含位置、速度、力等参数 | - 机器人关节状态监测与控制 |


### 3.2 `JointState`
这是一个 protobuf 类型，用于描述一个或一组关节的状态信息。

| Field  | Type                                                         | Description  |
| ------ | ------------------------------------------------------------ | ------------ |
| header | [aimrt.protocols.common.Header](#aimrtprotocolscommonheader) | 消息头信息   |
| data   | [repeated SingleJointState ](#singlejointstate)              | 关节状态数组 |



### `SingleJointState`
用于描述单个关节的状态信息。
| Field    | Type   | Description                            |
| -------- | ------ | -------------------------------------- |
| name     | string | 关节名称                               |
| position | double | 位置 (旋转关节:弧度rad / 移动关节:米m) |
| velocity | double | 速度 (旋转关节:rad/s / 移动关节:m/s)   |
| effort   | double | 力/力矩 (旋转关节:N⋅m / 移动关节:N)    |


### `aimrt.protocols.common.Header`
这是 aimrt 自带的 protobuf 类型，用于描述消息头信息。
| Field      | Type   | Description    |
| ---------- | ------ | -------------- |
| time_stamp | uint64 | 时间戳（纳秒） |
| frame_id   | string | 坐标系ID       |


