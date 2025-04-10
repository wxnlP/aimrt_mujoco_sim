# 第2章 消息类型 

本章介绍本项目关于消息协议类型的规范。通信协议采用 `protobuf`以及 `ros2 msg`标准消息格式，这些消息将作为系统的状态与控制信息的载体。本章将介绍每种消息类型的详细信息， 用户需要根据实际应用场景选择合适的消息类型以实现和 Aimrt_Mujoco_Sim 模块的通信， 了解这些消息的具体格式以便正确地解析和使用。

## 目录
* protobuf 类型
  * [JointStateArray](./protobuf/joint_state_array.md)
  * [JointCommandArray](./protobuf/joint_command_array.md)
  * [TouchSensorStateArray](./protobuf/touch_sensor_state_array.md)
  * [ImuState](./protobuf/imu_state.md)
* ros2 msg 类型
  * [JointStateArray](./ros2/joint_state_array.md)
  * [JointCommandArray](./ros2/joint_command_array.md)
  * [TouchStateArray](./ros2/touch_sensor_state_array.md)
  * [Imu](./ros2/imu.md)











