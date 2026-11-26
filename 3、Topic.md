# 3. 系统通信与话题设计

## 3.1 数据流与话题总表

### 3.1.1 端到端数据流

Quest 通过 WSS 向 Ubuntu 发送左右手柄数据。Bridge 校验数据后，将位姿、按键和追踪状态发布为 ROS 2 输入话题。相对跟随模块使用同一帧手柄数据生成 TCP 目标，求解模块结合 FR3 实测关节角生成七关节目标。LABS 把对应侧 Leader 话题上的关节目标转发给 Follower。

```mermaid
flowchart TD
    Quest["Quest 左右手柄"]
    Bridge["Ubuntu 上的 Quest ROS 2 Bridge"]
    Input["6 个 ROS 2 手柄输入话题"]
    Mapping["按侧计算相对 TCP 目标"]
    Solve["笛卡尔与肘部空间求解"]
    Leader["左右臂 Leader 关节目标"]
    Relay["LABS TeleopRobotNode"]
    Follower["左右臂 Follower"]
    Robot["左右 FR3"]
    Feedback["左右臂实测关节反馈"]

    Quest -->|WSS| Bridge
    Bridge --> Input
    Bridge -->|已校验的同一帧| Mapping
    Mapping --> Solve --> Leader --> Relay --> Follower --> Robot
    Robot --> Feedback
    Feedback -->|正运动学计算 TCP 起点| Mapping
    Feedback -->|当前实测关节角| Solve
```

### 3.1.2 ROS 2 话题总表

| 用途 | 左手话题 | 右手话题 | ROS 2 消息类型 | 发布方 → 接收方 | 开发范围 |
| --- | --- | --- | --- | --- | --- |
| 手柄位姿 | `/quest/left/pose` | `/quest/right/pose` | `geometry_msgs/msg/PoseStamped` | Bridge → ROS 2 输入订阅端 | 新增 |
| 手柄按键和摇杆 | `/quest/left/joy` | `/quest/right/joy` | `sensor_msgs/msg/Joy` | Bridge → ROS 2 输入订阅端 | 新增 |
| 手柄追踪有效性 | `/quest/left/tracking_valid` | `/quest/right/tracking_valid` | `std_msgs/msg/Bool` | Bridge → ROS 2 输入订阅端 | 新增 |
| Leader 关节目标 | `/left/leader/quest/joint_states` | `/right/leader/quest/joint_states` | `sensor_msgs/msg/JointState` | Quest 求解模块 → LABS | 新增 |
| Follower 关节目标 | `/left/follower/quest/joint_states` | `/right/follower/quest/joint_states` | `sensor_msgs/msg/JointState` | LABS → FR3 Follower Controller、MCAP 录制 | Quest 站点新增话题，复用控制器与录制链路 |
| 机器人实测关节状态 | `/left/franka_robot_state_broadcaster/measured_joint_states` | `/right/franka_robot_state_broadcaster/measured_joint_states` | `sensor_msgs/msg/JointState` | FR3 状态广播器 → Quest 跟随与求解模块 | 复用 |

Quest 双臂部署新增 10 个话题：6 个手柄输入、2 个 Leader 关节目标、2 个 Follower 关节目标。FR3 实测反馈话题沿用站点现有名称。

### 3.1.3 话题接口约定

- **手柄坐标系。** Bridge 发布手柄在 WebXR `local-floor` 下的位姿，`frame_id` 统一用 `quest_local_floor`。位置用米，姿态按四元数的 `x, y, z, w` 填写。标定和相对映射模块负责转换到机器人坐标系。
- **时间戳。** Bridge 在 Ubuntu 接收一帧时读取一次 ROS 时钟，同一帧的 `PoseStamped` 和 `Joy` 共用这个 ROS 时间戳。Quest 的采样时间戳用于计算帧间隔，Ubuntu 的单调时钟用于检查收包超时。
- **按键顺序。** 在 Bridge 配置中固定左右手柄的 `Joy.buttons` 和 `Joy.axes` 索引。跟随模块和按键处理模块共用此映射。
- **关节目标。** 每侧的 `JointState.name` 和 `position` 一一对应，按该臂控制器的 J1 到 J7 顺序填写，位置单位为弧度。`header.stamp` 使用生成目标时的 ROS 时间，运行时与 LABS 和 Follower 使用同一时钟基准。

## 3.2 Quest WebSocket 通信与 ROS 2 Bridge

### 3.2.1 WebSocket 连接与数据帧

Quest 网页启动 WebXR 后，通过 WSS `/ws` 连接 Ubuntu，每帧发送左右手柄的位置、姿态、按键、摇杆和追踪状态。公共字段包含采样时间戳、帧序号、XR 会话编号和参考空间版本。参考空间发生重置时更新版本。读不到手柄位姿时，该侧位姿标为无效。

### 3.2.2 数据校验与 ROS 2 发布

Bridge 收到数据后检查左右手标识、会话编号、帧序号和数据结构。对位姿检查数值是否有效、四元数能否归一化，对按键和摇杆检查数据类型及取值范围。通过检查后，向对应侧的位姿、按键和追踪状态话题发布消息。该侧位姿无效时停止发布该侧位姿，并将该侧 `tracking_valid` 置为 `false`。按键输入仍可读时，可以继续发布有效的按键数据。

```mermaid
flowchart TD
    Receive["接收一帧 Quest 数据"]
    Check{"帧结构和序号有效？"}
    Drop["丢弃该帧"]
    Side["分别处理左右手柄"]
    Pose{"该侧位姿有效？"}
    Publish["发布该侧位姿和追踪有效状态"]
    Invalid["发布该侧追踪无效状态"]
    Joy["按键有效时发布 Joy"]

    Receive --> Check
    Check -->|否| Drop
    Check -->|是| Side
    Side --> Pose
    Side --> Joy
    Pose -->|是| Publish
    Pose -->|否| Invalid
```

### 3.2.3 断线、超时与追踪失效

- **无效数据。** 整帧校验失败时丢弃该帧；只有一侧位姿无效时，仅丢弃该侧本帧位姿并发布追踪无效状态，另一侧的有效数据照常发布。后续帧校验通过后继续发布。偶发无效帧不触发暂停，也不使用无效位姿或旧位姿生成新的跟随目标。
- **断线与超时。** Bridge 使用 Ubuntu 单调时钟，分别记录各启用侧最近一次收到有效位姿的时间。无效帧不刷新计时。连接明确断开，或任一启用侧超过配置时限仍没有新的有效位姿时，Bridge 清除相关缓存并报告故障，跟随程序锁定所有启用侧的关节目标发布，请求 LABS 停止。
- **数据恢复。** 收到通过校验的新帧后恢复对应侧的输入发布。短时丢帧未超时时，保留本轮跟随起点；已触发停止时，先确认停止，再由操作者按 A 启动、按 B 同步，程序重新取点后恢复运动。XR 会话号或参考空间版本变化时，废弃旧起点，复核映射方向后再重新同步和取点。

## 3.3 LABS Leader/Follower 映射与接口复用

### 3.3.1 左右臂七关节目标发布

求解模块按侧生成七关节目标，检查数值、关节限位和与上一目标的连续性，再将关节角写入 `JointState.position`，发布到该臂的 Quest Leader 话题。同步目标和跟随目标使用同一个话题，根据 LABS 当前状态选择发布的目标。Follower 按 `position` 数组顺序读取目标，因此发布前先保证七项顺序正确。[Follower 目标订阅](/home/xzz/Gitspace/labs/services/actuators/franka-robot/src/third_party/franka_follower_controllers/src/joint_follower_controller.cpp:109)

### 3.3.2 LABS 话题转发与 Follower 执行

Quest 遥操作站点配置中，将 `left_arm`、`right_arm` 的 `leader_topic` 都设为相对名称 `leader/quest/joint_states`，分别由 `left`、`right` 命名空间解析成完整名称。每侧将 `follower_topic` 和控制器的 `target_topic_name` 都设为 `follower/quest/joint_states`，在各自命名空间内解析为 `/left/follower/quest/joint_states` 或 `/right/follower/quest/joint_states`。MCAP 录制的左右 `action` 目标分别选择对应的完整 Follower 话题，录制 LABS 转发给控制器的关节目标。Quest 站点每侧只启用一条 Quest Leader → Follower 转发；独立的 GELLO 站点保留原路由。[话题解析](/home/xzz/Gitspace/labs/services/pipeline/packages/pipeline-configs/src/pipeline_configs/station.py:51) · [FR3 控制器配置](/home/xzz/Gitspace/labs/deployments/fr3_duo_example/config_franka_robot.yml:12) · [录制目标配置](/home/xzz/Gitspace/labs/deployments/fr3_duo_example/config_contract_gr00t.yml:95)

```mermaid
flowchart LR
    LLeader["左臂 Quest Leader"]
    LRelay["LABS 左臂控制"]
    LFollower["左臂 Follower"]
    RLeader["右臂 Quest Leader"]
    RRelay["LABS 右臂控制"]
    RFollower["右臂 Follower"]

    LLeader --> LRelay --> LFollower
    RLeader --> RRelay --> RFollower
```

启动遥操作时，LABS 按站点配置创建 `TeleopRobotNode`，订阅 Leader 并转发到 Follower。联调时分别确认左右臂的订阅关系，检查 Follower 收到的关节角顺序和时间戳，再验证同步、跟随与录制。[LABS 启动遥操作](/home/xzz/Gitspace/labs/services/pipeline/data-collection/src/services/teleop.py:75) · [LABS 转发节点](/home/xzz/Gitspace/labs/services/pipeline/data-collection/src/services/teleop_robot_node.py:24)

### 3.3.3 FR3 实测反馈接入

跟随程序分别订阅两侧的实测关节状态，按 `JointState.name` 将 `position` 重排成 J1 至 J7。建立跟随起点时，用对应侧最新有效的实测关节角做正运动学，得到 TCP 起点。跟随期间，把实测关节角作为求解当前目标的参考。任一启用侧的实测反馈缺失或过期时，跟随程序锁定所有启用侧的目标发布，并请求 LABS 停止。
