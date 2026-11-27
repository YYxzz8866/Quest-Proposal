# 2. Quest 手柄采集及标定

## 2.1 采集启动与管理

本地运行 Python，提供 HTTPS 采集页和接收接口。Quest 和本地连到同一个局域网后，在 Quest Browser 打开页面，点一次 `Start Tracking`（启动追踪）。页面建立 WebXR 会话，用同一个 `local-floor` 参考空间读取手柄位置。每次启动新的 XR 会话，都要在 Quest 上完成这一步。

有追踪后，左手柄依次放到 O、X、Y 点。每到一个点，放稳左手柄，按一次右手 Trigger。网页判断手柄是否停稳并提交点位，本地在三点齐全后计算并输出标定结果。

- `Capture point {point}.`（请采集指定点，例如 `Capture point O.`）
- `Point {point} captured: ({x}, {y}, {z}) m`（已采集该点，坐标单位为米）
- `Rotation matrix (Quest to fr3_duo_mount_origin)`（Quest 到双臂公共基座的旋转矩阵）
- `Axis angle errors (deg): X={x_error}, Y={y_error}`（X、Y 两条轴的角度误差，单位为度）

## 2.2 Quest 网页采集与数据处理计算

Python 同时提供采集页和点位接口。启动时用 `--bind` 绑定本地的局域网地址。通过 `--cert` 和 `--key` 启用 HTTPS，证书须被 Quest Browser 接受。Quest 浏览器访问本地的局域网地址和端口，打开根路径 `/` 后点一次 `Start Tracking`（启动追踪）。

1. **网页读取左手柄位置。** 用 WebXR 的 `requestAnimationFrame()` 逐帧读取左手柄在 `local-floor` 下的三维位置，给每条采集加时间戳，保留最近 350 毫秒的数据。手柄追踪丢失时清空缓存，等追踪恢复后重新积累样本。
2. **右手 Trigger 确认当前点。** 网页收到右手柄的 `selectstart` 事件后，取按键前 350 毫秒内的样本判断左手柄是否停稳。联调时先设为至少 5 条有效样本，最新一条距按键不超过 100 毫秒，每条样本距平均位置不超过 5 毫米。通过检查后，把平均位置作为当前点坐标。网页通过 `POST /status` 上报本轮就绪或检查失败，Python 在终端打印对应提示。就绪时显示 `Capture point O.`（请采集 O 点）。检查失败时保留当前点，显示 `Not enough tracking data. Press the right trigger again.`（追踪数据不足，请再次按右手 Trigger）或 `Hold the left controller still, then press the right trigger again.`（请放稳左手柄，再按右手 Trigger）。
3. **网页提交点位，Python 确认接收。** 网页通过 HTTPS `POST /points` 发送组号、点名和以米为单位的三维坐标。Python 按组号缓存点位，校验坐标和 O → X → Y 的顺序。接受 O、X 后，终端打印本次坐标，并分别显示 `Capture point X.`（请采集 X 点）或 `Capture point Y.`（请采集 Y 点）。拒收时，终端打印 `Point {point} rejected: {reason}`（该点未被接受），并将原因返回网页。网页收到成功确认后才推进点位。
4. **Python 计算标定结果。** 收到 Y 点时，先用三个点计算 O→X 和 O→Y 两个向量，检查长度和夹角。通过后保存 Y 点，将两个向量归一化，用 SVD 拟合到双臂公共基座的 +X、+Y 方向，按统一格式输出旋转矩阵和角度误差。几何检查失败时终端显示 `Calibration failed: {reason}`（标定失败），网页停在 Y 点。若发现 O 或 X 放错，则重开一组。
5. **处理超时、重试和会话重置。** 网页给 `POST /points` 设置请求超时。超时显示 `Request timed out. Waiting to retry.`（请求超时，等待重试），保留待确认的组号、点名和坐标。网络恢复后重发同一份数据，收到成功确认再清除待发送数据并推进。Python 对同组同点只记录一次，重复提交时返回当前进度。服务端明确返回点位校验失败时，清除待发送数据，留在当前点重新采样。XR 会话重新建立或参考空间重置时，清空旧样本和待发送数据，生成新组号，并显示 `Calibration restarted. Capture point O.`（已重开一轮，请从 O 点开始）。

先联调 HTTPS 页面和 `POST /points`，验证能采齐 O、X、Y 并输出矩阵。再补终端状态提示和超时重发，验证断网恢复后同一个点只记录一次。
