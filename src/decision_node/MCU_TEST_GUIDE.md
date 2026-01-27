# MCU通讯模块 - 测试指南

## 快速开始

### 编译
```bash
cd ~/decision_ws
catkin_make
source devel/setup.bash
```

## 测试方案1: 使用虚拟串口模拟器

### 步骤1: 创建虚拟串口对
如果系统未安装socat，先安装：
```bash
# Ubuntu/Debian
sudo apt-get install socat

# CentOS/RHEL
sudo yum install socat
```

创建虚拟串口对：
```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

输出类似：
```
2024/01/27 10:00:00 socat[12345] N PTY is /dev/pts/11
2024/01/27 10:00:00 socat[12345] N PTY is /dev/pts/12
2024/01/27 10:00:00 socat[12345] N starting data transfer loop with FDs [5,5] and [6,6]
```

记下这两个端口（如 /dev/pts/11 和 /dev/pts/12）

### 步骤2: 启动ROS主节点
```bash
roscore
```

### 步骤3: 启动MCU通讯模块（新终端）
```bash
source ~/decision_ws/devel/setup.bash
rosrun decision_node mcu_communicator _serial_port:=/dev/pts/11 _baudrate:=115200
```

### 步骤4: 启动MCU模拟器（新终端）
```bash
source ~/decision_ws/devel/setup.bash
rosrun decision_node mcu_simulator _serial_port:=/dev/pts/12 _baudrate:=115200
```

### 步骤5: 验证（新终端）
```bash
source ~/decision_ws/devel/setup.bash

# 列出所有发布的topic
rostopic list | grep -E "referee|nav|robot"

# 查看数据
rostopic echo /referee/game_progress
rostopic echo /nav/yaw_angle
rostopic echo /robot/self_hp

# 检查频率
rostopic hz /referee/game_progress
```

## 测试方案2: 使用launch文件

### 修改launch文件
编辑 `mcu_communicator.launch`，修改串口号为实际的串口：
```xml
<param name="serial_port" value="/dev/ttyUSB0" />
```

### 启动
```bash
source ~/decision_ws/devel/setup.bash
roslaunch decision_node mcu_communicator.launch
```

## 测试方案3: 与决策系统集成

### 启动整个系统
```bash
# 终端1: roscore
roscore

# 终端2: MCU通讯
source ~/decision_ws/devel/setup.bash
rosrun decision_node mcu_communicator _serial_port:=/dev/ttyUSB0

# 终端3: 决策节点
source ~/decision_ws/devel/setup.bash
rosrun decision_node strategy_node
```

### 验证决策系统收到数据
```bash
# 查看黑板数据
source ~/decision_ws/devel/setup.bash
rosparam get /BehaviorTree  # 如果有参数的话

# 或通过ROS echo查看各topic
rostopic echo /referee/game_progress
rostopic echo /nav/yaw_angle
```

## 测试数据说明

模拟器发送的测试数据特点：
- **云台角度(yaw_angle)**: 正弦波摇摆 $\sin(t \cdot 0.01) \cdot \pi$
- **底盘IMU(chassis_imu)**: 余弦波旋转 $\cos(t \cdot 0.01) \cdot \pi$
- **操作手位置**: 圆形轨迹运动
  - x: $100 + 50 \cdot \sin(t \cdot 0.01)$
  - y: $200 + 50 \cdot \cos(t \cdot 0.01)$
- **血量**: 周期性波动
- **占领状态**: 0 → 1 → 2 循环 (未占领→友方→敌方)
- **死亡状态**: 英雄和3号交替死亡

## 常见问题排查

### 1. 无法创建虚拟串口
```bash
# 检查socat是否安装
which socat

# 如未安装，进行安装
sudo apt-get install socat
```

### 2. 权限错误: Permission denied
```bash
# 给用户权限
sudo usermod -a -G dialout $USER

# 需要重新登录生效
logout
# 重新登录
```

### 3. MCU通讯模块无法打开串口
```bash
# 检查端口是否存在
ls -l /dev/pts/11

# 检查是否有其他程序占用
lsof /dev/ttyUSB0
```

### 4. 收不到任何数据
- 检查通讯模块输出，确认串口已打开
- 确认模拟器也已启动
- 查看ROS日志: `rosnode info /mcu_communicator`

### 5. 数据格式错误
```bash
# 检查数据帧大小
rostopic echo -n 1 /referee/game_progress | head -20

# 查看通讯模块日志
rosnode info /mcu_communicator
```

## 性能测试

### 测试接收频率
```bash
rostopic hz /referee/game_progress
# 应该显示 ~10Hz (模拟器发送频率)
```

### 测试延迟
```bash
# 测试从下位机到ROS topic的延迟
rostopic delay /referee/game_progress
```

### 测试带宽
模拟器发送 46字节/帧 × 10Hz = 460 字节/秒 ≈ 3.68 kbps

## 高级测试

### 测试数据准确性
编写ROS subscriber验证数据是否正确解析：
```cpp
void callback(const std_msgs::Float32::ConstPtr& msg) {
    // 验证yaw_angle在 [-π, π] 范围内
    if (msg->data < -3.14159 || msg->data > 3.14159) {
        ROS_ERROR("Invalid yaw_angle: %f", msg->data);
    }
}
```

### 压力测试
修改模拟器发送频率（增加速率），测试系统稳定性：
```cpp
ros::Rate loop_rate(100);  // 改为100Hz
```

### 错误恢复测试
1. 启动通讯模块
2. 停止模拟器，验证模块如何处理
3. 重启模拟器，验证是否自动恢复

## 日志分析

### 启用详细日志
```bash
# 启用ROS_DEBUG日志
export ROSCONSOLE_DEFAULT_CONFIG_FILE=$ROS_PACKAGE_PATH/decision_node/config/rosconsole.conf

# 或在launch文件中配置
<param name="log_level" value="debug" />
```

### 查看通讯模块日志
```bash
rosbag record /rosout &
# 运行测试...
rosnode info /mcu_communicator
```

## 集成测试清单

- [ ] MCU通讯模块能成功打开串口
- [ ] 模拟器能成功发送数据帧
- [ ] 所有期望的ROS topic都被发布
- [ ] 数据值在合理范围内
- [ ] 发送频率符合预期 (~10Hz)
- [ ] 决策系统能收到并处理数据
- [ ] 网络延迟在可接受范围内 (<100ms)
- [ ] 系统稳定性测试通过（运行>1小时无错误）

## 下一步

1. **替换为真实下位机**: 用实际的串口设备替换虚拟串口
2. **添加CRC校验**: 实现正确的CRC8校验算法
3. **数据验证**: 添加更多的数据合法性检查
4. **错误处理**: 改进串口异常处理和重连机制
5. **性能优化**: 根据实际需求调整接收频率和缓冲大小
