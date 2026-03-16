#!/bin/bash
# 自动检测USB串口号并启动ROS节点

# 查找USB串口设备
find_serial_port() {
    # 优先查找 /dev/ttyACM*（STM32虚拟COM口）
    if ls /dev/ttyACM* 2>/dev/null | head -1 > /dev/null; then
        ls /dev/ttyACM* 2>/dev/null | head -1
    # 其次查找 /dev/ttyUSB*
    elif ls /dev/ttyUSB* 2>/dev/null | head -1 > /dev/null; then
        ls /dev/ttyUSB* 2>/dev/null | head -1
    else
        echo "/dev/ttyACM0"  # 默认值
    fi
}

# 检测串口号
SERIAL_PORT=$(find_serial_port)

echo "================================================"
echo "MCU通讯节点启动器"
echo "================================================"
echo "检测到的串口号: $SERIAL_PORT"
echo "波特率: ${2:-115200}"
echo "================================================"

# 获取命令行参数（如果有的话可以覆盖自动检测的端口）
if [ ! -z "$1" ]; then
    SERIAL_PORT=$1
fi

# 启动ROS launch文件
roslaunch decision_node mcu_communicator.launch serial_port:=$SERIAL_PORT baudrate:=${2:-115200}
