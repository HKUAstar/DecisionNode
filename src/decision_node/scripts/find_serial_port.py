#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
自动检测USB串口号的脚本
用于在launch文件中自动查找并设置串口设备
"""

import subprocess
import sys
import re

def find_usb_serial_port():
    """
    查找USB虚拟串口设备
    返回串口设备路径，如果找不到则返回默认值
    """
    try:
        # 使用 lsusb 命令查找USB设备
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        
        # 查找 STM32、Arduino 或其他常见的USB转串口芯片
        # 这些是常见的标识符：
        # - STM32: STMicroelectronics
        # - CP210x: Silicon Labs
        # - CH340: WinChipHead
        # - PL2303: Prolific
        usb_patterns = [
            r'STM32',  # STM32 MCU
            r'Silicon Labs',  # CP210x
            r'WinChipHead',  # CH340
            r'Prolific',  # PL2303
            r'STMicroelectronics',  # STM32 Virtual COM Port
        ]
        
        for line in result.stdout.split('\n'):
            for pattern in usb_patterns:
                if pattern.lower() in line.lower():
                    print(f"检测到USB设备: {line}")
        
        # 列出所有ttyACM*和ttyUSB*设备
        try:
            ls_result = subprocess.run(['ls', '-la', '/dev/ttyACM*', '/dev/ttyUSB*'], 
                                      shell=True, capture_output=True, text=True)
            ports = ls_result.stdout.strip().split('\n')
            
            if ports and ports[0]:
                # 优先使用 /dev/ttyACM0 (STM32虚拟COM口)
                for port in ports:
                    if 'ttyACM' in port:
                        device = re.search(r'/dev/tty[^ ]+', port)
                        if device:
                            return device.group(0)
                
                # 其次使用 /dev/ttyUSB0
                for port in ports:
                    if 'ttyUSB' in port:
                        device = re.search(r'/dev/tty[^ ]+', port)
                        if device:
                            return device.group(0)
        except:
            pass
    except Exception as e:
        print(f"检测串口时出错: {e}", file=sys.stderr)
    
    # 默认值
    return "/dev/ttyACM0"

if __name__ == "__main__":
    port = find_usb_serial_port()
    print(port)
